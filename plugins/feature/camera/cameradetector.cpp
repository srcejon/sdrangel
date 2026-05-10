///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include <QDebug>
#include <QFile>
#include <QProcess>
#include <QTimer>
#include <QMutableHashIterator>
#include <QTextStream>

#include "maincore.h"
#include "channel/channelwebapiutils.h"
#include "device/deviceset.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"
#include "util/profiler.h"
#include "cameradetector.h"
#include "camerapostprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraDetector::MsgConfigureCameraDetector, Message)
MESSAGE_CLASS_DEFINITION(CameraDetector::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraDetector::MsgCaptureActive, Message)

namespace {
QString substituteObjectClass(QString text, const QString& className)
{
    text.replace(QStringLiteral("${class}"), className);
    return text;
}

class ProtoReader
{
public:
    ProtoReader(const uchar *data, int size) :
        m_data(data),
        m_size(size),
        m_pos(0)
    {}

    bool atEnd() const { return m_pos >= m_size; }

    bool readVarint(quint64& value)
    {
        value = 0;
        int shift = 0;

        while (m_pos < m_size && shift < 64)
        {
            const quint8 byte = m_data[m_pos++];
            value |= static_cast<quint64>(byte & 0x7f) << shift;

            if ((byte & 0x80) == 0) {
                return true;
            }

            shift += 7;
        }

        return false;
    }

    bool readSubMessage(ProtoReader& subMessage)
    {
        quint64 length = 0;

        if (!readVarint(length) || (length > static_cast<quint64>(m_size - m_pos))) {
            return false;
        }

        subMessage = ProtoReader(m_data + m_pos, static_cast<int>(length));
        m_pos += static_cast<int>(length);
        return true;
    }

    bool skipField(int wireType)
    {
        switch (wireType)
        {
        case 0:
        {
            quint64 value = 0;
            return readVarint(value);
        }
        case 1:
            return skipBytes(8);
        case 2:
        {
            quint64 length = 0;
            return readVarint(length) && skipBytes(static_cast<int>(length));
        }
        case 5:
            return skipBytes(4);
        default:
            return false;
        }
    }

private:
    bool skipBytes(int count)
    {
        if ((count < 0) || (count > (m_size - m_pos))) {
            return false;
        }

        m_pos += count;
        return true;
    }

    const uchar *m_data;
    int m_size;
    int m_pos;
};

bool parseOnnxTensorShape(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 1) && (wireType == 2))
        {
            ProtoReader dimReader(nullptr, 0);

            if (!reader.readSubMessage(dimReader)) {
                return false;
            }

            int dimValue = 0;
            bool hasDimValue = false;

            while (!dimReader.atEnd())
            {
                quint64 dimTag = 0;

                if (!dimReader.readVarint(dimTag)) {
                    return false;
                }

                const int dimFieldNumber = static_cast<int>(dimTag >> 3);
                const int dimWireType = static_cast<int>(dimTag & 0x7);

                if ((dimFieldNumber == 1) && (dimWireType == 0))
                {
                    quint64 value = 0;

                    if (!dimReader.readVarint(value)) {
                        return false;
                    }

                    dimValue = static_cast<int>(value);
                    hasDimValue = true;
                }
                else if (!dimReader.skipField(dimWireType))
                {
                    return false;
                }
            }

            dims.append(hasDimValue ? dimValue : 0);
        }
        else if (!reader.skipField(wireType))
        {
            return false;
        }
    }

    return true;
}

bool parseOnnxTensorType(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 2) && (wireType == 2))
        {
            ProtoReader shapeReader(nullptr, 0);
            return reader.readSubMessage(shapeReader) && parseOnnxTensorShape(shapeReader, dims);
        }

        if (!reader.skipField(wireType)) {
            return false;
        }
    }

    return false;
}

bool parseOnnxValueInfo(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 2) && (wireType == 2))
        {
            ProtoReader typeReader(nullptr, 0);

            if (!reader.readSubMessage(typeReader)) {
                return false;
            }

            while (!typeReader.atEnd())
            {
                quint64 typeTag = 0;

                if (!typeReader.readVarint(typeTag)) {
                    return false;
                }

                const int typeFieldNumber = static_cast<int>(typeTag >> 3);
                const int typeWireType = static_cast<int>(typeTag & 0x7);

                if ((typeFieldNumber == 1) && (typeWireType == 2))
                {
                    ProtoReader tensorTypeReader(nullptr, 0);
                    return typeReader.readSubMessage(tensorTypeReader) && parseOnnxTensorType(tensorTypeReader, dims);
                }

                if (!typeReader.skipField(typeWireType)) {
                    return false;
                }
            }
        }
        else if (!reader.skipField(wireType))
        {
            return false;
        }
    }

    return false;
}

cv::Size readOnnxInputSize(const QString& modelPath)
{
    QFile modelFile(modelPath);

    if (!modelFile.open(QIODevice::ReadOnly)) {
        return cv::Size();
    }

    const QByteArray modelData = modelFile.readAll();
    ProtoReader modelReader(reinterpret_cast<const uchar *>(modelData.constData()), modelData.size());

    while (!modelReader.atEnd())
    {
        quint64 tag = 0;

        if (!modelReader.readVarint(tag)) {
            break;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 7) && (wireType == 2))
        {
            ProtoReader graphReader(nullptr, 0);

            if (!modelReader.readSubMessage(graphReader)) {
                break;
            }

            while (!graphReader.atEnd())
            {
                quint64 graphTag = 0;

                if (!graphReader.readVarint(graphTag)) {
                    return cv::Size();
                }

                const int graphFieldNumber = static_cast<int>(graphTag >> 3);
                const int graphWireType = static_cast<int>(graphTag & 0x7);

                if ((graphFieldNumber == 11) && (graphWireType == 2))
                {
                    ProtoReader inputReader(nullptr, 0);
                    QVector<int> dims;

                    if (!graphReader.readSubMessage(inputReader) || !parseOnnxValueInfo(inputReader, dims)) {
                        continue;
                    }

                    if (dims.size() >= 4)
                    {
                        const int width = dims.at(dims.size() - 1);
                        const int height = dims.at(dims.size() - 2);

                        if ((width > 0) && (height > 0)) {
                            return cv::Size(width, height);
                        }
                    }
                }
                else if (!graphReader.skipField(graphWireType))
                {
                    return cv::Size();
                }
            }
        }
        else if (!modelReader.skipField(wireType))
        {
            break;
        }
    }

    return cv::Size();
}
}

CameraDetector::CameraDetector() :
    m_nextStage(nullptr),
    m_captureActive(false),
    m_motionPersistenceRemaining(0),
    m_motionConfirmCount(0),
    m_streakPersistenceRemaining(0),
    m_yoloInputSize(640, 640),
    m_processingFrame(false)
#ifdef QT_TEXTTOSPEECH_FOUND
    , m_speech(new QTextToSpeech(this))
#endif
{
}

CameraDetector::~CameraDetector() = default;

void CameraDetector::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraDetector::handleInputMessages);
    handleInputMessages();
}

void CameraDetector::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraDetector::handleInputMessages);
}

bool CameraDetector::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraDetector::match(cmd))
    {
        const MsgConfigureCameraDetector& cfg = (const MsgConfigureCameraDetector&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        const MsgProcessFrame& frameMsg = (const MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        const MsgCaptureActive& activeMsg = (const MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();
        if (m_captureActive)
        {
            m_previousInputFrame = CameraPipelineFrame();
            m_lastInputFrame = CameraPipelineFrame();
            m_diffMaskHistory.clear();
            m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
            m_streakBgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
            m_lastMotionBoxes.clear();
            m_motionPersistenceRemaining = 0;
            m_motionConfirmCount = 0;
            m_lastStreakDetections.clear();
            m_streakPersistenceRemaining = 0;
            m_detectedObjectClasses.clear();
            m_pendingDisappearDeadlines.clear();
        }
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrame.reset();
        if (!m_captureActive) {
            m_processingFrame = false;
        }
        return true;
    }

    return false;
}

void CameraDetector::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    const bool sourceChanged = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("cameraBinX")
        || settingsKeys.contains("cameraBinY")
        || settingsKeys.contains("cameraNumX")
        || settingsKeys.contains("cameraNumY")
        || settingsKeys.contains("cameraStartX")
        || settingsKeys.contains("cameraStartY")
        || settingsKeys.contains("cameraGain")
        || settingsKeys.contains("cameraOffset")
        || settingsKeys.contains("cameraReadoutMode")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("stackEnabled")
        || settingsKeys.contains("stackFrameCount")
        || settingsKeys.contains("stackMethod")
        || settingsKeys.contains("stackAlignmentMethod");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged) {
        m_previousInputFrame = CameraPipelineFrame();
        m_lastInputFrame = CameraPipelineFrame();
        m_diffMaskHistory.clear();
        m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
        m_streakBgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
        m_lastMotionBoxes.clear();
        m_motionPersistenceRemaining = 0;
        m_motionConfirmCount = 0;
        m_lastStreakDetections.clear();
        m_streakPersistenceRemaining = 0;
    }

    if (force
        || settingsKeys.contains("diffMask")
        || settingsKeys.contains("diffThreshold")
        || settingsKeys.contains("diffMaskOpenSize")
        || settingsKeys.contains("dilationSize")
        || settingsKeys.contains("diffMaskHistoryFrames")
        || settingsKeys.contains("diffMaskCloseSize")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight"))
    {
        m_diffMaskHistory.clear();
    }

    if ((force && !m_settings.m_diffMask)
        || (settingsKeys.contains("diffMask") && !m_settings.m_diffMask)
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight")) {
        m_previousInputFrame = CameraPipelineFrame();
        m_lastInputFrame = CameraPipelineFrame();
    }

    if (force
        || settingsKeys.contains("motionDetect")
        || settingsKeys.contains("motionBackgroundSubtractor")
        || settingsKeys.contains("motionHistory")
        || settingsKeys.contains("motionVarThreshold")
        || settingsKeys.contains("motionLearningRate")
        || settingsKeys.contains("motionDownscale")
        || settingsKeys.contains("motionDetectShadows")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight"))
    {
        m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
    }

    if (force
        || settingsKeys.contains("motionDetect")
        || settingsKeys.contains("motionBackgroundSubtractor")
        || settingsKeys.contains("motionHistory")
        || settingsKeys.contains("motionVarThreshold")
        || settingsKeys.contains("motionLearningRate")
        || settingsKeys.contains("motionConfirmFrames")
        || settingsKeys.contains("motionDownscale")
        || settingsKeys.contains("motionDetectShadows")
        || settingsKeys.contains("motionOpenSize")
        || settingsKeys.contains("motionCloseSize")
        || settingsKeys.contains("motionPersistenceFrames")
        || settingsKeys.contains("minContourArea")
        || settingsKeys.contains("motionExclusionRects")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight"))
    {
        m_lastMotionBoxes.clear();
        m_motionPersistenceRemaining = 0;
        m_motionConfirmCount = 0;
    }

    if (force
        || settingsKeys.contains("streakDetect")
        || settingsKeys.contains("streakDownscale")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight"))
    {
        m_streakBgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
        m_lastStreakDetections.clear();
        m_streakPersistenceRemaining = 0;
    }

    if (settingsKeys.contains("yoloModelPath") || (force && m_yoloLoadedModelPath != m_settings.m_yoloModelPath))
    {
        m_yoloNet = cv::dnn::Net();
        m_yoloLoadedModelPath.clear();
    }

    if (settingsKeys.contains("yoloLabelsPath") || (force && m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath))
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();
    }

    if ((force && !m_settings.m_yoloEnabled)
        || settingsKeys.contains("yoloEnabled")
        || settingsKeys.contains("yoloLabelsPath")
        || settingsKeys.contains("objectDeviceSettings"))
    {
        m_detectedObjectClasses.clear();
        m_pendingDisappearDeadlines.clear();
    }

    const bool detectorVisualsChanged = force
        || settingsKeys.contains("diffMask")
        || settingsKeys.contains("diffThreshold")
        || settingsKeys.contains("diffMaskOpenSize")
        || settingsKeys.contains("dilationSize")
        || settingsKeys.contains("diffMaskHistoryFrames")
        || settingsKeys.contains("diffMaskCloseSize")
        || settingsKeys.contains("motionDetect")
        || settingsKeys.contains("motionBackgroundSubtractor")
        || settingsKeys.contains("motionHistory")
        || settingsKeys.contains("motionVarThreshold")
        || settingsKeys.contains("motionLearningRate")
        || settingsKeys.contains("motionConfirmFrames")
        || settingsKeys.contains("motionDownscale")
        || settingsKeys.contains("motionDetectShadows")
        || settingsKeys.contains("motionOpenSize")
        || settingsKeys.contains("motionCloseSize")
        || settingsKeys.contains("motionPersistenceFrames")
        || settingsKeys.contains("minContourArea")
        || settingsKeys.contains("motionExclusionRects")
        || settingsKeys.contains("streakDetect")
        || settingsKeys.contains("streakThreshold")
        || settingsKeys.contains("streakMinLength")
        || settingsKeys.contains("streakHoughThreshold")
        || settingsKeys.contains("streakMaxGap")
        || settingsKeys.contains("streakPersistenceFrames")
        || settingsKeys.contains("streakDownscale")
        || settingsKeys.contains("streakColor")
        || settingsKeys.contains("streakDebugView")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight");

    if (detectorVisualsChanged) {
        reprocessLastFrame();
    }
}

void CameraDetector::reprocessLastFrame()
{
    if (m_lastInputFrame.m_image.isNull()) {
        return;
    }

    CameraPipelineFramePtr frame(new CameraPipelineFrame(m_lastInputFrame));
    processFrame(frame, m_previousInputFrame, false);
}

void CameraDetector::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            qDebug() << "CameraDetector: Dropping pending frame in favor of new frame";
        }
        m_pendingFrame = frame;
        if (!m_processingFrame)
        {
            m_processingFrame = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraDetector::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraDetector::processNextFrame()
{
    CameraPipelineFramePtr frame;

    {
        QMutexLocker locker(&m_frameMutex);
        frame = m_pendingFrame;
        m_pendingFrame.reset();

        if (!frame)
        {
            m_processingFrame = false;
            return;
        }
    }

    processNewFrame(frame);

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            schedule = true;
        } else {
            m_processingFrame = false;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraDetector::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    processFrame(frame, m_lastInputFrame, true);
}

void CameraDetector::processFrame(const CameraPipelineFramePtr& frame, const CameraPipelineFrame& diffReferenceFrame, bool updateInputHistory)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    CameraPipelineFrame inputFrameSnapshot(*frame);
    frame->m_motionBoxes.clear();
    frame->m_detections.clear();
    frame->m_streakDetections.clear();

    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
    const cv::Rect detectionRoi = resolveDetectionRoi(bgrMat.size());

    if (m_settings.m_diffMask && !diffReferenceFrame.m_image.isNull()
        && diffReferenceFrame.m_image.width() == frame->m_image.width()
        && diffReferenceFrame.m_image.height() == frame->m_image.height()) {
        applyDiffMask(bgrMat, detectionRoi, diffReferenceFrame);
    }

    if (m_settings.m_motionDetect) {
        cv::Mat motionDebugMask;
        applyMotionDetection(
            bgrMat,
            detectionRoi,
            frame->m_motionBoxes,
            (m_settings.m_motionMaskView != CameraSettings::MotionMaskViewOff) ? &motionDebugMask : nullptr);

        if (!motionDebugMask.empty())
        {
            cv::Mat maskCanvas = cv::Mat::zeros(bgrMat.size(), CV_8UC1);
            cv::Mat roiMask = motionDebugMask;
            if (motionDebugMask.size() != detectionRoi.size()) {
                cv::resize(motionDebugMask, roiMask, detectionRoi.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            roiMask.copyTo(maskCanvas(detectionRoi));
            cv::cvtColor(maskCanvas, bgrMat, cv::COLOR_GRAY2BGR);
        }
    }

    if (m_settings.m_streakDetect) {
        cv::Mat streakDebugMask;
        applyStreakDetection(
            bgrMat,
            detectionRoi,
            frame->m_streakDetections,
            updateInputHistory,
            (m_settings.m_streakDebugView != CameraSettings::StreakDebugViewOff) ? &streakDebugMask : nullptr);

        if (!streakDebugMask.empty())
        {
            cv::Mat maskCanvas = cv::Mat::zeros(bgrMat.size(), streakDebugMask.type());
            cv::Mat roiMask = streakDebugMask;
            if (streakDebugMask.size() != detectionRoi.size()) {
                cv::resize(streakDebugMask, roiMask, detectionRoi.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            roiMask.copyTo(maskCanvas(detectionRoi));
            if (maskCanvas.channels() == 1) {
                cv::cvtColor(maskCanvas, bgrMat, cv::COLOR_GRAY2BGR);
            } else {
                bgrMat = maskCanvas;
            }
        }
    }

    if (m_settings.m_yoloEnabled && !m_settings.m_yoloModelPath.isEmpty()) {
        runYoloDetections(bgrMat, detectionRoi, frame->m_detections);
    }

    QSet<QString> currentDetectedClasses;
    for (const CameraPipelineDetection& detection : frame->m_detections) {
        currentDetectedClasses.insert(detection.m_label);
    }
    const QDateTime detectionTime = frame->m_captureDateTime.isValid() ? frame->m_captureDateTime : QDateTime::currentDateTime();
    processObjectDetections(currentDetectedClasses, detectionTime);

    frame->m_image = convertBgrToRgbImage(bgrMat);

    if (updateInputHistory)
    {
        m_previousInputFrame = m_lastInputFrame;
        m_lastInputFrame = inputFrameSnapshot;
    }

    if (m_nextStage) {
        m_nextStage->submitFrame(frame);
    }
}

cv::Rect CameraDetector::resolveDetectionRoi(const cv::Size& frameSize) const
{
    const int frameWidth = std::max(1, frameSize.width);
    const int frameHeight = std::max(1, frameSize.height);
    const int x = qBound(0, m_settings.m_detectionRoiX, frameWidth - 1);
    const int y = qBound(0, m_settings.m_detectionRoiY, frameHeight - 1);
    const int width = (m_settings.m_detectionRoiWidth <= 0)
        ? (frameWidth - x)
        : qBound(1, m_settings.m_detectionRoiWidth, frameWidth - x);
    const int height = (m_settings.m_detectionRoiHeight <= 0)
        ? (frameHeight - y)
        : qBound(1, m_settings.m_detectionRoiHeight, frameHeight - y);
    return cv::Rect(x, y, width, height);
}

void CameraDetector::applyDiffMask(cv::Mat& bgrMat, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame)
{
    PROFILER_START();
    QImage convertedPrevRgb;
    const QImage& prevRgb = ensureRgb888(diffReferenceFrame.m_image, convertedPrevRgb);
    cv::Mat prevMat = wrapRgb888Image(prevRgb);
    cv::Mat prevBgr;
    cv::cvtColor(prevMat, prevBgr, cv::COLOR_RGB2BGR);

    cv::Mat gray, prevGray;
    cv::cvtColor(bgrMat, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(prevBgr, prevGray, cv::COLOR_BGR2GRAY);
    cv::Mat diff;
    cv::absdiff(gray(roi), prevGray(roi), diff);
    cv::Mat mask;
    cv::threshold(diff, mask, m_settings.m_diffThreshold, 255, cv::THRESH_BINARY);

    if (m_settings.m_diffMaskOpenSize > 0)
    {
        const int openKsize = 2 * m_settings.m_diffMaskOpenSize + 1;
        const cv::Mat openKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(openKsize, openKsize));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, openKernel);
    }

    if (m_settings.m_dilationSize > 0)
    {
        const int ksize = 2 * m_settings.m_dilationSize + 1;
        const cv::Mat dilationKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::dilate(mask, mask, dilationKernel);
    }

    if (!m_diffMaskHistory.empty() &&
        (m_diffMaskHistory.front().size() != mask.size() || m_diffMaskHistory.front().type() != mask.type()))
    {
        m_diffMaskHistory.clear();
    }

    m_diffMaskHistory.push_back(mask.clone());

    const size_t historyFrames = static_cast<size_t>(std::max(1, m_settings.m_diffMaskHistoryFrames));
    while (m_diffMaskHistory.size() > historyFrames) {
        m_diffMaskHistory.pop_front();
    }

    cv::Mat combinedMask = m_diffMaskHistory.front().clone();
    for (size_t i = 1; i < m_diffMaskHistory.size(); ++i) {
        cv::bitwise_or(combinedMask, m_diffMaskHistory[i], combinedMask);
    }
    cv::bitwise_and(combinedMask, buildExclusionMask(roi, combinedMask.size()), combinedMask);
    if (m_settings.m_diffMaskCloseSize > 0) {
        const int closeKsize = 2 * m_settings.m_diffMaskCloseSize + 1;
        const cv::Mat closeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closeKsize, closeKsize));
        cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, closeKernel);
    }

    cv::Mat result = cv::Mat::zeros(bgrMat.size(), bgrMat.type());
    cv::Mat fullMask = cv::Mat::zeros(bgrMat.rows, bgrMat.cols, combinedMask.type());
    combinedMask.copyTo(fullMask(roi));
    cv::bitwise_and(bgrMat, bgrMat, result, fullMask);
    bgrMat = result;
    PROFILER_STOP(__FUNCTION__);
}

cv::Mat CameraDetector::buildExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const
{
    cv::Mat mask(workSize, CV_8UC1, cv::Scalar(255));

    if (m_settings.m_motionExclusionRects.isEmpty()) {
        return mask;
    }

    const double scaleX = static_cast<double>(workSize.width) / std::max(1, roi.width);
    const double scaleY = static_cast<double>(workSize.height) / std::max(1, roi.height);

    for (const QRect& rect : m_settings.m_motionExclusionRects)
    {
        const QRect intersected = rect.intersected(QRect(roi.x, roi.y, roi.width, roi.height));
        if (intersected.isEmpty()) {
            continue;
        }

        const int x0 = std::clamp(static_cast<int>(std::floor((intersected.left() - roi.x) * scaleX)), 0, std::max(0, workSize.width - 1));
        const int y0 = std::clamp(static_cast<int>(std::floor((intersected.top() - roi.y) * scaleY)), 0, std::max(0, workSize.height - 1));
        const int x1 = std::clamp(static_cast<int>(std::ceil((intersected.right() + 1 - roi.x) * scaleX)), x0 + 1, workSize.width);
        const int y1 = std::clamp(static_cast<int>(std::ceil((intersected.bottom() + 1 - roi.y) * scaleY)), y0 + 1, workSize.height);

        cv::rectangle(mask, cv::Rect(x0, y0, x1 - x0, y1 - y0), cv::Scalar(0), cv::FILLED);
    }

    return mask;
}

cv::Ptr<cv::BackgroundSubtractor> CameraDetector::createBackgroundSubtractor() const
{
    if (m_settings.m_motionBackgroundSubtractor == CameraSettings::MotionBackgroundSubtractorKNN)
    {
        return cv::createBackgroundSubtractorKNN(
            m_settings.m_motionHistory,
            m_settings.m_motionVarThreshold,
            m_settings.m_motionDetectShadows);
    }

    return cv::createBackgroundSubtractorMOG2(
        m_settings.m_motionHistory,
        m_settings.m_motionVarThreshold,
        m_settings.m_motionDetectShadows);
}

cv::Ptr<cv::BackgroundSubtractor> CameraDetector::createStreakBackgroundSubtractor() const
{
    constexpr int streakHistory = 4;
    constexpr double streakVarThreshold = 16.0;
    constexpr bool streakDetectShadows = false;
    return cv::createBackgroundSubtractorMOG2(streakHistory, streakVarThreshold, streakDetectShadows);
}

void CameraDetector::applyMotionDetection(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<QRect>& motionBoxes, cv::Mat* debugMask)
{
    PROFILER_START();
    if (!m_bgSubtractor) {
        m_bgSubtractor = createBackgroundSubtractor();
    }

    const double downscale = m_settings.m_motionDownscale;
    cv::Mat motionInput = bgrMat(roi);
    cv::Mat downscaledInput;
    if (downscale < 0.999)
    {
        const cv::Size downscaledSize(
            std::max(1, static_cast<int>(std::lround(roi.width * downscale))),
            std::max(1, static_cast<int>(std::lround(roi.height * downscale))));
        cv::resize(motionInput, downscaledInput, downscaledSize, 0.0, 0.0, cv::INTER_AREA);
        motionInput = downscaledInput;
    }

    cv::Mat fgMask;
    m_bgSubtractor->apply(motionInput, fgMask, m_settings.m_motionLearningRate);
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewRaw)) {
        *debugMask = fgMask.clone();
    }
    cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);
    cv::bitwise_and(fgMask, buildExclusionMask(roi, fgMask.size()), fgMask);
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewThresholded)) {
        *debugMask = fgMask.clone();
    }

    if (m_settings.m_motionOpenSize > 0)
    {
        const int ksize = 2 * m_settings.m_motionOpenSize + 1;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(fgMask, fgMask, cv::MORPH_OPEN, kernel);
    }
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewOpened)) {
        *debugMask = fgMask.clone();
    }

    if (m_settings.m_motionCloseSize > 0)
    {
        const int ksize = 2 * m_settings.m_motionCloseSize + 1;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, kernel);
    }
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewClosed)) {
        *debugMask = fgMask.clone();
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewFinal)) {
        *debugMask = fgMask.clone();
    }

    QVector<QRect> boxes;
    boxes.reserve(static_cast<qsizetype>(contours.size()));
    const double scaledMinArea = static_cast<double>(m_settings.m_minContourArea) * downscale * downscale;
    for (const auto& contour : contours)
    {
        if (cv::contourArea(contour) >= scaledMinArea) {
            cv::Rect box = cv::boundingRect(contour);
            if (downscale < 0.999)
            {
                box.x = static_cast<int>(std::floor(box.x / downscale));
                box.y = static_cast<int>(std::floor(box.y / downscale));
                box.width = std::max(1, static_cast<int>(std::ceil(box.width / downscale)));
                box.height = std::max(1, static_cast<int>(std::ceil(box.height / downscale)));
            }
            box.x += roi.x;
            box.y += roi.y;
            boxes.append(QRect(box.x, box.y, box.width, box.height));
        }
    }

    if (!boxes.isEmpty()) {
        m_motionConfirmCount = std::min(m_settings.m_motionConfirmFrames, m_motionConfirmCount + 1);
        if (m_motionConfirmCount < m_settings.m_motionConfirmFrames) {
            boxes.clear();
        }
    } else {
        m_motionConfirmCount = 0;
    }

    if (!boxes.isEmpty()) {
        m_lastMotionBoxes = boxes;
        m_motionPersistenceRemaining = m_settings.m_motionPersistenceFrames;
    } else if ((m_motionPersistenceRemaining > 0) && !m_lastMotionBoxes.isEmpty()) {
        boxes = m_lastMotionBoxes;
        --m_motionPersistenceRemaining;
    } else {
        m_lastMotionBoxes.clear();
        m_motionPersistenceRemaining = 0;
    }

    motionBoxes = boxes;
    PROFILER_STOP(__FUNCTION__);
}

void CameraDetector::applyStreakDetection(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineStreakDetection>& streakDetections, bool updateBackgroundModel, cv::Mat* debugMask)
{
    PROFILER_START();

    cv::Mat currentGray;
    cv::cvtColor(bgrMat(roi), currentGray, cv::COLOR_BGR2GRAY);

    const double downscale = m_settings.m_streakDownscale;
    if (downscale < 0.999)
    {
        const cv::Size downscaledSize(
            std::max(1, static_cast<int>(std::lround(roi.width * downscale))),
            std::max(1, static_cast<int>(std::lround(roi.height * downscale))));
        cv::resize(currentGray, currentGray, downscaledSize, 0.0, 0.0, cv::INTER_AREA);
    }

    if (!m_streakBgSubtractor) {
        m_streakBgSubtractor = createStreakBackgroundSubtractor();
    }

    cv::Mat backgroundGray;
    m_streakBgSubtractor->getBackgroundImage(backgroundGray);

    cv::Mat foregroundMask;
    const double streakLearningRate = updateBackgroundModel ? 0.25 : 0.0;
    m_streakBgSubtractor->apply(currentGray, foregroundMask, streakLearningRate);

    cv::Mat diff;
    if (!backgroundGray.empty() && (backgroundGray.size() == currentGray.size()))
    {
        if (backgroundGray.type() != currentGray.type()) {
            backgroundGray.convertTo(backgroundGray, currentGray.type());
        }
        cv::absdiff(currentGray, backgroundGray, diff);
    }
    else
    {
        diff = foregroundMask.clone();
    }

    if (debugMask && (m_settings.m_streakDebugView == CameraSettings::StreakDebugViewDiff)) {
        double minValue = 0.0;
        double maxValue = 0.0;
        cv::minMaxLoc(diff, &minValue, &maxValue);
        if (maxValue > minValue) {
            cv::normalize(diff, *debugMask, 0, 255, cv::NORM_MINMAX);
            debugMask->convertTo(*debugMask, CV_8UC1);
        } else {
            *debugMask = diff.clone();
        }
    }
    cv::threshold(diff, diff, m_settings.m_streakThreshold, 255, cv::THRESH_BINARY);
    cv::threshold(foregroundMask, foregroundMask, 126, 255, cv::THRESH_BINARY);
    cv::bitwise_and(diff, foregroundMask, diff);

    cv::Mat exclusionMask = buildExclusionMask(roi, diff.size());
    cv::bitwise_and(diff, exclusionMask, diff);
    if (debugMask && (m_settings.m_streakDebugView == CameraSettings::StreakDebugViewThresholded)) {
        *debugMask = diff.clone();
    }

    cv::Mat edges;
    cv::Canny(diff, edges, std::max(1, m_settings.m_streakThreshold / 2), std::max(2, m_settings.m_streakThreshold));
    if (debugMask && (m_settings.m_streakDebugView == CameraSettings::StreakDebugViewEdges)) {
        *debugMask = edges.clone();
    }

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(
        edges,
        lines,
        1.0,
        CV_PI / 180.0,
        std::max(1, m_settings.m_streakHoughThreshold),
        std::max(1, m_settings.m_streakMinLength),
        std::max(0.0, m_settings.m_streakMaxGap));

    cv::Mat lineMask;
    if (debugMask && ((m_settings.m_streakDebugView == CameraSettings::StreakDebugViewLines)
        || (m_settings.m_streakDebugView == CameraSettings::StreakDebugViewFinal)))
    {
        lineMask = cv::Mat::zeros(edges.size(), CV_8UC1);
        for (const cv::Vec4i& line : lines) {
            cv::line(
                lineMask,
                cv::Point(line[0], line[1]),
                cv::Point(line[2], line[3]),
                cv::Scalar(255),
                2,
                cv::LINE_AA);
        }
        if (m_settings.m_streakDebugView == CameraSettings::StreakDebugViewLines) {
            *debugMask = lineMask.clone();
        }
    }

    QVector<CameraPipelineStreakDetection> detections;
    detections.reserve(static_cast<qsizetype>(lines.size()));

    for (const cv::Vec4i& line : lines)
    {
        QPointF p1(line[0], line[1]);
        QPointF p2(line[2], line[3]);

        if (downscale < 0.999)
        {
            p1 = p1 / downscale;
            p2 = p2 / downscale;
        }

        p1 += QPointF(roi.x, roi.y);
        p2 += QPointF(roi.x, roi.y);

        const QLineF qline(p1, p2);
        if (qline.length() < m_settings.m_streakMinLength) {
            continue;
        }

        CameraPipelineStreakDetection detection;
        detection.m_line = qline;
        detection.m_label = QStringLiteral("Streak");
        detection.m_score = static_cast<float>(qline.length());
        detections.append(detection);
    }

    if (detections.size() > 1)
    {
        const double angleToleranceDegrees = 8.0;
        const double angleToleranceRadians = angleToleranceDegrees * CV_PI / 180.0;
        const double maxPerpendicularDistance = std::max(4.0, m_settings.m_streakMaxGap);
        const double strongOverlapPerpendicularDistance = std::max(12.0, m_settings.m_streakMaxGap * 2.0);
        const double maxProjectedGap = std::max(12.0, m_settings.m_streakMaxGap * 2.0);

        auto canonicalDirection = [](const QLineF& line) -> QPointF
        {
            const double length = line.length();
            if (length <= 0.0) {
                return QPointF(1.0, 0.0);
            }

            QPointF direction = (line.p2() - line.p1()) / length;
            if ((direction.x() < 0.0) || ((std::abs(direction.x()) < 1e-9) && (direction.y() < 0.0))) {
                direction = -direction;
            }
            return direction;
        };

        auto dotProduct = [](const QPointF& a, const QPointF& b) -> double
        {
            return a.x() * b.x() + a.y() * b.y();
        };

        auto perpendicularDistance = [&](const QLineF& a, const QLineF& b) -> double
        {
            const QPointF direction = canonicalDirection(a);
            const QPointF normal(-direction.y(), direction.x());
            const QPointF delta = b.center() - a.center();
            return std::abs(dotProduct(delta, normal));
        };

        auto projectedGap = [&](const QLineF& a, const QLineF& b) -> double
        {
            const QPointF direction = canonicalDirection(a);
            const QPointF origin = a.p1();
            const double a0 = dotProduct(a.p1() - origin, direction);
            const double a1 = dotProduct(a.p2() - origin, direction);
            const double b0 = dotProduct(b.p1() - origin, direction);
            const double b1 = dotProduct(b.p2() - origin, direction);
            const double aMin = std::min(a0, a1);
            const double aMax = std::max(a0, a1);
            const double bMin = std::min(b0, b1);
            const double bMax = std::max(b0, b1);

            if ((aMax >= bMin) && (bMax >= aMin)) {
                return 0.0;
            }

            return (bMin > aMax) ? (bMin - aMax) : (aMin - bMax);
        };

        auto projectedOverlap = [&](const QLineF& a, const QLineF& b) -> double
        {
            const QPointF direction = canonicalDirection(a);
            const QPointF origin = a.p1();
            const double a0 = dotProduct(a.p1() - origin, direction);
            const double a1 = dotProduct(a.p2() - origin, direction);
            const double b0 = dotProduct(b.p1() - origin, direction);
            const double b1 = dotProduct(b.p2() - origin, direction);
            const double aMin = std::min(a0, a1);
            const double aMax = std::max(a0, a1);
            const double bMin = std::min(b0, b1);
            const double bMax = std::max(b0, b1);
            return std::max(0.0, std::min(aMax, bMax) - std::max(aMin, bMin));
        };

        auto mergePair = [&](const CameraPipelineStreakDetection& first, const CameraPipelineStreakDetection& second) -> CameraPipelineStreakDetection
        {
            const QPointF firstDirection = canonicalDirection(first.m_line);
            const QPointF secondDirection = canonicalDirection(second.m_line);
            QPointF mergedDirection = firstDirection + secondDirection;
            const double mergedDirLength = std::hypot(mergedDirection.x(), mergedDirection.y());
            if (mergedDirLength <= 1e-9) {
                mergedDirection = firstDirection;
            } else {
                mergedDirection /= mergedDirLength;
            }

            const QPointF normal(-mergedDirection.y(), mergedDirection.x());
            const QPointF points[] = { first.m_line.p1(), first.m_line.p2(), second.m_line.p1(), second.m_line.p2() };

            double minProjection = std::numeric_limits<double>::max();
            double maxProjection = -std::numeric_limits<double>::max();
            double normalSum = 0.0;
            for (const QPointF& point : points)
            {
                const double parallel = dotProduct(point, mergedDirection);
                minProjection = std::min(minProjection, parallel);
                maxProjection = std::max(maxProjection, parallel);
                normalSum += dotProduct(point, normal);
            }

            const double averageNormal = normalSum / 4.0;
            const QPointF start = mergedDirection * minProjection + normal * averageNormal;
            const QPointF end = mergedDirection * maxProjection + normal * averageNormal;

            CameraPipelineStreakDetection merged = first;
            merged.m_line = QLineF(start, end);
            merged.m_score = static_cast<float>(merged.m_line.length());
            return merged;
        };

        bool mergedAny = true;
        while (mergedAny)
        {
            mergedAny = false;
            for (int i = 0; i < detections.size() && !mergedAny; ++i)
            {
                for (int j = i + 1; j < detections.size(); ++j)
                {
                    const QLineF& first = detections[i].m_line;
                    const QLineF& second = detections[j].m_line;
                    const QPointF dirA = canonicalDirection(first);
                    const QPointF dirB = canonicalDirection(second);
                    const double cosine = std::clamp(dotProduct(dirA, dirB), -1.0, 1.0);
                    const double angle = std::acos(cosine);

                    if (angle > angleToleranceRadians) {
                        continue;
                    }

                    const double perpendicular = perpendicularDistance(first, second);
                    const double overlap = projectedOverlap(first, second);
                    const double minLength = std::min(first.length(), second.length());
                    const bool stronglyOverlapping = (minLength > 0.0) && (overlap >= minLength * 0.5);

                    if (perpendicular > (stronglyOverlapping ? strongOverlapPerpendicularDistance : maxPerpendicularDistance)) {
                        continue;
                    }

                    if (projectedGap(first, second) > maxProjectedGap) {
                        continue;
                    }

                    detections[i] = mergePair(detections[i], detections[j]);
                    detections.removeAt(j);
                    mergedAny = true;
                    break;
                }
            }
        }
    }

    if (!detections.isEmpty()) {
        m_lastStreakDetections = detections;
        m_streakPersistenceRemaining = m_settings.m_streakPersistenceFrames;
    } else if ((m_streakPersistenceRemaining > 0) && !m_lastStreakDetections.isEmpty()) {
        detections = m_lastStreakDetections;
        --m_streakPersistenceRemaining;
    } else {
        m_lastStreakDetections.clear();
        m_streakPersistenceRemaining = 0;
    }

    if (debugMask && (m_settings.m_streakDebugView == CameraSettings::StreakDebugViewFinal))
    {
        if (lineMask.empty()) {
            lineMask = cv::Mat::zeros(edges.size(), CV_8UC1);
        } else {
            lineMask = lineMask.clone();
        }

        for (const CameraPipelineStreakDetection& detection : detections)
        {
            QPointF p1 = detection.m_line.p1() - QPointF(roi.x, roi.y);
            QPointF p2 = detection.m_line.p2() - QPointF(roi.x, roi.y);
            if (downscale < 0.999) {
                p1 *= downscale;
                p2 *= downscale;
            }

            cv::line(
                lineMask,
                cv::Point(cvRound(p1.x()), cvRound(p1.y())),
                cv::Point(cvRound(p2.x()), cvRound(p2.y())),
                cv::Scalar(255),
                3,
                cv::LINE_AA);
        }

        *debugMask = lineMask;
    }

    streakDetections = detections;
    PROFILER_STOP(__FUNCTION__);
}

void CameraDetector::runYoloDetections(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineDetection>& detections)
{
    PROFILER_START();

    if (m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath)
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();

        if (!m_settings.m_yoloLabelsPath.isEmpty() && !(m_settings.m_yoloLabelsPath.startsWith("http://") || m_settings.m_yoloLabelsPath.startsWith("https://")))
        {
            QFile f(m_settings.m_yoloLabelsPath);
            if (f.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QTextStream ts(&f);
                while (!ts.atEnd())
                {
                    const QString line = ts.readLine().trimmed();
                    if (!line.isEmpty()) {
                        m_yoloLabels.append(line);
                    }
                }
                m_yoloLoadedLabelsPath = m_settings.m_yoloLabelsPath;
            }
            else
            {
                qWarning() << "CameraDetector::runYoloDetections: cannot open labels file:" << m_settings.m_yoloLabelsPath;
            }
        }
    }

    if (m_yoloLoadedModelPath != m_settings.m_yoloModelPath)
    {
        m_yoloNet = cv::dnn::Net();
        m_yoloInputSize = cv::Size(640, 640);
        m_yoloLoadedModelPath.clear();

        if (!m_settings.m_yoloModelPath.isEmpty() && !(m_settings.m_yoloModelPath.startsWith("http://") || m_settings.m_yoloModelPath.startsWith("https://")))
        {
            try
            {
                const QString localFile = m_settings.m_yoloModelPath;

                m_yoloNet = cv::dnn::readNetFromONNX(localFile.toStdString());

                const cv::Size modelInputSize = readOnnxInputSize(localFile);

                if ((modelInputSize.width > 0) && (modelInputSize.height > 0))
                {
                    m_yoloInputSize = modelInputSize;
                }
                else
                {
                    qWarning() << "CameraDetector::runYoloDetections: unable to read model input size, using fallback 640x640 for"
                               << localFile;
                }

                m_yoloLoadedModelPath = m_settings.m_yoloModelPath;
                qDebug() << "CameraDetector::runYoloDetections: loaded model" << localFile
                         << "with input size" << m_yoloInputSize.width << "x" << m_yoloInputSize.height;
            }
            catch (const cv::Exception& e)
            {
                qWarning() << "CameraDetector::runYoloDetections: failed to load model:" << e.what();
                return;
            }
        }
        else
        {
            return;
        }
    }

    if (m_yoloNet.empty()) {
        return;
    }

    if (m_settings.m_yoloDnnTarget == CameraSettings::CUDA)
    {
        m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else if (m_settings.m_yoloDnnTarget == CameraSettings::CUDA_FP16)
    {
        m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    cv::Mat roiMat = bgrMat(roi);
    const float scaleX = static_cast<float>(roiMat.cols) / m_yoloInputSize.width;
    const float scaleY = static_cast<float>(roiMat.rows) / m_yoloInputSize.height;

    cv::Mat blob;
    cv::dnn::blobFromImage(roiMat, blob, 1.0 / 255.0, m_yoloInputSize, cv::Scalar(), true, false);
    m_yoloNet.setInput(blob);

    std::vector<cv::Mat> outputs;
    try
    {
        m_yoloNet.forward(outputs, m_yoloNet.getUnconnectedOutLayersNames());
    }
    catch (const cv::Exception& e)
    {
        qWarning() << "CameraDetector::runYoloDetections: inference failed:" << e.what();
        return;
    }

    if (outputs.empty()) {
        return;
    }

    cv::Mat det = outputs[0];
    if (det.dims == 3) {
        det = det.reshape(1, det.size[1]);
    }

    const float confThresh = static_cast<float>(m_settings.m_yoloConfThreshold);
    const float nmsThresh = static_cast<float>(m_settings.m_yoloNmsThreshold);
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classIds;
    const bool isV8Style = (det.rows < det.cols);

    if (isV8Style)
    {
        const int numAnchors = det.cols;
        const int numClasses = det.rows - 4;
        if (numClasses <= 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            float bestScore = 0.0f;
            int bestClass = 0;
            for (int c = 0; c < numClasses; ++c)
            {
                const float s = det.at<float>(4 + c, a);
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = det.at<float>(0, a) * scaleX;
            const float cy = det.at<float>(1, a) * scaleY;
            const float w = det.at<float>(2, a) * scaleX;
            const float h = det.at<float>(3, a) * scaleY;

            boxes.push_back(cv::Rect(
                roi.x + static_cast<int>(cx - w / 2.0f),
                roi.y + static_cast<int>(cy - h / 2.0f),
                static_cast<int>(w),
                static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }
    else
    {
        const int numAnchors = det.rows;
        const int numClasses = det.cols - 5;
        if (numClasses < 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            const float objectness = det.at<float>(a, 4);
            if (objectness < confThresh) {
                continue;
            }

            float bestScore = 0.0f;
            int bestClass = 0;
            for (int c = 0; c < numClasses; ++c)
            {
                const float s = objectness * det.at<float>(a, 5 + c);
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = det.at<float>(a, 0) * scaleX;
            const float cy = det.at<float>(a, 1) * scaleY;
            const float w = det.at<float>(a, 2) * scaleX;
            const float h = det.at<float>(a, 3) * scaleY;

            boxes.push_back(cv::Rect(
                roi.x + static_cast<int>(cx - w / 2.0f),
                roi.y + static_cast<int>(cy - h / 2.0f),
                static_cast<int>(w),
                static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, confThresh, nmsThresh, indices);

    detections.reserve(static_cast<qsizetype>(indices.size()));
    for (int idx : indices)
    {
        QString label;
        if (!m_yoloLabels.isEmpty() && classIds[idx] < m_yoloLabels.size()) {
            label = m_yoloLabels[classIds[idx]];
        } else {
            label = QStringLiteral("cls%1").arg(classIds[idx]);
        }

        CameraPipelineDetection detection;
        detection.m_box = QRect(boxes[idx].x, boxes[idx].y, boxes[idx].width, boxes[idx].height);
        detection.m_label = label;
        detection.m_score = scores[idx];
        detections.append(detection);
    }

    PROFILER_STOP("YOLO");
}

const QImage& CameraDetector::ensureRgb888(const QImage& image, QImage& convertedImage)
{
    if (image.format() == QImage::Format_RGB888) {
        return image;
    }

    convertedImage = image.convertToFormat(QImage::Format_RGB888);
    return convertedImage;
}

cv::Mat CameraDetector::wrapRgb888Image(const QImage& image)
{
    return cv::Mat(image.height(), image.width(), CV_8UC3,
                   const_cast<uchar*>(image.constBits()),
                   static_cast<size_t>(image.bytesPerLine()));
}

QImage CameraDetector::convertBgrToRgbImage(const cv::Mat& bgrMat)
{
    PROFILER_START();
    QImage result(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
    cv::Mat rgbMat(result.height(), result.width(), CV_8UC3,
                   result.bits(),
                   static_cast<size_t>(result.bytesPerLine()));
    cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
    return result;
}

void CameraDetector::processObjectDetections(const QSet<QString>& currentDetectedClasses, const QDateTime& now)
{
    for (const QString& className : currentDetectedClasses)
    {
        m_pendingDisappearDeadlines.remove(className);

        if (!m_detectedObjectClasses.contains(className))
        {
            m_detectedObjectClasses.insert(className);
            applyObjectDetectedSettings(className);
        }
    }

    for (const QString& className : m_detectedObjectClasses)
    {
        if (!currentDetectedClasses.contains(className) && !m_pendingDisappearDeadlines.contains(className)) {
            m_pendingDisappearDeadlines.insert(className, now.addMSecs(static_cast<qint64>(m_settings.m_yoloDisappearDebounce * 1000.0)));
        }
    }

    QMutableHashIterator<QString, QDateTime> it(m_pendingDisappearDeadlines);
    while (it.hasNext())
    {
        it.next();

        if (currentDetectedClasses.contains(it.key())) {
            it.remove();
            continue;
        }

        if (it.value() <= now)
        {
            m_detectedObjectClasses.remove(it.key());
            applyObjectDisappearedSettings(it.key());
            it.remove();
        }
    }
}

bool CameraDetector::shouldRecordVideoForDetectedObjects() const
{
    for (const QString& className : m_detectedObjectClasses)
    {
        QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
        if (deviceSettingsList == nullptr) {
            continue;
        }

        for (CameraSettings::ObjectDeviceSettings *devSettings : *deviceSettingsList)
        {
            if (devSettings && devSettings->m_recordVideo) {
                return true;
            }
        }
    }

    return false;
}

void CameraDetector::setVideoRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveVideo == enabled) {
        return;
    }

    m_settings.m_saveVideo = enabled;

    if (m_nextStage) {
        m_nextStage->getInputMessageQueue()->push(CameraPostProcessor::MsgSetVideoRecordingEnabled::create(enabled));
    }
}

void CameraDetector::executeCommand(const QString& command, const QString& className)
{
    if (command.isEmpty()) {
        return;
    }

#if QT_CONFIG(process)
    const QString cmd = substituteObjectClass(command, className);
    QStringList allArgs = QProcess::splitCommand(cmd);

    if (allArgs.isEmpty()) {
        return;
    }

    qDebug() << "CameraDetector::executeCommand: Executing:" << allArgs;
    const QString program = allArgs.takeFirst();
    QProcess::startDetached(program, allArgs);
#else
    qWarning() << "CameraDetector::executeCommand: QProcess not supported. Can't run:" << command;
    (void) className;
#endif
}

void CameraDetector::saySpeech(const QString& speech, const QString& className)
{
    if (speech.isEmpty()) {
        return;
    }

    const QString expandedSpeech = substituteObjectClass(speech, className);

#ifdef QT_TEXTTOSPEECH_FOUND
    m_speech->say(expandedSpeech);
#else
    qWarning() << "CameraDetector::saySpeech: TextToSpeech not supported. Unable to say" << expandedSpeech;
#endif
}

void CameraDetector::applyObjectDetectedSettings(const QString& className)
{
    if (!m_settings.m_objectDeviceSettings.contains(className)) {
        return;
    }

    QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
    if (deviceSettingsList == nullptr) {
        return;
    }

    MainCore *mainCore = MainCore::instance();
    const MainSettings& mainSettings = mainCore->getSettings();
    const std::vector<DeviceSet*>& deviceSets = mainCore->getDeviceSets();

    for (int i = 0; i < deviceSettingsList->size(); ++i)
    {
        CameraSettings::ObjectDeviceSettings *devSettings = deviceSettingsList->at(i);
        if (devSettings == nullptr) {
            continue;
        }

        if (devSettings->m_deviceSetIndex < 0 || devSettings->m_deviceSetIndex >= static_cast<int>(deviceSets.size()))
        {
            qWarning() << "CameraDetector::applyObjectDetectedSettings: device set at"
                       << devSettings->m_deviceSetIndex << "does not exist";
            continue;
        }

        if (!devSettings->m_presetGroup.isEmpty())
        {
            const DeviceSet *deviceSet = deviceSets[devSettings->m_deviceSetIndex];
            QString presetType;
            if (deviceSet->m_deviceSourceEngine != nullptr) {
                presetType = "R";
            } else if (deviceSet->m_deviceSinkEngine != nullptr) {
                presetType = "T";
            } else if (deviceSet->m_deviceMIMOEngine != nullptr) {
                presetType = "M";
            }

            const Preset *preset = mainSettings.getPreset(
                devSettings->m_presetGroup,
                devSettings->m_presetFrequency,
                devSettings->m_presetDescription,
                presetType);

            if (preset != nullptr)
            {
                qDebug() << "CameraDetector::applyObjectDetectedSettings: loading preset"
                         << preset->getDescription() << "for class" << className
                         << "to device set" << devSettings->m_deviceSetIndex;
                mainCore->getMainMessageQueue()->push(
                    MainCore::MsgLoadPreset::create(preset, devSettings->m_deviceSetIndex));
            }
            else
            {
                qWarning() << "CameraDetector::applyObjectDetectedSettings: unable to get preset"
                           << devSettings->m_presetGroup
                           << devSettings->m_presetFrequency
                           << devSettings->m_presetDescription;
            }
        }

        if (devSettings->m_recordVideo) {
            setVideoRecordingEnabled(true);
        }
    }

    QTimer::singleShot(1000, this, [this, deviceSettingsList, className]()
    {
        for (int i = 0; i < deviceSettingsList->size(); ++i)
        {
            CameraSettings::ObjectDeviceSettings *devSettings = deviceSettingsList->at(i);
            if (devSettings == nullptr) {
                continue;
            }

            if (devSettings->m_startOnDetect) {
                ChannelWebAPIUtils::run(devSettings->m_deviceSetIndex);
            }

            if (devSettings->m_startStopFileSink) {
                ChannelWebAPIUtils::startStopFileSinks(devSettings->m_deviceSetIndex, true);
            }

            if (!devSettings->m_detectCommand.isEmpty()) {
                executeCommand(devSettings->m_detectCommand, className);
            }

            if (!devSettings->m_detectSpeech.isEmpty()) {
                saySpeech(devSettings->m_detectSpeech, className);
            }
        }
    });
}

void CameraDetector::applyObjectDisappearedSettings(const QString& className)
{
    if (!m_settings.m_objectDeviceSettings.contains(className)) {
        return;
    }

    QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
    if (deviceSettingsList == nullptr) {
        return;
    }

    for (int i = 0; i < deviceSettingsList->size(); ++i)
    {
        CameraSettings::ObjectDeviceSettings *devSettings = deviceSettingsList->at(i);
        if (devSettings == nullptr) {
            continue;
        }

        if (devSettings->m_startStopFileSink) {
            ChannelWebAPIUtils::startStopFileSinks(devSettings->m_deviceSetIndex, false);
        }

        if (devSettings->m_stopOnDisappear) {
            ChannelWebAPIUtils::stop(devSettings->m_deviceSetIndex);
        }

        if (!devSettings->m_disappearCommand.isEmpty()) {
            executeCommand(devSettings->m_disappearCommand, className);
        }

        if (!devSettings->m_disappearSpeech.isEmpty()) {
            saySpeech(devSettings->m_disappearSpeech, className);
        }
    }

    if (!shouldRecordVideoForDetectedObjects()) {
        setVideoRecordingEnabled(false);
    }
}
