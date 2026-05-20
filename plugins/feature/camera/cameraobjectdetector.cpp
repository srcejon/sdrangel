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
#include <QMutableHashIterator>
#include <QProcess>
#include <QTextStream>
#include <QTimer>

#include "maincore.h"
#include "channel/channelwebapiutils.h"
#include "device/deviceset.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"
#include "util/profiler.h"
#include "camera.h"
#include "cameraobjectdetector.h"
#include "camerapostprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraObjectDetector::MsgReportObjectDetectionHistory, Message)
MESSAGE_CLASS_DEFINITION(CameraObjectDetector::MsgClearObjectDetectionHistory, Message)

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

CameraObjectDetector::CameraObjectDetector(Camera *camera) :
    m_camera(camera),
    m_msgQueueToGUI(nullptr),
    m_msgQueueToFeature(nullptr),
    m_postProcessorInputMessageQueue(nullptr),
    m_yoloInputSize(640, 640)
#ifdef QT_TEXTTOSPEECH_FOUND
    , m_speech(new QTextToSpeech(this))
#endif
{
}

CameraObjectDetector::~CameraObjectDetector() = default;

bool CameraObjectDetector::handleStageMessage(const Message& cmd)
{
    if (MsgClearObjectDetectionHistory::match(cmd))
    {
        clearObjectDetectionHistory();
        return true;
    }

    return false;
}

void CameraObjectDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraObjectDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;
    CameraDetectionStage::applySettings(settings, settingsKeys, force);

    if (settingsKeys.contains("yoloModelPath") || (force && m_yoloLoadedModelPath != m_settings.m_yoloModelPath))
    {
        m_yoloNet = cv::dnn::Net();
        m_yoloLoadedModelPath.clear();
        m_reportedErrorKeys.clear();
    }

    if (settingsKeys.contains("yoloLabelsPath") || (force && m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath))
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();
        m_reportedErrorKeys.clear();
    }

    if ((force && !m_settings.m_yoloEnabled)
        || settingsKeys.contains("yoloEnabled")
        || settingsKeys.contains("yoloLabelsPath")
        || settingsKeys.contains("objectDeviceSettings"))
    {
        clearObjectDetectionState(false);
    }
}

void CameraObjectDetector::captureActiveChanged(bool active)
{
    if (active) {
        clearObjectDetectionState();
    }
}

void CameraObjectDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    frame->m_detections.clear();

    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
    const cv::Rect detectionRoi = resolveDetectionRoi(bgrMat.size());

    if (m_settings.m_yoloEnabled && !m_settings.m_yoloModelPath.isEmpty()) {
        runYoloDetections(bgrMat, detectionRoi, frame->m_detections);
    }

    const QDateTime detectionTime = frame->m_captureDateTime.isValid() ? frame->m_captureDateTime : QDateTime::currentDateTime();
    processObjectDetections(frame->m_detections, detectionTime, *frame);
    forwardFrame(frame);
}

void CameraObjectDetector::runYoloDetections(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineDetection>& detections)
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
                qWarning() << "CameraObjectDetector::runYoloDetections: cannot open labels file:" << m_settings.m_yoloLabelsPath;
                reportErrorToFeature(
                    QStringLiteral("yoloLabels:%1").arg(m_settings.m_yoloLabelsPath),
                    tr("YOLO labels file load failed"),
                    tr("Failed to open YOLO labels file:\n%1").arg(m_settings.m_yoloLabelsPath));
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
            const QString localFile = m_settings.m_yoloModelPath;
            try
            {
                m_yoloNet = cv::dnn::readNetFromONNX(localFile.toStdString());

                const cv::Size modelInputSize = readOnnxInputSize(localFile);

                if ((modelInputSize.width > 0) && (modelInputSize.height > 0))
                {
                    m_yoloInputSize = modelInputSize;
                }
                else
                {
                    qWarning() << "CameraObjectDetector::runYoloDetections: unable to read model input size, using fallback 640x640 for"
                               << localFile;
                }

                m_yoloLoadedModelPath = m_settings.m_yoloModelPath;
                qDebug() << "CameraObjectDetector::runYoloDetections: loaded model" << localFile
                         << "with input size" << m_yoloInputSize.width << "x" << m_yoloInputSize.height;
            }
            catch (const cv::Exception& e)
            {
                qWarning() << "CameraObjectDetector::runYoloDetections: failed to load model:" << e.what();
                reportErrorToFeature(
                    QStringLiteral("yoloModelLoad:%1").arg(localFile),
                    tr("YOLO model load failed"),
                    tr("Failed to load YOLO model:\n%1\n\n%2").arg(localFile, QString::fromUtf8(e.what())));
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
        qWarning() << "CameraObjectDetector::runYoloDetections: inference failed:" << e.what();
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
        if (intersectsExclusionRects(detection.m_box)) {
            continue;
        }
        detection.m_label = label;
        detection.m_score = scores[idx];
        detections.append(detection);
    }

    PROFILER_STOP("YOLO");
}

void CameraObjectDetector::clearObjectDetectionState(bool clearHistory)
{
    m_detectedObjectClasses.clear();
    m_pendingDisappearStates.clear();
    if (clearHistory) {
        clearObjectDetectionHistory();
    }
}

void CameraObjectDetector::clearObjectDetectionHistory()
{
    m_activeObjectDetectionHistory.clear();
    m_completedObjectDetectionHistory.clear();
    reportObjectDetectionHistoryToGUI();
}

QList<CameraDetectionHistoryEntry> CameraObjectDetector::getObjectDetectionHistorySnapshot() const
{
    QList<CameraDetectionHistoryEntry> history = m_completedObjectDetectionHistory;
    for (auto it = m_activeObjectDetectionHistory.cbegin(); it != m_activeObjectDetectionHistory.cend(); ++it) {
        history.append(it.value());
    }

    std::sort(history.begin(), history.end(), [](const CameraDetectionHistoryEntry& lhs, const CameraDetectionHistoryEntry& rhs) {
        if (lhs.m_firstDetected != rhs.m_firstDetected) {
            return lhs.m_firstDetected > rhs.m_firstDetected;
        }

        return lhs.m_label < rhs.m_label;
    });

    return history;
}

void CameraObjectDetector::reportObjectDetectionHistoryToGUI() const
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportObjectDetectionHistory::create(getObjectDetectionHistorySnapshot()));
    }
}

void CameraObjectDetector::reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage)
{
    if (!m_msgQueueToFeature || m_reportedErrorKeys.contains(errorKey)) {
        return;
    }

    m_reportedErrorKeys.insert(errorKey);
    m_msgQueueToFeature->push(Camera::MsgReportError::create(title, errorMessage));
}

void CameraObjectDetector::processObjectDetections(const QVector<CameraPipelineDetection>& detections, const QDateTime& now, CameraPipelineFrame& frame)
{
    QHash<QString, float> currentDetectedScores;
    QSet<QString> currentDetectedClasses;
    bool historyChanged = false;

    for (const CameraPipelineDetection& detection : detections)
    {
        if (detection.m_label.isEmpty()) {
            continue;
        }

        currentDetectedClasses.insert(detection.m_label);
        float& peakScore = currentDetectedScores[detection.m_label];
        peakScore = std::max(peakScore, detection.m_score);
    }

    for (const QString& className : currentDetectedClasses)
    {
        m_pendingDisappearStates.remove(className);

        const float currentPeakScore = currentDetectedScores.value(className);

        auto activeHistoryIt = m_activeObjectDetectionHistory.find(className);
        if (activeHistoryIt == m_activeObjectDetectionHistory.end())
        {
            CameraDetectionHistoryEntry entry;
            entry.m_label = className;
            entry.m_firstDetected = now;
            entry.m_peakConfidence = currentPeakScore;
            activeHistoryIt = m_activeObjectDetectionHistory.insert(className, entry);
            historyChanged = true;
        }
        else if (currentPeakScore > activeHistoryIt->m_peakConfidence)
        {
            activeHistoryIt->m_peakConfidence = currentPeakScore;
            historyChanged = true;
        }

        if (!m_detectedObjectClasses.contains(className))
        {
            m_detectedObjectClasses.insert(className);
            if (applyObjectDetectedSettings(className, now)) {
                frame.m_saveCurrentImage = true;
            }
        }
    }

    for (const QString& className : m_detectedObjectClasses)
    {
        if (!currentDetectedClasses.contains(className) && !m_pendingDisappearStates.contains(className))
        {
            PendingDisappearState state;
            state.m_firstMissing = now;
            state.m_deadline = now.addMSecs(static_cast<qint64>(m_settings.m_yoloDisappearDebounce * 1000.0));
            m_pendingDisappearStates.insert(className, state);
        }
    }

    QMutableHashIterator<QString, PendingDisappearState> it(m_pendingDisappearStates);
    while (it.hasNext())
    {
        it.next();

        if (currentDetectedClasses.contains(it.key())) {
            it.remove();
            continue;
        }

        if (it.value().m_deadline <= now)
        {
            m_detectedObjectClasses.remove(it.key());
            auto activeHistoryIt = m_activeObjectDetectionHistory.find(it.key());
            if (activeHistoryIt != m_activeObjectDetectionHistory.end())
            {
                activeHistoryIt->m_disappeared = it.value().m_firstMissing;
                m_completedObjectDetectionHistory.append(activeHistoryIt.value());
                m_activeObjectDetectionHistory.erase(activeHistoryIt);
                historyChanged = true;
            }
            applyObjectDisappearedSettings(it.key(), now);
            it.remove();
        }
    }

    if (historyChanged) {
        reportObjectDetectionHistoryToGUI();
    }
}

bool CameraObjectDetector::shouldRecordVideoForDetectedObjects() const
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

void CameraObjectDetector::setVideoRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveVideo == enabled) {
        return;
    }

    m_settings.m_saveVideo = enabled;

    if (m_postProcessorInputMessageQueue) {
        m_postProcessorInputMessageQueue->push(CameraPostProcessor::MsgSetVideoRecordingEnabled::create(enabled));
    }
}

void CameraObjectDetector::executeCommand(const QString& command, const QString& className)
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

    qDebug() << "CameraObjectDetector::executeCommand: Executing:" << allArgs;
    const QString program = allArgs.takeFirst();
    QProcess::startDetached(program, allArgs);
#else
    qWarning() << "CameraObjectDetector::executeCommand: QProcess not supported. Can't run:" << command;
    (void) className;
#endif
}

void CameraObjectDetector::saySpeech(const QString& speech, const QString& className)
{
    if (speech.isEmpty()) {
        return;
    }

    const QString expandedSpeech = substituteObjectClass(speech, className);

#ifdef QT_TEXTTOSPEECH_FOUND
    m_speech->say(expandedSpeech);
#else
    qWarning() << "CameraObjectDetector::saySpeech: TextToSpeech not supported. Unable to say" << expandedSpeech;
#endif
}

bool CameraObjectDetector::applyObjectDetectedSettings(const QString& className, const QDateTime& now)
{
    sendEvent(className, true, now);

    if (!m_settings.m_objectDeviceSettings.contains(className)) {
        return false;
    }

    QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
    if (deviceSettingsList == nullptr) {
        return false;
    }

    MainCore *mainCore = MainCore::instance();
    const MainSettings& mainSettings = mainCore->getSettings();
    const std::vector<DeviceSet*>& deviceSets = mainCore->getDeviceSets();
    bool saveCurrentImage = false;

    for (int i = 0; i < deviceSettingsList->size(); ++i)
    {
        CameraSettings::ObjectDeviceSettings *devSettings = deviceSettingsList->at(i);
        if (devSettings == nullptr) {
            continue;
        }

        if (devSettings->m_deviceSetIndex < 0 || devSettings->m_deviceSetIndex >= static_cast<int>(deviceSets.size()))
        {
            qWarning() << "CameraObjectDetector::applyObjectDetectedSettings: device set at"
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
                qDebug() << "CameraObjectDetector::applyObjectDetectedSettings: loading preset"
                         << preset->getDescription() << "for class" << className
                         << "to device set" << devSettings->m_deviceSetIndex;
                mainCore->getMainMessageQueue()->push(
                    MainCore::MsgLoadPreset::create(preset, devSettings->m_deviceSetIndex));
            }
            else
            {
                qWarning() << "CameraObjectDetector::applyObjectDetectedSettings: unable to get preset"
                           << devSettings->m_presetGroup
                           << devSettings->m_presetFrequency
                           << devSettings->m_presetDescription;
            }
        }

        if (devSettings->m_saveCurrentImage) {
            saveCurrentImage = true;
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

    return saveCurrentImage;
}

void CameraObjectDetector::applyObjectDisappearedSettings(const QString& className, const QDateTime& now)
{
    sendEvent(className, false, now);

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

void CameraObjectDetector::sendEvent(const QString& className, bool detected, const QDateTime& eventTime)
{
    QList<ObjectPipe*> eventPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_camera, "event", eventPipes);
    QString eventData = QString("name=%1").arg(className);
    MainCore::MsgEvent::EventType eventType = detected ? MainCore::MsgEvent::EventType::CameraObjectDetectedEvent : MainCore::MsgEvent::CameraObjectLostEvent;
    for (const auto& pipe : eventPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        messageQueue->push(MainCore::MsgEvent::create(m_camera, eventTime, eventType, eventData));
    }
}




