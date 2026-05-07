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
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QMutableHashIterator>
#include <QPainter>
#include <QProcess>
#include <QSharedPointer>
#include <QTextDocument>
#include <QTextStream>
#include <QTimer>
#include <QVector>

#include "maincore.h"
#include "channel/channelwebapiutils.h"
#include "device/deviceset.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"
#include "util/profiler.h"
#include "camerapostprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgConfigureCameraPostProcessor, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSpectrumFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportSaveVideoState, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgCaptureActive, Message)

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

CameraPostProcessor::CameraPostProcessor() :
    m_msgQueueToGUI(nullptr),
    m_captureActive(false)
#ifdef QT_TEXTTOSPEECH_FOUND
    , m_speech(new QTextToSpeech(this))
#endif
{}

CameraPostProcessor::~CameraPostProcessor()
{
    stopWork();
    m_inputMessageQueue.clear();
}

void CameraPostProcessor::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraPostProcessor::handleInputMessages);
    handleInputMessages();
}

void CameraPostProcessor::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraPostProcessor::handleInputMessages);

    if (m_videoWriter.isOpened()) {
        m_videoWriter.release();
    }
}

void CameraPostProcessor::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
        // Post processing can be slow, so drop image frames if the queue gets too large
        if (m_inputMessageQueue.size() > 20)
        {
            while (m_inputMessageQueue.size() > 0)
            {
                message = m_inputMessageQueue.pop();
                if (MsgProcessFrame::match(*message))
                {
                    qDebug() << "CameraPostProcessor: Dropping frame to catch up";
                    delete message;
                } else
                {
                    if (handleMessage(*message)) {
                        delete message;
                    }
                }
            }
        }
    }
}

bool CameraPostProcessor::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraPostProcessor::match(cmd))
    {
        MsgConfigureCameraPostProcessor& cfg = (MsgConfigureCameraPostProcessor&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        MsgProcessFrame& frameMsg = (MsgProcessFrame&) cmd;
        processNewFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgSpectrumFrame::match(cmd))
    {
        MsgSpectrumFrame& frameMsg = (MsgSpectrumFrame&) cmd;
        m_spectrumViewImage = frameMsg.getImage();
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        MsgCaptureActive& activeMsg = (MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();

        if (m_captureActive)
        {
            m_lastFrame = CameraPipelineFrame();
        }
        else if (m_videoWriter.isOpened())
        {
            m_videoWriter.release();
        }

        return true;
    }

    return false;
}

void CameraPostProcessor::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraPostProcessor::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    static const QStringList kPostProcessingKeys = {
        "overlayDateTime", "dateTimeColor",
        "dateTimeFormat", "dateTimePosX", "dateTimePosY",
        "overlayText", "overlayTextString", "overlayTextColor",
        "overlayTextFontFamily", "overlayTextFontScale", "overlayTextPosX", "overlayTextPosY",
        "overlayFontFamily", "overlayFontScale",
        "motionBoxColor",
        "overlaySpectrum", "spectrumDevice", "spectrumOffsetX", "spectrumOffsetY", "spectrumScale",
        "yoloBoxColor"
    };
    const bool postProcessChanged = force || std::any_of(kPostProcessingKeys.cbegin(), kPostProcessingKeys.cend(),
        [&settingsKeys](const QString& k) { return settingsKeys.contains(k); });
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
        m_lastFrame = CameraPipelineFrame();
    }

    if ((force && !m_settings.m_yoloEnabled)
        || settingsKeys.contains("yoloEnabled")
        || settingsKeys.contains("objectDeviceSettings"))
    {
        m_detectedObjectClasses.clear();
        m_pendingDisappearDeadlines.clear();
    }

    if (force || settingsKeys.contains("spectrumDevice")) {
        m_spectrumViewImage = QImage();
    }

    if (settingsKeys.contains("saveVideo") || settingsKeys.contains("videoFileName") || settingsKeys.contains("videoHwAcceleration"))
    {
        if (m_videoWriter.isOpened()) {
            m_videoWriter.release();
        }
    }

    if (postProcessChanged && !m_lastFrame.m_image.isNull()) {
        const QImage processed = applyPostProcessing(m_lastFrame);
        reportFrameToGUI(processed);
    }
}

void CameraPostProcessor::processNewFrame(const CameraPipelineFrame& frame)
{
    if (frame.m_image.isNull()) {
        return;
    }

    m_captureDateTime = frame.m_captureDateTime.isValid() ? frame.m_captureDateTime : QDateTime::currentDateTime();
    const QImage& pipelineImage = frame.m_image;
    const QImage& unprocessedImage = frame.m_unprocessedImage.isNull() ? frame.m_image : frame.m_unprocessedImage;
    const QSet<QString> currentDetectedClasses = [&frame]() {
        QSet<QString> classes;
        for (const CameraPipelineDetection& detection : frame.m_detections) {
            classes.insert(detection.m_label);
        }
        return classes;
    }();
    processObjectDetections(currentDetectedClasses, m_captureDateTime);
    const QImage processed = applyPostProcessing(frame);

    m_lastFrame = frame;

    reportFrameToGUI(processed);

    if (m_captureActive && m_settings.m_saveImage && !m_settings.m_imageFileName.isEmpty())
    {
        QFileInfo fileInfo(m_settings.m_imageFileName);
        QString filename = fileInfo.path() + "/" + fileInfo.baseName() + "." + QDateTime::currentDateTimeUtc().toString("yyyy-MM-ddTHH_mm_ss_zzz") + "." + fileInfo.suffix();
        qDebug() << "CameraPostProcessor: Saving image to" << filename;
        const QImage& frameToSave = m_settings.m_videoPostProcess ? processed : unprocessedImage;
        frameToSave.save(filename);
    }

    if (m_captureActive && m_settings.m_saveVideo && !m_settings.m_videoFileName.isEmpty())
    {
        if (!m_videoWriter.isOpened())
        {
            QFileInfo fileInfo(m_settings.m_videoFileName);
            QString filename = fileInfo.path() + "/" + fileInfo.baseName() + "." + QDateTime::currentDateTimeUtc().toString("yyyy-MM-ddTHH_mm_ss_zzz") + "." + fileInfo.suffix();

            const QImage& frameForSize = m_settings.m_videoPostProcess ? processed : unprocessedImage;
            const int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
            const std::vector<int> params = {
                cv::VIDEOWRITER_PROP_HW_ACCELERATION,
                m_settings.m_videoHwAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
            };
            m_videoWriter.open(
                filename.toStdString(),
                fourcc,
                m_settings.getCaptureFrameRate(),
                cv::Size(frameForSize.width(), frameForSize.height()),
                params);
            if (m_videoWriter.isOpened()) {
                qDebug() << "CameraPostProcessor opened:" << filename << "backend:" << m_videoWriter.getBackendName();
            } else {
                qWarning() << "CameraPostProcessor failed to open:" << filename;
            }
        }

        if (m_videoWriter.isOpened())
        {
            const QImage& frameToWrite = m_settings.m_videoPostProcess ? processed : unprocessedImage;
            const QImage rgb = frameToWrite.convertToFormat(QImage::Format_RGB888);
            cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                        const_cast<uchar*>(rgb.bits()),
                        static_cast<size_t>(rgb.bytesPerLine()));
            cv::Mat bgrMat;
            cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
            m_videoWriter.write(bgrMat);
        }
    }
}

void CameraPostProcessor::reportFrameToGUI(const QImage& image)
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportFrame::create(image));
    }
}

void CameraPostProcessor::applyMotionOverlay(cv::Mat& bgrMat, const QVector<QRect>& motionBoxes) const
{
    PROFILER_START();
    const QColor& bc = m_settings.m_motionBoxColor;
    const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
    for (const QRect& box : motionBoxes) {
        cv::Rect cvBox(box.x(), box.y(), box.width(), box.height());
        cv::rectangle(bgrMat, cvBox, boxColor, 2);
    }
    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyDetectionOverlay(cv::Mat& bgrMat, const QVector<CameraPipelineDetection>& detections) const
{
    PROFILER_START();
    const QColor& bc = m_settings.m_yoloBoxColor;
    const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
    const cv::Scalar textBg(0, 0, 0);

    for (const CameraPipelineDetection& detection : detections)
    {
        const cv::Rect box(detection.m_box.x(), detection.m_box.y(), detection.m_box.width(), detection.m_box.height());
        cv::rectangle(bgrMat, box, boxColor, 2);

        QString label = detection.m_label + QStringLiteral(" %1%").arg(static_cast<int>(detection.m_score * 100.0f + 0.5f));
        const std::string labelStd = label.toStdString();
        int baseLine = 0;
        const cv::Size textSize = cv::getTextSize(labelStd, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        const int labelY = std::max(box.y, textSize.height + 2);
        cv::rectangle(bgrMat,
                      cv::Point(box.x, labelY - textSize.height - 2),
                      cv::Point(box.x + textSize.width, labelY + baseLine),
                      textBg, cv::FILLED);
        cv::putText(bgrMat, labelStd, cv::Point(box.x, labelY), cv::FONT_HERSHEY_SIMPLEX, 0.5, boxColor, 1, cv::LINE_AA);
    }
    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applySpectrumOverlay(cv::Mat& bgrMat) const
{
    PROFILER_START();
    QImage specSrc = m_spectrumViewImage;
    if (qAbs(m_settings.m_spectrumScale - 1.0) > 1e-4)
    {
        const int sw = static_cast<int>(specSrc.width() * m_settings.m_spectrumScale);
        const int sh = static_cast<int>(specSrc.height() * m_settings.m_spectrumScale);
        if (sw > 0 && sh > 0) {
            specSrc = specSrc.scaled(sw, sh, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        }
    }

    const QImage specRgb = specSrc.convertToFormat(QImage::Format_RGBA8888);
    const int dstW = bgrMat.cols;
    const int dstH = bgrMat.rows;
    const int ox = m_settings.m_spectrumOffsetX;
    const int oy = m_settings.m_spectrumOffsetY;
    const int sw = specRgb.width();
    const int sh = specRgb.height();

    const int srcX0 = std::max(0, -ox);
    const int srcY0 = std::max(0, -oy);
    const int srcX1 = std::min(sw, dstW - ox);
    const int srcY1 = std::min(sh, dstH - oy);

    for (int sy = srcY0; sy < srcY1; ++sy)
    {
        const uchar* srcRow = specRgb.constScanLine(sy);
        const int dy = oy + sy;
        uchar* dstRow = bgrMat.ptr<uchar>(dy);

        for (int sx = srcX0; sx < srcX1; ++sx)
        {
            const int srcPx = sx * 4;
            const uchar alpha = srcRow[srcPx + 3];
            if (alpha == 0) {
                continue;
            }
            const int dx = (ox + sx) * 3;
            if (alpha == 255)
            {
                dstRow[dx] = srcRow[srcPx + 2];
                dstRow[dx + 1] = srcRow[srcPx + 1];
                dstRow[dx + 2] = srcRow[srcPx];
            }
            else
            {
                const int a = alpha;
                const int invA = 255 - a;
                dstRow[dx] = static_cast<uchar>((srcRow[srcPx + 2] * a + dstRow[dx] * invA) / 255);
                dstRow[dx + 1] = static_cast<uchar>((srcRow[srcPx + 1] * a + dstRow[dx + 1] * invA) / 255);
                dstRow[dx + 2] = static_cast<uchar>((srcRow[srcPx] * a + dstRow[dx + 2] * invA) / 255);
            }
        }
    }
    PROFILER_STOP(__FUNCTION__);
}

QImage CameraPostProcessor::convertBgrToRgbImage(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::cvtColor(bgrMat, bgrMat, cv::COLOR_BGR2RGB);
    const QImage rawResult(bgrMat.data, bgrMat.cols, bgrMat.rows,
                           static_cast<qsizetype>(bgrMat.step[0]),
                           QImage::Format_RGB888);
    return rawResult.copy();
}

void CameraPostProcessor::applyDateTimeOverlay(QImage& image) const
{
    PROFILER_START();
    const QString fmt = m_settings.m_dateTimeFormat.isEmpty()
                        ? QStringLiteral("yyyy-MM-dd hh:mm:ss")
                        : m_settings.m_dateTimeFormat;
    const QString text = m_captureDateTime.toString(fmt);
    QFont font;
    if (!m_settings.m_overlayFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_overlayFontFamily);
    }
    font.setPointSizeF(m_settings.m_overlayFontScale);
    const QFontMetrics fm(font);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setFont(font);
    painter.setPen(m_settings.m_dateTimeColor);
    const int x = m_settings.m_dateTimePosX;
    const int y = m_settings.m_dateTimePosY + fm.ascent();
    painter.drawText(x, y, text);
    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyTextOverlay(QImage& image, QTextDocument& overlayTextDocument) const
{
    PROFILER_START();
    QFont font;
    if (!m_settings.m_overlayTextFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_overlayTextFontFamily);
    }
    font.setPointSizeF(m_settings.m_overlayTextFontScale);
    overlayTextDocument.setDefaultFont(font);
    overlayTextDocument.setDefaultStyleSheet(QStringLiteral("* { color: %1; }").arg(m_settings.m_overlayTextColor.name()));
    overlayTextDocument.setHtml(QString("<div>%1</div>").arg(m_settings.m_overlayTextString));

    const int x = std::max(0, m_settings.m_overlayTextPosX);
    const qreal maxTextWidth = std::max(1, image.width() - x);
    overlayTextDocument.setTextWidth(maxTextWidth);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.save();
    painter.translate(x, m_settings.m_overlayTextPosY);
    overlayTextDocument.drawContents(&painter);
    painter.restore();
    PROFILER_STOP(__FUNCTION__);
}

QImage CameraPostProcessor::applyPostProcessing(const CameraPipelineFrame& frame)
{
    PROFILER_START();

    const QImage& input = frame.m_image;
    const bool needsSpectrumOverlay = m_settings.m_overlaySpectrum && !m_spectrumViewImage.isNull();
    QTextDocument overlayTextDocument;
    overlayTextDocument.setHtml(m_settings.m_overlayTextString);
    const bool needsTextOverlay = m_settings.m_overlayText && !overlayTextDocument.toPlainText().trimmed().isEmpty();
    const bool needsAny = m_settings.m_overlayDateTime
        || needsTextOverlay
        || !frame.m_motionBoxes.isEmpty()
        || !frame.m_detections.isEmpty()
        || needsSpectrumOverlay;

    if (!needsAny) {
        return input;
    }

    const QImage rgb = input.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    if (!frame.m_motionBoxes.isEmpty()) { applyMotionOverlay(bgrMat, frame.m_motionBoxes); }
    if (!frame.m_detections.isEmpty()) { applyDetectionOverlay(bgrMat, frame.m_detections); }
    if (needsSpectrumOverlay) { applySpectrumOverlay(bgrMat); }

    QImage result = convertBgrToRgbImage(bgrMat);

    if (m_settings.m_overlayDateTime) { applyDateTimeOverlay(result); }
    if (needsTextOverlay) { applyTextOverlay(result, overlayTextDocument); }

    PROFILER_STOP("CameraPostProcessor::applyPostProcessing");
    return result;
}

void CameraPostProcessor::processObjectDetections(const QSet<QString>& currentDetectedClasses, const QDateTime& now)
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

bool CameraPostProcessor::shouldRecordVideoForDetectedObjects() const
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

void CameraPostProcessor::setVideoRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveVideo == enabled) {
        return;
    }

    m_settings.m_saveVideo = enabled;

    if (!enabled && m_videoWriter.isOpened()) {
        m_videoWriter.release();
    }

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportSaveVideoState::create(enabled));
    }
}

void CameraPostProcessor::executeCommand(const QString& command, const QString& className)
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

    qDebug() << "CameraPostProcessor::executeCommand: Executing:" << allArgs;
    const QString program = allArgs.takeFirst();
    QProcess::startDetached(program, allArgs);
#else
    qWarning() << "CameraPostProcessor::executeCommand: QProcess not supported. Can't run:" << command;
    (void) className;
#endif
}

void CameraPostProcessor::saySpeech(const QString& speech, const QString& className)
{
    if (speech.isEmpty()) {
        return;
    }

    const QString expandedSpeech = substituteObjectClass(speech, className);

#ifdef QT_TEXTTOSPEECH_FOUND
    m_speech->say(expandedSpeech);
#else
    qWarning() << "CameraPostProcessor::saySpeech: TextToSpeech not supported. Unable to say" << expandedSpeech;
#endif
}

void CameraPostProcessor::applyObjectDetectedSettings(const QString& className)
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
            qWarning() << "CameraPostProcessor::applyObjectDetectedSettings: device set at"
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
                qDebug() << "CameraPostProcessor::applyObjectDetectedSettings: loading preset"
                         << preset->getDescription() << "for class" << className
                         << "to device set" << devSettings->m_deviceSetIndex;
                mainCore->getMainMessageQueue()->push(
                    MainCore::MsgLoadPreset::create(preset, devSettings->m_deviceSetIndex));
            }
            else
            {
                qWarning() << "CameraPostProcessor::applyObjectDetectedSettings: unable to get preset"
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

void CameraPostProcessor::applyObjectDisappearedSettings(const QString& className)
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
