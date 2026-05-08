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
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QTextDocument>
#include "util/profiler.h"
#include "camerapostprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgConfigureCameraPostProcessor, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSpectrumFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportSaveVideoState, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSetVideoRecordingEnabled, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgCaptureActive, Message)

CameraPostProcessor::CameraPostProcessor() :
    m_msgQueueToGUI(nullptr),
    m_captureActive(false),
    m_processingFrame(false)
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
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgSpectrumFrame::match(cmd))
    {
        MsgSpectrumFrame& frameMsg = (MsgSpectrumFrame&) cmd;
        m_spectrumViewImage = frameMsg.getImage();
        return true;
    }
    else if (MsgSetVideoRecordingEnabled::match(cmd))
    {
        MsgSetVideoRecordingEnabled& enabledMsg = (MsgSetVideoRecordingEnabled&) cmd;
        setVideoRecordingEnabled(enabledMsg.getEnabled());
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

        QMutexLocker locker(&m_frameMutex);
        m_pendingFrame.reset();
        if (!m_captureActive) {
            m_processingFrame = false;
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
        reportFrameToGUI(processed, m_lastFrame.m_histogramData);
    }
}

void CameraPostProcessor::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            qDebug() << "CameraPostProcessor: Dropping pending frame in favor of new frame";
        }
        m_pendingFrame = frame;
        if (!m_processingFrame)
        {
            m_processingFrame = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraPostProcessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraPostProcessor::processNextFrame()
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
        QMetaObject::invokeMethod(this, &CameraPostProcessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraPostProcessor::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    m_captureDateTime = frame->m_captureDateTime.isValid() ? frame->m_captureDateTime : QDateTime::currentDateTime();
    const QImage& pipelineImage = frame->m_image;
    const QImage& unprocessedImage = frame->m_unprocessedImage.isNull() ? frame->m_image : frame->m_unprocessedImage;
    const QImage processed = applyPostProcessing(*frame);

    m_lastFrame = *frame;

    reportFrameToGUI(processed, frame->m_histogramData);

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
            QImage convertedRgb;
            const QImage& rgb = ensureRgb888(frameToWrite, convertedRgb);
            cv::Mat mat = wrapRgb888Image(rgb);
            cv::Mat bgrMat;
            cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
            m_videoWriter.write(bgrMat);
        }
    }
}

void CameraPostProcessor::reportFrameToGUI(const QImage& image, const CameraHistogramData& histogramData)
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportFrame::create(image, histogramData));
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

const QImage& CameraPostProcessor::ensureRgb888(const QImage& image, QImage& convertedImage)
{
    if (image.format() == QImage::Format_RGB888) {
        return image;
    }

    convertedImage = image.convertToFormat(QImage::Format_RGB888);
    return convertedImage;
}

cv::Mat CameraPostProcessor::wrapRgb888Image(const QImage& image)
{
    return cv::Mat(image.height(), image.width(), CV_8UC3,
                   const_cast<uchar*>(image.constBits()),
                   static_cast<size_t>(image.bytesPerLine()));
}

QImage CameraPostProcessor::convertBgrToRgbImage(const cv::Mat& bgrMat)
{
    PROFILER_START();
    QImage result(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
    cv::Mat rgbMat(result.height(), result.width(), CV_8UC3,
                   result.bits(),
                   static_cast<size_t>(result.bytesPerLine()));
    cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
    return result;
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

    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(input, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
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
