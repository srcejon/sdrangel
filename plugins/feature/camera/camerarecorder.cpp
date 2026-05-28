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
#include <cmath>

#include <QDebug>
#include <QFileInfo>
#include <QVariantMap>

#include <opencv2/imgproc.hpp>

#include "cameraimageutils.h"
#include "camerapostprocessor.h"
#include "camerarecorder.h"
#include "util/fits.h"

MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgConfigureCameraRecorder, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgSetVideoRecordingEnabled, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgCaptureActive, Message)

namespace {

constexpr qint64 kPreRecordBufferMaxBytes = 2LL * 1024LL * 1024LL * 1024LL;

qint64 imageSizeBytes(const QImage& image)
{
    return image.isNull() ? 0 : static_cast<qint64>(image.sizeInBytes());
}

qint64 bufferedFrameSizeBytes(const QImage& raw, const QImage& processed)
{
    return imageSizeBytes(raw) + imageSizeBytes(processed);
}

QString bayerPatternName(CameraPipelineFrame::BayerPattern pattern)
{
    switch (pattern)
    {
    case CameraPipelineFrame::BayerRGGB:
        return QStringLiteral("RGGB");
    case CameraPipelineFrame::BayerBGGR:
        return QStringLiteral("BGGR");
    case CameraPipelineFrame::BayerGRBG:
        return QStringLiteral("GRBG");
    case CameraPipelineFrame::BayerGBRG:
        return QStringLiteral("GBRG");
    case CameraPipelineFrame::BayerNone:
    default:
        return QString();
    }
}

bool imageToMonoBytes(const QImage& image, QByteArray& bytes, int& bitsPerPixel)
{
    if (image.isNull()) {
        return false;
    }

    QImage mono = image;
    if ((mono.format() != QImage::Format_Grayscale8) && (mono.format() != QImage::Format_Grayscale16))
    {
        if (mono.depth() > 8) {
            return false;
        }
        mono = mono.convertToFormat(QImage::Format_Grayscale8);
    }

    bitsPerPixel = (mono.format() == QImage::Format_Grayscale16) ? 16 : 8;
    const int bytesPerPixel = bitsPerPixel / 8;
    bytes.clear();
    bytes.reserve(mono.width() * mono.height() * bytesPerPixel);
    const int rowBytes = mono.width() * bytesPerPixel;

    for (int y = 0; y < mono.height(); ++y) {
        bytes.append(reinterpret_cast<const char*>(mono.constScanLine(y)), rowBytes);
    }

    return bytes.size() == (mono.width() * mono.height() * bytesPerPixel);
}

} // namespace

CameraRecorder::CameraRecorder() :
    m_msgQueueToGUI(nullptr),
    m_nextStage(nullptr),
    m_captureActive(false),
    m_preRecordBufferFlushed(false),
    m_recordedImageFrames(0),
    m_processingFrames(false),
    m_droppedOutputFrames(0)
{
}

CameraRecorder::~CameraRecorder()
{
    closeVideoWriters();
}

void CameraRecorder::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraRecorder::handleInputMessages);
    handleInputMessages();
}

void CameraRecorder::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraRecorder::handleInputMessages);
}

bool CameraRecorder::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraRecorder::match(cmd))
    {
        const MsgConfigureCameraRecorder& cfg = (const MsgConfigureCameraRecorder&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        const MsgProcessFrame& frameMsg = (const MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgSetVideoRecordingEnabled::match(cmd))
    {
        const MsgSetVideoRecordingEnabled& videoMsg = (const MsgSetVideoRecordingEnabled&) cmd;
        setVideoRecordingEnabled(videoMsg.getEnabled());
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        const MsgCaptureActive& activeMsg = (const MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();
        if (m_captureActive) {
            resetRecordingLimits();
        } else {
            closeVideoWriters();
        }

        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.clear();
        if (!m_captureActive) {
            m_processingFrames = false;
        }
        return true;
    }

    return false;
}

void CameraRecorder::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraRecorder::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraRecorder::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    const bool wasSavingVideo = m_settings.m_saveVideo;
    const QString previousVideoFileName = m_settings.m_videoFileName;
    const bool previousRecordCalibratedMedia = m_settings.m_recordCalibratedMedia;
    const bool previousRecordPostProcessedMedia = m_settings.m_recordPostProcessedMedia;

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (force
        || settingsKeys.contains("videoFileName")
        || settingsKeys.contains("videoPostProcess")
        || settingsKeys.contains("recordCalibratedMedia")
        || settingsKeys.contains("recordPostProcessedMedia")
        || settingsKeys.contains("videoHwAcceleration"))
    {
        if ((previousVideoFileName != m_settings.m_videoFileName)
            || (previousRecordCalibratedMedia != m_settings.m_recordCalibratedMedia)
            || (previousRecordPostProcessedMedia != m_settings.m_recordPostProcessedMedia)
            || force)
        {
            closeVideoWriters();
            m_preRecordBufferFlushed = false;
        }
    }

    if (force || settingsKeys.contains("saveVideo"))
    {
        if (!m_settings.m_saveVideo) {
            closeVideoWriters();
        } else if (!wasSavingVideo) {
            m_videoRecordingStartDateTime = QDateTime::currentDateTimeUtc();
            m_preRecordBufferFlushed = false;
        }
    }

    if (force || settingsKeys.contains("saveImage"))
    {
        if (m_settings.m_saveImage) {
            m_recordedImageFrames = 0;
        }
    }
}

void CameraRecorder::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.push_back(frame);
        const int frameLimit = outputQueueFrameLimit();
        while ((frameLimit > 0) && (static_cast<int>(m_pendingFrames.size()) > frameLimit))
        {
            m_pendingFrames.pop_front();
            ++m_droppedOutputFrames;
            qDebug() << "CameraRecorder: Dropping queued frame in favor of newer frame. dropped:" << m_droppedOutputFrames;
        }

        if (!m_processingFrames)
        {
            m_processingFrames = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraRecorder::processNextFrames, Qt::QueuedConnection);
    }
}

void CameraRecorder::processNextFrames()
{
    for (;;)
    {
        CameraPipelineFramePtr frame;
        {
            QMutexLocker locker(&m_frameMutex);
            if (m_pendingFrames.empty())
            {
                m_processingFrames = false;
                return;
            }

            frame = m_pendingFrames.front();
            m_pendingFrames.pop_front();
        }

        processNewFrame(frame);
    }
}

void CameraRecorder::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData())
    {
        forwardFrame(frame);
        return;
    }

    if (!frame->ensureCpuImageFromCuda())
    {
        forwardFrame(frame);
        return;
    }

    const QImage& rawImage = frame->m_unprocessedImage.isNull() ? frame->m_image : frame->m_unprocessedImage;
    const QImage& processedImage = frame->m_postProcessedImage.isNull() ? frame->m_image : frame->m_postProcessedImage;
    bool savedImageFrame = false;
    bool savedVideoFrame = false;

    if (m_captureActive && (m_settings.m_saveImage || frame->m_saveCurrentImage) && !m_settings.m_imageFileName.isEmpty())
    {
        if (shouldSaveRawFits())
        {
            const QImage& rawBayerImage = frame->m_rawBayerImage.isNull() ? frame->m_image : frame->m_rawBayerImage;
            const QString rawFitsFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, QStringLiteral("raw"), QStringLiteral("fits"));
            if (saveRawFits(rawFitsFilename, rawBayerImage, frame->m_rawBayerPattern, *frame)) {
                savedImageFrame = m_settings.m_saveImage;
            }
        }

        if (shouldSaveCalibratedMedia())
        {
            const QString calibratedFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, QStringLiteral("calibrated"));
            qDebug() << "CameraRecorder: Saving calibrated image to" << calibratedFilename;
            rawImage.save(calibratedFilename);
            savedImageFrame = m_settings.m_saveImage;
        }

        if (shouldSavePostProcessedMedia())
        {
            const QString processedFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, QStringLiteral("post"));
            qDebug() << "CameraRecorder: Saving post-processed image to" << processedFilename;
            processedImage.save(processedFilename);
            savedImageFrame = m_settings.m_saveImage;
        }
    }

    if (m_captureActive && m_settings.m_saveVideo && !m_settings.m_videoFileName.isEmpty())
    {
        if (!m_preRecordBufferFlushed) {
            flushPreRecordFrames(rawImage, processedImage);
        }

        if (shouldSaveCalibratedMedia() && ensureVideoWriter(m_rawVideoWriter, m_settings.m_videoFileName, rawImage, QStringLiteral("calibrated"))) {
            writeVideoFrame(m_rawVideoWriter, rawImage);
            savedVideoFrame = true;
        }

        if (shouldSavePostProcessedMedia() && ensureVideoWriter(m_processedVideoWriter, m_settings.m_videoFileName, processedImage, QStringLiteral("post"))) {
            writeVideoFrame(m_processedVideoWriter, processedImage);
            savedVideoFrame = true;
        }
    }
    else if (m_captureActive && !m_settings.m_saveVideo)
    {
        appendPreRecordFrame(rawImage, processedImage);
    }

    updateRecordingLimitsAfterFrame(savedImageFrame, savedVideoFrame);
    forwardFrame(frame);
}

void CameraRecorder::forwardFrame(const CameraPipelineFramePtr& frame)
{
    if (m_nextStage && frame) {
        m_nextStage->submitFrame(frame);
    }
}

void CameraRecorder::setVideoRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveVideo == enabled) {
        return;
    }

    m_settings.m_saveVideo = enabled;
    m_preRecordBufferFlushed = false;

    if (enabled) {
        m_videoRecordingStartDateTime = QDateTime::currentDateTimeUtc();
    } else {
        closeVideoWriters();
        m_videoRecordingStartDateTime = QDateTime();
    }

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(CameraPostProcessor::MsgReportSaveVideoState::create(enabled));
    }
}

void CameraRecorder::setImageRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveImage == enabled) {
        return;
    }

    m_settings.m_saveImage = enabled;

    if (enabled) {
        m_recordedImageFrames = 0;
    }

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(CameraPostProcessor::MsgReportSaveImageState::create(enabled));
    }
}

void CameraRecorder::resetRecordingLimits()
{
    m_recordedImageFrames = 0;
    m_videoRecordingStartDateTime = m_settings.m_saveVideo ? QDateTime::currentDateTimeUtc() : QDateTime();
}

void CameraRecorder::updateRecordingLimitsAfterFrame(bool savedImageFrame, bool savedVideoFrame)
{
    if (savedImageFrame && (m_settings.m_imageRecordLimit > 0))
    {
        ++m_recordedImageFrames;

        if (m_recordedImageFrames >= m_settings.m_imageRecordLimit) {
            setImageRecordingEnabled(false);
        }
    }

    if (savedVideoFrame && (m_settings.m_videoRecordLimitSeconds > 0))
    {
        if (!m_videoRecordingStartDateTime.isValid()) {
            m_videoRecordingStartDateTime = QDateTime::currentDateTimeUtc();
        }

        if (m_videoRecordingStartDateTime.msecsTo(QDateTime::currentDateTimeUtc()) >= (m_settings.m_videoRecordLimitSeconds * 1000LL)) {
            setVideoRecordingEnabled(false);
        }
    }
}

QString CameraRecorder::createTimestampedOutputFilename(const QString& baseFileName, const QString& variant, const QString& suffixOverride)
{
    const QFileInfo fileInfo(baseFileName);
    const QString timestamp = QDateTime::currentDateTimeUtc().toString("yyyy-MM-ddTHH_mm_ss_zzz");
    const QString infix = variant.isEmpty() ? QStringLiteral(".") : QStringLiteral(".%1.").arg(variant);
    const QString suffix = suffixOverride.isEmpty() ? fileInfo.suffix() : suffixOverride;
    return fileInfo.path() + "/" + fileInfo.baseName() + infix + timestamp + "." + suffix;
}

bool CameraRecorder::shouldSaveRawFits() const
{
    return m_settings.m_recordRawFits;
}

bool CameraRecorder::shouldSaveCalibratedMedia() const
{
    return m_settings.m_recordCalibratedMedia;
}

bool CameraRecorder::shouldSavePostProcessedMedia() const
{
    return m_settings.m_recordPostProcessedMedia;
}

bool CameraRecorder::saveRawFits(const QString& fileName,
                                 const QImage& image,
                                 CameraPipelineFrame::BayerPattern bayerPattern,
                                 const CameraPipelineFrame& frame) const
{
    QByteArray imageBytes;
    int bitsPerPixel = 0;
    if (!imageToMonoBytes(image, imageBytes, bitsPerPixel))
    {
        qWarning() << "CameraRecorder: cannot save raw FITS from image format" << image.format();
        return false;
    }

    QVariantMap headers;
    const QString bayer = bayerPatternName(bayerPattern);
    if (!bayer.isEmpty()) {
        headers.insert(QStringLiteral("BAYERPAT"), bayer);
    }
    if (frame.m_captureDateTime.isValid()) {
        headers.insert(QStringLiteral("DATE-OBS"), frame.m_captureDateTime.toUTC());
    }
    headers.insert(QStringLiteral("EXPTIME"), m_settings.m_exposureTimeMs / 1000.0);
    headers.insert(QStringLiteral("GAIN"), m_settings.m_cameraGain);
    headers.insert(QStringLiteral("OFFSET"), m_settings.m_cameraOffset);
    headers.insert(QStringLiteral("XBINNING"), m_settings.m_cameraBinX);
    headers.insert(QStringLiteral("YBINNING"), m_settings.m_cameraBinY);
    headers.insert(QStringLiteral("SITELAT"), m_settings.m_latitude);
    headers.insert(QStringLiteral("SITELONG"), m_settings.m_longitude);
    headers.insert(QStringLiteral("SITEELEV"), m_settings.m_altitude);
    headers.insert(QStringLiteral("AZSTART"), m_settings.m_azimuth);
    headers.insert(QStringLiteral("ELSTART"), m_settings.m_elevation);
    if (!m_settings.m_cameraDescription.isEmpty()) {
        headers.insert(QStringLiteral("INSTRUME"), m_settings.m_cameraDescription);
    }

    QString errorMessage;
    if (!FITS::saveImage(fileName, imageBytes, image.width(), image.height(), bitsPerPixel, headers, &errorMessage))
    {
        qWarning() << "CameraRecorder: failed to save raw FITS" << fileName << errorMessage;
        return false;
    }

    qDebug() << "CameraRecorder: Saved raw FITS to" << fileName;
    return true;
}

void CameraRecorder::closeVideoWriters()
{
    if (m_rawVideoWriter.isOpened()) {
        m_rawVideoWriter.release();
    }
    if (m_processedVideoWriter.isOpened()) {
        m_processedVideoWriter.release();
    }
    m_rawVideoWriterSize = QSize();
    m_processedVideoWriterSize = QSize();
}

int CameraRecorder::preRecordBufferFrameLimit() const
{
    const int seconds = qBound(0, m_settings.m_videoPreRecordBufferSeconds, 60);
    if (seconds <= 0) {
        return 0;
    }

    return std::max(1, static_cast<int>(std::ceil(m_settings.getCaptureFrameRate() * seconds)));
}

int CameraRecorder::outputQueueFrameLimit() const
{
    const int seconds = std::max(2, qBound(0, m_settings.m_videoPreRecordBufferSeconds, 60));
    return std::max(5, static_cast<int>(std::ceil(m_settings.getCaptureFrameRate() * seconds)));
}

void CameraRecorder::trimPreRecordBuffer()
{
    const int frameLimit = preRecordBufferFrameLimit();
    if (frameLimit == 0)
    {
        m_preRecordVideoFrames.clear();
        return;
    }

    while (static_cast<int>(m_preRecordVideoFrames.size()) > frameLimit) {
        m_preRecordVideoFrames.pop_front();
    }

    qint64 totalBytes = 0;
    for (const BufferedVideoFrame& f : m_preRecordVideoFrames) {
        totalBytes += bufferedFrameSizeBytes(f.m_rawImage, f.m_processedImage);
    }
    while (!m_preRecordVideoFrames.empty() && (totalBytes > kPreRecordBufferMaxBytes))
    {
        const BufferedVideoFrame& front = m_preRecordVideoFrames.front();
        totalBytes -= bufferedFrameSizeBytes(front.m_rawImage, front.m_processedImage);
        m_preRecordVideoFrames.pop_front();
    }
}

void CameraRecorder::appendPreRecordFrame(const QImage& rawImage, const QImage& processedImage)
{
    if (preRecordBufferFrameLimit() <= 0)
    {
        m_preRecordVideoFrames.clear();
        return;
    }

    BufferedVideoFrame entry;
    if (shouldSaveCalibratedMedia() && !rawImage.isNull()) {
        entry.m_rawImage = rawImage.copy();
    }
    if (shouldSavePostProcessedMedia() && !processedImage.isNull()) {
        entry.m_processedImage = processedImage.copy();
    }
    if (entry.m_rawImage.isNull() && entry.m_processedImage.isNull()) {
        return;
    }

    m_preRecordVideoFrames.push_back(std::move(entry));
    trimPreRecordBuffer();
}

void CameraRecorder::flushPreRecordFrames(const QImage& currentRawImage, const QImage& currentProcessedImage)
{
    if (!m_settings.m_saveVideo || m_settings.m_videoFileName.isEmpty())
    {
        m_preRecordBufferFlushed = true;
        return;
    }

    if (shouldSaveCalibratedMedia() && ensureVideoWriter(m_rawVideoWriter, m_settings.m_videoFileName, currentRawImage, QStringLiteral("calibrated")))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_rawImage.size() == currentRawImage.size()) {
                writeVideoFrame(m_rawVideoWriter, bufferedFrame.m_rawImage);
            }
        }
    }

    if (shouldSavePostProcessedMedia() && ensureVideoWriter(m_processedVideoWriter, m_settings.m_videoFileName, currentProcessedImage, QStringLiteral("post")))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_processedImage.size() == currentProcessedImage.size()) {
                writeVideoFrame(m_processedVideoWriter, bufferedFrame.m_processedImage);
            }
        }
    }

    m_preRecordVideoFrames.clear();
    m_preRecordBufferFlushed = true;
}

bool CameraRecorder::ensureVideoWriter(cv::VideoWriter& writer, const QString& baseFileName, const QImage& frameForSize, const QString& variant)
{
    QSize& openedSize = (variant == QLatin1String("calibrated")) ? m_rawVideoWriterSize : m_processedVideoWriterSize;
    const QSize requestedSize = frameForSize.size();

    if (writer.isOpened() && (openedSize == requestedSize)) {
        return true;
    }
    if (writer.isOpened() && (openedSize != requestedSize))
    {
        writer.release();
        openedSize = QSize();
    }

    const QString filename = createTimestampedOutputFilename(baseFileName, variant);
    const int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    const std::vector<int> params = {
        cv::VIDEOWRITER_PROP_HW_ACCELERATION,
        m_settings.m_videoHwAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    };

    writer.open(
        filename.toStdString(),
        fourcc,
        m_settings.getCaptureFrameRate(),
        cv::Size(requestedSize.width(), requestedSize.height()),
        params);

    if (writer.isOpened()) {
        openedSize = requestedSize;
        qDebug() << "CameraRecorder opened:" << filename << "backend:" << QString::fromStdString(writer.getBackendName());
    } else {
        qWarning() << "CameraRecorder failed to open:" << filename;
    }

    return writer.isOpened();
}

void CameraRecorder::writeVideoFrame(cv::VideoWriter& writer, const QImage& frameToWrite)
{
    QImage convertedRgb;
    const QImage& rgb = CameraImageUtils::ensureRgb888(frameToWrite, convertedRgb);
    cv::Mat mat = CameraImageUtils::wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
    writer.write(bgrMat);
}
