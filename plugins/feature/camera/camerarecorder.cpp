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
#include <limits>

#include <QDebug>
#include <QFileInfo>
#include <QVariantMap>

#include <opencv2/imgproc.hpp>

#include "cameraimageutils.h"
#include "camera.h"
#include "camerapostprocessor.h"
#include "camerarecorder.h"
#include "util/fits.h"

MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgSetVideoRecordingEnabled, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgReportSaveVideoState, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgReportSaveImageState, Message)

namespace {

constexpr qint64 kPreRecordBufferMaxBytes = 2LL * 1024LL * 1024LL * 1024LL;

qint64 imageSizeBytes(const QImage& image)
{
    return image.isNull() ? 0 : static_cast<qint64>(image.sizeInBytes());
}

qint64 bufferedFrameSizeBytes(const QImage& calibrated, const QImage& processed)
{
    return imageSizeBytes(calibrated) + imageSizeBytes(processed);
}

bool checkedFitsByteCount(int width, int height, int bytesPerPixel, qsizetype& byteCount)
{
    if ((width <= 0) || (height <= 0) || (bytesPerPixel <= 0)) {
        return false;
    }

    const qint64 maxByteCount = static_cast<qint64>(std::numeric_limits<qsizetype>::max());
    const qint64 width64 = static_cast<qint64>(width);
    const qint64 height64 = static_cast<qint64>(height);
    const qint64 bytesPerPixel64 = static_cast<qint64>(bytesPerPixel);

    if ((width64 > maxByteCount / height64)
        || ((width64 * height64) > maxByteCount / bytesPerPixel64))
    {
        return false;
    }

    byteCount = static_cast<qsizetype>(width64 * height64 * bytesPerPixel64);
    return true;
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

bool imageToFitsBytes(const QImage& image, QByteArray& bytes, int& bitsPerPixel, int& channels)
{
    if (image.isNull()) {
        return false;
    }

    if ((image.format() == QImage::Format_Grayscale8) || (image.format() == QImage::Format_Grayscale16))
    {
        bitsPerPixel = (image.format() == QImage::Format_Grayscale16) ? 16 : 8;
        channels = 1;
        const int bytesPerPixel = bitsPerPixel / 8;
        qsizetype imageBytes = 0;
        qsizetype rowBytes = 0;
        if (!checkedFitsByteCount(image.width(), image.height(), bytesPerPixel, imageBytes)
            || !checkedFitsByteCount(image.width(), 1, bytesPerPixel, rowBytes))
        {
            qWarning() << "CameraRecorder: FITS image is too large to save"
                       << image.width() << "x" << image.height()
                       << "bytesPerPixel" << bytesPerPixel;
            return false;
        }

        bytes.clear();
        bytes.reserve(imageBytes);

        for (int y = 0; y < image.height(); ++y) {
            bytes.append(reinterpret_cast<const char*>(image.constScanLine(y)), rowBytes);
        }

        return bytes.size() == imageBytes;
    }

    const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    bitsPerPixel = 8;
    channels = 3;
    qsizetype planeBytes = 0;
    qsizetype imageBytes = 0;
    if (!checkedFitsByteCount(rgb.width(), rgb.height(), 1, planeBytes)
        || !checkedFitsByteCount(rgb.width(), rgb.height(), channels, imageBytes))
    {
        qWarning() << "CameraRecorder: RGB FITS image is too large to save"
                   << rgb.width() << "x" << rgb.height()
                   << "channels" << channels;
        return false;
    }

    bytes.clear();
    bytes.resize(imageBytes);
    if (bytes.size() != imageBytes) {
        return false;
    }
    uchar *dst = reinterpret_cast<uchar*>(bytes.data());

    for (int y = 0; y < rgb.height(); ++y)
    {
        const uchar *row = rgb.constScanLine(y);
        for (int x = 0; x < rgb.width(); ++x)
        {
            const qsizetype pixelOffset = static_cast<qsizetype>(y) * rgb.width() + x;
            dst[pixelOffset] = row[x * 3];
            dst[planeBytes + pixelOffset] = row[x * 3 + 1];
            dst[(2 * planeBytes) + pixelOffset] = row[x * 3 + 2];
        }
    }

    return true;
}

} // namespace

CameraRecorder::CameraRecorder() :
    m_msgQueueToGUI(nullptr),
    m_msgQueueToFeature(nullptr),
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
    if (Camera::MsgConfigureCamera::match(cmd))
    {
        const Camera::MsgConfigureCamera& cfg = (const Camera::MsgConfigureCamera&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (Camera::MsgProcessFrame::match(cmd))
    {
        const Camera::MsgProcessFrame& frameMsg = (const Camera::MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgSetVideoRecordingEnabled::match(cmd))
    {
        const MsgSetVideoRecordingEnabled& videoMsg = (const MsgSetVideoRecordingEnabled&) cmd;
        setVideoRecordingEnabled(videoMsg.getEnabled());
        return true;
    }
    else if (Camera::MsgCaptureActive::match(cmd))
    {
        const Camera::MsgCaptureActive& activeMsg = (const Camera::MsgCaptureActive&) cmd;
        Camera::discardQueuedProcessFrames(m_inputMessageQueue);
        m_captureActive = activeMsg.isActive();
        m_captureEpoch = activeMsg.getCaptureEpoch();
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

    Camera::discardQueuedProcessFramesOnCaptureActive(m_inputMessageQueue);

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
            m_reportedVideoWriterErrorKeys.clear();
        }
    }

    if (force || settingsKeys.contains("saveVideo"))
    {
        if (!m_settings.m_saveVideo) {
            closeVideoWriters();
        } else if (!wasSavingVideo) {
            m_videoRecordingStartDateTime = QDateTime::currentDateTimeUtc();
            m_preRecordBufferFlushed = false;
            m_reportedVideoWriterErrorKeys.clear();
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
    if (!Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
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

        if (Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
            processNewFrame(frame);
        }
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

    const QImage& calibratedImage = frame->m_unprocessedImage.isNull() ? frame->m_image : frame->m_unprocessedImage;
    const QImage& processedImage = frame->m_postProcessedImage.isNull() ? frame->m_image : frame->m_postProcessedImage;
    bool savedImageFrame = false;
    bool savedVideoFrame = false;

    if (m_captureActive && (m_settings.m_saveImage || frame->m_saveCurrentImage) && !m_settings.m_imageFileName.isEmpty())
    {
        if (shouldSaveRawFits())
        {
            const bool haveCapturedRawFrame = !frame->m_rawInputImage.isNull();
            const QImage& rawFitsImage = haveCapturedRawFrame ? frame->m_rawInputImage : calibratedImage;
            if (rawFitsImage.isNull())
            {
                qDebug() << "CameraRecorder: raw FITS requested but no image frame is available; skipping";
            }
            else
            {
                const QString rawFitsFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, QStringLiteral("raw"), QStringLiteral("fits"));
                const CameraPipelineFrame::BayerPattern bayerPattern = haveCapturedRawFrame ? frame->m_rawInputBayerPattern : CameraPipelineFrame::BayerNone;
                if (saveRawFits(rawFitsFilename, rawFitsImage, bayerPattern, *frame)) {
                    savedImageFrame = m_settings.m_saveImage;
                }
            }
        }

        if (shouldSaveCalibratedMedia())
        {
            const QString calibratedFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, QStringLiteral("calibrated"));
            qDebug() << "CameraRecorder: Saving calibrated image to" << calibratedFilename;
            if (calibratedImage.save(calibratedFilename)) {
                savedImageFrame = m_settings.m_saveImage;
            } else {
                qWarning() << "CameraRecorder: Failed to save calibrated image to" << calibratedFilename;
                reportErrorToFeature(QStringLiteral("image-save:calibrated"),
                                     tr("Camera image recording error"),
                                     tr("Failed to save calibrated image:\n%1").arg(calibratedFilename));
            }
        }

        if (shouldSavePostProcessedMedia())
        {
            const QString processedFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, QStringLiteral("post"));
            qDebug() << "CameraRecorder: Saving post-processed image to" << processedFilename;
            if (processedImage.save(processedFilename)) {
                savedImageFrame = m_settings.m_saveImage;
            } else {
                qWarning() << "CameraRecorder: Failed to save post-processed image to" << processedFilename;
                reportErrorToFeature(QStringLiteral("image-save:post"),
                                     tr("Camera image recording error"),
                                     tr("Failed to save post-processed image:\n%1").arg(processedFilename));
            }
        }
    }

    if (m_captureActive && m_settings.m_saveVideo && !m_settings.m_videoFileName.isEmpty())
    {
        if (!m_preRecordBufferFlushed) {
            flushPreRecordFrames(calibratedImage, processedImage);
        }

        if (shouldSaveCalibratedMedia() && ensureVideoWriter(m_calibratedVideoWriter, m_settings.m_videoFileName, calibratedImage, QStringLiteral("calibrated"))) {
            writeVideoFrame(m_calibratedVideoWriter, calibratedImage);
            savedVideoFrame = true;
        }

        if (shouldSavePostProcessedMedia() && ensureVideoWriter(m_processedVideoWriter, m_settings.m_videoFileName, processedImage, QStringLiteral("post"))) {
            writeVideoFrame(m_processedVideoWriter, processedImage);
            savedVideoFrame = true;
        }
    }
    else if (m_captureActive && !m_settings.m_saveVideo)
    {
        appendPreRecordFrame(calibratedImage, processedImage);
    }

    updateRecordingLimitsAfterFrame(savedImageFrame, savedVideoFrame);
    forwardFrame(frame);
}

void CameraRecorder::forwardFrame(const CameraPipelineFramePtr& frame)
{
    if (m_nextStage && Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
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
        m_msgQueueToGUI->push(MsgReportSaveVideoState::create(enabled));
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
        m_msgQueueToGUI->push(MsgReportSaveImageState::create(enabled));
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
    int channels = 0;
    if (!imageToFitsBytes(image, imageBytes, bitsPerPixel, channels))
    {
        qWarning() << "CameraRecorder: cannot save raw FITS from image format" << image.format();
        return false;
    }

    QVariantMap headers;
    const QString bayer = bayerPatternName(bayerPattern);
    if (!bayer.isEmpty()) {
        headers.insert(QStringLiteral("BAYERPAT"), bayer);
    }
    if (channels == 3) {
        headers.insert(QStringLiteral("COLORSPC"), QStringLiteral("RGB"));
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
    if (!FITS::saveImage(fileName, imageBytes, image.width(), image.height(), bitsPerPixel, channels, headers, &errorMessage))
    {
        qWarning() << "CameraRecorder: failed to save raw FITS" << fileName << errorMessage;
        return false;
    }

    qDebug() << "CameraRecorder: Saved raw FITS to" << fileName;
    return true;
}

void CameraRecorder::closeVideoWriters()
{
    if (m_calibratedVideoWriter.isOpened()) {
        m_calibratedVideoWriter.release();
    }
    if (m_processedVideoWriter.isOpened()) {
        m_processedVideoWriter.release();
    }
    m_calibratedVideoWriterSize = QSize();
    m_processedVideoWriterSize = QSize();
    m_reportedVideoWriterErrorKeys.clear();
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
        totalBytes += bufferedFrameSizeBytes(f.m_calibratedImage, f.m_processedImage);
    }
    while (!m_preRecordVideoFrames.empty() && (totalBytes > kPreRecordBufferMaxBytes))
    {
        const BufferedVideoFrame& front = m_preRecordVideoFrames.front();
        totalBytes -= bufferedFrameSizeBytes(front.m_calibratedImage, front.m_processedImage);
        m_preRecordVideoFrames.pop_front();
    }
}

void CameraRecorder::appendPreRecordFrame(const QImage& calibratedImage, const QImage& processedImage)
{
    if (preRecordBufferFrameLimit() <= 0)
    {
        m_preRecordVideoFrames.clear();
        return;
    }

    BufferedVideoFrame entry;
    if (shouldSaveCalibratedMedia() && !calibratedImage.isNull()) {
        entry.m_calibratedImage = calibratedImage.copy();
    }
    if (shouldSavePostProcessedMedia() && !processedImage.isNull()) {
        entry.m_processedImage = processedImage.copy();
    }
    if (entry.m_calibratedImage.isNull() && entry.m_processedImage.isNull()) {
        return;
    }

    m_preRecordVideoFrames.push_back(std::move(entry));
    trimPreRecordBuffer();
}

void CameraRecorder::flushPreRecordFrames(const QImage& currentCalibratedImage, const QImage& currentProcessedImage)
{
    if (!m_settings.m_saveVideo || m_settings.m_videoFileName.isEmpty())
    {
        m_preRecordBufferFlushed = true;
        return;
    }

    if (shouldSaveCalibratedMedia() && ensureVideoWriter(m_calibratedVideoWriter, m_settings.m_videoFileName, currentCalibratedImage, QStringLiteral("calibrated")))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_calibratedImage.size() == currentCalibratedImage.size()) {
                writeVideoFrame(m_calibratedVideoWriter, bufferedFrame.m_calibratedImage);
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
    QSize& openedSize = (variant == QLatin1String("calibrated")) ? m_calibratedVideoWriterSize : m_processedVideoWriterSize;
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
        m_reportedVideoWriterErrorKeys.remove(QStringLiteral("video-writer:%1:%2:%3x%4")
            .arg(variant, filename)
            .arg(requestedSize.width())
            .arg(requestedSize.height()));
        qDebug() << "CameraRecorder opened:" << filename << "backend:" << QString::fromStdString(writer.getBackendName());
    } else {
        const QString errorMessage = tr("Failed to open video file for %1 recording:\n%2\n\nCodec: avc1\nFrame size: %3x%4\nFrame rate: %5 fps")
            .arg(variant,
                 filename)
            .arg(requestedSize.width())
            .arg(requestedSize.height())
            .arg(QString::number(m_settings.getCaptureFrameRate(), 'f', 3));
        qWarning() << "CameraRecorder failed to open:" << filename;
        reportErrorToFeature(QStringLiteral("video-writer:%1:%2:%3x%4")
                                 .arg(variant, filename)
                                 .arg(requestedSize.width())
                                 .arg(requestedSize.height()),
                             tr("Camera video recording error"),
                             errorMessage);
    }

    return writer.isOpened();
}

void CameraRecorder::reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage)
{
    if (!m_msgQueueToFeature || m_reportedVideoWriterErrorKeys.contains(errorKey)) {
        return;
    }

    m_reportedVideoWriterErrorKeys.insert(errorKey);
    m_msgQueueToFeature->push(Camera::MsgReportError::create(title, errorMessage));
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
