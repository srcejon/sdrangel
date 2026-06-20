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

#include "camera.h"
#include "camerapostprocessor.h"
#include "camerarecorder.h"
#include "cameravideowriter.h"
#include "camerayoutubestreamer.h"
#include "util/fits.h"

MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgSetVideoRecordingEnabled, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgReportSaveVideoState, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgReportSaveImageState, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgReportKeogram, Message)
MESSAGE_CLASS_DEFINITION(CameraRecorder::MsgAudioSamples, Message)

namespace {

constexpr qint64 kPreRecordBufferMaxBytes = 2LL * 1024LL * 1024LL * 1024LL;
constexpr qint64 kAudioBufferMaxBytes = 10LL * 48000LL * 4LL;
constexpr qint64 kKeogramWindowSeconds = 24LL * 60LL * 60LL;

qint64 imageSizeBytes(const QImage& image)
{
    return image.isNull() ? 0 : static_cast<qint64>(image.sizeInBytes());
}

qint64 bufferedFrameSizeBytes(const QImage& calibrated, const QImage& filtered, const QImage& processed)
{
    return imageSizeBytes(calibrated) + imageSizeBytes(filtered) + imageSizeBytes(processed);
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

    if ((image.format() == QImage::Format_RGBX64)
        || (image.format() == QImage::Format_RGBA64)
        || (image.format() == QImage::Format_RGBA64_Premultiplied))
    {
        const QImage rgb64 = image.convertToFormat(QImage::Format_RGBA64);
        bitsPerPixel = 16;
        channels = 3;
        const int bytesPerPixel = bitsPerPixel / 8;
        qsizetype planeBytes = 0;
        qsizetype imageBytes = 0;
        if (!checkedFitsByteCount(rgb64.width(), rgb64.height(), bytesPerPixel, planeBytes)
            || !checkedFitsByteCount(rgb64.width(), rgb64.height(), bytesPerPixel * channels, imageBytes))
        {
            qWarning() << "CameraRecorder: RGB16 FITS image is too large to save"
                       << rgb64.width() << "x" << rgb64.height()
                       << "channels" << channels;
            return false;
        }

        bytes.clear();
        bytes.resize(imageBytes);
        if (bytes.size() != imageBytes) {
            return false;
        }

        uchar *dst = reinterpret_cast<uchar*>(bytes.data());
        for (int y = 0; y < rgb64.height(); ++y)
        {
            const QRgba64 *row = reinterpret_cast<const QRgba64*>(rgb64.constScanLine(y));
            for (int x = 0; x < rgb64.width(); ++x)
            {
                const qsizetype pixelOffset = (static_cast<qsizetype>(y) * rgb64.width() + x) * bytesPerPixel;
                const quint16 values[3] = {
                    row[x].red(),
                    row[x].green(),
                    row[x].blue()
                };
                for (int channel = 0; channel < channels; ++channel)
                {
                    uchar *pixel = dst + (static_cast<qsizetype>(channel) * planeBytes) + pixelOffset;
                    pixel[0] = static_cast<uchar>(values[channel] & 0xff);
                    pixel[1] = static_cast<uchar>((values[channel] >> 8) & 0xff);
                }
            }
        }

        return true;
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
    m_keogramLastSampleIndex(-1),
    m_pendingAudioBytes(0),
    m_youtubeStreamer(new CameraYouTubeStreamer()),
    m_youtubeStreamErrorReported(false),
    m_processingFrames(false),
    m_droppedOutputFrames(0)
{
}

CameraRecorder::~CameraRecorder()
{
    closeYouTubeStream();
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
    else if (MsgAudioSamples::match(cmd))
    {
        const MsgAudioSamples& audioMsg = (const MsgAudioSamples&) cmd;
        appendAudioSamples(audioMsg.getPcmS16Stereo(), audioMsg.getSampleRate());
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
            closeYouTubeStream();
            closeVideoWriters();
            m_pendingAudioChunks.clear();
            m_pendingAudioBytes = 0;
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
    const CameraSettings::VideoCodec previousVideoCodec = m_settings.m_videoCodec;
    const int previousVideoRecordBitrateKbps = m_settings.m_videoRecordBitrateKbps;
    const bool previousRecordCalibratedMedia = m_settings.m_recordCalibratedMedia;
    const bool previousRecordFilteredMedia = m_settings.m_recordFilteredMedia;
    const bool previousRecordPostProcessedMedia = m_settings.m_recordPostProcessedMedia;
    const bool previousVideoHwAcceleration = m_settings.m_videoHwAcceleration;
    const bool previousKeogramEnabled = m_settings.m_keogramEnabled;
    const QString previousKeogramFileName = m_settings.m_keogramFileName;
    const CameraSettings::KeogramDirection previousKeogramDirection = m_settings.m_keogramDirection;
    const CameraSettings::KeogramDayMode previousKeogramDayMode = m_settings.m_keogramDayMode;
    const int previousKeogramSamplePeriodMinutes = m_settings.m_keogramSamplePeriodMinutes;
    const bool previousKeogramShowPreview = m_settings.m_keogramShowPreview;
    const bool previousYouTubeStreamEnabled = m_settings.m_youtubeStreamEnabled;
    const QString previousYouTubeStreamUrl = m_settings.m_youtubeStreamUrl;
    const QString previousYouTubeStreamKey = m_settings.m_youtubeStreamKey;
    const bool previousYouTubeStreamPostProcessed = m_settings.m_youtubeStreamPostProcessed;
    const int previousYouTubeStreamBitrateKbps = m_settings.m_youtubeStreamBitrateKbps;
    const int previousYouTubeStreamFps = m_settings.m_youtubeStreamFps;
    const int previousYouTubeStreamWidth = m_settings.m_youtubeStreamWidth;
    const int previousYouTubeStreamHeight = m_settings.m_youtubeStreamHeight;

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (force
        || settingsKeys.contains("videoFileName")
        || settingsKeys.contains("videoCodec")
        || settingsKeys.contains("videoRecordBitrateKbps")
        || settingsKeys.contains("recordCalibratedMedia")
        || settingsKeys.contains("recordFilteredMedia")
        || settingsKeys.contains("recordPostProcessedMedia")
        || settingsKeys.contains("videoHwAcceleration"))
    {
        if ((previousVideoFileName != m_settings.m_videoFileName)
            || (previousVideoCodec != m_settings.m_videoCodec)
            || (previousVideoRecordBitrateKbps != m_settings.m_videoRecordBitrateKbps)
            || (previousRecordCalibratedMedia != m_settings.m_recordCalibratedMedia)
            || (previousRecordFilteredMedia != m_settings.m_recordFilteredMedia)
            || (previousRecordPostProcessedMedia != m_settings.m_recordPostProcessedMedia)
            || (previousVideoHwAcceleration != m_settings.m_videoHwAcceleration)
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

    if (force
        || settingsKeys.contains("youtubeStreamEnabled")
        || settingsKeys.contains("youtubeStreamUrl")
        || settingsKeys.contains("youtubeStreamKey")
        || settingsKeys.contains("youtubeStreamPostProcessed")
        || settingsKeys.contains("youtubeStreamBitrateKbps")
        || settingsKeys.contains("youtubeStreamFps")
        || settingsKeys.contains("youtubeStreamWidth")
        || settingsKeys.contains("youtubeStreamHeight"))
    {
        if (force
            || (previousYouTubeStreamEnabled != m_settings.m_youtubeStreamEnabled)
            || (previousYouTubeStreamUrl != m_settings.m_youtubeStreamUrl)
            || (previousYouTubeStreamKey != m_settings.m_youtubeStreamKey)
            || (previousYouTubeStreamPostProcessed != m_settings.m_youtubeStreamPostProcessed)
            || (previousYouTubeStreamBitrateKbps != m_settings.m_youtubeStreamBitrateKbps)
            || (previousYouTubeStreamFps != m_settings.m_youtubeStreamFps)
            || (previousYouTubeStreamWidth != m_settings.m_youtubeStreamWidth)
            || (previousYouTubeStreamHeight != m_settings.m_youtubeStreamHeight))
        {
            closeYouTubeStream();
            m_youtubeStreamErrorReported = false;
        }
    }

    if (force || settingsKeys.contains("saveImage"))
    {
        if (m_settings.m_saveImage) {
            m_recordedImageFrames = 0;
        }
    }

    if (force
        || settingsKeys.contains("keogramEnabled")
        || settingsKeys.contains("keogramFileName")
        || settingsKeys.contains("keogramDirection")
        || settingsKeys.contains("keogramDayMode")
        || settingsKeys.contains("keogramSamplePeriodMinutes"))
    {
        if (force
            || (previousKeogramEnabled != m_settings.m_keogramEnabled)
            || (previousKeogramFileName != m_settings.m_keogramFileName)
            || (previousKeogramDirection != m_settings.m_keogramDirection)
            || (previousKeogramDayMode != m_settings.m_keogramDayMode)
            || (previousKeogramSamplePeriodMinutes != m_settings.m_keogramSamplePeriodMinutes))
        {
            resetKeogram();
        }
    }

    if ((force || settingsKeys.contains("keogramShowPreview") || settingsKeys.contains("keogramEnabled"))
        && (previousKeogramShowPreview || previousKeogramEnabled)
        && (!m_settings.m_keogramShowPreview || !m_settings.m_keogramEnabled)
        && m_msgQueueToGUI)
    {
        m_msgQueueToGUI->push(MsgReportKeogram::create(QImage(), QString(), false));
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
    const QImage& filteredImage = frame->m_image;
    const QImage& processedImage = frame->m_postProcessedImage.isNull() ? frame->m_image : frame->m_postProcessedImage;
    bool savedImageFrame = false;
    bool savedVideoFrame = false;

    if (m_captureActive && m_settings.m_keogramEnabled) {
        updateKeogram(calibratedImage, frame->m_captureDateTime);
    }
    if (m_captureActive && m_settings.m_youtubeStreamEnabled) {
        updateYouTubeStream(calibratedImage, processedImage);
    } else {
        closeYouTubeStream();
    }

    if (((m_captureActive && m_settings.m_saveImage) || frame->m_saveCurrentImage) && !m_settings.m_imageFileName.isEmpty())
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

        if (shouldSaveFilteredMedia())
        {
            const QString filteredFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, QStringLiteral("filtered"));
            qDebug() << "CameraRecorder: Saving filtered image to" << filteredFilename;
            if (filteredImage.save(filteredFilename)) {
                savedImageFrame = m_settings.m_saveImage;
            } else {
                qWarning() << "CameraRecorder: Failed to save filtered image to" << filteredFilename;
                reportErrorToFeature(QStringLiteral("image-save:filtered"),
                                     tr("Camera image recording error"),
                                     tr("Failed to save filtered image:\n%1").arg(filteredFilename));
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
        const double videoFrameRate = frame->m_playbackFrameRate > 0.0
            ? frame->m_playbackFrameRate
            : m_settings.getCaptureFrameRate();

        if (!m_preRecordBufferFlushed) {
            flushPreRecordFrames(calibratedImage, filteredImage, processedImage, videoFrameRate);
        }

        if (shouldSaveCalibratedMedia() && ensureVideoWriter(m_calibratedVideoWriter, m_settings.m_videoFileName, calibratedImage, QStringLiteral("calibrated"), videoFrameRate)) {
            writePendingAudio(*m_calibratedVideoWriter, QStringLiteral("calibrated"));
            savedVideoFrame = writeVideoFrame(*m_calibratedVideoWriter, calibratedImage, QStringLiteral("calibrated"), frame->m_playbackPositionMs) || savedVideoFrame;
        }

        if (shouldSaveFilteredMedia() && ensureVideoWriter(m_filteredVideoWriter, m_settings.m_videoFileName, filteredImage, QStringLiteral("filtered"), videoFrameRate)) {
            writePendingAudio(*m_filteredVideoWriter, QStringLiteral("filtered"));
            savedVideoFrame = writeVideoFrame(*m_filteredVideoWriter, filteredImage, QStringLiteral("filtered"), frame->m_playbackPositionMs) || savedVideoFrame;
        }

        if (shouldSavePostProcessedMedia() && ensureVideoWriter(m_processedVideoWriter, m_settings.m_videoFileName, processedImage, QStringLiteral("post"), videoFrameRate)) {
            writePendingAudio(*m_processedVideoWriter, QStringLiteral("post"));
            savedVideoFrame = writeVideoFrame(*m_processedVideoWriter, processedImage, QStringLiteral("post"), frame->m_playbackPositionMs) || savedVideoFrame;
        }
        m_pendingAudioChunks.clear();
        m_pendingAudioBytes = 0;
    }
    else if (m_captureActive && !m_settings.m_saveVideo)
    {
        appendPreRecordFrame(calibratedImage, filteredImage, processedImage);
        if (m_settings.m_youtubeStreamEnabled)
        {
            m_pendingAudioChunks.clear();
            m_pendingAudioBytes = 0;
        }
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

void CameraRecorder::appendAudioSamples(const QByteArray& pcmS16Stereo, int sampleRate)
{
    if (pcmS16Stereo.isEmpty() || (sampleRate <= 0)) {
        return;
    }
    if (!m_captureActive || (!m_settings.m_saveVideo && !m_settings.m_youtubeStreamEnabled))
    {
        m_pendingAudioChunks.clear();
        m_pendingAudioBytes = 0;
        return;
    }

    AudioChunk chunk;
    chunk.m_pcmS16Stereo = pcmS16Stereo;
    chunk.m_sampleRate = sampleRate;
    m_pendingAudioBytes += chunk.m_pcmS16Stereo.size();
    m_pendingAudioChunks.push_back(std::move(chunk));
    trimPendingAudio();
}

bool CameraRecorder::writePendingAudio(CameraVideoWriter& writer, const QString& variant)
{
    bool ok = true;

    for (const AudioChunk& chunk : m_pendingAudioChunks)
    {
        QString error;
        if (!writer.writePcmS16Stereo(chunk.m_pcmS16Stereo, chunk.m_sampleRate, error))
        {
            ok = false;
            qWarning() << "CameraRecorder failed to write audio samples:" << error;
            reportErrorToFeature(QStringLiteral("video-writer-audio:%1").arg(variant),
                                 tr("Camera video recording audio warning"),
                                 tr("Failed to write %1 audio samples. Video recording will continue:\n%2").arg(variant, error));
            break;
        }
    }

    return ok;
}

void CameraRecorder::trimPendingAudio()
{
    while (!m_pendingAudioChunks.empty() && (m_pendingAudioBytes > kAudioBufferMaxBytes))
    {
        m_pendingAudioBytes -= m_pendingAudioChunks.front().m_pcmS16Stereo.size();
        m_pendingAudioChunks.pop_front();
    }
}

void CameraRecorder::resetKeogram()
{
    m_keogramImage = QImage();
    m_keogramWindowStartUtc = QDateTime();
    m_keogramSourceSize = QSize();
    m_keogramOutputFileName.clear();
    m_keogramLastSampleIndex = -1;
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

bool CameraRecorder::shouldSaveFilteredMedia() const
{
    return m_settings.m_recordFilteredMedia;
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
    const double exposureTimeMs = (std::isfinite(frame.m_exposureTimeMs) && (frame.m_exposureTimeMs > 0.0))
        ? frame.m_exposureTimeMs
        : m_settings.m_exposureTimeMs;
    headers.insert(QStringLiteral("EXPTIME"), exposureTimeMs / 1000.0);
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
    m_calibratedVideoWriter.reset();
    m_filteredVideoWriter.reset();
    m_processedVideoWriter.reset();
    m_calibratedVideoWriterSize = QSize();
    m_filteredVideoWriterSize = QSize();
    m_processedVideoWriterSize = QSize();
    m_reportedVideoWriterErrorKeys.clear();
}

void CameraRecorder::closeYouTubeStream()
{
    if (m_youtubeStreamer) {
        m_youtubeStreamer->close();
    }
}

void CameraRecorder::updateYouTubeStream(const QImage& calibratedImage, const QImage& processedImage)
{
    if (!m_youtubeStreamer || m_youtubeStreamErrorReported) {
        return;
    }

    const QImage& streamImage = m_settings.m_youtubeStreamPostProcessed && !processedImage.isNull()
        ? processedImage
        : calibratedImage;
    if (streamImage.isNull()) {
        return;
    }

    if (!m_youtubeStreamer->isOpen())
    {
        CameraYouTubeStreamer::Settings streamSettings;
        streamSettings.m_url = m_settings.m_youtubeStreamUrl;
        streamSettings.m_key = m_settings.m_youtubeStreamKey;
        streamSettings.m_bitrateKbps = m_settings.m_youtubeStreamBitrateKbps;
        streamSettings.m_fps = m_settings.m_youtubeStreamFps;
        if ((m_settings.m_youtubeStreamWidth > 0) && (m_settings.m_youtubeStreamHeight > 0)) {
            streamSettings.m_size = QSize(m_settings.m_youtubeStreamWidth, m_settings.m_youtubeStreamHeight);
        }

        QString errorMessage;
        if (!m_youtubeStreamer->open(streamSettings, streamImage, errorMessage))
        {
            qWarning() << "CameraRecorder: YouTube stream open failed:" << errorMessage;
            reportErrorToFeature(QStringLiteral("youtube-stream-open"),
                                 tr("Camera YouTube stream error"),
                                 errorMessage);
            m_youtubeStreamErrorReported = true;
            closeYouTubeStream();
            return;
        }
    }

    for (const AudioChunk& chunk : m_pendingAudioChunks)
    {
        QString audioError;
        if (!m_youtubeStreamer->writePcmS16Stereo(chunk.m_pcmS16Stereo, chunk.m_sampleRate, audioError))
        {
            qWarning() << "CameraRecorder: YouTube audio write failed:" << audioError;
            reportErrorToFeature(QStringLiteral("youtube-stream-audio"),
                                 tr("Camera YouTube stream audio warning"),
                                 tr("Failed to write YouTube audio samples. Video streaming will continue:\n%1").arg(audioError));
            break;
        }
    }

    QString errorMessage;
    if (!m_youtubeStreamer->writeFrame(streamImage, errorMessage))
    {
        qWarning() << "CameraRecorder: YouTube stream write failed:" << errorMessage;
        reportErrorToFeature(QStringLiteral("youtube-stream-write"),
                             tr("Camera YouTube stream error"),
                             errorMessage);
        m_youtubeStreamErrorReported = true;
        closeYouTubeStream();
    }
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
        totalBytes += bufferedFrameSizeBytes(f.m_calibratedImage, f.m_filteredImage, f.m_processedImage);
    }
    while (!m_preRecordVideoFrames.empty() && (totalBytes > kPreRecordBufferMaxBytes))
    {
        const BufferedVideoFrame& front = m_preRecordVideoFrames.front();
        totalBytes -= bufferedFrameSizeBytes(front.m_calibratedImage, front.m_filteredImage, front.m_processedImage);
        m_preRecordVideoFrames.pop_front();
    }
}

void CameraRecorder::appendPreRecordFrame(const QImage& calibratedImage, const QImage& filteredImage, const QImage& processedImage)
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
    if (shouldSaveFilteredMedia() && !filteredImage.isNull()) {
        entry.m_filteredImage = filteredImage.copy();
    }
    if (shouldSavePostProcessedMedia() && !processedImage.isNull()) {
        entry.m_processedImage = processedImage.copy();
    }
    if (entry.m_calibratedImage.isNull() && entry.m_filteredImage.isNull() && entry.m_processedImage.isNull()) {
        return;
    }

    m_preRecordVideoFrames.push_back(std::move(entry));
    trimPreRecordBuffer();
}

void CameraRecorder::flushPreRecordFrames(const QImage& currentCalibratedImage, const QImage& currentFilteredImage, const QImage& currentProcessedImage, double frameRate)
{
    if (!m_settings.m_saveVideo || m_settings.m_videoFileName.isEmpty())
    {
        m_preRecordBufferFlushed = true;
        return;
    }

    if (m_preRecordVideoFrames.empty())
    {
        m_preRecordBufferFlushed = true;
        return;
    }

    const double videoFrameRate = std::max(0.001, frameRate);

    if (shouldSaveCalibratedMedia() && ensureVideoWriter(m_calibratedVideoWriter, m_settings.m_videoFileName, currentCalibratedImage, QStringLiteral("calibrated"), videoFrameRate))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_calibratedImage.size() == currentCalibratedImage.size()) {
                writeVideoFrame(*m_calibratedVideoWriter, bufferedFrame.m_calibratedImage, QStringLiteral("calibrated"));
            }
        }
    }

    if (shouldSaveFilteredMedia() && ensureVideoWriter(m_filteredVideoWriter, m_settings.m_videoFileName, currentFilteredImage, QStringLiteral("filtered"), videoFrameRate))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_filteredImage.size() == currentFilteredImage.size()) {
                writeVideoFrame(*m_filteredVideoWriter, bufferedFrame.m_filteredImage, QStringLiteral("filtered"));
            }
        }
    }

    if (shouldSavePostProcessedMedia() && ensureVideoWriter(m_processedVideoWriter, m_settings.m_videoFileName, currentProcessedImage, QStringLiteral("post"), videoFrameRate))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_processedImage.size() == currentProcessedImage.size()) {
                writeVideoFrame(*m_processedVideoWriter, bufferedFrame.m_processedImage, QStringLiteral("post"));
            }
        }
    }

    m_preRecordVideoFrames.clear();
    m_preRecordBufferFlushed = true;
}

QDateTime CameraRecorder::keogramWindowStartUtc(const QDateTime& captureDateTime) const
{
    const QDateTime localCapture = captureDateTime.isValid()
        ? captureDateTime.toLocalTime()
        : QDateTime::currentDateTime();
    const QTime boundaryTime = (m_settings.m_keogramDayMode == CameraSettings::KeogramMiddayToMidday)
        ? QTime(12, 0, 0)
        : QTime(0, 0, 0);

    QDate boundaryDate = localCapture.date();
    if (localCapture.time() < boundaryTime) {
        boundaryDate = boundaryDate.addDays(-1);
    }

    return QDateTime(boundaryDate, boundaryTime, Qt::LocalTime).toUTC();
}

int CameraRecorder::keogramSampleCount() const
{
    return std::max(1, static_cast<int>(std::ceil(1440.0 / std::max(1, m_settings.m_keogramSamplePeriodMinutes))));
}

int CameraRecorder::keogramSampleIndex(const QDateTime& captureDateTime) const
{
    if (!m_keogramWindowStartUtc.isValid()) {
        return -1;
    }

    const QDateTime captureUtc = captureDateTime.isValid()
        ? captureDateTime.toUTC()
        : QDateTime::currentDateTimeUtc();
    const qint64 elapsedMs = m_keogramWindowStartUtc.msecsTo(captureUtc);
    if ((elapsedMs < 0) || (elapsedMs >= (kKeogramWindowSeconds * 1000LL))) {
        return -1;
    }

    const qint64 sampleMs = static_cast<qint64>(std::max(1, m_settings.m_keogramSamplePeriodMinutes)) * 60LL * 1000LL;
    return qBound(0, static_cast<int>(elapsedMs / sampleMs), keogramSampleCount() - 1);
}

QString CameraRecorder::keogramOutputFileName(const QDateTime& windowStartUtc) const
{
    const QFileInfo fileInfo(m_settings.m_keogramFileName);
    const QString suffix = fileInfo.suffix().compare(QStringLiteral("jpg"), Qt::CaseInsensitive) == 0
        || fileInfo.suffix().compare(QStringLiteral("jpeg"), Qt::CaseInsensitive) == 0
        || fileInfo.suffix().compare(QStringLiteral("png"), Qt::CaseInsensitive) == 0
            ? fileInfo.suffix()
            : QStringLiteral("png");
    const QString baseName = fileInfo.completeBaseName().isEmpty() ? QStringLiteral("keogram") : fileInfo.completeBaseName();
    const QString timestamp = windowStartUtc.toLocalTime().toString(QStringLiteral("yyyy-MM-dd_HH-mm"));
    return fileInfo.path() + QStringLiteral("/") + baseName + QStringLiteral(".") + timestamp + QStringLiteral(".") + suffix;
}

QImage CameraRecorder::rgbImageForKeogram(const QImage& image)
{
    if (image.isNull()) {
        return QImage();
    }
    return image.convertToFormat(QImage::Format_RGB888);
}

void CameraRecorder::updateKeogram(const QImage& calibratedImage, const QDateTime& captureDateTime)
{
    if (calibratedImage.isNull() || m_settings.m_keogramFileName.isEmpty()) {
        return;
    }

    const QImage rgb = rgbImageForKeogram(calibratedImage);
    if (rgb.isNull()) {
        return;
    }

    const QDateTime windowStartUtc = keogramWindowStartUtc(captureDateTime);
    const bool vertical = m_settings.m_keogramDirection == CameraSettings::KeogramVertical;
    const QSize outputSize = vertical
        ? QSize(keogramSampleCount(), rgb.height())
        : QSize(rgb.width(), keogramSampleCount());

    if ((m_keogramWindowStartUtc != windowStartUtc)
        || (m_keogramSourceSize != rgb.size())
        || (m_keogramImage.size() != outputSize)
        || (m_keogramImage.format() != QImage::Format_RGB888))
    {
        const QString outputFileName = keogramOutputFileName(windowStartUtc);
        QImage loadedKeogram;
        if (QFileInfo::exists(outputFileName))
        {
            QImage existingKeogram(outputFileName);
            if (!existingKeogram.isNull() && (existingKeogram.size() == outputSize))
            {
                loadedKeogram = existingKeogram.convertToFormat(QImage::Format_RGB888);
                qDebug() << "CameraRecorder: Loaded existing keogram" << outputFileName << outputSize;
            }
            else
            {
                qDebug() << "CameraRecorder: Existing keogram is incompatible, starting new"
                         << outputFileName
                         << "existing" << existingKeogram.size()
                         << "expected" << outputSize;
            }
        }

        m_keogramWindowStartUtc = windowStartUtc;
        m_keogramSourceSize = rgb.size();
        if (loadedKeogram.isNull())
        {
            m_keogramImage = QImage(outputSize, QImage::Format_RGB888);
            m_keogramImage.fill(Qt::black);
        }
        else
        {
            m_keogramImage = loadedKeogram;
        }
        m_keogramOutputFileName = outputFileName;
        m_keogramLastSampleIndex = -1;
    }

    const int sampleIndex = keogramSampleIndex(captureDateTime);
    if (sampleIndex < 0) {
        return;
    }
    if (sampleIndex == m_keogramLastSampleIndex) {
        return;
    }

    const int stripWidth = 3;
    if (vertical)
    {
        const int x0 = qBound(0, (rgb.width() - stripWidth) / 2, rgb.width() - 1);
        const int x1 = qMin(rgb.width() - 1, x0 + stripWidth - 1);
        for (int y = 0; y < rgb.height(); ++y)
        {
            int r = 0;
            int g = 0;
            int b = 0;
            int count = 0;
            const uchar *src = rgb.constScanLine(y);
            for (int x = x0; x <= x1; ++x)
            {
                r += src[x * 3];
                g += src[x * 3 + 1];
                b += src[x * 3 + 2];
                ++count;
            }
            uchar *dst = m_keogramImage.scanLine(y) + sampleIndex * 3;
            dst[0] = static_cast<uchar>(r / count);
            dst[1] = static_cast<uchar>(g / count);
            dst[2] = static_cast<uchar>(b / count);
        }
    }
    else
    {
        const int y0 = qBound(0, (rgb.height() - stripWidth) / 2, rgb.height() - 1);
        const int y1 = qMin(rgb.height() - 1, y0 + stripWidth - 1);
        uchar *dst = m_keogramImage.scanLine(sampleIndex);
        for (int x = 0; x < rgb.width(); ++x)
        {
            int r = 0;
            int g = 0;
            int b = 0;
            int count = 0;
            for (int y = y0; y <= y1; ++y)
            {
                const uchar *src = rgb.constScanLine(y) + x * 3;
                r += src[0];
                g += src[1];
                b += src[2];
                ++count;
            }
            dst[x * 3] = static_cast<uchar>(r / count);
            dst[x * 3 + 1] = static_cast<uchar>(g / count);
            dst[x * 3 + 2] = static_cast<uchar>(b / count);
        }
    }

    if (!m_keogramOutputFileName.isEmpty() && !m_keogramImage.save(m_keogramOutputFileName))
    {
        qWarning() << "CameraRecorder: Failed to save keogram to" << m_keogramOutputFileName;
        reportErrorToFeature(QStringLiteral("keogram-save"),
                             tr("Camera keogram error"),
                             tr("Failed to save keogram:\n%1").arg(m_keogramOutputFileName));
    }

    if (m_settings.m_keogramShowPreview && m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportKeogram::create(m_keogramImage, m_keogramOutputFileName, true));
    }
    m_keogramLastSampleIndex = sampleIndex;
}

bool CameraRecorder::ensureVideoWriter(std::unique_ptr<CameraVideoWriter>& writer, const QString& baseFileName, const QImage& frameForSize, const QString& variant, double frameRate)
{
    QSize *openedSize = &m_processedVideoWriterSize;
    if (variant == QLatin1String("calibrated")) {
        openedSize = &m_calibratedVideoWriterSize;
    } else if (variant == QLatin1String("filtered")) {
        openedSize = &m_filteredVideoWriterSize;
    }
    const QSize requestedSize = frameForSize.size();
    const double requestedFrameRate = std::max(0.001, frameRate);

    if (frameForSize.isNull()) {
        return false;
    }

    if (writer
        && writer->isOpen()
        && (*openedSize == requestedSize)
        && (writer->codec() == m_settings.m_videoCodec)
        && (writer->bitrateKbps() == m_settings.m_videoRecordBitrateKbps)
        && (std::abs(writer->fps() - requestedFrameRate) <= 0.001)) {
        return true;
    }
    if (writer)
    {
        writer.reset();
        *openedSize = QSize();
    }

    const QString filename = createTimestampedOutputFilename(baseFileName, variant);
    CameraVideoWriter::Settings writerSettings;
    writerSettings.m_fileName = filename;
    writerSettings.m_codec = m_settings.m_videoCodec;
    writerSettings.m_bitrateKbps = m_settings.m_videoRecordBitrateKbps;
    writerSettings.m_fps = requestedFrameRate;
    writerSettings.m_preferHardwareEncoding = m_settings.m_videoHwAcceleration;

    writer.reset(new CameraVideoWriter());
    QString error;
    if (writer->open(writerSettings, frameForSize, error)) {
        *openedSize = requestedSize;
        m_reportedVideoWriterErrorKeys.remove(QStringLiteral("video-writer:%1:%2:%3x%4")
            .arg(variant, filename)
            .arg(requestedSize.width())
            .arg(requestedSize.height()));
        qDebug() << "CameraRecorder opened:" << filename << "codec:" << writer->codecName();
    } else {
        const QString errorMessage = tr("Failed to open video file for %1 recording:\n%2\n\nCodec: %3\nFrame size: %4x%5\nFrame rate: %6 fps\n\n%7")
            .arg(variant,
                 filename,
                 CameraVideoWriter::codecName(m_settings.m_videoCodec))
            .arg(requestedSize.width())
            .arg(requestedSize.height())
            .arg(QString::number(requestedFrameRate, 'f', 3),
                 error);
        qWarning() << "CameraRecorder failed to open:" << filename << error;
        reportErrorToFeature(QStringLiteral("video-writer:%1:%2:%3x%4")
                                 .arg(variant, filename)
                                 .arg(requestedSize.width())
                                 .arg(requestedSize.height()),
                             tr("Camera video recording error"),
                             errorMessage);
        writer.reset();
    }

    return writer && writer->isOpen();
}

void CameraRecorder::reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage)
{
    if (!m_msgQueueToFeature || m_reportedVideoWriterErrorKeys.contains(errorKey)) {
        return;
    }

    m_reportedVideoWriterErrorKeys.insert(errorKey);
    m_msgQueueToFeature->push(Camera::MsgReportError::create(title, errorMessage));
}

bool CameraRecorder::writeVideoFrame(CameraVideoWriter& writer, const QImage& frameToWrite, const QString& variant, qint64 timestampMs)
{
    QString error;
    if (writer.writeFrame(frameToWrite, error, timestampMs)) {
        return true;
    }

    qWarning() << "CameraRecorder failed to write video frame:" << error;
    reportErrorToFeature(QStringLiteral("video-writer-write:%1").arg(variant),
                         tr("Camera video recording error"),
                         tr("Failed to write %1 video frame:\n%2").arg(variant, error));
    return false;
}
