///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
///////////////////////////////////////////////////////////////////////////////////

// Cloud detector tests. All test images are synthesised in code (dark night skies with
// smooth cloud bumps and point stars; blue/grey day skies with white cloud patches), so
// the suite needs no image assets and expected coverage is known by construction.

#include <algorithm>
#include <cmath>
#include <iostream>

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QEventLoop>
#include <QGuiApplication>
#include <QImage>
#include <QStringList>
#include <QTimeZone>
#include <QTimer>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera.h"
#include "cameraclouddetector.h"
#include "cameramotiondetector.h"
#include "camerastardetector.h"
#include "cameraskyprojector.h"
#include "util/astronomy.h"
#include "util/messagequeue.h"

MESSAGE_CLASS_DEFINITION(Camera::MsgConfigureCamera, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgCaptureActive, Message)

// The test executable links cameradetector.cpp but not camera.cpp, so the
// pipeline-frame helpers used by the detector are duplicated here verbatim
// (same pattern as the message definitions above).
static int discardQueuedProcessFramesImpl(MessageQueue& queue, bool requireCaptureActive)
{
    QList<Message*> messages;
    Message *message = nullptr;
    bool hasCaptureActive = false;

    while ((message = queue.pop()) != nullptr)
    {
        hasCaptureActive = hasCaptureActive || Camera::MsgCaptureActive::match(*message);
        messages.append(message);
    }

    int dropped = 0;

    for (Message *queuedMessage : messages)
    {
        if ((!requireCaptureActive || hasCaptureActive) && Camera::MsgProcessFrame::match(*queuedMessage))
        {
            delete queuedMessage;
            ++dropped;
        }
        else
        {
            queue.push(queuedMessage, false);
        }
    }

    return dropped;
}

int Camera::discardQueuedProcessFrames(MessageQueue& queue)
{
    return discardQueuedProcessFramesImpl(queue, false);
}

int Camera::discardQueuedProcessFramesOnCaptureActive(MessageQueue& queue)
{
    return discardQueuedProcessFramesImpl(queue, true);
}

bool Camera::acceptsPipelineFrame(const CameraPipelineFramePtr& frame, bool captureActive, quint64 captureEpoch)
{
    return frame
        && (frame->m_manualPreviewFrame || (captureActive && (frame->m_captureEpoch == captureEpoch)));
}

#ifndef CAMERA_CLOUD_TEST_DATA_DIR
#define CAMERA_CLOUD_TEST_DATA_DIR "."
#endif

namespace
{

constexpr int imageWidth = 640;
constexpr int imageHeight = 480;

QImage bgrToImage(const cv::Mat& bgr)
{
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
}

void addNoise(cv::Mat& bgr, double sigma, quint32 seed)
{
    cv::RNG rng(seed);
    cv::Mat noise(bgr.size(), CV_16SC3);
    rng.fill(noise, cv::RNG::NORMAL, 0.0, sigma);
    cv::Mat wide;
    bgr.convertTo(wide, CV_16SC3);
    wide += noise;
    wide.convertTo(bgr, CV_8UC3);
}

// Additive brightness bump, as a cloud lit by light pollution would appear at night
void addGaussianBump(cv::Mat& bgr, const cv::Point& center, double sigma, double amplitude)
{
    for (int row = 0; row < bgr.rows; ++row)
    {
        cv::Vec3b *line = bgr.ptr<cv::Vec3b>(row);
        for (int col = 0; col < bgr.cols; ++col)
        {
            const double dx = col - center.x;
            const double dy = row - center.y;
            const double bump = amplitude * std::exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma));
            for (int channel = 0; channel < 3; ++channel) {
                line[col][channel] = cv::saturate_cast<uchar>(line[col][channel] + bump);
            }
        }
    }
}

// A patchy cloud made of several overlapping blobs. Real night cloud has structure at cloud
// scale; a single smooth Gaussian is unrealistic and, being a smooth surface, is absorbed by
// the dark-path polynomial background model. This clustered form both looks more like cloud
// and exercises the detector's structure-vs-smooth-gradient discrimination.
void addStructuredCloud(cv::Mat& bgr, const cv::Point& center)
{
    // Deterministic scatter of blob centres around the cloud centre, covering the region
    // that holds the clouded test stars
    const int offsets[][2] = {
        {-45, -45}, {-15, -55}, {20, -40}, {50, -30}, {-50, -10}, {-20, -10},
        {10, -5}, {40, 0}, {-40, 25}, {-10, 30}, {25, 25}, {50, 35},
        {-25, 50}, {5, 55}, {35, 45}, {0, 0}, {-30, -25}, {30, -20}
    };
    for (const auto& offset : offsets) {
        addGaussianBump(bgr, cv::Point(center.x + offset[0], center.y + offset[1]), 18.0, 55.0);
    }
}

void addStar(cv::Mat& bgr, const cv::Point& center)
{
    cv::circle(bgr, center, 1, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
}

const QVector<cv::Point> cloudedStars{{180, 190}, {220, 210}, {200, 160}};
const QVector<cv::Point> clearStars{{500, 100}, {520, 350}, {450, 240}, {560, 200}};
const cv::Point nightCloudCenter(200, 200);

QImage makeNightImage(bool withCloud, bool withStars)
{
    cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(15, 15, 15));
    if (withCloud) {
        addStructuredCloud(bgr, nightCloudCenter);
    }
    addNoise(bgr, 2.0, 12345);
    if (withStars)
    {
        for (const cv::Point& star : cloudedStars) {
            addStar(bgr, star);
        }
        for (const cv::Point& star : clearStars) {
            addStar(bgr, star);
        }
    }
    return bgrToImage(bgr);
}

QImage makeDayImage(double cloudFraction)
{
    // Clear blue sky; R/B ratio ~0.4, well below the day threshold
    cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
    if (cloudFraction >= 1.0)
    {
        bgr.setTo(cv::Scalar(180, 180, 180));
    }
    else if (cloudFraction > 0.0)
    {
        // White rectangle of the requested area fraction
        const int cloudHeight = static_cast<int>(std::lround(imageHeight * cloudFraction));
        cv::rectangle(bgr, cv::Rect(0, 0, imageWidth, cloudHeight), cv::Scalar(250, 250, 250), cv::FILLED);
    }
    addNoise(bgr, 2.0, 54321);
    return bgrToImage(bgr);
}

CameraSettings makeCloudSettings()
{
    CameraSettings settings;
    settings.resetToDefaults();
    settings.m_cloudDetect = true;
    settings.m_cloudMode = CameraSettings::CloudModeAuto;
    return settings;
}

struct CloudRunResult
{
    QVector<CameraPipelineFramePtr> frames;
    QString error;

    bool completed(int expectedFrames) const { return frames.size() == expectedFrames; }
};

// Runs frames through a detector chain one at a time (submitting the next frame only after
// the previous one is forwarded, so the bounded stage backlog never drops frames)
CloudRunResult runChain(
    CameraDetectionStage& firstStage,
    const QVector<CameraDetectionStage*>& stages,
    const CameraSettings& settings,
    const QVector<QImage>& images)
{
    CloudRunResult result;
    MessageQueue outputQueue;
    stages.last()->setNextStageInputMessageQueue(&outputQueue);
    QEventLoop loop;
    QTimer timeout;
    timeout.setSingleShot(true);

    int submitted = 0;
    auto submitNext = [&]() {
        if (submitted < images.size())
        {
            CameraPipelineFramePtr frame(new CameraPipelineFrame);
            frame->m_image = images[submitted];
            frame->m_unprocessedImage = images[submitted];
            firstStage.getInputMessageQueue()->push(Camera::MsgProcessFrame::create(frame));
            ++submitted;
        }
    };

    QObject::connect(&outputQueue, &MessageQueue::messageEnqueued, &loop, [&]() {
        Message *message = nullptr;
        while ((message = outputQueue.pop()) != nullptr)
        {
            if (Camera::MsgProcessFrame::match(*message))
            {
                const Camera::MsgProcessFrame& frameMessage =
                    static_cast<const Camera::MsgProcessFrame&>(*message);
                result.frames.append(frameMessage.getFrame());
            }
            delete message;
        }
        if (result.frames.size() >= images.size()) {
            loop.quit();
        } else {
            submitNext();
        }
    });
    QObject::connect(&timeout, &QTimer::timeout, &loop, [&]() {
        result.error = QStringLiteral("Timed out waiting for detector output (%1 of %2 frames)")
            .arg(result.frames.size()).arg(images.size());
        loop.quit();
    });

    for (CameraDetectionStage *stage : stages)
    {
        stage->startWork();
        stage->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, QList<QString>(), true));
        stage->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, 0));
    }

    submitNext();
    timeout.start(60000);
    loop.exec();

    for (CameraDetectionStage *stage : stages)
    {
        stage->stopWork();
        stage->getInputMessageQueue()->clear();
    }
    outputQueue.clear();
    return result;
}

// Frames carry an invalid capture time by default so auto-mode tests exercise the
// deterministic brightness fallback rather than the sun elevation at the real wall clock
CloudRunResult runCloudDetector(const CameraSettings& settings, const QVector<QImage>& images, const CameraPipelineImageTransform* imageTransform = nullptr, bool uploadToGpu = false, const QDateTime& captureDateTime = QDateTime())
{
    CameraCloudDetector detector;
    MessageQueue outputQueue;
    detector.setNextStageInputMessageQueue(&outputQueue);

    CloudRunResult result;
    QEventLoop loop;
    QTimer timeout;
    timeout.setSingleShot(true);

    int submitted = 0;
    auto submitNext = [&]() {
        if (submitted < images.size())
        {
            CameraPipelineFramePtr frame(new CameraPipelineFrame);
#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
            if (uploadToGpu)
            {
                // GPU-resident frame with no CPU image, as produced by CUDA upstream stages
                const QImage& rgbImage = images[submitted];
                cv::Mat rgbMat(rgbImage.height(), rgbImage.width(), CV_8UC3,
                    const_cast<uchar*>(rgbImage.constBits()),
                    static_cast<size_t>(rgbImage.bytesPerLine()));
                cv::Mat bgr;
                cv::cvtColor(rgbMat, bgr, cv::COLOR_RGB2BGR);
                frame->m_cudaBgrImage.upload(bgr);
            }
            else
#else
            (void) uploadToGpu;
#endif
            {
                frame->m_image = images[submitted];
                frame->m_unprocessedImage = images[submitted];
            }
            frame->m_captureDateTime = captureDateTime;
            if (imageTransform) {
                frame->m_imageTransform = *imageTransform;
            }
            detector.getInputMessageQueue()->push(Camera::MsgProcessFrame::create(frame));
            ++submitted;
        }
    };

    QObject::connect(&outputQueue, &MessageQueue::messageEnqueued, &loop, [&]() {
        Message *message = nullptr;
        while ((message = outputQueue.pop()) != nullptr)
        {
            if (Camera::MsgProcessFrame::match(*message))
            {
                const Camera::MsgProcessFrame& frameMessage =
                    static_cast<const Camera::MsgProcessFrame&>(*message);
                result.frames.append(frameMessage.getFrame());
            }
            delete message;
        }
        if (result.frames.size() >= images.size()) {
            loop.quit();
        } else {
            submitNext();
        }
    });
    QObject::connect(&timeout, &QTimer::timeout, &loop, [&]() {
        result.error = QStringLiteral("Timed out waiting for cloud detector output (%1 of %2 frames)")
            .arg(result.frames.size()).arg(images.size());
        loop.quit();
    });

    detector.startWork();
    detector.getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, QList<QString>(), true));
    detector.getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, 0));

    submitNext();
    timeout.start(60000);
    loop.exec();
    detector.stopWork();
    detector.getInputMessageQueue()->clear();
    outputQueue.clear();
    return result;
}

struct TestContext
{
    int failures = 0;

    void check(bool condition, const QString& testName, const QString& message)
    {
        if (condition)
        {
            std::cout << "PASS: " << testName.toStdString() << ": " << message.toStdString() << "\n";
        }
        else
        {
            std::cout << "FAIL: " << testName.toStdString() << ": " << message.toStdString() << "\n";
            ++failures;
        }
    }
};

bool coverageInRange(const CameraPipelineFramePtr& frame, float minPercent, float maxPercent)
{
    return frame
        && frame->m_cloud.m_valid
        && (frame->m_cloud.m_coveragePercent >= minPercent)
        && (frame->m_cloud.m_coveragePercent <= maxPercent);
}

QString coverageText(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return QStringLiteral("no frame");
    }
    if (!frame->m_cloud.m_valid) {
        return QStringLiteral("invalid cloud result");
    }
    return QStringLiteral("coverage %1 % (%2)")
        .arg(frame->m_cloud.m_coveragePercent, 0, 'f', 1)
        .arg(frame->m_cloud.m_night ? QStringLiteral("night") : QStringLiteral("day"));
}

void testNightClear(TestContext& context)
{
    const CloudRunResult result = runCloudDetector(makeCloudSettings(), {makeNightImage(false, true)});
    context.check(result.error.isEmpty() && result.completed(1), "night-clear", result.error.isEmpty() ? "ran" : result.error);
    if (!result.completed(1)) {
        return;
    }
    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "night-clear", "classified via night path");
    context.check(coverageInRange(frame, 0.0f, 2.0f), "night-clear", coverageText(frame) + " expected < 2 %");
}

void testNightCloudy(TestContext& context)
{
    const CloudRunResult result = runCloudDetector(makeCloudSettings(), {makeNightImage(true, true)});
    if (!result.completed(1))
    {
        context.check(false, "night-cloudy", result.error);
        return;
    }
    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "night-cloudy", "classified via night path");
    // The dark-path surface fit absorbs the smooth part of the cloud, so it reports the
    // structured (edge/peak) fraction rather than the full extent
    context.check(coverageInRange(frame, 4.0f, 45.0f), "night-cloudy", coverageText(frame) + " expected structured cloud detected");
    context.check(frame->m_cloud.isCloudAtImagePoint(QPointF(nightCloudCenter.x, nightCloudCenter.y)),
        "night-cloudy", "cloud centre classified as cloud");
    context.check(!frame->m_cloud.isCloudAtImagePoint(QPointF(clearStars[0].x, clearStars[0].y)),
        "night-cloudy", "clear-sky region classified as clear");
}

void testDayClear(TestContext& context)
{
    const CloudRunResult result = runCloudDetector(makeCloudSettings(), {makeDayImage(0.0)});
    if (!result.completed(1))
    {
        context.check(false, "day-clear", result.error);
        return;
    }
    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && !frame->m_cloud.m_night, "day-clear", "classified via day path");
    context.check(coverageInRange(frame, 0.0f, 2.0f), "day-clear", coverageText(frame) + " expected < 2 %");
}

void testDayOvercast(TestContext& context)
{
    const CloudRunResult result = runCloudDetector(makeCloudSettings(), {makeDayImage(1.0)});
    if (!result.completed(1))
    {
        context.check(false, "day-overcast", result.error);
        return;
    }
    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && !frame->m_cloud.m_night, "day-overcast", "classified via day path");
    context.check(coverageInRange(frame, 90.0f, 100.0f), "day-overcast", coverageText(frame) + " expected > 90 %");
}

void testDayPartial(TestContext& context)
{
    const CloudRunResult result = runCloudDetector(makeCloudSettings(), {makeDayImage(0.25)});
    if (!result.completed(1))
    {
        context.check(false, "day-partial", result.error);
        return;
    }
    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(coverageInRange(frame, 15.0f, 35.0f), "day-partial", coverageText(frame) + " expected ~25 %");
}

void testEdgeMarginMask(TestContext& context)
{
    // Blue clear sky with a bright white band along the top edge, standing in for the
    // vignetted rim / lens flare that fisheye all-sky frames show around the image circle.
    // With no edge margin the band classifies as cloud; an inward erosion margin should
    // exclude the whole edge band from both classification and the coverage denominator.
    const cv::Point bandProbe(imageWidth / 2, 10);
    auto makeRimImage = [&]() {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
        cv::rectangle(bgr, cv::Rect(0, 0, imageWidth, 20), cv::Scalar(250, 250, 250), cv::FILLED);
        addNoise(bgr, 2.0, 24680);
        return bgrToImage(bgr);
    };

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;

    settings.m_cloudEdgeMarginPercent = 0.0;
    const CloudRunResult noMargin = runCloudDetector(settings, {makeRimImage()});

    settings.m_cloudEdgeMarginPercent = 8.0; // ~38 px on a 480 px frame, well over the 20 px band
    const CloudRunResult withMargin = runCloudDetector(settings, {makeRimImage()});

    if (!noMargin.completed(1) || !withMargin.completed(1))
    {
        context.check(false, "edge-margin", noMargin.error + " " + withMargin.error);
        return;
    }

    const CameraPipelineFramePtr& baseFrame = noMargin.frames.first();
    const CameraPipelineFramePtr& marginFrame = withMargin.frames.first();
    context.check(baseFrame->m_cloud.m_valid && marginFrame->m_cloud.m_valid, "edge-margin", "both runs valid");
    context.check(baseFrame->m_cloud.isCloudAtImagePoint(QPointF(bandProbe.x, bandProbe.y)),
        "edge-margin", "edge band classified as cloud with no margin");
    context.check(!marginFrame->m_cloud.isCloudAtImagePoint(QPointF(bandProbe.x, bandProbe.y)),
        "edge-margin", "edge band excluded once the margin is applied");
    context.check(marginFrame->m_cloud.m_coveragePercent < baseFrame->m_cloud.m_coveragePercent,
        "edge-margin", QStringLiteral("coverage dropped from %1 to %2 %")
            .arg(baseFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(marginFrame->m_cloud.m_coveragePercent, 0, 'f', 1));
}

void testSunMoonMask(TestContext& context)
{
    // All-sky fisheye pose (pointing at the zenith, ~180 deg FoV). Project the sun for a
    // known location and time using the same lens model the detector uses, paint a bright
    // white blob there on an otherwise-clear blue sky, and confirm the sun/moon mask removes
    // it. The moon path is identical code with the moon's ephemeris, so this covers both.
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;
    settings.m_latitude = 51.5;
    settings.m_longitude = -0.12;
    settings.m_fov = 178.0f;
    settings.m_azimuth = 0.0f;
    settings.m_elevation = 90.0f;
    settings.m_roll = 0.0f;
    settings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
    settings.m_plateSolveUseCaptureDateTime = true;
    settings.m_cloudSunMoonRadiusDeg = 15.0;

    const QDateTime captureTime(QDate(2024, 6, 21), QTime(12, 0, 0), QTimeZone::utc());

    AzAlt sunAzAlt;
    RADec sunRaDec;
    Astronomy::sunPosition(sunAzAlt, sunRaDec, settings.m_latitude, settings.m_longitude, captureTime);

    const SkyProjector projector = SkyProjector::create(settings, QSize(imageWidth, imageHeight));
    QPointF sunPoint;
    const bool projected = projector.projectAltAz(sunAzAlt.az, sunAzAlt.alt, sunPoint);
    const bool inFrame = projected
        && (sunPoint.x() >= 0) && (sunPoint.x() < imageWidth)
        && (sunPoint.y() >= 0) && (sunPoint.y() < imageHeight);
    context.check(inFrame, "sun-mask", QStringLiteral("sun projects into the frame at (%1, %2)")
        .arg(sunPoint.x(), 0, 'f', 0).arg(sunPoint.y(), 0, 'f', 0));
    if (!inFrame) {
        return;
    }

    auto makeSunImage = [&]() {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
        cv::circle(bgr, cv::Point(static_cast<int>(sunPoint.x()), static_cast<int>(sunPoint.y())),
                   30, cv::Scalar(250, 250, 250), cv::FILLED);
        addNoise(bgr, 2.0, 13579);
        return bgrToImage(bgr);
    };

    settings.m_cloudMaskSunMoon = false;
    const CloudRunResult unmasked = runCloudDetector(settings, {makeSunImage()}, nullptr, false, captureTime);

    settings.m_cloudMaskSunMoon = true;
    const CloudRunResult masked = runCloudDetector(settings, {makeSunImage()}, nullptr, false, captureTime);

    if (!unmasked.completed(1) || !masked.completed(1))
    {
        context.check(false, "sun-mask", unmasked.error + " " + masked.error);
        return;
    }

    const CameraPipelineFramePtr& unmaskedFrame = unmasked.frames.first();
    const CameraPipelineFramePtr& maskedFrame = masked.frames.first();
    context.check(unmaskedFrame->m_cloud.m_valid && maskedFrame->m_cloud.m_valid, "sun-mask", "both runs valid");
    context.check(unmaskedFrame->m_cloud.isCloudAtImagePoint(sunPoint),
        "sun-mask", "sun blob classified as cloud without the mask");
    context.check(!maskedFrame->m_cloud.isCloudAtImagePoint(sunPoint),
        "sun-mask", "sun disc excluded with the mask enabled");
    context.check(maskedFrame->m_cloud.m_coveragePercent < unmaskedFrame->m_cloud.m_coveragePercent,
        "sun-mask", QStringLiteral("coverage dropped from %1 to %2 %")
            .arg(unmaskedFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(maskedFrame->m_cloud.m_coveragePercent, 0, 'f', 1));
}

void testExclusionRects(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    const CloudRunResult baseline = runCloudDetector(settings, {makeNightImage(true, false)});

    // Exclude a rectangle over the cloud centre; those pixels must not be classified and
    // coverage (percent of evaluated sky) must drop because the excluded area is mostly cloud
    settings.m_motionExclusionRects.append(QRect(100, 100, 200, 200));
    const CloudRunResult excluded = runCloudDetector(settings, {makeNightImage(true, false)});

    if (!baseline.completed(1) || !excluded.completed(1))
    {
        context.check(false, "exclusion-rects", baseline.error + " " + excluded.error);
        return;
    }

    const CameraPipelineFramePtr& baseFrame = baseline.frames.first();
    const CameraPipelineFramePtr& excludedFrame = excluded.frames.first();
    context.check(baseFrame->m_cloud.m_valid && excludedFrame->m_cloud.m_valid, "exclusion-rects", "both runs valid");
    context.check(!excludedFrame->m_cloud.isCloudAtImagePoint(QPointF(nightCloudCenter.x, nightCloudCenter.y)),
        "exclusion-rects", "excluded region not classified as cloud");
    context.check(excludedFrame->m_cloud.m_coveragePercent < baseFrame->m_cloud.m_coveragePercent,
        "exclusion-rects", QStringLiteral("coverage dropped from %1 to %2 %")
            .arg(baseFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(excludedFrame->m_cloud.m_coveragePercent, 0, 'f', 1));
}

void testUpdateIntervalCaching(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudUpdateIntervalFrames = 2;

    const QImage image = makeNightImage(true, false);
    const CloudRunResult result = runCloudDetector(settings, {image, image, image});
    if (!result.completed(3))
    {
        context.check(false, "update-interval", result.error);
        return;
    }

    const uchar *mask0 = result.frames[0]->m_cloud.m_mask.data;
    const uchar *mask1 = result.frames[1]->m_cloud.m_mask.data;
    const uchar *mask2 = result.frames[2]->m_cloud.m_mask.data;
    context.check(result.frames[1]->m_cloud.m_valid && (mask1 == mask0),
        "update-interval", "intermediate frame reuses the cached mask");
    context.check(result.frames[2]->m_cloud.m_valid && (mask2 != mask0),
        "update-interval", "mask recomputed after the update interval");
}

void testStarFiltering(TestContext& context)
{
    const QImage image = makeNightImage(true, true);

    auto runStarChain = [&](bool filterStars) -> CloudRunResult
    {
        CameraSettings settings = makeCloudSettings();
        settings.m_cloudMode = CameraSettings::CloudModeNight;
        settings.m_cloudFilterStars = filterStars;
        settings.m_starDetect = true;
        settings.m_plateSolve = false;

        CameraCloudDetector cloudDetector;
        CameraStarDetector starDetector;
        cloudDetector.setNextStage(&starDetector);
        return runChain(cloudDetector, {&cloudDetector, &starDetector}, settings, {image});
    };

    const CloudRunResult unfiltered = runStarChain(false);
    const CloudRunResult filtered = runStarChain(true);
    if (!unfiltered.completed(1) || !filtered.completed(1))
    {
        context.check(false, "star-filtering", unfiltered.error + " " + filtered.error);
        return;
    }

    const int unfilteredCount = unfiltered.frames.first()->m_starDetections.size();
    const int filteredCount = filtered.frames.first()->m_starDetections.size();
    context.check(unfilteredCount > 0, "star-filtering",
        QStringLiteral("stars detected without filtering: %1").arg(unfilteredCount));
    context.check(filteredCount < unfilteredCount, "star-filtering",
        QStringLiteral("filtering dropped clouded detections: %1 -> %2").arg(unfilteredCount).arg(filteredCount));

    // Every star injected into the clear region must survive filtering
    int survivingClearStars = 0;
    for (const cv::Point& expected : clearStars)
    {
        for (const CameraPipelineStarDetection& detection : filtered.frames.first()->m_starDetections)
        {
            const double distance = std::hypot(
                detection.m_center.x() - expected.x,
                detection.m_center.y() - expected.y);
            if (distance <= 5.0)
            {
                ++survivingClearStars;
                break;
            }
        }
    }
    context.check(survivingClearStars == clearStars.size(), "star-filtering",
        QStringLiteral("clear-sky stars survive filtering: %1 of %2").arg(survivingClearStars).arg(clearStars.size()));

    // No filtered detection may sit inside the cloud mask
    bool cloudedDetection = false;
    for (const CameraPipelineStarDetection& detection : filtered.frames.first()->m_starDetections)
    {
        if (filtered.frames.first()->m_cloud.isCloudAtImagePoint(detection.m_center)) {
            cloudedDetection = true;
        }
    }
    context.check(!cloudedDetection, "star-filtering", "no filtered detection inside the cloud mask");
}

void testMotionFiltering(TestContext& context)
{
    // A bright blob moving across a black background; the cloud mask is hand-stamped as
    // full coverage, so with filtering enabled every motion box must be suppressed
    auto makeMotionImage = [](int blobX) -> QImage
    {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::rectangle(bgr, cv::Rect(blobX, 200, 30, 30), cv::Scalar(255, 255, 255), cv::FILLED);
        return bgrToImage(bgr);
    };

    auto runMotion = [&](bool filterMotion) -> CloudRunResult
    {
        CameraSettings settings = makeCloudSettings();
        settings.m_cloudDetect = false;
        settings.m_cloudFilterMotion = filterMotion;
        settings.m_motionDetect = true;
        settings.m_motionConfirmFrames = 1;
        settings.m_motionDownscale = 1.0;

        CameraMotionDetector detector(nullptr);
        MessageQueue outputQueue;
        detector.setNextStageInputMessageQueue(&outputQueue);

        CloudRunResult result;
        QEventLoop loop;
        QTimer timeout;
        timeout.setSingleShot(true);

        // Several static frames to learn the background, then the blob moves
        QVector<QImage> images;
        for (int i = 0; i < 5; ++i) {
            images.append(makeMotionImage(100));
        }
        images.append(makeMotionImage(300));

        CameraPipelineCloud cloud;
        cloud.m_mask = cv::Mat(imageHeight / 4, imageWidth / 4, CV_8UC1, cv::Scalar(255));
        cloud.m_roi = cv::Rect(0, 0, imageWidth, imageHeight);
        cloud.m_coveragePercent = 100.0f;
        cloud.m_valid = true;

        int submitted = 0;
        auto submitNext = [&]() {
            if (submitted < images.size())
            {
                CameraPipelineFramePtr frame(new CameraPipelineFrame);
                frame->m_image = images[submitted];
                frame->m_unprocessedImage = images[submitted];
                frame->m_cloud = cloud;
                detector.getInputMessageQueue()->push(Camera::MsgProcessFrame::create(frame));
                ++submitted;
            }
        };

        QObject::connect(&outputQueue, &MessageQueue::messageEnqueued, &loop, [&]() {
            Message *message = nullptr;
            while ((message = outputQueue.pop()) != nullptr)
            {
                if (Camera::MsgProcessFrame::match(*message))
                {
                    const Camera::MsgProcessFrame& frameMessage =
                        static_cast<const Camera::MsgProcessFrame&>(*message);
                    result.frames.append(frameMessage.getFrame());
                }
                delete message;
            }
            if (result.frames.size() >= images.size()) {
                loop.quit();
            } else {
                submitNext();
            }
        });
        QObject::connect(&timeout, &QTimer::timeout, &loop, [&]() {
            result.error = QStringLiteral("Timed out waiting for motion detector output");
            loop.quit();
        });

        detector.startWork();
        detector.getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, QList<QString>(), true));
        detector.getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, 0));

        submitNext();
        timeout.start(60000);
        loop.exec();
        detector.stopWork();
        detector.getInputMessageQueue()->clear();
        outputQueue.clear();
        return result;
    };

    const CloudRunResult unfiltered = runMotion(false);
    const CloudRunResult filtered = runMotion(true);
    if (!unfiltered.completed(6) || !filtered.completed(6))
    {
        context.check(false, "motion-filtering", unfiltered.error + " " + filtered.error);
        return;
    }

    context.check(!unfiltered.frames.last()->m_motionBoxes.isEmpty(), "motion-filtering",
        QStringLiteral("motion detected without filtering: %1 boxes").arg(unfiltered.frames.last()->m_motionBoxes.size()));
    context.check(filtered.frames.last()->m_motionBoxes.isEmpty(), "motion-filtering",
        QStringLiteral("motion suppressed under full cloud mask: %1 boxes").arg(filtered.frames.last()->m_motionBoxes.size()));
}

// When the frame carries a valid observation time, auto mode must decide day/night from the
// sun elevation at the camera position, overriding frame brightness entirely: a dark frame
// captured at midday takes the day path, a bright frame captured at midnight the night path
void testSunElevationMode(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_latitude = 52.0f;
    settings.m_longitude = 0.0f;

    const QDateTime noon(QDate(2026, 1, 15), QTime(12, 0), QTimeZone::UTC);
    const CloudRunResult day = runCloudDetector(settings, {makeNightImage(false, false)}, nullptr, false, noon);

    const QDateTime midnight(QDate(2026, 1, 15), QTime(0, 0), QTimeZone::UTC);
    const CloudRunResult night = runCloudDetector(settings, {makeDayImage(1.0)}, nullptr, false, midnight);

    if (!day.completed(1) || !night.completed(1))
    {
        context.check(false, "sun-elevation", day.error + " " + night.error);
        return;
    }
    context.check(day.frames.first()->m_cloud.m_valid && !day.frames.first()->m_cloud.m_night,
        "sun-elevation", "dark frame at midday classified via day path");
    context.check(night.frames.first()->m_cloud.m_valid && night.frames.first()->m_cloud.m_night,
        "sun-elevation", "bright frame at midnight classified via night path");

    // With use-capture-date-time disabled, the plate-solve fixed date/time overrides the
    // frame's capture time (recorded media without a usable timestamp)
    settings.m_plateSolveUseCaptureDateTime = false;
    settings.m_plateSolveDateTime = midnight;
    const CloudRunResult overridden = runCloudDetector(settings, {makeDayImage(1.0)}, nullptr, false, noon);
    if (!overridden.completed(1))
    {
        context.check(false, "sun-elevation", overridden.error);
        return;
    }
    context.check(overridden.frames.first()->m_cloud.m_valid && overridden.frames.first()->m_cloud.m_night,
        "sun-elevation", "fixed plate-solve date/time overrides the frame capture time");
}

// In the pre-dawn/dusk twilight band the sun is below the horizon but a high-gain camera
// makes the sky read bright, so brightness cannot be trusted to pick day/night. Auto mode
// must route the whole sub-day-elevation range to night. A bright overcast frame at a
// high-latitude winter noon (sun a few degrees below the horizon) must classify as night,
// not day (which the old brightness fallback would have chosen for a bright frame).
void testTwilightRoutesToNight(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    // At ~76N in mid-January the sun peaks a few degrees below the horizon at local noon:
    // permanent civil/nautical twilight, never full day, never full night
    settings.m_latitude = 76.0f;
    settings.m_longitude = 0.0f;

    const QDateTime twilightNoon(QDate(2026, 1, 15), QTime(12, 0), QTimeZone::UTC);
    const CloudRunResult result = runCloudDetector(settings, {makeDayImage(1.0)}, nullptr, false, twilightNoon);
    if (!result.completed(1))
    {
        context.check(false, "twilight-routing", result.error);
        return;
    }
    context.check(result.frames.first()->m_cloud.m_valid && result.frames.first()->m_cloud.m_night,
        "twilight-routing", "bright twilight frame routes to night, not day");
}

void testAutoModeSelection(TestContext& context)
{
    // Bright overcast image must take the day path; dark image must take the night path
    const CloudRunResult day = runCloudDetector(makeCloudSettings(), {makeDayImage(1.0)});
    const CloudRunResult night = runCloudDetector(makeCloudSettings(), {makeNightImage(false, false)});
    if (!day.completed(1) || !night.completed(1))
    {
        context.check(false, "auto-mode", day.error + " " + night.error);
        return;
    }
    context.check(day.frames.first()->m_cloud.m_valid && !day.frames.first()->m_cloud.m_night,
        "auto-mode", "bright frame classified via day path");
    context.check(night.frames.first()->m_cloud.m_valid && night.frames.first()->m_cloud.m_night,
        "auto-mode", "dark frame classified via night path");
}

QImage loadTestImage(const QString& imagePath)
{
    const cv::Mat bgr = cv::imread(imagePath.toStdString(), cv::IMREAD_COLOR);
    if (bgr.empty()) {
        return QImage();
    }
    return bgrToImage(bgr);
}

// Fraction of a full-image rectangle classified as cloud, sampled on a grid
double regionCloudFraction(const CameraPipelineFramePtr& frame, const QRect& region, int step)
{
    int cloudSamples = 0;
    int totalSamples = 0;
    for (int y = region.top(); y <= region.bottom(); y += step)
    {
        for (int x = region.left(); x <= region.right(); x += step)
        {
            ++totalSamples;
            if (frame->m_cloud.isCloudAtImagePoint(QPointF(x, y))) {
                ++cloudSamples;
            }
        }
    }
    return totalSamples > 0 ? static_cast<double>(cloudSamples) / totalSamples : 0.0;
}

// Day-path validation on a real all-sky fisheye frame (dense white cloud in the upper half,
// clear blue sky lower-right, dark roof/foreground at the bottom): the white cloud is
// detected while the dark roof and the clear blue sky are not. The texture veto still trims
// bright textured foliage, so enabling it lowers overall coverage.
void testDayFisheye(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "day-fisheye", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;

    const CloudRunResult withVeto = runCloudDetector(settings, {image});

    settings.m_cloudTextureThreshold = 0;
    const CloudRunResult withoutVeto = runCloudDetector(settings, {image});

    if (!withVeto.completed(1) || !withoutVeto.completed(1))
    {
        context.check(false, "day-fisheye", withVeto.error + " " + withoutVeto.error);
        return;
    }

    const CameraPipelineFramePtr& vetoFrame = withVeto.frames.first();
    const CameraPipelineFramePtr& noVetoFrame = withoutVeto.frames.first();

    context.check(vetoFrame->m_cloud.m_valid && !vetoFrame->m_cloud.m_night, "day-fisheye", "classified via day path");
    context.check(coverageInRange(vetoFrame, 20.0f, 42.0f), "day-fisheye",
        coverageText(vetoFrame) + " expected roughly 30 %");
    context.check(regionCloudFraction(vetoFrame, QRect(180, 180, 420, 420), 20) > 0.9,
        "day-fisheye", "dense white cloud classified as cloud");
    context.check(regionCloudFraction(vetoFrame, QRect(120, 920, 260, 128), 15) < 0.05,
        "day-fisheye", "dark roof/foreground classified as clear");
    context.check(regionCloudFraction(vetoFrame, QRect(700, 700, 240, 240), 20) < 0.15,
        "day-fisheye", "clear blue sky classified as clear");
    // The texture veto trims bright textured foliage, so it lowers overall coverage
    context.check(vetoFrame->m_cloud.m_coveragePercent < noVetoFrame->m_cloud.m_coveragePercent,
        "day-fisheye", QStringLiteral("texture veto lowers coverage %1 -> %2 %")
            .arg(noVetoFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(vetoFrame->m_cloud.m_coveragePercent, 0, 'f', 1));
}

// When output scaling pads the image inside a larger canvas (recorded on the frame's image
// transform), the padded borders must be excluded from the coverage denominator and from the
// mask. Uses synthetic images so it does not depend on any particular test photo.
void testScalingBorders(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;

    // Content = the top half of the frame; the bottom half is treated as padding/border
    CameraPipelineImageTransform transform;
    transform.setScaled(QSize(imageWidth, imageHeight / 2), QRect(0, 0, imageWidth, imageHeight / 2));

    // Cloud over the top half, clear blue over the bottom half: cloud is ~half of the whole
    // frame but ~all of the content, so restricting the denominator to the content raises the
    // coverage substantially
    const QImage partial = makeDayImage(0.5);
    const CloudRunResult full = runCloudDetector(settings, {partial});
    const CloudRunResult content = runCloudDetector(settings, {partial}, &transform);
    if (!full.completed(1) || !content.completed(1))
    {
        context.check(false, "scaling-borders", full.error + " " + content.error);
        return;
    }
    const float coverageFull = full.frames.first()->m_cloud.m_coveragePercent;
    const float coverageContent = content.frames.first()->m_cloud.m_coveragePercent;
    context.check(coverageContent > coverageFull + 25.0f, "scaling-borders",
        QStringLiteral("coverage over content (%1 %) exceeds over full frame (%2 %)")
            .arg(coverageContent, 0, 'f', 1).arg(coverageFull, 0, 'f', 1));

    // A fully cloudy frame whose bottom half is border: the content cloud classifies, the
    // border cloud does not (it is excluded from the mask), and the border is not counted
    const QImage allCloud = makeDayImage(1.0);
    const CloudRunResult bordered = runCloudDetector(settings, {allCloud}, &transform);
    if (!bordered.completed(1))
    {
        context.check(false, "scaling-borders", bordered.error);
        return;
    }
    const CameraPipelineCloud& cloud = bordered.frames.first()->m_cloud;
    context.check(cloud.isCloudAtImagePoint(QPointF(imageWidth / 2.0, imageHeight / 4.0)),
        "scaling-borders", "cloud inside the content classified as cloud");
    context.check(!cloud.isCloudAtImagePoint(QPointF(imageWidth / 2.0, imageHeight * 3.0 / 4.0)),
        "scaling-borders", "cloudy padded border not classified as cloud");
}

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
// The CUDA path runs the prepare step (crop/downscale/luminance/median) on the GPU and shares
// the classification tail with the CPU path, so the two must agree apart from resize/filter
// rounding differences
void testCudaParity(TestContext& context)
{
    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        context.check(true, "cuda-parity", "skipped: no CUDA-enabled OpenCV device available");
        return;
    }

    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "cuda-parity", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;

    settings.m_postProcessUseCuda = false;
    const CloudRunResult cpu = runCloudDetector(settings, {image});

    settings.m_postProcessUseCuda = true;
    const CloudRunResult gpu = runCloudDetector(settings, {image}, nullptr, true);

    if (!cpu.completed(1) || !gpu.completed(1))
    {
        context.check(false, "cuda-parity", cpu.error + " " + gpu.error);
        return;
    }

    const CameraPipelineFramePtr& cpuFrame = cpu.frames.first();
    const CameraPipelineFramePtr& gpuFrame = gpu.frames.first();
    context.check(cpuFrame->m_cloud.m_valid && gpuFrame->m_cloud.m_valid, "cuda-parity", "both paths produced a result");
    context.check(cpuFrame->m_cloud.m_night == gpuFrame->m_cloud.m_night, "cuda-parity", "day/night decisions agree");

    const float coverageDelta = std::abs(cpuFrame->m_cloud.m_coveragePercent - gpuFrame->m_cloud.m_coveragePercent);
    context.check(coverageDelta < 1.5f, "cuda-parity",
        QStringLiteral("coverage CPU %1 % vs CUDA %2 % (delta %3)")
            .arg(cpuFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(gpuFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(coverageDelta, 0, 'f', 2));

    // Masks must agree pointwise apart from boundary rounding: sample a grid over the frame
    int samples = 0;
    int disagreements = 0;
    for (int y = 0; y < image.height(); y += 12)
    {
        for (int x = 0; x < image.width(); x += 12)
        {
            ++samples;
            if (cpuFrame->m_cloud.isCloudAtImagePoint(QPointF(x, y)) != gpuFrame->m_cloud.isCloudAtImagePoint(QPointF(x, y))) {
                ++disagreements;
            }
        }
    }
    const double disagreementFraction = static_cast<double>(disagreements) / std::max(1, samples);
    context.check(disagreementFraction < 0.01, "cuda-parity",
        QStringLiteral("mask agreement: %1 of %2 sampled points differ").arg(disagreements).arg(samples));

    // The dark-foreground rejection and the cloud detection must hold on the GPU images too
    context.check(regionCloudFraction(gpuFrame, QRect(120, 920, 260, 128), 15) < 0.05,
        "cuda-parity", "dark roof/foreground rejected on the CUDA path");
    context.check(regionCloudFraction(gpuFrame, QRect(180, 180, 420, 420), 20) > 0.9,
        "cuda-parity", "real cloud detected on the CUDA path");
}
#endif

// Regression test for a real moonlit night frame (fisheye, high gain, blue clear sky with
// stars, white/pink cirrus): the luminance-deviation night path used to classify almost the
// whole frame as cloud; the moonlit branch must classify by adaptive colour ratio instead
void testMoonlitNight(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "moonlit-night", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "moonlit-night", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "moonlit-night", "classified via night path");
    context.check(coverageInRange(frame, 25.0f, 55.0f), "moonlit-night",
        coverageText(frame) + " expected roughly 40 % (was ~93 % with the luminance-deviation approach)");
    context.check(regionCloudFraction(frame, QRect(400, 520, 200, 200), 20) < 0.05,
        "moonlit-night", "clear blue centre (with stars) classified as clear");
    context.check(regionCloudFraction(frame, QRect(400, 800, 200, 120), 20) < 0.05,
        "moonlit-night", "clear blue lower sky classified as clear");
    context.check(regionCloudFraction(frame, QRect(800, 400, 200, 240), 20) > 0.5,
        "moonlit-night", "cirrus band classified as cloud");
    context.check(regionCloudFraction(frame, QRect(120, 160, 160, 160), 20) > 0.8,
        "moonlit-night", "moonlit cloud patch classified as cloud");
}

// Regression test for a mostly-overcast moonlit night frame: bright white/grey cloud
// dominates, with darker clear gaps showing stars. The colour anchor alone under-detected
// here (little clear sky to calibrate against), so the brightness cue must fill in the
// moonlit cloud while the dark clear gaps stay clear
void testMoonlitOvercast(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night2.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "moonlit-overcast", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "moonlit-overcast", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "moonlit-overcast", "classified via night path");
    // ~80 % of the full frame, once the black corners are discounted, is nearly all of the
    // fisheye circle apart from the darkest clear gap (was ~22 % before the brightness cue
    // and the day-threshold cap)
    context.check(coverageInRange(frame, 60.0f, 88.0f), "moonlit-overcast",
        coverageText(frame) + " expected roughly 80 % of the frame");
    // The core of the gap; its boundary picks up close-morphology bleed from the
    // surrounding fully-flagged cloud
    context.check(regionCloudFraction(frame, QRect(1320, 1220, 160, 180), 20) < 0.05,
        "moonlit-overcast", "dark clear gap (with stars) classified as clear");
    context.check(regionCloudFraction(frame, QRect(296, 1408, 444, 516), 20) > 0.9,
        "moonlit-overcast", "dense white cloud classified as cloud");
    context.check(regionCloudFraction(frame, QRect(740, 88, 740, 280), 20) > 0.7,
        "moonlit-overcast", "bright cloud sheet at the top classified as cloud");
}

// Regression test for a fully overcast moonlit night frame: the clear-sky colour anchor has
// no clear sky to land on, so without the day-threshold cap the adaptive threshold rises
// above the cloud's own colour and under-detects (40 % in the user's central ROI, where day
// mode correctly saw 86 %)
void testMoonlitFullOvercast(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night4.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "moonlit-full-overcast", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult fullFrame = runCloudDetector(settings, {image});
    if (!fullFrame.completed(1))
    {
        context.check(false, "moonlit-full-overcast", fullFrame.error);
        return;
    }

    const CameraPipelineFramePtr& frame = fullFrame.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "moonlit-full-overcast", "classified via night path");
    context.check(coverageInRange(frame, 65.0f, 88.0f), "moonlit-full-overcast",
        coverageText(frame) + " expected roughly 76 % of the frame (fisheye circle nearly fully overcast)");
    context.check(regionCloudFraction(frame, QRect(800, 1400, 720, 800), 20) > 0.95,
        "moonlit-full-overcast", "overcast centre classified as cloud");

    // The user's scenario: a central detection ROI that contains only cloud
    settings.m_detectionRoiX = 549;
    settings.m_detectionRoiY = 801;
    settings.m_detectionRoiWidth = 1100;
    settings.m_detectionRoiHeight = 1604;
    const CloudRunResult centralRoi = runCloudDetector(settings, {image});
    if (!centralRoi.completed(1))
    {
        context.check(false, "moonlit-full-overcast", centralRoi.error);
        return;
    }
    context.check(coverageInRange(centralRoi.frames.first(), 90.0f, 100.0f), "moonlit-full-overcast",
        coverageText(centralRoi.frames.first()) + " expected > 90 % for the all-cloud central ROI (was ~40 %)");
}

// Regression test for a half-overcast moonlit night frame: bright cloud over one half, dark
// clear starfield over the other. The sky median is dark, so gating the moonlit branch on
// the median sent this down the dark path, whose local contrast misses the sheet interior
// (~14 %); gating on the bright quartile keeps it on the colour path (~57 %)
void testMoonlitHalfOvercast(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night5.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "moonlit-half-overcast", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "moonlit-half-overcast", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "moonlit-half-overcast", "classified via night path");
    context.check(coverageInRange(frame, 45.0f, 70.0f), "moonlit-half-overcast",
        coverageText(frame) + " expected roughly 57 % (was ~14 % on the dark path)");
    context.check(regionCloudFraction(frame, QRect(600, 160, 1000, 480), 20) > 0.95,
        "moonlit-half-overcast", "bright cloud mass classified as cloud");
    context.check(regionCloudFraction(frame, QRect(1080, 1320, 520, 560), 20) < 0.15,
        "moonlit-half-overcast", "dark clear starfield classified as clear");
}

// Regression test for a bright pre-dawn frame (high gain, ~1h before sunrise) that is mostly
// cloud with real blue-sky gaps: the moonlit path must flag the cloud while leaving the gaps
// clear. The day path badly under-detects this pastel twilight cloud (~40 %), which is why
// twilight routes to night.
void testTwilightBrokenCloud(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night8.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "twilight-broken-cloud", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "twilight-broken-cloud", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "twilight-broken-cloud", "classified via night path");
    context.check(coverageInRange(frame, 65.0f, 88.0f), "twilight-broken-cloud",
        coverageText(frame) + " expected roughly 78 %");
    context.check(regionCloudFraction(frame, QRect(600, 1200, 680, 680), 20) > 0.85,
        "twilight-broken-cloud", "solid cloud mass classified as cloud");
    context.check(regionCloudFraction(frame, QRect(1600, 320, 280, 480), 20) < 0.25,
        "twilight-broken-cloud", "blue-sky gap classified as clear");
}

// Regression test for a fully-overcast pastel pre-dawn frame. This is the fundamental limit
// of colour-based night detection: the overcast is bluish, so it is spectrally identical to
// a clear blue sky and the adaptive anchor lands on the bluest cloud, leaving that cloud
// under-detected (~69 % instead of ~90 %). Local structure cannot rescue it either (a fix
// that flags night9's bluish region also flags the genuine clear starfield in cloud-night).
// Only star-visibility sensing could separate them; documented here so a real improvement
// visibly changes the numbers.
void testFullOvercastPastelLimit(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night9.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "full-overcast-pastel", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "full-overcast-pastel", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "full-overcast-pastel", "classified via night path");
    // Majority is detected, but not the ~90 % the fully-overcast sky truly is
    context.check(coverageInRange(frame, 55.0f, 82.0f), "full-overcast-pastel",
        coverageText(frame) + " majority detected (bluish overcast under-detected - known limit)");
    context.check(regionCloudFraction(frame, QRect(1000, 240, 800, 720), 20) > 0.9,
        "full-overcast-pastel", "bright pink cloud classified as cloud");
}

// Regression test for a mostly-clear dark night frame with a strong smooth sky gradient
// (glow brightening the top of the fisheye) and a few small cloud patches. The dark path
// fits a smooth surface to the sky and flags deviations, so the smooth top-glow gradient is
// absorbed (not flagged) while the localized cloud patches stand out.
void testDarkNightGradient(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night3.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "dark-night-gradient", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "dark-night-gradient", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "dark-night-gradient", "classified via night path");
    context.check(coverageInRange(frame, 1.0f, 12.0f), "dark-night-gradient",
        coverageText(frame) + " expected under ~10 % (was ~45 % with the global-median approach)");
    context.check(regionCloudFraction(frame, QRect(920, 228, 612, 384), 20) < 0.02,
        "dark-night-gradient", "bright clear sky under the moon glow classified as clear");
    context.check(regionCloudFraction(frame, QRect(1376, 1224, 536, 460), 20) < 0.02,
        "dark-night-gradient", "dark clear sky (with stars) classified as clear");
    // The smooth horizon-glow band across the top must not classify (the surface fit
    // absorbs it); a local box-average background used to flag it as cloud
    context.check(regionCloudFraction(frame, QRect(1120, 200, 560, 320), 20) < 0.05,
        "dark-night-gradient", "top horizon-glow band classified as clear");
    context.check(regionCloudFraction(frame, QRect(32, 1148, 352, 612), 20) > 0.1,
        "dark-night-gradient", "cloud patches at the left edge detected");
}

// Regression test for a genuinely clear dark night (starfield, no cloud). The sky brightens
// toward the horizon (glow + vignette); a local box-average background flagged that bright
// rim as cloud (~17 %), while the surface-fit background absorbs the smooth gradient so the
// clear sky reads near zero.
void testDarkNightClear(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/no-cloud.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "dark-night-clear", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "dark-night-clear", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "dark-night-clear", "classified via night path");
    context.check(coverageInRange(frame, 0.0f, 8.0f), "dark-night-clear",
        coverageText(frame) + " expected near 0 % for a clear starfield (was ~17 %)");
    // The broad horizon-glow band that a local box-average flagged is now absorbed by the
    // surface fit: the left edge and the lower sky read clear. (A small residual can remain
    // where glow meets foliage at the top-right, which the smooth fit cannot fully model.)
    context.check(regionCloudFraction(frame, QRect(20, 160, 160, 320), 15) < 0.05,
        "dark-night-clear", "left-edge horizon glow classified as clear");
    context.check(regionCloudFraction(frame, QRect(720, 720, 240, 220), 15) < 0.05,
        "dark-night-clear", "lower sky classified as clear");
}

// Regression test for a dark, mostly-clear night frame with dim white cloud wisps along one
// edge: the local-contrast dark path must pick up the wisps (a large dim veil needs the wide
// surround window to retain contrast) while the dark starry sky stays completely clear
void testDarkNightWisps(TestContext& context)
{
    const QString imagePath = QDir(QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR))
        .filePath(QStringLiteral("images-cloud/cloud-night6.jpg"));
    const QImage image = loadTestImage(imagePath);
    if (image.isNull())
    {
        context.check(false, "dark-night-wisps", QStringLiteral("failed to load %1").arg(imagePath));
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;

    const CloudRunResult result = runCloudDetector(settings, {image});
    if (!result.completed(1))
    {
        context.check(false, "dark-night-wisps", result.error);
        return;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    context.check(frame->m_cloud.m_valid && frame->m_cloud.m_night, "dark-night-wisps", "classified via night path");
    context.check(coverageInRange(frame, 3.0f, 14.0f), "dark-night-wisps",
        coverageText(frame) + " expected roughly 7 %");
    context.check(regionCloudFraction(frame, QRect(40, 280, 400, 1040), 20) > 0.2,
        "dark-night-wisps", "dim cloud wisps at the left edge detected");
    context.check(regionCloudFraction(frame, QRect(1000, 800, 800, 1000), 20) < 0.02,
        "dark-night-wisps", "dark clear starfield classified as clear");
    context.check(regionCloudFraction(frame, QRect(600, 1920, 1000, 480), 20) < 0.02,
        "dark-night-wisps", "dark clear lower sky classified as clear");
}

// Scheduler-event state machine: an event describing the initial sky state fires on the
// first report after capture start, then only on threshold transitions with hysteresis
void testCloudEventTracker(TestContext& context)
{
    constexpr double threshold = 80.0;
    CameraCloudEventTracker tracker;

    // Clear sky at capture start announces Low
    context.check(tracker.update(10.0, threshold) == CameraCloudEventTracker::Low,
        "cloud-events", "startup event announces the initial clear sky");
    context.check(tracker.update(20.0, threshold) == CameraCloudEventTracker::None,
        "cloud-events", "no event while staying below the threshold");
    context.check(tracker.update(85.0, threshold) == CameraCloudEventTracker::High,
        "cloud-events", "crossing the threshold fires High");
    context.check(tracker.update(90.0, threshold) == CameraCloudEventTracker::None,
        "cloud-events", "no event while staying above the threshold");
    // Hovering inside the hysteresis band must not chatter
    context.check(tracker.update(75.0, threshold) == CameraCloudEventTracker::None,
        "cloud-events", "no event inside the hysteresis band");
    context.check(tracker.update(82.0, threshold) == CameraCloudEventTracker::None,
        "cloud-events", "no repeat High when rising back inside the band");
    context.check(tracker.update(69.0, threshold) == CameraCloudEventTracker::Low,
        "cloud-events", "falling 10 points below the threshold fires Low");
    context.check(tracker.update(79.0, threshold) == CameraCloudEventTracker::None,
        "cloud-events", "no event rising towards the threshold");

    // A new capture with an overcast sky announces High immediately
    tracker.reset();
    context.check(tracker.update(95.0, threshold) == CameraCloudEventTracker::High,
        "cloud-events", "startup event announces the initial overcast sky");
}

void testDisabledDetector(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudDetect = false;
    const CloudRunResult result = runCloudDetector(settings, {makeNightImage(true, false)});
    if (!result.completed(1))
    {
        context.check(false, "disabled", result.error);
        return;
    }
    context.check(!result.frames.first()->m_cloud.m_valid, "disabled", "frames carry an invalid cloud result");
}

// Writes an overlay PNG: the original image with a 50 % red tint over the computed cloud
// mask, so the detector's actual classification can be inspected by eye.
void writeMaskOverlay(const QString& imagePath, const CameraPipelineCloud& cloud, const QString& outPath)
{
    cv::Mat bgr = cv::imread(imagePath.toStdString(), cv::IMREAD_COLOR);
    if (bgr.empty()) {
        return;
    }

    cv::Mat overlay = bgr.clone();
    if (cloud.m_valid && !cloud.m_mask.empty() && (cloud.m_roi.width > 0) && (cloud.m_roi.height > 0))
    {
        // The mask is in downscaled detection-ROI space; upscale it to the ROI and place it
        // in a full-frame mask
        cv::Mat roiMask;
        cv::resize(cloud.m_mask, roiMask, cv::Size(cloud.m_roi.width, cloud.m_roi.height), 0.0, 0.0, cv::INTER_NEAREST);
        cv::Mat fullMask = cv::Mat::zeros(bgr.size(), CV_8UC1);
        const cv::Rect roi = cloud.m_roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
        roiMask(cv::Rect(0, 0, roi.width, roi.height)).copyTo(fullMask(roi));

        cv::Mat red(bgr.size(), CV_8UC3, cv::Scalar(0, 0, 255));
        cv::Mat blended;
        cv::addWeighted(bgr, 0.5, red, 0.5, 0.0, blended);
        blended.copyTo(overlay, fullMask);
    }
    cv::imwrite(outPath.toStdString(), overlay);
}

// Runs the real detector over every image in images-cloud/ in both day and night mode,
// writes red-tint mask overlays to outDir, and prints a coverage table.
int dumpMasks(const QString& outDir)
{
    const QString imagesDir = QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR) + QStringLiteral("/images-cloud");
    QDir().mkpath(outDir);
    QDir dir(imagesDir);
    const QStringList files = dir.entryList(QStringList{QStringLiteral("*.jpg")}, QDir::Files, QDir::Name);
    if (files.isEmpty())
    {
        std::cout << "No images found in " << imagesDir.toStdString() << "\n";
        return 1;
    }

    std::cout << "image,night_coverage%,night_path,day_coverage%\n";
    for (const QString& file : files)
    {
        const QString path = dir.filePath(file);
        const QImage image = loadTestImage(path);
        if (image.isNull()) {
            continue;
        }
        const QString base = QFileInfo(file).completeBaseName();

        CameraSettings nightSettings = makeCloudSettings();
        nightSettings.m_cloudMode = CameraSettings::CloudModeNight;
        const CloudRunResult nightRun = runCloudDetector(nightSettings, {image});

        CameraSettings daySettings = makeCloudSettings();
        daySettings.m_cloudMode = CameraSettings::CloudModeDay;
        const CloudRunResult dayRun = runCloudDetector(daySettings, {image});

        const bool nightOk = nightRun.completed(1) && nightRun.frames.first()->m_cloud.m_valid;
        const bool dayOk = dayRun.completed(1) && dayRun.frames.first()->m_cloud.m_valid;
        const float nightCov = nightOk ? nightRun.frames.first()->m_cloud.m_coveragePercent : -1.0f;
        const float dayCov = dayOk ? dayRun.frames.first()->m_cloud.m_coveragePercent : -1.0f;
        // Which internal night path ran: moonlit (colour) vs dark (local contrast)
        const char *nightPath = nightOk ? (nightRun.frames.first()->m_cloud.m_night ? "night" : "day") : "-";

        if (nightOk) {
            writeMaskOverlay(path, nightRun.frames.first()->m_cloud, outDir + "/" + base + "_night.png");
        }
        if (dayOk) {
            writeMaskOverlay(path, dayRun.frames.first()->m_cloud, outDir + "/" + base + "_day.png");
        }

        std::cout << file.toStdString() << ","
                  << QString::number(nightCov, 'f', 1).toStdString() << ","
                  << nightPath << ","
                  << QString::number(dayCov, 'f', 1).toStdString() << "\n";
    }
    std::cout << "overlays written to " << QDir(outDir).absolutePath().toStdString() << "\n";
    return 0;
}

} // namespace

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QGuiApplication::setOrganizationName(QStringLiteral("f4exb"));
    QGuiApplication::setApplicationName(QStringLiteral("SDRangel"));

    const QStringList args = app.arguments();

    if (args.contains(QStringLiteral("--dump-masks")))
    {
        const int idx = args.indexOf(QStringLiteral("--dump-masks"));
        const QString outDir = (idx + 1 < args.size())
            ? args.at(idx + 1)
            : (QString::fromUtf8(CAMERA_CLOUD_TEST_DATA_DIR) + QStringLiteral("/cloud-mask-output"));
        return dumpMasks(outDir);
    }

    const QString filter = (args.size() > 1) ? args.at(1) : QString();

    struct NamedTest
    {
        const char *name;
        void (*run)(TestContext&);
    };
    const NamedTest tests[] = {
        {"night-clear", testNightClear},
        {"night-cloudy", testNightCloudy},
        {"day-clear", testDayClear},
        {"day-overcast", testDayOvercast},
        {"day-partial", testDayPartial},
        {"exclusion-rects", testExclusionRects},
        {"edge-margin", testEdgeMarginMask},
        {"sun-mask", testSunMoonMask},
        {"update-interval", testUpdateIntervalCaching},
        {"star-filtering", testStarFiltering},
        {"motion-filtering", testMotionFiltering},
        {"auto-mode", testAutoModeSelection},
        {"sun-elevation", testSunElevationMode},
        {"twilight-routing", testTwilightRoutesToNight},
        {"day-fisheye", testDayFisheye},
        {"moonlit-night", testMoonlitNight},
        {"moonlit-overcast", testMoonlitOvercast},
        {"moonlit-full-overcast", testMoonlitFullOvercast},
        {"moonlit-half-overcast", testMoonlitHalfOvercast},
        {"twilight-broken-cloud", testTwilightBrokenCloud},
        {"full-overcast-pastel", testFullOvercastPastelLimit},
        {"dark-night-gradient", testDarkNightGradient},
        {"dark-night-clear", testDarkNightClear},
        {"dark-night-wisps", testDarkNightWisps},
        {"scaling-borders", testScalingBorders},
#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
        {"cuda-parity", testCudaParity},
#endif
        {"cloud-events", testCloudEventTracker},
        {"disabled", testDisabledDetector},
    };

    TestContext context;
    int run = 0;
    for (const NamedTest& test : tests)
    {
        if (!filter.isEmpty() && (filter != QLatin1String(test.name))) {
            continue;
        }
        test.run(context);
        ++run;
    }

    if (run == 0)
    {
        std::cout << "No test matched filter: " << filter.toStdString() << "\n";
        return 1;
    }

    std::cout << (context.failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED") << "\n";
    return context.failures == 0 ? 0 : 1;
}
