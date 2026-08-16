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

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QEventLoop>
#include <QGuiApplication>
#include <QImage>
#include <QStringList>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTimeZone>
#include <QTimer>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera.h"
#include "cameraclearskyreference.h"
#include "cameraclouddetector.h"
#include "cameramotiondetector.h"
#include "cameraplatesolver.h"
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
CloudRunResult runCloudDetector(const CameraSettings& settings, const QVector<QImage>& images, const CameraPipelineImageTransform* imageTransform = nullptr, bool uploadToGpu = false, const QDateTime& captureDateTime = QDateTime(), bool saveReferenceFirst = false, const QString& saveTestCaseDir = QString())
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
    if (saveReferenceFirst) {
        detector.getInputMessageQueue()->push(CameraCloudDetector::MsgSaveClearSkyReference::create());
    }
    if (!saveTestCaseDir.isEmpty()) {
        detector.getInputMessageQueue()->push(CameraCloudDetector::MsgSaveCloudTestCase::create(saveTestCaseDir));
    }

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
    // Real all-sky lens calibration (rolled, off-centre, distorted), so the projection path
    // is exercised end to end rather than in an idealised straight-up configuration.
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;
    settings.m_latitude = 51.5;
    settings.m_longitude = -0.12;
    settings.m_fov = 159.0f;
    settings.m_azimuth = 157.0f;
    settings.m_elevation = 86.0f;
    settings.m_roll = -129.0f;
    settings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
    settings.m_lensDistortionK1 = -0.0743f;
    settings.m_lensCenterOffsetX = 6.3f;
    settings.m_lensCenterOffsetY = -10.5f;
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

    // A detached flare speck near the sun (inside the max-radius disc) and a genuine cloud
    // far from the sun (outside it): the former is glare debris the removal must also clear,
    // the latter must survive to prove the removal is bounded by the disc.
    const QPointF speckPoint = sunPoint + QPointF(45.0, 0.0);
    const QPointF cloudPoint = sunPoint + QPointF(250.0, 100.0);
    auto makeSunImage = [&]() {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
        cv::circle(bgr, cv::Point(static_cast<int>(sunPoint.x()), static_cast<int>(sunPoint.y())),
                   30, cv::Scalar(250, 250, 250), cv::FILLED);
        cv::circle(bgr, cv::Point(static_cast<int>(speckPoint.x()), static_cast<int>(speckPoint.y())),
                   6, cv::Scalar(250, 250, 250), cv::FILLED);
        cv::circle(bgr, cv::Point(static_cast<int>(cloudPoint.x()), static_cast<int>(cloudPoint.y())),
                   40, cv::Scalar(250, 250, 250), cv::FILLED);
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
    context.check(!maskedFrame->m_cloud.isCloudAtImagePoint(speckPoint),
        "sun-mask", "detached flare speck inside the disc also removed");
    context.check(maskedFrame->m_cloud.isCloudAtImagePoint(cloudPoint),
        "sun-mask", "genuine cloud outside the disc survives the removal");
    context.check(maskedFrame->m_cloud.m_coveragePercent < unmaskedFrame->m_cloud.m_coveragePercent,
        "sun-mask", QStringLiteral("coverage dropped from %1 to %2 %")
            .arg(unmaskedFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(maskedFrame->m_cloud.m_coveragePercent, 0, 'f', 1));
}

// Star-visibility sensing: predicted catalog stars visible through a flagged region prove it
// is not (opaque) cloud. A dim smooth haze patch with stars shining through must be vetoed,
// while an opaque cloud blob that blocks its stars must stay flagged.
void testStarSense(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;
    // The synthetic haze/cloud features are small relative to a real frame, so use a
    // background radius proportionate to the 640x480 test image
    settings.m_cloudBackgroundBlur = 8;
    settings.m_latitude = 51.5;
    settings.m_longitude = -0.12;
    settings.m_fov = 178.0f;
    settings.m_azimuth = 0.0f;
    settings.m_elevation = 90.0f;
    settings.m_roll = 0.0f;
    settings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
    settings.m_plateSolveUseCaptureDateTime = true;
    const QDateTime captureTime(QDate(2024, 1, 15), QTime(22, 0, 0), QTimeZone::utc());

    // Replicate the detector's prediction rules (elevation floor, sun/moon avoidance, frame
    // margin) so the test knows exactly which stars the detector will check
    const SkyProjector projector = SkyProjector::create(settings, QSize(imageWidth, imageHeight));
    AzAlt bodyAzAlt;
    RADec bodyRaDec;
    Astronomy::sunPosition(bodyAzAlt, bodyRaDec, settings.m_latitude, settings.m_longitude, captureTime);
    const SkyVector sunVector = skyVectorFromAltAz(bodyAzAlt.az, bodyAzAlt.alt);
    Astronomy::moonPosition(bodyAzAlt, bodyRaDec, settings.m_latitude, settings.m_longitude, captureTime);
    const SkyVector moonVector = skyVectorFromAltAz(bodyAzAlt.az, bodyAzAlt.alt);
    const double avoidCos = std::cos(skyDegToRad(10.0)); // starSenseAvoidBodyDeg

    QVector<QPointF> predicted;
    for (const CameraPlateSolver::BrightStar& star : CameraPlateSolver::brightStarCatalog(settings))
    {
        if (star.magnitude > settings.m_cloudStarSenseMagnitude) {
            continue;
        }
        const RADec raDec{star.rightAscensionDegrees / 15.0, star.declinationDegrees}; // ra in hours
        const AzAlt azAlt = Astronomy::raDecToAzAlt(raDec, settings.m_latitude, settings.m_longitude, captureTime);
        if (azAlt.alt < 15.0) {
            continue;
        }
        const SkyVector starVector = skyVectorFromAltAz(azAlt.az, azAlt.alt);
        if ((skyDot(starVector, sunVector) > avoidCos) || (skyDot(starVector, moonVector) > avoidCos)) {
            continue;
        }
        QPointF point;
        if (!projector.projectAltAz(azAlt.az, azAlt.alt, point)) {
            continue;
        }
        if ((point.x() < 20.0) || (point.y() < 20.0)
            || (point.x() >= imageWidth - 20.0) || (point.y() >= imageHeight - 20.0)) {
            continue;
        }
        predicted.append(point);
    }
    context.check(predicted.size() >= 4, "star-sense",
        QStringLiteral("%1 predicted stars available in the frame").arg(predicted.size()));
    if (predicted.size() < 4) {
        return;
    }

    // Haze region: the closest pair of predicted stars, so both land inside one flagged blob
    const auto pairDistance = [&](int a, int b) {
        return QLineF(predicted[a], predicted[b]).length();
    };
    int hazeA = 0, hazeB = 1;
    for (int a = 0; a < predicted.size(); ++a)
    {
        for (int b = a + 1; b < predicted.size(); ++b)
        {
            if (pairDistance(a, b) < pairDistance(hazeA, hazeB)) {
                hazeA = a; hazeB = b;
            }
        }
    }
    const QPointF hazeCentre = (predicted[hazeA] + predicted[hazeB]) / 2.0;
    const double hazeSigma = std::max(pairDistance(hazeA, hazeB) / 2.0 + 40.0, 60.0);

    // Cloud region: the closest pair well clear of the haze
    int cloudA = -1, cloudB = -1;
    for (int a = 0; a < predicted.size(); ++a)
    {
        if (QLineF(predicted[a], hazeCentre).length() < 3.0 * hazeSigma) {
            continue;
        }
        for (int b = a + 1; b < predicted.size(); ++b)
        {
            if (QLineF(predicted[b], hazeCentre).length() < 3.0 * hazeSigma) {
                continue;
            }
            if ((cloudA < 0) || (pairDistance(a, b) < pairDistance(cloudA, cloudB))) {
                cloudA = a; cloudB = b;
            }
        }
    }
    context.check(cloudA >= 0, "star-sense", "found a star pair clear of the haze for the cloud blob");
    if (cloudA < 0) {
        return;
    }
    const QPointF cloudCentre = (predicted[cloudA] + predicted[cloudB]) / 2.0;
    const double cloudRadius = std::max(pairDistance(cloudA, cloudB) / 2.0 + 40.0, 50.0);

    cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(15, 15, 15));
    // Dim smooth haze bump: enough local contrast for the dark path to flag it
    for (int row = 0; row < imageHeight; ++row)
    {
        cv::Vec3b *line = bgr.ptr<cv::Vec3b>(row);
        for (int col = 0; col < imageWidth; ++col)
        {
            const double dx = col - hazeCentre.x();
            const double dy = row - hazeCentre.y();
            const int bump = static_cast<int>(std::lround(16.0 * std::exp(-(dx * dx + dy * dy) / (2.0 * hazeSigma * hazeSigma))));
            if (bump > 0)
            {
                for (int channel = 0; channel < 3; ++channel) {
                    line[col][channel] = static_cast<uchar>(std::min(255, line[col][channel] + bump));
                }
            }
        }
    }
    // Opaque cloud blob over the cloud star pair
    cv::circle(bgr, cv::Point(static_cast<int>(cloudCentre.x()), static_cast<int>(cloudCentre.y())),
               static_cast<int>(cloudRadius), cv::Scalar(70, 70, 70), cv::FILLED);
    cv::GaussianBlur(bgr, bgr, cv::Size(15, 15), 5.0);
    addNoise(bgr, 2.0, 97531);
    // Stars shine everywhere except through the opaque cloud
    for (const QPointF& point : predicted)
    {
        if (QLineF(point, cloudCentre).length() <= cloudRadius + 5.0) {
            continue;
        }
        addStar(bgr, cv::Point(static_cast<int>(point.x()), static_cast<int>(point.y())));
    }
    const QImage image = bgrToImage(bgr);

    settings.m_cloudStarSense = false;
    const CloudRunResult unsensed = runCloudDetector(settings, {image}, nullptr, false, captureTime);
    settings.m_cloudStarSense = true;
    const CloudRunResult sensed = runCloudDetector(settings, {image}, nullptr, false, captureTime);

    if (!unsensed.completed(1) || !sensed.completed(1))
    {
        context.check(false, "star-sense", unsensed.error + " " + sensed.error);
        return;
    }

    const CameraPipelineFramePtr& unsensedFrame = unsensed.frames.first();
    const CameraPipelineFramePtr& sensedFrame = sensed.frames.first();
    context.check(unsensedFrame->m_cloud.m_valid && sensedFrame->m_cloud.m_valid, "star-sense", "both runs valid");
    context.check(unsensedFrame->m_cloud.isCloudAtImagePoint(hazeCentre),
        "star-sense", "haze flagged as cloud without star sensing");
    context.check(unsensedFrame->m_cloud.isCloudAtImagePoint(cloudCentre),
        "star-sense", "opaque cloud flagged without star sensing");
    context.check(!sensedFrame->m_cloud.isCloudAtImagePoint(hazeCentre),
        "star-sense", "haze with stars shining through vetoed by star sensing");
    context.check(sensedFrame->m_cloud.isCloudAtImagePoint(cloudCentre),
        "star-sense", "opaque cloud that blocks its stars stays flagged");
    context.check(sensedFrame->m_cloud.m_coveragePercent < unsensedFrame->m_cloud.m_coveragePercent,
        "star-sense", QStringLiteral("coverage dropped from %1 to %2 %")
            .arg(unsensedFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
            .arg(sensedFrame->m_cloud.m_coveragePercent, 0, 'f', 1));

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
    // CUDA parity: a GPU-resident frame exercises the CUDA work-image preparation and the
    // GPU patch-download path of the visibility check; the outcome must match the CPU path
    if (cv::cuda::getCudaEnabledDeviceCount() > 0)
    {
        settings.m_postProcessUseCuda = true;
        const CloudRunResult sensedGpu = runCloudDetector(settings, {image}, nullptr, true, captureTime);
        if (sensedGpu.completed(1) && sensedGpu.frames.first()->m_cloud.m_valid)
        {
            const CameraPipelineFramePtr& gpuFrame = sensedGpu.frames.first();
            context.check(!gpuFrame->m_cloud.isCloudAtImagePoint(hazeCentre),
                "star-sense", "CUDA-input path also vetoes the haze");
            context.check(gpuFrame->m_cloud.isCloudAtImagePoint(cloudCentre),
                "star-sense", "CUDA-input path keeps the opaque cloud");
            context.check(std::abs(gpuFrame->m_cloud.m_coveragePercent - sensedFrame->m_cloud.m_coveragePercent) < 1.5f,
                "star-sense", QStringLiteral("coverage parity CPU %1 vs CUDA %2 %")
                    .arg(sensedFrame->m_cloud.m_coveragePercent, 0, 'f', 1)
                    .arg(gpuFrame->m_cloud.m_coveragePercent, 0, 'f', 1));
        }
        else
        {
            context.check(false, "star-sense", "CUDA-input run completed");
        }
    }
#endif
}

// Star-blank cue: a cluster of RECENTLY SEEN stars that all vanish is cloud, even when the
// region carries no brightness or colour signature at all (thin/dark cloud). Two frames:
// the first shows every star (recording the sightings), the second hides a cluster. The
// same frames run with star sensing disabled to show nothing else flags the region, and
// the cue must abstain on the first frame (no prior sightings).
void testStarBlankCue(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;
    settings.m_cloudBackgroundBlur = 8;
    settings.m_cloudUpdateIntervalFrames = 1; // recompute on both frames
    settings.m_latitude = 51.5;
    settings.m_longitude = -0.12;
    settings.m_fov = 178.0f;
    settings.m_azimuth = 0.0f;
    settings.m_elevation = 90.0f;
    settings.m_roll = 0.0f;
    settings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
    settings.m_plateSolveUseCaptureDateTime = true;
    settings.m_cloudStarSenseMagnitude = 6.0f; // deep enough for star clusters to exist
    const QDateTime captureTime(QDate(2024, 1, 15), QTime(22, 0, 0), QTimeZone::utc());

    // Predict the stars the detector will check, with magnitudes (same rules as the detector)
    struct Predicted
    {
        QPointF point;
        float magnitude;
    };
    const SkyProjector projector = SkyProjector::create(settings, QSize(imageWidth, imageHeight));
    AzAlt bodyAzAlt;
    RADec bodyRaDec;
    Astronomy::sunPosition(bodyAzAlt, bodyRaDec, settings.m_latitude, settings.m_longitude, captureTime);
    const SkyVector sunVector = skyVectorFromAltAz(bodyAzAlt.az, bodyAzAlt.alt);
    Astronomy::moonPosition(bodyAzAlt, bodyRaDec, settings.m_latitude, settings.m_longitude, captureTime);
    const SkyVector moonVector = skyVectorFromAltAz(bodyAzAlt.az, bodyAzAlt.alt);
    const double avoidCos = std::cos(skyDegToRad(10.0)); // starSenseAvoidBodyDeg
    QVector<Predicted> predicted;
    for (const CameraPlateSolver::BrightStar& star : CameraPlateSolver::brightStarCatalog(settings))
    {
        if (star.magnitude > settings.m_cloudStarSenseMagnitude) {
            continue;
        }
        const RADec raDec{star.rightAscensionDegrees / 15.0, star.declinationDegrees}; // ra in hours
        const AzAlt azAlt = Astronomy::raDecToAzAlt(raDec, settings.m_latitude, settings.m_longitude, captureTime);
        if (azAlt.alt < 15.0) {
            continue;
        }
        const SkyVector starVector = skyVectorFromAltAz(azAlt.az, azAlt.alt);
        if ((skyDot(starVector, sunVector) > avoidCos) || (skyDot(starVector, moonVector) > avoidCos)) {
            continue;
        }
        QPointF point;
        if (!projector.projectAltAz(azAlt.az, azAlt.alt, point)) {
            continue;
        }
        if ((point.x() < 20.0) || (point.y() < 20.0)
            || (point.x() >= imageWidth - 20.0) || (point.y() >= imageHeight - 20.0)) {
            continue;
        }
        predicted.append({point, static_cast<float>(star.magnitude)});
    }

    // A blank cluster: a bright star with at least two neighbours inside the cue's
    // neighbourhood radius (kStarBlankNeighbourFraction of the long side); everything in
    // the neighbourhood goes undrawn so no visible star can veto the cluster
    const double neighbourRadius = 0.08 * imageWidth;
    int centreIndex = -1;
    for (int i = 0; i < predicted.size(); ++i)
    {
        if (predicted[i].magnitude > 4.5f) {
            continue; // the blanked star must be clearly brighter than the faintest drawn one
        }
        int neighbours = 0;
        for (int j = 0; j < predicted.size(); ++j)
        {
            if ((j != i) && (QLineF(predicted[j].point, predicted[i].point).length() <= neighbourRadius)) {
                ++neighbours;
            }
        }
        if ((neighbours >= 2) && ((centreIndex < 0) || (predicted[i].magnitude < predicted[centreIndex].magnitude))) {
            centreIndex = i;
        }
    }
    if (centreIndex < 0)
    {
        context.check(true, "star-blank", "skipped: no suitable star cluster in this configuration");
        return;
    }
    const QPointF blankCentre = predicted[centreIndex].point;

    // Frame 1 draws every predicted star; frame 2 hides the cluster on a featureless dark
    // sky (pixel-identical to clear sky there, so only the vanished stars can reveal it).
    // The undrawn region extends a patch half-width plus margin beyond the cue's
    // neighbourhood radius: the visibility patches tolerate position error by design, so
    // an undrawn star just inside the neighbourhood would otherwise read "visible" off a
    // drawn star's dot at the edge of its patch and veto the cluster.
    const double blankRadius = neighbourRadius + 30.0;
    QPointF drawnFarAway;
    int drawn = 0;
    const auto makeSkyImage = [&](bool hideCluster) {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(15, 15, 15));
        addNoise(bgr, 2.0, 24601);
        for (const Predicted& star : predicted)
        {
            if (hideCluster && (QLineF(star.point, blankCentre).length() <= blankRadius)) {
                continue;
            }
            addStar(bgr, cv::Point(static_cast<int>(star.point.x()), static_cast<int>(star.point.y())));
            if (hideCluster)
            {
                ++drawn;
                if (QLineF(star.point, blankCentre).length() > 3.0 * neighbourRadius) {
                    drawnFarAway = star.point;
                }
            }
        }
        return bgrToImage(bgr);
    };
    const QImage allStarsImage = makeSkyImage(false);
    const QImage clusterHiddenImage = makeSkyImage(true);
    context.check(drawn >= 5, "star-blank",
        QStringLiteral("%1 stars drawn outside the blank cluster").arg(drawn));
    if (drawn < 5) {
        return;
    }

    settings.m_cloudStarSense = false;
    const CloudRunResult unsensed = runCloudDetector(settings, {allStarsImage, clusterHiddenImage}, nullptr, false, captureTime);
    settings.m_cloudStarSense = true;
    const CloudRunResult sensed = runCloudDetector(settings, {allStarsImage, clusterHiddenImage}, nullptr, false, captureTime);
    if (!unsensed.completed(2) || !sensed.completed(2))
    {
        context.check(false, "star-blank", unsensed.error + " " + sensed.error);
        return;
    }

    const CameraPipelineFramePtr& unsensedFrame = unsensed.frames.last();
    const CameraPipelineFramePtr& sensedFirst = sensed.frames.first();
    const CameraPipelineFramePtr& sensedFrame = sensed.frames.last();
    context.check(unsensedFrame->m_cloud.m_valid && sensedFrame->m_cloud.m_valid, "star-blank", "both runs valid");
    context.check(!unsensedFrame->m_cloud.isCloudAtImagePoint(blankCentre),
        "star-blank", "featureless cloud invisible to the brightness/colour cues");
    context.check(sensedFirst->m_cloud.m_valid && !sensedFirst->m_cloud.isCloudAtImagePoint(blankCentre),
        "star-blank", "cue abstains on the first frame (no prior sightings)");
    context.check(sensedFrame->m_cloud.isCloudAtImagePoint(blankCentre),
        "star-blank", "cluster of vanished stars flagged as cloud by the blank cue");
    if (!drawnFarAway.isNull()) {
        context.check(!sensedFrame->m_cloud.isCloudAtImagePoint(drawnFarAway),
            "star-blank", "sky with its stars visible stays clear");
    }
}

// Minimum-elevation floor: sky below the configured elevation is excluded from
// classification and the coverage denominator. Also round-trips the projector's new
// unprojection against the forward projection it inverts.
void testMinElevationMask(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;
    settings.m_fov = 159.0f;
    settings.m_azimuth = 157.0f;
    settings.m_elevation = 86.0f;
    settings.m_roll = -129.0f;
    settings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
    settings.m_lensDistortionK1 = -0.0743f;
    settings.m_lensCenterOffsetX = 6.3f;
    settings.m_lensCenterOffsetY = -10.5f;

    const SkyProjector projector = SkyProjector::create(settings, QSize(imageWidth, imageHeight));

    // Unprojection round-trip on a spread of sky directions (exercises the distortion
    // inversion and the camera basis inverse)
    int roundTrips = 0;
    double worstError = 0.0;
    for (int azimuth = 0; azimuth < 360; azimuth += 45)
    {
        for (int elevation = 15; elevation <= 75; elevation += 30)
        {
            QPointF point;
            if (!projector.projectAltAz(azimuth, elevation, point)
                || (point.x() < 0) || (point.y() < 0) || (point.x() >= imageWidth) || (point.y() >= imageHeight)) {
                continue;
            }
            double azimuthBack = 0.0;
            double elevationBack = 0.0;
            if (projector.unprojectToAltAz(point, azimuthBack, elevationBack))
            {
                ++roundTrips;
                double azimuthError = std::fabs(azimuthBack - azimuth);
                if (azimuthError > 180.0) {
                    azimuthError = 360.0 - azimuthError;
                }
                worstError = std::max(worstError, std::max(azimuthError, std::fabs(elevationBack - elevation)));
            }
        }
    }
    context.check((roundTrips >= 8) && (worstError < 0.2), "min-elevation",
        QStringLiteral("unprojection round-trips projection (%1 samples, worst error %2°)")
            .arg(roundTrips).arg(worstError, 0, 'f', 3));

    // Two white blobs on blue sky: one low (below the floor), one high (above it), found
    // adaptively so the test does not depend on the exact lens numbers
    QPointF lowPoint;
    QPointF highPoint;
    bool found = false;
    for (int azimuth = 0; (azimuth < 360) && !found; azimuth += 10)
    {
        found = projector.projectAltAz(azimuth, 12.0, lowPoint)
            && projector.projectAltAz(azimuth, 70.0, highPoint)
            && (lowPoint.x() >= 60) && (lowPoint.y() >= 60)
            && (lowPoint.x() < imageWidth - 60) && (lowPoint.y() < imageHeight - 60)
            && (highPoint.x() >= 60) && (highPoint.y() >= 60)
            && (highPoint.x() < imageWidth - 60) && (highPoint.y() < imageHeight - 60);
    }
    context.check(found, "min-elevation", "found in-frame test points above and below the floor");
    if (!found) {
        return;
    }

    // Blob radius must comfortably exceed the texture-veto window at the default 0.25
    // downscale, or the edge texture bleeds across the whole interior and vetoes it
    auto makeImage = [&]() {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
        cv::circle(bgr, cv::Point(static_cast<int>(lowPoint.x()), static_cast<int>(lowPoint.y())),
                   45, cv::Scalar(250, 250, 250), cv::FILLED);
        cv::circle(bgr, cv::Point(static_cast<int>(highPoint.x()), static_cast<int>(highPoint.y())),
                   45, cv::Scalar(250, 250, 250), cv::FILLED);
        addNoise(bgr, 2.0, 11223);
        return bgrToImage(bgr);
    };

    settings.m_cloudMinElevation = 0.0;
    const CloudRunResult unbounded = runCloudDetector(settings, {makeImage()});
    settings.m_cloudMinElevation = 40.0;
    const CloudRunResult bounded = runCloudDetector(settings, {makeImage()});
    if (!unbounded.completed(1) || !bounded.completed(1))
    {
        context.check(false, "min-elevation", unbounded.error + " " + bounded.error);
        return;
    }

    const CameraPipelineFramePtr& unboundedFrame = unbounded.frames.first();
    const CameraPipelineFramePtr& boundedFrame = bounded.frames.first();
    context.check(unboundedFrame->m_cloud.m_valid && boundedFrame->m_cloud.m_valid, "min-elevation",
        QStringLiteral("both runs valid (coverage %1 / %2 %)")
            .arg(unboundedFrame->m_cloud.m_coveragePercent, 0, 'f', 2)
            .arg(boundedFrame->m_cloud.m_coveragePercent, 0, 'f', 2));
    context.check(unboundedFrame->m_cloud.isCloudAtImagePoint(lowPoint),
        "min-elevation", QStringLiteral("low blob at (%1, %2) flagged without a floor")
            .arg(lowPoint.x(), 0, 'f', 0).arg(lowPoint.y(), 0, 'f', 0));
    context.check(unboundedFrame->m_cloud.isCloudAtImagePoint(highPoint),
        "min-elevation", QStringLiteral("high blob at (%1, %2) flagged without a floor")
            .arg(highPoint.x(), 0, 'f', 0).arg(highPoint.y(), 0, 'f', 0));
    context.check(!boundedFrame->m_cloud.isCloudAtImagePoint(lowPoint),
        "min-elevation", "blob below the elevation floor excluded from evaluation");
    context.check(boundedFrame->m_cloud.isCloudAtImagePoint(highPoint),
        "min-elevation", "blob above the floor stays flagged");
}

void testSkyProjectorAzimuthWrapping(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_fov = 150.0f;
    settings.m_azimuth = 220.0f;
    settings.m_elevation = 60.0f;
    settings.m_roll = 0.0f;
    settings.m_lensProjection = CameraSettings::LensProjectionEquidistant;

    const SkyProjector projector = SkyProjector::create(settings, QSize(imageWidth, imageHeight));
    QPointF negativePoint;
    QPointF canonicalPoint;
    QPointF overflowPoint;
    QPointF distinctPoint;
    const bool projected = projector.projectAltAz(-170.0, 25.0, negativePoint)
        && projector.projectAltAz(190.0, 25.0, canonicalPoint)
        && projector.projectAltAz(550.0, 25.0, overflowPoint)
        && projector.projectAltAz(230.0, 25.0, distinctPoint);
    context.check(projected, "azimuth-wrap", "wrapped and canonical azimuths project");
    if (!projected) {
        return;
    }

    const auto pointDistance = [](const QPointF& lhs, const QPointF& rhs) {
        return std::hypot(lhs.x() - rhs.x(), lhs.y() - rhs.y());
    };
    context.check(pointDistance(negativePoint, canonicalPoint) < 1e-6
            && pointDistance(overflowPoint, canonicalPoint) < 1e-6,
        "azimuth-wrap", "-170, 190 and 550 degrees are equivalent");
    context.check(pointDistance(canonicalPoint, distinctPoint) > 10.0,
        "azimuth-wrap", "distinct azimuths above 180 degrees remain spatially distinct");
}

// Day auto-learn sun gate: a day frame whose sun is projected in-frame but shows no glare
// is overcast in front of the sun and must not be learned as the clear-sky reference, even
// when the (self-measured) coverage passes the gate; visible glare permits learning.
void testDayLearnSunGate(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;
    settings.m_latitude = 51.5;
    settings.m_longitude = -0.12;
    settings.m_fov = 159.0f;
    settings.m_azimuth = 157.0f;
    settings.m_elevation = 86.0f;
    settings.m_roll = -129.0f;
    settings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
    settings.m_lensDistortionK1 = -0.0743f;
    settings.m_lensCenterOffsetX = 6.3f;
    settings.m_lensCenterOffsetY = -10.5f;
    settings.m_plateSolveUseCaptureDateTime = true;
    settings.m_cloudUseReference = true;
    settings.m_cloudAutoReference = true;

    const QDateTime captureTime(QDate(2024, 6, 21), QTime(12, 0, 0), QTimeZone::utc());

    AzAlt sunAzAlt;
    RADec sunRaDec;
    Astronomy::sunPosition(sunAzAlt, sunRaDec, settings.m_latitude, settings.m_longitude, captureTime);
    const SkyProjector projector = SkyProjector::create(settings, QSize(imageWidth, imageHeight));
    QPointF sunPoint;
    const bool inFrame = projector.projectAltAz(sunAzAlt.az, sunAzAlt.alt, sunPoint)
        && (sunPoint.x() >= 30) && (sunPoint.x() < imageWidth - 30)
        && (sunPoint.y() >= 30) && (sunPoint.y() < imageHeight - 30);
    context.check(inFrame, "sun-gate", "sun projects into the frame");
    if (!inFrame) {
        return;
    }

    const QString clearSkyDir = QString::fromLocal8Bit(qgetenv("SDRANGEL_CAMERA_CLEARSKY_DIR"));
    const auto storeCount = [&]() {
        return QDir(clearSkyDir).entryList(QStringList{QStringLiteral("*.csr")}, QDir::Files).size();
    };

    auto makeSkyImage = [&](bool sunVisible) {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
        if (sunVisible) {
            cv::circle(bgr, cv::Point(static_cast<int>(sunPoint.x()), static_cast<int>(sunPoint.y())),
                       18, cv::Scalar(250, 250, 250), cv::FILLED);
        }
        addNoise(bgr, 2.0, 33445);
        return bgrToImage(bgr);
    };

    // Blocked: an otherwise clear-looking day frame with the sun obscured must not learn.
    // (This is the exact frame a uniformly dark overcast produces on a camera where cloud
    // is colorimetrically invisible - coverage reads near zero and self-verifies.)
    settings.m_cameraId = QStringLiteral("test-sungate-blocked");
    const int beforeBlocked = storeCount();
    const CloudRunResult blocked = runCloudDetector(settings, {makeSkyImage(false)}, nullptr, false, captureTime);
    context.check(blocked.completed(1), "sun-gate", "obscured-sun run completed");
    context.check(storeCount() == beforeBlocked, "sun-gate",
        "no reference learned while the sun is obscured");

    // Allowed: the same sky with the sun's glare visible learns normally
    settings.m_cameraId = QStringLiteral("test-sungate-visible");
    const int beforeVisible = storeCount();
    const CloudRunResult visible = runCloudDetector(settings, {makeSkyImage(true)}, nullptr, false, captureTime);
    context.check(visible.completed(1), "sun-gate", "visible-sun run completed");
    context.check(storeCount() == beforeVisible + 1, "sun-gate",
        "reference learned once the sun's glare is visible");
}

// Day dark cue: dark grey overcast shares clear blue sky's red/blue ratio on IR-sensitive
// cameras, so with a day reference the only signature is standing darker than the known
// clear sky. The patch must go undetected without the reference (nothing else can see it)
// and detected with it.
void testDarkDayCue(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;
    settings.m_cloudUseReference = true;
    settings.m_cameraId = QStringLiteral("test-darkday");
    settings.m_latitude = 51.5;
    settings.m_longitude = -0.12;
    settings.m_plateSolveUseCaptureDateTime = true;
    const QDateTime captureTime(QDate(2024, 6, 21), QTime(12, 0, 0), QTimeZone::utc());

    const cv::Point patchCentre(450, 160);
    const cv::Point clearProbe(150, 300);

    auto makeClearImage = [&]() {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
        addNoise(bgr, 2.0, 55667);
        return bgrToImage(bgr);
    };
    // Dark patch with the SAME red/blue ratio as the sky, just darker - colorimetrically
    // identical to clear sky, as dark overcast is on IR-heavy cameras
    auto makeDarkPatchImage = [&]() {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(200, 120, 80));
        cv::circle(bgr, patchCentre, 60, cv::Scalar(110, 66, 44), cv::FILLED);
        cv::GaussianBlur(bgr, bgr, cv::Size(9, 9), 3.0);
        addNoise(bgr, 2.0, 77889);
        return bgrToImage(bgr);
    };

    // Save the day reference from the clear frame
    const CloudRunResult saved = runCloudDetector(settings, {makeClearImage()}, nullptr, false, captureTime, true);
    context.check(saved.completed(1), "ref-darkday", "reference capture run completed");

    // Without the reference nothing can see the patch (same colour, above the brightness floor)
    CameraSettings noRefSettings = settings;
    noRefSettings.m_cloudUseReference = false;
    const CloudRunResult withoutRef = runCloudDetector(noRefSettings, {makeDarkPatchImage()}, nullptr, false, captureTime);
    // With the reference the dark cue flags it
    const CloudRunResult withRef = runCloudDetector(settings, {makeDarkPatchImage()}, nullptr, false, captureTime);
    if (!withoutRef.completed(1) || !withRef.completed(1))
    {
        context.check(false, "ref-darkday", withoutRef.error + " " + withRef.error);
        return;
    }

    const CameraPipelineFramePtr& withoutFrame = withoutRef.frames.first();
    const CameraPipelineFramePtr& withFrame = withRef.frames.first();
    context.check(withoutFrame->m_cloud.m_valid && withFrame->m_cloud.m_valid, "ref-darkday", "both runs valid");
    context.check(!withoutFrame->m_cloud.isCloudAtImagePoint(QPointF(patchCentre.x, patchCentre.y)),
        "ref-darkday", "colour-identical dark patch invisible without the reference");
    context.check(withFrame->m_cloud.isCloudAtImagePoint(QPointF(patchCentre.x, patchCentre.y)),
        "ref-darkday", "dark patch flagged by the darker-than-reference cue");
    context.check(!withFrame->m_cloud.isCloudAtImagePoint(QPointF(clearProbe.x, clearProbe.y)),
        "ref-darkday", "unchanged clear sky stays clear");
}

// Clear-sky reference model: a saved reference vetoes this camera's static clear-sky
// quirks (v1), genuine deviations from it stay detected (v1), foreground learned from the
// reference is excluded from evaluation (v3), and auto-learning fills the store from
// verified-clear frames without the button (v2). Persistence is exercised throughout:
// every run constructs a fresh detector that loads the reference from disk.
void testClearSkyReference(TestContext& context)
{
    const QDateTime captureTime(QDate(2024, 1, 15), QTime(22, 0, 0), QTimeZone::utc());
    const cv::Point quirkCentre(200, 160);      // static glow pocket: false positive without a reference
    const cv::Point cloudCentre(450, 160);      // genuine cloud in the second frame
    const cv::Point foregroundProbe(320, 420);  // inside the dark foreground band

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;
    settings.m_cloudBackgroundBlur = 8; // proportionate to the small synthetic frame
    settings.m_cloudUseReference = true;
    settings.m_cameraId = QStringLiteral("test-clearsky");
    settings.m_latitude = 51.5;
    settings.m_longitude = -0.12;
    settings.m_plateSolveUseCaptureDateTime = true;

    // Clear night sky with this camera's quirks: airglow base, a smooth glow pocket the
    // dark path misreads as cloud, and a dark foreground band (trees/roof silhouette)
    auto makeClearImage = [&]() {
        cv::Mat bgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(40, 40, 40));
        for (int row = 0; row < imageHeight; ++row)
        {
            cv::Vec3b *line = bgr.ptr<cv::Vec3b>(row);
            for (int col = 0; col < imageWidth; ++col)
            {
                const double dx = col - quirkCentre.x;
                const double dy = row - quirkCentre.y;
                const int bump = static_cast<int>(std::lround(18.0 * std::exp(-(dx * dx + dy * dy) / (2.0 * 50.0 * 50.0))));
                if (bump > 0)
                {
                    for (int channel = 0; channel < 3; ++channel) {
                        line[col][channel] = static_cast<uchar>(std::min(255, line[col][channel] + bump));
                    }
                }
            }
        }
        cv::rectangle(bgr, cv::Rect(0, 360, imageWidth, imageHeight - 360), cv::Scalar(5, 5, 5), cv::FILLED);
        addNoise(bgr, 2.0, 86420);
        return bgrToImage(bgr);
    };
    const QImage clearImage = makeClearImage();

    // Without a reference the glow pocket is a false positive
    const CloudRunResult before = runCloudDetector(settings, {clearImage}, nullptr, false, captureTime);
    if (!before.completed(1) || !before.frames.first()->m_cloud.m_valid)
    {
        context.check(false, "clear-sky-ref", before.error);
        return;
    }
    context.check(before.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(quirkCentre.x, quirkCentre.y)),
        "clear-sky-ref", "glow quirk misread as cloud without a reference");

    // Save the reference from the clear frame
    const CloudRunResult saved = runCloudDetector(settings, {clearImage}, nullptr, false, captureTime, true);
    context.check(saved.completed(1), "clear-sky-ref", "reference capture run completed");

    // A fresh detector loads the reference from disk and vetoes the quirk
    const CloudRunResult vetoed = runCloudDetector(settings, {clearImage}, nullptr, false, captureTime);
    if (!vetoed.completed(1) || !vetoed.frames.first()->m_cloud.m_valid)
    {
        context.check(false, "clear-sky-ref", vetoed.error);
        return;
    }
    context.check(!vetoed.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(quirkCentre.x, quirkCentre.y)),
        "clear-sky-ref", "glow quirk vetoed once the reference knows it");
    context.check(vetoed.frames.first()->m_cloud.m_coveragePercent < 1.0f,
        "clear-sky-ref", QStringLiteral("clear frame reads %1 % with the reference")
            .arg(vetoed.frames.first()->m_cloud.m_coveragePercent, 0, 'f', 1));

    // Genuine cloud deviating from the reference stays detected; foreground band is
    // excluded from evaluation even when something bright appears there (v3)
    cv::Mat cloudyBgr(imageHeight, imageWidth, CV_8UC3, cv::Scalar(40, 40, 40));
    {
        const QImage& base = clearImage;
        for (int row = 0; row < imageHeight; ++row)
        {
            cv::Vec3b *line = cloudyBgr.ptr<cv::Vec3b>(row);
            const uchar *scan = base.constScanLine(row);
            for (int col = 0; col < imageWidth; ++col)
            {
                // base is RGB888 from bgrToImage
                line[col] = cv::Vec3b(scan[3 * col + 2], scan[3 * col + 1], scan[3 * col]);
            }
        }
        cv::circle(cloudyBgr, cloudCentre, 60, cv::Scalar(90, 90, 90), cv::FILLED);
        cv::circle(cloudyBgr, foregroundProbe, 40, cv::Scalar(90, 90, 90), cv::FILLED);
        cv::GaussianBlur(cloudyBgr, cloudyBgr, cv::Size(9, 9), 3.0);
    }
    const QImage cloudyImage = bgrToImage(cloudyBgr);

    const CloudRunResult cloudy = runCloudDetector(settings, {cloudyImage}, nullptr, false, captureTime);
    if (!cloudy.completed(1) || !cloudy.frames.first()->m_cloud.m_valid)
    {
        context.check(false, "clear-sky-ref", cloudy.error);
        return;
    }
    context.check(cloudy.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(cloudCentre.x, cloudCentre.y)),
        "clear-sky-ref", "genuine cloud deviating from the reference stays detected");
    context.check(!cloudy.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(quirkCentre.x, quirkCentre.y)),
        "clear-sky-ref", "quirk stays vetoed in the cloudy frame");
    context.check(!cloudy.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(foregroundProbe.x, foregroundProbe.y)),
        "clear-sky-ref", "learned foreground band excluded from evaluation");

    // Control: without the reference the bright blob on the foreground band is flagged
    CameraSettings noRefSettings = settings;
    noRefSettings.m_cloudUseReference = false;
    const CloudRunResult control = runCloudDetector(noRefSettings, {cloudyImage}, nullptr, false, captureTime);
    if (control.completed(1) && control.frames.first()->m_cloud.m_valid)
    {
        context.check(control.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(foregroundProbe.x, foregroundProbe.y)),
            "clear-sky-ref", "control: foreground blob flagged without the reference");
    }
    else
    {
        context.check(false, "clear-sky-ref", control.error);
    }

    // Twilight drift: the sky evolves as a smooth gradient within minutes of a save. The
    // deviation surface removal must keep the quirk vetoed on the drifted frame instead of
    // the veto going stale (and the cue misreading the drift as cloud).
    {
        cv::Mat driftedBgr(imageHeight, imageWidth, CV_8UC3);
        const QImage& base = clearImage;
        for (int row = 0; row < imageHeight; ++row)
        {
            cv::Vec3b *line = driftedBgr.ptr<cv::Vec3b>(row);
            const uchar *scan = base.constScanLine(row);
            for (int col = 0; col < imageWidth; ++col)
            {
                const int drift = (25 * col) / imageWidth; // smooth horizontal brightening
                line[col] = cv::Vec3b(
                    static_cast<uchar>(std::min(255, scan[3 * col + 2] + drift)),
                    static_cast<uchar>(std::min(255, scan[3 * col + 1] + drift)),
                    static_cast<uchar>(std::min(255, scan[3 * col] + drift)));
            }
        }
        const QImage driftedImage = bgrToImage(driftedBgr);
        const CloudRunResult drifted = runCloudDetector(settings, {driftedImage}, nullptr, false, captureTime);
        if (drifted.completed(1) && drifted.frames.first()->m_cloud.m_valid)
        {
            context.check(!drifted.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(quirkCentre.x, quirkCentre.y)),
                "clear-sky-ref", "quirk stays vetoed after smooth twilight-style drift");
            context.check(drifted.frames.first()->m_cloud.m_coveragePercent < 2.0f,
                "clear-sky-ref", QStringLiteral("drifted clear frame reads %1 % with the reference")
                    .arg(drifted.frames.first()->m_cloud.m_coveragePercent, 0, 'f', 1));
        }
        else
        {
            context.check(false, "clear-sky-ref", drifted.error);
        }
    }

    // v2 auto-learning: with a fresh camera id and no button press, a verified-clear frame
    // fills the slot on its own; a later run vetoes the quirk
    CameraSettings autoSettings = settings;
    autoSettings.m_cameraId = QStringLiteral("test-autolearn");
    autoSettings.m_cloudAutoReference = true;
    const CloudRunResult learnRun = runCloudDetector(autoSettings, {clearImage}, nullptr, false, captureTime);
    context.check(learnRun.completed(1), "clear-sky-ref", "auto-learn run completed");
    const CloudRunResult afterLearn = runCloudDetector(autoSettings, {clearImage}, nullptr, false, captureTime);
    if (afterLearn.completed(1) && afterLearn.frames.first()->m_cloud.m_valid)
    {
        context.check(!afterLearn.frames.first()->m_cloud.isCloudAtImagePoint(QPointF(quirkCentre.x, quirkCentre.y)),
            "clear-sky-ref", "auto-learned reference vetoes the quirk without a button press");
    }
    else
    {
        context.check(false, "clear-sky-ref", afterLearn.error);
    }
}

void writeMaskOverlay(const QString& imagePath, const CameraPipelineCloud& cloud, const QString& outPath); // defined with the dump harness below

// Loads a test-case bundle saved by the GUI's "Save test case" button (or by the
// round-trip test below) and reruns the detection it captured
struct TestCaseBundle
{
    CameraSettings settings;
    QImage image;
    QDateTime captureTime;
    double savedCoverage = -1.0;
    QString error;

    [[nodiscard]] bool valid() const { return error.isEmpty(); }
};

TestCaseBundle loadTestCaseBundle(const QString& directory)
{
    TestCaseBundle bundle;
    QFile settingsFile(QDir(directory).filePath(QStringLiteral("settings.dat")));
    if (!settingsFile.open(QIODevice::ReadOnly))
    {
        bundle.error = QStringLiteral("cannot read settings.dat in %1").arg(directory);
        return bundle;
    }
    const QByteArray blob = settingsFile.readAll();
    bundle.settings.resetToDefaults();
    if (!bundle.settings.deserialize(blob))
    {
        bundle.error = QStringLiteral("cannot deserialize settings.dat in %1").arg(directory);
        return bundle;
    }

    bundle.image = QImage(QDir(directory).filePath(QStringLiteral("image.png")));
    if (bundle.image.isNull())
    {
        bundle.error = QStringLiteral("cannot read image.png in %1").arg(directory);
        return bundle;
    }

    QFile metaFile(QDir(directory).filePath(QStringLiteral("testcase.json")));
    if (metaFile.open(QIODevice::ReadOnly))
    {
        const QJsonObject meta = QJsonDocument::fromJson(metaFile.readAll()).object();
        const QString when = meta.value(QStringLiteral("captureDateTime")).toString();
        if (!when.isEmpty())
        {
            bundle.captureTime = QDateTime::fromString(when, Qt::ISODateWithMs);
            bundle.captureTime.setTimeZone(QTimeZone::utc());
        }
        bundle.savedCoverage = meta.value(QStringLiteral("coveragePercent")).toDouble(-1.0);
    }
    return bundle;
}

int runTestCaseBundle(const QString& directory, bool debugViews)
{
    // The bundle doubles as the clear-sky reference store for this run
    qputenv("SDRANGEL_CAMERA_CLEARSKY_DIR", QDir(directory).absolutePath().toLocal8Bit());

    TestCaseBundle bundle = loadTestCaseBundle(directory);
    if (!bundle.valid())
    {
        std::cout << "FAIL: " << bundle.error.toStdString() << "\n";
        return 1;
    }
    // Reruns must be idempotent: auto-learning would write into the bundle's own
    // reference store (the env dir above) and change subsequent reruns
    bundle.settings.m_cloudAutoReference = false;

    const CloudRunResult result = runCloudDetector(bundle.settings, {bundle.image}, nullptr, false, bundle.captureTime);
    if (!result.completed(1) || !result.frames.first()->m_cloud.m_valid)
    {
        std::cout << "FAIL: " << result.error.toStdString() << "\n";
        return 1;
    }

    const CameraPipelineFramePtr& frame = result.frames.first();
    std::cout << "coverage " << QString::number(frame->m_cloud.m_coveragePercent, 'f', 1).toStdString()
              << " % (" << (frame->m_cloud.m_night ? "night" : "day") << " path)";
    if (bundle.savedCoverage >= 0.0) {
        std::cout << " - saved run measured " << QString::number(bundle.savedCoverage, 'f', 1).toStdString() << " %";
    }
    std::cout << "\n";

    const QString maskPath = QDir(directory).filePath(QStringLiteral("mask.png"));
    writeMaskOverlay(QDir(directory).filePath(QStringLiteral("image.png")), frame->m_cloud, maskPath);
    std::cout << "mask overlay written to " << maskPath.toStdString() << "\n";

    std::cout << "settings: mode " << bundle.settings.m_cloudMode
              << " dayThreshold " << bundle.settings.m_cloudDayThreshold
              << " textureThreshold " << bundle.settings.m_cloudTextureThreshold
              << " nightThreshold " << bundle.settings.m_cloudNightThreshold
              << " downscale " << bundle.settings.m_cloudDownscale
              << " open/close " << bundle.settings.m_cloudOpenSize << "/" << bundle.settings.m_cloudCloseSize
              << " edgeMargin " << bundle.settings.m_cloudEdgeMarginPercent
              << " sunMoonMask " << bundle.settings.m_cloudMaskSunMoon << " r=" << bundle.settings.m_cloudSunMoonRadiusDeg
              << " starSense " << bundle.settings.m_cloudStarSense
              << " useReference " << bundle.settings.m_cloudUseReference << "\n";

    // Each classification stage rendered as the detector's debug views, for diagnosis
    if (!debugViews) {
        return 0;
    }
    const struct { CameraSettings::CloudDebugView view; const char *name; } views[] = {
        {CameraSettings::CloudDebugViewBackground, "background"},
        {CameraSettings::CloudDebugViewSignal, "signal"},
        {CameraSettings::CloudDebugViewTexture, "texture"},
        {CameraSettings::CloudDebugViewThresholded, "thresholded"},
        {CameraSettings::CloudDebugViewFinal, "final"},
    };
    for (const auto& v : views)
    {
        CameraSettings debugSettings = bundle.settings;
        debugSettings.m_cloudDebugView = v.view;
        const CloudRunResult debugRun = runCloudDetector(debugSettings, {bundle.image}, nullptr, false, bundle.captureTime);
        if (debugRun.completed(1) && !debugRun.frames.first()->m_image.isNull()) {
            debugRun.frames.first()->m_image.save(QDir(directory).filePath(QStringLiteral("debug_%1.png").arg(QLatin1String(v.name))));
        }
    }
    std::cout << "debug views written\n";
    return 0;
}

// Round trip of the GUI's "Save test case" bundle: saving a case and rerunning it from the
// bundle alone must reproduce the same coverage
void testTestCaseBundle(TestContext& context)
{
    static QTemporaryDir caseDir;
    if (!caseDir.isValid())
    {
        context.check(false, "test-case-bundle", "cannot create temporary directory");
        return;
    }

    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeNight;
    const QDateTime captureTime(QDate(2024, 3, 1), QTime(1, 30, 0), QTimeZone::utc());
    const QImage image = makeNightImage(true, false);

    const CloudRunResult saved = runCloudDetector(settings, {image}, nullptr, false, captureTime, false, caseDir.path());
    if (!saved.completed(1) || !saved.frames.first()->m_cloud.m_valid)
    {
        context.check(false, "test-case-bundle", saved.error);
        return;
    }
    const float savedCoverage = saved.frames.first()->m_cloud.m_coveragePercent;

    context.check(QFileInfo::exists(QDir(caseDir.path()).filePath(QStringLiteral("image.png"))),
        "test-case-bundle", "bundle contains image.png");
    context.check(QFileInfo::exists(QDir(caseDir.path()).filePath(QStringLiteral("settings.dat"))),
        "test-case-bundle", "bundle contains settings.dat");
    context.check(QFileInfo::exists(QDir(caseDir.path()).filePath(QStringLiteral("testcase.json"))),
        "test-case-bundle", "bundle contains testcase.json");

    const TestCaseBundle bundle = loadTestCaseBundle(caseDir.path());
    context.check(bundle.valid(), "test-case-bundle", bundle.valid() ? QStringLiteral("bundle loads") : bundle.error);
    if (!bundle.valid()) {
        return;
    }
    context.check(bundle.settings.m_cloudMode == CameraSettings::CloudModeNight,
        "test-case-bundle", "settings round-trip preserves the cloud mode");
    context.check(bundle.captureTime == captureTime,
        "test-case-bundle", "capture time round-trips");

    const CloudRunResult rerun = runCloudDetector(bundle.settings, {bundle.image}, nullptr, false, bundle.captureTime);
    if (!rerun.completed(1) || !rerun.frames.first()->m_cloud.m_valid)
    {
        context.check(false, "test-case-bundle", rerun.error);
        return;
    }
    context.check(std::abs(rerun.frames.first()->m_cloud.m_coveragePercent - savedCoverage) < 0.05f,
        "test-case-bundle", QStringLiteral("bundle rerun reproduces coverage (%1 vs %2 %)")
            .arg(rerun.frames.first()->m_cloud.m_coveragePercent, 0, 'f', 2)
            .arg(savedCoverage, 0, 'f', 2));

    // Exercise the standalone --run-case runner itself (it repoints the clear-sky store at
    // the bundle, so restore the harness environment afterwards)
    const QByteArray previousClearSkyDir = qgetenv("SDRANGEL_CAMERA_CLEARSKY_DIR");
    const int runnerExit = runTestCaseBundle(caseDir.path(), false);
    qputenv("SDRANGEL_CAMERA_CLEARSKY_DIR", previousClearSkyDir);
    context.check(runnerExit == 0, "test-case-bundle", "--run-case runner succeeds on the bundle");
    context.check(QFileInfo::exists(QDir(caseDir.path()).filePath(QStringLiteral("mask.png"))),
        "test-case-bundle", "--run-case writes the mask overlay");
}

// Incremental (patchwork) reference learning: when no frame is ever wholly clear, the
// confirmed-clear regions of partly cloudy frames must assemble into a usable reference.
// A pixel joins the comparison only after two throttle-separated confirmations, the
// anchors must come from the confirmed region alone (cloud elsewhere must not bias them),
// a half-assembled slot must not block the neighbouring-bin fallback, and a later
// verified-clear whole frame must adopt the never-confirmed gaps outright.
void testReferencePatchLearning(TestContext& context)
{
    CameraClearSkyReference reference;
    reference.ensureLoaded(QStringLiteral("ref-patch-test"));

    const QRectF roiNorm(0.0, 0.0, 1.0, 1.0);
    const cv::Size work(160, 120);
    const cv::Mat eval(work, CV_8UC1, cv::Scalar(255));
    const cv::Mat texture = cv::Mat::zeros(work, CV_8UC1);
    const QDateTime start(QDate(2024, 6, 1), QTime(1, 0), QTimeZone::utc());
    const int slot = CameraClearSkyReference::slotFromBin(2, false);

    // Frame: clear sky (gray 60) on the left 40%, bright cloud (gray 220) on the right
    // 60%. The whole-sky median brightness lands on the CLOUD, so a whole-frame capture
    // would store a contaminated anchor - the patch path must anchor on the clear region.
    const int clearWidth = (work.width * 2) / 5;
    const cv::Rect cloudRect(clearWidth, 0, work.width - clearWidth, work.height);
    cv::Mat gray(work, CV_8UC1, cv::Scalar(60));
    gray(cloudRect).setTo(220);
    cv::Mat bgr(work, CV_8UC3, cv::Scalar(60, 60, 60));
    bgr(cloudRect).setTo(cv::Scalar(220, 220, 220));
    // The detector always keeps confirmed regions a dilation clearance away from
    // anything flagged as cloud; mirror that here, or the reference-resolution smoothing
    // would bleed the cloud step into the confirmed pixels
    const int clearance = 8;
    cv::Mat confirmed = cv::Mat::zeros(work, CV_8UC1);
    confirmed(cv::Rect(0, 0, clearWidth - clearance, work.height)).setTo(255);

    const auto learnMixed = [&](const QDateTime& when) {
        // Coverage 50% with strong star confirmation: the whole-frame gates must refuse
        // this (star-strong bootstrap caps at 35%), leaving only the patch path
        return reference.autoLearn(slot, gray, bgr, texture, eval, roiNorm, when,
                                   50.0f, true, true, 30, 25, confirmed);
    };

    // Day slots must refuse patch learning outright: no day-time colour test proves a
    // pixel clear (dark cloud and blue sky share the same red/blue ratio on IR-sensitive
    // cameras - observed poisoning a Day reference in the field within two updates)
    context.check(reference.autoLearn(0, gray, bgr, texture, eval, roiNorm, start,
                                      50.0f, false, false, 0, 0, confirmed) == CameraClearSkyReference::LearnResult::None,
        "ref-patch", "day slot refuses patchwork confirmation");

    context.check(learnMixed(start) == CameraClearSkyReference::LearnResult::Patches,
        "ref-patch", "unverified frame learns its confirmed patches");

    // One confirmation is not yet trusted: the comparison abstains entirely
    cv::Mat once(work, CV_8UC1, cv::Scalar(255));
    context.check(!reference.applyCueAndVeto(slot, once, gray, bgr, eval, roiNorm, 8),
        "ref-patch", "a single confirmation is not yet usable");

    // ... and a half-assembled slot must not block the fallback to a usable neighbour
    reference.capture(CameraClearSkyReference::slotFromBin(3, false), gray, bgr, texture, eval, roiNorm, start, -10.0, -20.0);
    cv::Mat viaNeighbour(work, CV_8UC1, cv::Scalar(255));
    context.check(reference.applyCueAndVeto(slot, viaNeighbour, gray, bgr, eval, roiNorm, 8)
            && (cv::countNonZero(viaNeighbour) < static_cast<int>(0.05 * work.area())),
        "ref-patch", "half-assembled slot does not block the neighbouring-bin fallback");

    context.check(learnMixed(start.addSecs(60)) == CameraClearSkyReference::LearnResult::None,
        "ref-patch", "patch learning honours the throttle");
    context.check(learnMixed(start.addSecs(700)) == CameraClearSkyReference::LearnResult::Patches,
        "ref-patch", "second confirmation accepted after the throttle");

    // The confirmed region is now established. A fully clear frame carries the CLEAR
    // anchor (60): if the patch path had anchored on the whole mixed sky (220), the
    // anchor guard would reject this comparison outright.
    cv::Mat clearGray(work, CV_8UC1, cv::Scalar(60));
    cv::Mat clearBgr(work, CV_8UC3, cv::Scalar(60, 60, 60));
    cv::Mat mask(work, CV_8UC1, cv::Scalar(255));
    context.check(reference.applyCueAndVeto(slot, mask, clearGray, clearBgr, eval, roiNorm, 8),
        "ref-patch", "patchwork reference applies (anchors from the confirmed region)");
    const cv::Rect vetoRect(0, 0, clearWidth - 2 * clearance, work.height);
    const double confirmedFlagged = static_cast<double>(cv::countNonZero(mask(vetoRect))) / vetoRect.area();
    const double gapFlagged = static_cast<double>(cv::countNonZero(mask(cloudRect))) / cloudRect.area();
    context.check(confirmedFlagged < 0.1, "ref-patch",
        QStringLiteral("confirmed region vetoed (%1 % still flagged)").arg(confirmedFlagged * 100.0, 0, 'f', 1));
    context.check(gapFlagged > 0.9, "ref-patch",
        QStringLiteral("unconfirmed gap untouched (%1 % still flagged)").arg(gapFlagged * 100.0, 0, 'f', 1));

    // A verified-clear whole frame adopts the never-confirmed gaps outright
    const auto frameLearn = reference.autoLearn(slot, clearGray, clearBgr, texture, eval, roiNorm, start.addSecs(1400),
                                                1.0f, true, true, 30, 26, cv::Mat());
    context.check(frameLearn == CameraClearSkyReference::LearnResult::Frame,
        "ref-patch", "verified-clear frame learns whole-frame");
    cv::Mat adopted(work, CV_8UC1, cv::Scalar(255));
    context.check(reference.applyCueAndVeto(slot, adopted, clearGray, clearBgr, eval, roiNorm, 8)
            && (cv::countNonZero(adopted(cloudRect)) == 0),
        "ref-patch", "whole-frame learn adopts the never-confirmed gap outright");

    // Weights persist: a fresh instance reloading the store behaves identically
    CameraClearSkyReference reloaded;
    reloaded.ensureLoaded(QStringLiteral("ref-patch-test"));
    cv::Mat reloadedMask(work, CV_8UC1, cv::Scalar(255));
    context.check(reloaded.applyCueAndVeto(slot, reloadedMask, clearGray, clearBgr, eval, roiNorm, 8)
            && (cv::countNonZero(reloadedMask) == 0),
        "ref-patch", "patchwork weights survive a save/load round trip");
}

// The debug view is rendered once per recompute and reused on the intermediate frames
// that carry the cached mask. Every frame must still carry a debug image (a frame showing
// the live camera image between mask frames would flicker), and the image must follow the
// mask when classification re-runs.
void testDebugViewCadence(TestContext& context)
{
    CameraSettings settings = makeCloudSettings();
    settings.m_cloudMode = CameraSettings::CloudModeDay;
    settings.m_cloudDebugView = CameraSettings::CloudDebugViewFinal;
    settings.m_cloudUpdateIntervalFrames = 3;

    // Four clear frames, then four heavily clouded ones: the mask (and so the debug view)
    // must change once a recompute lands on the new sky
    QVector<QImage> images;
    for (int i = 0; i < 4; ++i) {
        images.append(makeDayImage(0.0));
    }
    for (int i = 0; i < 4; ++i) {
        images.append(makeDayImage(1.0));
    }

    const CloudRunResult result = runCloudDetector(settings, images);
    if (!result.completed(images.size()))
    {
        context.check(false, "debug-view", result.error);
        return;
    }

    bool everyFrameRendered = true;
    for (int i = 0; i < result.frames.size(); ++i)
    {
        const QImage& shown = result.frames[i]->m_image;
        everyFrameRendered = everyFrameRendered
            && !shown.isNull()
            && (shown.size() == images[i].size())
            && (shown != images[i]); // the mask render replaced the camera image
    }
    context.check(everyFrameRendered, "debug-view", "every frame carries a debug render");

    const QImage first = result.frames.first()->m_image;
    const QImage last = result.frames.last()->m_image;
    context.check(first != last, "debug-view", "the debug view follows the mask across recomputes");
}

// Learning is throttled per slot on ATTEMPTS, not only on successful learns: an empty slot
// has no update time to throttle against, so a slot that keeps failing its gates would
// otherwise have the detector rebuild the (expensive) confirmed-clear mask every recompute.
void testLearnAttemptThrottle(TestContext& context)
{
    using Ref = CameraClearSkyReference;
    CameraClearSkyReference reference;
    reference.ensureLoaded(QStringLiteral("learn-throttle-test"));

    const QRectF roiNorm(0.0, 0.0, 1.0, 1.0);
    const cv::Size work(160, 120);
    const cv::Mat eval(work, CV_8UC1, cv::Scalar(255));
    const cv::Mat texture = cv::Mat::zeros(work, CV_8UC1);
    const cv::Mat gray(work, CV_8UC1, cv::Scalar(150));
    const cv::Mat bgr(work, CV_8UC3, cv::Scalar(200, 150, 110));
    const QDateTime start(QDate(2024, 6, 1), QTime(12, 0), QTimeZone::utc());
    const int slot = Ref::slotFromDayBin(0);

    context.check(reference.learnDue(slot, roiNorm, start), "learn-throttle",
        "an empty slot is due for learning");

    // Coverage far above the day gate: the attempt is made and refused
    const auto refused = reference.autoLearn(slot, gray, bgr, texture, eval, roiNorm, start,
                                             50.0f, false, false, 0, 0, cv::Mat());
    context.check(refused == Ref::LearnResult::None, "learn-throttle", "a too-cloudy day frame is refused");

    context.check(!reference.learnDue(slot, roiNorm, start.addSecs(60)), "learn-throttle",
        "a refused attempt still starts the throttle");
    context.check(reference.learnDue(slot, roiNorm, start.addSecs(700)), "learn-throttle",
        "the slot is due again once the throttle lapses");
}

// Migration replicates one stored slot into several bins. Those replicas must own their
// pixel storage: cv::Mat assignment is a refcounted shallow copy and the update paths
// write into an existing destination buffer, so a shallow replica would have its pixels
// rewritten by a save into any sibling bin while keeping its own (stale) anchors.
void testReferenceMigrationIsolation(TestContext& context)
{
    using Ref = CameraClearSkyReference;
    const QRectF roiNorm(0.0, 0.0, 1.0, 1.0);
    const cv::Size work(160, 120);
    const cv::Mat eval(work, CV_8UC1, cv::Scalar(255));
    const cv::Mat texture = cv::Mat::zeros(work, CV_8UC1);
    const QDateTime noon(QDate(2024, 6, 1), QTime(12, 0), QTimeZone::utc());
    const auto makeGray = [&](int level) { return cv::Mat(work, CV_8UC1, cv::Scalar(level)); };
    const auto makeBgr = [&](int b, int g, int r) { return cv::Mat(work, CV_8UC3, cv::Scalar(b, g, r)); };
    // Patterned skies: the maps are stored normalised by their own anchor, so a uniform
    // sky would read ~1.0 whatever was written over it - only a spatial pattern reveals
    // one bin's pixels being overwritten by a save into another
    const auto patternGray = [&](bool inverted) {
        cv::Mat gray(work, CV_8UC1);
        gray.setTo(inverted ? 120 : 180);
        gray(cv::Rect(work.width / 2, 0, work.width / 2, work.height)).setTo(inverted ? 180 : 120);
        return gray;
    };
    const auto patternBgr = [&](bool inverted) {
        cv::Mat bgr(work, CV_8UC3);
        bgr.setTo(inverted ? cv::Scalar(160, 120, 90) : cv::Scalar(240, 180, 132));
        bgr(cv::Rect(work.width / 2, 0, work.width / 2, work.height))
            .setTo(inverted ? cv::Scalar(240, 180, 132) : cv::Scalar(160, 120, 90));
        return bgr;
    };

    // A store written with a single Day slot, then downgraded to the pre-CSR4 format so
    // reloading it exercises the day-bin replication path. CSR3 differs only in the magic
    // and in holding 19 slots instead of 23; the four extra trailing "invalid" flags a
    // CSR4 writer leaves behind are simply not read back, so flipping the magic is enough.
    const QString cameraId = QStringLiteral("ref-migration-test");
    {
        CameraClearSkyReference seed;
        seed.ensureLoaded(cameraId);
        seed.capture(Ref::slotFromDayBin(0), patternGray(false), patternBgr(false), texture, eval, roiNorm, noon, 30.0, -20.0);
    }
    const QString storePath = QDir(QString::fromLocal8Bit(qgetenv("SDRANGEL_CAMERA_CLEARSKY_DIR")))
        .filePath(QString::fromLatin1(QCryptographicHash::hash(cameraId.toUtf8(), QCryptographicHash::Sha1).toHex().left(16))
            + QStringLiteral(".csr"));
    {
        QFile store(storePath);
        if (!store.open(QIODevice::ReadWrite))
        {
            context.check(false, "ref-migration", QStringLiteral("cannot open the seeded store %1").arg(storePath));
            return;
        }
        QByteArray content = store.readAll();
        if (content.size() < 4)
        {
            context.check(false, "ref-migration", "seeded store is truncated");
            return;
        }
        content[3] = char(0x33); // "CSR4" -> "CSR3"
        store.seek(0);
        store.write(content);
    }

    CameraClearSkyReference reference;
    reference.ensureLoaded(cameraId);

    // Every day bin answers for the migrated reference: an identical frame is fully vetoed
    const auto vetoesIdenticalFrame = [&](int bin) {
        cv::Mat mask(work, CV_8UC1, cv::Scalar(255));
        const bool applied = reference.applyCueAndVeto(Ref::slotFromDayBin(bin), mask,
            patternGray(false), patternBgr(false), eval, roiNorm, 8);
        return applied && (cv::countNonZero(mask) == 0);
    };
    bool allBinsWork = true;
    for (int bin = 0; bin < Ref::kDayBins; ++bin) {
        allBinsWork = allBinsWork && vetoesIdenticalFrame(bin);
    }
    context.check(allBinsWork, "ref-migration", "migrated day reference serves every day bin");

    // Saving a DIFFERENT sky into one bin must not disturb its siblings
    reference.capture(Ref::slotFromDayBin(2), patternGray(true), patternBgr(true), texture, eval, roiNorm, noon, 7.0, -20.0);
    bool siblingsIntact = true;
    for (int bin = 0; bin < Ref::kDayBins; ++bin)
    {
        if (bin == 2) {
            continue;
        }
        siblingsIntact = siblingsIntact && vetoesIdenticalFrame(bin);
    }
    context.check(siblingsIntact, "ref-migration",
        "a save into one migrated bin leaves the other bins' maps intact");

    cv::Mat rewritten(work, CV_8UC1, cv::Scalar(255));
    context.check(reference.applyCueAndVeto(Ref::slotFromDayBin(2), rewritten, patternGray(true), patternBgr(true), eval, roiNorm, 8)
            && (cv::countNonZero(rewritten) == 0),
        "ref-migration", "the rewritten bin matches its own new reference");
}

// The whole-frame auto-learn blend must not pull in pixels the frame did not evaluate:
// the sun/moon glare disc, the rim margin and the learned foreground are excluded per
// frame and move, so blending them would walk glare into the reference over time.
void testAutoLearnSkipsUnevaluated(TestContext& context)
{
    using Ref = CameraClearSkyReference;
    CameraClearSkyReference reference;
    reference.ensureLoaded(QStringLiteral("ref-unevaluated-test"));

    const QRectF roiNorm(0.0, 0.0, 1.0, 1.0);
    const cv::Size work(160, 120);
    const cv::Mat fullEval(work, CV_8UC1, cv::Scalar(255));
    const cv::Mat texture = cv::Mat::zeros(work, CV_8UC1);
    const QDateTime start(QDate(2024, 6, 1), QTime(12, 0), QTimeZone::utc());
    const int slot = Ref::slotFromDayBin(0);

    const cv::Mat clearGray(work, CV_8UC1, cv::Scalar(150));
    const cv::Mat clearBgr(work, CV_8UC3, cv::Scalar(200, 150, 110));
    reference.capture(slot, clearGray, clearBgr, texture, fullEval, roiNorm, start, 30.0, -20.0);

    // A later verified-clear frame carrying a bright "glare" patch, with that patch
    // excluded from evaluation exactly as the sun/moon mask would exclude it
    const cv::Rect glare(0, 0, work.width / 4, work.height);
    cv::Mat glareGray = clearGray.clone();
    glareGray(glare).setTo(255);
    cv::Mat glareBgr = clearBgr.clone();
    glareBgr(glare).setTo(cv::Scalar(255, 255, 255));
    cv::Mat maskedEval = fullEval.clone();
    maskedEval(glare).setTo(0);

    for (int i = 0; i < 8; ++i)
    {
        reference.autoLearn(slot, glareGray, glareBgr, texture, maskedEval, roiNorm,
                            start.addSecs(700 * (i + 1)), 0.5f, false, false, 0, 0, cv::Mat());
    }

    // The reference must still match a genuinely clear sky under the (moved on) glare
    cv::Mat mask(work, CV_8UC1, cv::Scalar(255));
    const bool applied = reference.applyCueAndVeto(slot, mask, clearGray, clearBgr, fullEval, roiNorm, 8);
    const double flagged = static_cast<double>(cv::countNonZero(mask(glare))) / glare.area();
    context.check(applied, "ref-unevaluated", "reference still applies after learning past a glare patch");
    context.check(flagged < 0.05, "ref-unevaluated",
        QStringLiteral("clear sky under the excluded patch stays vetoed (%1 % flagged)").arg(flagged * 100.0, 0, 'f', 1));
}

// Day sun-elevation sub-bins: the slot mapping, the day-bin adjacency fallback, and the
// wall between day and night references (neither may ever serve the other)
void testReferenceDayBins(TestContext& context)
{
    using Ref = CameraClearSkyReference;
    context.check(Ref::slotFor(30.0, -10.0) == Ref::slotFromDayBin(0), "ref-daybins", "high sun maps to day bin 0 (slot 0)");
    context.check(Ref::slotFor(15.0, -10.0) == Ref::slotFromDayBin(1), "ref-daybins", "sun 15 maps to day bin 1");
    context.check(Ref::slotFor(6.0, 20.0) == Ref::slotFromDayBin(2), "ref-daybins", "sun 6 maps to day bin 2 (moon irrelevant by day)");
    context.check(Ref::slotFor(2.0, -10.0) == Ref::slotFromDayBin(3), "ref-daybins", "sun 2 maps to day bin 3");
    context.check(Ref::slotFor(-2.0, -10.0) == Ref::slotFromDayBin(4), "ref-daybins", "dusk maps to day bin 4");
    context.check(Ref::slotFor(-4.5, -10.0) == Ref::slotFromBin(0, false), "ref-daybins", "below -4 degrees the night bins begin");
    context.check(!Ref::slotIsNight(Ref::slotFromDayBin(4)), "ref-daybins", "late day bins are not night slots");

    CameraClearSkyReference reference;
    reference.ensureLoaded(QStringLiteral("ref-daybins-test"));
    const QRectF roiNorm(0.0, 0.0, 1.0, 1.0);
    const cv::Size work(160, 120);
    const cv::Mat eval(work, CV_8UC1, cv::Scalar(255));
    const cv::Mat texture = cv::Mat::zeros(work, CV_8UC1);
    const QDateTime when(QDate(2024, 6, 1), QTime(12, 0), QTimeZone::utc());
    const cv::Mat gray(work, CV_8UC1, cv::Scalar(150));
    const cv::Mat bgr(work, CV_8UC3, cv::Scalar(200, 150, 110));

    // Fill day bin 1: its neighbouring day bins reach it, night slots and distant day
    // bins do not
    reference.capture(Ref::slotFromDayBin(1), gray, bgr, texture, eval, roiNorm, when, 15.0, -20.0);
    cv::Mat viaNeighbour(work, CV_8UC1, cv::Scalar(255));
    context.check(reference.applyCueAndVeto(Ref::slotFromDayBin(0), viaNeighbour, gray, bgr, eval, roiNorm, 8)
            && (cv::countNonZero(viaNeighbour) == 0),
        "ref-daybins", "day fallback reaches the neighbouring day bin");
    cv::Mat nightMask(work, CV_8UC1, cv::Scalar(255));
    context.check(!reference.applyCueAndVeto(Ref::slotFromBin(0, false), nightMask, gray, bgr, eval, roiNorm, 8),
        "ref-daybins", "a day reference never serves a night slot");
    cv::Mat farMask(work, CV_8UC1, cv::Scalar(255));
    context.check(!reference.applyCueAndVeto(Ref::slotFromDayBin(3), farMask, gray, bgr, eval, roiNorm, 8),
        "ref-daybins", "day fallback spans one bin only");
}

// Unit-level guards on the clear-sky reference model: the Day reference must never be
// reached through a night slot's adjacency fallback, night adjacency must work within the
// same moon state, and the raw-deviation caps must keep a smooth gradient overcast (which
// the surface removal would otherwise absorb) from being vetoed away.
void testReferenceModelGuards(TestContext& context)
{
    CameraClearSkyReference reference;
    reference.ensureLoaded(QStringLiteral("ref-guards-test"));

    const QRectF roiNorm(0.0, 0.0, 1.0, 1.0);
    const cv::Size work(160, 120);
    const cv::Mat eval(work, CV_8UC1, cv::Scalar(255));
    const cv::Mat texture = cv::Mat::zeros(work, CV_8UC1);
    const QDateTime when(QDate(2024, 1, 1), QTime(0, 0), QTimeZone::utc());
    const auto makeGray = [&](int level) { return cv::Mat(work, CV_8UC1, cv::Scalar(level)); };
    const auto makeBgr = [&](int b, int g, int r) { return cv::Mat(work, CV_8UC3, cv::Scalar(b, g, r)); };

    // Day reference filled; a night frame whose own bin is empty must NOT fall back to it
    reference.capture(0, makeGray(180), makeBgr(200, 150, 110), texture, eval, roiNorm, when, 10.0, -20.0);
    cv::Mat mask(work, CV_8UC1, cv::Scalar(255));
    const int nightSlot = CameraClearSkyReference::slotFromBin(0, true);
    const bool dayApplied = reference.applyCueAndVeto(nightSlot, mask, makeGray(170), makeBgr(200, 150, 110), eval, roiNorm, 8);
    context.check(!dayApplied, "ref-guards", "night slot does not fall back to the Day reference");

    // Night adjacency: the neighbouring bin with the same moon state is reachable
    reference.capture(CameraClearSkyReference::slotFromBin(1, true), makeGray(100), makeBgr(180, 140, 100), texture, eval, roiNorm, when, -7.0, 20.0);
    cv::Mat neighbourMask(work, CV_8UC1, cv::Scalar(255));
    const bool neighbourApplied = reference.applyCueAndVeto(nightSlot, neighbourMask, makeGray(100), makeBgr(180, 140, 100), eval, roiNorm, 8);
    context.check(neighbourApplied, "ref-guards", "night fallback reaches the neighbouring bin (same moon state)");
    context.check(cv::countNonZero(neighbourMask) == 0, "ref-guards", "identical frame fully vetoed via the neighbour reference");

    // Gradient overcast: reference is clear blue; the frame's colour ramps to white across
    // the image. The surface removal absorbs the ramp, but the raw-deviation cap must stop
    // the veto erasing the in-frame detection on the whitened side.
    const int clearSlot = CameraClearSkyReference::slotFromBin(2, false);
    reference.capture(clearSlot, makeGray(150), makeBgr(180, 140, 100), texture, eval, roiNorm, when, -9.0, -20.0);
    cv::Mat overcastBgr(work, CV_8UC3);
    for (int row = 0; row < work.height; ++row)
    {
        cv::Vec3b *line = overcastBgr.ptr<cv::Vec3b>(row);
        for (int col = 0; col < work.width; ++col)
        {
            const int red = 100 + (90 * col) / work.width; // ratio ~0.55 -> ~1.05 left to right
            line[col] = cv::Vec3b(180, 140, static_cast<uchar>(red));
        }
    }
    cv::Mat overcastMask(work, CV_8UC1, cv::Scalar(255)); // in-frame paths flagged everything
    const bool overcastApplied = reference.applyCueAndVeto(clearSlot, overcastMask, makeGray(150), overcastBgr, eval, roiNorm, 8);
    context.check(overcastApplied, "ref-guards", "gradient-overcast comparison applies");
    const cv::Rect whitenedQuarter(3 * work.width / 4, 0, work.width / 4, work.height);
    const double survived = static_cast<double>(cv::countNonZero(overcastMask(whitenedQuarter))) / whitenedQuarter.area();
    context.check(survived > 0.6, "ref-guards",
        QStringLiteral("whitened side survives the veto (%1 % of quarter still flagged)").arg(survived * 100.0, 0, 'f', 0));
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

// Regression test for a fully-overcast pastel pre-dawn frame. Colour alone cannot detect
// the bluish-white part of the overcast: it is spectrally identical to a clear blue sky and
// the adaptive anchor lands on the bluest cloud. The structure vote closes the gap - lumpy
// pale cloud carries band-pass detections over 4-8 % of a region's interior where clear sky
// measures under 0.5 %, so the unflagged pastel banks are flipped to cloud region-wide and
// the whole overcast is detected.
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
    context.check(coverageInRange(frame, 85.0f, 100.0f), "full-overcast-pastel",
        coverageText(frame) + " expected roughly 95 % (fully overcast)");
    context.check(regionCloudFraction(frame, QRect(1000, 240, 800, 720), 20) > 0.9,
        "full-overcast-pastel", "bright pink cloud classified as cloud");
    context.check(regionCloudFraction(frame, QRect(109, 1185, 544, 527), 20) > 0.7,
        "full-overcast-pastel", "pale bluish-white cloud bank classified as cloud (structure vote)");
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

    // A threshold below the hysteresis band width must still allow Low: the band caps at
    // half the threshold so the transition sits at a reachable coverage (5 % -> 2.5 %)
    constexpr double lowThreshold = 5.0;
    tracker.reset();
    context.check(tracker.update(6.0, lowThreshold) == CameraCloudEventTracker::High,
        "cloud-events", "low threshold: startup above the threshold fires High");
    context.check(tracker.update(3.0, lowThreshold) == CameraCloudEventTracker::None,
        "cloud-events", "low threshold: no event inside the shrunk hysteresis band");
    context.check(tracker.update(2.0, lowThreshold) == CameraCloudEventTracker::Low,
        "cloud-events", "low threshold: Low is reachable below half the threshold");
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
        // With SDRANGEL_CLOUD_STAR_SENSE set, the night run also exercises star-visibility
        // sensing with the camera's real calibration and the file's modification time as the
        // capture time. Safe by construction on files whose mtime is not the capture time:
        // predicted positions then miss the real stars, nothing is "visible" and the veto
        // abstains.
        QDateTime nightCaptureTime;
        if (!qgetenv("SDRANGEL_CLOUD_STAR_SENSE").isEmpty())
        {
            nightSettings.m_cloudStarSense = true;
            nightSettings.m_plateSolveUseCaptureDateTime = true;
            nightSettings.m_fov = 159.0f;
            nightSettings.m_azimuth = 157.0f;
            nightSettings.m_elevation = 86.0f;
            nightSettings.m_roll = -129.0f;
            nightSettings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
            nightSettings.m_lensDistortionK1 = -0.0743f;
            nightSettings.m_lensCenterOffsetX = 6.3f;
            nightSettings.m_lensCenterOffsetY = -10.5f;
            nightSettings.m_latitude = 51.368801f;
            nightSettings.m_longitude = -0.121f;
            nightCaptureTime = QFileInfo(path).lastModified().toUTC();
        }
        const CloudRunResult nightRun = runCloudDetector(nightSettings, {image}, nullptr, false, nightCaptureTime);

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

        float maskedCov = -1.0f;
        if (base.startsWith(QStringLiteral("sun")))
        {
            // The two new geometric masks are opt-in and off in the default runs above.
            // Emit an extra "_masked" overlay for the sun* images with the fisheye edge-margin
            // rim mask enabled (removes the lens rim and the window frame at the image edge)
            // and the sun/moon mask configured with the camera's real lens calibration and
            // location. The capture time defaults to the image file's modification time (these
            // snapshots are written at capture), overridable via SDRANGEL_CLOUD_SUN_TIME
            // (ISO 8601 UTC); SDRANGEL_CLOUD_SUN_LAT / _LON / _RADIUS override the rest.
            CameraSettings maskedSettings = daySettings;
            maskedSettings.m_cloudEdgeMarginPercent = 4.0;
            maskedSettings.m_cloudMaskSunMoon = true;
            maskedSettings.m_cloudSunMoonRadiusDeg = 25.0; // max cap; the mask sizes itself to the bloom
            const QByteArray radiusEnv = qgetenv("SDRANGEL_CLOUD_SUN_RADIUS");
            if (!radiusEnv.isEmpty()) {
                maskedSettings.m_cloudSunMoonRadiusDeg = QString::fromUtf8(radiusEnv).toDouble();
            }
            maskedSettings.m_fov = 159.0f;
            maskedSettings.m_azimuth = 157.0f;
            maskedSettings.m_elevation = 86.0f;
            maskedSettings.m_roll = -129.0f;
            maskedSettings.m_lensProjection = CameraSettings::LensProjectionEquidistant;
            maskedSettings.m_lensDistortionK1 = -0.0743f;
            maskedSettings.m_lensCenterOffsetX = 6.3f;
            maskedSettings.m_lensCenterOffsetY = -10.5f;
            maskedSettings.m_latitude = 51.368801f;
            maskedSettings.m_longitude = -0.121f;
            maskedSettings.m_plateSolveUseCaptureDateTime = true;

            QDateTime captureTime = QFileInfo(path).lastModified().toUTC();
            const QByteArray timeEnv = qgetenv("SDRANGEL_CLOUD_SUN_TIME");
            const QByteArray latEnv = qgetenv("SDRANGEL_CLOUD_SUN_LAT");
            const QByteArray lonEnv = qgetenv("SDRANGEL_CLOUD_SUN_LON");
            if (!timeEnv.isEmpty())
            {
                captureTime = QDateTime::fromString(QString::fromUtf8(timeEnv), Qt::ISODate);
                captureTime.setTimeZone(QTimeZone::utc());
            }
            if (!latEnv.isEmpty()) {
                maskedSettings.m_latitude = QString::fromUtf8(latEnv).toDouble();
            }
            if (!lonEnv.isEmpty()) {
                maskedSettings.m_longitude = QString::fromUtf8(lonEnv).toDouble();
            }

            if (captureTime.isValid())
            {
                AzAlt sunAzAlt;
                RADec sunRaDec;
                Astronomy::sunPosition(sunAzAlt, sunRaDec, maskedSettings.m_latitude, maskedSettings.m_longitude, captureTime);
                const SkyProjector projector = SkyProjector::create(maskedSettings, image.size());
                QPointF sunPoint;
                const bool ok = projector.projectAltAz(sunAzAlt.az, sunAzAlt.alt, sunPoint);
                std::cout << "  " << file.toStdString() << " " << image.width() << "x" << image.height()
                          << " time " << captureTime.toString(Qt::ISODate).toStdString()
                          << " sun az=" << QString::number(sunAzAlt.az, 'f', 1).toStdString()
                          << " el=" << QString::number(sunAzAlt.alt, 'f', 1).toStdString()
                          << " -> pixel " << (ok ? QStringLiteral("(%1, %2)").arg(sunPoint.x(), 0, 'f', 0).arg(sunPoint.y(), 0, 'f', 0).toStdString() : std::string("off-frame"))
                          << "\n";
            }

            const CloudRunResult maskedRun = runCloudDetector(maskedSettings, {image}, nullptr, false, captureTime);
            if (maskedRun.completed(1) && maskedRun.frames.first()->m_cloud.m_valid)
            {
                maskedCov = maskedRun.frames.first()->m_cloud.m_coveragePercent;
                writeMaskOverlay(path, maskedRun.frames.first()->m_cloud, outDir + "/" + base + "_masked.png");
            }
        }

        std::cout << file.toStdString() << ","
                  << QString::number(nightCov, 'f', 1).toStdString() << ","
                  << nightPath << ","
                  << QString::number(dayCov, 'f', 1).toStdString();
        if (maskedCov >= 0.0f) {
            std::cout << " (masked day: " << QString::number(maskedCov, 'f', 1).toStdString() << ")";
        }
        std::cout << "\n";
    }
    std::cout << "overlays written to " << QDir(outDir).absolutePath().toStdString() << "\n";
    return 0;
}

} // namespace

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    // Isolate clear-sky reference persistence from the user's real store
    static QTemporaryDir clearSkyDir;
    if (clearSkyDir.isValid()) {
        qputenv("SDRANGEL_CAMERA_CLEARSKY_DIR", clearSkyDir.path().toLocal8Bit());
    }
    QGuiApplication::setOrganizationName(QStringLiteral("f4exb"));
    QGuiApplication::setApplicationName(QStringLiteral("SDRangel"));

    const QStringList args = app.arguments();

    if (args.contains(QStringLiteral("--run-case")))
    {
        const int idx = args.indexOf(QStringLiteral("--run-case"));
        if (idx + 1 >= args.size())
        {
            std::cout << "usage: --run-case <bundle directory>\n";
            return 1;
        }
        return runTestCaseBundle(args.at(idx + 1), args.contains(QStringLiteral("--debug-views")));
    }

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
        {"star-sense", testStarSense},
        {"star-blank", testStarBlankCue},
        {"min-elevation", testMinElevationMask},
        {"azimuth-wrap", testSkyProjectorAzimuthWrapping},
        {"sun-gate", testDayLearnSunGate},
        {"ref-darkday", testDarkDayCue},
        {"clear-sky-ref", testClearSkyReference},
        {"ref-guards", testReferenceModelGuards},
        {"ref-patch", testReferencePatchLearning},
        {"ref-daybins", testReferenceDayBins},
        {"debug-view", testDebugViewCadence},
        {"learn-throttle", testLearnAttemptThrottle},
        {"ref-migration", testReferenceMigrationIsolation},
        {"ref-unevaluated", testAutoLearnSkipsUnevaluated},
        {"test-case-bundle", testTestCaseBundle},
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
