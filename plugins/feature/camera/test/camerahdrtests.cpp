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
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QString>

#include <opencv2/core/cuda.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

#include "camerahdrfusion.h"

#ifndef CAMERA_HDR_TEST_DATA_DIR
#define CAMERA_HDR_TEST_DATA_DIR "."
#endif

namespace
{
struct DifferenceMetrics
{
    double m_meanAbs = 0.0;
    double m_rmse = 0.0;
    double m_p99Abs = 0.0;
    int m_maxAbs = 0;
    double m_badFraction = 0.0;
};

struct TimingStats
{
    double m_minMs = 0.0;
    double m_avgMs = 0.0;
};

TimingStats measureTiming(int iterations, const std::function<bool()>& fn)
{
    TimingStats stats;
    if (iterations <= 0) {
        return stats;
    }

    double totalMs = 0.0;
    stats.m_minMs = std::numeric_limits<double>::max();

    for (int i = 0; i < iterations; ++i)
    {
        const auto start = std::chrono::steady_clock::now();
        if (!fn())
        {
            stats.m_minMs = 0.0;
            stats.m_avgMs = 0.0;
            return stats;
        }
        const auto end = std::chrono::steady_clock::now();
        const double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
        totalMs += elapsedMs;
        stats.m_minMs = std::min(stats.m_minMs, elapsedMs);
    }

    stats.m_avgMs = totalMs / iterations;
    return stats;
}

cv::Mat clampFloat01(const cv::Mat& input)
{
    cv::Mat output = input.clone();
    cv::patchNaNs(output, 0.0);
    cv::max(output, 0.0, output);
    cv::min(output, 1.0, output);
    return output;
}

cv::Mat floatRgbTo8u(const cv::Mat& input)
{
    cv::Mat clamped = clampFloat01(input);
    cv::Mat output;
    clamped.convertTo(output, CV_8UC3, 255.0);
    return output;
}

DifferenceMetrics compareImages(const cv::Mat& a, const cv::Mat& b)
{
    DifferenceMetrics metrics;
    cv::Mat diff;
    cv::absdiff(a, b, diff);

    std::vector<int> values;
    values.reserve(static_cast<size_t>(diff.total() * diff.channels()));
    long long sum = 0;
    long long sumSquares = 0;
    int bad = 0;
    for (int y = 0; y < diff.rows; ++y)
    {
        const cv::Vec3b *line = diff.ptr<cv::Vec3b>(y);
        for (int x = 0; x < diff.cols; ++x)
        {
            for (int c = 0; c < 3; ++c)
            {
                const int value = line[x][c];
                values.push_back(value);
                sum += value;
                sumSquares += value * value;
                metrics.m_maxAbs = std::max(metrics.m_maxAbs, value);
                if (value > 25) {
                    ++bad;
                }
            }
        }
    }

    if (!values.empty())
    {
        std::sort(values.begin(), values.end());
        const size_t p99Index = std::min(values.size() - 1, static_cast<size_t>(std::floor(values.size() * 0.99)));
        metrics.m_meanAbs = static_cast<double>(sum) / static_cast<double>(values.size());
        metrics.m_rmse = std::sqrt(static_cast<double>(sumSquares) / static_cast<double>(values.size()));
        metrics.m_p99Abs = values[p99Index];
        metrics.m_badFraction = static_cast<double>(bad) / static_cast<double>(values.size());
    }
    return metrics;
}

bool writeRgbImage(const QString& fileName, const cv::Mat& rgb)
{
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return cv::imwrite(fileName.toStdString(), bgr);
}

bool runHdrComparison(const QString& dataDir, const QString& outputDir)
{
    const int timingIterations = 5;
    const QString hdr1Name = QDir(dataDir).filePath(QStringLiteral("images/hdr1.jpg"));
    const QString hdr2Name = QDir(dataDir).filePath(QStringLiteral("images/hdr2.jpg"));

    cv::Mat hdr1Bgr = cv::imread(hdr1Name.toStdString(), cv::IMREAD_COLOR);
    cv::Mat hdr2Bgr = cv::imread(hdr2Name.toStdString(), cv::IMREAD_COLOR);
    if (hdr1Bgr.empty() || hdr2Bgr.empty())
    {
        std::cerr << "FAIL: cannot read HDR test images: "
                  << hdr1Name.toStdString() << " "
                  << hdr2Name.toStdString() << "\n";
        return false;
    }
    if (hdr1Bgr.size() != hdr2Bgr.size())
    {
        std::cerr << "FAIL: HDR test images have different sizes\n";
        return false;
    }
    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        std::cerr << "FAIL: OpenCV CUDA device is not available\n";
        return false;
    }

    std::vector<cv::Mat> bgrFrames = {hdr1Bgr, hdr2Bgr};
    cv::Mat cpuBgr;
    cv::Ptr<cv::MergeMertens> mergeMertens = cv::createMergeMertens();
    mergeMertens->process(bgrFrames, cpuBgr);
    const TimingStats cpuTiming = measureTiming(timingIterations, [&]() {
        cv::Mat timedCpuBgr;
        mergeMertens->process(bgrFrames, timedCpuBgr);
        return !timedCpuBgr.empty();
    });
    cv::Mat cpuRgb;
    cv::cvtColor(cpuBgr, cpuRgb, cv::COLOR_BGR2RGB);

    cv::Mat hdr1Rgb;
    cv::Mat hdr2Rgb;
    cv::cvtColor(hdr1Bgr, hdr1Rgb, cv::COLOR_BGR2RGB);
    cv::cvtColor(hdr2Bgr, hdr2Rgb, cv::COLOR_BGR2RGB);
    std::vector<cv::Mat> rgbFrames = {hdr1Rgb, hdr2Rgb};

    cv::cuda::Stream stream;
    cv::Ptr<cv::cuda::Filter> laplacianFilter;
    cv::Mat cudaRgb;
    QString errorMessage;
    if (!CameraHdrFusion::mergeMertensCudaRgb(rgbFrames, cudaRgb, stream, laplacianFilter, &errorMessage))
    {
        std::cerr << "FAIL: CUDA Mertens fusion failed: " << errorMessage.toStdString() << "\n";
        return false;
    }
    const TimingStats cudaTiming = measureTiming(timingIterations, [&]() {
        cv::Mat timedCudaRgb;
        QString timedErrorMessage;
        return CameraHdrFusion::mergeMertensCudaRgb(rgbFrames, timedCudaRgb, stream, laplacianFilter, &timedErrorMessage)
            && !timedCudaRgb.empty();
    });
    std::vector<cv::cuda::GpuMat> rgbFramesGpu;
    rgbFramesGpu.reserve(rgbFrames.size());
    for (const cv::Mat& frame : rgbFrames)
    {
        cv::cuda::GpuMat frameGpu;
        frameGpu.upload(frame, stream);
        rgbFramesGpu.push_back(frameGpu);
    }
    cv::Mat preUploadedCudaRgb;
    QString preUploadedErrorMessage;
    if (!CameraHdrFusion::mergeMertensCudaRgb(rgbFramesGpu, preUploadedCudaRgb, stream, laplacianFilter, &preUploadedErrorMessage))
    {
        std::cerr << "FAIL: CUDA Mertens fusion with pre-uploaded frames failed: "
                  << preUploadedErrorMessage.toStdString() << "\n";
        return false;
    }
    const TimingStats cudaGpuInputTiming = measureTiming(timingIterations, [&]() {
        cv::Mat timedCudaRgb;
        QString timedErrorMessage;
        return CameraHdrFusion::mergeMertensCudaRgb(rgbFramesGpu, timedCudaRgb, stream, laplacianFilter, &timedErrorMessage)
            && !timedCudaRgb.empty();
    });

    const cv::Mat cpu8u = floatRgbTo8u(cpuRgb);
    const cv::Mat cuda8u = floatRgbTo8u(cudaRgb);
    const DifferenceMetrics metrics = compareImages(cpu8u, cuda8u);

    QDir().mkpath(outputDir);
    writeRgbImage(QDir(outputDir).filePath(QStringLiteral("hdr-cpu.jpg")), cpu8u);
    writeRgbImage(QDir(outputDir).filePath(QStringLiteral("hdr-cuda.jpg")), cuda8u);

    cv::Mat diff;
    cv::absdiff(cpu8u, cuda8u, diff);
    writeRgbImage(QDir(outputDir).filePath(QStringLiteral("hdr-diff.jpg")), diff);

    std::cout << "HDR CPU/CUDA comparison"
              << " meanAbs=" << metrics.m_meanAbs
              << " rmse=" << metrics.m_rmse
              << " p99Abs=" << metrics.m_p99Abs
              << " maxAbs=" << metrics.m_maxAbs
              << " badFraction=" << metrics.m_badFraction
              << " cpuMinMs=" << cpuTiming.m_minMs
              << " cpuAvgMs=" << cpuTiming.m_avgMs
              << " cudaMinMs=" << cudaTiming.m_minMs
              << " cudaAvgMs=" << cudaTiming.m_avgMs
              << " cudaGpuInputMinMs=" << cudaGpuInputTiming.m_minMs
              << " cudaGpuInputAvgMs=" << cudaGpuInputTiming.m_avgMs
              << " speedupMin=" << (cudaTiming.m_minMs > 0.0 ? cpuTiming.m_minMs / cudaTiming.m_minMs : 0.0)
              << " speedupAvg=" << (cudaTiming.m_avgMs > 0.0 ? cpuTiming.m_avgMs / cudaTiming.m_avgMs : 0.0)
              << " speedupGpuInputMin=" << (cudaGpuInputTiming.m_minMs > 0.0 ? cpuTiming.m_minMs / cudaGpuInputTiming.m_minMs : 0.0)
              << " speedupGpuInputAvg=" << (cudaGpuInputTiming.m_avgMs > 0.0 ? cpuTiming.m_avgMs / cudaGpuInputTiming.m_avgMs : 0.0)
              << " iterations=" << timingIterations
              << " outputDir=" << outputDir.toStdString()
              << "\n";

    if ((metrics.m_meanAbs > 3.0) || (metrics.m_rmse > 7.0) || (metrics.m_p99Abs > 30.0) || (metrics.m_badFraction > 0.02))
    {
        std::cerr << "FAIL: CUDA HDR output differs too much from CPU output\n";
        return false;
    }

    const std::vector<float> exposureTimesSeconds = {0.0125f, 0.05f};
    cv::Mat exposureTimesMat(static_cast<int>(exposureTimesSeconds.size()), 1, CV_32F, const_cast<float*>(exposureTimesSeconds.data()));
    auto runCpuDebevec = [&]() -> cv::Mat {
        cv::Mat hdrRadiance;
        cv::Mat tonemappedBgr;
        cv::Mat tonemappedRgb;
        cv::Ptr<cv::MergeDebevec> mergeDebevec = cv::createMergeDebevec();
        mergeDebevec->process(bgrFrames, hdrRadiance, exposureTimesMat);
        cv::Ptr<cv::TonemapReinhard> tonemap = cv::createTonemapReinhard(2.2f, 0.0f, 1.0f, 0.0f);
        tonemap->process(hdrRadiance, tonemappedBgr);
        cv::cvtColor(tonemappedBgr, tonemappedRgb, cv::COLOR_BGR2RGB);
        return tonemappedRgb;
    };

    cv::Mat cpuDebevecRgb = runCpuDebevec();
    const TimingStats cpuDebevecTiming = measureTiming(timingIterations, [&]() {
        cv::Mat timedCpuDebevecRgb = runCpuDebevec();
        return !timedCpuDebevecRgb.empty();
    });

    cv::cuda::GpuMat cudaDebevecGpu;
    QString debevecErrorMessage;
    if (!CameraHdrFusion::mergeDebevecCudaRgb(rgbFramesGpu, exposureTimesSeconds, cudaDebevecGpu, stream, &debevecErrorMessage))
    {
        std::cerr << "FAIL: CUDA Debevec fusion failed: " << debevecErrorMessage.toStdString() << "\n";
        return false;
    }
    cv::Mat cudaDebevecRgb;
    cudaDebevecGpu.download(cudaDebevecRgb, stream);
    stream.waitForCompletion();
    const TimingStats cudaDebevecTiming = measureTiming(timingIterations, [&]() {
        cv::cuda::GpuMat timedCudaDebevecGpu;
        QString timedDebevecErrorMessage;
        return CameraHdrFusion::mergeDebevecCudaRgb(rgbFramesGpu, exposureTimesSeconds, timedCudaDebevecGpu, stream, &timedDebevecErrorMessage)
            && !timedCudaDebevecGpu.empty();
    });

    const cv::Mat cpuDebevec8u = floatRgbTo8u(cpuDebevecRgb);
    const cv::Mat cudaDebevec8u = floatRgbTo8u(cudaDebevecRgb);
    const DifferenceMetrics debevecMetrics = compareImages(cpuDebevec8u, cudaDebevec8u);
    writeRgbImage(QDir(outputDir).filePath(QStringLiteral("hdr-debevec-cpu.jpg")), cpuDebevec8u);
    writeRgbImage(QDir(outputDir).filePath(QStringLiteral("hdr-debevec-cuda.jpg")), cudaDebevec8u);

    std::cout << "Debevec CPU/CUDA diagnostic"
              << " meanAbs=" << debevecMetrics.m_meanAbs
              << " rmse=" << debevecMetrics.m_rmse
              << " p99Abs=" << debevecMetrics.m_p99Abs
              << " maxAbs=" << debevecMetrics.m_maxAbs
              << " badFraction=" << debevecMetrics.m_badFraction
              << " cpuMinMs=" << cpuDebevecTiming.m_minMs
              << " cpuAvgMs=" << cpuDebevecTiming.m_avgMs
              << " cudaGpuInputMinMs=" << cudaDebevecTiming.m_minMs
              << " cudaGpuInputAvgMs=" << cudaDebevecTiming.m_avgMs
              << " speedupGpuInputMin=" << (cudaDebevecTiming.m_minMs > 0.0 ? cpuDebevecTiming.m_minMs / cudaDebevecTiming.m_minMs : 0.0)
              << " speedupGpuInputAvg=" << (cudaDebevecTiming.m_avgMs > 0.0 ? cpuDebevecTiming.m_avgMs / cudaDebevecTiming.m_avgMs : 0.0)
              << " iterations=" << timingIterations
              << "\n";

    std::cout << "PASS: CUDA HDR output is close to CPU output\n";
    return true;
}
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    const QString dataDir = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QStringLiteral(CAMERA_HDR_TEST_DATA_DIR);
    const QString outputDir = argc > 2
        ? QString::fromLocal8Bit(argv[2])
        : QDir(QStringLiteral(CAMERA_HDR_TEST_DATA_DIR)).filePath(QStringLiteral("hdr-test-output"));
    return runHdrComparison(dataDir, outputDir) ? 0 : 1;
}
