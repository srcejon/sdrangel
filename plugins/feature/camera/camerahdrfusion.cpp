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

#include "camerahdrfusion.h"

#ifdef CAMERA_OPENCV_CUDA_STACKING

#include <algorithm>
#include <cmath>

#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

namespace
{
cv::cuda::GpuMat sizeMatchedPyramidExpansion(const cv::cuda::GpuMat& expandedGpu, const cv::Size& targetSize, cv::cuda::Stream& stream)
{
    if (expandedGpu.size() == targetSize) {
        return expandedGpu;
    }

    if ((expandedGpu.cols >= targetSize.width) && (expandedGpu.rows >= targetSize.height)) {
        return expandedGpu(cv::Rect(0, 0, targetSize.width, targetSize.height));
    }

    cv::cuda::GpuMat resizedGpu;
    cv::cuda::resize(expandedGpu, resizedGpu, targetSize, 0.0, 0.0, cv::INTER_LINEAR, stream);
    return resizedGpu;
}

void setError(QString *errorMessage, const QString& text)
{
    if (errorMessage) {
        *errorMessage = text;
    }
}
}

bool CameraHdrFusion::mergeMertensCudaRgb(
    const std::vector<cv::Mat>& rgbFrames,
    cv::Mat& tonemappedRgb,
    cv::cuda::Stream& stream,
    cv::Ptr<cv::cuda::Filter>& laplacianFilter,
    QString *errorMessage)
{
    tonemappedRgb.release();
    if (rgbFrames.empty())
    {
        setError(errorMessage, QStringLiteral("No HDR frames supplied"));
        return false;
    }

    cv::cuda::GpuMat tonemappedRgbGpu;
    if (!mergeMertensCudaRgb(rgbFrames, tonemappedRgbGpu, stream, laplacianFilter, errorMessage)) {
        return false;
    }

    tonemappedRgbGpu.download(tonemappedRgb, stream);
    stream.waitForCompletion();
    return !tonemappedRgb.empty() && (tonemappedRgb.type() == CV_32FC3);
}

bool CameraHdrFusion::mergeMertensCudaRgb(
    const std::vector<cv::Mat>& rgbFrames,
    cv::cuda::GpuMat& tonemappedRgbGpu,
    cv::cuda::Stream& stream,
    cv::Ptr<cv::cuda::Filter>& laplacianFilter,
    QString *errorMessage)
{
    tonemappedRgbGpu.release();
    if (rgbFrames.empty())
    {
        setError(errorMessage, QStringLiteral("No HDR frames supplied"));
        return false;
    }

    std::vector<cv::cuda::GpuMat> rgbFramesGpu;
    rgbFramesGpu.reserve(rgbFrames.size());
    for (const cv::Mat& frame : rgbFrames)
    {
        if (frame.empty())
        {
            setError(errorMessage, QStringLiteral("HDR frames must be non-empty same-size RGB images"));
            return false;
        }

        cv::cuda::GpuMat uploadedGpu;
        uploadedGpu.upload(frame, stream);
        rgbFramesGpu.push_back(uploadedGpu);
    }

    return mergeMertensCudaRgb(rgbFramesGpu, tonemappedRgbGpu, stream, laplacianFilter, errorMessage);
}

bool CameraHdrFusion::mergeMertensCudaRgb(
    const std::vector<cv::cuda::GpuMat>& rgbFramesGpu,
    cv::Mat& tonemappedRgb,
    cv::cuda::Stream& stream,
    cv::Ptr<cv::cuda::Filter>& laplacianFilter,
    QString *errorMessage)
{
    tonemappedRgb.release();
    cv::cuda::GpuMat tonemappedRgbGpu;
    if (!mergeMertensCudaRgb(rgbFramesGpu, tonemappedRgbGpu, stream, laplacianFilter, errorMessage)) {
        return false;
    }

    tonemappedRgbGpu.download(tonemappedRgb, stream);
    stream.waitForCompletion();
    return !tonemappedRgb.empty() && (tonemappedRgb.type() == CV_32FC3);
}

bool CameraHdrFusion::mergeMertensCudaRgb(
    const std::vector<cv::cuda::GpuMat>& rgbFramesGpu,
    cv::cuda::GpuMat& tonemappedRgbGpu,
    cv::cuda::Stream& stream,
    cv::Ptr<cv::cuda::Filter>& laplacianFilter,
    QString *errorMessage)
{
    tonemappedRgbGpu.release();
    if (rgbFramesGpu.empty())
    {
        setError(errorMessage, QStringLiteral("No HDR frames supplied"));
        return false;
    }

    try
    {
        const cv::Size frameSize = rgbFramesGpu.front().size();
        std::vector<cv::cuda::GpuMat> frameFloatGpu;
        frameFloatGpu.reserve(rgbFramesGpu.size());

        if (!laplacianFilter) {
            laplacianFilter = cv::cuda::createLaplacianFilter(CV_32FC1, CV_32FC1, 1);
        }

        std::vector<cv::Size> pyramidSizes;
        pyramidSizes.push_back(frameSize);
        const int maxLevel = static_cast<int>(std::log(static_cast<double>(std::min(frameSize.width, frameSize.height))) / std::log(2.0));
        while (static_cast<int>(pyramidSizes.size()) <= maxLevel)
        {
            const cv::Size previousSize = pyramidSizes.back();
            const cv::Size nextSize(std::max(1, (previousSize.width + 1) / 2), std::max(1, (previousSize.height + 1) / 2));
            if (nextSize == previousSize) {
                break;
            }
            pyramidSizes.push_back(nextSize);
        }
        const int pyramidLevels = static_cast<int>(pyramidSizes.size());

        std::vector<std::vector<cv::cuda::GpuMat>> weightPyramids(rgbFramesGpu.size());
        std::vector<cv::cuda::GpuMat> weights(rgbFramesGpu.size());
        cv::cuda::GpuMat weightSum(frameSize, CV_32FC1, cv::Scalar::all(0.0));

        for (size_t sampleIndex = 0; sampleIndex < rgbFramesGpu.size(); ++sampleIndex)
        {
            const cv::cuda::GpuMat& uploadedGpu = rgbFramesGpu[sampleIndex];
            if (uploadedGpu.empty()
                || (uploadedGpu.size() != frameSize)
                || (uploadedGpu.channels() != 3))
            {
                setError(errorMessage, QStringLiteral("HDR frames must be non-empty same-size RGB images"));
                return false;
            }

            cv::cuda::GpuMat floatGpu;
            if (uploadedGpu.depth() == CV_16U) {
                uploadedGpu.convertTo(floatGpu, CV_32FC3, 1.0 / 65535.0, 0.0, stream);
            } else if (uploadedGpu.depth() == CV_8U) {
                uploadedGpu.convertTo(floatGpu, CV_32FC3, 1.0 / 255.0, 0.0, stream);
            } else {
                uploadedGpu.convertTo(floatGpu, CV_32FC3, 1.0, 0.0, stream);
            }

            std::vector<cv::cuda::GpuMat> channels;
            cv::cuda::split(floatGpu, channels, stream);
            if (channels.size() != 3)
            {
                setError(errorMessage, QStringLiteral("CUDA split did not produce three RGB channels"));
                return false;
            }

            cv::cuda::GpuMat grayGpu;
            cv::cuda::cvtColor(floatGpu, grayGpu, cv::COLOR_RGB2GRAY, 0, stream);

            cv::cuda::GpuMat contrastGpu;
            laplacianFilter->apply(grayGpu, contrastGpu, stream);
            cv::cuda::abs(contrastGpu, contrastGpu, stream);

            cv::cuda::GpuMat meanGpu;
            cv::cuda::add(channels[0], channels[1], meanGpu, cv::noArray(), -1, stream);
            cv::cuda::add(meanGpu, channels[2], meanGpu, cv::noArray(), -1, stream);
            meanGpu.convertTo(meanGpu, CV_32FC1, 1.0 / 3.0, 0.0, stream);

            cv::cuda::GpuMat saturationGpu(frameSize, CV_32FC1, cv::Scalar::all(0.0));
            cv::cuda::GpuMat tempGpu;
            for (const cv::cuda::GpuMat& channel : channels)
            {
                cv::cuda::subtract(channel, meanGpu, tempGpu, cv::noArray(), -1, stream);
                cv::cuda::multiply(tempGpu, tempGpu, tempGpu, 1.0, -1, stream);
                cv::cuda::add(saturationGpu, tempGpu, saturationGpu, cv::noArray(), -1, stream);
            }
            cv::cuda::sqrt(saturationGpu, saturationGpu, stream);

            cv::cuda::GpuMat weight;
            cv::cuda::multiply(contrastGpu, saturationGpu, weight, 1.0, -1, stream);
            cv::cuda::add(weight, cv::Scalar::all(1.0e-12), weight, cv::noArray(), -1, stream);

            frameFloatGpu.push_back(floatGpu);
            weights[sampleIndex] = weight;
            cv::cuda::add(weightSum, weight, weightSum, cv::noArray(), -1, stream);
        }

        for (size_t sampleIndex = 0; sampleIndex < weights.size(); ++sampleIndex)
        {
            weightPyramids[sampleIndex].resize(pyramidLevels);
            cv::cuda::divide(weights[sampleIndex], weightSum, weightPyramids[sampleIndex][0], 1.0, -1, stream);
            for (int level = 1; level < pyramidLevels; ++level) {
                cv::cuda::pyrDown(weightPyramids[sampleIndex][level - 1], weightPyramids[sampleIndex][level], stream);
            }
        }

        std::vector<cv::cuda::GpuMat> fusedPyramid;
        fusedPyramid.reserve(pyramidLevels);
        for (const cv::Size& pyramidSize : pyramidSizes) {
            fusedPyramid.emplace_back(pyramidSize, CV_32FC3, cv::Scalar::all(0.0));
        }

        for (size_t frameIndex = 0; frameIndex < frameFloatGpu.size(); ++frameIndex)
        {
            std::vector<cv::cuda::GpuMat> imageGaussianPyramid(pyramidLevels);
            imageGaussianPyramid[0] = frameFloatGpu[frameIndex];
            for (int level = 1; level < pyramidLevels; ++level)
            {
                cv::cuda::pyrDown(
                    imageGaussianPyramid[static_cast<size_t>(level - 1)],
                    imageGaussianPyramid[static_cast<size_t>(level)],
                    stream);
            }

            for (int level = 0; level < pyramidLevels; ++level)
            {
                cv::cuda::GpuMat laplacianGpu;
                if (level == pyramidLevels - 1)
                {
                    laplacianGpu = imageGaussianPyramid[static_cast<size_t>(level)];
                }
                else
                {
                    cv::cuda::GpuMat expandedGpu;
                    cv::cuda::pyrUp(
                        imageGaussianPyramid[static_cast<size_t>(level + 1)],
                        expandedGpu,
                        stream);
                    expandedGpu = sizeMatchedPyramidExpansion(expandedGpu, pyramidSizes[static_cast<size_t>(level)], stream);
                    cv::cuda::subtract(
                        imageGaussianPyramid[static_cast<size_t>(level)],
                        expandedGpu,
                        laplacianGpu,
                        cv::noArray(),
                        -1,
                        stream);
                }

                std::vector<cv::cuda::GpuMat> laplacianChannels;
                cv::cuda::split(laplacianGpu, laplacianChannels, stream);
                for (cv::cuda::GpuMat& channel : laplacianChannels) {
                    cv::cuda::multiply(channel, weightPyramids[frameIndex][static_cast<size_t>(level)], channel, 1.0, -1, stream);
                }
                cv::cuda::GpuMat weightedLaplacian;
                cv::cuda::merge(laplacianChannels, weightedLaplacian, stream);
                cv::cuda::add(
                    fusedPyramid[static_cast<size_t>(level)],
                    weightedLaplacian,
                    fusedPyramid[static_cast<size_t>(level)],
                    cv::noArray(),
                    -1,
                    stream);
            }
        }

        cv::cuda::GpuMat outputGpu = fusedPyramid.back();
        for (int level = pyramidLevels - 2; level >= 0; --level)
        {
            cv::cuda::GpuMat expandedGpu;
            cv::cuda::pyrUp(outputGpu, expandedGpu, stream);
            outputGpu = sizeMatchedPyramidExpansion(expandedGpu, pyramidSizes[static_cast<size_t>(level)], stream);
            cv::cuda::add(outputGpu, fusedPyramid[static_cast<size_t>(level)], outputGpu, cv::noArray(), -1, stream);
        }

        outputGpu.copyTo(tonemappedRgbGpu, stream);
        return !tonemappedRgbGpu.empty() && (tonemappedRgbGpu.type() == CV_32FC3);
    }
    catch (const cv::Exception& error)
    {
        setError(errorMessage, QString::fromLocal8Bit(error.what()));
    }

    return false;
}

bool CameraHdrFusion::mergeDebevecCudaRgb(
    const std::vector<cv::Mat>& rgbFrames,
    const std::vector<float>& exposureTimesSeconds,
    cv::cuda::GpuMat& tonemappedRgbGpu,
    cv::cuda::Stream& stream,
    QString *errorMessage)
{
    tonemappedRgbGpu.release();
    if (rgbFrames.empty())
    {
        setError(errorMessage, QStringLiteral("No HDR frames supplied"));
        return false;
    }

    std::vector<cv::cuda::GpuMat> rgbFramesGpu;
    rgbFramesGpu.reserve(rgbFrames.size());
    for (const cv::Mat& frame : rgbFrames)
    {
        if (frame.empty())
        {
            setError(errorMessage, QStringLiteral("HDR frames must be non-empty same-size RGB images"));
            return false;
        }

        cv::cuda::GpuMat uploadedGpu;
        uploadedGpu.upload(frame, stream);
        rgbFramesGpu.push_back(uploadedGpu);
    }

    return mergeDebevecCudaRgb(rgbFramesGpu, exposureTimesSeconds, tonemappedRgbGpu, stream, errorMessage);
}

bool CameraHdrFusion::mergeDebevecCudaRgb(
    const std::vector<cv::cuda::GpuMat>& rgbFramesGpu,
    const std::vector<float>& exposureTimesSeconds,
    cv::cuda::GpuMat& tonemappedRgbGpu,
    cv::cuda::Stream& stream,
    QString *errorMessage)
{
    tonemappedRgbGpu.release();
    if (rgbFramesGpu.empty())
    {
        setError(errorMessage, QStringLiteral("No HDR frames supplied"));
        return false;
    }
    if (rgbFramesGpu.size() != exposureTimesSeconds.size())
    {
        setError(errorMessage, QStringLiteral("HDR frame/exposure count mismatch"));
        return false;
    }

    try
    {
        const cv::Size frameSize = rgbFramesGpu.front().size();
        cv::cuda::GpuMat numerator(frameSize, CV_32FC3, cv::Scalar::all(0.0));
        cv::cuda::GpuMat denominator(frameSize, CV_32FC3, cv::Scalar::all(1.0e-6));

        std::vector<float> sortedExposureTimes = exposureTimesSeconds;
        std::sort(sortedExposureTimes.begin(), sortedExposureTimes.end());
        const float referenceExposure = std::max(1.0e-6f, sortedExposureTimes[sortedExposureTimes.size() / 2]);

        for (size_t sampleIndex = 0; sampleIndex < rgbFramesGpu.size(); ++sampleIndex)
        {
            const cv::cuda::GpuMat& uploadedGpu = rgbFramesGpu[sampleIndex];
            if (uploadedGpu.empty()
                || (uploadedGpu.size() != frameSize)
                || (uploadedGpu.channels() != 3))
            {
                setError(errorMessage, QStringLiteral("HDR frames must be non-empty same-size RGB images"));
                return false;
            }

            cv::cuda::GpuMat floatGpu;
            if (uploadedGpu.depth() == CV_16U) {
                uploadedGpu.convertTo(floatGpu, CV_32FC3, 1.0 / 65535.0, 0.0, stream);
            } else if (uploadedGpu.depth() == CV_8U) {
                uploadedGpu.convertTo(floatGpu, CV_32FC3, 1.0 / 255.0, 0.0, stream);
            } else {
                uploadedGpu.convertTo(floatGpu, CV_32FC3, 1.0, 0.0, stream);
            }

            cv::cuda::GpuMat weightGpu;
            cv::cuda::subtract(floatGpu, cv::Scalar::all(0.5), weightGpu, cv::noArray(), -1, stream);
            cv::cuda::abs(weightGpu, weightGpu, stream);
            weightGpu.convertTo(weightGpu, CV_32FC3, -2.0, 1.0, stream);
            cv::cuda::max(weightGpu, cv::Scalar::all(1.0e-4), weightGpu, stream);

            cv::cuda::GpuMat weightedRadianceGpu;
            cv::cuda::multiply(floatGpu, weightGpu, weightedRadianceGpu, 1.0, -1, stream);
            weightedRadianceGpu.convertTo(
                weightedRadianceGpu,
                CV_32FC3,
                1.0 / std::max(1.0e-6f, exposureTimesSeconds[sampleIndex]),
                0.0,
                stream);

            cv::cuda::add(numerator, weightedRadianceGpu, numerator, cv::noArray(), -1, stream);
            cv::cuda::add(denominator, weightGpu, denominator, cv::noArray(), -1, stream);
        }

        cv::cuda::GpuMat radianceGpu;
        cv::cuda::divide(numerator, denominator, radianceGpu, 1.0, -1, stream);
        radianceGpu.convertTo(radianceGpu, CV_32FC3, referenceExposure, 0.0, stream);

        cv::cuda::GpuMat denominatorToneGpu;
        cv::cuda::add(radianceGpu, cv::Scalar::all(1.0), denominatorToneGpu, cv::noArray(), -1, stream);
        cv::cuda::divide(radianceGpu, denominatorToneGpu, tonemappedRgbGpu, 1.0, -1, stream);
        cv::cuda::pow(tonemappedRgbGpu, 1.0 / 2.2, tonemappedRgbGpu, stream);
        return !tonemappedRgbGpu.empty() && (tonemappedRgbGpu.type() == CV_32FC3);
    }
    catch (const cv::Exception& error)
    {
        setError(errorMessage, QString::fromLocal8Bit(error.what()));
    }

    return false;
}
#endif
