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

#include <cstring>

#include <QColor>
#include <QThreadStorage>

#include <opencv2/imgproc.hpp>

#include "cameraimageutils.h"
#include "cameraimagepool.h"

namespace {

// Recycle the full-resolution QImage backing buffers produced by the two central
// cv::Mat -> QImage bridges below. These run once per frame per pipeline stage
// (preprocessor, aligner, stacker, image processor, every detector) and are the
// dominant per-frame host allocation on the always-on path, so reusing the
// buffers is the biggest single cut to the per-frame page-fault churn that hurts
// slower machines. The pool is per-thread (QThreadStorage): each pipeline stage
// runs on its own thread, so every thread gets a contention-free pool that
// naturally holds that stage's frame size/format, and the pool is destroyed when
// that QThread exits. The produced image is often released on another thread
// (GUI/worker) — CameraImagePool's intrusive-refcounted state makes that
// cross-thread release (and thread exit while an image is still in flight) safe.
QImage acquirePooledImage(int width, int height, QImage::Format format)
{
    static QThreadStorage<CameraImagePool> pool;
    QImage image = pool.localData().acquire(width, height, format);
    if (image.isNull()) {
        image = QImage(width, height, format);
    }
    return image;
}

}

const QImage& CameraImageUtils::ensureRgb888(const QImage& image, QImage& convertedImage)
{
    if (image.format() == QImage::Format_RGB888) {
        return image;
    }

    convertedImage = image.convertToFormat(QImage::Format_RGB888);
    return convertedImage;
}

cv::Mat CameraImageUtils::wrapRgb888Image(const QImage& image)
{
    return cv::Mat(image.height(), image.width(), CV_8UC3,
                   const_cast<uchar*>(image.constBits()),
                   static_cast<size_t>(image.bytesPerLine()));
}

QImage CameraImageUtils::convertBgrToRgbImage(const cv::Mat& bgrMat)
{
    QImage result = acquirePooledImage(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
    cv::Mat rgbMat(result.height(), result.width(), CV_8UC3,
                   result.bits(),
                   static_cast<size_t>(result.bytesPerLine()));
    cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
    return result;
}

cv::Mat CameraImageUtils::imageToWorkingMat(const QImage& input, bool *highBitDepthInput)
{
    const bool highBitDepth = (input.format() == QImage::Format_RGBA64)
        || (input.format() == QImage::Format_RGBX64)
        || (input.format() == QImage::Format_Grayscale16);
    if (highBitDepthInput) {
        *highBitDepthInput = highBitDepth;
    }

    if (input.format() == QImage::Format_Grayscale16)
    {
        return cv::Mat(input.height(), input.width(), CV_16UC1,
            const_cast<uchar*>(input.bits()),
            static_cast<size_t>(input.bytesPerLine()));
    }

    if (input.format() == QImage::Format_Grayscale8)
    {
        return cv::Mat(input.height(), input.width(), CV_8UC1,
            const_cast<uchar*>(input.bits()),
            static_cast<size_t>(input.bytesPerLine()));
    }

    if ((input.format() == QImage::Format_RGBA64) || (input.format() == QImage::Format_RGBX64))
    {
        cv::Mat frameMat(input.height(), input.width(), CV_16UC3);
        for (int y = 0; y < input.height(); ++y)
        {
            const QRgba64 *inputLine = reinterpret_cast<const QRgba64*>(input.constScanLine(y));
            cv::Vec<uint16_t, 3> *outputLine = frameMat.ptr<cv::Vec<uint16_t, 3>>(y);

            for (int x = 0; x < input.width(); ++x)
            {
                outputLine[x][0] = inputLine[x].red();
                outputLine[x][1] = inputLine[x].green();
                outputLine[x][2] = inputLine[x].blue();
            }
        }
        return frameMat;
    }

    if (input.format() == QImage::Format_RGB888)
    {
        return cv::Mat(input.height(), input.width(), CV_8UC3,
            const_cast<uchar*>(input.bits()),
            static_cast<size_t>(input.bytesPerLine()));
    }

    const QImage rgb = input.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3,
                   const_cast<uchar*>(rgb.bits()),
                   static_cast<size_t>(rgb.bytesPerLine()));
    return rgbMat.clone();
}

QImage CameraImageUtils::workingMatToImage(const cv::Mat& frameMat)
{
    if (frameMat.channels() == 1)
    {
        if (frameMat.depth() == CV_16U)
        {
            QImage image = acquirePooledImage(frameMat.cols, frameMat.rows, QImage::Format_Grayscale16);
            for (int row = 0; row < frameMat.rows; ++row) {
                std::memcpy(image.scanLine(row), frameMat.ptr(row), static_cast<size_t>(frameMat.cols * sizeof(quint16)));
            }
            return image;
        }

        QImage image = acquirePooledImage(frameMat.cols, frameMat.rows, QImage::Format_Grayscale8);
        for (int row = 0; row < frameMat.rows; ++row) {
            std::memcpy(image.scanLine(row), frameMat.ptr(row), static_cast<size_t>(frameMat.cols));
        }
        return image;
    }

    if (frameMat.depth() == CV_16U)
    {
        QImage image = acquirePooledImage(frameMat.cols, frameMat.rows, QImage::Format_RGBA64);
        for (int y = 0; y < frameMat.rows; ++y)
        {
            const cv::Vec<uint16_t, 3> *inputLine = frameMat.ptr<cv::Vec<uint16_t, 3>>(y);
            QRgba64 *outputLine = reinterpret_cast<QRgba64*>(image.scanLine(y));

            for (int x = 0; x < frameMat.cols; ++x) {
                outputLine[x] = qRgba64(inputLine[x][0], inputLine[x][1], inputLine[x][2], 65535);
            }
        }
        return image;
    }

    QImage image = acquirePooledImage(frameMat.cols, frameMat.rows, QImage::Format_RGB888);
    for (int row = 0; row < frameMat.rows; ++row) {
        std::memcpy(image.scanLine(row), frameMat.ptr(row), static_cast<size_t>(frameMat.cols * 3));
    }
    return image;
}

int CameraImageUtils::bayerPatternToOpenCvCode(CameraPipelineFrame::BayerPattern bayerPattern)
{
    switch (bayerPattern)
    {
    case CameraPipelineFrame::BayerRGGB:
        return cv::COLOR_BayerBG2BGR;
    case CameraPipelineFrame::BayerBGGR:
        return cv::COLOR_BayerRG2BGR;
    case CameraPipelineFrame::BayerGRBG:
        return cv::COLOR_BayerGB2BGR;
    case CameraPipelineFrame::BayerGBRG:
        return cv::COLOR_BayerGR2BGR;
    case CameraPipelineFrame::BayerNone:
    default:
        return -1;
    }
}

cv::Mat CameraImageUtils::debayerRawMat(const cv::Mat& input, CameraPipelineFrame::BayerPattern bayerPattern)
{
    const int cvCode = bayerPatternToOpenCvCode(bayerPattern);
    if ((cvCode < 0) || (input.channels() != 1)) {
        return input;
    }

    cv::Mat debayered;
    cv::cvtColor(input, debayered, cvCode);
    if (debayered.channels() == 3) {
        cv::cvtColor(debayered, debayered, cv::COLOR_BGR2RGB);
    }
    return debayered;
}
