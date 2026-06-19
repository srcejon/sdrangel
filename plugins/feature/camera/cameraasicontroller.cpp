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

#include "cameraasicontroller.h"

#ifdef ASICAMERA_FOUND

#include "cameraasisdk.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <QDebug>
#include <QElapsedTimer>
#include <QThread>

#include <opencv2/imgproc/imgproc.hpp>

CameraAsiController::CameraAsiController() :
    m_cameraOpen(false),
    m_videoCaptureStarted(false),
    m_settingsApplied(false),
    m_continuousCaptureScheduled(false),
    m_continuousCaptureGeneration(0),
    m_openCameraId(-1),
    m_triggerCamera(false),
    m_cameraSizeX(0),
    m_cameraSizeY(0),
    m_maxBinX(1),
    m_maxBinY(1),
    m_bayerPattern(ASI_BAYER_RG),
    m_colorCamera(false),
    m_bitDepth(8),
    m_imageType(ASI_IMG_Y8),
    m_rgb24Supported(false),
    m_raw16Supported(false),
    m_raw8Supported(false),
    m_pixelSizeUm(0.0),
    m_exposureMinMs(0.001),
    m_exposureMaxMs(60000.0),
    m_frameWidth(0),
    m_frameHeight(0),
    m_frameBuffer(),
    m_lastCcdTemperature(0.0),
    m_lastCcdTemperatureValid(false),
    m_lastCaptureTimeMs(-1),
    m_lastErrorNumber(0),
    m_lastErrorMessage(),
    m_lastOpenFailureStage(OpenFailureNone)
{
}

bool CameraAsiController::openCamera(int cameraId)
{
    CameraAsiSdkLocker asiSdkLocker;
    m_lastOpenFailureStage = OpenFailureNone;

    if (m_cameraOpen) {
        return true;
    }

    if (cameraId < 0) {
        return false;
    }

    const ASI_ERROR_CODE openError = ASIOpenCamera(cameraId);
    if (openError != ASI_SUCCESS)
    {
        m_lastOpenFailureStage = OpenFailureOpen;
        setLastError(openError, errorCodeToString(openError));
        qDebug() << "CameraAsiController: ASIOpenCamera failed:" << openError << errorCodeToString(openError);
        return false;
    }

    const ASI_ERROR_CODE initError = ASIInitCamera(cameraId);
    if (initError != ASI_SUCCESS)
    {
        m_lastOpenFailureStage = OpenFailureInit;
        setLastError(initError, errorCodeToString(initError));
        qDebug() << "CameraAsiController: ASIInitCamera failed:" << initError << errorCodeToString(initError);
        const ASI_ERROR_CODE closeError = ASICloseCamera(cameraId);
        if (closeError != ASI_SUCCESS) {
            qDebug() << "CameraAsiController: ASICloseCamera failed after init error:" << closeError << errorCodeToString(closeError);
        }
        return false;
    }

    ASI_CAMERA_INFO cameraInfo {};
    if (getCameraInfoById(cameraId, cameraInfo)) {
        m_triggerCamera = cameraInfo.IsTriggerCam == ASI_TRUE;
    }

    if (m_triggerCamera)
    {
        const ASI_ERROR_CODE modeError = ASISetCameraMode(cameraId, ASI_MODE_NORMAL);

        if (modeError != ASI_SUCCESS)
        {
            m_lastOpenFailureStage = OpenFailureMode;
            setLastError(modeError, errorCodeToString(modeError));
            qDebug() << "CameraAsiController: ASISetCameraMode failed:" << modeError << errorCodeToString(modeError);
            const ASI_ERROR_CODE closeError = ASICloseCamera(cameraId);
            if (closeError != ASI_SUCCESS) {
                qDebug() << "CameraAsiController: ASICloseCamera failed after mode error:" << closeError << errorCodeToString(closeError);
            }
            return false;
        }
    }

    setLastError(ASI_SUCCESS, QString());
    m_cameraOpen = true;
    m_openCameraId = cameraId;
    return true;
}

void CameraAsiController::closeCamera(int fallbackCameraId)
{
    CameraAsiSdkLocker asiSdkLocker;
    const int cameraId = (m_openCameraId >= 0) ? m_openCameraId : fallbackCameraId;

    stopVideoCapture(cameraId);

    if (m_cameraOpen && (cameraId >= 0))
    {
        const ASI_ERROR_CODE closeError = ASICloseCamera(cameraId);
        if (closeError != ASI_SUCCESS) {
            qDebug() << "CameraAsiController: ASICloseCamera failed:" << closeError << errorCodeToString(closeError);
        }
        m_cameraOpen = false;
    }

    if (!m_cameraOpen) {
        m_openCameraId = -1;
    }
    m_settingsApplied = false;
}

bool CameraAsiController::stopVideoCapture(int fallbackCameraId)
{
    CameraAsiSdkLocker asiSdkLocker;
    const int cameraId = (m_openCameraId >= 0) ? m_openCameraId : fallbackCameraId;

    if (!m_videoCaptureStarted || (cameraId < 0)) {
        return true;
    }

    const ASI_ERROR_CODE stopError = ASIStopVideoCapture(cameraId);
    if (stopError != ASI_SUCCESS)
    {
        setLastError(stopError, errorCodeToString(stopError));
        qDebug() << "CameraAsiController: ASIStopVideoCapture failed:" << stopError << errorCodeToString(stopError);
        return false;
    }

    m_videoCaptureStarted = false;
    return true;
}

bool CameraAsiController::stopExposure(int fallbackCameraId)
{
    CameraAsiSdkLocker asiSdkLocker;
    const int cameraId = (m_openCameraId >= 0) ? m_openCameraId : fallbackCameraId;

    if (!m_cameraOpen || (cameraId < 0)) {
        return true;
    }

    const ASI_ERROR_CODE stopExposureError = ASIStopExposure(cameraId);
    if ((stopExposureError != ASI_SUCCESS)
        && (stopExposureError != ASI_ERROR_GENERAL_ERROR)
        && (stopExposureError != ASI_ERROR_INVALID_MODE))
    {
        setLastError(stopExposureError, errorCodeToString(stopExposureError));
        qDebug() << "CameraAsiController: ASIStopExposure failed:" << stopExposureError << errorCodeToString(stopExposureError);
        return false;
    }

    return true;
}

void CameraAsiController::setLastError(int errorCode, const QString& errorMessage)
{
    m_lastErrorNumber = errorCode;
    m_lastErrorMessage = errorMessage;
}

bool CameraAsiController::queryCameraCapabilities(int cameraId, const CameraSettings& settings, CapabilitiesReport& report)
{
    CameraAsiSdkLocker asiSdkLocker;
    ASI_CAMERA_INFO cameraInfo {};

    if (!getCameraInfoById(cameraId, cameraInfo)) {
        return false;
    }

    ASI_CONTROL_CAPS gainRange {};
    ASI_CONTROL_CAPS offsetRange {};
    ASI_CONTROL_CAPS exposureRange {};
    ASI_CONTROL_CAPS coolerOnCaps {};
    ASI_CONTROL_CAPS targetTempCaps {};
    ASI_CONTROL_CAPS usbBandwidthCaps {};
    ASI_CONTROL_CAPS highSpeedModeCaps {};
    const bool hasGainRange = getControlCapsByType(cameraId, ASI_GAIN, gainRange);
    const bool hasOffsetRange = getControlCapsByType(cameraId, ASI_OFFSET, offsetRange);
    const bool hasExposureRange = getControlCapsByType(cameraId, ASI_EXPOSURE, exposureRange);
    const bool hasCoolerOn = getControlCapsByType(cameraId, ASI_COOLER_ON, coolerOnCaps);
    const bool hasTargetTemp = getControlCapsByType(cameraId, ASI_TARGET_TEMP, targetTempCaps);
    const bool hasUsbBandwidth = getControlCapsByType(cameraId, ASI_BANDWIDTHOVERLOAD, usbBandwidthCaps);
    const bool hasHighSpeedMode = getControlCapsByType(cameraId, ASI_HIGH_SPEED_MODE, highSpeedModeCaps);
    long coolerOnValue = 0;
    long targetTempValue = 0;
    long usbBandwidthValue = 0;
    long highSpeedModeValue = 0;
    ASI_BOOL isAuto = ASI_FALSE;
    const bool hasCoolerOnValue = hasCoolerOn && getControlValueByType(cameraId, ASI_COOLER_ON, coolerOnValue, isAuto);
    const bool hasTargetTempValue = hasTargetTemp && getControlValueByType(cameraId, ASI_TARGET_TEMP, targetTempValue, isAuto);
    const bool hasUsbBandwidthValue = hasUsbBandwidth && getControlValueByType(cameraId, ASI_BANDWIDTHOVERLOAD, usbBandwidthValue, isAuto);
    const bool hasHighSpeedModeValue = hasHighSpeedMode && getControlValueByType(cameraId, ASI_HIGH_SPEED_MODE, highSpeedModeValue, isAuto);

    m_cameraSizeX = static_cast<int>(cameraInfo.MaxWidth);
    m_cameraSizeY = static_cast<int>(cameraInfo.MaxHeight);
    m_maxBinX = 1;
    m_maxBinY = 1;
    for (int bin : cameraInfo.SupportedBins)
    {
        if (bin <= 0) {
            break;
        }

        m_maxBinX = std::max(m_maxBinX, bin);
        m_maxBinY = std::max(m_maxBinY, bin);
    }
    m_bayerPattern = cameraInfo.BayerPattern;
    m_colorCamera = cameraInfo.IsColorCam == ASI_TRUE;
    m_triggerCamera = cameraInfo.IsTriggerCam == ASI_TRUE;
    m_bitDepth = cameraInfo.BitDepth;
    m_pixelSizeUm = cameraInfo.PixelSize;
    m_exposureMinMs = hasExposureRange ? std::max(0.001, exposureRange.MinValue / 1000.0) : 0.001;
    m_exposureMaxMs = hasExposureRange ? std::max(m_exposureMinMs, exposureRange.MaxValue / 1000.0) : 60000.0;
    m_rgb24Supported = supportsImageType(cameraInfo, ASI_IMG_RGB24);
    m_raw16Supported = supportsImageType(cameraInfo, ASI_IMG_RAW16);
    m_raw8Supported = supportsImageType(cameraInfo, ASI_IMG_RAW8);
    m_imageType = selectImageType(cameraInfo, settings);

    report.m_name = QString::fromUtf8(cameraInfo.Name);
    report.m_maxBinX = m_maxBinX;
    report.m_maxBinY = m_maxBinY;
    report.m_gainMin = hasGainRange ? static_cast<int>(gainRange.MinValue) : 0;
    report.m_gainMax = hasGainRange ? static_cast<int>(gainRange.MaxValue) : 100;
    report.m_offsetMin = hasOffsetRange ? static_cast<int>(offsetRange.MinValue) : 0;
    report.m_offsetMax = hasOffsetRange ? static_cast<int>(offsetRange.MaxValue) : 100;
    report.m_cameraSizeX = m_cameraSizeX;
    report.m_cameraSizeY = m_cameraSizeY;
    report.m_pixelSizeUm = m_pixelSizeUm;
    report.m_bitDepth = m_bitDepth;
    report.m_colorCamera = m_colorCamera;
    report.m_exposureMinMs = m_exposureMinMs;
    report.m_exposureMaxMs = m_exposureMaxMs;
    report.m_coolerSupported = hasCoolerOn && coolerOnCaps.IsWritable == ASI_TRUE;
    report.m_coolerOn = hasCoolerOnValue ? (coolerOnValue != 0) : false;
    report.m_targetTempSupported = hasTargetTemp && targetTempCaps.IsWritable == ASI_TRUE;
    report.m_targetTempMin = hasTargetTemp ? static_cast<int>(targetTempCaps.MinValue) : 0;
    report.m_targetTempMax = hasTargetTemp ? static_cast<int>(targetTempCaps.MaxValue) : 0;
    report.m_targetTemp = hasTargetTempValue ? static_cast<int>(targetTempValue) : 0;
    report.m_usbBandwidthSupported = hasUsbBandwidth && usbBandwidthCaps.IsWritable == ASI_TRUE;
    report.m_usbBandwidthMin = hasUsbBandwidth ? static_cast<int>(usbBandwidthCaps.MinValue) : 0;
    report.m_usbBandwidthMax = hasUsbBandwidth ? static_cast<int>(usbBandwidthCaps.MaxValue) : 0;
    report.m_usbBandwidth = hasUsbBandwidthValue ? static_cast<int>(usbBandwidthValue) : 0;
    report.m_highSpeedModeSupported = hasHighSpeedMode && highSpeedModeCaps.IsWritable == ASI_TRUE;
    report.m_highSpeedMode = hasHighSpeedModeValue ? (highSpeedModeValue != 0) : false;
    report.m_rgb24Supported = m_rgb24Supported;
    report.m_raw16Supported = m_raw16Supported;
    report.m_raw8Supported = m_raw8Supported;
    return true;
}

bool CameraAsiController::applyCameraSettings(int cameraId, const CameraSettings& settings, double exposureTimeMs)
{
    CameraAsiSdkLocker asiSdkLocker;
    ASI_CAMERA_INFO cameraInfo {};
    if (getCameraInfoById(cameraId, cameraInfo)) {
        m_imageType = selectImageType(cameraInfo, settings);
    }

    const int bin = std::max(1, std::min(settings.m_cameraBinX, settings.m_cameraBinY));
    const int roiWidthStep = 8;
    const int roiHeightStep = 2;
    const int minWidth = 16;
    const int minHeight = 16;
    const int maxWidth = std::max(minWidth, m_cameraSizeX / std::max(1, bin));
    const int maxHeight = std::max(minHeight, m_cameraSizeY / std::max(1, bin));

    auto alignDown = [](int value, int step, int minimum) {
        const int aligned = (value / step) * step;
        return std::max(minimum, aligned);
    };

    const int requestedWidth = (settings.m_cameraNumX == 0)
        ? maxWidth
        : qBound(minWidth, settings.m_cameraNumX, maxWidth);
    const int requestedHeight = (settings.m_cameraNumY == 0)
        ? maxHeight
        : qBound(minHeight, settings.m_cameraNumY, maxHeight);

    const int width = alignDown(requestedWidth, roiWidthStep, minWidth);
    const int height = alignDown(requestedHeight, roiHeightStep, minHeight);
    const int startX = qBound(0, settings.m_cameraStartX, std::max(0, maxWidth - width));
    const int startY = qBound(0, settings.m_cameraStartY, std::max(0, maxHeight - height));

    const ASI_ERROR_CODE roiError = ASISetROIFormat(cameraId, width, height, bin, static_cast<ASI_IMG_TYPE>(m_imageType));
    if (roiError != ASI_SUCCESS) {
        setLastError(roiError, errorCodeToString(roiError));
        qDebug() << "CameraAsiController: ASISetROIFormat failed:" << roiError << errorCodeToString(roiError)
                 << "width" << width << "height" << height << "bin" << bin << "imageType" << m_imageType;
        return false;
    }

    const ASI_ERROR_CODE startPosError = ASISetStartPos(cameraId, startX, startY);
    if (startPosError != ASI_SUCCESS) {
        setLastError(startPosError, errorCodeToString(startPosError));
        qDebug() << "CameraAsiController: ASISetStartPos failed:" << startPosError << errorCodeToString(startPosError)
                 << "startX" << startX << "startY" << startY << "width" << width << "height" << height << "bin" << bin;
        return false;
    }

    const ASI_BOOL autoExposureGain = (settings.m_asiAutoExposureGain
            && !settings.m_autoExposureGainEnabled
            && (settings.m_captureMode == CameraSettings::CaptureModeFrameRate))
        ? ASI_TRUE
        : ASI_FALSE;
    auto writableControl = [cameraId](ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS *controlCaps = nullptr) -> bool {
        ASI_CONTROL_CAPS caps {};
        if (!getControlCapsByType(cameraId, controlType, caps) || (caps.IsWritable != ASI_TRUE)) {
            return false;
        }

        if (controlCaps) {
            *controlCaps = caps;
        }
        return true;
    };

    ASI_CONTROL_CAPS coolerOnCaps {};
    ASI_CONTROL_CAPS targetTempCaps {};
    ASI_CONTROL_CAPS usbBandwidthCaps {};
    ASI_CONTROL_CAPS highSpeedModeCaps {};
    const bool canSetCoolerOn = writableControl(ASI_COOLER_ON, &coolerOnCaps);
    const bool canSetTargetTemp = writableControl(ASI_TARGET_TEMP, &targetTempCaps);
    const bool canSetUsbBandwidth = writableControl(ASI_BANDWIDTHOVERLOAD, &usbBandwidthCaps);
    const bool canSetHighSpeedMode = writableControl(ASI_HIGH_SPEED_MODE, &highSpeedModeCaps);

    const ASI_ERROR_CODE exposureError = ASISetControlValue(cameraId, ASI_EXPOSURE,
        std::max(1L, static_cast<long>(std::llround(exposureTimeMs * 1000.0))), autoExposureGain);
    const ASI_ERROR_CODE gainError = ASISetControlValue(cameraId, ASI_GAIN,
        std::max(0L, static_cast<long>(settings.m_cameraGain)), autoExposureGain);
    const ASI_ERROR_CODE offsetError = ASISetControlValue(cameraId, ASI_OFFSET,
        std::max(0L, static_cast<long>(settings.m_cameraOffset)), ASI_FALSE);
    const ASI_ERROR_CODE coolerOnError = ((settings.m_asiCoolerOn >= 0) && canSetCoolerOn)
        ? ASISetControlValue(cameraId, ASI_COOLER_ON, settings.m_asiCoolerOn != 0 ? 1L : 0L, ASI_FALSE)
        : ASI_SUCCESS;
    const ASI_ERROR_CODE targetTempError = ((settings.m_asiTargetTemp != std::numeric_limits<int>::min()) && canSetTargetTemp)
        ? ASISetControlValue(cameraId, ASI_TARGET_TEMP,
            qBound(targetTempCaps.MinValue, static_cast<long>(settings.m_asiTargetTemp), targetTempCaps.MaxValue),
            ASI_FALSE)
        : ASI_SUCCESS;
    const ASI_ERROR_CODE usbBandwidthError = ((settings.m_asiUsbBandwidth >= 0) && canSetUsbBandwidth)
        ? ASISetControlValue(cameraId, ASI_BANDWIDTHOVERLOAD,
            qBound(usbBandwidthCaps.MinValue, static_cast<long>(settings.m_asiUsbBandwidth), usbBandwidthCaps.MaxValue),
            ASI_FALSE)
        : ASI_SUCCESS;
    const ASI_ERROR_CODE highSpeedModeError = ((settings.m_asiHighSpeedMode >= 0) && canSetHighSpeedMode)
        ? ASISetControlValue(cameraId, ASI_HIGH_SPEED_MODE, settings.m_asiHighSpeedMode != 0 ? 1L : 0L, ASI_FALSE)
        : ASI_SUCCESS;

    auto handleControlError = [this](ASI_ERROR_CODE error, const char *name) -> bool {
        if (error == ASI_SUCCESS) {
            return true;
        }
        setLastError(error, errorCodeToString(error));
        qDebug() << "CameraAsiController: ASISetControlValue(" << name << ") failed:" << error << errorCodeToString(error);
        return false;
    };

    if (!handleControlError(exposureError, "EXPOSURE")
        || !handleControlError(gainError, "GAIN")
        || !handleControlError(offsetError, "OFFSET")
        || !handleControlError(coolerOnError, "COOLER_ON")
        || !handleControlError(targetTempError, "TARGET_TEMP")
        || !handleControlError(usbBandwidthError, "BANDWIDTHOVERLOAD")
        || !handleControlError(highSpeedModeError, "HIGH_SPEED_MODE"))
    {
        return false;
    }

    m_frameWidth = width;
    m_frameHeight = height;

    int bytesPerPixel = 1;
    switch (m_imageType)
    {
    case ASI_IMG_RGB24:
        bytesPerPixel = 3;
        break;
    case ASI_IMG_RAW16:
        bytesPerPixel = 2;
        break;
    default:
        bytesPerPixel = 1;
        break;
    }

    m_frameBuffer.resize(width * height * bytesPerPixel);
    m_settingsApplied = true;
    setLastError(ASI_SUCCESS, QString());
    return true;
}

CameraAsiController::CaptureResult CameraAsiController::captureExposureFrame(int cameraId, double exposureTimeMs)
{
    CameraAsiSdkLocker asiSdkLocker;
    if (m_videoCaptureStarted && !stopVideoCapture(cameraId)) {
        return CaptureDataFailed;
    }

    const ASI_ERROR_CODE startExposureError = ASIStartExposure(cameraId, ASI_FALSE);
    if (startExposureError != ASI_SUCCESS)
    {
        setLastError(startExposureError, errorCodeToString(startExposureError));
        qDebug() << "CameraAsiController: ASIStartExposure failed:" << startExposureError << errorCodeToString(startExposureError);
        return CaptureStartFailed;
    }

    setLastError(ASI_SUCCESS, QString());
    QElapsedTimer captureTimer;
    captureTimer.start();

    const qint64 timeoutMs = std::max<qint64>(1000, static_cast<qint64>(std::ceil(exposureTimeMs)) + 5000);
    const unsigned long pollSleepMs = static_cast<unsigned long>(
        std::min<qint64>(50, std::max<qint64>(2, static_cast<qint64>(std::ceil(exposureTimeMs / 4.0)))));
    ASI_EXPOSURE_STATUS exposureStatus = ASI_EXP_IDLE;

    while (captureTimer.elapsed() <= timeoutMs)
    {
        const ASI_ERROR_CODE statusError = ASIGetExpStatus(cameraId, &exposureStatus);
        if (statusError != ASI_SUCCESS)
        {
            setLastError(statusError, errorCodeToString(statusError));
            qDebug() << "CameraAsiController: ASIGetExpStatus failed:" << statusError << errorCodeToString(statusError);
            return CaptureDataFailed;
        }

        if (exposureStatus == ASI_EXP_SUCCESS) {
            break;
        }

        if (exposureStatus == ASI_EXP_FAILED)
        {
            setLastError(ASI_ERROR_GENERAL_ERROR, QStringLiteral("Exposure failed"));
            qDebug() << "CameraAsiController: ASI exposure failed";
            return CaptureDataFailed;
        }

        QThread::msleep(pollSleepMs);
    }

    if (exposureStatus != ASI_EXP_SUCCESS)
    {
        setLastError(ASI_ERROR_TIMEOUT, errorCodeToString(ASI_ERROR_TIMEOUT));
        qDebug() << "CameraAsiController: ASI exposure timed out after" << timeoutMs << "ms";
        return CaptureDataFailed;
    }

    const ASI_ERROR_CODE dataError = ASIGetDataAfterExp(cameraId, m_frameBuffer.data(), m_frameBuffer.size());
    if (dataError != ASI_SUCCESS)
    {
        setLastError(dataError, errorCodeToString(dataError));
        qDebug() << "CameraAsiController: ASIGetDataAfterExp failed:" << dataError << errorCodeToString(dataError)
                 << "bufferSize" << m_frameBuffer.size()
                 << "width" << m_frameWidth << "height" << m_frameHeight;
        return CaptureDataFailed;
    }

    setLastError(ASI_SUCCESS, QString());
    m_lastCaptureTimeMs = captureTimer.elapsed();
    return CaptureSuccess;
}

CameraAsiController::CaptureResult CameraAsiController::captureVideoFrame(int cameraId, int waitMs)
{
    CameraAsiSdkLocker asiSdkLocker;
    if (!m_videoCaptureStarted)
    {
        const ASI_ERROR_CODE startCaptureError = ASIStartVideoCapture(cameraId);
        if (startCaptureError != ASI_SUCCESS)
        {
            setLastError(startCaptureError, errorCodeToString(startCaptureError));
            qDebug() << "CameraAsiController: ASIStartVideoCapture failed:" << startCaptureError << errorCodeToString(startCaptureError);
            return CaptureStartFailed;
        }
        setLastError(ASI_SUCCESS, QString());
        m_videoCaptureStarted = true;
    }

    QElapsedTimer captureTimer;
    captureTimer.start();
    const ASI_ERROR_CODE getVideoError = ASIGetVideoData(cameraId, m_frameBuffer.data(), m_frameBuffer.size(), waitMs);
    if (getVideoError != ASI_SUCCESS)
    {
        setLastError(getVideoError, errorCodeToString(getVideoError));
        qDebug() << "CameraAsiController: ASIGetVideoData failed:" << getVideoError << errorCodeToString(getVideoError)
                 << "waitMs" << waitMs << "bufferSize" << m_frameBuffer.size()
                 << "width" << m_frameWidth << "height" << m_frameHeight;
        return CaptureDataFailed;
    }

    setLastError(ASI_SUCCESS, QString());
    m_lastCaptureTimeMs = captureTimer.elapsed();
    return CaptureSuccess;
}

CameraAsiController::StatusReport CameraAsiController::pollStatus(int cameraId)
{
    CameraAsiSdkLocker asiSdkLocker;
    long temperatureTenthsC = 0;
    ASI_BOOL isAuto = ASI_FALSE;
    const ASI_ERROR_CODE temperatureError = ASIGetControlValue(cameraId, ASI_TEMPERATURE, &temperatureTenthsC, &isAuto);

    if (temperatureError == ASI_SUCCESS)
    {
        setLastError(ASI_SUCCESS, QString());
        m_lastCcdTemperature = temperatureTenthsC / 10.0;
        m_lastCcdTemperatureValid = true;
    }
    else
    {
        setLastError(temperatureError, errorCodeToString(temperatureError));
        m_lastCcdTemperatureValid = false;
    }

    return statusReport();
}

CameraAsiController::StatusReport CameraAsiController::statusReport() const
{
    StatusReport report;
    report.m_ccdTemperature = m_lastCcdTemperature;
    report.m_ccdTemperatureValid = m_lastCcdTemperatureValid;
    report.m_lastCaptureTimeMs = m_lastCaptureTimeMs;
    report.m_imageTypeName = imageTypeName();
    report.m_lastErrorNumber = m_lastErrorNumber;
    report.m_lastErrorMessage = m_lastErrorMessage;
    return report;
}

void CameraAsiController::cancelContinuousCapture()
{
    ++m_continuousCaptureGeneration;
    m_continuousCaptureScheduled = false;
}

void CameraAsiController::markContinuousCaptureScheduled()
{
    m_continuousCaptureScheduled = true;
}

bool CameraAsiController::clearContinuousCaptureScheduled(quint64 generation)
{
    if (generation != m_continuousCaptureGeneration) {
        return false;
    }

    m_continuousCaptureScheduled = false;
    return true;
}

QString CameraAsiController::imageTypeName() const
{
    switch (m_imageType)
    {
    case ASI_IMG_RGB24:
        return QStringLiteral("RGB24");
    case ASI_IMG_RAW16:
        return QStringLiteral("RAW16");
    case ASI_IMG_RAW8:
        return QStringLiteral("RAW8");
    default:
        return QStringLiteral("Y8");
    }
}

QString CameraAsiController::errorCodeToString(ASI_ERROR_CODE errorCode)
{
    switch (errorCode)
    {
    case ASI_SUCCESS: return QStringLiteral("ASI_SUCCESS");
    case ASI_ERROR_INVALID_INDEX: return QStringLiteral("ASI_ERROR_INVALID_INDEX");
    case ASI_ERROR_INVALID_ID: return QStringLiteral("ASI_ERROR_INVALID_ID");
    case ASI_ERROR_INVALID_CONTROL_TYPE: return QStringLiteral("ASI_ERROR_INVALID_CONTROL_TYPE");
    case ASI_ERROR_CAMERA_CLOSED: return QStringLiteral("ASI_ERROR_CAMERA_CLOSED");
    case ASI_ERROR_CAMERA_REMOVED: return QStringLiteral("ASI_ERROR_CAMERA_REMOVED");
    case ASI_ERROR_INVALID_PATH: return QStringLiteral("ASI_ERROR_INVALID_PATH");
    case ASI_ERROR_INVALID_FILEFORMAT: return QStringLiteral("ASI_ERROR_INVALID_FILEFORMAT");
    case ASI_ERROR_INVALID_SIZE: return QStringLiteral("ASI_ERROR_INVALID_SIZE");
    case ASI_ERROR_INVALID_IMGTYPE: return QStringLiteral("ASI_ERROR_INVALID_IMGTYPE");
    case ASI_ERROR_OUTOF_BOUNDARY: return QStringLiteral("ASI_ERROR_OUTOF_BOUNDARY");
    case ASI_ERROR_TIMEOUT: return QStringLiteral("ASI_ERROR_TIMEOUT");
    case ASI_ERROR_INVALID_SEQUENCE: return QStringLiteral("ASI_ERROR_INVALID_SEQUENCE");
    case ASI_ERROR_BUFFER_TOO_SMALL: return QStringLiteral("ASI_ERROR_BUFFER_TOO_SMALL");
    case ASI_ERROR_VIDEO_MODE_ACTIVE: return QStringLiteral("ASI_ERROR_VIDEO_MODE_ACTIVE");
    case ASI_ERROR_EXPOSURE_IN_PROGRESS: return QStringLiteral("ASI_ERROR_EXPOSURE_IN_PROGRESS");
    case ASI_ERROR_GENERAL_ERROR: return QStringLiteral("ASI_ERROR_GENERAL_ERROR");
    case ASI_ERROR_INVALID_MODE: return QStringLiteral("ASI_ERROR_INVALID_MODE");
    case ASI_ERROR_GPS_NOT_SUPPORTED: return QStringLiteral("ASI_ERROR_GPS_NOT_SUPPORTED");
    case ASI_ERROR_GPS_VER_ERR: return QStringLiteral("ASI_ERROR_GPS_VER_ERR");
    case ASI_ERROR_GPS_FPGA_ERR: return QStringLiteral("ASI_ERROR_GPS_FPGA_ERR");
    case ASI_ERROR_GPS_PARAM_OUT_OF_RANGE: return QStringLiteral("ASI_ERROR_GPS_PARAM_OUT_OF_RANGE");
    case ASI_ERROR_GPS_DATA_INVALID: return QStringLiteral("ASI_ERROR_GPS_DATA_INVALID");
    default: return QStringLiteral("ASI_ERROR_UNKNOWN");
    }
}

bool CameraAsiController::getCameraInfoById(int cameraId, ASI_CAMERA_INFO& cameraInfo)
{
    CameraAsiSdkLocker asiSdkLocker;
    const ASI_ERROR_CODE error = ASIGetCameraPropertyByID(cameraId, &cameraInfo);

    if (error != ASI_SUCCESS) {
        qDebug() << "CameraAsiController: ASIGetCameraPropertyByID failed:" << error << errorCodeToString(error);
    }

    return error == ASI_SUCCESS;
}

bool CameraAsiController::getControlCapsByType(int cameraId, ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS& controlCaps)
{
    CameraAsiSdkLocker asiSdkLocker;
    int numControls = 0;
    const ASI_ERROR_CODE numControlsError = ASIGetNumOfControls(cameraId, &numControls);

    if (numControlsError != ASI_SUCCESS) {
        qDebug() << "CameraAsiController: ASIGetNumOfControls failed:" << numControlsError << errorCodeToString(numControlsError);
        return false;
    }

    for (int controlIndex = 0; controlIndex < numControls; ++controlIndex)
    {
        ASI_CONTROL_CAPS candidate {};
        const ASI_ERROR_CODE controlCapsError = ASIGetControlCaps(cameraId, controlIndex, &candidate);

        if (controlCapsError != ASI_SUCCESS)
        {
            qDebug() << "CameraAsiController: ASIGetControlCaps failed:" << controlCapsError << errorCodeToString(controlCapsError)
                     << "controlIndex" << controlIndex;
            continue;
        }

        if (candidate.ControlType == controlType)
        {
            controlCaps = candidate;
            return true;
        }
    }

    return false;
}

bool CameraAsiController::getControlValueByType(int cameraId, ASI_CONTROL_TYPE controlType, long& value, ASI_BOOL& isAuto)
{
    CameraAsiSdkLocker asiSdkLocker;
    const ASI_ERROR_CODE error = ASIGetControlValue(cameraId, controlType, &value, &isAuto);

    if (error != ASI_SUCCESS) {
        qDebug() << "CameraAsiController: ASIGetControlValue failed:" << error << errorCodeToString(error)
                 << "controlType" << static_cast<int>(controlType);
    }

    return error == ASI_SUCCESS;
}

bool CameraAsiController::supportsImageType(const ASI_CAMERA_INFO& cameraInfo, ASI_IMG_TYPE imageType)
{
    for (ASI_IMG_TYPE candidate : cameraInfo.SupportedVideoFormat)
    {
        if (candidate == ASI_IMG_END) {
            break;
        }

        if (candidate == imageType) {
            return true;
        }
    }

    return false;
}

int CameraAsiController::bayerToOpenCvCode(int bayerPattern)
{
    switch (bayerPattern)
    {
    case ASI_BAYER_RG:
        return cv::COLOR_BayerBG2BGR;
    case ASI_BAYER_BG:
        return cv::COLOR_BayerRG2BGR;
    case ASI_BAYER_GR:
        return cv::COLOR_BayerGB2BGR;
    case ASI_BAYER_GB:
        return cv::COLOR_BayerGR2BGR;
    default:
        return cv::COLOR_BayerBG2BGR;
    }
}

CameraPipelineFrame::BayerPattern CameraAsiController::bayerToPipelinePattern(int bayerPattern)
{
    switch (bayerPattern)
    {
    case ASI_BAYER_RG:
        return CameraPipelineFrame::BayerRGGB;
    case ASI_BAYER_BG:
        return CameraPipelineFrame::BayerBGGR;
    case ASI_BAYER_GR:
        return CameraPipelineFrame::BayerGRBG;
    case ASI_BAYER_GB:
        return CameraPipelineFrame::BayerGBRG;
    default:
        return CameraPipelineFrame::BayerRGGB;
    }
}

ASI_IMG_TYPE CameraAsiController::selectImageType(const ASI_CAMERA_INFO& cameraInfo, const CameraSettings& settings)
{
    if (cameraInfo.IsColorCam == ASI_TRUE)
    {
        if ((settings.m_asiColorImageType == CameraSettings::AsiColorImageTypeRaw16)
            && supportsImageType(cameraInfo, ASI_IMG_RAW16))
        {
            return ASI_IMG_RAW16;
        }
        if ((settings.m_asiColorImageType == CameraSettings::AsiColorImageTypeRaw8)
            && supportsImageType(cameraInfo, ASI_IMG_RAW8))
        {
            return ASI_IMG_RAW8;
        }
        if ((settings.m_asiColorImageType == CameraSettings::AsiColorImageTypeRgb24)
            && supportsImageType(cameraInfo, ASI_IMG_RGB24))
        {
            return ASI_IMG_RGB24;
        }
        if (supportsImageType(cameraInfo, ASI_IMG_RGB24)) {
            return ASI_IMG_RGB24;
        }
        if (supportsImageType(cameraInfo, ASI_IMG_RAW16)) {
            return ASI_IMG_RAW16;
        }
        if (supportsImageType(cameraInfo, ASI_IMG_RAW8)) {
            return ASI_IMG_RAW8;
        }
        if (supportsImageType(cameraInfo, ASI_IMG_Y8)) {
            return ASI_IMG_Y8;
        }
    }
    else
    {
        if (supportsImageType(cameraInfo, ASI_IMG_RAW16)) {
            return ASI_IMG_RAW16;
        }
        if (supportsImageType(cameraInfo, ASI_IMG_RAW8)) {
            return ASI_IMG_RAW8;
        }
        if (supportsImageType(cameraInfo, ASI_IMG_Y8)) {
            return ASI_IMG_Y8;
        }
    }

    return ASI_IMG_Y8;
}

QImage CameraAsiController::frameToImage(const QImage& fallbackImage, CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

    if (m_frameWidth <= 0 || m_frameHeight <= 0 || m_frameBuffer.isEmpty()) {
        return fallbackImage;
    }

    if (m_imageType == ASI_IMG_RGB24)
    {
        // ASI RGB24 is BGR; swap into a pooled RGB888 buffer (replaces the
        // alloc-and-swap that rgbSwapped() would do).
        QImage image = m_imagePool.acquire(m_frameWidth, m_frameHeight, QImage::Format_RGB888);
        if (image.isNull()) {
            return fallbackImage;
        }
        const uchar *src = reinterpret_cast<const uchar*>(m_frameBuffer.constData());
        for (int y = 0; y < m_frameHeight; ++y)
        {
            const uchar *s = src + static_cast<size_t>(y) * m_frameWidth * 3;
            uchar *d = image.scanLine(y);
            for (int x = 0; x < m_frameWidth; ++x)
            {
                d[3 * x + 0] = s[3 * x + 2];
                d[3 * x + 1] = s[3 * x + 1];
                d[3 * x + 2] = s[3 * x + 0];
            }
        }
        return image;
    }

    if (m_imageType == ASI_IMG_Y8 || (!m_colorCamera && m_imageType == ASI_IMG_RAW8))
    {
        QImage image = m_imagePool.acquire(m_frameWidth, m_frameHeight, QImage::Format_Grayscale8);
        if (image.isNull()) {
            return fallbackImage;
        }
        for (int y = 0; y < m_frameHeight; ++y) {
            std::memcpy(image.scanLine(y), m_frameBuffer.constData() + (y * m_frameWidth), static_cast<size_t>(m_frameWidth));
        }
        return image;
    }

    cv::Mat rawMat;
    if (m_imageType == ASI_IMG_RAW16)
    {
        cv::Mat raw16(m_frameHeight, m_frameWidth, CV_16UC1, const_cast<uchar*>(m_frameBuffer.constData()));

        if (!m_colorCamera)
        {
            QImage image = m_imagePool.acquire(m_frameWidth, m_frameHeight, QImage::Format_Grayscale16);
            if (image.isNull()) {
                return fallbackImage;
            }
            for (int y = 0; y < m_frameHeight; ++y) {
                std::memcpy(image.scanLine(y), raw16.ptr(y), static_cast<size_t>(m_frameWidth * sizeof(quint16)));
            }
            return image;
        }

        if (bayerPattern) {
            *bayerPattern = bayerToPipelinePattern(m_bayerPattern);
        }

        QImage image = m_imagePool.acquire(m_frameWidth, m_frameHeight, QImage::Format_Grayscale16);
        if (image.isNull()) {
            return fallbackImage;
        }
        for (int y = 0; y < m_frameHeight; ++y) {
            std::memcpy(image.scanLine(y), raw16.ptr(y), static_cast<size_t>(m_frameWidth * sizeof(quint16)));
        }
        return image;
    }
    else
    {
        rawMat = cv::Mat(m_frameHeight, m_frameWidth, CV_8UC1, const_cast<uchar*>(m_frameBuffer.constData())).clone();
    }

    if (!m_colorCamera)
    {
        QImage image = m_imagePool.acquire(m_frameWidth, m_frameHeight, QImage::Format_Grayscale8);
        if (image.isNull()) {
            return fallbackImage;
        }
        for (int y = 0; y < m_frameHeight; ++y) {
            std::memcpy(image.scanLine(y), rawMat.ptr(y), static_cast<size_t>(m_frameWidth));
        }
        return image;
    }

    if (bayerPattern) {
        *bayerPattern = bayerToPipelinePattern(m_bayerPattern);
    }

    QImage image = m_imagePool.acquire(m_frameWidth, m_frameHeight, QImage::Format_Grayscale8);
    if (image.isNull()) {
        return fallbackImage;
    }
    for (int y = 0; y < m_frameHeight; ++y) {
        std::memcpy(image.scanLine(y), rawMat.ptr(y), static_cast<size_t>(m_frameWidth));
    }
    return image;
}

#endif
