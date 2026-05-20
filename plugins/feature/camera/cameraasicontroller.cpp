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

#include <cstring>

#include <QDebug>

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
    const ASI_ERROR_CODE error = ASIGetCameraPropertyByID(cameraId, &cameraInfo);

    if (error != ASI_SUCCESS) {
        qDebug() << "CameraAsiController: ASIGetCameraPropertyByID failed:" << error << errorCodeToString(error);
    }

    return error == ASI_SUCCESS;
}

bool CameraAsiController::getControlCapsByType(int cameraId, ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS& controlCaps)
{
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
        QImage image(m_frameBuffer.constData(), m_frameWidth, m_frameHeight, m_frameWidth * 3, QImage::Format_RGB888);
        return image.rgbSwapped();
    }

    if (m_imageType == ASI_IMG_Y8 || (!m_colorCamera && m_imageType == ASI_IMG_RAW8))
    {
        QImage image(m_frameWidth, m_frameHeight, QImage::Format_Grayscale8);
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
            QImage image(m_frameWidth, m_frameHeight, QImage::Format_Grayscale16);
            for (int y = 0; y < m_frameHeight; ++y) {
                std::memcpy(image.scanLine(y), raw16.ptr(y), static_cast<size_t>(m_frameWidth * sizeof(quint16)));
            }
            return image;
        }

        if (bayerPattern) {
            *bayerPattern = bayerToPipelinePattern(m_bayerPattern);
        }

        QImage image(m_frameWidth, m_frameHeight, QImage::Format_Grayscale16);
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
        QImage image(m_frameWidth, m_frameHeight, QImage::Format_Grayscale8);
        for (int y = 0; y < m_frameHeight; ++y) {
            std::memcpy(image.scanLine(y), rawMat.ptr(y), static_cast<size_t>(m_frameWidth));
        }
        return image;
    }

    if (bayerPattern) {
        *bayerPattern = bayerToPipelinePattern(m_bayerPattern);
    }

    QImage image(m_frameWidth, m_frameHeight, QImage::Format_Grayscale8);
    for (int y = 0; y < m_frameHeight; ++y) {
        std::memcpy(image.scanLine(y), rawMat.ptr(y), static_cast<size_t>(m_frameWidth));
    }
    return image;
}

#endif
