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

#ifndef INCLUDE_FEATURE_CAMERAASICONTROLLER_H_
#define INCLUDE_FEATURE_CAMERAASICONTROLLER_H_

#include <QString>
#include <QVector>
#include <QImage>

#include "camerapipelineframe.h"
#include "camerasettings.h"

#ifdef ASICAMERA_FOUND
#include <ASICamera2.h>
#endif

class CameraAsiController
{
public:
#ifdef ASICAMERA_FOUND
    enum OpenFailureStage
    {
        OpenFailureNone,
        OpenFailureOpen,
        OpenFailureInit,
        OpenFailureMode
    };

    enum CaptureResult
    {
        CaptureSuccess,
        CaptureStartFailed,
        CaptureDataFailed
    };

    struct CapabilitiesReport
    {
        QString m_name;
        int m_maxBinX = 1;
        int m_maxBinY = 1;
        int m_gainMin = 0;
        int m_gainMax = 100;
        int m_offsetMin = 0;
        int m_offsetMax = 100;
        int m_cameraSizeX = 0;
        int m_cameraSizeY = 0;
        double m_pixelSizeUm = 0.0;
        int m_bitDepth = 8;
        bool m_colorCamera = false;
        double m_exposureMinMs = 0.001;
        double m_exposureMaxMs = 60000.0;
        bool m_coolerSupported = false;
        bool m_coolerOn = false;
        bool m_targetTempSupported = false;
        int m_targetTempMin = 0;
        int m_targetTempMax = 0;
        int m_targetTemp = 0;
        bool m_usbBandwidthSupported = false;
        int m_usbBandwidthMin = 0;
        int m_usbBandwidthMax = 0;
        int m_usbBandwidth = 0;
        bool m_highSpeedModeSupported = false;
        bool m_highSpeedMode = false;
        bool m_rgb24Supported = false;
        bool m_raw16Supported = false;
        bool m_raw8Supported = false;
    };

    struct StatusReport
    {
        double m_ccdTemperature = 0.0;
        bool m_ccdTemperatureValid = false;
        qint64 m_lastCaptureTimeMs = -1;
        QString m_imageTypeName;
        int m_lastErrorNumber = 0;
        QString m_lastErrorMessage;
    };

    CameraAsiController();

    bool openCamera(int cameraId);
    void closeCamera(int fallbackCameraId);
    bool stopVideoCapture(int fallbackCameraId);
    bool stopExposure(int fallbackCameraId);
    void setLastError(int errorCode, const QString& errorMessage);
    bool queryCameraCapabilities(int cameraId, const CameraSettings& settings, CapabilitiesReport& report);
    bool applyCameraSettings(int cameraId, const CameraSettings& settings, double exposureTimeMs);
    CaptureResult captureExposureFrame(int cameraId, double exposureTimeMs);
    CaptureResult captureVideoFrame(int cameraId, int waitMs);
    StatusReport pollStatus(int cameraId);
    StatusReport statusReport() const;

    bool hasCameraSize() const { return (m_cameraSizeX > 0) && (m_cameraSizeY > 0); }
    bool settingsApplied() const { return m_settingsApplied; }
    void invalidateSettings() { m_settingsApplied = false; }
    bool continuousCaptureScheduled() const { return m_continuousCaptureScheduled; }
    quint64 continuousCaptureGeneration() const { return m_continuousCaptureGeneration; }
    void cancelContinuousCapture();
    void markContinuousCaptureScheduled();
    bool clearContinuousCaptureScheduled(quint64 generation);
    OpenFailureStage lastOpenFailureStage() const { return m_lastOpenFailureStage; }
    int lastErrorNumber() const { return m_lastErrorNumber; }
    const QString& lastErrorMessage() const { return m_lastErrorMessage; }
    double exposureMinMs() const { return m_exposureMinMs; }
    double exposureMaxMs() const { return m_exposureMaxMs; }

    static QString errorCodeToString(ASI_ERROR_CODE errorCode);
    static bool getCameraInfoById(int cameraId, ASI_CAMERA_INFO& cameraInfo);
    static bool getControlCapsByType(int cameraId, ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS& controlCaps);
    static bool getControlValueByType(int cameraId, ASI_CONTROL_TYPE controlType, long& value, ASI_BOOL& isAuto);
    static bool supportsImageType(const ASI_CAMERA_INFO& cameraInfo, ASI_IMG_TYPE imageType);
    static int bayerToOpenCvCode(int bayerPattern);
    static CameraPipelineFrame::BayerPattern bayerToPipelinePattern(int bayerPattern);
    static ASI_IMG_TYPE selectImageType(const ASI_CAMERA_INFO& cameraInfo, const CameraSettings& settings);

    QImage frameToImage(const QImage& fallbackImage, CameraPipelineFrame::BayerPattern *bayerPattern = nullptr) const;

private:
    QString imageTypeName() const;

    bool m_cameraOpen;
    bool m_videoCaptureStarted;
    bool m_settingsApplied;
    bool m_continuousCaptureScheduled;
    quint64 m_continuousCaptureGeneration;
    int m_openCameraId;
    bool m_triggerCamera;
    int m_cameraSizeX;
    int m_cameraSizeY;
    int m_maxBinX;
    int m_maxBinY;
    int m_bayerPattern;
    bool m_colorCamera;
    int m_bitDepth;
    int m_imageType;
    bool m_rgb24Supported;
    bool m_raw16Supported;
    bool m_raw8Supported;
    double m_pixelSizeUm;
    double m_exposureMinMs;
    double m_exposureMaxMs;
    int m_frameWidth;
    int m_frameHeight;
    QVector<uchar> m_frameBuffer;
    double m_lastCcdTemperature;
    bool m_lastCcdTemperatureValid;
    qint64 m_lastCaptureTimeMs;
    int m_lastErrorNumber;
    QString m_lastErrorMessage;
    OpenFailureStage m_lastOpenFailureStage;
#endif
};

#endif // INCLUDE_FEATURE_CAMERAASICONTROLLER_H_
