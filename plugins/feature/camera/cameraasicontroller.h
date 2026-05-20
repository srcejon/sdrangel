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

    CameraAsiController();

    bool openCamera(int cameraId);
    void closeCamera(int fallbackCameraId);
    bool stopVideoCapture(int fallbackCameraId);
    bool stopExposure(int fallbackCameraId);
    void setLastError(int errorCode, const QString& errorMessage);

    static QString errorCodeToString(ASI_ERROR_CODE errorCode);
    static bool getCameraInfoById(int cameraId, ASI_CAMERA_INFO& cameraInfo);
    static bool getControlCapsByType(int cameraId, ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS& controlCaps);
    static bool getControlValueByType(int cameraId, ASI_CONTROL_TYPE controlType, long& value, ASI_BOOL& isAuto);
    static bool supportsImageType(const ASI_CAMERA_INFO& cameraInfo, ASI_IMG_TYPE imageType);
    static int bayerToOpenCvCode(int bayerPattern);
    static CameraPipelineFrame::BayerPattern bayerToPipelinePattern(int bayerPattern);
    static ASI_IMG_TYPE selectImageType(const ASI_CAMERA_INFO& cameraInfo, const CameraSettings& settings);

    QImage frameToImage(const QImage& fallbackImage, CameraPipelineFrame::BayerPattern *bayerPattern = nullptr) const;

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
