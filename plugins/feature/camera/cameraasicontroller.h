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
#include "cameraimagepool.h"

#ifdef ASICAMERA_FOUND
#include <ASICamera2.h>
#endif

/**
 * \brief Wraps the ZWO ASI camera SDK (ASICamera2) for opening, configuring and capturing frames.
 *
 * Encapsulates the stateful ASICamera2 C API behind a higher-level interface: opening/closing a
 * camera by id, querying its capabilities (sensor size, gain/offset/exposure ranges, cooler, USB
 * bandwidth, supported image types), applying CameraSettings, and capturing either single exposure
 * frames or continuous video frames. Captured raw buffers are converted to QImage via frameToImage(),
 * with optional Bayer-pattern reporting for colour sensors. Tracks the last error, CCD temperature
 * and capture timing for status reporting, plus a "continuous capture" generation counter used to
 * cancel stale scheduled captures.
 *
 * \note The whole class (and most of its members) only exists when the SDK is present
 *       (\c ASICAMERA_FOUND); otherwise it compiles to an empty shell.
 * \note Not thread-safe and holds no internal locking. The ASICamera2 SDK is process-global, so
 *       callers must serialise access to it with CameraAsiSdkLocker around SDK calls.
 * \note frameToImage() is const but uses a mutable CameraImagePool to recycle per-frame backing
 *       buffers; the pool is refcounted and release-on-last-copy, so handing the produced image to
 *       another (pipeline) thread is safe.
 */
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
    // Last ROI/format actually pushed to the SDK via ASISetROIFormat/ASISetStartPos.
    // Used to skip redundant sensor reconfiguration (which can corrupt the next
    // snap-mode frame) when only exposure/gain/other live controls change.
    // -1 means nothing has been applied yet (forces a fresh ROI apply).
    int m_appliedWidth;
    int m_appliedHeight;
    int m_appliedBin;
    int m_appliedImageType;
    int m_appliedStartX;
    int m_appliedStartY;
    QVector<uchar> m_frameBuffer;
    // Recycles the per-frame QImage backing buffers produced in frameToImage
    // (Grayscale8/16 and RGB888 for sustained high-res capture). mutable because
    // frameToImage is const but the pool is a cache. Released-on-last-copy and
    // refcounted, so it is safe even though the produced image is handed to the
    // pipeline on another thread (see CameraImagePool).
    mutable CameraImagePool m_imagePool;
    double m_lastCcdTemperature;
    bool m_lastCcdTemperatureValid;
    qint64 m_lastCaptureTimeMs;
    int m_lastErrorNumber;
    QString m_lastErrorMessage;
    OpenFailureStage m_lastOpenFailureStage;
#endif
};

#endif // INCLUDE_FEATURE_CAMERAASICONTROLLER_H_
