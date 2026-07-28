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

#ifndef INCLUDE_FEATURE_CAMERASETTINGS_H_
#define INCLUDE_FEATURE_CAMERASETTINGS_H_

#include <array>

#include <QByteArray>
#include <QColor>
#include <QDateTime>
#include <QHash>
#include <QList>
#include <QRect>
#include <QString>
#include <QStringList>

#include "cameradrawing.h"

class Serializable;

/**
 * \brief Complete, serializable configuration for the Camera feature and its pipeline.
 *
 * CameraSettings is the single value-type aggregate that holds every user-configurable
 * option for the camera feature: source selection (Qt webcam, ZWO ASI, ASCOM Alpaca,
 * video file, image sequence or network stream), capture cadence/exposure, stacking and
 * alignment, image post-processing, motion/star/object/diff detection, plate solving,
 * recording/keogram/YouTube output, sky overlays, and reverse-API settings. A copy is
 * embedded in Camera, CameraWorker, each pipeline stage and CameraWebAPIAdapter; changes
 * propagate as MsgConfigureCamera carrying the settings plus a key list of changed fields.
 *
 * The struct also defines the enums for those options, validation bounds as static
 * constexpr members, and helper accessors (camera-protocol predicates such as
 * isAsiCamera()/isStreamCamera(), HDR exposure helpers, capture-interval conversions).
 *
 * \note Plain value type: it owns no pipeline objects (m_rollupState is a non-owned
 *       Serializable pointer set via setRollupState()). It is copied freely between
 *       threads, so it must stay cheap and self-contained.
 * \note serialize()/deserialize() define the persisted format and the WebAPI mapping;
 *       keep field changes in sync with those and with applySettings()/the changed-key
 *       lists used by MsgConfigureCamera.
 * \see Camera, CameraWorker, Camera::MsgConfigureCamera
 */
struct CameraSettings
{
    enum CaptureMode
    {
        CaptureModeFrameRate = 0,
        CaptureModeInterval = 1
    };

    enum CaptureIntervalUnits
    {
        CaptureIntervalSeconds = 0,
        CaptureIntervalMinutes = 1
    };

    enum StackMethod
    {
        StackMethodAverage = 0,
        StackMethodMedian,
        StackMethodSigmaClippedAverage,
        StackMethodLuckySharpAverage,
        StackMethodHDR
    };

    enum StackHdrAlgorithm
    {
        StackHdrAlgorithmDebevec = 0,
        StackHdrAlgorithmRobertson,
        StackHdrAlgorithmMertens
    };

    enum StackAlignmentMethod
    {
        StackAlignmentNone = 0,
        StackAlignmentPhaseCorrelation,
        StackAlignmentStarCentroidMatching
    };

    enum StackDisplayMode
    {
        StackDisplayStacked = 0,
        StackDisplayHistoryFrame,
        StackDisplayHistoryTiles
    };

    enum ScaleJustification
    {
        ScaleJustifyCenter = 0,
        ScaleJustifyLeft,
        ScaleJustifyRight,
        ScaleJustifyTop,
        ScaleJustifyBottom
    };

    enum TrackObjectLabelDisplay
    {
        TrackObjectLabelAlways = 0,
        TrackObjectLabelNearDetection
    };

    enum AsiColorImageType
    {
        AsiColorImageTypeRgb24 = 0,
        AsiColorImageTypeRaw16,
        AsiColorImageTypeRaw8
    };

    enum AutoExposureGainMode
    {
        AutoExposureGainExposureFirst = 0,
        AutoExposureGainGainFirst,
        AutoExposureGainExposureOnly,
        AutoExposureGainGainOnly
    };

    enum KeogramDirection
    {
        KeogramHorizontal = 0,
        KeogramVertical
    };

    enum KeogramDayMode
    {
        KeogramMidnightToMidnight = 0,
        KeogramMiddayToMidday
    };

    enum VideoCodec
    {
        VideoCodecH264 = 0,
        VideoCodecH265
    };

    enum LensProjection
    {
        LensProjectionRectilinear = 0,
        LensProjectionEquidistant,
        LensProjectionEquisolid
    };

    enum SensorOpticalAxis
    {
        SensorOpticalAxisAuto = 0,
        SensorOpticalAxisRear,
        SensorOpticalAxisFront
    };

    enum ThermalDecoder
    {
        ThermalDecoderOff = 0,
        ThermalDecoderAuto,
        ThermalDecoderThermalMasterP2,
        ThermalDecoderTopdonTc001
    };

    enum ThermalPalette
    {
        ThermalPaletteWhiteHot = 0,
        ThermalPaletteBlackHot,
        ThermalPaletteIron,
        ThermalPaletteInferno,
        ThermalPaletteTurbo,
        ThermalPaletteViridis
    };

    enum ThermalUnits
    {
        ThermalUnitsCelsius = 0,
        ThermalUnitsFahrenheit
    };

    enum HistogramStretch
    {
        HistogramStretchOff = 0,
        HistogramStretchLinear,
        HistogramStretchGamma,
        HistogramStretchAsinh,
        HistogramStretchLog,
        HistogramStretchCLAHE
    };

    enum EdgeDisplayMode
    {
        EdgeDisplayOverlay = 0,
        EdgeDisplayEdgesOnly
    };

    enum MotionBackgroundSubtractor
    {
        MotionBackgroundSubtractorMOG2 = 0,
        MotionBackgroundSubtractorKNN
    };

    enum MotionMaskView
    {
        MotionMaskViewOff = 0,
        MotionMaskViewRaw,
        MotionMaskViewThresholded,
        MotionMaskViewOpened,
        MotionMaskViewClosed,
        MotionMaskViewFinal
    };

    enum StarDebugView
    {
        StarDebugViewOff = 0,
        StarDebugViewBackground,
        StarDebugViewResidual,
        StarDebugViewThresholded,
        StarDebugViewFinal
    };

    enum CloudDetectionMode
    {
        CloudModeAuto = 0,
        CloudModeDay,
        CloudModeNight
    };

    enum CloudDebugView
    {
        CloudDebugViewOff = 0,
        CloudDebugViewBackground,
        CloudDebugViewSignal,
        CloudDebugViewThresholded,
        CloudDebugViewFinal,
        CloudDebugViewTexture
    };

    enum ConstellationOverlay
    {
        ConstellationOverlayUrsaMajor = 0,
        ConstellationOverlayOrion,
        ConstellationOverlayCrux,
        ConstellationOverlayAuriga,
        ConstellationOverlayBootes,
        ConstellationOverlayCanisMajor,
        ConstellationOverlayCassiopeia,
        ConstellationOverlayCygnus,
        ConstellationOverlayGemini,
        ConstellationOverlayLeo,
        ConstellationOverlayLyra,
        ConstellationOverlayPegasus,
        ConstellationOverlaySagittarius,
        ConstellationOverlayScorpius,
        ConstellationOverlayTaurus
    };

    enum PlateSolveApplyMode
    {
        PlateSolveApplyAzEl = 0,
        PlateSolveApplyAzElRoll,
        PlateSolveApplyAzElRollFov,
        PlateSolveApplyAzElRollFovLens
    };

    enum PlateSolveStartMode
    {
        PlateSolveStartBlind = 0,
        PlateSolveStartFov,
        PlateSolveStartFovElevation,
        PlateSolveStartFovAzEl,
        PlateSolveStartFovAzElRoll,
        PlateSolveStartFovAzElRollLens,
        PlateSolveStartCurrentSettingsOnly
    };

    enum PlateSolveLabelMode
    {
        PlateSolveLabelNone = 0,
        PlateSolveLabelName,
        PlateSolveLabelNameMagnitude,
        PlateSolveLabelNameMagnitudeSpectralType
    };

    enum PlateSolveCatalogSource
    {
        PlateSolveCatalogAuto = 0,
        PlateSolveCatalogHyg,
        PlateSolveCatalogSirilSpccGaia,
        PlateSolveCatalogSirilAstroGaia
    };

    struct WindowOverlay
    {
        bool m_enabled = true;
        QString m_windowClass;
        QString m_windowTitle;
        QString m_windowId;
        QString m_regionObjectName;
        QString m_regionTitle;
        int m_offsetX = 0;
        int m_offsetY = 0;
        double m_scale = 1.0;
        double m_captureFps = 2.0;
    };

    struct SpectrumOverlay
    {
        bool m_enabled = true;
        QString m_source;
        int m_offsetX = 0;
        int m_offsetY = 0;
        double m_scale = 1.0;
        double m_captureFps = 2.0;
    };

    enum FovMode
    {
        FovModeDirect = 0,
        FovModeSensorFocalLength
    };

    static constexpr int m_minHdrExposureCount = 2;
    static constexpr int m_maxHdrExposureCount = 4;

    bool isHdrStackingEnabled() const
    {
        return m_stackEnabled && (m_stackMethod == StackMethodHDR);
    }

    int getHdrExposureCount() const
    {
        return qBound(m_minHdrExposureCount, m_stackHdrExposureCount, m_maxHdrExposureCount);
    }

    double getHdrExposureTimeMs(int index) const
    {
        if ((index < 0) || (index >= static_cast<int>(m_stackHdrExposureTimesMs.size()))) {
            return m_minExposureTimeMs;
        }

        return std::max(m_minExposureTimeMs, m_stackHdrExposureTimesMs[static_cast<size_t>(index)]);
    }

    static constexpr int m_minResolution = 16;
    static constexpr int m_minFramesPerSecond = 1;
    static constexpr double m_minCaptureInterval = 0.1;
    static constexpr double m_minExposureTimeMs = 0.001;
    static constexpr int m_minIsoSensitivity = -1;
    static constexpr int m_minNonNegative = 0;
    static constexpr int m_minPositive = 1;
    static constexpr int m_minAsiControl = -1;
    static constexpr int m_maxAsiControl = 1;
    static constexpr int m_minStackFrameCount = 1;
    static constexpr int m_maxStackFrameCount = 256;
    static constexpr float m_minLatitude = -90.0f;
    static constexpr float m_maxLatitude = 90.0f;
    static constexpr float m_minLongitude = -180.0f;
    static constexpr float m_maxLongitude = 180.0f;
    static constexpr float m_minAltitude = -1000.0f;
    static constexpr float m_maxAltitude = 100000.0f;
    static constexpr float m_minAzimuth = 0.0f;
    static constexpr float m_maxAzimuth = 360.0f;
    static constexpr float m_fullRotationDegrees = 360.0f;
    static constexpr float m_minRoll = -180.0f;
    static constexpr float m_maxRoll = 180.0f;
    static constexpr float m_minElevation = -90.0f;
    static constexpr float m_maxElevation = 90.0f;
    static constexpr float m_minFov = 0.01f;
    static constexpr float m_maxFov = 180.0f;
    static constexpr double m_minNormalized = 0.0;
    static constexpr double m_maxNormalized = 1.0;
    static constexpr double m_minWhiteBalanceGain = 0.1;
    static constexpr double m_maxWhiteBalanceGain = 8.0;
    static constexpr double m_minHistogramWhitePointGap = 0.001;
    static constexpr double m_minHistogramStrength = 0.1;
    static constexpr double m_maxHistogramStrength = 100.0;
    static constexpr double m_minFilterAmount = 0.0;
    static constexpr double m_maxFilterAmount = 3.0;
    static constexpr int m_minBlurRadius = 0;
    static constexpr int m_maxBlurRadius = 15;
    static constexpr double m_minBrightness = -100.0;
    static constexpr double m_maxBrightness = 100.0;
    static constexpr int m_minThreshold8Bit = 0;
    static constexpr int m_maxThreshold8Bit = 255;
    static constexpr int m_minMorphologyKernel = 0;
    static constexpr int m_maxMorphologyKernel = 20;
    static constexpr int m_minShortHistoryFrames = 1;
    static constexpr int m_maxShortHistoryFrames = 120;
    static constexpr int m_minUiPixelOffset = 0;
    static constexpr int m_maxUiPixelOffset = 4096;
    static constexpr int m_minSignedUiPixelOffset = -4096;
    static constexpr int m_maxSignedUiPixelOffset = 4096;
    static constexpr double m_minOverlayFontScale = 4.0;
    static constexpr double m_maxOverlayFontScale = 144.0;
    static constexpr int m_minMotionHistory = 1;
    static constexpr int m_maxMotionHistory = 5000;
    static constexpr double m_minMotionVarThreshold = 1.0;
    static constexpr double m_maxMotionVarThreshold = 200.0;
    static constexpr double m_minLearningRate = -1.0;
    static constexpr double m_maxLearningRate = 1.0;
    static constexpr int m_minMotionConfirmFrames = 1;
    static constexpr int m_maxMotionConfirmFrames = 60;
    static constexpr int m_minContourAreaBound = 0;
    static constexpr int m_maxContourAreaBound = 10000;
    static constexpr int m_minStarBackgroundBlur = 1;
    static constexpr int m_maxStarBackgroundBlur = 50;
    static constexpr double m_minCloudRatioThreshold = 0.0;
    static constexpr double m_maxCloudRatioThreshold = 2.0;
    static constexpr int m_minCloudBackgroundBlur = 1;
    static constexpr int m_maxCloudBackgroundBlur = 128;
    static constexpr int m_minCloudUpdateInterval = 1;
    static constexpr int m_maxCloudUpdateInterval = 600;
    static constexpr double m_minCoveragePercent = 0.0;
    static constexpr double m_maxCoveragePercent = 100.0;
    static constexpr double m_minCloudEdgeMargin = 0.0;
    static constexpr double m_maxCloudEdgeMargin = 25.0;
    static constexpr double m_minCloudSunMoonRadius = 0.0;
    static constexpr double m_maxCloudSunMoonRadius = 45.0;
    static constexpr double m_minCloudStarMagnitude = 0.0;
    static constexpr double m_maxCloudStarMagnitude = 6.0;
    static constexpr double m_minStarAspectRatio = 1.0;
    static constexpr double m_maxStarAspectRatio = 10.0;
    static constexpr double m_minPlateSolveMagnitude = -2.0;
    static constexpr double m_maxPlateSolveMagnitude = 20.0;
    static constexpr int m_minPlateSolveMatches = 2;
    static constexpr int m_maxPlateSolveMatches = 32;
    static constexpr double m_minPlateSolveMatchRadius = 1.0;
    static constexpr double m_maxPlateSolveMatchRadius = 256.0;
    static constexpr double m_minPlateSolveAzElSearchRadius = 0.0;
    static constexpr double m_maxPlateSolveAzElSearchRadius = 45.0;
    static constexpr double m_minPlateSolveFovTolerance = 0.0;
    static constexpr double m_maxPlateSolveFovTolerance = 50.0;
    static constexpr int m_minStarCatalogDiskCacheSizeGb = 0;
    static constexpr int m_maxStarCatalogDiskCacheSizeGb = 1024;
    static constexpr qint64 m_minPlateSolveDateTimeMs = 0;
    static constexpr double m_minLensCenterOffset = -2048.0;
    static constexpr double m_maxLensCenterOffset = 2048.0;
    static constexpr double m_minLensDistortionK1 = -1.0;
    static constexpr double m_maxLensDistortionK1 = 1.0;
    static constexpr double m_minSpectrumScale = 0.1;
    static constexpr double m_maxSpectrumScale = 4.0;
    static constexpr double m_minWindowOverlayScale = 0.1;
    static constexpr double m_maxWindowOverlayScale = 4.0;
    static constexpr double m_minWindowOverlayFps = 0.1;
    static constexpr double m_maxWindowOverlayFps = 30.0;
    static constexpr double m_minExposureCompensation = -2.0;
    static constexpr double m_maxExposureCompensation = 2.0;
    static constexpr double m_minZoomFactor = 1.0;
    static constexpr double m_minVideoPlaybackRate = 0.1;
    static constexpr double m_maxVideoPlaybackRate = 120.0;
    static constexpr int m_minVideoPlaybackAudioOffsetMs = -2000;
    static constexpr int m_maxVideoPlaybackAudioOffsetMs = 5000;
    static constexpr double m_minStreamBufferingSeconds = 0.1;
    static constexpr double m_maxStreamBufferingSeconds = 30.0;

    QString m_title;
    quint32 m_rgbColor;
    QString m_cameraProtocol;
    QString m_cameraId;
    QString m_cameraDescription;
    int m_resolutionWidth;
    int m_resolutionHeight;
    int m_framesPerSecond;
    CaptureMode m_captureMode;
    double m_captureInterval;
    CaptureIntervalUnits m_captureIntervalUnits;
    double m_exposureTimeMs;
    int m_isoSensitivity;
    bool m_alpacaDiscoveryEnabled;
    bool m_alpacaApiLogEnabled;
    QString m_alpacaHost;
    uint16_t m_alpacaPort;
    bool m_alpacaFocuserEnabled;
    QString m_alpacaFocuserHost;
    uint16_t m_alpacaFocuserPort;
    int m_alpacaFocuserDeviceNumber;
    int m_alpacaFocusPosition;
    int m_alpacaFocusStepSize;
    bool m_alpacaFilterWheelEnabled;
    QString m_alpacaFilterWheelHost;
    uint16_t m_alpacaFilterWheelPort;
    int m_alpacaFilterWheelDeviceNumber;
    int m_alpacaFilterWheelPosition;
    int m_cameraBinX;
    int m_cameraBinY;
    int m_cameraNumX;
    int m_cameraNumY;
    int m_cameraStartX;
    int m_cameraStartY;
    int m_cameraGain;         // index into named gains list, or numeric value; -1 = do not set
    int m_cameraOffset;       // index into named offsets list, or numeric value; -1 = do not set
    int m_cameraReadoutMode;  // index into readoutmodes list
    int m_asiCoolerOn;        // -1 = do not set, 0 = off, 1 = on
    int m_asiTargetTemp;      // target temperature in Celsius; sentinel means do not set
    int m_asiUsbBandwidth;    // USB bandwidth overload setting; -1 = do not set
    int m_asiHighSpeedMode;   // -1 = do not set, 0 = off, 1 = on
    bool m_asiAutoExposureGain; ///< Enable ASI auto exposure and gain in frame-rate mode
    bool m_autoExposureGainEnabled; ///< Software auto exposure/gain loop
    AutoExposureGainMode m_autoExposureGainMode;
    double m_autoExposureTargetPercentile;
    double m_autoExposureTargetBrightness;
    double m_autoExposureMaxChangePercent;
    double m_autoExposureMinMs;
    double m_autoExposureMaxMs;
    int m_autoExposureMinGain;
    int m_autoExposureMaxGain;
    AsiColorImageType m_asiColorImageType;
    bool m_saveImage;
    QString m_imageFileName;
    bool m_saveVideo;
    QString m_videoFileCameraPath;
    QString m_streamUrl;
    QStringList m_imageFileCameraPaths;
    QStringList m_streamUrlHistory;
    double m_streamBufferingSeconds; ///< Decoded media cushion for stream: camera sources
    QString m_videoFileName;
    QString m_recordingOutputDirectoryUri; ///< Android Storage Access Framework output tree URI
    bool m_recordRawFits;          ///< Save uncalibrated Bayer still images as FITS
    bool m_recordCalibratedMedia;  ///< Save calibrated/debayered image/video media
    bool m_recordFilteredMedia;    ///< Save camera image processor output image/video media
    bool m_recordPostProcessedMedia; ///< Save post-processed image/video media
    bool m_keogramEnabled;         ///< Build and save a 24-hour keogram from calibrated frames
    QString m_keogramFileName;     ///< File basename for saved keograms (.jpg/.png)
    KeogramDirection m_keogramDirection; ///< Whether samples are horizontal or vertical image slices
    KeogramDayMode m_keogramDayMode; ///< Whether the 24-hour keogram window starts at midnight or midday
    int m_keogramSamplePeriodMinutes; ///< Keogram sample period in minutes
    bool m_keogramShowPreview;     ///< Show a live keogram preview dialog in the GUI
    bool m_youtubeStreamEnabled;   ///< Stream captured video to YouTube
    QString m_youtubeStreamUrl;    ///< YouTube RTMP/RTMPS ingest URL
    QString m_youtubeStreamKey;    ///< YouTube stream key
    bool m_youtubeStreamPostProcessed; ///< Stream post-processed frames rather than calibrated frames
    int m_youtubeStreamBitrateKbps; ///< Video bitrate in kbit/s
    int m_youtubeStreamFps;        ///< Stream frame rate
    int m_youtubeStreamWidth;      ///< Output stream width; 0 = source width
    int m_youtubeStreamHeight;     ///< Output stream height; 0 = source height
    VideoCodec m_videoCodec;       ///< Codec used for saved video recordings
    int m_videoRecordBitrateKbps;  ///< Saved video bitrate in kbit/s; 0 = automatic from frame size/rate
    bool m_videoLoop;
    double m_videoPlaybackRate;
    int m_videoPlaybackAudioOffsetMs; ///< Preview audio offset for video playback in ms; negative plays audio earlier
    bool m_videoHwAcceleration; ///< Prefer hardware-accelerated video encoding when supported
    int m_videoPreRecordBufferSeconds; ///< Seconds of frames to prepend when recording starts
    int m_imageRecordLimit; ///< Number of image frames to save before disabling image recording; 0 = unlimited
    int m_videoRecordLimitSeconds; ///< Seconds of video to record before disabling video recording; 0 = unlimited
    bool m_stackEnabled;
    int m_stackFrameCount;
    StackMethod m_stackMethod;
    StackHdrAlgorithm m_stackHdrAlgorithm;
    int m_stackHdrExposureCount;
    std::array<double, 4> m_stackHdrExposureTimesMs;
    StackAlignmentMethod m_stackAlignmentMethod;
    StackDisplayMode m_stackDisplayMode;
    int m_stackDisplayFrameIndex;
    bool m_stackRejectBadFrames;
    bool m_scaleEnabled;           ///< Scale calibrated/stacked image output after stacking
    int m_scaleWidth;              ///< Scaled output width in pixels; <= 0 disables scaling
    int m_scaleHeight;             ///< Scaled output height in pixels; <= 0 disables scaling
    bool m_scaleKeepAspectRatio;   ///< Preserve aspect ratio within the requested scale size
    ScaleJustification m_scaleJustification; ///< Placement when keep-aspect scaling leaves padding
    QString m_stackDarkFileName;
    QString m_stackFlatFileName;
    QString m_stackBiasFileName;
    float m_latitude;              ///< Camera latitude in degrees
    float m_longitude;             ///< Camera longitude in degrees
    float m_altitude;              ///< Camera altitude in metres
    bool m_positionSync;           ///< Continually sync camera location from Main Settings
    QString m_owmAPIKey;           ///< API key for openweathermap.org weather updates
    float m_azimuth;               ///< Camera pointing azimuth in degrees
    float m_elevation;             ///< Camera pointing elevation in degrees
    float m_roll;                  ///< Camera roll about optical axis in degrees
    QString m_rotator;             ///< "<featureSetIndex>:<featureIndex>" of rotator to follow
    QString m_directionSensor;      ///< Qt Sensors compass identifier used to follow camera azimuth/elevation
    SensorOpticalAxis m_sensorOpticalAxis; ///< Camera optical axis relative to the Qt Sensors device coordinates
    bool m_directionSensorFilterEnabled; ///< Smooth Qt Sensors direction readings
    double m_directionSensorFilterTimeConstant; ///< Qt Sensors smoothing time constant in seconds
    bool m_directionApplyToCurrentImage; ///< Reproject the current frame when azimuth/elevation/roll changes
    float m_azimuthOffset;         ///< Offset added to synced rotator/sensor azimuth in degrees
    float m_elevationOffset;       ///< Offset added to synced rotator/sensor elevation in degrees
    float m_rollOffset;            ///< Offset added to synced sensor roll in degrees
    float m_fov;                   ///< Camera field of view in degrees
    FovMode m_fovMode;             ///< Whether FoV is entered directly or calculated from sensor/focal length
    double m_fovSensorWidthMm;     ///< Sensor width for calculated FoV
    double m_fovSensorHeightMm;    ///< Sensor height for calculated FoV
    double m_fovFocalLengthMm;     ///< Lens focal length for calculated FoV
    LensProjection m_lensProjection; ///< Lens projection model used for sky overlays
    double m_lensCenterOffsetX;    ///< Principal point X offset in pixels from image center
    double m_lensCenterOffsetY;    ///< Principal point Y offset in pixels from image center
    double m_lensDistortionK1;     ///< Radial distortion coefficient for plate solving / overlays
    bool m_lensMirror;             ///< Image is horizontally mirrored relative to the sky (up-looking all-sky camera, star diagonal). Plate solve flips detections; overlays reflect x.
    bool m_playbackProjectionEnabled; ///< Treat a rectangular sub-area of video/images playback as the optical image
    int m_playbackProjectionX;      ///< Playback optical content rectangle left position in pixels
    int m_playbackProjectionY;      ///< Playback optical content rectangle top position in pixels
    int m_playbackProjectionWidth;  ///< Playback optical content rectangle width in pixels
    int m_playbackProjectionHeight; ///< Playback optical content rectangle height in pixels
    Serializable *m_rollupState;
    int m_workspaceIndex;
    QByteArray m_geometryBytes;

    // Post-processing settings
    int m_postProcessWhiteBalanceMode; ///< 0=Off, 1=Auto, 2=Manual
    double m_postProcessWhiteBalanceRedGain;   ///< Manual red gain: 0.1..8.0
    double m_postProcessWhiteBalanceGreenGain; ///< Manual green gain: 0.1..8.0
    double m_postProcessWhiteBalanceBlueGain;  ///< Manual blue gain: 0.1..8.0
    double m_postProcessWhiteBalanceHighlightProtection; ///< Roll manual WB gains back toward neutral in highlights
    bool m_postProcessUseCuda; ///< Use OpenCV CUDA functions for supported camera processing steps
    bool m_postProcessUnwarp; ///< Unwarp fisheye images using the configured lens projection and FoV
    HistogramStretch m_histogramStretch; ///< Histogram stretch mode
    double m_histogramStretchBlackPoint; ///< Black point normalized to [0,1]
    double m_histogramStretchWhitePoint; ///< White point normalized to [0,1]
    double m_histogramStretchGamma;      ///< Gamma stretch exponent
    double m_histogramStretchAsinhStrength; ///< Asinh stretch strength
    double m_histogramStretchLogStrength;   ///< Log stretch strength
    bool m_histogramVisible; ///< Transient: compute histogram data only while the histogram dialog is visible
    bool m_histogramUseDetectionRoi; ///< Compute histogram data from the shared detection RoI
    bool m_histogramLogScale; ///< Display histogram counts on a logarithmic vertical axis
    bool m_postProcessGreyscale; ///< Convert the post-processed image to greyscale after white balance
    double m_saturation;      ///< Saturation multiplier: 0.0..3.0
    double m_gamma;           ///< Gamma correction exponent: 0.1..3.0
    int m_gaussianBlur;       ///< Gaussian blur strength: 0..15 (0 = off)
    int m_medianBlur;         ///< Median blur strength: 0..15 (0 = off)
    double m_sharpen;         ///< Sharpen amount: 0.0..3.0
    EdgeDisplayMode m_edgeDisplayMode; ///< Whether Sobel/Canny edges are overlaid or shown alone
    double m_sobelEdge;       ///< Sobel edge blend amount: 0.0..3.0
    double m_cannyEdge;       ///< Canny edge blend amount: 0.0..3.0
    bool m_flipX;             ///< Flip image horizontally
    bool m_flipY;             ///< Flip image vertically
    int m_imageRotation;      ///< Rotate image clockwise by 0, 90, 180, or 270 degrees
    double m_brightness;      ///< Brightness adjustment: -100.0..100.0
    double m_contrast;        ///< Contrast multiplier: 0.1..3.0
    bool m_invertColors;      ///< Invert all colour channels
    bool m_overlayDateTime;   ///< Draw current date/time on frame
    QColor m_dateTimeColor;   ///< Colour for the date/time overlay text
    QString m_dateTimeFormat; ///< QDateTime::toString format, or template text with a date/time format in {...}
    bool m_dateTimeUtc;       ///< Display date/time overlay in UTC instead of local time
    int m_dateTimePosX;       ///< X pixel offset from left for date/time text: 0..4096
    int m_dateTimePosY;       ///< Y pixel offset from top for date/time text: 0..4096 (0 = auto bottom)
    bool m_equatorialGrid;    ///< Draw equatorial sky grid overlay
    QColor m_equatorialGridColor; ///< Colour for equatorial sky grid
    bool m_altAzGrid;         ///< Draw alt-az sky grid overlay
    QColor m_altAzGridColor;  ///< Colour for alt-az sky grid
    bool m_constellation;    ///< Draw the selected constellation stars as projected boxes
    QColor m_constellationColor; ///< Colour for constellation star boxes
    ConstellationOverlay m_constellationOverlay; ///< Which constellation's major stars to project
    bool m_messier;          ///< Overlay Messier objects (marker at true angular size + name label) using the current pointing/solve
    QColor m_messierColor;   ///< Colour for Messier object markers and labels
    double m_messierMaxMagnitude; ///< Only overlay Messier objects at least this bright (visual magnitude)
    bool m_messierDetect;    ///< Photometrically check each Messier position (aperture vs local background) and dim objects not detected in the frame
    bool m_trackObjects;      ///< Draw ADS-B / satellite tracked object overlay
    bool m_trackObjectTrails; ///< Draw recent tracks for ADS-B / satellite tracked objects
    bool m_trackObjectHeatMap; ///< Draw a heat map from recent tracked object positions
    bool m_trackObjectRange; ///< Append range to tracked object labels
    double m_trackObjectMinElevation; ///< Minimum elevation in degrees for tracked object overlay
    double m_trackObjectMaxRangeKm; ///< Maximum range in km for tracked object overlay; 0 disables filtering
    TrackObjectLabelDisplay m_trackObjectLabelDisplay; ///< When tracked object labels are displayed
    double m_trackObjectLabelDetectionRadius; ///< Pixel distance from a detected object center for tracked object labels
    QColor m_trackObjectColor; ///< Colour for tracked object labels
    QString m_trackObjectFontFamily; ///< QPainter font family for tracked object labels
    double m_trackObjectFontScale; ///< Font point size for tracked object labels: 4.0..144.0
    QString m_gridLabelFontFamily; ///< QPainter font family for sky grid labels
    double  m_gridLabelFontScale;  ///< Font point size for sky grid labels: 4.0..144.0
    bool m_overlayText;       ///< Draw custom HTML text on frame
    QString m_overlayTextString; ///< HTML text content to render on the frame
    QColor m_overlayTextColor;   ///< Colour for the text overlay content
    QString m_overlayTextFontFamily; ///< QPainter font family for the text overlay
    double  m_overlayTextFontScale;  ///< Font point size for the text overlay: 4.0..144.0
    int m_overlayTextPosX;     ///< X pixel offset from left for text overlay: 0..4096
    int m_overlayTextPosY;     ///< Y pixel offset from top for text overlay: 0..4096 (0 = auto bottom)
    bool m_diffMask;          ///< Show pixel differences from previous frame
    int m_diffThreshold;      ///< Threshold for diff-mask binary classification: 0..255
    int m_diffMaskOpenSize;   ///< Kernel radius for morphological open on diff mask before accumulation: 0..20
    int m_dilationSize;       ///< Kernel radius for diff-mask dilation: 0..20
    int m_diffMaskHistoryFrames; ///< Number of diff masks to accumulate with bitwise_or: 1..120
    int m_diffMaskCloseSize;  ///< Kernel radius for morphological close on accumulated diff mask: 0..20
    QString m_overlayFontFamily; ///< QPainter font family name (empty = system default)
    double  m_overlayFontScale;  ///< Font point size for QPainter text: 4.0..144.0
    int    m_detectionRoiX;     ///< Detection ROI X origin in pixels; 0..4096
    int    m_detectionRoiY;     ///< Detection ROI Y origin in pixels; 0..4096
    int    m_detectionRoiWidth; ///< Detection ROI width in pixels; 0 disables ROI/full width
    int    m_detectionRoiHeight; ///< Detection ROI height in pixels; 0 disables ROI/full height
    bool   m_showDetectionRoi;  ///< Show detection ROI rectangle on the preview image
    bool   m_motionDetect;      ///< Enable motion background subtraction
    MotionBackgroundSubtractor m_motionBackgroundSubtractor; ///< Background subtractor algorithm
    MotionMaskView m_motionMaskView; ///< Optional debug view of motion fgMask stages
    int    m_motionHistory;     ///< Background subtractor history length: 1..5000
    double m_motionVarThreshold; ///< MOG2 variance or KNN distance threshold: 1.0..200.0
    double m_motionLearningRate; ///< Explicit background subtractor learning rate: -1.0 = auto, otherwise 0.0..1.0
    int    m_motionConfirmFrames; ///< Require motion this many consecutive frames before reporting: 1..60
    double m_motionDownscale;   ///< Downscale factor for motion detection path: 1.0, 0.5, 0.25
    bool   m_motionDetectShadows; ///< Enable MOG2 shadow detection
    int    m_motionOpenSize;    ///< Kernel radius for motion-mask morphological open: 0..20
    int    m_motionCloseSize;   ///< Kernel radius for motion-mask morphological close: 0..20
    int    m_motionPersistenceFrames; ///< Keep last motion boxes for this many frames after motion disappears: 0..120
    QColor m_motionBoxColor;    ///< Bounding box colour for motion contours
    int    m_minContourArea;    ///< Minimum contour area (px²) to draw: 0..10000
    bool   m_showMotionExclusionRects; ///< Show motion exclusion rectangles on the preview image
    QList<QRect> m_motionExclusionRects; ///< Full-resolution exclusion rectangles for diff/motion detection
    bool   m_cloudDetect;       ///< Enable cloud detection
    CloudDetectionMode m_cloudMode; ///< Cloud classification path: auto, day (R/B ratio) or night (background deviation)
    CloudDebugView m_cloudDebugView; ///< Optional debug view of cloud detection stages
    QColor m_cloudColor;        ///< Overlay tint colour for cloud-classified regions
    bool   m_cloudShowOverlay;  ///< Draw the cloud mask tint on the preview image
    double m_cloudDayThreshold; ///< Day path: minimum red/blue ratio classified as cloud: 0.0..2.0
    int    m_cloudTextureThreshold; ///< Day path: fine-scale texture level above which a region cannot be cloud (rejects roofs/trees); 0 disables: 0..255
    int    m_cloudNightThreshold; ///< Dark night path: how much brighter than the surrounding sky a region must be to classify as cloud: 0..255
    int    m_cloudBackgroundBlur; ///< Night path: radius used to estimate the sky background, in downscaled pixels: 1..128
    int    m_cloudOpenSize;     ///< Kernel radius for cloud-mask morphological open: 0..20
    int    m_cloudCloseSize;    ///< Kernel radius for cloud-mask morphological close: 0..20
    double m_cloudDownscale;    ///< Downscale factor for the cloud detection path: 1.0, 0.5, 0.25, 0.125
    int    m_cloudUpdateIntervalFrames; ///< Recompute the cloud mask every this many frames: 1..600
    bool   m_cloudFilterStars;  ///< Drop star detections whose centroid falls inside the cloud mask
    bool   m_cloudFilterMotion; ///< Suppress motion boxes that substantially overlap the cloud mask
    double m_cloudMotionOverlapThreshold; ///< Cloud overlap fraction at which a motion box is suppressed: 0.0..1.0
    double m_cloudEventThreshold; ///< Coverage percentage at which a Scheduler coverage-high event is emitted (low again min(10, threshold/2) points below): 0..100
    double m_cloudEdgeMarginPercent; ///< Exclude a margin this % of the frame in from the sky-region edge (fisheye rim / vignette / foreground): 0 disables, 0..25
    double m_cloudDayRelativeMargin; ///< Day path: how much whiter (red/blue ratio) than the clear sky at the same elevation counts as cloud; 0 disables: 0..1
    double m_cloudMinElevation;    ///< Exclude sky below this elevation in degrees from cloud evaluation (needs the lens pose), so coverage tracks the usable sky: 0 disables, 0..90
    bool   m_cloudUseDetectionRoi; ///< Restrict cloud detection to the shared detection RoI; when false the whole frame is evaluated
    bool   m_cloudMaskSunMoon;  ///< Exclude the projected sun (day) / moon (night) so their bright bloom is not classified as cloud
    double m_cloudSunMoonRadiusDeg; ///< Maximum angular radius in degrees the sun/moon exclusion grows to (dynamic bloom, capped here): 0..45
    bool   m_cloudStarSense;    ///< Veto cloud where predicted catalog stars are visible (needs calibrated pose, position and capture time)
    double m_cloudStarSenseMagnitude; ///< Faintest catalog star magnitude checked by star-visibility sensing: 0..6
    bool   m_cloudUseReference; ///< Compare against the saved per-camera clear-sky reference (veto static quirks, cue deviations, learned foreground)
    bool   m_cloudAutoReference; ///< Auto-learn the clear-sky reference from verified-clear frames
    bool   m_starDetect;        ///< Enable star detection
    int    m_starThreshold;     ///< Threshold on the star residual image
    int    m_starBackgroundBlur; ///< Radius used to estimate the smooth sky background
    int    m_starMinArea;       ///< Minimum star blob area in pixels
    int    m_starMaxArea;       ///< Maximum star blob area in pixels
    double m_starMaxAspectRatio; ///< Maximum accepted blob aspect ratio
    StarDebugView m_starDebugView; ///< Optional debug view of star detection stages
    QColor m_starColor;         ///< Overlay colour for detected stars
    bool   m_showStarDetectionBoxes; ///< Draw the small box marker around each detected star on the preview
    bool   m_plateSolve;        ///< Attempt to identify detected stars from a built-in bright-star catalog
    double m_plateSolveMaxMagnitude; ///< Faintest catalog star to consider during plate solving
    int    m_plateSolveMinMatches; ///< Minimum matches required for a successful solve
    double m_plateSolveMatchRadius; ///< Maximum image-space distance in pixels for acquisition/coarse star matching
    double m_plateSolveFinalMatchRadius; ///< Maximum image-space distance in pixels for final accepted star matches
    double m_plateSolveAzElSearchRadius; ///< Az/El search radius in degrees around the current pointing estimate when the chosen start mode uses it
    double m_plateSolveFovTolerance; ///< How well the entered FoV is known, as a percentage of FoV. 0 = exact (pin to the entered FoV); larger lets the solver refine the FoV away from the seed (for un-calibrated wide/fisheye lenses)
    PlateSolveStartMode m_plateSolveStartMode; ///< Which current camera settings should be used as starting inputs for plate solving
    PlateSolveLabelMode m_plateSolveLabelMode; ///< Which catalog metadata should be shown for solved stars
    bool   m_plateSolveLabelHideSyntheticNames; ///< Skip labels for stars that only have a synthetic Gaia coordinate name (keep real catalogue names)
    bool   m_plateSolveUseCaptureDateTime; ///< Use the frame capture date/time for plate solving instead of a fixed timestamp
    QDateTime m_plateSolveDateTime; ///< User-specified date/time for plate solving recorded media
    bool   m_plateSolveDateTimeUtc; ///< Treat the user-specified plate solve date/time as UTC instead of local time
    bool   m_plateSolveUseDownloadedCatalog; ///< Prefer the downloaded HYG catalog over the bundled catalog when available
    PlateSolveCatalogSource m_plateSolveCatalogSource; ///< Catalog source used for plate solving
    PlateSolveApplyMode m_plateSolveApplyMode; ///< Which solved values should be copied back into camera settings
    int    m_starCatalogDiskCacheSizeGb; ///< Maximum Siril/Gaia region disk cache size in GB; 0 = unlimited

    // Radiometric UVC thermal-camera settings
    ThermalDecoder m_thermalDecoder;
    ThermalPalette m_thermalPalette;
    ThermalUnits m_thermalUnits;
    bool m_thermalAutoRange;
    double m_thermalMinimumC;
    double m_thermalMaximumC;
    double m_thermalAutoLowPercentile;
    double m_thermalAutoHighPercentile;
    double m_thermalAutoRangeSmoothing;
    bool m_thermalMarkerEnabled;
    double m_thermalMarkerX;
    double m_thermalMarkerY;
    bool m_thermalShowMinMax;
    bool m_thermalChartEnabled;
    int m_thermalChartHistorySeconds;
    int m_thermalChartSampleIntervalMs;

    // Optical spectrum (diffraction grating spectroscopy) settings; opticalSpectrum keys kept for compatibility
    enum OpticalSpectrumDirection {
        OpticalSpectrumDirectionAuto,    ///< Determine the red end from the red/blue channel centroids
        OpticalSpectrumDirectionNormal,  ///< Wavelength increases with increasing pixel coordinate
        OpticalSpectrumDirectionFlipped  ///< Wavelength increases with decreasing pixel coordinate
    };
    bool   m_opticalSpectrumVisible;       ///< Transient: compute optical spectrum data only while the spectrum dialog is visible
    double m_opticalSpectrumDispersion;    ///< Wavelength calibration in nm/pixel; 0 = uncalibrated (axis shown in pixels)
    bool   m_opticalSpectrumZeroOrderAuto; ///< Auto-locate the zero-order star image as the strongest peak in the RoI
    double m_opticalSpectrumZeroOrderX;    ///< Manual zero-order position along the dispersion axis in image pixels
    OpticalSpectrumDirection m_opticalSpectrumDirection; ///< Which way wavelength increases along the dispersion axis
    int    m_opticalSpectrumApertureRows;  ///< Rows summed across the spectrum trace; 0 = full RoI height: 0..256
    bool   m_opticalSpectrumBackgroundSub; ///< Subtract the per-column sky background estimated outside the aperture
    int    m_opticalSpectrumSmoothing;     ///< Moving-average width in samples for display; 1 = off: 1..99
    int    m_opticalSpectrumAverageFrames; ///< Average the displayed profile over this many frames; 1 = off: 1..100
    bool   m_opticalSpectrumNormalize;     ///< Normalise the displayed profile to a peak of 1.0
    bool   m_opticalSpectrumColourStrip;   ///< Show a strip below the chart with the colour of each wavelength, brightness following intensity
    bool   m_opticalSpectrumImageStrip;    ///< Show a strip below the chart with the colours extracted from the image along the dispersion axis
    QString m_opticalSpectrumRefLines;     ///< Comma-separated reference line sets to overlay: balmer,hei,naca,o2
    QString m_opticalSpectrumCustomLines;  ///< User reference lines as "label:nm;label:nm"
    double m_opticalSpectrumRedshift;      ///< Redshift z applied to non-terrestrial reference lines: -0.9..10, 0 = rest
    QString m_opticalSpectrumReferenceTemplate; ///< Pickles library template key (e.g. "a0v") overlaid on the chart; empty = none
    bool   m_opticalSpectrumApplyResponse; ///< Divide the displayed luminance by the captured instrument response curve
    QString m_opticalSpectrumResponseFile; ///< Instrument response CSV path; empty = default (camera/spectrum-response.csv). Separate files suit different cameras/conditions
    bool   m_opticalSpectrumLogY;          ///< Plot the spectrum on a logarithmic intensity axis
    bool   m_opticalSpectrumAutoIdentify;  ///< Annotate detected spectral features with matching reference lines

    // Spectrum overlay settings
    bool   m_overlaySpectrum;   ///< Legacy single-spectrum enable flag; m_spectrumOverlays is the active model
    QString m_spectrumDevice;   ///< Legacy single-spectrum device; m_spectrumOverlays is the active model
    int    m_spectrumOffsetX;   ///< Legacy single-spectrum X offset; m_spectrumOverlays is the active model
    int    m_spectrumOffsetY;   ///< Legacy single-spectrum Y offset; m_spectrumOverlays is the active model
    double m_spectrumScale;     ///< Legacy single-spectrum scale; m_spectrumOverlays is the active model
    QList<SpectrumOverlay> m_spectrumOverlays; ///< Spectrum view captures to composite over the post-processed frame
    QList<WindowOverlay> m_windowOverlays; ///< GUI window or rollup captures to composite over the post-processed frame

    // Persistent image annotations. Coordinates are normalized to the image dimensions.
    bool m_drawingsEnabled;
    double m_drawingLineWidth;
    QColor m_drawingStrokeColor;
    bool m_drawingFillEnabled;
    QColor m_drawingFillColor;
    QString m_drawingFontFamily;
    int m_drawingFontPixelSize;
    bool m_drawingFontBold;
    bool m_drawingFontItalic;
    QList<CameraDrawing> m_drawings;

    // YOLO object-detection settings
    enum YoloInferenceMode {
        YoloInferenceScale,
        YoloInferenceTile,
        YoloInferenceTileAndScale
    };
    bool   m_yoloEnabled;        ///< Run YOLO ONNX/LiteRT inference and draw bounding boxes
    QString m_yoloModelPath;     ///< Path to the .onnx or .tflite model used for scaled/full-image inference
    QString m_yoloTileModelPath; ///< Optional .onnx or .tflite model used for tiled inference; empty = use m_yoloModelPath
    QString m_yoloLabelsPath;    ///< Path to a plain-text file with one class name per line (optional)
    double m_yoloConfThreshold;  ///< Minimum confidence to keep a detection: 0.0..1.0
    double m_yoloNmsThreshold;   ///< IoU threshold for non-maximum suppression: 0.0..1.0
    double m_yoloDisappearDebounce; ///< Seconds a class must remain absent before it is reported as disappeared: 0.0..60.0
    QColor m_yoloBoxColor;       ///< Bounding-box colour when no per-class colour is available
    QString m_yoloLabelFontFamily; ///< Font family for object detection labels
    double m_yoloLabelFontScale; ///< Font point size for object detection labels
    bool   m_yoloTileLargeImages; ///< Legacy mirror for m_yoloInferenceMode != YoloInferenceScale
    YoloInferenceMode m_yoloInferenceMode; ///< Scale full image, tile image, or run both and merge detections
    int    m_yoloTileOverlapPercent; ///< Tile overlap percentage for large-image YOLO inference: 0..90
    QStringList m_yoloIgnoredClassNames; ///< Object class labels to ignore after detection
    enum DNNTarget {
        CPU,
        CUDA,
        CUDA_FP16,
        TensorRT,
        TensorRT_FP16,
        Vulkan,
        LiteRT_CPU,
        LiteRT_GPU,
    } m_yoloDnnTarget;          ///< OpenCV DNN target backend for running the YOLO model

    // Audio settings (Qt camera only)
    bool   m_audioMute;          ///< When true, captured camera audio is silenced
    int    m_audioPreviewVolume; ///< Preview audio volume in percent: 0..100
    QString m_audioDeviceName;   ///< Name of the audio output device (empty = system default)

    // Qt camera image-control settings
    int    m_whiteBalanceMode;      ///< White balance mode index (QCamera::WhiteBalanceMode / QCameraImageProcessing::WhiteBalanceMode); 0 = Auto
    double m_exposureCompensation;  ///< Exposure compensation in EV steps: -2.0..2.0 (Qt 6 only)
    int    m_focusMode;             ///< Focus mode index (QCamera::FocusMode); 0 stored as FocusModeAuto (Qt 6 only)
    double m_focusDistance;         ///< Normalised focus distance: 0.0 (near) .. 1.0 (far/infinity) (Qt 6 manual focus only)
    double m_zoomFactor;            ///< Hardware optical zoom factor; 1.0 = no zoom

    bool m_useReverseAPI;
    QString m_reverseAPIAddress;
    uint16_t m_reverseAPIPort;
    uint16_t m_reverseAPIFeatureSetIndex;
    uint16_t m_reverseAPIFeatureIndex;

    CameraSettings();
    ~CameraSettings() = default;
    void resetToDefaults();
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }
    QByteArray serializeMotionExclusionRects(const QList<QRect>& rects) const;
    void deserializeMotionExclusionRects(const QByteArray& data, QList<QRect>& rects);
    void applySettings(const QStringList& settingsKeys, const CameraSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force=false) const;
    bool isAlpacaCamera() const;
    bool isAsiCamera() const;
    bool isQtCamera() const;
    bool isFileCamera() const;
    bool isVideoFileCamera() const;
    bool isImageFileSequenceCamera() const;
    bool isStreamCamera() const;
    bool isFfmpegMediaSource() const;
    bool hasFileCameraSource() const;
    QString ffmpegMediaSourcePath() const;
    int cameraIdInt() const;
    QString cameraIdString() const;
    QString cameraDescription() const;
    QString cameraDisplayName() const;
    bool isIntervalCaptureMode() const;
    double getCaptureIntervalSeconds() const;
    int getCaptureIntervalMs() const;
    double getCaptureFrameRate() const;
    static QString urlToFilename(const QString &url, const QString& destSubDir);
};

#endif // INCLUDE_FEATURE_CAMERASETTINGS_H_
