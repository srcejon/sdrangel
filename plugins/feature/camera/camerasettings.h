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

class Serializable;

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

    enum AsiColorImageType
    {
        AsiColorImageTypeRgb24 = 0,
        AsiColorImageTypeRaw16,
        AsiColorImageTypeRaw8
    };

    enum SavedMediaMode
    {
        SavedMediaRaw = 0,
        SavedMediaProcessed,
        SavedMediaBoth
    };

    enum LensProjection
    {
        LensProjectionRectilinear = 0,
        LensProjectionEquidistant,
        LensProjectionEquisolid
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

    enum ConstellationOverlay
    {
        ConstellationOverlayUrsaMajor = 0,
        ConstellationOverlayOrion,
        ConstellationOverlayCrux
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
        PlateSolveLabelName = 0,
        PlateSolveLabelNameMagnitude,
        PlateSolveLabelNameMagnitudeSpectralType
    };

    enum PlateSolveCatalogSource
    {
        PlateSolveCatalogAuto = 0,
        PlateSolveCatalogHyg,
        PlateSolveCatalogSirilSpccGaia
    };

    enum FovMode
    {
        FovModeDirect = 0,
        FovModeSensorFocalLength
    };

    struct ObjectDeviceSettings
    {
        int m_deviceSetIndex;           //!< Device set index in SDRangel
        QString m_presetGroup;          //!< Preset group to load on object detection
        quint64 m_presetFrequency;      //!< Preset center frequency
        QString m_presetDescription;    //!< Preset description to identify the preset
        bool m_startOnDetect;           //!< Start acquisition when the object class is detected
        bool m_stopOnDisappear;         //!< Stop acquisition when the object class disappears
        bool m_startStopFileSink;       //!< Start/stop file sinks with detection/disappearance
        bool m_saveCurrentImage;        //!< Save a single image when the object class is detected
        bool m_recordVideo;             //!< Start video recording on detection and stop when the object disappears
        QString m_detectCommand;        //!< Command/script to execute when the object class is detected
        QString m_disappearCommand;     //!< Command/script to execute when the object class disappears
        QString m_detectSpeech;         //!< Speech to say when the object class is detected
        QString m_disappearSpeech;      //!< Speech to say when the object class disappears

        ObjectDeviceSettings();
        void getDebugString(std::ostringstream& ostr) const;
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
    static constexpr double m_minStarAspectRatio = 1.0;
    static constexpr double m_maxStarAspectRatio = 10.0;
    static constexpr double m_minPlateSolveMagnitude = -2.0;
    static constexpr double m_maxPlateSolveMagnitude = 11.0;
    static constexpr int m_minPlateSolveMatches = 2;
    static constexpr int m_maxPlateSolveMatches = 32;
    static constexpr double m_minPlateSolveMatchRadius = 1.0;
    static constexpr double m_maxPlateSolveMatchRadius = 256.0;
    static constexpr double m_minPlateSolveSearchRadius = 0.0;
    static constexpr double m_maxPlateSolveSearchRadius = 45.0;
    static constexpr qint64 m_minPlateSolveDateTimeMs = 0;
    static constexpr double m_minLensCenterOffset = -2048.0;
    static constexpr double m_maxLensCenterOffset = 2048.0;
    static constexpr double m_minLensDistortionK1 = -1.0;
    static constexpr double m_maxLensDistortionK1 = 1.0;
    static constexpr double m_minSpectrumScale = 0.1;
    static constexpr double m_maxSpectrumScale = 4.0;
    static constexpr double m_minYoloDisappearDebounce = 0.0;
    static constexpr double m_maxYoloDisappearDebounce = 60.0;
    static constexpr double m_minExposureCompensation = -2.0;
    static constexpr double m_maxExposureCompensation = 2.0;
    static constexpr double m_minZoomFactor = 1.0;
    static constexpr double m_minVideoPlaybackRate = 0.1;
    static constexpr double m_maxVideoPlaybackRate = 10.0;

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
    AsiColorImageType m_asiColorImageType;
    bool m_saveImage;
    QString m_imageFileName;
    bool m_saveVideo;
    QString m_videoFileCameraPath;
    QString m_videoFileName;
    bool m_videoLoop;
    double m_videoPlaybackRate;
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
    float m_fov;                   ///< Camera field of view in degrees
    FovMode m_fovMode;             ///< Whether FoV is entered directly or calculated from sensor/focal length
    double m_fovSensorWidthMm;     ///< Sensor width for calculated FoV
    double m_fovSensorHeightMm;    ///< Sensor height for calculated FoV
    double m_fovFocalLengthMm;     ///< Lens focal length for calculated FoV
    LensProjection m_lensProjection; ///< Lens projection model used for sky overlays
    double m_lensCenterOffsetX;    ///< Principal point X offset in pixels from image center
    double m_lensCenterOffsetY;    ///< Principal point Y offset in pixels from image center
    double m_lensDistortionK1;     ///< Radial distortion coefficient for plate solving / overlays
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
    QString m_dateTimeFormat; ///< QDateTime::toString format string, e.g. "yyyy-MM-dd hh:mm:ss"
    int m_dateTimePosX;       ///< X pixel offset from left for date/time text: 0..4096
    int m_dateTimePosY;       ///< Y pixel offset from top for date/time text: 0..4096 (0 = auto bottom)
    bool m_equatorialGrid;    ///< Draw equatorial sky grid overlay
    QColor m_equatorialGridColor; ///< Colour for equatorial sky grid
    bool m_altAzGrid;         ///< Draw alt-az sky grid overlay
    QColor m_altAzGridColor;  ///< Colour for alt-az sky grid
    bool m_constellation;    ///< Draw the selected constellation stars as projected boxes
    QColor m_constellationColor; ///< Colour for constellation star boxes
    ConstellationOverlay m_constellationOverlay; ///< Which constellation's major stars to project
    bool m_trackObjects;      ///< Draw ADS-B / satellite tracked object overlay
    double m_trackObjectMinElevation; ///< Minimum elevation in degrees for tracked object overlay
    QColor m_trackObjectColor; ///< Colour for tracked object labels
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
    bool   m_starDetect;        ///< Enable star detection
    int    m_starThreshold;     ///< Threshold on the star residual image
    int    m_starBackgroundBlur; ///< Radius used to estimate the smooth sky background
    int    m_starMinArea;       ///< Minimum star blob area in pixels
    int    m_starMaxArea;       ///< Maximum star blob area in pixels
    double m_starMaxAspectRatio; ///< Maximum accepted blob aspect ratio
    StarDebugView m_starDebugView; ///< Optional debug view of star detection stages
    QColor m_starColor;         ///< Overlay colour for detected stars
    bool   m_plateSolve;        ///< Attempt to identify detected stars from a built-in bright-star catalog
    double m_plateSolveMaxMagnitude; ///< Faintest catalog star to consider during plate solving
    int    m_plateSolveMinMatches; ///< Minimum matches required for a successful solve
    double m_plateSolveMatchRadius; ///< Maximum image-space distance in pixels for acquisition/coarse star matching
    double m_plateSolveFinalMatchRadius; ///< Maximum image-space distance in pixels for final accepted star matches
    double m_plateSolveSearchRadius; ///< Search radius in degrees around the current pointing estimate when the chosen start mode uses it
    PlateSolveStartMode m_plateSolveStartMode; ///< Which current camera settings should be used as starting inputs for plate solving
    PlateSolveLabelMode m_plateSolveLabelMode; ///< Which catalog metadata should be shown for solved stars
    bool   m_plateSolveUseCurrentDateTime; ///< Use the current system date/time for plate solving instead of a fixed timestamp
    QDateTime m_plateSolveDateTime; ///< User-specified date/time for plate solving recorded media
    bool   m_plateSolveDateTimeUtc; ///< Treat the user-specified plate solve date/time as UTC instead of local time
    bool   m_plateSolveUseDownloadedCatalog; ///< Prefer the downloaded HYG catalog over the bundled catalog when available
    PlateSolveCatalogSource m_plateSolveCatalogSource; ///< Catalog source used for plate solving
    PlateSolveApplyMode m_plateSolveApplyMode; ///< Which solved values should be copied back into camera settings
    SavedMediaMode m_recordMode;  ///< Save raw frames, post-processed frames, or both

    // Spectrum overlay settings
    bool   m_overlaySpectrum;   ///< Enable overlaying the spectrum view image on the post-processed frame
    QString m_spectrumDevice;   ///< Long ID of the device whose spectrum view to overlay (e.g. "R0 HackRF")
    int    m_spectrumOffsetX;   ///< X offset (px) for the top-left corner of the spectrum overlay: -4096..4096
    int    m_spectrumOffsetY;   ///< Y offset (px) for the top-left corner of the spectrum overlay: -4096..4096
    double m_spectrumScale;     ///< Scale factor applied to the spectrum image before compositing: 0.1..4.0

    // YOLO object-detection settings
    bool   m_yoloEnabled;        ///< Run YOLO ONNX inference and draw bounding boxes
    QString m_yoloModelPath;     ///< Path to the .onnx model file
    QString m_yoloLabelsPath;    ///< Path to a plain-text file with one class name per line (optional)
    double m_yoloConfThreshold;  ///< Minimum confidence to keep a detection: 0.0..1.0
    double m_yoloNmsThreshold;   ///< IoU threshold for non-maximum suppression: 0.0..1.0
    QColor m_yoloBoxColor;       ///< Bounding-box colour when no per-class colour is available
    double m_yoloDisappearDebounce; ///< Seconds a class must remain absent before it is treated as disappeared
    bool   m_yoloTileLargeImages; ///< Tile frames larger than the YOLO input size and batch infer the tiles
    int    m_yoloTileOverlapPercent; ///< Tile overlap percentage for large-image YOLO inference: 0..90
    QHash<QString, QList<ObjectDeviceSettings *> *> m_objectDeviceSettings; //!< Device control settings per YOLO class name
    enum DNNTarget {
        CPU,
        CUDA,
        CUDA_FP16,
    } m_yoloDnnTarget;          ///< OpenCV DNN target backend for running the YOLO model

    // Audio settings (Qt camera only)
    bool   m_audioMute;          ///< When true, captured camera audio is silenced
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
    QByteArray serializeObjectDeviceSettings(QHash<QString, QList<ObjectDeviceSettings *> *> objectDeviceSettings) const;
    void deserializeObjectDeviceSettings(const QByteArray& data, QHash<QString, QList<ObjectDeviceSettings *> *>& objectDeviceSettings);
    QByteArray serializeMotionExclusionRects(const QList<QRect>& rects) const;
    void deserializeMotionExclusionRects(const QByteArray& data, QList<QRect>& rects);
    void applySettings(const QStringList& settingsKeys, const CameraSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force=false) const;
    bool isAlpacaCamera() const;
    bool isAsiCamera() const;
    bool isQtCamera() const;
    bool isFileCamera() const;
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
