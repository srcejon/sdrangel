///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#include <QByteArray>
#include <QColor>
#include <QHash>
#include <QList>
#include <QString>
#include <QStringList>
#include <sstream>

class Serializable;

struct CameraSettings
{
    struct ObjectDeviceSettings
    {
        int m_deviceSetIndex;           //!< Device set index in SDRangel
        QString m_presetGroup;          //!< Preset group to load on object detection
        quint64 m_presetFrequency;      //!< Preset center frequency
        QString m_presetDescription;    //!< Preset description to identify the preset
        bool m_startOnDetect;           //!< Start acquisition when the object class is detected
        bool m_stopOnDisappear;         //!< Stop acquisition when the object class disappears
        bool m_startStopFileSink;       //!< Start/stop file sinks with detection/disappearance
        bool m_recordVideo;             //!< Start video recording on detection and stop when the object disappears
        QString m_detectCommand;        //!< Command/script to execute when the object class is detected
        QString m_disappearCommand;     //!< Command/script to execute when the object class disappears

        ObjectDeviceSettings();
        void getDebugString(std::ostringstream& ostr) const;
    };

    QString m_title;
    quint32 m_rgbColor;
    QString m_cameraId;
    int m_resolutionWidth;
    int m_resolutionHeight;
    int m_framesPerSecond;
    int m_exposureTimeMs;
    int m_isoSensitivity;
    QString m_alpacaHost;
    uint16_t m_alpacaPort;
    int m_alpacaBinX;
    int m_alpacaBinY;
    int m_alpacaGain;         // index into named gains list, or numeric value; -1 = do not set
    int m_alpacaOffset;       // index into named offsets list, or numeric value; -1 = do not set
    int m_alpacaReadoutMode;  // index into readoutmodes list
    bool m_saveImage;
    QString m_imageFileName;
    bool m_saveVideo;
    QString m_videoFileName;
    bool m_captureActive;
    Serializable *m_rollupState;
    int m_workspaceIndex;
    QByteArray m_geometryBytes;

    // Post-processing settings
    double m_brightness;      ///< Brightness adjustment: -100.0..100.0
    double m_contrast;        ///< Contrast multiplier: 0.1..3.0
    bool m_invertColors;      ///< Invert all colour channels
    bool m_overlayDateTime;   ///< Draw current date/time on frame
    QColor m_dateTimeColor;   ///< Colour for the date/time overlay text
    QString m_dateTimeFormat; ///< QDateTime::toString format string, e.g. "yyyy-MM-dd hh:mm:ss"
    int m_dateTimePosX;       ///< X pixel offset from left for date/time text: 0..4096
    int m_dateTimePosY;       ///< Y pixel offset from top for date/time text: 0..4096 (0 = auto bottom)
    bool m_overlayText;       ///< Draw custom HTML text on frame
    QString m_overlayTextString; ///< HTML text content to render on the frame
    QColor m_overlayTextColor;   ///< Colour for the text overlay content
    QString m_overlayTextFontFamily; ///< QPainter font family for the text overlay
    double  m_overlayTextFontScale;  ///< Font point size for the text overlay: 4.0..144.0
    int m_overlayTextPosX;     ///< X pixel offset from left for text overlay: 0..4096
    int m_overlayTextPosY;     ///< Y pixel offset from top for text overlay: 0..4096 (0 = auto bottom)
    bool m_diffMask;          ///< Show pixel differences from previous frame
    int m_dilationSize;       ///< Kernel radius for diff-mask dilation: 0..20
    QString m_overlayFontFamily; ///< QPainter font family name (empty = system default)
    double  m_overlayFontScale;  ///< Font point size for QPainter text: 4.0..144.0
    bool   m_motionDetect;      ///< Enable MOG2 background subtractor
    QColor m_motionBoxColor;    ///< Bounding box colour for motion contours
    int    m_minContourArea;    ///< Minimum contour area (px²) to draw: 0..10000
    bool   m_videoPostProcess;  ///< When true, write post-processed frames to video; when false, write raw frames

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
    QHash<QString, QList<ObjectDeviceSettings *> *> m_objectDeviceSettings; //!< Device control settings per YOLO class name

    // Audio settings (Qt camera only)
    bool   m_audioMute;          ///< When true, captured camera audio is silenced
    QString m_audioDeviceName;   ///< Name of the audio output device (empty = system default)

    // Qt camera image-control settings
    int    m_whiteBalanceMode;      ///< White balance mode index (QCamera::WhiteBalanceMode / QCameraImageProcessing::WhiteBalanceMode); 0 = Auto
    double m_exposureCompensation;  ///< Exposure compensation in EV steps: -2.0..2.0 (Qt 6 only)
    int    m_focusMode;             ///< Focus mode index (QCamera::FocusMode); 0 stored as FocusModeAuto (Qt 6 only)
    double m_focusDistance;         ///< Normalised focus distance: 0.0 (near) .. 1.0 (far/infinity) (Qt 6 manual focus only)
    double m_zoomFactor;            ///< Hardware optical zoom factor; 1.0 = no zoom

    CameraSettings();
    ~CameraSettings() = default;
    void resetToDefaults();
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }
    QByteArray serializeObjectDeviceSettings(QHash<QString, QList<ObjectDeviceSettings *> *> objectDeviceSettings) const;
    void deserializeObjectDeviceSettings(const QByteArray& data, QHash<QString, QList<ObjectDeviceSettings *> *>& objectDeviceSettings);
    void applySettings(const QStringList& settingsKeys, const CameraSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force=false) const;
    bool isAlpacaCamera() const;
    bool isQtCamera() const;
    int cameraIdInt() const;
    QString cameraIdString() const;
    QString cameraDescription() const;
};

#endif // INCLUDE_FEATURE_CAMERASETTINGS_H_
