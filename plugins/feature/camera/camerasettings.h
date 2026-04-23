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
#include <QString>
#include <QStringList>

class Serializable;

struct CameraSettings
{
    enum CameraAPI
    {
        CameraAPIAlpaca = 0,
        CameraAPIQtCamera = 1
    };

    QString m_title;
    quint32 m_rgbColor;
    CameraAPI m_cameraAPI;
    QString m_cameraId;
    int m_resolutionWidth;
    int m_resolutionHeight;
    int m_framesPerSecond;
    int m_exposureTimeMs;
    int m_isoSensitivity;
    QString m_alpacaHost;
    uint16_t m_alpacaPort;
    int m_alpacaCameraId;
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
    bool m_diffMask;          ///< Show pixel differences from previous frame
    int m_dilationSize;       ///< Kernel radius for diff-mask dilation: 0..20
    int    m_overlayFontIndex;  ///< OpenCV Hershey font index: 0..7
    double m_overlayFontScale;  ///< Font scale for cv::putText: 0.3..3.0
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

    CameraSettings();
    ~CameraSettings() = default;
    void resetToDefaults();
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }
    void applySettings(const QStringList& settingsKeys, const CameraSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force=false) const;
};

#endif // INCLUDE_FEATURE_CAMERASETTINGS_H_
