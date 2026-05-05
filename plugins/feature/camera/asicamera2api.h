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

#ifndef INCLUDE_FEATURE_ASICAMERA2API_H_
#define INCLUDE_FEATURE_ASICAMERA2API_H_

#include <QString>
#include <QVector>

namespace AsiCamera2
{
    enum ImageType
    {
        ImageTypeRaw8 = 0,
        ImageTypeRgb24 = 1,
        ImageTypeRaw16 = 2,
        ImageTypeY8 = 3
    };

    enum BayerPattern
    {
        BayerRG = 0,
        BayerBG = 1,
        BayerGR = 2,
        BayerGB = 3
    };

    enum ControlType
    {
        ControlGain = 0,
        ControlExposure = 1,
        ControlGamma = 2,
        ControlWhiteBalanceRed = 3,
        ControlWhiteBalanceBlue = 4,
        ControlOffset = 5,
        ControlBandwidthOverload = 6,
        ControlOverclock = 7,
        ControlTemperature = 8,
        ControlFlip = 9,
        ControlAutoMaxGain = 10,
        ControlAutoMaxExposure = 11,
        ControlAutoTargetBrightness = 12,
        ControlHardwareBin = 13,
        ControlHighSpeedMode = 14,
        ControlCoolerPowerPercent = 15,
        ControlTargetTemperature = 16,
        ControlCoolerOn = 17,
        ControlMonoBin = 18,
        ControlFanOn = 19,
        ControlPatternAdjust = 20,
        ControlAntiDewHeater = 21
    };

    struct CameraInfo
    {
        int m_cameraId = -1;
        QString m_name;
        int m_maxWidth = 0;
        int m_maxHeight = 0;
        bool m_isColor = false;
        int m_bayerPattern = BayerRG;
        QVector<int> m_supportedBins;
        QVector<int> m_supportedImageTypes;
        double m_pixelSizeUm = 0.0;
        int m_bitDepth = 8;
    };

    struct ControlRange
    {
        bool m_available = false;
        long m_minValue = 0;
        long m_maxValue = 0;
        long m_defaultValue = 0;
        bool m_autoSupported = false;
        bool m_writable = false;
    };

    class Api
    {
    public:
        static Api& instance();

        bool isAvailable() const;
        QString libraryPath() const;
        QString errorString(int errorCode) const;

        QVector<CameraInfo> enumerateCameras() const;
        bool getCameraInfo(int cameraId, CameraInfo& cameraInfo) const;
        bool openCamera(int cameraId) const;
        bool initCamera(int cameraId) const;
        void closeCamera(int cameraId) const;

        bool getControlRange(int cameraId, ControlType controlType, ControlRange& range) const;
        bool setControlValue(int cameraId, ControlType controlType, long value, bool autoValue = false) const;
        bool getControlValue(int cameraId, ControlType controlType, long& value, bool& autoValue) const;

        bool setRoiFormat(int cameraId, long width, long height, int bin, ImageType imageType) const;
        bool setStartPos(int cameraId, long startX, long startY) const;
        bool startVideoCapture(int cameraId) const;
        bool stopVideoCapture(int cameraId) const;
        bool getVideoData(int cameraId, unsigned char *buffer, long bufferSize, int waitMs) const;

    private:
        Api();
        ~Api() = default;
        Api(const Api&) = delete;
        Api& operator=(const Api&) = delete;

        bool load();
        bool ensureLoaded() const;
    };
}

#endif // INCLUDE_FEATURE_ASICAMERA2API_H_
