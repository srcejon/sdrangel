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

#include <QLibrary>
#include <QStringList>

#include "asicamera2api.h"

namespace
{
    using ASI_BOOL = int;
    using ASI_IMG_TYPE = int;
    using ASI_CONTROL_TYPE = int;
    using ASI_ERROR_CODE = int;

    constexpr ASI_ERROR_CODE ASI_SUCCESS = 0;

    struct ASI_CAMERA_INFO
    {
        char Name[64];
        int CameraID;
        long MaxHeight;
        long MaxWidth;
        ASI_BOOL IsColorCam;
        int BayerPattern;
        int SupportedBins[16];
        ASI_IMG_TYPE SupportedVideoFormat[8];
        double PixelSize;
        ASI_BOOL MechanicalShutter;
        ASI_BOOL ST4Port;
        ASI_BOOL IsCoolerCam;
        ASI_BOOL IsUSB3Host;
        ASI_BOOL IsUSB3Camera;
        float ElecPerADU;
        int BitDepth;
        ASI_BOOL IsTriggerCam;
    };

    struct ASI_CONTROL_CAPS
    {
        char Name[64];
        char Description[128];
        long MaxValue;
        long MinValue;
        long DefaultValue;
        ASI_BOOL IsAutoSupported;
        ASI_BOOL IsWritable;
        ASI_CONTROL_TYPE ControlType;
        char Unused[32];
    };

    class ApiStorage
    {
    public:
        QLibrary m_library;
        mutable bool m_loadAttempted = false;
        mutable bool m_loaded = false;

        using FnGetNumOfConnectedCameras = int (*)();
        using FnGetCameraProperty = ASI_ERROR_CODE (*)(ASI_CAMERA_INFO*, int);
        using FnOpenCamera = ASI_ERROR_CODE (*)(int);
        using FnInitCamera = ASI_ERROR_CODE (*)(int);
        using FnCloseCamera = ASI_ERROR_CODE (*)(int);
        using FnGetNumOfControls = ASI_ERROR_CODE (*)(int, int*);
        using FnGetControlCaps = ASI_ERROR_CODE (*)(int, int, ASI_CONTROL_CAPS*);
        using FnSetControlValue = ASI_ERROR_CODE (*)(int, ASI_CONTROL_TYPE, long, ASI_BOOL);
        using FnGetControlValue = ASI_ERROR_CODE (*)(int, ASI_CONTROL_TYPE, long*, ASI_BOOL*);
        using FnSetROIFormat = ASI_ERROR_CODE (*)(int, int, int, int, ASI_IMG_TYPE);
        using FnSetStartPos = ASI_ERROR_CODE (*)(int, int, int);
        using FnStartVideoCapture = ASI_ERROR_CODE (*)(int);
        using FnStopVideoCapture = ASI_ERROR_CODE (*)(int);
        using FnGetVideoData = ASI_ERROR_CODE (*)(int, unsigned char*, long, int);

        FnGetNumOfConnectedCameras m_getNumOfConnectedCameras = nullptr;
        FnGetCameraProperty m_getCameraProperty = nullptr;
        FnOpenCamera m_openCamera = nullptr;
        FnInitCamera m_initCamera = nullptr;
        FnCloseCamera m_closeCamera = nullptr;
        FnGetNumOfControls m_getNumOfControls = nullptr;
        FnGetControlCaps m_getControlCaps = nullptr;
        FnSetControlValue m_setControlValue = nullptr;
        FnGetControlValue m_getControlValue = nullptr;
        FnSetROIFormat m_setROIFormat = nullptr;
        FnSetStartPos m_setStartPos = nullptr;
        FnStartVideoCapture m_startVideoCapture = nullptr;
        FnStopVideoCapture m_stopVideoCapture = nullptr;
        FnGetVideoData m_getVideoData = nullptr;
    };

    ApiStorage& storage()
    {
        static ApiStorage s_storage;
        return s_storage;
    }

    template<typename Function>
    Function resolveFunction(QLibrary& library, const char *name)
    {
        return reinterpret_cast<Function>(library.resolve(name));
    }

    QStringList libraryCandidates()
    {
        return {
            QStringLiteral("ASICamera2"),
            QStringLiteral("C:/Program Files/ASIStudio/ASICamera2.dll"),
            QStringLiteral("C:/Program Files (x86)/ASIStudio/ASICamera2.dll")
        };
    }
}

namespace AsiCamera2
{
Api& Api::instance()
{
    static Api s_api;
    return s_api;
}

Api::Api()
{
    load();
}

bool Api::load()
{
    ApiStorage& s = storage();

    if (s.m_loadAttempted) {
        return s.m_loaded;
    }

    s.m_loadAttempted = true;

    for (const QString& candidate : libraryCandidates())
    {
        s.m_library.setFileName(candidate);

        if (!s.m_library.load()) {
            continue;
        }

        s.m_getNumOfConnectedCameras = resolveFunction<ApiStorage::FnGetNumOfConnectedCameras>(s.m_library, "ASIGetNumOfConnectedCameras");
        s.m_getCameraProperty = resolveFunction<ApiStorage::FnGetCameraProperty>(s.m_library, "ASIGetCameraProperty");
        s.m_openCamera = resolveFunction<ApiStorage::FnOpenCamera>(s.m_library, "ASIOpenCamera");
        s.m_initCamera = resolveFunction<ApiStorage::FnInitCamera>(s.m_library, "ASIInitCamera");
        s.m_closeCamera = resolveFunction<ApiStorage::FnCloseCamera>(s.m_library, "ASICloseCamera");
        s.m_getNumOfControls = resolveFunction<ApiStorage::FnGetNumOfControls>(s.m_library, "ASIGetNumOfControls");
        s.m_getControlCaps = resolveFunction<ApiStorage::FnGetControlCaps>(s.m_library, "ASIGetControlCaps");
        s.m_setControlValue = resolveFunction<ApiStorage::FnSetControlValue>(s.m_library, "ASISetControlValue");
        s.m_getControlValue = resolveFunction<ApiStorage::FnGetControlValue>(s.m_library, "ASIGetControlValue");
        s.m_setROIFormat = resolveFunction<ApiStorage::FnSetROIFormat>(s.m_library, "ASISetROIFormat");
        s.m_setStartPos = resolveFunction<ApiStorage::FnSetStartPos>(s.m_library, "ASISetStartPos");
        s.m_startVideoCapture = resolveFunction<ApiStorage::FnStartVideoCapture>(s.m_library, "ASIStartVideoCapture");
        s.m_stopVideoCapture = resolveFunction<ApiStorage::FnStopVideoCapture>(s.m_library, "ASIStopVideoCapture");
        s.m_getVideoData = resolveFunction<ApiStorage::FnGetVideoData>(s.m_library, "ASIGetVideoData");

        s.m_loaded =
            s.m_getNumOfConnectedCameras
            && s.m_getCameraProperty
            && s.m_openCamera
            && s.m_initCamera
            && s.m_closeCamera
            && s.m_getNumOfControls
            && s.m_getControlCaps
            && s.m_setControlValue
            && s.m_getControlValue
            && s.m_setROIFormat
            && s.m_setStartPos
            && s.m_startVideoCapture
            && s.m_stopVideoCapture
            && s.m_getVideoData;

        if (s.m_loaded) {
            return true;
        }

        s.m_library.unload();
    }

    return false;
}

bool Api::ensureLoaded() const
{
    return const_cast<Api*>(this)->load();
}

bool Api::isAvailable() const
{
    return ensureLoaded();
}

QString Api::libraryPath() const
{
    const ApiStorage& s = storage();
    return s.m_library.fileName();
}

QString Api::errorString(int errorCode) const
{
    switch (errorCode)
    {
    case 0: return QStringLiteral("Success");
    case 1: return QStringLiteral("Invalid camera id");
    case 2: return QStringLiteral("Camera removed");
    case 3: return QStringLiteral("Camera closed");
    case 4: return QStringLiteral("Camera opened");
    case 5: return QStringLiteral("Invalid control type");
    case 6: return QStringLiteral("Invalid control value");
    case 7: return QStringLiteral("Invalid image type");
    case 8: return QStringLiteral("Invalid size");
    case 9: return QStringLiteral("Invalid mode");
    case 10: return QStringLiteral("Transfer error");
    case 11: return QStringLiteral("Timeout");
    case 12: return QStringLiteral("Invalid sequence");
    case 13: return QStringLiteral("Buffer too small");
    case 14: return QStringLiteral("Video mode active");
    case 15: return QStringLiteral("Exposure in progress");
    case 16: return QStringLiteral("General error");
    default: return QStringLiteral("ASI error %1").arg(errorCode);
    }
}

QVector<CameraInfo> Api::enumerateCameras() const
{
    QVector<CameraInfo> cameras;
    if (!ensureLoaded()) {
        return cameras;
    }

    const ApiStorage& s = storage();
    const int count = s.m_getNumOfConnectedCameras ? s.m_getNumOfConnectedCameras() : 0;

    for (int index = 0; index < count; ++index)
    {
        ASI_CAMERA_INFO rawInfo {};
        if (s.m_getCameraProperty(&rawInfo, index) != ASI_SUCCESS) {
            continue;
        }

        CameraInfo cameraInfo;
        cameraInfo.m_cameraId = rawInfo.CameraID;
        cameraInfo.m_name = QString::fromLocal8Bit(rawInfo.Name);
        cameraInfo.m_maxWidth = static_cast<int>(rawInfo.MaxWidth);
        cameraInfo.m_maxHeight = static_cast<int>(rawInfo.MaxHeight);
        cameraInfo.m_isColor = rawInfo.IsColorCam != 0;
        cameraInfo.m_bayerPattern = rawInfo.BayerPattern;
        cameraInfo.m_pixelSizeUm = rawInfo.PixelSize;
        cameraInfo.m_bitDepth = rawInfo.BitDepth;
        for (int bin : rawInfo.SupportedBins)
        {
            if (bin <= 0) {
                break;
            }
            cameraInfo.m_supportedBins.append(bin);
        }
        for (ASI_IMG_TYPE imageType : rawInfo.SupportedVideoFormat)
        {
            if (imageType < 0) {
                break;
            }
            cameraInfo.m_supportedImageTypes.append(imageType);
        }
        cameras.append(cameraInfo);
    }

    return cameras;
}

bool Api::getCameraInfo(int cameraId, CameraInfo& cameraInfo) const
{
    if (!ensureLoaded()) {
        return false;
    }

    const ApiStorage& s = storage();
    ASI_CAMERA_INFO rawInfo {};
    const int count = s.m_getNumOfConnectedCameras ? s.m_getNumOfConnectedCameras() : 0;
    bool found = false;

    for (int index = 0; index < count; ++index)
    {
        ASI_CAMERA_INFO candidate {};

        if (s.m_getCameraProperty(&candidate, index) != ASI_SUCCESS) {
            continue;
        }

        if (candidate.CameraID == cameraId)
        {
            rawInfo = candidate;
            found = true;
            break;
        }
    }

    if (!found) {
        return false;
    }

    cameraInfo.m_cameraId = rawInfo.CameraID;
    cameraInfo.m_name = QString::fromLocal8Bit(rawInfo.Name);
    cameraInfo.m_maxWidth = static_cast<int>(rawInfo.MaxWidth);
    cameraInfo.m_maxHeight = static_cast<int>(rawInfo.MaxHeight);
    cameraInfo.m_isColor = rawInfo.IsColorCam != 0;
    cameraInfo.m_bayerPattern = rawInfo.BayerPattern;
    cameraInfo.m_pixelSizeUm = rawInfo.PixelSize;
    cameraInfo.m_bitDepth = rawInfo.BitDepth;
    cameraInfo.m_supportedBins.clear();
    cameraInfo.m_supportedImageTypes.clear();

    for (int bin : rawInfo.SupportedBins)
    {
        if (bin <= 0) {
            break;
        }
        cameraInfo.m_supportedBins.append(bin);
    }

    for (ASI_IMG_TYPE imageType : rawInfo.SupportedVideoFormat)
    {
        if (imageType < 0) {
            break;
        }
        cameraInfo.m_supportedImageTypes.append(imageType);
    }

    return true;
}

bool Api::openCamera(int cameraId) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_openCamera(cameraId) == ASI_SUCCESS;
}

bool Api::initCamera(int cameraId) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_initCamera(cameraId) == ASI_SUCCESS;
}

void Api::closeCamera(int cameraId) const
{
    if (!ensureLoaded()) {
        return;
    }

    storage().m_closeCamera(cameraId);
}

bool Api::getControlRange(int cameraId, ControlType controlType, ControlRange& range) const
{
    range = ControlRange();

    if (!ensureLoaded()) {
        return false;
    }

    const ApiStorage& s = storage();
    int numControls = 0;

    if (s.m_getNumOfControls(cameraId, &numControls) != ASI_SUCCESS) {
        return false;
    }

    for (int index = 0; index < numControls; ++index)
    {
        ASI_CONTROL_CAPS caps {};

        if (s.m_getControlCaps(cameraId, index, &caps) != ASI_SUCCESS) {
            continue;
        }

        if (caps.ControlType == static_cast<ASI_CONTROL_TYPE>(controlType))
        {
            range.m_available = true;
            range.m_minValue = caps.MinValue;
            range.m_maxValue = caps.MaxValue;
            range.m_defaultValue = caps.DefaultValue;
            range.m_autoSupported = caps.IsAutoSupported != 0;
            range.m_writable = caps.IsWritable != 0;
            return true;
        }
    }

    return false;
}

bool Api::setControlValue(int cameraId, ControlType controlType, long value, bool autoValue) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_setControlValue(cameraId, static_cast<ASI_CONTROL_TYPE>(controlType), value, autoValue ? 1 : 0) == ASI_SUCCESS;
}

bool Api::getControlValue(int cameraId, ControlType controlType, long& value, bool& autoValue) const
{
    value = 0;
    autoValue = false;

    if (!ensureLoaded()) {
        return false;
    }

    ASI_BOOL rawAuto = 0;
    if (storage().m_getControlValue(cameraId, static_cast<ASI_CONTROL_TYPE>(controlType), &value, &rawAuto) != ASI_SUCCESS) {
        return false;
    }

    autoValue = (rawAuto != 0);
    return true;
}

bool Api::setRoiFormat(int cameraId, long width, long height, int bin, ImageType imageType) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_setROIFormat(cameraId, static_cast<int>(width), static_cast<int>(height), bin, static_cast<ASI_IMG_TYPE>(imageType)) == ASI_SUCCESS;
}

bool Api::setStartPos(int cameraId, long startX, long startY) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_setStartPos(cameraId, static_cast<int>(startX), static_cast<int>(startY)) == ASI_SUCCESS;
}

bool Api::startVideoCapture(int cameraId) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_startVideoCapture(cameraId) == ASI_SUCCESS;
}

bool Api::stopVideoCapture(int cameraId) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_stopVideoCapture(cameraId) == ASI_SUCCESS;
}

bool Api::getVideoData(int cameraId, unsigned char *buffer, long bufferSize, int waitMs) const
{
    if (!ensureLoaded()) {
        return false;
    }

    return storage().m_getVideoData(cameraId, buffer, bufferSize, waitMs) == ASI_SUCCESS;
}

} // namespace AsiCamera2
