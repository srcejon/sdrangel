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

#ifndef INCLUDE_FEATURE_CAMERAALPACACONTROLLER_H_
#define INCLUDE_FEATURE_CAMERAALPACACONTROLLER_H_

#include <QByteArray>
#include <QElapsedTimer>
#include <QImage>
#include <QStringList>
#include <QUrl>
#include <QString>
#include <QVector>

#include <functional>

#include "camerapipelineframe.h"
#include "camerasettings.h"

class QNetworkReply;
class QNetworkAccessManager;

class CameraAlpacaController
{
public:
    struct ImageFetchResult
    {
        QImage m_image;
        CameraPipelineFrame::BayerPattern m_bayerPattern = CameraPipelineFrame::BayerNone;
        QString m_receiveImageFormat;
    };

    struct CapabilitiesReport
    {
        QString m_name;
        QString m_description;
        int m_maxBinX = 1;
        int m_maxBinY = 1;
        QStringList m_gains;
        int m_gainMin = 0;
        int m_gainMax = 0;
        QStringList m_offsets;
        int m_offsetMin = 0;
        int m_offsetMax = 0;
        QStringList m_readoutModes;
        QString m_sensorName;
        int m_sensorType = 0;
        int m_bayerOffsetX = 0;
        int m_bayerOffsetY = 0;
        double m_pixelSizeX = 0.0;
        double m_pixelSizeY = 0.0;
        int m_cameraSizeX = 0;
        int m_cameraSizeY = 0;
        double m_ccdTemperature = 0.0;
        bool m_ccdTemperatureValid = false;
        double m_exposureMinMs = 1.0;
        double m_exposureMaxMs = 60000.0;
        double m_exposureResolutionMs = 1.0;
    };

    struct StatusReport
    {
        int m_cameraState = -1;
        double m_ccdTemperature = 0.0;
        bool m_ccdTemperatureValid = false;
    };

    struct FilterWheelInfo
    {
        QStringList m_names;
        int m_position = -1;
    };

    CameraAlpacaController();

    void resetConnectionState();
    void resetFocuserConnectionState();
    void resetFilterWheelConnectionState();
    void setLastError(int errorNumber, const QString& errorMessage);
    void resetCaptureState();
    void logRequest(const CameraSettings& settings, const QString& method, const QUrl& url, const QByteArray& payload = QByteArray()) const;
    void logResponse(const CameraSettings& settings, const QString& method, const QUrl& url, QNetworkReply *reply,
        const QByteArray& payload = QByteArray(), bool updateLastError = true);
    void disconnectCamera(QNetworkAccessManager *networkManager, const CameraSettings& settings);
    void setConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings, bool connected,
        std::function<void()> reportStatus, std::function<void()> continuation = {});
    void runWhenConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void()> reportStatus, std::function<void()> continuation);
    void setFocuserConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings, bool connected,
        std::function<void()> reportStatus, std::function<void()> continuation = {});
    void runFocuserWhenConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void()> reportStatus, std::function<void()> continuation);
    void moveFocuser(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void()> reportStatus);
    void setFilterWheelConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings, bool connected,
        std::function<void()> reportStatus, std::function<void()> continuation = {});
    void runFilterWheelWhenConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void()> reportStatus, std::function<void()> continuation);
    void queryFilterWheelInfo(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void(const FilterWheelInfo&)> continuation);
    void queryFilterWheelPosition(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void(int)> continuation);
    void setFilterWheelPosition(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void()> onSuccess, std::function<void()> reportStatus);
    void setCameraParams(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<bool()> isCapturing, std::function<void()> onStartExposure, std::function<void()> onFailure);
    void putIntProperty(
        QNetworkAccessManager *networkManager,
        const CameraSettings& settings,
        int cameraId,
        const QString& property,
        const QString& bodyKey,
        int value,
        std::function<void()> continuation,
        std::function<void()> onSuccess = {},
        std::function<void(int, const QString&)> onFailure = {});
    void startExposure(QNetworkAccessManager *networkManager, const CameraSettings& settings, double exposureTimeMs,
        std::function<void()> onSuccess, std::function<void()> onFailure);
    void abortExposure(QNetworkAccessManager *networkManager, const CameraSettings& settings);
    void checkImageReady(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void(bool)> onSuccess, std::function<void()> onFailure);
    void checkCameraStateForImageReady(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void(int)> onSuccess, std::function<void()> onFailure);
    void fetchImageArray(QNetworkAccessManager *networkManager, const CameraSettings& settings, const QImage& fallbackImage,
        std::function<void(const ImageFetchResult&)> onSuccess, std::function<void()> onFailure);
    void queryCameraCapabilities(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void(const CapabilitiesReport&)> continuation);
    void pollStatus(QNetworkAccessManager *networkManager, const CameraSettings& settings,
        std::function<void(const StatusReport&)> continuation);
    QImage parseImageArray(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat = nullptr,
        CameraPipelineFrame::BayerPattern *bayerPattern = nullptr) const;
    QImage parseImageBytes(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat = nullptr,
        CameraPipelineFrame::BayerPattern *bayerPattern = nullptr) const;

    static bool parseErrorPayload(const QByteArray& payload, int& errorNumber, QString& errorMessage);
    static bool parseImageBytesError(const QByteArray& payload, int& errorNumber);
    static bool isDriverError(int errorNumber);
    static bool isOptionalCapabilityPath(const QString& path);
    static QString baseUrl(const CameraSettings& settings);
    static QString focuserBaseUrl(const CameraSettings& settings);
    static QString filterWheelBaseUrl(const CameraSettings& settings);
    static QString transportError(QNetworkReply *reply);
    static QImage renderGrayscaleRaw(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit);

private:
    QImage renderRawPixelArray(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit,
        CameraPipelineFrame::BayerPattern *bayerPattern = nullptr) const;

public:
    bool m_frameRequestPending;
    quint32 m_clientId;
    quint32 m_clientTransactionId;
    int m_sensorType;          // 0=Mono, 1=Colour, 2=RGGB, 3=CMYG, 4=CMYG2, 5=LRGB
    int m_cameraSizeX;
    int m_cameraSizeY;
    int m_bayerOffsetX;
    int m_bayerOffsetY;
    bool m_imageBytesSupported; // true = try ImageBytes binary protocol; false = use JSON
    int m_lastErrorNumber;
    QString m_lastErrorMessage;
    QString m_lastReceiveImageFormat;
    bool m_connected;
    bool m_connectionPending;
    QVector<std::function<void()>> m_pendingConnectedContinuations;
    bool m_focuserConnected;
    bool m_focuserConnectionPending;
    QVector<std::function<void()>> m_pendingFocuserConnectedContinuations;
    bool m_filterWheelConnected;
    bool m_filterWheelConnectionPending;
    QVector<std::function<void()>> m_pendingFilterWheelConnectedContinuations;
    bool m_bootstrapPending;
    QVector<std::function<void()>> m_pendingBootstrapContinuations;
    bool m_paramsInitialized;
    bool m_exposureSeenActive;
    int m_lastBinX;
    int m_lastBinY;
    int m_lastNumX;
    int m_lastNumY;
    int m_lastEffectiveNumX;
    int m_lastEffectiveNumY;
    int m_lastStartX;
    int m_lastStartY;
    int m_lastGain;
    int m_lastOffset;
    int m_lastReadoutMode;
    double m_exposureMinMs;
    double m_exposureMaxMs;
    QElapsedTimer m_captureTimer;
    qint64 m_lastCaptureTimeMs;
};

#endif // INCLUDE_FEATURE_CAMERAALPACACONTROLLER_H_
