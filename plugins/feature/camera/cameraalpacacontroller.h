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
#include <QString>
#include <QVector>

#include <functional>

#include "camerapipelineframe.h"
#include "camerasettings.h"

class QNetworkReply;

class CameraAlpacaController
{
public:
    CameraAlpacaController();

    void resetConnectionState();
    void resetFocuserConnectionState();
    void resetFilterWheelConnectionState();
    void setLastError(int errorNumber, const QString& errorMessage);
    void resetCaptureState();
    QImage parseImageArray(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat = nullptr,
        CameraPipelineFrame::BayerPattern *bayerPattern = nullptr) const;
    QImage parseImageBytes(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat = nullptr,
        CameraPipelineFrame::BayerPattern *bayerPattern = nullptr) const;

    static bool parseErrorPayload(const QByteArray& payload, int& errorNumber, QString& errorMessage);
    static bool parseImageBytesError(const QByteArray& payload, int& errorNumber);
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
