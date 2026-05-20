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

#include "cameraalpacacontroller.h"

#include <limits>

#include <QJsonDocument>
#include <QJsonObject>
#include <QRandomGenerator>
#include <QtEndian>

CameraAlpacaController::CameraAlpacaController() :
    m_frameRequestPending(false),
    m_clientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_clientTransactionId(1),
    m_sensorType(0),
    m_cameraSizeX(0),
    m_cameraSizeY(0),
    m_bayerOffsetX(0),
    m_bayerOffsetY(0),
    m_imageBytesSupported(true),
    m_lastErrorNumber(0),
    m_lastErrorMessage(),
    m_lastReceiveImageFormat(),
    m_connected(false),
    m_connectionPending(false),
    m_focuserConnected(false),
    m_focuserConnectionPending(false),
    m_filterWheelConnected(false),
    m_filterWheelConnectionPending(false),
    m_bootstrapPending(false),
    m_paramsInitialized(false),
    m_exposureSeenActive(false),
    m_lastBinX(0),
    m_lastBinY(0),
    m_lastNumX(0),
    m_lastNumY(0),
    m_lastEffectiveNumX(-1),
    m_lastEffectiveNumY(-1),
    m_lastStartX(0),
    m_lastStartY(0),
    m_lastGain(-1),
    m_lastOffset(-1),
    m_lastReadoutMode(0),
    m_exposureMinMs(0.001),
    m_exposureMaxMs(60000.0),
    m_lastCaptureTimeMs(-1)
{
}

void CameraAlpacaController::resetConnectionState()
{
    m_connected = false;
    m_connectionPending = false;
    m_pendingConnectedContinuations.clear();
    m_bootstrapPending = false;
    m_pendingBootstrapContinuations.clear();
    setLastError(0, QString());
}

void CameraAlpacaController::resetFocuserConnectionState()
{
    m_focuserConnected = false;
    m_focuserConnectionPending = false;
    m_pendingFocuserConnectedContinuations.clear();
}

void CameraAlpacaController::resetFilterWheelConnectionState()
{
    m_filterWheelConnected = false;
    m_filterWheelConnectionPending = false;
    m_pendingFilterWheelConnectedContinuations.clear();
}

void CameraAlpacaController::setLastError(int errorNumber, const QString& errorMessage)
{
    m_lastErrorNumber = errorNumber;
    m_lastErrorMessage = errorMessage;
}

void CameraAlpacaController::resetCaptureState()
{
    m_lastCaptureTimeMs = -1;
    m_captureTimer.invalidate();
    m_paramsInitialized = false;
}

bool CameraAlpacaController::parseErrorPayload(const QByteArray& payload, int& errorNumber, QString& errorMessage)
{
    errorNumber = 0;
    errorMessage.clear();

    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (!doc.isObject()) {
        return false;
    }

    const QJsonObject root = doc.object();

    if (!root.contains(QStringLiteral("ErrorNumber")) && !root.contains(QStringLiteral("ErrorMessage"))) {
        return false;
    }

    errorNumber = root.value(QStringLiteral("ErrorNumber")).toInt(0);
    errorMessage = root.value(QStringLiteral("ErrorMessage")).toString();
    return true;
}

bool CameraAlpacaController::parseImageBytesError(const QByteArray& payload, int& errorNumber)
{
    errorNumber = 0;

    if (payload.size() < 8) {
        return false;
    }

    const char *data = payload.constData();
    const qint32 metadataVersion = qFromLittleEndian<qint32>(data);

    if (metadataVersion != 1) {
        return false;
    }

    errorNumber = qFromLittleEndian<qint32>(data + 4);
    return true;
}

bool CameraAlpacaController::isOptionalCapabilityPath(const QString& path)
{
    const QString prop = path.section('/', -1).toLower();
    return (prop == QStringLiteral("gains"))
        || (prop == QStringLiteral("gainmin"))
        || (prop == QStringLiteral("gainmax"))
        || (prop == QStringLiteral("offsets"))
        || (prop == QStringLiteral("offsetmin"))
        || (prop == QStringLiteral("offsetmax"))
        || (prop == QStringLiteral("readoutmodes"))
        || (prop == QStringLiteral("ccdtemperature"));
}
