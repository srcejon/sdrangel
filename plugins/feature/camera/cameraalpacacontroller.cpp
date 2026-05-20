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

#include <QJsonDocument>
#include <QJsonObject>
#include <QtEndian>

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
