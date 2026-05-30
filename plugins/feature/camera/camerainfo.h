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

#ifndef INCLUDE_FEATURE_CAMERAINFO_H_
#define INCLUDE_FEATURE_CAMERAINFO_H_

#include <QList>
#include <QString>

namespace CameraProtocol
{
    inline QString qt() { return QStringLiteral("qt"); }
    inline QString asi() { return QStringLiteral("asi"); }
    inline QString alpaca() { return QStringLiteral("alpaca"); }
    inline QString video() { return QStringLiteral("video"); }
    inline QString images() { return QStringLiteral("images"); }

    inline bool isPlaybackSource(const QString& protocol)
    {
        return (protocol == video()) || (protocol == images());
    }

    inline QString playbackDisplayText(const QString& protocol, const QString& description)
    {
        if (protocol == video()) {
            return description.isEmpty() ? QStringLiteral("video:") : QStringLiteral("video:%1").arg(description);
        }
        if (protocol == images()) {
            return description.isEmpty() ? QStringLiteral("images:") : QStringLiteral("images:%1").arg(description);
        }
        return QString();
    }
}

struct CameraInfo
{
    QString m_protocol;     // "qt", "asi", "alpaca", "video" or "images"
    QString m_id;
    QString m_description;
    QString m_host;         // alpaca only
    quint16 m_port = 0;     // alpaca only
};

struct AlpacaDeviceInfo
{
    QString m_type;         // e.g. "camera", "focuser", "filterwheel"
    QString m_id;           // A number, typically 0
    QString m_description;
    QString m_host;
    quint16 m_port = 0;
};

#endif // INCLUDE_FEATURE_CAMERAINFO_H_
