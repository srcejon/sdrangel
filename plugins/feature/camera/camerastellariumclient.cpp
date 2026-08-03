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

#include "camerastellariumclient.h"

#include <cmath>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>
#include <QtMath>

CameraStellariumClient::CameraStellariumClient(QObject *parent) :
    QObject(parent),
    m_networkManager(new QNetworkAccessManager(this))
{
}

void CameraStellariumClient::focusJ2000(
    const QString& remoteControlUrl,
    double rightAscensionDegrees,
    double declinationDegrees)
{
    if (!std::isfinite(rightAscensionDegrees) || !std::isfinite(declinationDegrees))
    {
        emit focusFailed(tr("The selected star has no valid catalog coordinates."));
        return;
    }

    QUrl endpoint = QUrl::fromUserInput(remoteControlUrl.trimmed());
    if (!endpoint.isValid() || endpoint.host().isEmpty()
        || ((endpoint.scheme() != QStringLiteral("http")) && (endpoint.scheme() != QStringLiteral("https"))))
    {
        emit focusFailed(tr("The Stellarium Remote Control URL is invalid."));
        return;
    }

    endpoint.setPath(QStringLiteral("/api/main/focus"));
    endpoint.setQuery(QString());
    endpoint.setFragment(QString());

    const double rightAscensionRadians = qDegreesToRadians(rightAscensionDegrees);
    const double declinationRadians = qDegreesToRadians(declinationDegrees);
    const double cosDeclination = std::cos(declinationRadians);
    const QString position = QStringLiteral("[%1,%2,%3]")
        .arg(cosDeclination * std::cos(rightAscensionRadians), 0, 'g', 17)
        .arg(cosDeclination * std::sin(rightAscensionRadians), 0, 'g', 17)
        .arg(std::sin(declinationRadians), 0, 'g', 17);

    QUrlQuery form;
    form.addQueryItem(QStringLiteral("position"), position);
    QNetworkRequest request(endpoint);
    request.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/x-www-form-urlencoded"));

    if (m_pendingReply) {
        m_pendingReply->abort();
    }

    QNetworkReply *reply = m_networkManager->post(request, form.toString(QUrl::FullyEncoded).toUtf8());
    m_pendingReply = reply;

    connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        const bool timedOut = reply->property("stellariumTimedOut").toBool();
        const int statusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        const QByteArray responseBody = reply->readAll().trimmed();

        if (m_pendingReply == reply) {
            m_pendingReply.clear();
        }

        if ((reply->error() == QNetworkReply::NoError) && (statusCode >= 200) && (statusCode < 300))
        {
            emit focusSucceeded();
        }
        else if (timedOut)
        {
            emit focusFailed(tr("Stellarium did not respond. Ensure it is running and its Remote Control plugin is enabled."));
        }
        else if (reply->error() != QNetworkReply::OperationCanceledError)
        {
            QString detail = QString::fromUtf8(responseBody);
            if (detail.isEmpty()) {
                detail = reply->errorString();
            }
            emit focusFailed(tr("Cannot focus the selected star in Stellarium: %1").arg(detail));
        }

        reply->deleteLater();
    });

    QTimer::singleShot(5000, reply, [reply]() {
        if (reply->isRunning())
        {
            reply->setProperty("stellariumTimedOut", true);
            reply->abort();
        }
    });
}
