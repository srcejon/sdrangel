///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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

#include "simbad.h"

#include <cmath>

#include <QDebug>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRegularExpression>
#include <QUrl>

namespace {

const QString SIMBAD_URL = QStringLiteral("https://simbad.cds.unistra.fr/simbad/sim-script");
const QString RESPONSE_PREFIX = QStringLiteral("SDRANGEL_SIMBAD|");
const char IDENTIFIER_PROPERTY[] = "simbadIdentifier";

}

Simbad::Simbad() :
    m_networkManager(new QNetworkAccessManager(this))
{
    connect(m_networkManager, &QNetworkAccessManager::finished, this, &Simbad::handleReply);
}

Simbad::~Simbad()
{
    disconnect(m_networkManager, &QNetworkAccessManager::finished, this, &Simbad::handleReply);
}

Simbad* Simbad::create()
{
    return new Simbad();
}

void Simbad::lookup(const QString& identifier)
{
    const QString trimmedIdentifier = identifier.trimmed();
    static const QRegularExpression controlCharacters(QStringLiteral("[\\x00-\\x1f\\x7f]"));

    if (trimmedIdentifier.isEmpty())
    {
        emit lookupFailed(trimmedIdentifier, QStringLiteral("The identifier is empty."));
        return;
    }

    if (trimmedIdentifier.size() > 256)
    {
        emit lookupFailed(trimmedIdentifier, QStringLiteral("The identifier is too long."));
        return;
    }

    if (controlCharacters.match(trimmedIdentifier).hasMatch())
    {
        emit lookupFailed(trimmedIdentifier, QStringLiteral("The identifier contains invalid control characters."));
        return;
    }

    const QString script = QStringLiteral(
        "output console=off script=off\n"
        "format object \"SDRANGEL_SIMBAD|%IDLIST(1)|%12.8COO(d;A;ICRS;J2000;2000)|%+12.8COO(d;D;ICRS;J2000;2000)\"\n"
        "query id %1").arg(trimmedIdentifier);

    QNetworkRequest request{QUrl(SIMBAD_URL)};
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    request.setRawHeader("Accept", "text/plain");
    request.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/x-www-form-urlencoded"));
    request.setRawHeader("User-Agent", "SDRangel SIMBAD resolver");

    const QByteArray formData = QByteArrayLiteral("script=") + QUrl::toPercentEncoding(script);
    QNetworkReply *reply = m_networkManager->post(request, formData);
    reply->setProperty(IDENTIFIER_PROPERTY, trimmedIdentifier);
}

void Simbad::handleReply(QNetworkReply* reply)
{
    if (!reply)
    {
        qWarning() << "Simbad::handleReply: reply is null";
        return;
    }

    const QString identifier = reply->property(IDENTIFIER_PROPERTY).toString();

    if (reply->error() != QNetworkReply::NoError)
    {
        const QString error = reply->errorString();
        qWarning() << "Simbad::handleReply:" << identifier << error;
        emit lookupFailed(identifier, error);
        reply->deleteLater();
        return;
    }

    Object object;
    QString error;

    if (parseResponse(identifier, reply->readAll(), object, error)) {
        emit objectResolved(object);
    } else {
        qWarning() << "Simbad::handleReply:" << identifier << error;
        emit lookupFailed(identifier, error);
    }

    reply->deleteLater();
}

bool Simbad::parseResponse(
    const QString& identifier,
    const QByteArray& bytes,
    Object& object,
    QString& error)
{
    const QString response = QString::fromUtf8(bytes);
    const QStringList lines = response.split(QLatin1Char('\n'));

    for (const QString& untrimmedLine : lines)
    {
        const QString line = untrimmedLine.trimmed();

        if (!line.startsWith(RESPONSE_PREFIX)) {
            continue;
        }

        const QStringList fields = line.split(QLatin1Char('|'));

        if (fields.size() != 4)
        {
            error = QStringLiteral("SIMBAD returned an unexpected response.");
            return false;
        }

        bool raOk;
        bool decOk;
        const double raDegrees = fields[2].trimmed().toDouble(&raOk);
        const double decDegrees = fields[3].trimmed().toDouble(&decOk);

        if (!raOk
            || !decOk
            || !std::isfinite(raDegrees)
            || !std::isfinite(decDegrees)
            || (raDegrees < 0.0)
            || (raDegrees >= 360.0)
            || (decDegrees < -90.0)
            || (decDegrees > 90.0))
        {
            error = QStringLiteral("SIMBAD returned invalid coordinates.");
            return false;
        }

        object.m_identifier = identifier;
        object.m_name = fields[1].trimmed();
        object.m_ra = raDegrees / 15.0;
        object.m_dec = decDegrees;
        return true;
    }

    const int errorSection = response.indexOf(QStringLiteral("::error::"));

    if (errorSection >= 0)
    {
        const QStringList errorLines = response.mid(errorSection).split(QLatin1Char('\n'));

        for (const QString& errorLine : errorLines)
        {
            const QString trimmedLine = errorLine.trimmed();

            if (!trimmedLine.isEmpty() && !trimmedLine.startsWith(QStringLiteral("::")))
            {
                error = trimmedLine;
                return false;
            }
        }
    }

    error = QStringLiteral("SIMBAD did not recognize \"%1\".").arg(identifier);
    return false;
}
