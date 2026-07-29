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

#ifndef INCLUDE_SIMBAD_H
#define INCLUDE_SIMBAD_H

#include <QByteArray>
#include <QObject>
#include <QString>

#include "export.h"

class QNetworkAccessManager;
class QNetworkReply;

// SIMBAD astronomical object identifier resolver.
class SDRBASE_API Simbad : public QObject
{
    Q_OBJECT

protected:
    Simbad();

public:
    struct Object
    {
        QString m_identifier; // Identifier supplied to lookup()
        QString m_name;       // Main identifier returned by SIMBAD
        double m_ra;          // J2000/ICRS right ascension in decimal hours
        double m_dec;         // J2000/ICRS declination in decimal degrees
    };

    static Simbad* create();
    ~Simbad();

public slots:
    void lookup(const QString& identifier);

signals:
    void objectResolved(const Simbad::Object& object);
    void lookupFailed(const QString& identifier, const QString& error);

private slots:
    void handleReply(QNetworkReply* reply);

private:
    static bool parseResponse(
        const QString& identifier,
        const QByteArray& bytes,
        Object& object,
        QString& error);

    QNetworkAccessManager *m_networkManager;
};

#endif /* INCLUDE_SIMBAD_H */
