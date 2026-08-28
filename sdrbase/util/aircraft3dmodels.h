///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                      //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, GNU General Public       //
// License for more details.                                                     //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_AIRCRAFT3DMODELS_H
#define INCLUDE_AIRCRAFT3DMODELS_H

#include <QString>
#include <QStringList>
#include <QHash>
#include <QList>
#include <QRegularExpression>

#include "export.h"

struct AircraftInformation;

// Matches an aircraft's database model name to an ICAO type designator
class SDRBASE_API ModelMatch {
public:
    ModelMatch(const QString &aircraftRegExp, const QString &model) :
        m_aircraftRegExp(aircraftRegExp),
        m_model(model)
    {
        m_aircraftRegExp.optimize();
    }

    virtual ~ModelMatch() = default;

    virtual bool match(const QString &aircraft, const QString &manufacturer, QString &model)
    {
        (void) manufacturer;

        QRegularExpressionMatch match = m_aircraftRegExp.match(aircraft);
        if (match.hasMatch())
        {
            model = m_model;
            return true;
        }
        else
        {
            return false;
        }
    }

protected:
    QRegularExpression m_aircraftRegExp;
    QString m_model;
};

// For very generic aircraft names, also match against manufacturer name
class SDRBASE_API ManufacturerModelMatch : public ModelMatch {
public:
    ManufacturerModelMatch(const QString &modelRegExp, const QString &manufacturerRegExp, const QString &model) :
        ModelMatch(modelRegExp, model),
        m_manufacturerRegExp(manufacturerRegExp)
    {
        m_manufacturerRegExp.optimize();
    }

    virtual bool match(const QString &aircraft, const QString &manufacturer, QString &model) override
    {
        QRegularExpressionMatch matchManufacturer = m_manufacturerRegExp.match(manufacturer);
        if (matchManufacturer.hasMatch())
        {
            QRegularExpressionMatch matchAircraft = m_aircraftRegExp.match(aircraft);
            if (matchAircraft.hasMatch())
            {
                model = m_model;
                return true;
            }
        }
        return false;
    }

protected:
    QRegularExpression m_manufacturerRegExp;
};

// Picks a 3D model for an aircraft, for the 3D map.
//
// The tables are built once, on first use, from the .gltf files in the application data
// directory - so a user without the optional 3D model pack simply gets no models, and
// everything falls back to the flat billboard icons.
class SDRBASE_API Aircraft3DModels
{
public:
    // Built on first use. Callers may be on different worker threads - the ADS-B tracker
    // and the Aircraft tracker are - so construction is guarded and everything is read
    // only afterwards.
    static const Aircraft3DModels& instance();

    // A model for an aircraft in the OpenSky database, preferring the operator's livery.
    // False when nothing matches: no model pack, or a type with no model.
    bool modelFor(const AircraftInformation *info, bool favourLivery, bool verbose,
                  QString &model, float &altitudeOffset, float &labelOffset) const;

    // A model for a bare ICAO type designator, with a livery chosen at random
    QString modelForType(const QString &type) const;

    // A model for a type in a particular operator's livery, empty if there is not one
    QString modelForTypeAndOperator(const QString &type, const QString &operatorICAO,
                                    bool verbose = false) const;

    // Where a type's model has to sit so its undercarriage is not underground
    void offsetsForType(const QString &type, float &altitudeOffset, float &labelOffset) const;

    bool isEmpty() const { return m_3DModelsByType.isEmpty(); }

    // A stand-in for an aircraft the matcher has no model for. Only a fraction of the
    // types that fly have a model, so without this a Falcon 900, a King Air or anything
    // else outside the set shows as a flat billboard next to fully modelled airliners.
    bool defaultModel(const QString &icaoType, QString &model,
                      float &altitudeOffset, float &labelOffset) const;

    static QString dataDir();

private:
    Aircraft3DModels(bool verbose);
    ~Aircraft3DModels();

    QHash<QString, QString> m_3DModels;             // "TYPE_LIVERY" to model path
    QHash<QString, QStringList> m_3DModelsByType;   // Type to every livery of it
    QList<ModelMatch *> m_3DModelMatch;             // Database names to type designators
    QHash<QString, float> m_modelAltitudeOffset;
    QHash<QString, float> m_labelAltitudeOffset;
};

#endif // INCLUDE_AIRCRAFT3DMODELS_H
