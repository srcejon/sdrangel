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

#include <QDir>
#include <QElapsedTimer>
#include <QDebug>
#include <QRandomGenerator>
#include <QStandardPaths>

#include "util/aircraft3dmodels.h"
#include "util/osndb.h"

// Whether the one-off table build logs what it matched. Read from the environment
// rather than from settings because the tables are shared by every plugin and built
// once, before any of their settings exist.
static bool verboseFromEnvironment()
{
    return qEnvironmentVariableIsSet("SDRANGEL_VERBOSE_MODEL_MATCHING");
}

const Aircraft3DModels& Aircraft3DModels::instance()
{
    // Function local static: the C++11 guarantee that this is constructed exactly once,
    // even when two worker threads arrive together. That matters here - the ADS-B tracker
    // and the Aircraft tracker each run on their own thread and both want these tables.
    static Aircraft3DModels models(verboseFromEnvironment());
    return models;
}

QString Aircraft3DModels::dataDir()
{
    // Where the aircraft and airport databases and the 3D models live
    QStringList locations = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation);
    return locations[0];
}

Aircraft3DModels::~Aircraft3DModels()
{
    qDeleteAll(m_3DModelMatch);
}

Aircraft3DModels::Aircraft3DModels(bool verbose)
{
    // Timed and logged unconditionally: it runs once per session, reads a directory tree
    // that can be well over a gigabyte, and is the first thing to suspect when start up
    // is slow. A silent multi-second pause is precisely what makes that hard to find.
    QElapsedTimer timer;
    timer.start();

    // Look for all aircraft gltfs in 3d directory
    QString modelDir = dataDir() + "/3d";
    static const QStringList subDirs = {"BB_Airbus_png", "BB_Boeing_png", "BB_Jets_png", "BB_Props_png", "BB_GA_png", "BB_Mil_png", "BB_Heli_png"};

    for (auto subDir : subDirs)
    {
        QString dirName = modelDir + "/" + subDir;
        QDir dir(dirName);
        QStringList aircrafts = dir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
        for (auto aircraft : aircrafts)
        {
            if (verbose) {
                qDebug() << "Aircraft3DModels:" << aircraft;
            }
            QDir aircraftDir(dir.filePath(aircraft));
            QStringList gltfs = aircraftDir.entryList({"*.gltf"});
            QStringList allAircraft;
            for (auto gltf : gltfs)
            {
                QStringList filenameParts = gltf.split(".")[0].split("_");
                if (filenameParts.size() == 2)
                {
                    QString livery = filenameParts[1];
                    if (verbose) {
                        qDebug() << "Aircraft3DModels:" << aircraft << "Livery " << livery;
                    }
                    // Only use relative path, as Map feature will add the prefix
                    QString filename = subDir + "/" + aircraft + "/" + gltf;
                    m_3DModels.insert(aircraft + "_" + livery, filename);
                    allAircraft.append(filename);
                }
            }
            if (gltfs.size() > 0) {
                m_3DModelsByType.insert(aircraft, allAircraft);
            }
        }
    }

    // Vertical offset so undercarriage isn't underground, because 0,0,0 is in the middle of the model
    // rather than at the bottom
    m_modelAltitudeOffset.insert("A306", 4.6f);
    m_modelAltitudeOffset.insert("A310", 4.6f);
    m_modelAltitudeOffset.insert("A318", 3.7f);
    m_modelAltitudeOffset.insert("A319", 3.5f);
    m_modelAltitudeOffset.insert("A320", 3.5f);
    m_modelAltitudeOffset.insert("A321", 3.5f);
    m_modelAltitudeOffset.insert("A332", 5.52f);
    m_modelAltitudeOffset.insert("A333", 5.52f);
    m_modelAltitudeOffset.insert("A334", 5.52f);
    m_modelAltitudeOffset.insert("A343", 4.65f);
    m_modelAltitudeOffset.insert("A345", 4.65f);
    m_modelAltitudeOffset.insert("A346", 4.65f);
    m_modelAltitudeOffset.insert("A388", 5.75f);
    m_modelAltitudeOffset.insert("B717", 0.0f);
    m_modelAltitudeOffset.insert("B733", 3.1f);
    m_modelAltitudeOffset.insert("B734", 3.27f);
    m_modelAltitudeOffset.insert("B737", 3.0f);
    m_modelAltitudeOffset.insert("B738", 3.31f);
    m_modelAltitudeOffset.insert("B739", 3.32f);
    m_modelAltitudeOffset.insert("B74F", 5.3f);
    m_modelAltitudeOffset.insert("B744", 5.25f);
    m_modelAltitudeOffset.insert("B752", 3.6f);
    m_modelAltitudeOffset.insert("B763", 4.44f);
    m_modelAltitudeOffset.insert("B772", 5.57f);
    m_modelAltitudeOffset.insert("B773", 5.6f);
    m_modelAltitudeOffset.insert("B77L", 5.57f);
    m_modelAltitudeOffset.insert("B77W", 5.57f);
    m_modelAltitudeOffset.insert("B788", 4.1f);
    m_modelAltitudeOffset.insert("BE20", 1.48f);
    m_modelAltitudeOffset.insert("C150", 1.05f);
    m_modelAltitudeOffset.insert("C172", 1.16f);
    m_modelAltitudeOffset.insert("C421", 1.16f);
    m_modelAltitudeOffset.insert("H25B", 1.45f);
    m_modelAltitudeOffset.insert("LJ45", 1.27f);
    m_modelAltitudeOffset.insert("B462", 1.8f);
    m_modelAltitudeOffset.insert("B463", 1.9f);
    m_modelAltitudeOffset.insert("CRJ2", 1.3f);
    m_modelAltitudeOffset.insert("CRJ7", 1.66f);
    m_modelAltitudeOffset.insert("CRJ9", 2.27f);
    m_modelAltitudeOffset.insert("CRJX", 2.49f);
    m_modelAltitudeOffset.insert("DC10", 5.2f);
    m_modelAltitudeOffset.insert("E135", 1.88f);
    m_modelAltitudeOffset.insert("E145", 1.86f);
    m_modelAltitudeOffset.insert("E170", 2.3f);
    m_modelAltitudeOffset.insert("E190", 3.05f);
    m_modelAltitudeOffset.insert("E195", 2.97f);
    m_modelAltitudeOffset.insert("F28", 2.34f);
    m_modelAltitudeOffset.insert("F70", 2.43f);
    m_modelAltitudeOffset.insert("F100", 2.23f);
    m_modelAltitudeOffset.insert("J328", 1.01f);
    m_modelAltitudeOffset.insert("MD11", 5.22f);
    m_modelAltitudeOffset.insert("MD83", 2.71f);
    m_modelAltitudeOffset.insert("MD90", 2.62f);
    m_modelAltitudeOffset.insert("AT42", 1.75f);
    m_modelAltitudeOffset.insert("AT72", 1.83f);
    m_modelAltitudeOffset.insert("D328", 0.99f);
    m_modelAltitudeOffset.insert("DH8D", 1.65f);
    m_modelAltitudeOffset.insert("F50", 2.16f);
    m_modelAltitudeOffset.insert("JS41", 1.9f);
    m_modelAltitudeOffset.insert("L410", 1.1f);
    m_modelAltitudeOffset.insert("SB20", 2.0f);
    m_modelAltitudeOffset.insert("SF34", 1.89f);

    // Label offsets (from bottom of aircraft)
    m_labelAltitudeOffset.insert("A306", 10.0f);
    m_labelAltitudeOffset.insert("A310", 15.0f);
    m_labelAltitudeOffset.insert("A318", 10.0f);
    m_labelAltitudeOffset.insert("A319", 10.0f);
    m_labelAltitudeOffset.insert("A320", 10.0f);
    m_labelAltitudeOffset.insert("A321", 10.0f);
    m_labelAltitudeOffset.insert("A332", 14.0f);
    m_labelAltitudeOffset.insert("A333", 14.0f);
    m_labelAltitudeOffset.insert("A334", 14.0f);
    m_labelAltitudeOffset.insert("A343", 14.0f);
    m_labelAltitudeOffset.insert("A345", 14.0f);
    m_labelAltitudeOffset.insert("A346", 14.0f);
    m_labelAltitudeOffset.insert("A388", 20.0f);
    m_labelAltitudeOffset.insert("B717", 7.5f);
    m_labelAltitudeOffset.insert("B733", 10.0f);
    m_labelAltitudeOffset.insert("B734", 10.0f);
    m_labelAltitudeOffset.insert("B737", 10.0f);
    m_labelAltitudeOffset.insert("B738", 10.0f);
    m_labelAltitudeOffset.insert("B739", 10.0f);
    m_labelAltitudeOffset.insert("B74F", 15.0f);
    m_labelAltitudeOffset.insert("B744", 15.0f);
    m_labelAltitudeOffset.insert("B752", 12.0f);
    m_labelAltitudeOffset.insert("B763", 14.0f);
    m_labelAltitudeOffset.insert("B772", 14.0f);
    m_labelAltitudeOffset.insert("B773", 14.0f);
    m_labelAltitudeOffset.insert("B77L", 14.0f);
    m_labelAltitudeOffset.insert("B77W", 14.0f);
    m_labelAltitudeOffset.insert("B788", 14.0f);
    m_labelAltitudeOffset.insert("BE20", 4.0f);
    m_labelAltitudeOffset.insert("C150", 3.0f);
    m_labelAltitudeOffset.insert("C172", 3.0f);
    m_labelAltitudeOffset.insert("C421", 4.0f);
    m_labelAltitudeOffset.insert("H25B", 5.0f);
    m_labelAltitudeOffset.insert("LJ45", 5.0f);
    m_labelAltitudeOffset.insert("B462", 7.0f);
    m_labelAltitudeOffset.insert("B463", 7.0f);
    m_labelAltitudeOffset.insert("CRJ2", 5.5f);
    m_labelAltitudeOffset.insert("CRJ7", 6.0f);
    m_labelAltitudeOffset.insert("CRJ9", 6.0f);
    m_labelAltitudeOffset.insert("CRJX", 6.0f);
    m_labelAltitudeOffset.insert("DC10", 15.0f);
    m_labelAltitudeOffset.insert("E135", 5.0f);
    m_labelAltitudeOffset.insert("E145", 5.0f);
    m_labelAltitudeOffset.insert("E170", 8.0f);
    m_labelAltitudeOffset.insert("E190", 8.5f);
    m_labelAltitudeOffset.insert("E195", 8.5f);
    m_labelAltitudeOffset.insert("F28", 7.0f);
    m_labelAltitudeOffset.insert("F70", 6.5f);
    m_labelAltitudeOffset.insert("F100", 6.5f);
    m_labelAltitudeOffset.insert("J328", 5.0f);  // Check
    m_labelAltitudeOffset.insert("MD11", 15.0f);
    m_labelAltitudeOffset.insert("MD83", 7.5f);
    m_labelAltitudeOffset.insert("MD90", 7.5f);
    m_labelAltitudeOffset.insert("AT42", 7.0f);
    m_labelAltitudeOffset.insert("AT72", 7.0f);
    m_labelAltitudeOffset.insert("D328", 6.0f);
    m_labelAltitudeOffset.insert("DH8D", 6.5f);
    m_labelAltitudeOffset.insert("F50",  7.0f);
    m_labelAltitudeOffset.insert("JS41", 5.0f);
    m_labelAltitudeOffset.insert("L410", 5.0f);
    m_labelAltitudeOffset.insert("SB20", 6.5f);
    m_labelAltitudeOffset.insert("SF34", 6.0f);

    // Map from OpenSky database names to 3D model names
    m_3DModelMatch.append(new ModelMatch("A300.*", "A306")); // A300 B4 is A300-600, but use for others as closest match
    m_3DModelMatch.append(new ModelMatch("A310.*", "A310"));
    m_3DModelMatch.append(new ModelMatch("A318.*", "A318"));
    m_3DModelMatch.append(new ModelMatch("A.?319.*", "A319"));
    m_3DModelMatch.append(new ModelMatch("A.?320.*", "A320"));
    m_3DModelMatch.append(new ModelMatch("A.?321.*", "A321"));
    m_3DModelMatch.append(new ModelMatch("A330.2.*", "A332"));
    m_3DModelMatch.append(new ModelMatch("A330.3.*", "A333"));
    m_3DModelMatch.append(new ModelMatch("A330.7.*", "A333")); // BelugaXL
    m_3DModelMatch.append(new ModelMatch("A330.8.*", "A332")); // 200 Neo
    m_3DModelMatch.append(new ModelMatch("A330.9.*", "A333")); // 300 Neo
    m_3DModelMatch.append(new ModelMatch("A340.3.*", "A343"));
    m_3DModelMatch.append(new ModelMatch("A340.5.*", "A345"));
    m_3DModelMatch.append(new ModelMatch("A340.6.*", "A346"));
    m_3DModelMatch.append(new ModelMatch("A350.*", "A333"));   // No A350 model - use 330 as twin engine
    m_3DModelMatch.append(new ModelMatch("A380.*", "A388"));

    m_3DModelMatch.append(new ModelMatch("737.2.*", "B733"));  // No 200 model
    m_3DModelMatch.append(new ModelMatch("737.3.*", "B733"));
    m_3DModelMatch.append(new ModelMatch("737.4.*", "B734"));
    m_3DModelMatch.append(new ModelMatch("737.5.*", "B734"));  // No 500 model
    m_3DModelMatch.append(new ModelMatch("737.6.*", "B737"));  // No 600 model
    m_3DModelMatch.append(new ModelMatch("737NG.6.*", "B737"));
    m_3DModelMatch.append(new ModelMatch("737.7.*", "B737"));
    m_3DModelMatch.append(new ModelMatch("737NG.7.*", "B737"));
    m_3DModelMatch.append(new ModelMatch("737.8.*", "B738"));
    m_3DModelMatch.append(new ModelMatch("737NG.8.*", "B738"));  // No Max model yet
    m_3DModelMatch.append(new ModelMatch("737MAX.8.*", "B738"));
    m_3DModelMatch.append(new ModelMatch("737.9", "B739"));
    m_3DModelMatch.append(new ModelMatch("737NG.9", "B739"));
    m_3DModelMatch.append(new ModelMatch("737MAX.9", "B739"));
    m_3DModelMatch.append(new ModelMatch("B747.*F", "B74F"));
    m_3DModelMatch.append(new ModelMatch("B747.*\\(F\\)", "B74F"));
    m_3DModelMatch.append(new ModelMatch("747.*", "B744"));
    m_3DModelMatch.append(new ModelMatch("757.*", "B752"));
    m_3DModelMatch.append(new ModelMatch("767.*", "B763"));
    m_3DModelMatch.append(new ModelMatch("777.2.*LR.*", "B77L"));
    m_3DModelMatch.append(new ModelMatch("777.2.*", "B772"));
    m_3DModelMatch.append(new ModelMatch("777.3.*ER.*", "B77W"));
    m_3DModelMatch.append(new ModelMatch("777.3.*", "B773"));
    m_3DModelMatch.append(new ModelMatch("777.*", "B772"));
    m_3DModelMatch.append(new ModelMatch("787.*", "B788"));
    m_3DModelMatch.append(new ModelMatch("717.*", "B717"));
    // No 727 model

    // Jets
    m_3DModelMatch.append(new ModelMatch(".*EMB.135.*", "E135"));
    m_3DModelMatch.append(new ModelMatch(".*ERJ.135.*", "E135"));
    m_3DModelMatch.append(new ModelMatch("Embraer 135.*", "E135"));
    m_3DModelMatch.append(new ModelMatch(".*EMB.145.*", "E145"));
    m_3DModelMatch.append(new ModelMatch(".*ERJ.145.*", "E145"));
    m_3DModelMatch.append(new ModelMatch("Embraer 145.*", "E145"));
    m_3DModelMatch.append(new ModelMatch(".*EMB.170.*", "E170"));
    m_3DModelMatch.append(new ModelMatch(".*ERJ.170.*", "E170"));
    m_3DModelMatch.append(new ModelMatch("Embraer 170.*", "E170"));
    m_3DModelMatch.append(new ModelMatch(".*EMB.190.*", "E190"));
    m_3DModelMatch.append(new ModelMatch(".*ERJ.190.*", "E190"));
    m_3DModelMatch.append(new ModelMatch("Embraer 190.*", "E190"));
    m_3DModelMatch.append(new ModelMatch(".*EMB.195.*", "E195"));
    m_3DModelMatch.append(new ModelMatch(".*ERJ.195.*", "E195"));
    m_3DModelMatch.append(new ModelMatch("Embraer 195.*", "E195"));

    m_3DModelMatch.append(new ModelMatch(".*CRJ.200.*", "CRJ2"));
    m_3DModelMatch.append(new ModelMatch(".*CRJ.700.*", "CRJ7"));
    m_3DModelMatch.append(new ModelMatch(".*CRJ.900.*", "CRJ9"));
    m_3DModelMatch.append(new ModelMatch(".*CRJ.1000.*", "CRJX"));

    // PNGs missing
    //m_3DModelMatch.append(new ModelMatch("(BAE )?146.2.*", "B462"));
    //m_3DModelMatch.append(new ModelMatch("(BAE )?146.3.*", "B463"));

    m_3DModelMatch.append(new ModelMatch("DC-10.*", "DC10"));

    m_3DModelMatch.append(new ModelMatch(".*MD.11.*", "MD11"));
    m_3DModelMatch.append(new ModelMatch(".*MD.83.*", "MD83"));
    m_3DModelMatch.append(new ModelMatch(".*MD.90.*", "MD90"));

    m_3DModelMatch.append(new ModelMatch(".*F28.*", "F28"));
    m_3DModelMatch.append(new ModelMatch(".*F70.*", "F70"));
    m_3DModelMatch.append(new ModelMatch(".*F100.*", "F100"));

    // GA
    m_3DModelMatch.append(new ModelMatch(".*B200.*", "BE20"));
    m_3DModelMatch.append(new ManufacturerModelMatch(".*200.*", ".*Beech.*", "BE20"));
    m_3DModelMatch.append(new ModelMatch(".*150.*", "C150"));
    m_3DModelMatch.append(new ModelMatch(".*172.*", "C172"));
    m_3DModelMatch.append(new ModelMatch(".*421.*", "C421"));
    m_3DModelMatch.append(new ModelMatch(".*125.*", "H25B"));
    m_3DModelMatch.append(new ManufacturerModelMatch(".*400.*", "Hawker.*", "H25B"));
    m_3DModelMatch.append(new ManufacturerModelMatch(".*400.*", "Raytheon.*", "H25B"));
    m_3DModelMatch.append(new ModelMatch(".*Learjet.*", "LJ45"));

    // Props
    m_3DModelMatch.append(new ModelMatch("ATR.*42.*", "AT42"));
    m_3DModelMatch.append(new ModelMatch("ATR.*72.*", "AT72"));
    m_3DModelMatch.append(new ModelMatch("Do 328.*", "D328"));
    m_3DModelMatch.append(new ModelMatch("DHC-8.*", "DH8D"));
    m_3DModelMatch.append(new ModelMatch(".*F50.*", "F50"));
    m_3DModelMatch.append(new ModelMatch("Jetstream 41.*", "JS41"));
    m_3DModelMatch.append(new ModelMatch(".*L.410.*", "L410"));
    m_3DModelMatch.append(new ModelMatch("SAAB.2000.*", "SB20"));
    m_3DModelMatch.append(new ManufacturerModelMatch(".*340.*", "Saab.*", "SF34"));

    qDebug() << "Aircraft3DModels: scanned" << dataDir() + "/3d" << "in" << timer.elapsed()
             << "ms -" << m_3DModelsByType.size() << "types," << m_3DModels.size()
             << "liveries," << m_3DModelMatch.size() << "name matches";
    (void) verbose;
}

QString Aircraft3DModels::modelForTypeAndOperator(const QString &type, const QString &operatorICAO, bool verbose) const
{
    QString typeOperator = type + "_" + operatorICAO;
    if (m_3DModels.contains(typeOperator)) {
        return m_3DModels.value(typeOperator);
    }
    if (verbose) {
        qDebug() << "Aircraft3DModels: no livery for" << typeOperator;
    }
    return "";
}

QString Aircraft3DModels::modelForType(const QString &type) const
{
    if (m_3DModelsByType.contains(type))
    {
        // Choose a random livery. QRandomGenerator::global() is thread safe, which the
        // ADS-B tracker's own generator was not once these tables became shared.
        const QStringList models = m_3DModelsByType.value(type);
        return models[QRandomGenerator::global()->bounded(models.size())];
    }
    return "";
}

void Aircraft3DModels::offsetsForType(const QString &type, float &altitudeOffset, float &labelOffset) const
{
    if (m_modelAltitudeOffset.contains(type))
    {
        altitudeOffset = m_modelAltitudeOffset.value(type);
        labelOffset = m_labelAltitudeOffset.value(type);
    }
}

// Types are grouped by what they LOOK like rather than by what they are, because the
// stand-in only has to be the right shape and size at map distances: a four engine
// heavy, a widebody twin, a narrowbody twin, or something small. Within a group the
// candidates are ordered by how commonly a model for them is installed, and the search
// widens rather than giving up - a wrong-but-plausible airliner reads far better than a
// billboard among modelled aircraft.
bool Aircraft3DModels::defaultModel(const QString &icaoType, QString &model,
                                    float &altitudeOffset, float &labelOffset) const
{
    if (m_3DModelsByType.isEmpty()) {
        return false;
    }

    static const QStringList fourEngine = {
        "A342", "A343", "A345", "A346", "A388", "B741", "B742", "B743", "B744",
        "B748", "B74F", "BLCF", "C130", "C17", "IL76", "AN12", "E3TF", "K35R"};
    static const QStringList heavyTwin = {
        "B762", "B763", "B764", "B772", "B773", "B77L", "B77W", "B788", "B789",
        "B78X", "A306", "A310", "A332", "A333", "A338", "A339", "A359", "A35K",
        "MD11", "DC10"};
    static const QStringList small = {
        "GLF4", "GLF5", "GLF6", "GLEX", "GL5T", "CL60", "C25A", "C25B", "C25C",
        "C525", "C550", "C560", "C56X", "C680", "C68A", "C750", "E50P", "E55P",
        "E545", "E550", "F2TH", "F900", "FA7X", "FA8X", "LJ35", "LJ45", "LJ60",
        "PC24", "PRM1", "HDJT", "BE20", "C172", "C182", "PA28", "SR22", "DA42",
        "P28A", "TBM9", "PC12", "SF50"};

    // Preference order per group. First one with a model installed wins
    static const QStringList fourEngineModels = {"B744", "A388", "A343", "B74F", "B742"};
    static const QStringList heavyTwinModels  = {"B77W", "A333", "B788", "A332", "B763", "B772"};
    static const QStringList smallModels      = {"GLF5", "GLEX", "C750", "CL60", "LJ45", "C680"};
    static const QStringList narrowBodyModels = {"A320", "B738", "A319", "A321", "B737", "B739", "B733"};

    QList<const QStringList *> order;
    if (fourEngine.contains(icaoType)) {
        order = {&fourEngineModels, &heavyTwinModels, &narrowBodyModels, &smallModels};
    } else if (heavyTwin.contains(icaoType)) {
        order = {&heavyTwinModels, &fourEngineModels, &narrowBodyModels, &smallModels};
    } else if (small.contains(icaoType)) {
        order = {&smallModels, &narrowBodyModels, &heavyTwinModels, &fourEngineModels};
    } else {
        // Unknown or blank type. Most of what flies with a transponder and no model of
        // its own is an airliner-shaped thing, so that is the safer guess
        order = {&narrowBodyModels, &heavyTwinModels, &smallModels, &fourEngineModels};
    }

    for (const QStringList *candidates : order)
    {
        for (const QString& candidate : *candidates)
        {
            const QString found = modelForType(candidate);
            if (!found.isEmpty())
            {
                model = found;
                offsetsForType(candidate, altitudeOffset, labelOffset);
                return true;
            }
        }
    }

    // Nothing from any preference list is installed, so take whatever is. Sorted so the
    // same aircraft does not change shape from one run to the next.
    QStringList types = m_3DModelsByType.keys();
    types.sort();
    const QString type = types.first();
    model = modelForType(type);
    offsetsForType(type, altitudeOffset, labelOffset);
    return !model.isEmpty();
}

bool Aircraft3DModels::modelFor(const AircraftInformation *info, bool favourLivery, bool verbose,
                                QString &model, float &altitudeOffset, float &labelOffset) const
{
    if (!info || info->m_model.isEmpty()) {
        return false;
    }

    model = "";
    QString type;

    for (auto mm : m_3DModelMatch)
    {
        if (!mm->match(info->m_model, info->m_manufacturerName, type)) {
            continue;
        }

        QString operatorICAO = info->m_operatorICAO;

        // Look for operator specific livery
        if (!operatorICAO.isEmpty()) {
            model = modelForTypeAndOperator(type, operatorICAO, verbose);
        }

        if (model.isEmpty())
        {
            // Try similar operator (E.g. EasyJet instead of EasyJet Europe)
            static const QHash<QString, QString> alternateOperator = {
                {"EJU", "EZY"},
                {"WUK", "WZZ"},
                {"TFL", "TOM"},
                {"NOZ", "NAX"},
                {"NSZ", "NAX"},
                {"BCS", "DHK"},
            };

            if (alternateOperator.contains(operatorICAO))
            {
                operatorICAO = alternateOperator.value(operatorICAO);
                model = modelForTypeAndOperator(type, operatorICAO, verbose);
            }

            if (model.isEmpty() && favourLivery && !operatorICAO.isEmpty())
            {
                // Try to find similar aircraft with matching livery
                static const QHash<QString, QStringList> alternateTypes = {
                    {"B788", {"B77W", "B77L", "B772", "B773", "B763", "A332", "A333"}},
                    {"B77W", {"B77L", "B772", "B773", "B788", "B763", "A332", "A333"}},
                    {"B77L", {"B77W", "B772", "B773", "B788", "B763", "A332", "A333"}},
                    {"B772", {"B77W", "B77L", "B773", "B788", "B763", "A332", "A333"}},
                    {"B773", {"B77W", "B77L", "B772", "B788", "B763", "A332", "A333"}},
                    {"A332", {"A333", "B77W", "B77L", "B773", "B772", "B788", "B763"}},
                    {"A333", {"A332", "B77W", "B77L", "B773", "B772", "B788", "B763"}},
                    {"A342", {"A343", "A345", "A346"}},
                    {"A343", {"A342", "A345", "A346"}},
                    {"A345", {"A343", "A342", "A346"}},
                    {"A346", {"A345", "A343", "A342"}},
                    {"B744", {"B74F"}},
                    {"B74F", {"B744"}},
                    {"B733", {"B734", "B737", "B738", "B739", "B752", "A320", "A319", "A321"}},
                    {"B734", {"B733", "B737", "B738", "B739", "B752", "A320", "A319", "A321"}},
                    {"B737", {"B733", "B734", "B738", "B739", "B752", "A320", "A319", "A321"}},
                    {"B738", {"B733", "B734", "B737", "B739", "B752", "A320", "A319", "A321"}},
                    {"B739", {"B733", "B734", "B737", "B738", "B752", "A320", "A319", "A321"}},
                    {"A319", {"A320", "A321", "B733", "B734", "B737", "B738", "B739"}},
                    {"A320", {"A319", "A321", "B733", "B734", "B737", "B738", "B739"}},
                    {"A321", {"A319", "A320", "B733", "B734", "B737", "B738", "B739"}},
                    {"A306", {"A332", "A333", "B763"}},
                };

                if (alternateTypes.contains(type))
                {
                    for (const auto& alternate : alternateTypes.value(type))
                    {
                        model = modelForTypeAndOperator(alternate, operatorICAO, verbose);
                        if (!model.isEmpty()) {
                            break;
                        }
                    }
                }
            }

            if (model.isEmpty())
            {
                // Try for aircraft without specific livery
                model = modelForType(type);
            }
        }
        break;
    }

    if (verbose)
    {
        if (model.isEmpty()) {
            qDebug() << "Aircraft3DModels: no model for" << info->m_model << info->m_operatorICAO;
        } else {
            qDebug() << "Aircraft3DModels: matched" << info->m_model << info->m_operatorICAO << "to" << model;
        }
    }

    if (model.isEmpty()) {
        return false;
    }
    offsetsForType(type, altitudeOffset, labelOffset);
    return true;
}
