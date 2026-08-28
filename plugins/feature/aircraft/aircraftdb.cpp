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

// Session persistence for the Aircraft feature: the collated aircraft, flights,
// documents and ATC log are kept in a SQLite database (vendored amalgamation -
// public domain, no Qt SQL dependency), written every minute and on exit, and
// restored on startup. Runs on the tracker thread, so the GUI never blocks on a
// save. Aircraft older than the removal timeout are pruned on load; the
// self-contained ATC log is restored in full.

#include <cmath>
#include <limits>

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>

#include "sqlite/sqlite3.h"

#include "aircrafttracker.h"

// Current schema version. v2 added the aliases column to flights.
#define AIRCRAFT_DB_VERSION 3

// Where the session is kept. The setting wins; otherwise the default location, whose
// directory may not exist yet
QString AircraftTracker::databasePath(const AircraftSettings& settings)
{
    if (!settings.m_databaseFilename.isEmpty())
    {
        QDir().mkpath(QFileInfo(settings.m_databaseFilename).absolutePath());
        return settings.m_databaseFilename;
    }

    QStringList locations = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation);
    QDir().mkpath(locations[0]);
    return locations[0] + "/aircraft.db";
}

static bool execSQL(sqlite3 *db, const char *sql)
{
    char *errMsg = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &errMsg) != SQLITE_OK)
    {
        qWarning() << "AircraftTracker: SQL failed:" << sql << ":" << (errMsg ? errMsg : "");
        sqlite3_free(errMsg);
        return false;
    }
    return true;
}

// Run a prepared INSERT and say whether the row went in.
//
// sqlite3_step returns SQLITE_DONE for a successful INSERT; anything else - a full disk,
// an I/O error, a constraint - means the row is not there. Ignoring it let a failed save
// look successful, and saveDatabase() then cleared the dirty flag, so nothing retried and
// the database silently stayed as it was.
static bool stepInsert(sqlite3 *db, sqlite3_stmt *stmt, const char *what, bool& failed)
{
    if (failed) {
        return false;       // Already given up on this transaction
    }
    if (sqlite3_step(stmt) != SQLITE_DONE)
    {
        qWarning() << "AircraftTracker::saveDatabase: writing" << what << "failed:"
                   << sqlite3_errmsg(db);
        failed = true;
        return false;
    }
    return true;
}

static void bindText(sqlite3_stmt *stmt, int idx, const QString& text)
{
    QByteArray utf8 = text.toUtf8();
    sqlite3_bind_text(stmt, idx, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
}

static QString columnText(sqlite3_stmt *stmt, int idx)
{
    const char *text = (const char *) sqlite3_column_text(stmt, idx);
    return text ? QString::fromUtf8(text) : QString();
}

static QDateTime columnDateTime(sqlite3_stmt *stmt, int idx)
{
    qint64 ms = sqlite3_column_int64(stmt, idx);
    return ms ? QDateTime::fromMSecsSinceEpoch(ms) : QDateTime();
}

static qint64 dateTimeValue(const QDateTime& dateTime)
{
    return dateTime.isValid() ? dateTime.toMSecsSinceEpoch() : 0;
}

// Add a column if the table does not already have it.
//
// Migrating on the version number alone is not enough: a column added to CREATE TABLE
// and to the migration for the version BELOW the current one never reaches a database
// that is already at the current version, and the next INSERT then binds more values
// than the table has columns and fails to prepare - taking the whole save with it.
// Asking the table what it actually has is immune to that.
static bool tableExists(sqlite3 *db, const char *table)
{
    sqlite3_stmt *stmt = nullptr;
    bool exists = false;
    if (sqlite3_prepare_v2(db, "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
            -1, &stmt, nullptr) == SQLITE_OK)
    {
        sqlite3_bind_text(stmt, 1, table, -1, SQLITE_STATIC);
        exists = (sqlite3_step(stmt) == SQLITE_ROW);
    }
    sqlite3_finalize(stmt);
    return exists;
}

static void ensureColumn(sqlite3 *db, const char *table, const char *column, const char *decl)
{
    sqlite3_stmt *stmt = nullptr;
    const QString query = QString("PRAGMA table_info(%1)").arg(table);
    bool found = false;
    if (sqlite3_prepare_v2(db, query.toUtf8().constData(), -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            const unsigned char *name = sqlite3_column_text(stmt, 1);
            if (name && (qstrcmp((const char *) name, column) == 0))
            {
                found = true;
                break;
            }
        }
    }
    sqlite3_finalize(stmt);
    if (!found)
    {
        qDebug() << "AircraftTracker::openDatabase: adding" << table << "." << column;
        execSQL(db, QString("ALTER TABLE %1 ADD COLUMN %2 %3").arg(table).arg(column).arg(decl).toUtf8().constData());
    }
}

bool AircraftTracker::openDatabase()
{
    QString path = databasePath(m_settings);
    if (sqlite3_open(path.toUtf8().constData(), &m_db) != SQLITE_OK)
    {
        qWarning() << "AircraftTracker::openDatabase: Failed to open" << path << ":" << sqlite3_errmsg(m_db);
        sqlite3_close(m_db);
        m_db = nullptr;
        return false;
    }

    // Write-ahead logging is considerably faster for the periodic full rewrite
    execSQL(m_db, "PRAGMA journal_mode=WAL");
    execSQL(m_db, "PRAGMA synchronous=NORMAL");

    // Whether the all time table is about to be created, which says this database has
    // never had one and so its existing aircraft have never been counted into it
    const bool seenIsNew = !tableExists(m_db, "seen");

    bool ok = execSQL(m_db,
        "CREATE TABLE IF NOT EXISTS meta (version INTEGER);"
        "CREATE TABLE IF NOT EXISTS aircraft ("
            "id INTEGER PRIMARY KEY, icao INTEGER, registration TEXT, flight TEXT,"
            "position_valid INTEGER, latitude REAL, longitude REAL,"
            "altitude_valid INTEGER, altitude REAL,"
            "heading_valid INTEGER, heading REAL,"
            "speed_valid INTEGER, speed REAL,"
            "position_time INTEGER, last_seen INTEGER,"
            "departure TEXT, arrival TEXT, route TEXT,"
            "last_document TEXT, messages INTEGER, model3d TEXT);"
        "CREATE TABLE IF NOT EXISTS sources ("
            "aircraft_id INTEGER, protocol INTEGER, frequency INTEGER, count INTEGER);"
        // The same tally per flight rather than per airframe - what a given flight was
        // heard on, which is what the Map shows. Added without a version bump because
        // IF NOT EXISTS creates it on the next open of an older database, where bumping
        // would send every existing database down the "starting afresh" branch and
        // delete everything in it.
        "CREATE TABLE IF NOT EXISTS flight_sources ("
            "flight_id INTEGER, protocol INTEGER, frequency INTEGER, count INTEGER);"
        "CREATE TABLE IF NOT EXISTS track ("
            "flight_id INTEGER, seq INTEGER, latitude REAL, longitude REAL, altitude REAL, time INTEGER);"
        "CREATE TABLE IF NOT EXISTS flights ("
            "id INTEGER PRIMARY KEY, aircraft_id INTEGER, flight TEXT,"
            "first_seen INTEGER, last_seen INTEGER,"
            "departure TEXT, arrival TEXT, route TEXT, is_current INTEGER,"
            "aliases TEXT DEFAULT '', messages INTEGER DEFAULT 0,"
            "out_time INTEGER DEFAULT 0, off_time INTEGER DEFAULT 0,"
            "on_time INTEGER DEFAULT 0, in_time INTEGER DEFAULT 0);"
        "CREATE TABLE IF NOT EXISTS documents ("
            "flight_id INTEGER, kind INTEGER, title TEXT, text TEXT, received INTEGER);"
        "CREATE TABLE IF NOT EXISTS profile ("
            "flight_id INTEGER, time INTEGER, altitude REAL, speed REAL);"
        "CREATE TABLE IF NOT EXISTS atc ("
            "time INTEGER, protocol TEXT, uplink INTEGER,"
            "from_station TEXT, to_station TEXT, message TEXT, tooltip TEXT,"
            "aircraft_id INTEGER, flight_id INTEGER, map_name TEXT);"
        // Every airframe ever heard, for the all time aircraft count. One row each, no
        // matter how often it comes back, which is the whole reason a count cannot do
        // the job. Kept out of every DELETE below: this is the one table that has to
        // outlive both the retention period and a schema reset, and losing it would
        // silently reset a figure the user cannot recover. Added without a version bump
        // because IF NOT EXISTS creates it on the next open of an older database - and
        // bumping would send every existing v3 database down the "starting afresh"
        // branch, which deletes everything
        "CREATE TABLE IF NOT EXISTS seen ("
            "key TEXT PRIMARY KEY, first_seen INTEGER, last_seen INTEGER);");
    if (!ok)
    {
        sqlite3_close(m_db);
        m_db = nullptr;
        return false;
    }

    // An existing database already knows about aircraft heard before the all time table
    // existed, so seed it from them rather than starting the count at nothing on an
    // installation that has been running for months. Only what is still within the
    // retention period can be recovered, so the figure starts low and is honest about it
    // via the "since" date. Done once, on creation: repeating it every open would undo a
    // reset, since the aircraft table would put the same rows straight back.
    if (seenIsNew)
    {
        execSQL(m_db,
            "INSERT OR IGNORE INTO seen (key, first_seen, last_seen) "
            "SELECT CASE WHEN icao IS NOT NULL AND icao != 0 THEN printf('%06X', icao) "
                       "ELSE 'R:' || registration END, last_seen, last_seen "
            "FROM aircraft "
            "WHERE (icao IS NOT NULL AND icao != 0) "
               "OR (registration IS NOT NULL AND registration != '')");
    }

    // Columns added after a version bump, which the version chain below would miss on a
    // database already at the current version
    ensureColumn(m_db, "flights", "messages", "INTEGER DEFAULT 0");
    ensureColumn(m_db, "flights", "out_time", "INTEGER DEFAULT 0");
    ensureColumn(m_db, "flights", "off_time", "INTEGER DEFAULT 0");
    ensureColumn(m_db, "flights", "on_time", "INTEGER DEFAULT 0");
    ensureColumn(m_db, "flights", "in_time", "INTEGER DEFAULT 0");
    // The departure, arrival and route are stored, so what decides whether a report may
    // replace them has to be stored with them - otherwise the first message after a
    // restart wins however delayed it is, which is the case the timestamp exists for
    ensureColumn(m_db, "flights", "route_filed", "INTEGER DEFAULT 0");
    // A model is no use on its own: without the offsets it sits half underground, and
    // without knowing whether we chose it, the livery setting may not replace it
    ensureColumn(m_db, "aircraft", "model_altitude_offset", "REAL DEFAULT 0");
    ensureColumn(m_db, "aircraft", "label_altitude_offset", "REAL DEFAULT 0");
    // Defaulted to -1, which no row written since says: that is what distinguishes a
    // row from before these columns existed, whose model has no offsets to go with it,
    // from one that legitimately holds a zero offset
    ensureColumn(m_db, "aircraft", "model_is_ours", "INTEGER DEFAULT -1");

    // Schema version, for future migrations
    sqlite3_stmt *stmt = nullptr;
    int version = 0;
    if (sqlite3_prepare_v2(m_db, "SELECT version FROM meta", -1, &stmt, nullptr) == SQLITE_OK)
    {
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            version = sqlite3_column_int(stmt, 0);
        }
    }
    sqlite3_finalize(stmt);
    if (version == 0) {
        execSQL(m_db, "INSERT INTO meta (version) VALUES (" QT_STRINGIFY(AIRCRAFT_DB_VERSION) ")");
    } else if ((version == 1) || (version == 2)) {
        if (version == 1)
        {
            // v1 -> v2: flight name aliases
            qDebug() << "AircraftTracker::openDatabase: Migrating database from version 1";
            execSQL(m_db, "ALTER TABLE flights ADD COLUMN aliases TEXT DEFAULT ''");
        }
        // v2 -> v3: the track belongs to the flight rather than the airframe. The old
        // rows are keyed by aircraft and cannot be divided between that aircraft's
        // flights after the fact, so they are discarded - tracks re-accumulate from the
        // next report, and they were only ever kept for m_retentionDays anyway
        qDebug() << "AircraftTracker::openDatabase: Migrating database from version" << version
                 << "- stored tracks are discarded";
        // v2 -> v3 also gives the flight its own message count. The Past Flights table
        // showed the document count under a "Msgs" heading, which reads as zero for any
        // flight heard only on ADS-B
        execSQL(m_db, "ALTER TABLE flights ADD COLUMN messages INTEGER DEFAULT 0");
        execSQL(m_db,
            "DROP TABLE IF EXISTS track;"
            "CREATE TABLE track ("
                "flight_id INTEGER, seq INTEGER, latitude REAL, longitude REAL, altitude REAL, time INTEGER);");
        execSQL(m_db, "UPDATE meta SET version = " QT_STRINGIFY(AIRCRAFT_DB_VERSION));
    } else if (version != AIRCRAFT_DB_VERSION) {
        qDebug() << "AircraftTracker::openDatabase: Database version" << version << "- starting afresh";
        execSQL(m_db,
            "DELETE FROM aircraft; DELETE FROM sources; DELETE FROM track;"
            "DELETE FROM flight_sources;"
            "DELETE FROM flights; DELETE FROM documents; DELETE FROM profile;"
            "DELETE FROM atc; DELETE FROM meta;");
        execSQL(m_db, "INSERT INTO meta (version) VALUES (" QT_STRINGIFY(AIRCRAFT_DB_VERSION) ")");
    }

    return true;
}

// The user has chosen a different file for the session. If nothing is there yet the
// current session moves with them, so the aircraft, flights and messages they have
// collected are not left behind; if a database is already there it is theirs and is
// opened as it stands, replacing what is in memory.
// Put the setting back to the file actually in use, and tell the GUI - it owns the
// settings that are serialised, so leaving it holding the path that failed would save a
// pointer to a database nothing is writing to.
void AircraftTracker::revertDatabaseFilename(const QString& path)
{
    m_settings.m_databaseFilename =
        (path == AircraftSettings::defaultDatabaseFilename()) ? QString() : path;
    emit databaseFilenameReverted(m_settings.m_databaseFilename);
}

void AircraftTracker::changeDatabase(const QString& oldPath)
{
    const QString newPath = databasePath(m_settings);

    if (newPath == oldPath) {
        return;
    }

    // Make sure everything collected so far is on disk before the file moves. If it
    // could not be, the switch does not happen: adopting an existing database calls
    // deleteAll() below, which would throw away the very session that failed to save.
    // The setting goes back to the file actually in use, so the two do not disagree.
    if (m_db && !m_loadingDatabase && !saveDatabase())
    {
        qWarning() << "AircraftTracker::changeDatabase: could not save the session to"
                   << oldPath << "- keeping it rather than switching to" << newPath;
        revertDatabaseFilename(oldPath);
        return;
    }
    closeDatabase();

    const bool haveExisting = QFileInfo::exists(newPath) && (QFileInfo(newPath).size() > 0);

    bool moved = false;
    if (!haveExisting && QFileInfo::exists(oldPath))
    {
        // QFile::rename copies and removes when the paths are on different volumes
        moved = QFile::rename(oldPath, newPath);
        if (moved) {
            qDebug() << "AircraftTracker::changeDatabase: moved the session from" << oldPath << "to" << newPath;
        } else {
            qWarning() << "AircraftTracker::changeDatabase: could not move" << oldPath << "to" << newPath
                       << "- the session stays in memory and will be written to the new file";
        }
    }

    if (!openDatabase())
    {
        // Without this there is no database at all: the old one is closed and the new one
        // would not open, so everything from here on would be kept in memory and lost on
        // exit, silently. Go back to the file that was working.
        qWarning() << "AircraftTracker::changeDatabase: could not open" << newPath
                   << "- going back to" << oldPath;
        // The file has to go back before it can be reopened. Reverting the setting alone
        // would point at a path the history is no longer at, and opening it would create
        // an empty database there - leaving every restored aircraft to be written out
        // again over nothing, with the real history stranded under the new name.
        if (moved && !QFile::rename(newPath, oldPath))
        {
            qWarning() << "AircraftTracker::changeDatabase: could not move" << newPath
                       << "back to" << oldPath << "- staying with it rather than opening"
                       << "an empty database over the session";
            if (!openDatabase()) {
                qWarning() << "AircraftTracker::changeDatabase: and could not reopen it"
                           << "- the session is now in memory only";
            }
            return;
        }
        revertDatabaseFilename(oldPath);
        if (!openDatabase()) {
            qWarning() << "AircraftTracker::changeDatabase: could not reopen" << oldPath
                       << "either - the session is now in memory only";
        }
        return;
    }

    if (haveExisting)
    {
        // Their database, their contents: start again from what it holds
        deleteAll();
        loadDatabase();
    }
    else
    {
        // Everything is still in memory; write it where it now belongs. The file may or
        // may not have been moved successfully, so do not assume the points came with it
        forgetSavedTracks();
        m_dbDirty = true;
        saveDatabase();
    }
}

void AircraftTracker::closeDatabase()
{
    if (m_db)
    {
        sqlite3_close(m_db);
        m_db = nullptr;
    }
}

// Write the whole session state in one transaction, replacing the previous save
// The small tables are still rewritten whole - all of them together came to about
// 21000 rows against 1.06 MILLION in track and profile, and rewriting those two every
// minute was blocking the tracker thread for the best part of a second. Measured on a
// real 72 MB session database: 870506 track and 194527 profile rows, 0.93 s to delete
// and re-insert, once a minute and growing for as long as the session ran. That stall
// was visible as a sawtooth on the message rate chart, because reports queued behind it
// and were then all counted into whichever sample followed.
//
// Track and profile only ever grow, so they are written incrementally: each flight
// remembers how many of its points are already stored and the save appends the rest.
// The three ways that can stop being true are all accounted for - a flight merged into
// another has its arrays rebuilt (m_flightsToRewrite), a flight discarded by the
// retention period has to take its rows with it (m_flightsToDelete), and clearing
// everything invalidates the lot (m_dbTrackWipe).
bool AircraftTracker::saveDatabase()
{
    // Nothing to do if nothing changed since the last save - which counts as saved.
    // No database at all does not: nothing has been written and nothing will be.
    if (!m_db) {
        return false;
    }
    if (!m_dbDirty) {
        return true;
    }

    // The save replaces the whole database, so a DELETE that fails after a BEGIN that
    // succeeded would leave the old rows in place and the new ones appended to them
    if (!execSQL(m_db, "BEGIN"))
    {
        qWarning() << "AircraftTracker::saveDatabase: could not start a transaction";
        return false;       // m_dbDirty stays set, so the next tick tries again
    }
    if (!execSQL(m_db,
            "DELETE FROM aircraft; DELETE FROM sources; DELETE FROM flight_sources;"
            "DELETE FROM flights; DELETE FROM documents;"
            "DELETE FROM atc;"))
    {
        execSQL(m_db, "ROLLBACK");
        return false;
    }

    // What is stored for these flights is stale, so it goes before anything is written
    if (m_dbTrackWipe)
    {
        if (!execSQL(m_db, "DELETE FROM track; DELETE FROM profile;"))
        {
            execSQL(m_db, "ROLLBACK");
            return false;
        }
        // The flag is cleared after the COMMIT, not here. forgetSavedTracks() has already
        // set every flight's saved count to zero, so a save that rolls back after this
        // point leaves the old rows in the database and the wipe forgotten - and the next
        // save appends the whole of every track on top of them.
    }
    else if (!m_flightsToDelete.isEmpty() || !m_flightsToRewrite.isEmpty())
    {
        sqlite3_stmt *dropStmt = nullptr;
        if (sqlite3_prepare_v2(m_db,
                "DELETE FROM track WHERE flight_id = ?", -1, &dropStmt, nullptr) == SQLITE_OK)
        {
            for (quint64 id : m_flightsToDelete + m_flightsToRewrite)
            {
                sqlite3_reset(dropStmt);
                sqlite3_bind_int64(dropStmt, 1, (qint64) id);
                sqlite3_step(dropStmt);
            }
        }
        sqlite3_finalize(dropStmt);

        dropStmt = nullptr;
        if (sqlite3_prepare_v2(m_db,
                "DELETE FROM profile WHERE flight_id = ?", -1, &dropStmt, nullptr) == SQLITE_OK)
        {
            for (quint64 id : m_flightsToDelete + m_flightsToRewrite)
            {
                sqlite3_reset(dropStmt);
                sqlite3_bind_int64(dropStmt, 1, (qint64) id);
                sqlite3_step(dropStmt);
            }
        }
        sqlite3_finalize(dropStmt);
    }

    bool failed = false;

    sqlite3_stmt *aircraftStmt = nullptr;
    sqlite3_stmt *sourceStmt = nullptr;
    sqlite3_stmt *trackStmt = nullptr;
    sqlite3_stmt *flightStmt = nullptr;
    sqlite3_stmt *flightSourceStmt = nullptr;
    sqlite3_stmt *documentStmt = nullptr;
    sqlite3_stmt *profileStmt = nullptr;
    sqlite3_stmt *atcStmt = nullptr;
    sqlite3_prepare_v2(m_db,
        "INSERT INTO aircraft VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", -1, &aircraftStmt, nullptr);
    sqlite3_prepare_v2(m_db, "INSERT INTO sources VALUES (?,?,?,?)", -1, &sourceStmt, nullptr);
    sqlite3_prepare_v2(m_db, "INSERT INTO flight_sources VALUES (?,?,?,?)", -1, &flightSourceStmt, nullptr);
    sqlite3_prepare_v2(m_db, "INSERT INTO track VALUES (?,?,?,?,?,?)", -1, &trackStmt, nullptr);
    sqlite3_prepare_v2(m_db, "INSERT INTO flights VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", -1, &flightStmt, nullptr);
    sqlite3_prepare_v2(m_db, "INSERT INTO documents VALUES (?,?,?,?,?)", -1, &documentStmt, nullptr);
    sqlite3_prepare_v2(m_db, "INSERT INTO profile VALUES (?,?,?,?)", -1, &profileStmt, nullptr);
    sqlite3_prepare_v2(m_db, "INSERT INTO atc VALUES (?,?,?,?,?,?,?,?,?,?)", -1, &atcStmt, nullptr);
    if (!aircraftStmt || !sourceStmt || !trackStmt || !flightStmt || !documentStmt
        || !profileStmt || !atcStmt || !flightSourceStmt)
    {
        // Name the one that failed. sqlite3_errmsg reports the LAST call, which is a
        // successful prepare when an earlier one failed, and so said "not an error"
        const char *which = !aircraftStmt ? "aircraft" : !sourceStmt ? "sources"
                          : !trackStmt ? "track" : !flightStmt ? "flights"
                          : !documentStmt ? "documents" : !profileStmt ? "profile"
                          : !atcStmt ? "atc" : "flight_sources";
        qWarning() << "AircraftTracker::saveDatabase: could not prepare the" << which << "insert";
        qWarning() << "AircraftTracker::saveDatabase: Failed to prepare statements:" << sqlite3_errmsg(m_db);
        execSQL(m_db, "ROLLBACK");
        sqlite3_finalize(aircraftStmt);
        sqlite3_finalize(sourceStmt);
        sqlite3_finalize(trackStmt);
        sqlite3_finalize(flightStmt);
        sqlite3_finalize(documentStmt);
        sqlite3_finalize(profileStmt);
        sqlite3_finalize(atcStmt);
        sqlite3_finalize(flightSourceStmt);
        return false;
    }

    for (const TrackedAircraft *item : m_aircraft)
    {
        sqlite3_reset(aircraftStmt);
        sqlite3_bind_int64(aircraftStmt, 1, (qint64) item->m_id);
        sqlite3_bind_int64(aircraftStmt, 2, item->m_icao);
        bindText(aircraftStmt, 3, item->m_registration);
        bindText(aircraftStmt, 4, item->m_flight);
        sqlite3_bind_int(aircraftStmt, 5, item->m_positionValid ? 1 : 0);
        sqlite3_bind_double(aircraftStmt, 6, item->m_latitude);
        sqlite3_bind_double(aircraftStmt, 7, item->m_longitude);
        sqlite3_bind_int(aircraftStmt, 8, item->m_altitudeValid ? 1 : 0);
        sqlite3_bind_double(aircraftStmt, 9, item->m_altitudeFt);
        sqlite3_bind_int(aircraftStmt, 10, item->m_headingValid ? 1 : 0);
        sqlite3_bind_double(aircraftStmt, 11, item->m_heading);
        sqlite3_bind_int(aircraftStmt, 12, item->m_speedValid ? 1 : 0);
        sqlite3_bind_double(aircraftStmt, 13, item->m_speedKts);
        sqlite3_bind_int64(aircraftStmt, 14, dateTimeValue(item->m_positionDateTime));
        sqlite3_bind_int64(aircraftStmt, 15, dateTimeValue(item->m_lastSeen));
        bindText(aircraftStmt, 16, item->m_departure);
        bindText(aircraftStmt, 17, item->m_arrival);
        bindText(aircraftStmt, 18, item->m_route);
        bindText(aircraftStmt, 19, item->m_lastDocumentText);
        sqlite3_bind_int(aircraftStmt, 20, item->m_messages);
        bindText(aircraftStmt, 21, item->m_model3D);
        sqlite3_bind_double(aircraftStmt, 22, item->m_modelAltitudeOffset);
        sqlite3_bind_double(aircraftStmt, 23, item->m_labelAltitudeOffset);
        sqlite3_bind_int(aircraftStmt, 24, item->m_modelIsOurs ? 1 : 0);
        stepInsert(m_db, aircraftStmt, "aircraft", failed);

        QHashIterator<QPair<int, qint64>, int> sourceIt(item->m_sources);
        while (sourceIt.hasNext())
        {
            sourceIt.next();
            sqlite3_reset(sourceStmt);
            sqlite3_bind_int64(sourceStmt, 1, (qint64) item->m_id);
            sqlite3_bind_int(sourceStmt, 2, sourceIt.key().first);
            sqlite3_bind_int64(sourceStmt, 3, sourceIt.key().second);
            sqlite3_bind_int(sourceStmt, 4, sourceIt.value());
            stepInsert(m_db, sourceStmt, "sources", failed);
        }

        for (TrackedFlight *flight : item->m_flights)
        {
            // Only what is not stored yet. m_trackSaved is advanced after the commit,
            // so a rolled back transaction leaves it describing what is really there.
            for (int p = flight->m_trackSaved; p < flight->m_track.size(); p++)
            {
                sqlite3_reset(trackStmt);
                sqlite3_bind_int64(trackStmt, 1, (qint64) flight->m_id);
                sqlite3_bind_int(trackStmt, 2, p);
                sqlite3_bind_double(trackStmt, 3, flight->m_track[p].latitude());
                sqlite3_bind_double(trackStmt, 4, flight->m_track[p].longitude());
                sqlite3_bind_double(trackStmt, 5, flight->m_track[p].altitude());
                sqlite3_bind_int64(trackStmt, 6, dateTimeValue(flight->m_trackTimes[p]));
                stepInsert(m_db, trackStmt, "track", failed);
            }

            sqlite3_reset(flightStmt);
            sqlite3_bind_int64(flightStmt, 1, (qint64) flight->m_id);
            sqlite3_bind_int64(flightStmt, 2, (qint64) item->m_id);
            bindText(flightStmt, 3, flight->m_flight);
            sqlite3_bind_int64(flightStmt, 4, dateTimeValue(flight->m_firstSeen));
            sqlite3_bind_int64(flightStmt, 5, dateTimeValue(flight->m_lastSeen));
            bindText(flightStmt, 6, flight->m_departure);
            bindText(flightStmt, 7, flight->m_arrival);
            bindText(flightStmt, 8, flight->m_route);
            sqlite3_bind_int(flightStmt, 9, flight == item->m_currentFlight ? 1 : 0);
            bindText(flightStmt, 10, flight->m_aliases.join(","));
            sqlite3_bind_int(flightStmt, 11, flight->m_messages);
            sqlite3_bind_int64(flightStmt, 12, dateTimeValue(flight->m_out));
            sqlite3_bind_int64(flightStmt, 13, dateTimeValue(flight->m_off));
            sqlite3_bind_int64(flightStmt, 14, dateTimeValue(flight->m_on));
            sqlite3_bind_int64(flightStmt, 15, dateTimeValue(flight->m_in));
            sqlite3_bind_int64(flightStmt, 16, dateTimeValue(flight->m_routeFiled));
            stepInsert(m_db, flightStmt, "flights", failed);

            QHashIterator<QPair<int, qint64>, int> flightSourceIt(flight->m_sources);
            while (flightSourceIt.hasNext())
            {
                flightSourceIt.next();
                sqlite3_reset(flightSourceStmt);
                sqlite3_bind_int64(flightSourceStmt, 1, (qint64) flight->m_id);
                sqlite3_bind_int(flightSourceStmt, 2, flightSourceIt.key().first);
                sqlite3_bind_int64(flightSourceStmt, 3, flightSourceIt.key().second);
                sqlite3_bind_int(flightSourceStmt, 4, flightSourceIt.value());
                stepInsert(m_db, flightSourceStmt, "flight_sources", failed);
            }

            for (const TrackedDocument *doc : flight->m_documents)
            {
                sqlite3_reset(documentStmt);
                sqlite3_bind_int64(documentStmt, 1, (qint64) flight->m_id);
                sqlite3_bind_int(documentStmt, 2, doc->m_kind);
                bindText(documentStmt, 3, doc->m_title);
                bindText(documentStmt, 4, doc->m_text);
                sqlite3_bind_int64(documentStmt, 5, dateTimeValue(doc->m_received));
                stepInsert(m_db, documentStmt, "documents", failed);
            }

            for (int p = flight->m_profileSaved; p < flight->m_profileTimes.size(); p++)
            {
                sqlite3_reset(profileStmt);
                sqlite3_bind_int64(profileStmt, 1, (qint64) flight->m_id);
                sqlite3_bind_int64(profileStmt, 2, flight->m_profileTimes[p]);
                if (std::isnan(flight->m_profileAltFt[p])) {
                    sqlite3_bind_null(profileStmt, 3);
                } else {
                    sqlite3_bind_double(profileStmt, 3, flight->m_profileAltFt[p]);
                }
                if (std::isnan(flight->m_profileSpeedKts[p])) {
                    sqlite3_bind_null(profileStmt, 4);
                } else {
                    sqlite3_bind_double(profileStmt, 4, flight->m_profileSpeedKts[p]);
                }
                stepInsert(m_db, profileStmt, "profile", failed);
            }
        }
    }

    for (const AtcEvent& e : m_atcLog)
    {
        sqlite3_reset(atcStmt);
        sqlite3_bind_int64(atcStmt, 1, dateTimeValue(e.m_received));
        bindText(atcStmt, 2, e.m_protocol);
        sqlite3_bind_int(atcStmt, 3, e.m_uplink ? 1 : 0);
        bindText(atcStmt, 4, e.m_from);
        bindText(atcStmt, 5, e.m_to);
        bindText(atcStmt, 6, e.m_message);
        bindText(atcStmt, 7, e.m_tooltip);
        sqlite3_bind_int64(atcStmt, 8, (qint64) e.m_aircraftId);
        sqlite3_bind_int64(atcStmt, 9, (qint64) e.m_flightId);
        bindText(atcStmt, 10, e.m_mapName);
        stepInsert(m_db, atcStmt, "atc", failed);
    }

    sqlite3_finalize(aircraftStmt);
    sqlite3_finalize(sourceStmt);
    sqlite3_finalize(trackStmt);
    sqlite3_finalize(flightStmt);
    sqlite3_finalize(documentStmt);
    sqlite3_finalize(profileStmt);
    sqlite3_finalize(atcStmt);
    sqlite3_finalize(flightSourceStmt);
    if (failed)
    {
        // Leave the previous contents alone and keep the dirty flag, so the next save
        // tries the whole thing again rather than leaving a half written database behind
        qWarning() << "AircraftTracker::saveDatabase: rolling back, session not saved";
        execSQL(m_db, "ROLLBACK");
        return false;
    }
    if (!execSQL(m_db, "COMMIT"))
    {
        qWarning() << "AircraftTracker::saveDatabase: commit failed, session not saved";
        execSQL(m_db, "ROLLBACK");
        return false;
    }

    // Only now that the rows are really there. Doing this as they were written would
    // have a rolled back save leave every flight claiming points the database does not
    // hold, and the next save would append after them - a permanent hole in the track.
    for (TrackedAircraft *item : m_aircraft)
    {
        for (TrackedFlight *flight : item->m_flights)
        {
            flight->m_trackSaved = flight->m_track.size();
            flight->m_profileSaved = flight->m_profileTimes.size();
        }
    }
    m_flightsToDelete.clear();
    m_flightsToRewrite.clear();
    m_dbTrackWipe = false;

    m_dbDirty = false;
    return true;
}

// Nothing stored relates to what is in memory any more, so the next save empties both
// tables and writes every point again
void AircraftTracker::forgetSavedTracks()
{
    m_dbTrackWipe = true;
    m_flightsToDelete.clear();
    m_flightsToRewrite.clear();
    for (TrackedAircraft *item : m_aircraft)
    {
        for (TrackedFlight *flight : item->m_flights)
        {
            flight->m_trackSaved = 0;
            flight->m_profileSaved = 0;
        }
    }
}

// Restore the previous session. Aircraft older than the removal timeout are
// pruned afterwards; the ATC log is restored in full. The GUI, if any, gets the
// restored state when it connects and requests a resync.
void AircraftTracker::loadDatabase()
{
    if (!m_db) {
        return;
    }

    m_loadingDatabase = true;
    QHash<qint64, TrackedAircraft *> aircraftById;
    QHash<qint64, TrackedFlight *> flightsById;
    sqlite3_stmt *stmt = nullptr;

    // Aircraft
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM aircraft", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            TrackedAircraft *item = new TrackedAircraft();
            qint64 id = sqlite3_column_int64(stmt, 0);
            item->m_id = (quint64) id;
            m_nextId = std::max(m_nextId, item->m_id + 1);
            item->m_icao = (quint32) sqlite3_column_int64(stmt, 1);
            item->m_registration = columnText(stmt, 2);
            item->m_flight = columnText(stmt, 3);
            item->m_positionValid = sqlite3_column_int(stmt, 4) != 0;
            item->m_latitude = (float) sqlite3_column_double(stmt, 5);
            item->m_longitude = (float) sqlite3_column_double(stmt, 6);
            item->m_altitudeValid = sqlite3_column_int(stmt, 7) != 0;
            item->m_altitudeFt = (float) sqlite3_column_double(stmt, 8);
            item->m_headingValid = sqlite3_column_int(stmt, 9) != 0;
            item->m_heading = (float) sqlite3_column_double(stmt, 10);
            item->m_speedValid = sqlite3_column_int(stmt, 11) != 0;
            item->m_speedKts = (float) sqlite3_column_double(stmt, 12);
            item->m_positionDateTime = columnDateTime(stmt, 13);
            item->m_lastSeen = columnDateTime(stmt, 14);
            item->m_departure = columnText(stmt, 15);
            item->m_arrival = columnText(stmt, 16);
            item->m_route = columnText(stmt, 17);
            item->m_lastDocumentText = columnText(stmt, 18);
            item->m_messages = sqlite3_column_int(stmt, 19);
            item->m_model3D = columnText(stmt, 20);
            // Added by ensureColumn(), so a database written before they existed reads
            // them as the defaults - which is the same as never having had a model
            item->m_modelAltitudeOffset = (float) sqlite3_column_double(stmt, 21);
            item->m_labelAltitudeOffset = (float) sqlite3_column_double(stmt, 22);
            const int modelIsOurs = sqlite3_column_int(stmt, 23);
            item->m_modelIsOurs = modelIsOurs > 0;
            // A row written before these columns existed has the model but nothing that
            // goes with it, so the model is dropped and chosen again on the next report
            // rather than drawn at the wrong height and never replaced
            if (modelIsOurs < 0) {
                item->m_model3D.clear();
            }

            m_aircraft.append(item);
            aircraftById.insert(id, item);
            if (item->m_icao) {
                m_byKey.insert(QString("I%1").arg(item->m_icao, 6, 16, QChar('0')), item);
            }
            if (!item->m_registration.isEmpty()) {
                m_byKey.insert("R" + item->m_registration, item);
            }
            if (!item->m_flight.isEmpty()) {
                m_byKey.insert("F" + item->m_flight, item);
            }
        }
    }
    sqlite3_finalize(stmt);
    stmt = nullptr;

    // Protocols/frequencies heard on
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM sources", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            TrackedAircraft *item = aircraftById.value(sqlite3_column_int64(stmt, 0));
            if (item) {
                item->m_sources.insert(
                    qMakePair(sqlite3_column_int(stmt, 1), (qint64) sqlite3_column_int64(stmt, 2)),
                    sqlite3_column_int(stmt, 3));
            }
        }
    }
    sqlite3_finalize(stmt);
    stmt = nullptr;

    // Flights
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM flights ORDER BY id", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            TrackedAircraft *item = aircraftById.value(sqlite3_column_int64(stmt, 1));
            if (!item) {
                continue;
            }
            TrackedFlight *flight = new TrackedFlight();
            flight->m_id = (quint64) sqlite3_column_int64(stmt, 0);
            m_nextId = std::max(m_nextId, flight->m_id + 1);
            flight->m_aircraft = item;
            flight->m_flight = columnText(stmt, 2);
            flight->m_firstSeen = columnDateTime(stmt, 3);
            flight->m_lastSeen = columnDateTime(stmt, 4);
            flight->m_departure = columnText(stmt, 5);
            flight->m_arrival = columnText(stmt, 6);
            flight->m_route = columnText(stmt, 7);
            flight->m_messages = sqlite3_column_int(stmt, 10);
            flight->m_out = columnDateTime(stmt, 11);
            flight->m_off = columnDateTime(stmt, 12);
            flight->m_on = columnDateTime(stmt, 13);
            flight->m_in = columnDateTime(stmt, 14);
            flight->m_routeFiled = columnDateTime(stmt, 15);
            QString aliases = columnText(stmt, 9);
            if (!aliases.isEmpty()) {
                flight->m_aliases = aliases.split(',');
            }
            item->m_flights.append(flight);
            if (sqlite3_column_int(stmt, 8)) {
                item->m_currentFlight = flight;
            }
            flightsById.insert(sqlite3_column_int64(stmt, 0), flight);
        }
    }
    sqlite3_finalize(stmt);
    stmt = nullptr;

    // Protocols/frequencies each flight was heard on. After the flights, because it is
    // keyed on their ids.
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM flight_sources", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            TrackedFlight *flight = flightsById.value(sqlite3_column_int64(stmt, 0));
            if (flight) {
                flight->m_sources.insert(
                    qMakePair(sqlite3_column_int(stmt, 1), (qint64) sqlite3_column_int64(stmt, 2)),
                    sqlite3_column_int(stmt, 3));
            }
        }
    }
    sqlite3_finalize(stmt);
    stmt = nullptr;

    // Tracks. Keyed by flight, so this has to come after the flights are loaded.
    // Exactly 0N 0E is a zeroed-out field saved before the filter for them existed,
    // not a position.
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM track ORDER BY flight_id, seq", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            TrackedFlight *flight = flightsById.value(sqlite3_column_int64(stmt, 0));
            double latitude = sqlite3_column_double(stmt, 2);
            double longitude = sqlite3_column_double(stmt, 3);
            if (flight && ((latitude != 0.0) || (longitude != 0.0)))
            {
                flight->m_track.append(QGeoCoordinate(latitude, longitude, sqlite3_column_double(stmt, 4)));
                flight->m_trackTimes.append(columnDateTime(stmt, 5));
            }
        }
    }
    sqlite3_finalize(stmt);
    stmt = nullptr;

    // Documents
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM documents ORDER BY received", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            TrackedFlight *flight = flightsById.value(sqlite3_column_int64(stmt, 0));
            if (!flight) {
                continue;
            }
            TrackedDocument *doc = new TrackedDocument();
            doc->m_kind = sqlite3_column_int(stmt, 1);
            doc->m_title = columnText(stmt, 2);
            doc->m_text = columnText(stmt, 3);
            doc->m_received = columnDateTime(stmt, 4);
            flight->m_documents.append(doc);
        }
    }
    sqlite3_finalize(stmt);
    stmt = nullptr;

    // Flight profiles
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM profile ORDER BY flight_id, time", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            TrackedFlight *flight = flightsById.value(sqlite3_column_int64(stmt, 0));
            if (flight)
            {
                flight->m_profileTimes.append(sqlite3_column_int64(stmt, 1));
                flight->m_profileAltFt.append(sqlite3_column_type(stmt, 2) == SQLITE_NULL
                    ? std::numeric_limits<float>::quiet_NaN() : (float) sqlite3_column_double(stmt, 2));
                flight->m_profileSpeedKts.append(sqlite3_column_type(stmt, 3) == SQLITE_NULL
                    ? std::numeric_limits<float>::quiet_NaN() : (float) sqlite3_column_double(stmt, 3));
            }
        }
    }
    sqlite3_finalize(stmt);
    stmt = nullptr;

    // ATC log
    if (sqlite3_prepare_v2(m_db, "SELECT * FROM atc ORDER BY time", -1, &stmt, nullptr) == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            AtcEvent e;
            e.m_received = columnDateTime(stmt, 0);
            e.m_protocol = columnText(stmt, 1);
            e.m_uplink = sqlite3_column_int(stmt, 2) != 0;
            e.m_from = columnText(stmt, 3);
            e.m_to = columnText(stmt, 4);
            e.m_message = columnText(stmt, 5);
            e.m_tooltip = columnText(stmt, 6);
            e.m_aircraftId = (quint64) sqlite3_column_int64(stmt, 7);
            e.m_flightId = (quint64) sqlite3_column_int64(stmt, 8);
            e.m_mapName = columnText(stmt, 9);
            m_atcLog.append(e);
        }
    }
    sqlite3_finalize(stmt);

    // Everything just read is, by definition, already in the database - without this the
    // first save would write the whole restored history out again.
    //
    // Before EVERY merge below, not after: absorbAircraft() runs one of its own
    // through mergeLoadedFlights(), so putting this between the two loops still
    // erased what the first had recorded. merging two restored flights deletes one and
    // rewrites the other, and says so through m_flightsToDelete, m_flightsToRewrite and a
    // zeroed m_trackSaved. Doing this afterwards overwrote all three, so the next save
    // left the merged-away flight's rows orphaned in the database and never wrote the
    // combined track - and the merged history was gone by the restart after that.
    for (TrackedAircraft *item : m_aircraft)
    {
        for (TrackedFlight *flight : item->m_flights)
        {
            flight->m_trackSaved = flight->m_track.size();
            flight->m_profileSaved = flight->m_profileTimes.size();
        }
    }
    m_flightsToDelete.clear();
    m_flightsToRewrite.clear();
    m_dbTrackWipe = false;

    // Fold weak flight-only aircraft into the identified airframe flying the
    // same flight, and merge flights that are the same flight under its ACARS
    // and ADS-B names (sessions saved before these existed live)
    QList<TrackedAircraft *> weakItems;
    for (TrackedAircraft *a : m_aircraft)
    {
        if ((a->m_icao == 0) && a->m_registration.isEmpty()) {
            weakItems.append(a);
        }
    }
    for (TrackedAircraft *weak : weakItems)
    {
        TrackedAircraft *strong = nullptr;
        for (TrackedAircraft *cand : m_aircraft)
        {
            if ((cand == weak) || ((cand->m_icao == 0) && cand->m_registration.isEmpty())) {
                continue;
            }
            for (TrackedFlight *weakFlight : weak->m_flights)
            {
                if (weakFlight->m_flight.isEmpty()) {
                    continue;
                }
                for (TrackedFlight *candFlight : cand->m_flights)
                {
                    // Exact name only: number-based matching is only safe
                    // within a single airframe
                    if ((candFlight->m_flight != weakFlight->m_flight)
                        && !candFlight->m_aliases.contains(weakFlight->m_flight)) {
                        continue;
                    }
                    // ... and only when the two were flying it at the same time. A
                    // callsign is flown again day after day, by a different airframe as
                    // often as by the same one, so a flight-only record from last week
                    // must not be folded into whichever aircraft flew that number most
                    // recently - which is what an unbounded name match did.
                    if (!weakFlight->m_lastSeen.isValid() || !candFlight->m_lastSeen.isValid()
                        || (std::abs(weakFlight->m_lastSeen.secsTo(candFlight->m_lastSeen))
                            >= m_flightKeyValidSecs)) {
                        continue;
                    }
                    strong = cand;
                    break;
                }
                if (strong) {
                    break;
                }
            }
            if (strong) {
                break;
            }
        }
        if (strong) {
            absorbAircraft(strong, weak);
        }
    }
    for (TrackedAircraft *item : m_aircraft) {
        mergeLoadedFlights(item);
    }

    // Prune aircraft not heard from within the removal timeout, then send the
    // survivors to the Map and register their current flight names
    removeOldAircraft();

    m_loadingDatabase = false;
    for (TrackedAircraft *item : m_aircraft)
    {
        if (item->m_currentFlight)
        {
            if (!item->m_currentFlight->m_flight.isEmpty())
            {
                item->m_flight = item->m_currentFlight->m_flight;
                m_byKey.insert("F" + item->m_flight, item);
            }
            for (const QString& alias : item->m_currentFlight->m_aliases) {
                m_byKey.insert("F" + alias, item);
            }
        }
        // Restored aircraft start inactive: they appear in the archive tables and are
        // not drawn until data arrives for them again
        item->m_active = false;
        m_dirtyAircraft.insert(item);
        for (TrackedFlight *flight : item->m_flights) {
            m_dirtyFlights.insert(flight);
        }
    }

    // Anything restored that is already past its retention goes now
    discardOldAircraft();
}


// The key an airframe is counted under for all time. The ICAO address where there is
// one, because it is the only identifier that is genuinely the airframe; otherwise the
// registration, which is nearly as good, prefixed to keep the two namespaces apart.
//
// A flight number is deliberately NOT accepted. It identifies a service rather than an
// airframe, so counting one would both inflate the total and count the same aircraft
// again the day it is heard with an address - and an aircraft with neither an address
// nor a registration is rare, since ADS-B always carries the first and an ACARS block
// almost always carries the second.
static QString seenKey(quint32 icao, const QString& registration)
{
    if (icao) {
        return QString("%1").arg(icao, 6, 16, QChar('0')).toUpper();
    }
    if (!registration.isEmpty()) {
        return "R:" + registration;
    }
    return QString();
}

int AircraftTracker::recordSeen(TrackedAircraft *item, int& countDelta)
{
    countDelta = 0;
    const QString key = seenKey(item->m_icao, item->m_registration);
    if (key.isEmpty()) {
        return -1;              // Nothing to recognise it by yet: ask again next report
    }
    if (!m_db) {
        return -1;              // Not written: ask again rather than counting it as done
    }

    // A registration-only airframe is recorded under its registration and moves to its
    // ICAO address the day one arrives. The old row has to move with it: left behind, a
    // later session hears the same aircraft, inserts the address as a key it has never
    // seen, and counts one airframe as two for ever.
    //
    // Written as "is there a stale registration row" rather than "did the key just
    // change", because the change may have happened in an earlier session - the aircraft
    // is restored already holding both identities, with nothing to say which of them it
    // was last recorded under.
    //
    // The two rows are folded into one that spans both: the earlier first_seen and the
    // later last_seen. That has to be done explicitly - an UPDATE that simply rekeys the
    // row cannot merge, and one that rekeys onto an address that already has a row
    // cannot even run.
    if (item->m_icao && !item->m_registration.isEmpty())
    {
        const QString stale = "R:" + item->m_registration;
        qint64 staleFirst = 0;
        qint64 staleLast = 0;
        bool staleExists = false;

        sqlite3_stmt *moveStmt = nullptr;
        if (sqlite3_prepare_v2(m_db, "SELECT first_seen, last_seen FROM seen WHERE key = ?",
                -1, &moveStmt, nullptr) == SQLITE_OK)
        {
            bindText(moveStmt, 1, stale);
            if (sqlite3_step(moveStmt) == SQLITE_ROW)
            {
                staleExists = true;
                staleFirst = sqlite3_column_int64(moveStmt, 0);
                staleLast = sqlite3_column_int64(moveStmt, 1);
            }
        }
        sqlite3_finalize(moveStmt);

        if (staleExists)
        {
            // Whether the address already had a row of its own decides whether this is a
            // rename or a merge - and a merge is one airframe fewer than the table held
            bool addressExists = false;
            moveStmt = nullptr;
            if (sqlite3_prepare_v2(m_db, "SELECT 1 FROM seen WHERE key = ?",
                    -1, &moveStmt, nullptr) == SQLITE_OK)
            {
                bindText(moveStmt, 1, key);
                addressExists = sqlite3_step(moveStmt) == SQLITE_ROW;
            }
            sqlite3_finalize(moveStmt);

            bool folded = false;
            moveStmt = nullptr;
            if (sqlite3_prepare_v2(m_db, "DELETE FROM seen WHERE key = ?",
                    -1, &moveStmt, nullptr) == SQLITE_OK)
            {
                bindText(moveStmt, 1, stale);
                folded = sqlite3_step(moveStmt) == SQLITE_DONE;
            }
            sqlite3_finalize(moveStmt);
            if (!folded) {
                qWarning() << "AircraftTracker::recordSeen: could not fold" << stale
                           << "into" << key << ":" << sqlite3_errmsg(m_db);
            }

            if (folded)
            {
                // Carry the span across. INSERT OR IGNORE below cannot widen a row it
                // does not create, so where the address had one already this is the only
                // thing that keeps the airframe's real first_seen.
                moveStmt = nullptr;
                if (sqlite3_prepare_v2(m_db,
                        "INSERT INTO seen (key, first_seen, last_seen) VALUES (?,?,?)"
                        " ON CONFLICT(key) DO UPDATE SET"
                        " first_seen = MIN(first_seen, excluded.first_seen),"
                        " last_seen = MAX(last_seen, excluded.last_seen)",
                        -1, &moveStmt, nullptr) == SQLITE_OK)
                {
                    bindText(moveStmt, 1, key);
                    sqlite3_bind_int64(moveStmt, 2, staleFirst);
                    sqlite3_bind_int64(moveStmt, 3, staleLast);
                    if (sqlite3_step(moveStmt) != SQLITE_DONE) {
                        qWarning() << "AircraftTracker::recordSeen:" << sqlite3_errmsg(m_db);
                    }
                }
                sqlite3_finalize(moveStmt);

                // Two rows became one, so the all time count the caller holds is now one
                // too high. It is only read from the table at startup, so without this it
                // would stay wrong until the next run.
                if (addressExists) {
                    countDelta--;
                }
            }
        }
    }

    const qint64 now = QDateTime::currentSecsSinceEpoch();
    sqlite3_stmt *stmt = nullptr;

    // INSERT OR IGNORE, then ask how many rows changed: exactly 1 when this airframe
    // was new and 0 when it was already there, which is the answer the count needs. An
    // upsert would report 1 either way, and last_insert_rowid cannot be used to tell
    // them apart because the aircraft and flight inserts move it too
    if (sqlite3_prepare_v2(m_db,
            "INSERT OR IGNORE INTO seen (key, first_seen, last_seen) VALUES (?,?,?)",
            -1, &stmt, nullptr) != SQLITE_OK)
    {
        qWarning() << "AircraftTracker::recordSeen:" << sqlite3_errmsg(m_db);
        return -1;              // A failure is not "already counted": try again
    }
    bindText(stmt, 1, key);
    sqlite3_bind_int64(stmt, 2, now);
    sqlite3_bind_int64(stmt, 3, now);

    bool inserted = false;
    bool stepped = false;
    if (sqlite3_step(stmt) == SQLITE_DONE)
    {
        stepped = true;
        inserted = sqlite3_changes(m_db) > 0;
    }
    else
    {
        qWarning() << "AircraftTracker::recordSeen:" << sqlite3_errmsg(m_db);
    }
    sqlite3_finalize(stmt);
    if (!stepped) {
        return -1;              // Nothing was written, so nothing has been counted
    }

    if (!inserted)
    {
        // Heard before, so only the last seen time moves
        stmt = nullptr;
        if (sqlite3_prepare_v2(m_db, "UPDATE seen SET last_seen = ? WHERE key = ?",
                -1, &stmt, nullptr) == SQLITE_OK)
        {
            sqlite3_bind_int64(stmt, 1, now);
            bindText(stmt, 2, key);
            sqlite3_step(stmt);
        }
        sqlite3_finalize(stmt);
    }
    if (inserted) {
        countDelta++;
    }
    return 1;
}

// How many different airframes have ever been heard
int AircraftTracker::seenCount()
{
    if (!m_db) {
        return 0;
    }
    sqlite3_stmt *stmt = nullptr;
    int count = 0;
    if (sqlite3_prepare_v2(m_db, "SELECT COUNT(*) FROM seen", -1, &stmt, nullptr) == SQLITE_OK)
    {
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            count = sqlite3_column_int(stmt, 0);
        }
    }
    sqlite3_finalize(stmt);
    return count;
}

// Only from an explicit "reset all time statistics", never from clearing the session
void AircraftTracker::resetSeen()
{
    if (m_db) {
        execSQL(m_db, "DELETE FROM seen");
    }
}
