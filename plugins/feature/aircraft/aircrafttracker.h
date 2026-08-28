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

#ifndef INCLUDE_FEATURE_AIRCRAFTTRACKER_H_
#define INCLUDE_FEATURE_AIRCRAFTTRACKER_H_

#include <QObject>
#include <QTimer>
#include <QDateTime>
#include <QHash>
#include <QSet>
#include <QGeoCoordinate>
#include <QMetaType>

#include "util/message.h"
#include "util/messagequeue.h"
#include "util/aircraftreport.h"
#include "util/osndb.h"
// For the tables a filed route is resolved against - see resolveRoute()
#include "util/openaip.h"
#include "util/ourairportsdb.h"
#include "util/waypoints.h"

#include "aircraftsettings.h"

class Feature;
struct sqlite3;

namespace SWGSDRangel {
    class SWGMapItem;
    class SWGMapCoordinate;
}

// Snapshots of the tracker's state, sent to the GUI over queued signals so the
// GUI never touches the tracker thread's data
struct AircraftDisplay {
    quint64 m_id = 0;
    quint32 m_icao = 0;
    QString m_registration;
    QString m_flight;
    bool m_positionValid = false;
    // Distance from the antenna that heard the aircraft, in km - see TrackedAircraft
    bool m_distanceValid = false;
    float m_distanceKm = 0.0f;
    float m_latitude = 0.0f;
    float m_longitude = 0.0f;
    bool m_altitudeValid = false;
    float m_altitudeFt = 0.0f;
    bool m_headingValid = false;
    float m_heading = 0.0f;
    bool m_speedValid = false;
    float m_speedKts = 0.0f;
    QString m_protocols;
    int m_messages = 0;
    QDateTime m_lastSeen;
    QString m_mapName;
    quint64 m_currentFlightId = 0;
    bool m_active = false;      // Data is arriving for this aircraft
};

struct FlightDisplay {
    quint64 m_id = 0;
    quint64 m_aircraftId = 0;
    QString m_flight;
    QString m_aliases;
    QString m_reg;
    QString m_departure;
    QString m_arrival;
    QString m_route;
    QDateTime m_firstSeen;
    QDateTime m_lastSeen;
    int m_documents = 0;
    int m_messages = 0;
    QString m_protocols;        // Protocols and frequencies THIS flight was heard on
    QDateTime m_out;
    QDateTime m_off;
    QDateTime m_on;
    QDateTime m_in;
    // Whether the flight has produced each of the documents worth knowing about at a
    // glance, so the table can say so without opening the flight
    bool m_hasLoadsheet = false;
    bool m_hasClearance = false;
    bool m_hasFlightPlan = false;
    bool m_active = false;      // The flight an active aircraft is currently flying
};

struct DocumentEvent {
    quint64 m_aircraftId = 0;
    quint64 m_flightId = 0;
    QDateTime m_received;
    QString m_flight;
    QString m_reg;
    QString m_kind;
    QString m_title;
    QString m_text;
    QString m_mapName;
};

struct AtcEvent {
    quint64 m_aircraftId = 0;
    quint64 m_flightId = 0;
    QDateTime m_received;
    QString m_protocol;
    bool m_uplink = false;
    QString m_from;
    QString m_to;
    QString m_message;
    QString m_tooltip;
    QString m_mapName;
};

Q_DECLARE_METATYPE(AircraftDisplay)
Q_DECLARE_METATYPE(FlightDisplay)
Q_DECLARE_METATYPE(DocumentEvent)
// A weather report is about an AIRPORT, not about the aircraft that happened to ask
// for it - so it outlives that aircraft and is kept in its own log
struct WeatherEvent {
    QDateTime m_received;
    QString m_airport;          // ICAO code, empty if the report did not name one
    QString m_kind;             // METAR, TAF, ATIS, TWIP, NOTAM, PIREP, SIGMET
    QString m_summary;          // First line, for the table
    QString m_text;             // The report as sent
    QString m_from;             // Aircraft that carried it, for provenance
};

Q_DECLARE_METATYPE(AtcEvent)
Q_DECLARE_METATYPE(WeatherEvent)

// Collates aircraft reports from the demodulators into aircraft, flights and
// documents, drives the Map and persists the session, all off the GUI thread.
// Runs on its own thread inside the Aircraft feature, in both GUI and server
// builds; the GUI, when there is one, is a view driven by this class's signals.
// Session and lifetime statistics shown on the Statistics tab.
//
// Every figure is kept twice: once for this session and once for all time. The two
// answer different questions - "is the aerial working as well as it did yesterday"
// wants the session, "how far have we ever heard" wants all time - and neither is
// recoverable from the other, so both are counted as reports arrive.
//
// The all time side persists. Most of it travels in the settings blob, as the records
// always have. The one exception is the aircraft count, which cannot be a running
// total: the same airframe heard on two days must count once, so it needs the set of
// who has been heard, not a number. That set is the database's "seen" table, which is
// also the only part of this that survives the retention period pruning the rest.
struct AircraftStatistics {
    // A record holder: the value, who set it, when, and where they were at the time.
    // The position is what lets the max range record be drawn on the Map at the point
    // it was actually achieved rather than wherever the aircraft is now - which for an
    // all time record is very probably nowhere, the aircraft having long since landed.
    struct Record {
        bool m_valid = false;
        float m_value = 0.0f;
        QString m_aircraft;         // Registration, else flight, else ICAO address
        QDateTime m_when;
        bool m_positionValid = false;
        float m_latitude = 0.0f;
        float m_longitude = 0.0f;
        bool m_altitudeValid = false;
        float m_altitudeFt = 0.0f;

        // Returns true if this beat the record
        bool update(float value, const QString& aircraft, const QDateTime& when,
                    bool positionValid, float latitude, float longitude,
                    bool altitudeValid, float altitudeFt)
        {
            if (m_valid && (value <= m_value)) {
                return false;
            }
            m_valid = true;
            m_value = value;
            m_aircraft = aircraft;
            m_when = when;
            m_positionValid = positionValid;
            m_latitude = latitude;
            m_longitude = longitude;
            m_altitudeValid = altitudeValid;
            m_altitudeFt = altitudeFt;
            return true;
        }
    };

    // One complete set of figures, over whatever period the holder covers
    struct Scope {
        // Furthest heard on each protocol, indexed by AircraftReport::Protocol. Per
        // protocol rather than one overall figure because the whole point of comparing
        // them is that they are so different - line of sight VHF against HF skywave
        // against a satellite.
        Record m_maxRange[AircraftReport::ProtocolCount];
        Record m_fastest;           // Ground speed, knots
        Record m_highest;           // Altitude, feet

        quint64 m_totalMessages = 0;    // Reports of any kind
        quint64 m_messagesByProtocol[AircraftReport::ProtocolCount] = {};
        int m_distinctAircraft = 0;     // Different aircraft heard
        int m_maxConcurrent = 0;        // Most aircraft heard within any 15 minute window
        QDateTime m_maxConcurrentWhen;
        qint64 m_seconds = 0;           // Time spent listening, filled in as it is emitted

        // Applies a report to every record this scope keeps
        void update(int protocol, bool rangeValid, float rangeKm, bool speedValid,
                    float speedKts, bool altitudeValid, float altitudeFt,
                    const QString& who, const QDateTime& when,
                    bool positionValid, float latitude, float longitude)
        {
            m_totalMessages++;
            if ((protocol >= 0) && (protocol < AircraftReport::ProtocolCount))
            {
                m_messagesByProtocol[protocol]++;
                if (rangeValid) {
                    m_maxRange[protocol].update(rangeKm, who, when, positionValid,
                                                latitude, longitude, altitudeValid, altitudeFt);
                }
            }
            if (speedValid) {
                m_fastest.update(speedKts, who, when, positionValid, latitude, longitude,
                                 altitudeValid, altitudeFt);
            }
            if (altitudeValid) {
                m_highest.update(altitudeFt, who, when, positionValid, latitude, longitude,
                                 altitudeValid, altitudeFt);
            }
        }
    };

    Scope m_session;
    Scope m_allTime;
    QDateTime m_sessionStart;
    QDateTime m_firstStart;         // When the all time figures began

    static const int ConcurrentWindowMins = 15;
};

Q_DECLARE_METATYPE(AircraftStatistics)

class AircraftTracker : public QObject
{
    Q_OBJECT

public:
    class MsgConfigureTracker : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        const AircraftSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }
        static MsgConfigureTracker* create(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force) {
            return new MsgConfigureTracker(settings, settingsKeys, force);
        }
    private:
        AircraftSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;
        MsgConfigureTracker(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force) :
            Message(), m_settings(settings), m_settingsKeys(settingsKeys), m_force(force)
        {}
    };

    class MsgReport : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        AircraftReport& getReport() { return m_report; }
        static MsgReport* create(const AircraftReport& report) { return new MsgReport(report); }
        ~MsgReport();   // Owns m_report.m_mapItem until processing transfers it
    private:
        AircraftReport m_report;
        MsgReport(const AircraftReport& report) : Message(), m_report(report) {}
    };

    class MsgDeleteAll : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        static MsgDeleteAll* create() { return new MsgDeleteAll(); }
    private:
        MsgDeleteAll() : Message() {}
    };

    // Zero one half of the statistics. The all time half also clears the record of
    // which airframes have been heard, which is the only way its aircraft count can go
    // back to nothing
    class MsgResetStatistics : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        bool getAllTime() const { return m_allTime; }
        static MsgResetStatistics* create(bool allTime) { return new MsgResetStatistics(allTime); }
    private:
        bool m_allTime;
        MsgResetStatistics(bool allTime) : Message(), m_allTime(allTime) {}
    };

    // GUI (re)connected: resend the complete state
    class MsgResync : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        static MsgResync* create() { return new MsgResync(); }
    private:
        MsgResync() : Message() {}
    };

    // Which flight's profile the GUI is charting; 0 for none
    class MsgWatchFlight : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        quint64 getFlightId() const { return m_flightId; }
        static MsgWatchFlight* create(quint64 flightId) { return new MsgWatchFlight(flightId); }
    private:
        quint64 m_flightId;
        MsgWatchFlight(quint64 flightId) : Message(), m_flightId(flightId) {}
    };

    explicit AircraftTracker(Feature *feature);
    ~AircraftTracker();

    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }

public:
    // Used by the GUI, which owns persistence: it holds the blob ready for serialize()
    static QByteArray serializeStatistics(const AircraftStatistics& statistics);
    static AircraftStatistics deserializeStatistics(const QByteArray& data);

public slots:
    void startWork();

signals:
    void aircraftUpdated(const QList<AircraftDisplay>& aircraft);
    void aircraftRemoved(const QList<quint64>& ids);
    void flightsUpdated(const QList<FlightDisplay>& flights);
    void flightsRemoved(const QList<quint64>& ids);
    void documentsAdded(const QList<DocumentEvent>& documents);
    void atcMessages(const QList<AtcEvent>& messages);
    void weatherReports(const QList<WeatherEvent>& reports);
    void profileUpdated(quint64 flightId, const QList<qint64>& times, const QList<float>& altitudeFt, const QList<float>& speedKts);
    void messageRates(const QList<float>& ratesPerProtocolThenTotal);
    void statisticsUpdated(const AircraftStatistics& statistics);
    void speechNotification(const QString& speech);
    void allCleared();
    //!< The database could not be changed, so the setting has been put back to the file
    //!< actually in use. The GUI holds the settings that get serialised, so it has to
    //!< hear about it or it would keep - and save - a path nothing is writing to.
    void databaseFilenameReverted(const QString& filename);

private:
    struct TrackedAircraft;

    struct TrackedDocument {
        int m_kind;
        QString m_title;
        QString m_text;
        QDateTime m_received;
    };

    struct TrackedFlight {
        quint64 m_id = 0;
        QString m_flight;
        QStringList m_aliases;
        TrackedAircraft *m_aircraft = nullptr;
        QDateTime m_firstSeen;
        QDateTime m_lastSeen;
        QString m_departure;
        QString m_arrival;
        QString m_route;
        //!< When the message that revealed m_departure, m_arrival and m_route was
        //!< received, so a delayed one cannot put back what a newer one has replaced.
        //!< Invalid where the facts came from the callsign to route database, which is a
        //!< standing guess rather than something the aircraft said.
        QDateTime m_routeFiled;
        // A route the aircraft has been given, drawn on the Map. Kept apart from
        // m_route because the callsign to route database also fills that in, with the
        // airport pair rather than a list of waypoints, and only a real waypoint list
        // can be drawn.
        struct FiledRoute {
            QString m_waypoints;    //!< The route as filed, names separated by spaces
            //!< Where the message put those waypoints, when it said. Same order and
            //!< length as m_waypoints, or empty when it named them without placing them.
            QList<AircraftReport::RouteWaypoint> m_points;
            QString m_text;         //!< The message that filed it, for the popup
            QDateTime m_filed;      //!< When, so a merge can tell two routes apart
            QString m_drawn;        //!< What was last sent, so it is only sent once
            //!< Held back for want of a Map to send it to or a name to key it on -
            //!< neither of which the flight is told about when it changes
            bool m_pending = false;
            //!< The Map item names last emitted, so that anything which will not be
            //!< drawn again can be taken off. The Map keys on the name, so a route that
            //!< has been renamed would otherwise be drawn twice over.
            QString m_drawnLine;
            QStringList m_drawnWaypoints;
        };
        FiledRoute m_flightPlan;    //!< An FPN plan, or a position report's route insert
        FiledRoute m_clearance;     //!< An oceanic clearance or its readback
        //!< Protocols and frequencies THIS flight has been heard on, the same tally the
        //!< airframe keeps but bounded to one flight. The Map shows this one: an
        //!< airframe heard on HFDL over the Atlantic last month is not being heard on
        //!< HFDL now, so naming it beside the aircraft on the Map is simply wrong.
        QHash<QPair<int, qint64>, int> m_sources;
        QList<TrackedDocument *> m_documents;
        QList<qint64> m_profileTimes;
        QList<float> m_profileAltFt;
        QList<float> m_profileSpeedKts;
        // The ground track belongs to the FLIGHT, not to the airframe. An aircraft is
        // heard again days later on a different route, and drawing every track it has
        // ever flown puts unrelated lines across the map
        // How many of the points below are already in the database. The track and the
        // profile only ever grow, so a save writes the tail rather than rewriting the
        // whole history - see saveDatabase(). Reset to zero, and the flight added to
        // m_flightsToRewrite, whenever the arrays are rebuilt rather than appended to.
        int m_trackSaved = 0;
        int m_profileSaved = 0;
        QList<QGeoCoordinate> m_track;
        QList<QDateTime> m_trackTimes;
        int m_messages = 0;             // Reports of any kind attributed to this flight
        // The OOOI times, as the flight reported them. Out of gate, take off, landing,
        // on gate - the four instants that describe a flight from the ground's point of
        // view, and the ones an airline's own systems run on.
        QDateTime m_out;
        QDateTime m_off;
        QDateTime m_on;
        QDateTime m_in;
        // The ground track belongs to the FLIGHT, not to the airframe. An aircraft is
        // heard again days later on a different route, and drawing every track it has
        // ever flown puts unrelated lines across the map

        ~TrackedFlight() { qDeleteAll(m_documents); }
    };

    // How much a position can be trusted, best first. A VDL-2 XID link management frame
    // carries an aircraft location parameter with 0.1 degree resolution - about 11 km -
    // which is enough for a ground station to decide a handoff, but drags an ADS-B track
    // sideways every time one arrives
    enum PositionRank {
        PositionAdsb = 0,       // ADS-B: metres, about once a second
        PositionReported = 1,   // A real position report, whatever protocol carried it
        PositionCoarse = 2,     // A location parameter from a link management frame
        PositionRanks = 3
    };

    static int positionRank(const AircraftReport& report);

    // Altitude is ranked separately from position, and for a different reason. An XID
    // location parameter carries an altitude as a single octet of thousands of feet, and
    // a minority of aircraft - overwhelmingly Boeing 737NGs - fill it with nonsense:
    // 59000, 65000, 90000, 123000 ft have all been observed off air, from aircraft whose
    // ADS-B put them at a normal cruise at the same moment. The value is not merely
    // coarse, it is sometimes wrong, so it must not set a record and must not displace a
    // real altitude. See the note above AcarsVdl2Atn's altitude handling for the
    // measurements behind that.
    enum AltitudeRank {
        AltitudeAdsb = 0,       // ADS-B: 25 ft, about once a second
        AltitudeReported = 1,   // A real position or performance report
        AltitudeCoarse = 2,     // A link management frame's location parameter
        AltitudeRanks = 3
    };

    static int altitudeRank(const AircraftReport& report);

    // A less accurate source is only used once the better ones have gone quiet for this long
    static const int m_positionRankTimeoutSecs = 60;
    // How long a flight number may still stand in for an identity. Long enough to cover
    // a flight heard intermittently, far short of the same number returning tomorrow.
    static const int m_flightKeyValidSecs = 12*60*60;
    // Positions that would need a faster aircraft than this to reach are stale or wrong
    static const int m_maxGroundSpeedKn = 700;
    static constexpr double m_positionToleranceKm = 5.0;

    struct TrackedAircraft {
        quint64 m_id = 0;
        quint32 m_icao = 0;
        QString m_registration;
        QString m_flight;
        bool m_positionValid = false;
        float m_latitude = 0.0f;
        float m_longitude = 0.0f;
        float m_altitudeFt = 0.0f;
        bool m_altitudeValid = false;
        // The two most recent altitude observations, so an altitude can be worked out
        // for the moment a position was measured rather than assuming they coincide
        QDateTime m_altitudeDateTime;
        float m_prevAltitudeFt = 0.0f;
        QDateTime m_prevAltitudeDateTime;
        float m_heading = 0.0f;
        bool m_headingValid = false;
        float m_speedKts = 0.0f;
        bool m_speedValid = false;
        //!< When each was observed. Neither is timestamped in its own right - a velocity
        //!< message says what the aircraft is doing now, not when - so this is the
        //!< report's reception time, which is enough to stop a delayed report putting
        //!< back a heading or speed the aircraft has already left behind.
        QDateTime m_headingDateTime;
        QDateTime m_speedDateTime;
        // Great circle distance from the receiver to the aircraft, in km. Measured from
        // the position of the DEVICE the report arrived on where it has one - a remote
        // SDR reports where it is - and from My Position otherwise, so a distance always
        // means "from the antenna that heard it" rather than "from here".
        bool m_distanceValid = false;
        float m_distanceKm = 0.0f;
        QDateTime m_positionDateTime;
        QDateTime m_positionRankTimes[PositionRanks];   // When a position of each rank last arrived
        int m_positionRank = PositionRanks;             // Rank of the position we are holding
        QDateTime m_altitudeRankTimes[AltitudeRanks];   // When an altitude of each rank last arrived
        QDateTime m_lastSeen;
        QString m_departure;
        QString m_arrival;
        QString m_route;
        QString m_lastDocumentText;
        // Every protocol and frequency this airframe has EVER been heard on, kept for
        // as long as the airframe is. The flight has its own, which is what the Map
        // shows: an airframe heard on HFDL over the Atlantic last month is not being
        // heard on HFDL now, and saying so on the Map is simply wrong.
        QHash<QPair<int, qint64>, int> m_sources;
        int m_messages = 0;
        QString m_mapName;
        QDateTime m_lastMapItemTime;
        QString m_model3D;
        float m_modelAltitudeOffset = 0.0f;     // So the undercarriage is not underground
        float m_labelAltitudeOffset = 0.0f;
        // Whether we chose the model or the ADS-B pass-through brought one. Only ours may
        // be thrown away and chosen again when the livery preference changes - replacing
        // a pass-through model with our own would fight the demodulator for it.
        bool m_modelIsOurs = false;
        bool m_mapModelForm = false;
        bool m_mapTrackChanged = true;
        //!< Owed to a Map that has just appeared, and still owed until something has
        //!< actually been sent - see checkMapConsumers()
        bool m_mapResync = false;
        QList<TrackedFlight *> m_flights;
        TrackedFlight *m_currentFlight = nullptr;
        // Set when a report arrives, cleared when the aircraft goes quiet. Aircraft
        // restored from the session database start inactive, so a new session begins
        // with empty active tables
        bool m_active = false;
        QSet<int> m_notifiedRules;

        ~TrackedAircraft() { qDeleteAll(m_flights); }
    };

    Feature *m_feature;                 // Pipe source for the Map
    AircraftSettings m_settings;
    MessageQueue m_inputMessageQueue;

    QList<TrackedAircraft *> m_aircraft;
    QHash<QString, TrackedAircraft *> m_byKey;
    quint64 m_nextId = 1;

    QSharedPointer<const QHash<QString, AircraftInformation *>> m_aircraftInfo;
    QSharedPointer<const QHash<int, AircraftInformation *>> m_aircraftInfoByIcao;
    QSharedPointer<const QHash<QString, AircraftRouteInformation *>> m_routeInfo;
    // Used to turn the named waypoints of a filed route into coordinates. All three are
    // process wide caches shared with the Map and the ACARS demodulator, so holding them
    // here costs a pointer rather than another copy of the data.
    QSharedPointer<const QList<NavAid *>> m_navAids;
    QSharedPointer<const QHash<QString, AirportInformation *>> m_airports;
    QSharedPointer<const QMultiHash<QString, Waypoint *>> m_waypoints;

    QList<AtcEvent> m_atcLog;           // Self-contained, outlives its aircraft
    QList<WeatherEvent> m_weatherLog;   // Likewise, and keyed on airports not aircraft
    // Which reports are already in the log, so a repeat is recognised without comparing
    // it against every entry ever logged. Maps a hash of the report to its position; the
    // full comparison still happens on a hash match, so a collision costs one comparison
    // rather than silently dropping a report that was not really a duplicate. The log is
    // only ever appended to, so the positions never move.
    QMultiHash<size_t, int> m_weatherIndex;

    // Batched GUI notifications
    QSet<TrackedAircraft *> m_dirtyAircraft;
    QSet<TrackedFlight *> m_dirtyFlights;
    QList<DocumentEvent> m_pendingDocuments;
    QList<AtcEvent> m_pendingAtc;
    QList<WeatherEvent> m_pendingWeather;
    QTimer *m_flushTimer = nullptr;
    QTimer *m_removalTimer = nullptr;
    QTimer *m_chartTimer = nullptr;
    // One counter per protocol plus a total
    static const int CHART_COUNTS = AircraftReport::ProtocolCount + 1;
    int m_chartCounts[CHART_COUNTS];
    QDateTime m_chartRateTime;

    quint64 m_watchedFlight = 0;        // Flight whose profile the GUI charts
    bool m_watchedFlightChanged = false;

    sqlite3 *m_db = nullptr;
    // Flights whose stored track and profile no longer match what is in memory. Deleted
    // flights have to be named, because the save no longer empties those tables and so
    // would otherwise leave their rows behind for ever.
    QSet<quint64> m_flightsToDelete;
    QSet<quint64> m_flightsToRewrite;
    bool m_dbTrackWipe = false;     //!< Everything in track and profile is stale
    bool m_dbDirty = false;
    bool m_loadingDatabase = false;

    bool handleMessage(const Message& message);
    void processReport(AircraftReport& report);

    TrackedAircraft *findOrCreateAircraft(const AircraftReport& report);
    void absorbAircraft(TrackedAircraft *item, TrackedAircraft *weak);
    TrackedFlight *updateFlight(TrackedAircraft *item, const AircraftReport& report);
    void mergeFlightInto(TrackedAircraft *item, TrackedFlight *dst, TrackedFlight *src);
    void mergeLoadedFlights(TrackedAircraft *item);
    static bool splitFlightName(const QString& flight, QString& airline, QString& number);
    static bool sameFlightNumber(const QString& a, const QString& b);
    static bool flightMatches(const TrackedFlight *flight, const QString& name);
    //!< When this aircraft was last heard flying this callsign, which is not when it was
    //!< last heard: it is very likely flying something else by now
    static QDateTime flightLastSeen(const TrackedAircraft *item, const QString& flight);
    //!< Whether two of one airframe's flights are one operation rather than the same
    //!< callsign flown again on another day
    bool sameOperation(const TrackedFlight *a, const TrackedFlight *b) const;
    void addFlightAlias(TrackedAircraft *item, TrackedFlight *flight, const QString& name);
    void addDocument(TrackedAircraft *item, TrackedFlight *flight, const AircraftReport& report);
    void addAtcMessage(TrackedAircraft *item, TrackedFlight *flight, const AircraftReport& report);
    void addWeatherReport(TrackedAircraft *item, const AircraftReport& report);
    static QString weatherKindName(int kind);
    QString documentKindName(int kind) const;
    QString protocolName(int protocol) const;
    static QString notificationSignature(const AircraftSettings& settings);
    void claimKey(TrackedAircraft *item, const QString& key);
    void updateRecordMapItem();
    static const char *RecordMapItemName;
    void resolveRoute(QList<AircraftReport::RouteWaypoint>& points,
                      QList<QGeoCoordinate>& coords) const;
    void sendRouteToMap(TrackedFlight *flight, TrackedFlight::FiledRoute& route,
                        const QString& kind);
    void removeDrawnRoute(TrackedFlight::FiledRoute& route);

    // What updateRecordMapItem() last put on the Map, so an unchanged record is not
    // re-sent 3 times a second, and so it can be taken off again when the option is
    // turned off or the record is reset
    QString m_recordMapKey;
    bool m_recordOnMap = false;
    bool m_recordUnplaceableWarned = false;

    // Flights with a route that could not be delivered. Held by id rather than by
    // pointer, so a flight deleted or merged away in the meantime simply fails to
    // resolve rather than leaving this dangling.
    QSet<quint64> m_routesPending;
    void retryRoutes();

    // WHICH Map features were listening last time anything was sent, by pipe id rather
    // than by count: closing one Map and opening another between flushes leaves the count
    // where it was, and the replacement would never be caught up. Delivery is remembered
    // per item rather than per Map - what has been drawn, what track was last sent - so a
    // Map opened after the fact is given later positions but none of what went out before
    // it existed. Saying it all again to everyone is the cheap answer; per consumer state
    // would be the thorough one.
    QSet<unsigned int> m_mapConsumers;
    // Aircraft owed to a Map that has just opened but not sendable yet: a basic item
    // would replace a richer pass-through one for a minute after the last of those
    // arrived. Kept here rather than in m_dirtyAircraft, which flush() empties every
    // pass and which drives a GUI update for everything in it.
    QSet<TrackedAircraft *> m_mapResyncPending;
    void checkMapConsumers();

    // Statistics
    void updateStatistics(const AircraftReport& report, TrackedAircraft *item,
                          bool rangeValid, float rangeKm,
                          bool altitudeValid, float altitudeFt);

    void updateConcurrent(const QDateTime& now);
    QGeoCoordinate receiverPosition(int deviceSetIndex);
    void update3DModel(TrackedAircraft *item);

    AircraftStatistics m_statistics;
    bool m_statisticsDirty = false;
    bool m_statisticsLoaded = false;
    // Seconds carried in from previous sessions, so the all time clock can be reported
    // as a live total rather than only advancing when the app is restarted
    qint64 m_allTimeSecondsBase = 0;
    // What m_allTimeSecondsBase is measured from. Normally the start of the session, but
    // an all time reset moves it to the moment of the reset - otherwise the all time
    // clock would restart at however long this session had already been running
    QDateTime m_allTimeSince;
    // Aircraft heard this session. The tracked list is restored from the database at
    // startup, so "a TrackedAircraft was allocated" counts only aircraft that fell out
    // of the retention period - which under-reported the session badly
    QSet<quint64> m_sessionHeard;
    // Those of them already written to the all time table. Separate from the above
    // because an aircraft can be heard before it is identified - an ACARS report can
    // arrive with only a flight number and gain its registration later - and it must be
    // counted under the identity it will keep, not the first one it happened to show
    QSet<quint64> m_sessionRecorded;
    // Drives a statistics update once a minute so both elapsed clocks tick even when
    // nothing is being received
    qint64 m_statisticsMinute = -1;
    // Receiver positions, cached by device set index: getDevicePosition() builds a whole
    // device report and parses it as JSON, which is far too heavy to do per message
    QHash<int, QGeoCoordinate> m_devicePositions;
    QDateTime m_devicePositionsRefreshed;
    // Taking the tally rather than its owner, because both the airframe and the
    // flight it is on keep one
    QStringList sourcesList(const QHash<QPair<int, qint64>, int>& sources) const;
    QString sourcesText(const QHash<QPair<int, qint64>, int>& sources) const;
    QString mapItemName(const TrackedAircraft *item) const;
    QString flightLabel(const QString& flight) const;
    QString atcLabel(const TrackedAircraft *item) const;
    QString aircraftImage(const TrackedAircraft *item) const;
    //!< The same choice for an aircraft that is no longer tracked - all the record
    //!< marker has is the name and, where the airframe is still known, its 3D model
    QString aircraftImage(const QString& registration, const QString& model3D) const;
    QList<SWGSDRangel::SWGMapCoordinate *> *buildTrack(const TrackedFlight *flight) const;
    QDateTime mapAvailableUntil(const TrackedAircraft *item) const;
    int positionTimeoutSecs(const TrackedAircraft *item) const;
    bool positionStale(const TrackedAircraft *item, const QDateTime& now) const;
    static float altitudeAt(const TrackedAircraft *item, const QDateTime& when, float fallbackFt);
    // Which of an aircraft's flights the map should draw the track of - the one it is
    // flying now, unless the GUI is looking at one of its past flights
    const TrackedFlight *trackFlight(const TrackedAircraft *item) const;
    void sendToMap(TrackedAircraft *item);
    void forwardMapItem(TrackedAircraft *item, SWGSDRangel::SWGMapItem *swgMapItem);

    void removeFromMap(const QString& name);
    void checkNotifications(TrackedAircraft *item, const AircraftReport& report);
    QString subAircraftString(TrackedAircraft *item, const AircraftReport& report, const QString& string) const;
    void commandNotification(const QString& command);
    void deleteAll();
    void resync();
    void applySettings(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force);
    AircraftDisplay displayFor(const TrackedAircraft *item) const;
    FlightDisplay displayFor(const TrackedFlight *flight) const;
    DocumentEvent documentEvent(const TrackedAircraft *item, const TrackedFlight *flight, const TrackedDocument *doc) const;
    void emitProfile(const TrackedFlight *flight);
    TrackedFlight *flightById(quint64 id) const;
    QString regFor(const TrackedAircraft *item) const;

    // Implemented in aircraftdb.cpp
    bool openDatabase();
    // Remembers that this airframe has been heard at all, for the all time aircraft
    // count. Returns 1 if it had not been heard before, 0 if it had, and -1 if it cannot
    // be identified yet - in which case the caller should try again on the next report
    //!< Writes this airframe into the all time "seen" table, folding away a row left
    //!< under an identity it has since outgrown. Returns 1 when the table was written,
    //!< -1 when it could not be and the caller should ask again, and sets countDelta to
    //!< the change in the number of distinct airframes the table holds: +1 for one not
    //!< seen before, -1 where two rows turned out to be one airframe, 0 otherwise.
    int recordSeen(TrackedAircraft *item, int& countDelta);
    int seenCount();
    void resetSeen();
    void closeDatabase();
    //!< Returns false when nothing was written - the transaction rolled back, or there
    //!< is no database. The session is then still only in memory, which is what makes it
    //!< unsafe to switch databases: see changeDatabase().
    bool saveDatabase();
    void revertDatabaseFilename(const QString& path);
    // The stored track and profile no longer relate to what is in memory at all
    void forgetSavedTracks();
    void loadDatabase();

private slots:
    void handleInputMessages();
    void flush();
    void removeOldAircraft();
    void setActive(TrackedAircraft *item, bool active);
    void discardOldAircraft();
    void changeDatabase(const QString& oldPath);
    static QString databasePath(const AircraftSettings& settings);
    void updateChartCounts();
};

#endif // INCLUDE_FEATURE_AIRCRAFTTRACKER_H_
