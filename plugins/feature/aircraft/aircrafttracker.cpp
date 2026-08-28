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

#include <algorithm>
#include <limits>
#include <cmath>

#include <QDebug>
#include <QDataStream>
#include <QFile>
#include <QFileInfo>
#include <QProcess>

#include "SWGMapItem.h"
#include "SWGMapCoordinate.h"

#include "feature/feature.h"
#include "maincore.h"
#include "util/units.h"

#include "util/airlines.h"
#include "channel/channelwebapiutils.h"
#include "util/aircraft3dmodels.h"

#include "aircrafttracker.h"

MESSAGE_CLASS_DEFINITION(AircraftTracker::MsgConfigureTracker, Message)
MESSAGE_CLASS_DEFINITION(AircraftTracker::MsgReport, Message)

AircraftTracker::MsgReport::~MsgReport()
{
    delete m_report.m_mapItem;
}
MESSAGE_CLASS_DEFINITION(AircraftTracker::MsgDeleteAll, Message)
MESSAGE_CLASS_DEFINITION(AircraftTracker::MsgResetStatistics, Message)
MESSAGE_CLASS_DEFINITION(AircraftTracker::MsgResync, Message)
MESSAGE_CLASS_DEFINITION(AircraftTracker::MsgWatchFlight, Message)

AircraftTracker::AircraftTracker(Feature *feature) :
    m_feature(feature)
{
    qRegisterMetaType<AircraftDisplay>("AircraftDisplay");
    qRegisterMetaType<FlightDisplay>("FlightDisplay");
    qRegisterMetaType<DocumentEvent>("DocumentEvent");
    qRegisterMetaType<AtcEvent>("AtcEvent");
    qRegisterMetaType<QList<AircraftDisplay>>("QList<AircraftDisplay>");
    qRegisterMetaType<QList<FlightDisplay>>("QList<FlightDisplay>");
    qRegisterMetaType<QList<DocumentEvent>>("QList<DocumentEvent>");
    qRegisterMetaType<QList<AtcEvent>>("QList<AtcEvent>");
    qRegisterMetaType<WeatherEvent>("WeatherEvent");
    qRegisterMetaType<QList<WeatherEvent>>("QList<WeatherEvent>");
    qRegisterMetaType<QList<quint64>>("QList<quint64>");
    qRegisterMetaType<QList<qint64>>("QList<qint64>");
    qRegisterMetaType<QList<float>>("QList<float>");

    for (int i = 0; i < CHART_COUNTS; i++) {
        m_chartCounts[i] = 0;
    }

    connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &AircraftTracker::handleInputMessages);
}

AircraftTracker::~AircraftTracker()
{
    saveDatabase();
    closeDatabase();
    qDeleteAll(m_aircraft);
}

// Runs on the tracker thread once it starts
void AircraftTracker::startWork()
{
    // As in the ADS-B tracker: get the shared 3D model tables built off the critical path
    Aircraft3DModels::instance();

    m_statistics.m_sessionStart = QDateTime::currentDateTime();
    m_statistics.m_session = AircraftStatistics::Scope();
    m_allTimeSince = m_statistics.m_sessionStart;
    m_sessionHeard.clear();
    m_sessionRecorded.clear();

    m_aircraftInfo = OsnDB::getAircraftInformationByReg();
    m_aircraftInfoByIcao = OsnDB::getAircraftInformation();
    m_routeInfo = OsnDB::getAircraftRouteInformation();
    m_navAids = OpenAIP::getNavAids();
    m_airports = OurAirportsDB::getAirportsByIdent();
    m_waypoints = Waypoints::getWaypoints();

    m_flushTimer = new QTimer(this);
    connect(m_flushTimer, &QTimer::timeout, this, &AircraftTracker::flush);
    m_flushTimer->start(300);

    m_removalTimer = new QTimer(this);
    connect(m_removalTimer, &QTimer::timeout, this, &AircraftTracker::removeOldAircraft);
    m_removalTimer->start(60*1000);

    m_chartTimer = new QTimer(this);
    connect(m_chartTimer, &QTimer::timeout, this, &AircraftTracker::updateChartCounts);
    m_chartTimer->start(10*1000);
    m_chartRateTime = QDateTime::currentDateTime();

    if (openDatabase())
    {
        loadDatabase();
        // The all time aircraft count lives in the database, so it can only be read once
        // that is open - which may be after the settings first arrived
        m_statistics.m_allTime.m_distinctAircraft = seenCount();
        m_statisticsDirty = true;
    }
}

void AircraftTracker::handleInputMessages()
{
    Message* message;
    while ((message = m_inputMessageQueue.pop()))
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

bool AircraftTracker::handleMessage(const Message& message)
{
    if (MsgReport::match(message))
    {
        MsgReport& report = (MsgReport&) message;
        processReport(report.getReport());
        return true;
    }
    else if (MsgConfigureTracker::match(message))
    {
        const MsgConfigureTracker& cfg = (const MsgConfigureTracker&) message;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgDeleteAll::match(message))
    {
        deleteAll();
        return true;
    }
    else if (MsgResetStatistics::match(message))
    {
        MsgResetStatistics& msg = (MsgResetStatistics&) message;
        if (msg.getAllTime())
        {
            m_statistics.m_allTime = AircraftStatistics::Scope();
            m_statistics.m_firstStart = QDateTime::currentDateTime();
            m_allTimeSecondsBase = 0;
            m_allTimeSince = m_statistics.m_firstStart;
            // Without this the aircraft count would come straight back, since it is
            // counted from the table rather than held as a number
            resetSeen();
            // Anything heard since the reset still belongs to the new all time figures,
            // so let this session's airframes be counted again
            m_sessionRecorded.clear();
        }
        else
        {
            m_statistics.m_session = AircraftStatistics::Scope();
            m_statistics.m_sessionStart = QDateTime::currentDateTime();
            m_sessionHeard.clear();
        }
        m_statisticsDirty = true;
        return true;
    }
    else if (MsgResync::match(message))
    {
        resync();
        return true;
    }
    else if (MsgWatchFlight::match(message))
    {
        const MsgWatchFlight& watch = (const MsgWatchFlight&) message;
        TrackedFlight *previous = flightById(m_watchedFlight);
        m_watchedFlight = watch.getFlightId();
        TrackedFlight *flight = flightById(m_watchedFlight);
        // The map draws the watched flight's track rather than the current one, so both
        // the aircraft that is no longer watched and the one now watched need theirs sent
        for (TrackedFlight *f : { previous, flight })
        {
            if (f && f->m_aircraft)
            {
                f->m_aircraft->m_mapTrackChanged = true;
                m_dirtyAircraft.insert(f->m_aircraft);
            }
        }
        if (flight) {
            emitProfile(flight);
        }
        return true;
    }
    return false;
}

void AircraftTracker::applySettings(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    bool atcLabelsChanged = settingsKeys.contains("atcLabels") && (settings.m_atcLabels != m_settings.m_atcLabels);
    // The callsign style is part of the label too - atcLabel() writes the flight through
    // flightLabel(), which is what turns BAW123 into SPEEDBIRD 123 - so switching it has
    // to refresh the Map for exactly the same reason toggling the ATC button does
    bool atcCallsignsChanged = settingsKeys.contains("atcCallsigns")
        && (settings.m_atcCallsigns != m_settings.m_atcCallsigns);
    bool liveryChanged = settingsKeys.contains("favourLivery")
        && (settings.m_favourLivery != m_settings.m_favourLivery);
    // The icon is part of the map item, so switching icon styles has to refresh the Map
    // just like a label change does
    bool liveryIconsChanged = settingsKeys.contains("useLiveryIcons")
        && (settings.m_useLiveryIcons != m_settings.m_useLiveryIcons);
    // A forced configuration replaces the settings wholesale and may carry no key list
    // at all, which is how a preset arrives - and the database is opened in startWork()
    // before that happens, on the default path. Comparing the values rather than trusting
    // the key list is what lets a preset's own database be opened.
    bool databaseChanged = (force || settingsKeys.contains("databaseFilename"))
        && (settings.m_databaseFilename != m_settings.m_databaseFilename);
    QString oldDatabasePath = databaseChanged ? databasePath(m_settings) : QString();

    const bool statisticsArrived = settingsKeys.contains("statistics") || force;
    // Which rule fired for an aircraft is remembered by its INDEX in the list, so
    // editing the list silently changes what those indices mean: a new rule that lands
    // on the index of one that has already fired would never fire itself. Compared by
    // content rather than by the key list, because a forced configuration carries no
    // keys at all, and re-firing every rule on an unchanged preset load would be its own
    // bug - the aircraft would all announce themselves again.
    const QString notificationsBefore = notificationSignature(m_settings);
    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
    if (notificationSignature(m_settings) != notificationsBefore)
    {
        for (TrackedAircraft *item : m_aircraft) {
            item->m_notifiedRules.clear();
        }
    }

    // The records come back from the preset the first time settings arrive. Reloading
    // them later would undo anything set since - EXCEPT when the blob is empty, which is
    // how the GUI asks for a reset. Without that exception "Reset records" did nothing
    // after startup and the next update put the old values straight back.
    const bool resetRequested = settingsKeys.contains("statistics")
                             && settings.m_statistics.isEmpty();
    // Wait for a blob with something in it. The GUI's constructor forces a configuration
    // through with default settings BEFORE the framework calls deserialize(), so keying
    // this on "the first settings to arrive" consumed the one chance to load on an empty
    // blob, and the stored records were then never read - the all time figures started
    // again from nothing on every launch.
    const bool haveStored = !m_settings.m_statistics.isEmpty();
    if (statisticsArrived && ((!m_statisticsLoaded && haveStored) || resetRequested))
    {
        m_statisticsLoaded = haveStored;
        AircraftStatistics stored = deserializeStatistics(m_settings.m_statistics);
        // Only the all time side is stored, so keep everything this session has counted
        stored.m_session = m_statistics.m_session;
        stored.m_sessionStart = m_statistics.m_sessionStart;
        if (!stored.m_firstStart.isValid())
        {
            // Nothing stored: either this is the first run, or a reset has just set the
            // moment the all time figures start from, which must not be discarded here
            stored.m_firstStart = m_statistics.m_firstStart.isValid()
                                ? m_statistics.m_firstStart : m_statistics.m_sessionStart;
        }
        m_allTimeSecondsBase = stored.m_allTime.m_seconds;
        m_allTimeSince = (m_allTimeSecondsBase == 0) && m_statistics.m_firstStart.isValid()
                       ? m_statistics.m_firstStart : m_statistics.m_sessionStart;
        // The aircraft count is the one figure the blob cannot hold - see the note on
        // AircraftStatistics. It comes from the database, which may not be open yet on
        // the first pass, in which case the next flush picks it up
        stored.m_allTime.m_distinctAircraft = seenCount();
        m_statistics = stored;
        m_statisticsDirty = true;
    }

    if (databaseChanged) {
        changeDatabase(oldDatabasePath);
    }

    // Refresh the labels of aircraft already on the Map. Only active aircraft are on
    // it, so archived ones must not be sent here - that would put the whole history
    // back on the Map every time settings are applied.
    //
    // Pass-through aircraft are included, which flush() deliberately does NOT do. The
    // reasoning that holds there does not hold here. flush() runs on every report, and
    // a basic item built by this feature would keep replacing the richer one ADS-B is
    // sending, so while ADS-B is talking it stands back. This is a one-off refresh after
    // the user toggled a setting, and skipping pass-through aircraft on the assumption
    // that "their next ADS-B report will relabel them" is only true for aircraft ADS-B
    // is STILL hearing. One that went quiet a few seconds ago gets skipped here and
    // never receives another report, so its label stayed stale indefinitely - which is
    // exactly the reported symptom of only some labels changing.
    //
    // The cost is small and self-correcting: an aircraft ADS-B is actively sending gets
    // a plainer popup for one update, restored on its next report a fraction of a second
    // later. The icon, the heading and the 3D model are all preserved, because
    // sendToMap() re-sends the model remembered from the pass-through.
    // Choosing again is only meaningful for models we picked ourselves; a pass-through
    // model belongs to the demodulator, which will send its own next report anyway
    if (liveryChanged)
    {
        for (TrackedAircraft *item : m_aircraft)
        {
            if (item->m_modelIsOurs)
            {
                item->m_model3D = "";
                item->m_modelIsOurs = false;
                update3DModel(item);
            }
        }
    }

    if (atcLabelsChanged || atcCallsignsChanged || liveryChanged || liveryIconsChanged || force)
    {
        for (TrackedAircraft *item : m_aircraft)
        {
            if (item->m_active && item->m_positionValid) {
                sendToMap(item);
            }
        }
    }
}

QString AircraftTracker::regFor(const TrackedAircraft *item) const
{
    return !item->m_registration.isEmpty() ? item->m_registration
        : (item->m_icao ? QString("%1").arg(item->m_icao, 6, 16, QChar('0')).toUpper() : QString());
}

// Identity keys are namespaced so an ICAO hex string can never collide with a
// registration or a flight number
static QString icaoKey(quint32 icao) { return QString("I%1").arg(icao, 6, 16, QChar('0')); }
static QString regKey(const QString& reg) { return "R" + reg; }
static QString flightKey(const QString& flight) { return "F" + flight; }

// Give an identity to this aircraft, folding in any other record that already held it.
// A registration and an ICAO address each belong to exactly one airframe, so two records
// answering to the same one ARE the same airframe - an ACARS registration-only record and
// an ADS-B ICAO-only record, tied together the moment a message carries both. The key was
// simply re-pointed at the survivor before, leaving the other record in the list holding
// its flights, documents and messages, and unreachable by any key.
//
// Deliberately not used for a flight number, which is not an identity: it is reused day
// after day and by different airframes, and the caller has its own narrower rule for it.
void AircraftTracker::claimKey(TrackedAircraft *item, const QString& key)
{
    TrackedAircraft *other = m_byKey.value(key);
    if (other && (other != item)) {
        absorbAircraft(item, other);
    }
    m_byKey.insert(key, item);
    // The identity has just strengthened, so what it is recognised by in the all time
    // table may have changed with it - see recordSeen()
    m_sessionRecorded.remove(item->m_id);
}

// A flight key stays registered for as long as the aircraft does, so asking when the
// AIRCRAFT was last heard says nothing about the callsign: an airframe heard a minute ago
// under today's callsign would keep every historical one it has ever flown looking just
// as fresh, and claim a report belonging to whoever is flying that number now.
QDateTime AircraftTracker::flightLastSeen(const TrackedAircraft *item, const QString& flight)
{
    QDateTime when;
    for (const TrackedFlight *f : item->m_flights)
    {
        if (!flightMatches(f, flight) || !f->m_lastSeen.isValid()) {
            continue;
        }
        if (!when.isValid() || (f->m_lastSeen > when)) {
            when = f->m_lastSeen;
        }
    }
    return when;
}

// Two flights of one airframe are the same operation when their times overlap, or when
// the gap between them is short enough that the aircraft never went away. The archive
// timeout is that boundary at run time - a gap longer than it starts a new flight - and
// using it here as well is what keeps a reload from rejoining what a session split.
bool AircraftTracker::sameOperation(const TrackedFlight *a, const TrackedFlight *b) const
{
    if (!a->m_firstSeen.isValid() || !a->m_lastSeen.isValid()
        || !b->m_firstSeen.isValid() || !b->m_lastSeen.isValid())
    {
        return true;        // Nothing to separate them by, so behave as before
    }
    if ((a->m_firstSeen <= b->m_lastSeen) && (b->m_firstSeen <= a->m_lastSeen)) {
        return true;        // Overlapping
    }
    const qint64 gap = (a->m_lastSeen < b->m_firstSeen)
        ? a->m_lastSeen.secsTo(b->m_firstSeen) : b->m_lastSeen.secsTo(a->m_firstSeen);
    return gap <= (qint64) m_settings.m_removalMins * 60;
}

AircraftTracker::TrackedAircraft *AircraftTracker::findOrCreateAircraft(const AircraftReport& report)
{
    TrackedAircraft *item = nullptr;

    // Strongest identity first
    if (report.m_icao && m_byKey.contains(icaoKey(report.m_icao))) {
        item = m_byKey.value(icaoKey(report.m_icao));
    } else if (!report.m_registration.isEmpty() && m_byKey.contains(regKey(report.m_registration))) {
        item = m_byKey.value(regKey(report.m_registration));
    }

    // A flight number is NOT an identity - it is reused, day after day and by different
    // airframes. Matching on one is only safe when the report offers nothing better:
    // an unseen ICAO address arriving on a historical flight number would otherwise be
    // folded in to the aircraft that flew it last, and then ignored, because the code
    // below only adopts an identity the item does not already have.
    //
    // Even with nothing stronger it has to be recent. The key lives as long as the
    // aircraft does, so without this a flight number seen days ago still claims today's.
    if (!item && !report.m_flight.isEmpty()
        && (report.m_icao == 0) && report.m_registration.isEmpty()
        && m_byKey.contains(flightKey(report.m_flight)))
    {
        TrackedAircraft *candidate = m_byKey.value(flightKey(report.m_flight));
        const QDateTime when = report.m_received.isValid() ? report.m_received
                                                           : QDateTime::currentDateTime();
        // How recently this aircraft flew THIS callsign, not how recently it was heard.
        //
        // Bounded at both ends: secsTo() runs negative when the report is OLDER than what
        // was last heard, and a negative number is less than any window - so a delayed
        // report claiming a callsign since taken up by another airframe was matched to
        // that airframe rather than rejected.
        const QDateTime flownAt = flightLastSeen(candidate, report.m_flight);
        const qint64 age = flownAt.secsTo(when);
        if (flownAt.isValid() && (age >= 0) && (age < m_flightKeyValidSecs)) {
            item = candidate;
        }
    }

    if (!item)
    {
        item = new TrackedAircraft();
        item->m_id = m_nextId++;
        m_aircraft.append(item);
    }

    // Adopt any stronger identities the report carries and register their keys
    if (report.m_icao && (item->m_icao == 0))
    {
        item->m_icao = report.m_icao;
        claimKey(item, icaoKey(report.m_icao));
    }
    if (!report.m_registration.isEmpty() && item->m_registration.isEmpty())
    {
        item->m_registration = report.m_registration;
        claimKey(item, regKey(report.m_registration));
    }
    if (!report.m_flight.isEmpty())
    {
        // If the flight key belongs to a weaker entry created before this
        // airframe was identified, fold it in rather than orphaning it - but only when
        // that entry was flying this callsign at about the same time. A callsign is
        // flown again day after day, so an unbounded match folded a flight-only record
        // of somebody else's operation into this airframe.
        TrackedAircraft *other = m_byKey.value(flightKey(report.m_flight));
        if (other && (other != item) && (other->m_icao == 0) && other->m_registration.isEmpty())
        {
            const QDateTime flownAt = flightLastSeen(other, report.m_flight);
            const QDateTime when = report.m_received.isValid() ? report.m_received
                                                               : QDateTime::currentDateTime();
            const qint64 age = flownAt.isValid() ? flownAt.secsTo(when) : -1;
            if (flownAt.isValid() && (age >= 0) && (age < m_flightKeyValidSecs)) {
                absorbAircraft(item, other);
            }
        }
        item->m_flight = report.m_flight;
        m_byKey.insert(flightKey(report.m_flight), item);
    }

    // Cross reference registration and ICAO through the OpenSky database
    if (item->m_icao && item->m_registration.isEmpty()
        && m_aircraftInfoByIcao && m_aircraftInfoByIcao->contains((int) item->m_icao))
    {
        item->m_registration = m_aircraftInfoByIcao->value((int) item->m_icao)->m_registration;
        if (!item->m_registration.isEmpty()) {
            claimKey(item, regKey(item->m_registration));
        }
    }
    if ((item->m_icao == 0) && !item->m_registration.isEmpty()
        && m_aircraftInfo && m_aircraftInfo->contains(item->m_registration))
    {
        item->m_icao = (quint32) m_aircraftInfo->value(item->m_registration)->m_icao;
        if (item->m_icao) {
            claimKey(item, icaoKey(item->m_icao));
        }
    }

    return item;
}

// A location in a link management frame is a coarse handoff hint rather than a fix, so
// it is trusted least. Everything else that isn't ADS-B is a real position report
// A link management frame's altitude is both coarse and, on some aircraft, wrong - so
// it is only believed when nothing better has been heard lately, and never sets a record
int AircraftTracker::altitudeRank(const AircraftReport& report)
{
    if (report.m_protocol == AircraftReport::ADSB) {
        return AltitudeAdsb;
    }
    if (report.m_documentKind == AircraftReport::Logon) {
        return AltitudeCoarse;
    }
    return AltitudeReported;
}

int AircraftTracker::positionRank(const AircraftReport& report)
{
    if (report.m_protocol == AircraftReport::ADSB) {
        return PositionAdsb;
    }
    if (report.m_documentKind == AircraftReport::Logon) {
        return PositionCoarse;
    }

    return PositionReported;
}

void AircraftTracker::processReport(AircraftReport& report)
{
    TrackedAircraft *item = findOrCreateAircraft(report);

    // Receiving data makes it active, which puts it in the active tables and on the map
    setActive(item, true);

    // The LATEST report, not the last one to arrive. A replayed or delayed report can
    // be hours old, and assigning it made a currently active aircraft look stale - at
    // which point the removal timer archived it and took it off the Map.
    if (report.m_received.isValid()
        && (!item->m_lastSeen.isValid() || (report.m_received > item->m_lastSeen))) {
        item->m_lastSeen = report.m_received;
    }
    item->m_messages++;
    item->m_sources[qMakePair((int) report.m_protocol, report.m_frequency)]++;

    if (!report.m_documentText.isEmpty()) {
        item->m_lastDocumentText = report.m_documentText;
    }

    // For the message rate chart: one series per protocol then a total
    if ((report.m_protocol >= 0) && (report.m_protocol < AircraftReport::ProtocolCount)) {
        m_chartCounts[report.m_protocol]++;
    }
    m_chartCounts[AircraftReport::ProtocolCount]++;

    // Track the flight this report belongs to (route facts are collated there),
    // and collect notable documents and ATC messages. The aircraft shows the
    // flight's primary name, so it doesn't flap between the ACARS flight number
    // and the ADS-B callsign forms.
    TrackedFlight *flight = updateFlight(item, report);
    // The same tally as the airframe's, but only for the flight it is on now
    if (flight) {
        flight->m_sources[qMakePair((int) report.m_protocol, report.m_frequency)]++;
    }
    if (flight) {
        flight->m_messages++;
    }
    if (!flight->m_flight.isEmpty()) {
        item->m_flight = flight->m_flight;
    }
    addDocument(item, flight, report);
    addAtcMessage(item, flight, report);
    addWeatherReport(item, report);

    checkNotifications(item, report);

    // Altitude is measured separately from position - on ADS-B by a different message
    // entirely - so it is taken in on its own account. It used to be updated only
    // alongside a position that had passed the ranking and plausibility checks, which
    // meant a distant aircraft whose position was rejected lost its altitude with it,
    // and a protocol that reports altitude without a position never set one at all.
    // Whether THIS report's altitude is one worth recording - see altitudeRank()
    bool reportAltitudeValid = false;
    float reportAltitudeFt = 0.0f;

    if (report.m_altitudeValid)
    {
        // Not if we already hold a later observation - a stored record from a log dump
        // arrives long after the altitude it describes
        const bool stale = report.m_altitudeDateTime.isValid()
            && item->m_altitudeDateTime.isValid()
            && (report.m_altitudeDateTime < item->m_altitudeDateTime);

        const int altRank = altitudeRank(report);
        // When each source was last heard, so it has to move forward only. A replayed or
        // delayed report carries an older reception time, and writing it here made live
        // ADS-B look as though it had gone quiet - at which point the coarser sources
        // below were let through to replace an altitude that was perfectly good.
        if (report.m_received.isValid()
            && (!item->m_altitudeRankTimes[altRank].isValid()
                || (report.m_received > item->m_altitudeRankTimes[altRank]))) {
            item->m_altitudeRankTimes[altRank] = report.m_received;
        }

        // Only fall back to a coarser source once the better ones have gone quiet, as
        // positions already do. Without this a single bad XID octet replaced a good
        // ADS-B altitude until the next ADS-B message arrived.
        bool altSuperseded = false;
        for (int better = 0; better < altRank; better++)
        {
            if (item->m_altitudeRankTimes[better].isValid()
                && (item->m_altitudeRankTimes[better].secsTo(report.m_received) < m_positionRankTimeoutSecs)) {
                altSuperseded = true;
            }
        }

        // The records take the altitude THIS report carried, and never a coarse one:
        // an aircraft heard only on link management frames is better left without a
        // highest-altitude record than given a wrong one, because a record is never
        // corrected afterwards
        reportAltitudeValid = (altRank != AltitudeCoarse);
        reportAltitudeFt = report.m_altitudeFt;

        if (!stale && !altSuperseded)
        {
            // The one before it is kept as well, so a position timestamped between the
            // two can have its altitude worked out rather than guessed - see altitudeAt()
            if (report.m_altitudeDateTime.isValid() && item->m_altitudeDateTime.isValid()
                && (report.m_altitudeDateTime > item->m_altitudeDateTime))
            {
                item->m_prevAltitudeFt = item->m_altitudeFt;
                item->m_prevAltitudeDateTime = item->m_altitudeDateTime;
            }
            item->m_altitudeFt = report.m_altitudeFt;
            item->m_altitudeValid = true;
            if (report.m_altitudeDateTime.isValid()) {
                item->m_altitudeDateTime = report.m_altitudeDateTime;
            }
        }
    }

    // Whether THIS report supplied the position we are now holding - see updateStatistics()
    bool positionAccepted = false;
    // How far away the position THIS report carried is, whether or not it was adopted
    bool reportRangeValid = false;
    float reportRangeKm = 0.0f;

    // Exactly 0N 0E is a zeroed-out field, not a position off the coast of Africa
    bool positionValid = report.m_positionValid
        && ((report.m_latitude != 0.0f) || (report.m_longitude != 0.0f));

    if (positionValid)
    {
        // A report timestamped before the position we are already holding is a report
        // of where the aircraft USED to be - a stored record from a log dump (seen with
        // HFDL performance data), or simply out of order. Adopting one rewound the
        // coordinates while the timestamp stayed where it was, so the two then described
        // different moments, and appended a point to the track out of time order.
        //
        // The one exception is a held position that has itself gone stale: it is on its
        // way off the Map, so an older fix is still an improvement on it. That exception
        // is needed because the protocols timestamp differently - ADS-B stamps when we
        // received the frame, an ACARS position report when the aircraft measured it, so
        // an ACARS fix routinely looks minutes old the moment it arrives. Without it, an
        // aircraft that had just left ADS-B coverage would refuse its own oceanic
        // positions until their clock caught up with the last ADS-B one.
        QDateTime posTime = report.m_positionDateTime.isValid() ? report.m_positionDateTime : report.m_received;
        bool current = !item->m_positionDateTime.isValid()
            || (item->m_positionDateTime.secsTo(posTime) >= 0)
            || positionStale(item, report.m_received);

        const QGeoCoordinate reported(report.m_latitude, report.m_longitude);
        const int rank = positionRank(report);

        // Only fall back to a less accurate source once the better ones have gone quiet
        bool superseded = false;
        for (int better = 0; better < rank; better++)
        {
            if (item->m_positionRankTimes[better].isValid()
                && (item->m_positionRankTimes[better].secsTo(report.m_received) < m_positionRankTimeoutSecs)) {
                superseded = true;
            }
        }

        // A position the aircraft could not have flown to since the last one is a report
        // of where it used to be, or a bad decode - not where it is now. The tolerance
        // covers the resolution of the position and any slop in the times
        // A better source is always allowed to correct a worse one - otherwise a coarse
        // position 11 km out could reject the ADS-B fix that would have put it right
        bool plausible = true;
        if (item->m_positionValid && item->m_positionDateTime.isValid() && (rank >= item->m_positionRank))
        {
            const double km = QGeoCoordinate(item->m_latitude, item->m_longitude).distanceTo(reported) / 1000.0;
            const double hours = std::abs((double) item->m_positionDateTime.secsTo(posTime)) / 3600.0;
            plausible = km <= (m_maxGroundSpeedKn * 1.852 * hours) + m_positionToleranceKm;
        }

        // The range record for a protocol asks how far away the aircraft was when we heard
        // it on that protocol, so it uses the position THIS report carried - not the one we
        // end up holding, which may have come from a better source. Gating it on the
        // position being ADOPTED was wrong: an aircraft inside ADS-B coverage has every
        // ACARS, VDL-2 and Aero position superseded within the rank timeout, so those
        // protocols could never record a range at all however far away they heard it. A
        // report carrying no position still measures nothing, which is what stopped
        // ADS-B distances being credited to HFDL.
        //
        // Checked against the position we are holding whatever its rank - unlike the
        // tracked position, a range record is never corrected later, so a bad decode
        // would leave a wrong figure in it for ever.
        bool rangePlausible = true;
        if (item->m_positionValid && item->m_positionDateTime.isValid())
        {
            const double km = QGeoCoordinate(item->m_latitude, item->m_longitude).distanceTo(reported) / 1000.0;
            const double hours = std::abs((double) item->m_positionDateTime.secsTo(posTime)) / 3600.0;
            rangePlausible = km <= (m_maxGroundSpeedKn * 1.852 * hours) + m_positionToleranceKm;
        }
        if (rangePlausible)
        {
            // Distance from the antenna that heard this report, not from "here" - a
            // remote SDR is somewhere else entirely, and a range figure measured from
            // the wrong end is worse than none
            const QGeoCoordinate receiver = receiverPosition(report.m_deviceSetIndex);
            if (receiver.isValid())
            {
                reportRangeKm = (float) (receiver.distanceTo(reported) / 1000.0);
                reportRangeValid = true;
            }
        }

        if (current && !superseded && plausible)
        {
            positionAccepted = true;
            item->m_positionRankTimes[rank] = report.m_received;
            item->m_positionRank = rank;
            item->m_positionValid = true;
            item->m_latitude = report.m_latitude;
            item->m_longitude = report.m_longitude;
            if (reportRangeValid)
            {
                item->m_distanceKm = reportRangeKm;
                item->m_distanceValid = true;
            }
            // The position and the moment it describes always come from the same
            // report. Keeping the newer of the two timestamps while taking an older
            // report's coordinates is what made the pair disagree.
            item->m_positionDateTime = posTime;

            // Accumulate the full track - it survives gaps between protocols
            // The altitude that belongs with this fix is the altitude AT posTime, which
            // on ADS-B is not the newest one received - see altitudeAt()
            const float trackAltFt = report.m_altitudeValid
                ? altitudeAt(item, posTime, report.m_altitudeFt) : 0.0f;
            QGeoCoordinate coordinate(report.m_latitude, report.m_longitude,
                report.m_altitudeValid ? Units::feetToMetres(trackAltFt) : 0.0f);
            // Every reported position is kept. There is deliberately no distance
            // threshold: a transponder reports at its own rate whatever the aircraft is
            // doing, so a distance guard only ever discards points from SLOW aircraft -
            // taxiing, on approach - which is precisely where the track needs detail,
            // while doing nothing at all about a fast one. The only thing worth dropping
            // is a position identical to the one before it, which carries nothing.
            // Position only. An altitude report does not move the aircraft, and it
            // arrives carrying the position and timestamp of the LAST position report,
            // so appending on any change of altitude filled the track with points at a
            // repeated place and time. The altitude recorded against a track point is
            // the altitude when that position was reported, which is what it should be.
            // ... and in time order. An older fix is adopted only when the position
            // being held has gone stale, but appending it after a newer point would
            // leave the track's times running backwards - and the Map's smoother divides
            // by the difference between consecutive times, so a negative one gives inf
            // and then nan, exactly as two identical times do.
            const bool inOrder = flight && (flight->m_trackTimes.isEmpty()
                || (posTime >= flight->m_trackTimes.last()));
            const bool moved = inOrder && (flight->m_track.isEmpty()
                || (flight->m_track.last().latitude() != coordinate.latitude())
                || (flight->m_track.last().longitude() != coordinate.longitude()));
            if (moved)
            {
                // Heading along the track when the report has none
                if (!flight->m_track.isEmpty())
                {
                    float trackHeading = (float) flight->m_track.last().azimuthTo(coordinate);
                    if (!report.m_headingValid)
                    {
                        item->m_heading = trackHeading;
                        item->m_headingValid = true;
                        item->m_headingDateTime = posTime;
                    }
                }
                flight->m_track.append(coordinate);
                flight->m_trackTimes.append(posTime);
                item->m_mapTrackChanged = true;

            }
        }
    }
    // Both only move forward, as the position, the altitude and the last seen time do:
    // a delayed report describes what the aircraft was doing then, not now
    const QDateTime observed = report.m_received.isValid() ? report.m_received
                                                           : QDateTime::currentDateTime();
    if (report.m_headingValid
        && (!item->m_headingDateTime.isValid() || (observed >= item->m_headingDateTime)))
    {
        item->m_heading = report.m_heading;
        item->m_headingValid = true;
        item->m_headingDateTime = observed;
    }
    if (report.m_speedValid
        && (!item->m_speedDateTime.isValid() || (observed >= item->m_speedDateTime)))
    {
        item->m_speedKts = report.m_speedKts;
        item->m_speedValid = true;
        item->m_speedDateTime = observed;
    }

    // The GUI and basic map item are updated from the flush timer
    update3DModel(item);
    updateStatistics(report, item, reportRangeValid, reportRangeKm,
                     reportAltitudeValid, reportAltitudeFt);
    m_dbDirty = true;
    m_dirtyAircraft.insert(item);

    if (report.m_mapItem)
    {
        // ADS-B pass-through goes straight out - the item carries this
        // report's aircraft state for the Map's PFD
        forwardMapItem(item, report.m_mapItem);
        report.m_mapItem = nullptr;     // Ownership transferred
        item->m_lastMapItemTime = report.m_received;
    }
}

// Flush batched GUI notifications and basic map updates
void AircraftTracker::flush()
{
    if (!m_dirtyAircraft.isEmpty())
    {
        const QDateTime now = QDateTime::currentDateTime();
        QList<AircraftDisplay> displays;
        for (TrackedAircraft *item : m_dirtyAircraft)
        {
            displays.append(displayFor(item));

            // While pass-through items are flowing, a basic item built here
            // would replace the rich one, so only send when ADS-B is quiet.
            // Only aircraft we are currently hearing are drawn.
            bool passThroughActive = item->m_lastMapItemTime.isValid()
                && (item->m_lastMapItemTime.secsTo(now) < 60);
            if (item->m_active && item->m_positionValid && !passThroughActive)
            {
                sendToMap(item);
                item->m_mapResync = false;
                m_mapResyncPending.remove(item);
            }
        }
        m_dirtyAircraft.clear();
        emit aircraftUpdated(displays);
    }

    // Aircraft a Map that has just opened is still owed. They were not sendable when
    // they were last looked at - typically their ADS-B pass-through had been quiet for
    // less than a minute, so a basic item would have replaced a richer one - and the
    // dirty set they came from is emptied every pass, so without a queue of their own a
    // quiet aircraft would simply never reach that Map.
    if (!m_mapResyncPending.isEmpty())
    {
        const QDateTime now = QDateTime::currentDateTime();
        const QSet<TrackedAircraft *> owed = m_mapResyncPending;
        for (TrackedAircraft *item : owed)
        {
            const bool passThroughActive = item->m_lastMapItemTime.isValid()
                && (item->m_lastMapItemTime.secsTo(now) < 60);
            if (!item->m_active)
            {
                m_mapResyncPending.remove(item);    // Archived, so not on any Map
            }
            else if (item->m_positionValid && !passThroughActive)
            {
                sendToMap(item);
                item->m_mapResync = false;
                m_mapResyncPending.remove(item);
            }
        }
    }

    // After the aircraft loop above has emptied m_dirtyAircraft, so the aircraft it marks
    // dirty go out on the next flush rather than being cleared unsent. The routes and the
    // record marker below are re-sent within this one.
    checkMapConsumers();

    // Both cheap unless something they depend on has changed
    updateRecordMapItem();
    retryRoutes();

    // The concurrent count has to be recomputed on a timer rather than only when a report
    // arrives: aircraft leaving the window lower it, and nothing signals that.
    const QDateTime stamp = QDateTime::currentDateTime();
    updateConcurrent(stamp);

    // Both elapsed figures advance whether or not anything is being received, so nudge
    // the display once a minute rather than leaving the clocks frozen on a quiet band
    if (m_statistics.m_sessionStart.isValid())
    {
        const qint64 minute = m_statistics.m_sessionStart.secsTo(stamp) / 60;
        if (minute != m_statisticsMinute)
        {
            m_statisticsMinute = minute;
            m_statisticsDirty = true;
        }
    }

    if (m_statisticsDirty)
    {
        m_statisticsDirty = false;
        if (m_statistics.m_sessionStart.isValid())
        {
            m_statistics.m_session.m_seconds = m_statistics.m_sessionStart.secsTo(stamp);
            m_statistics.m_allTime.m_seconds = m_allTimeSecondsBase
                + (m_allTimeSince.isValid() ? m_allTimeSince.secsTo(stamp)
                                            : m_statistics.m_session.m_seconds);
        }
        emit statisticsUpdated(m_statistics);
    }

    if (!m_dirtyFlights.isEmpty())
    {
        QList<FlightDisplay> displays;
        bool watchedDirty = false;
        for (TrackedFlight *flight : m_dirtyFlights)
        {
            displays.append(displayFor(flight));
            watchedDirty = watchedDirty || (flight->m_id == m_watchedFlight);
        }
        m_dirtyFlights.clear();
        emit flightsUpdated(displays);
        if (watchedDirty)
        {
            TrackedFlight *flight = flightById(m_watchedFlight);
            if (flight) {
                emitProfile(flight);
            }
        }
    }

    if (!m_pendingDocuments.isEmpty())
    {
        emit documentsAdded(m_pendingDocuments);
        m_pendingDocuments.clear();
    }
    if (!m_pendingAtc.isEmpty())
    {
        emit atcMessages(m_pendingAtc);
        m_pendingAtc.clear();
    }
    if (!m_pendingWeather.isEmpty())
    {
        emit weatherReports(m_pendingWeather);
        m_pendingWeather.clear();
    }
}

void AircraftTracker::updateChartCounts()
{
    const QDateTime now = QDateTime::currentDateTime();
    const qint64 elapsedMS = m_chartRateTime.msecsTo(now);
    if (elapsedMS <= 0) {
        return;
    }
    QList<float> rates;
    for (int i = 0; i < CHART_COUNTS; i++)
    {
        rates.append(m_chartCounts[i] / (elapsedMS / 1000.0f));
        m_chartCounts[i] = 0;
    }
    m_chartRateTime = now;
    emit messageRates(rates);
}

AircraftDisplay AircraftTracker::displayFor(const TrackedAircraft *item) const
{
    AircraftDisplay d;
    d.m_id = item->m_id;
    d.m_icao = item->m_icao;
    d.m_registration = item->m_registration;
    d.m_flight = item->m_flight;
    d.m_positionValid = item->m_positionValid;
    d.m_distanceValid = item->m_distanceValid;
    d.m_distanceKm = item->m_distanceKm;
    d.m_latitude = item->m_latitude;
    d.m_longitude = item->m_longitude;
    d.m_altitudeValid = item->m_altitudeValid;
    d.m_altitudeFt = item->m_altitudeFt;
    d.m_headingValid = item->m_headingValid;
    d.m_heading = item->m_heading;
    d.m_speedValid = item->m_speedValid;
    d.m_speedKts = item->m_speedKts;
    d.m_protocols = sourcesText(item->m_sources);
    d.m_messages = item->m_messages;
    d.m_lastSeen = item->m_lastSeen;
    d.m_mapName = item->m_mapName.isEmpty() ? mapItemName(item) : item->m_mapName;
    d.m_currentFlightId = item->m_currentFlight ? item->m_currentFlight->m_id : 0;
    d.m_active = item->m_active;
    return d;
}

FlightDisplay AircraftTracker::displayFor(const TrackedFlight *flight) const
{
    FlightDisplay d;
    d.m_id = flight->m_id;
    d.m_aircraftId = flight->m_aircraft->m_id;
    d.m_flight = flight->m_flight;
    d.m_aliases = flight->m_aliases.join(", ");
    d.m_reg = regFor(flight->m_aircraft);
    d.m_departure = flight->m_departure;
    d.m_arrival = flight->m_arrival;
    d.m_route = flight->m_route;
    d.m_firstSeen = flight->m_firstSeen;
    d.m_lastSeen = flight->m_lastSeen;
    d.m_documents = flight->m_documents.size();
    d.m_messages = flight->m_messages;
    d.m_protocols = sourcesText(flight->m_sources);
    d.m_out = flight->m_out;
    d.m_off = flight->m_off;
    d.m_on = flight->m_on;
    d.m_in = flight->m_in;
    for (const TrackedDocument *doc : flight->m_documents)
    {
        switch (doc->m_kind)
        {
        case AircraftReport::Loadsheet:  d.m_hasLoadsheet = true; break;
        case AircraftReport::Clearance:  d.m_hasClearance = true; break;
        case AircraftReport::FlightPlan: d.m_hasFlightPlan = true; break;
        default: break;
        }
    }
    // Only the flight an active aircraft is currently flying counts as active
    d.m_active = flight->m_aircraft->m_active && (flight->m_aircraft->m_currentFlight == flight);
    return d;
}

DocumentEvent AircraftTracker::documentEvent(const TrackedAircraft *item, const TrackedFlight *flight, const TrackedDocument *doc) const
{
    DocumentEvent e;
    e.m_aircraftId = item->m_id;
    e.m_flightId = flight->m_id;
    e.m_received = doc->m_received;
    e.m_flight = flight->m_flight;
    e.m_reg = regFor(item);
    e.m_kind = documentKindName(doc->m_kind);
    e.m_title = doc->m_title;
    e.m_text = doc->m_text;
    e.m_mapName = item->m_mapName.isEmpty() ? mapItemName(item) : item->m_mapName;
    return e;
}

AircraftTracker::TrackedFlight *AircraftTracker::flightById(quint64 id) const
{
    if (id == 0) {
        return nullptr;
    }
    for (TrackedAircraft *item : m_aircraft)
    {
        for (TrackedFlight *flight : item->m_flights)
        {
            if (flight->m_id == id) {
                return flight;
            }
        }
    }
    return nullptr;
}

void AircraftTracker::emitProfile(const TrackedFlight *flight)
{
    emit profileUpdated(flight->m_id, flight->m_profileTimes, flight->m_profileAltFt, flight->m_profileSpeedKts);
}

// Resend the complete state - the GUI has just connected
void AircraftTracker::resync()
{
    emit allCleared();

    QList<AircraftDisplay> aircraft;
    QList<FlightDisplay> flights;
    QList<DocumentEvent> documents;
    for (TrackedAircraft *item : m_aircraft)
    {
        aircraft.append(displayFor(item));
        for (TrackedFlight *flight : item->m_flights)
        {
            flights.append(displayFor(flight));
            for (TrackedDocument *doc : flight->m_documents) {
                documents.append(documentEvent(item, flight, doc));
            }
        }
    }
    std::sort(documents.begin(), documents.end(),
        [](const DocumentEvent& a, const DocumentEvent& b) { return a.m_received < b.m_received; });

    if (!aircraft.isEmpty()) {
        emit aircraftUpdated(aircraft);
    }
    if (!flights.isEmpty()) {
        emit flightsUpdated(flights);
    }
    if (!documents.isEmpty()) {
        emit documentsAdded(documents);
    }
    if (!m_atcLog.isEmpty()) {
        emit atcMessages(m_atcLog);
    }
    if (!m_weatherLog.isEmpty()) {
        emit weatherReports(m_weatherLog);
    }
}

void AircraftTracker::deleteAll()
{
    for (TrackedAircraft *item : m_aircraft)
    {
        if (!item->m_mapName.isEmpty()) {
            removeFromMap(item->m_mapName);
        }
        // The routes as well as the aircraft. They are named after the flight rather than
        // the airframe, so removing the aircraft leaves the lines and their waypoint
        // markers behind - and a moment later the flights holding the only record of what
        // those items are called are deleted, so nothing can ever take them off.
        for (TrackedFlight *flight : item->m_flights)
        {
            removeDrawnRoute(flight->m_flightPlan);
            removeDrawnRoute(flight->m_clearance);
        }
    }
    qDeleteAll(m_aircraft);
    m_aircraft.clear();
    m_byKey.clear();
    m_atcLog.clear();
    m_dirtyAircraft.clear();
    m_mapResyncPending.clear();
    m_dirtyFlights.clear();
    m_pendingDocuments.clear();
    m_pendingAtc.clear();
    // The weather log is keyed on airports rather than aircraft, so it is easy to
    // forget here - but the GUI clears its rows on allCleared(), and leaving the log
    // behind put every report straight back on the next resync. Worse, the dedupe index
    // survived too, so a report that arrived again after the clear was recognised as
    // one already held and never re-emitted at all.
    m_pendingWeather.clear();
    m_weatherLog.clear();
    m_weatherIndex.clear();
    m_routesPending.clear();
    m_watchedFlight = 0;
    forgetSavedTracks();
    m_dbDirty = true;
    emit allCleared();
}

// Aircraft we have stopped hearing from are not discarded - they move to the archive
// tables and come off the map, so the session's history stays available
void AircraftTracker::removeOldAircraft()
{
    QDateTime currentDateTime = QDateTime::currentDateTime();

    // The aircraft database may have been downloaded since we last looked
    m_aircraftInfo = OsnDB::getAircraftInformationByReg();
    m_aircraftInfoByIcao = OsnDB::getAircraftInformation();
    m_routeInfo = OsnDB::getAircraftRouteInformation();

    // The timer that drives this also drives the periodic session save
    if (m_db && !m_loadingDatabase) {
        saveDatabase();
    }

    for (TrackedAircraft *item : m_aircraft)
    {
        if (item->m_active
            && item->m_lastSeen.isValid()
            && (item->m_lastSeen.secsTo(currentDateTime) > m_settings.m_removalMins * 60))
        {
            setActive(item, false);
            continue;
        }

        // A stale position comes off the Map while the aircraft stays in the tables.
        // The 3D map drops the entity on its own when the availability interval ends,
        // but nothing takes it off the 2D map, so it would sit at its last known
        // position until the aircraft was archived - three hours later by default.
        if (item->m_active && !item->m_mapName.isEmpty()
            && positionStale(item, currentDateTime))
        {
            removeFromMap(item->m_mapName);
            item->m_mapName = "";       // Re-sent in full if a fresh position arrives
            item->m_mapTrackChanged = true;
        }
    }

    discardOldAircraft();
}

// Archived aircraft are kept for a while so their history is still there, but not for
// ever - the session database is rewritten in full on every save, so it has to be
// bounded. A retention of zero means the user wants to keep everything.
void AircraftTracker::discardOldAircraft()
{
    if (m_settings.m_retentionDays <= 0) {
        return;
    }

    const QDateTime cutoff = QDateTime::currentDateTime().addDays(-m_settings.m_retentionDays);
    QList<quint64> discardedAircraft;
    QList<quint64> discardedFlights;

    for (int i = m_aircraft.size() - 1; i >= 0; i--)
    {
        TrackedAircraft *item = m_aircraft[i];

        if (item->m_active || !item->m_lastSeen.isValid() || (item->m_lastSeen >= cutoff)) {
            continue;
        }

        discardedAircraft.append(item->m_id);
        for (TrackedFlight *flight : item->m_flights)
        {
            discardedFlights.append(flight->m_id);
            m_flightsToDelete.insert(flight->m_id);
            m_dirtyFlights.remove(flight);
            if (m_watchedFlight == flight->m_id) {
                m_watchedFlight = 0;
            }
        }
        for (const QString& key : m_byKey.keys(item)) {
            m_byKey.remove(key);
        }
        m_dirtyAircraft.remove(item);
        m_mapResyncPending.remove(item);
        m_aircraft.removeAt(i);
        delete item;
        m_dbDirty = true;
    }

    if (!discardedFlights.isEmpty()) {
        emit flightsRemoved(discardedFlights);
    }
    if (!discardedAircraft.isEmpty())
    {
        qDebug() << "AircraftTracker::discardOldAircraft: discarded" << discardedAircraft.size()
                 << "aircraft not heard for" << m_settings.m_retentionDays << "days";
        emit aircraftRemoved(discardedAircraft);
    }
}

// Moving an aircraft between the active and archive tables
void AircraftTracker::setActive(TrackedAircraft *item, bool active)
{
    if (item->m_active == active) {
        return;
    }

    item->m_active = active;

    if (!active && !item->m_mapName.isEmpty())
    {
        removeFromMap(item->m_mapName);
        item->m_mapTrackChanged = true;   // The whole track is resent if it returns
    }

    m_dirtyAircraft.insert(item);
    for (TrackedFlight *flight : item->m_flights) {
        m_dirtyFlights.insert(flight);
    }
    m_dbDirty = true;
}

// Fold a weaker duplicate into an identified aircraft. A flight-only sighting
// (e.g. ACARS with no registration) creates an aircraft keyed only by its
// flight; when a later report ties that flight to a real airframe, the weak
// entry's data moves across instead of being orphaned.
void AircraftTracker::absorbAircraft(TrackedAircraft *item, TrackedAircraft *weak)
{
    item->m_messages += weak->m_messages;
    QHashIterator<QPair<int, qint64>, int> sourceIt(weak->m_sources);
    while (sourceIt.hasNext())
    {
        sourceIt.next();
        item->m_sources[sourceIt.key()] += sourceIt.value();
    }
    // Which of the two is carrying the current state varies: the weak entry is usually
    // the flight-only ACARS record and the survivor the ICAO-backed one, but it is just
    // as often the other way round. Keeping the survivor's simply because it survives
    // threw away a position, altitude, speed or heading it did not have - so each is
    // merged on validity and then on time.
    const bool takeWeakPosition = weak->m_positionValid
        && (!item->m_positionValid
            || (weak->m_positionDateTime.isValid()
                && (!item->m_positionDateTime.isValid()
                    || (weak->m_positionDateTime > item->m_positionDateTime))));
    if (takeWeakPosition)
    {
        item->m_positionValid = true;
        item->m_latitude = weak->m_latitude;
        item->m_longitude = weak->m_longitude;
        item->m_positionDateTime = weak->m_positionDateTime;
        item->m_positionRank = weak->m_positionRank;
        item->m_distanceValid = weak->m_distanceValid;
        item->m_distanceKm = weak->m_distanceKm;
        // The track moves with the flights, below
        item->m_mapTrackChanged = true;
    }

    // When a position or altitude of each rank was last heard is the union of the two.
    // It is what decides whether a coarser source may overwrite a better one, so losing
    // half of it would let a stale ACARS fix supersede live ADS-B.
    for (int i = 0; i < PositionRanks; i++)
    {
        if (weak->m_positionRankTimes[i].isValid()
            && (!item->m_positionRankTimes[i].isValid()
                || (weak->m_positionRankTimes[i] > item->m_positionRankTimes[i]))) {
            item->m_positionRankTimes[i] = weak->m_positionRankTimes[i];
        }
    }
    for (int i = 0; i < AltitudeRanks; i++)
    {
        if (weak->m_altitudeRankTimes[i].isValid()
            && (!item->m_altitudeRankTimes[i].isValid()
                || (weak->m_altitudeRankTimes[i] > item->m_altitudeRankTimes[i]))) {
            item->m_altitudeRankTimes[i] = weak->m_altitudeRankTimes[i];
        }
    }

    // Altitude is measured separately from position on ADS-B - a Mode S altitude reply
    // carries no position - so it merges separately, and takes with it the pair of
    // observations altitudeAt() interpolates a track point's height between.
    // The two most recent samples of the four, rather than one record's pair. Taking a
    // pair whole discards a newer sample the other record held, and altitudeAt()
    // interpolates a track point's height BETWEEN these two - so the wrong pair does not
    // merely lose detail, it interpolates over the wrong interval.
    if (weak->m_altitudeValid)
    {
        QList<QPair<QDateTime, float>> samples;
        auto offer = [&samples](bool valid, const QDateTime& when, float value)
        {
            if (valid && when.isValid()) {
                samples.append(qMakePair(when, value));
            }
        };
        offer(item->m_altitudeValid, item->m_altitudeDateTime, item->m_altitudeFt);
        offer(item->m_altitudeValid, item->m_prevAltitudeDateTime, item->m_prevAltitudeFt);
        offer(true, weak->m_altitudeDateTime, weak->m_altitudeFt);
        offer(true, weak->m_prevAltitudeDateTime, weak->m_prevAltitudeFt);

        std::sort(samples.begin(), samples.end(),
                  [](const QPair<QDateTime, float>& a, const QPair<QDateTime, float>& b) {
                      return a.first > b.first;
                  });
        // Both records usually hold the same newest observation, having both just heard
        // it. Taking the top two of the sorted list would then take that one instant
        // twice, throw away the real second newest, and leave altitudeAt() with a zero
        // span to interpolate over - which it refuses, so the merge would cost exactly
        // the interpolation it is meant to preserve.
        for (int i = samples.size() - 1; i > 0; i--)
        {
            if (samples[i].first == samples[i - 1].first) {
                samples.removeAt(i);
            }
        }

        if (samples.isEmpty())
        {
            // Neither record timestamped its altitude, so there is nothing to order them
            // by and the survivor's is kept unless it has none at all
            if (!item->m_altitudeValid)
            {
                item->m_altitudeValid = true;
                item->m_altitudeFt = weak->m_altitudeFt;
            }
        }
        else
        {
            item->m_altitudeValid = true;
            item->m_altitudeDateTime = samples[0].first;
            item->m_altitudeFt = samples[0].second;
            if (samples.size() > 1)
            {
                item->m_prevAltitudeDateTime = samples[1].first;
                item->m_prevAltitudeFt = samples[1].second;
            }
        }
    }

    // Neither speed nor heading is timestamped, so "whichever one of the two has it" is
    // as far as this can honestly go
    if (weak->m_speedValid
        && (!item->m_speedValid
            || (weak->m_speedDateTime.isValid()
                && (!item->m_speedDateTime.isValid()
                    || (weak->m_speedDateTime > item->m_speedDateTime)))))
    {
        item->m_speedValid = true;
        item->m_speedKts = weak->m_speedKts;
        item->m_speedDateTime = weak->m_speedDateTime;
    }
    if (weak->m_headingValid
        && (!item->m_headingValid
            || (weak->m_headingDateTime.isValid()
                && (!item->m_headingDateTime.isValid()
                    || (weak->m_headingDateTime > item->m_headingDateTime)))))
    {
        item->m_headingValid = true;
        item->m_heading = weak->m_heading;
        item->m_headingDateTime = weak->m_headingDateTime;
    }
    if (weak->m_lastSeen.isValid()
        && (!item->m_lastSeen.isValid() || (weak->m_lastSeen > item->m_lastSeen))) {
        item->m_lastSeen = weak->m_lastSeen;
    }
    if (item->m_departure.isEmpty()) {
        item->m_departure = weak->m_departure;
    }
    if (item->m_arrival.isEmpty()) {
        item->m_arrival = weak->m_arrival;
    }
    if (item->m_route.isEmpty()) {
        item->m_route = weak->m_route;
    }
    if (item->m_lastDocumentText.isEmpty()) {
        item->m_lastDocumentText = weak->m_lastDocumentText;
    }

    // The weak entry's flight history moves across, oldest first
    for (TrackedFlight *flight : weak->m_flights) {
        flight->m_aircraft = item;
    }
    QList<TrackedFlight *> flights = weak->m_flights;
    flights.append(item->m_flights);
    item->m_flights = flights;
    if (!item->m_currentFlight) {
        item->m_currentFlight = weak->m_currentFlight;
    }
    weak->m_flights.clear();
    weak->m_currentFlight = nullptr;

    for (const QString& key : m_byKey.keys(weak)) {
        m_byKey.insert(key, item);
    }
    if (!weak->m_mapName.isEmpty()) {
        removeFromMap(weak->m_mapName);
    }
    m_dirtyAircraft.remove(weak);
    m_mapResyncPending.remove(weak);
    // It was counted as an aircraft heard when it was created, and it has just turned
    // out to be one already counted - so take it back off both the session set and the
    // count, or identifying an airframe part way through inflates the figure by one
    if (m_sessionHeard.remove(weak->m_id)) {
        m_statistics.m_session.m_distinctAircraft = m_sessionHeard.size();
        m_statisticsDirty = true;
    }
    m_sessionRecorded.remove(weak->m_id);
    emit aircraftRemoved(QList<quint64>{weak->m_id});
    m_aircraft.removeAll(weak);
    delete weak;

    // Merging histories can leave the same flight twice
    mergeLoadedFlights(item);
    m_dirtyAircraft.insert(item);
    for (TrackedFlight *flight : item->m_flights) {
        m_dirtyFlights.insert(flight);
    }
    m_dbDirty = true;
}

// Merge any of an aircraft's flights that are the same flight under different
// names - after loading a session saved before names were aliased, or after
// absorbing a duplicate aircraft
void AircraftTracker::mergeLoadedFlights(TrackedAircraft *item)
{
    for (int i = 0; i < item->m_flights.size(); i++)
    {
        TrackedFlight *dst = item->m_flights[i];
        for (int j = i + 1; j < item->m_flights.size(); )
        {
            TrackedFlight *src = item->m_flights[j];
            bool matches = flightMatches(dst, src->m_flight);
            for (const QString& alias : src->m_aliases) {
                matches = matches || flightMatches(dst, alias);
            }
            // The name is not enough. One airframe flies the same callsign again and
            // again - a daily rotation is the same number every day - and merging on the
            // name alone put every one of those operations into a single flight holding
            // days of track, several sets of OOOI times and several routes. Run time
            // splits them on the archive timeout; this is the same rule, so a reload
            // stops undoing it.
            if (matches && sameOperation(dst, src)) {
                mergeFlightInto(item, dst, src);    // Removes src from the list
            } else {
                j++;
            }
        }
    }

    // One airframe flies one flight at a time, so flights whose time spans
    // overlap are the same flight under names that can't be matched textually
    // (some airlines' callsigns are unrelated to the flight number)
    for (int i = 0; i < item->m_flights.size(); i++)
    {
        TrackedFlight *dst = item->m_flights[i];
        for (int j = i + 1; j < item->m_flights.size(); )
        {
            TrackedFlight *src = item->m_flights[j];
            if (dst->m_firstSeen.isValid() && dst->m_lastSeen.isValid()
                && src->m_firstSeen.isValid() && src->m_lastSeen.isValid()
                && (dst->m_firstSeen <= src->m_lastSeen)
                && (src->m_firstSeen <= dst->m_lastSeen)) {
                mergeFlightInto(item, dst, src);
            } else {
                j++;
            }
        }
    }

    // Keep the current flight at the end of the history
    if (item->m_currentFlight)
    {
        item->m_flights.removeAll(item->m_currentFlight);
        item->m_flights.append(item->m_currentFlight);
    }
}

// Split a flight identity into the airline designator and the flight number:
// a 3-letter ICAO designator (BAW31) or a 2-character IATA one (BA0031), with
// leading zeros stripped from the number so the two forms compare equal
bool AircraftTracker::splitFlightName(const QString& flight, QString& airline, QString& number)
{
    int prefixLen;
    if ((flight.size() > 3) && flight[0].isLetter() && flight[1].isLetter() && flight[2].isLetter()) {
        prefixLen = 3;
    } else if ((flight.size() > 2) && !(flight[0].isDigit() && flight[1].isDigit())) {
        prefixLen = 2;
    } else {
        return false;
    }
    airline = flight.left(prefixLen);
    QString rest = flight.mid(prefixLen);
    int i = 0;
    while ((i < rest.size() - 1) && (rest[i] == '0')) {
        i++;
    }
    number = rest.mid(i);
    return !number.isEmpty();
}

// Whether two names are the same flight in its IATA and ICAO callsign forms,
// e.g. BA0031 and BAW31. Only used within one airframe, so matching on the
// flight number alone is safe.
bool AircraftTracker::sameFlightNumber(const QString& a, const QString& b)
{
    QString airlineA, numberA, airlineB, numberB;
    if (!splitFlightName(a, airlineA, numberA) || !splitFlightName(b, airlineB, numberB)) {
        return false;
    }
    return numberA == numberB;
}

bool AircraftTracker::flightMatches(const TrackedFlight *flight, const QString& name)
{
    if ((flight->m_flight == name) || flight->m_aliases.contains(name)) {
        return true;
    }
    if (sameFlightNumber(flight->m_flight, name)) {
        return true;
    }
    for (const QString& alias : flight->m_aliases)
    {
        if (sameFlightNumber(alias, name)) {
            return true;
        }
    }
    return false;
}

// Record another name for a flight. The ICAO callsign form (3-letter airline
// designator) is preferred as the primary name - it is what the route database
// is keyed on - with the other forms kept as aliases.
void AircraftTracker::addFlightAlias(TrackedAircraft *item, TrackedFlight *flight, const QString& name)
{
    if ((flight->m_flight == name) || flight->m_aliases.contains(name)) {
        return;
    }

    QString airline, number;
    bool nameIsCallsign = splitFlightName(name, airline, number) && (airline.size() == 3);
    bool primaryIsCallsign = splitFlightName(flight->m_flight, airline, number) && (airline.size() == 3);

    if (nameIsCallsign && !primaryIsCallsign)
    {
        flight->m_aliases.append(flight->m_flight);
        flight->m_flight = name;
    }
    else
    {
        flight->m_aliases.append(name);
    }
    m_byKey.insert("F" + name, item);
}

// Fold one flight into another - they are the same flight under different names
void AircraftTracker::mergeFlightInto(TrackedAircraft *item, TrackedFlight *dst, TrackedFlight *src)
{
    addFlightAlias(item, dst, src->m_flight);
    for (const QString& alias : src->m_aliases) {
        addFlightAlias(item, dst, alias);
    }
    if (src->m_firstSeen.isValid() && (!dst->m_firstSeen.isValid() || (src->m_firstSeen < dst->m_firstSeen))) {
        dst->m_firstSeen = src->m_firstSeen;
    }
    if (src->m_lastSeen.isValid() && (src->m_lastSeen > dst->m_lastSeen)) {
        dst->m_lastSeen = src->m_lastSeen;
    }
    if (dst->m_departure.isEmpty()) {
        dst->m_departure = src->m_departure;
    }
    if (dst->m_arrival.isEmpty()) {
        dst->m_arrival = src->m_arrival;
    }
    if (dst->m_route.isEmpty())
    {
        dst->m_route = src->m_route;
        // With the route, not separately: left to itself the time would end up claiming
        // that whichever route dst kept was filed when src's was
        dst->m_routeFiled = src->m_routeFiled;
    }
    QHashIterator<QPair<int, qint64>, int> flightSourceIt(src->m_sources);
    while (flightSourceIt.hasNext())
    {
        flightSourceIt.next();
        dst->m_sources[flightSourceIt.key()] += flightSourceIt.value();
    }
    dst->m_documents.append(src->m_documents);
    src->m_documents.clear();

    // Interleave the profile samples in time order
    if (!src->m_profileTimes.isEmpty())
    {
        QList<qint64> times;
        QList<float> alts, speeds;
        int i = 0, j = 0;
        while ((i < dst->m_profileTimes.size()) || (j < src->m_profileTimes.size()))
        {
            bool takeDst = (j >= src->m_profileTimes.size())
                || ((i < dst->m_profileTimes.size()) && (dst->m_profileTimes[i] <= src->m_profileTimes[j]));
            if (takeDst)
            {
                times.append(dst->m_profileTimes[i]);
                alts.append(dst->m_profileAltFt[i]);
                speeds.append(dst->m_profileSpeedKts[i]);
                i++;
            }
            else
            {
                times.append(src->m_profileTimes[j]);
                alts.append(src->m_profileAltFt[j]);
                speeds.append(src->m_profileSpeedKts[j]);
                j++;
            }
        }
        dst->m_profileTimes = times;
        dst->m_profileAltFt = alts;
        dst->m_profileSpeedKts = speeds;
        // Rebuilt, not appended to, so what is stored for this flight is now wrong
        dst->m_profileSaved = 0;
        m_flightsToRewrite.insert(dst->m_id);
    }

    dst->m_messages += src->m_messages;
    if (!dst->m_out.isValid()) { dst->m_out = src->m_out; }
    if (!dst->m_off.isValid()) { dst->m_off = src->m_off; }
    if (!dst->m_on.isValid())  { dst->m_on  = src->m_on; }
    if (!dst->m_in.isValid())  { dst->m_in  = src->m_in; }

    if (!src->m_track.isEmpty())
    {
        QList<QGeoCoordinate> track;
        QList<QDateTime> times;
        int i = 0, j = 0;
        while ((i < dst->m_track.size()) || (j < src->m_track.size()))
        {
            const bool takeDst = (j >= src->m_track.size())
                || ((i < dst->m_track.size()) && (dst->m_trackTimes[i] <= src->m_trackTimes[j]));
            if (takeDst)
            {
                track.append(dst->m_track[i]);
                times.append(dst->m_trackTimes[i]);
                i++;
            }
            else
            {
                track.append(src->m_track[j]);
                times.append(src->m_trackTimes[j]);
                j++;
            }
        }
        dst->m_track = track;
        dst->m_trackTimes = times;
        dst->m_trackSaved = 0;
        m_flightsToRewrite.insert(dst->m_id);
        item->m_mapTrackChanged = true;
    }

    // The routes move too, or they are lost with src while what was drawn from them
    // stays on the Map with nothing left that knows how to take it off. Where dst has one
    // of its own, src's drawing is removed rather than kept: two flights have become one,
    // and only one route can be current.
    for (auto pair : { qMakePair(&src->m_flightPlan, &dst->m_flightPlan),
                       qMakePair(&src->m_clearance, &dst->m_clearance) })
    {
        TrackedFlight::FiledRoute *from = pair.first;
        TrackedFlight::FiledRoute *to = pair.second;
        // The newer route wins, not the destination's. Which flight is which here is
        // not the order they were flown: reconciling two names for one flight merges the
        // current flight INTO the alias it turns out to be, so keeping the destination's
        // route would systematically keep the older of the two.
        const bool takeFrom = !from->m_waypoints.isEmpty()
            && (to->m_waypoints.isEmpty()
                || (from->m_filed.isValid()
                    && (!to->m_filed.isValid() || (from->m_filed > to->m_filed))));
        if (takeFrom)
        {
            // What the destination had drawn comes off first: two flights have become
            // one, and only one route can be current
            removeDrawnRoute(*to);
            // Taken whole, drawn names included, so a later redraw knows what to replace
            *to = *from;
            if (to->m_pending) {
                m_routesPending.insert(dst->m_id);
            }
        }
        else {
            removeDrawnRoute(*from);
        }
    }
    m_routesPending.remove(src->m_id);

    m_flightsToDelete.insert(src->m_id);
    if (m_watchedFlight == src->m_id) {
        m_watchedFlight = dst->m_id;
    }
    if (item->m_currentFlight == src) {
        item->m_currentFlight = dst;
    }
    m_dirtyFlights.remove(src);
    m_dirtyFlights.insert(dst);
    m_dbDirty = true;
    emit flightsRemoved(QList<quint64>{src->m_id});
    item->m_flights.removeAll(src);
    delete src;
}

// The flight a report belongs to
AircraftTracker::TrackedFlight *AircraftTracker::updateFlight(TrackedAircraft *item, const AircraftReport& report)
{
    TrackedFlight *flight = item->m_currentFlight;

    // An airframe that comes back later on the same callsign is flying it again, not
    // still flying it. Without this the second operation was appended to the first, so
    // one flight held two days of track, two sets of OOOI times and two routes. The
    // archive timeout is the boundary because it is already the answer to "has this
    // aircraft gone away": if it was away long enough to be archived, its return is a
    // new flight.
    if (flight && flight->m_lastSeen.isValid() && report.m_received.isValid())
    {
        const qint64 gap = flight->m_lastSeen.secsTo(report.m_received);
        if (gap > (qint64) m_settings.m_removalMins * 60)
        {
            item->m_departure.clear();
            item->m_arrival.clear();
            item->m_route.clear();
            m_dirtyFlights.insert(flight);
            item->m_currentFlight = nullptr;
            flight = nullptr;
            item->m_mapTrackChanged = true;     // A different flight, a different track
        }
    }

    if (!flight)
    {
        flight = new TrackedFlight();
        flight->m_id = m_nextId++;
        flight->m_aircraft = item;
        flight->m_flight = report.m_flight;
        flight->m_firstSeen = report.m_received;
        item->m_flights.append(flight);
        item->m_currentFlight = flight;
    }
    else if (!report.m_flight.isEmpty() && (flight->m_flight != report.m_flight)
             && !flight->m_aliases.contains(report.m_flight))
    {
        if (flight->m_flight.isEmpty())
        {
            // The identity of the flight already under way became known
            flight->m_flight = report.m_flight;
        }
        else if (flightMatches(flight, report.m_flight))
        {
            // The same flight number in the other form, e.g. BA0031 vs BAW31
            addFlightAlias(item, flight, report.m_flight);
        }
        else
        {
            // Some airlines' callsigns are unrelated to the flight number
            // (e.g. LS0232 vs EXS79HP), so the forms can't be matched by name
            // alone. If the name matches the previous flight and that flight
            // was seen moments ago, the two are the same flight alternating
            // between its ACARS and ADS-B identities - reunite them.
            TrackedFlight *previous = (item->m_flights.size() >= 2)
                ? item->m_flights[item->m_flights.size() - 2] : nullptr;
            // Bounded at both ends, as the flight key match is: a report older than the
            // previous flight's last message is not evidence that the two are the same
            // flight alternating between its identities
            const qint64 sincePrevious = previous && previous->m_lastSeen.isValid()
                ? previous->m_lastSeen.secsTo(report.m_received) : -1;
            if (previous && flightMatches(previous, report.m_flight)
                && previous->m_lastSeen.isValid()
                && (sincePrevious >= 0) && (sincePrevious < 10*60))
            {
                mergeFlightInto(item, previous, flight);
                flight = previous;
            }
            else
            {
                // A new flight by the same airframe: the old flight's route no
                // longer describes the aircraft
                item->m_departure.clear();
                item->m_arrival.clear();
                item->m_route.clear();
                flight = new TrackedFlight();
                flight->m_id = m_nextId++;
                flight->m_aircraft = item;
                flight->m_flight = report.m_flight;
                flight->m_firstSeen = report.m_received;
                item->m_flights.append(flight);
                // The flight it was on is no longer the active one
                if (item->m_currentFlight) {
                    m_dirtyFlights.insert(item->m_currentFlight);
                }
                item->m_currentFlight = flight;
                item->m_mapTrackChanged = true;   // A different flight, a different track
            }
        }
    }

    // The latest report, not the last to arrive - as on the aircraft above. A flight
    // whose last seen time went backwards sorts wrongly and reads as finished.
    if (report.m_received.isValid()
        && (!flight->m_lastSeen.isValid() || (report.m_received > flight->m_lastSeen))) {
        flight->m_lastSeen = report.m_received;
    }

    // An event happens once, so the first report of it is the one to keep - the same
    // event is often re-sent, and a later copy is not a later event
    if ((report.m_oooiEvent != AircraftReport::OooiNone) && report.m_oooiTime.isValid())
    {
        switch (report.m_oooiEvent)
        {
        case AircraftReport::OooiOut: if (!flight->m_out.isValid()) { flight->m_out = report.m_oooiTime; } break;
        case AircraftReport::OooiOff: if (!flight->m_off.isValid()) { flight->m_off = report.m_oooiTime; } break;
        case AircraftReport::OooiOn:  if (!flight->m_on.isValid())  { flight->m_on  = report.m_oooiTime; } break;
        case AircraftReport::OooiIn:  if (!flight->m_in.isValid())  { flight->m_in  = report.m_oooiTime; } break;
        default: break;
        }
    }

    // Flight profile sample, at most one every 5 seconds
    if ((report.m_altitudeValid || report.m_speedValid) && report.m_received.isValid())
    {
        qint64 ms = report.m_received.toMSecsSinceEpoch();
        if (flight->m_profileTimes.isEmpty() || (ms - flight->m_profileTimes.last() >= 5000))
        {
            flight->m_profileTimes.append(ms);
            flight->m_profileAltFt.append(report.m_altitudeValid ? report.m_altitudeFt : std::numeric_limits<float>::quiet_NaN());
            flight->m_profileSpeedKts.append(report.m_speedValid ? report.m_speedKts : std::numeric_limits<float>::quiet_NaN());
        }
    }

    // Route facts a message revealed. They move forward in time only, for the same
    // reason the filed routes below do: a delayed message says where the flight was
    // going earlier, not where it is going now. Without this the Route column could
    // disagree with the route drawn on the Map, which does compare filing times.
    //
    // A fact we do not hold at all is still worth taking from an older message, since
    // the alternative is nothing - but it does not move the clock forward, or the next
    // genuinely newer message would be rejected by it.
    const bool routeIsNewer = !flight->m_routeFiled.isValid() || !report.m_received.isValid()
                           || (report.m_received >= flight->m_routeFiled);
    bool routeFactTaken = false;
    if (!report.m_departure.isEmpty() && (routeIsNewer || flight->m_departure.isEmpty()))
    {
        flight->m_departure = report.m_departure;
        routeFactTaken = true;
    }
    if (!report.m_arrival.isEmpty() && (routeIsNewer || flight->m_arrival.isEmpty()))
    {
        flight->m_arrival = report.m_arrival;
        routeFactTaken = true;
    }
    if (!report.m_route.isEmpty() && (routeIsNewer || flight->m_route.isEmpty()))
    {
        flight->m_route = report.m_route;
        routeFactTaken = true;
    }
    if (routeFactTaken && routeIsNewer && report.m_received.isValid()) {
        flight->m_routeFiled = report.m_received;
    }
    // A flight plan and an oceanic clearance both name a list of waypoints, and both
    // are drawn. Nothing else that fills in m_route does - the route database fills it
    // with the airport pair, which is not a route at all.
    if (!report.m_route.isEmpty())
    {
        // A route filed before the one being held is a record of what the aircraft was
        // cleared onto earlier, not what it is on now. Replacing on arrival rather than
        // on filing time let a delayed message overwrite the current route, and redraw
        // the Map with it.
        auto file = [&report](TrackedFlight::FiledRoute& route)
        {
            if (route.m_filed.isValid() && report.m_received.isValid()
                && (report.m_received < route.m_filed)) {
                return false;
            }
            route.m_waypoints = report.m_route;
            route.m_points = report.m_routeWaypoints;
            route.m_text = report.m_documentText;
            route.m_filed = report.m_received;
            return true;
        };

        if (report.m_documentKind == AircraftReport::FlightPlan)
        {
            if (file(flight->m_flightPlan)) {
                sendRouteToMap(flight, flight->m_flightPlan, "Flight plan");
            }
        }
        else if (report.m_documentKind == AircraftReport::Clearance)
        {
            if (file(flight->m_clearance)) {
                sendRouteToMap(flight, flight->m_clearance, "Oceanic clearance");
            }
        }
    }

    // Enrich from the callsign to route database when no message has revealed the
    // route - the same database the ADS-B demodulator uses
    if (!flight->m_flight.isEmpty() && flight->m_departure.isEmpty() && flight->m_arrival.isEmpty()
        && m_routeInfo && m_routeInfo->contains(flight->m_flight))
    {
        const AircraftRouteInformation *route = m_routeInfo->value(flight->m_flight);
        flight->m_departure = route->m_dep;
        flight->m_arrival = route->m_arr;
        if (!route->m_stops.isEmpty()) {
            flight->m_route = route->m_stops;
        }
    }

    // The aircraft-level route facts drive the Map popups
    if (item->m_departure.isEmpty()) {
        item->m_departure = flight->m_departure;
    }
    if (item->m_arrival.isEmpty()) {
        item->m_arrival = flight->m_arrival;
    }
    if (item->m_route.isEmpty()) {
        item->m_route = flight->m_route;
    }

    m_dirtyFlights.insert(flight);

    return flight;
}

QString AircraftTracker::documentKindName(int kind) const
{
    switch (kind)
    {
    case AircraftReport::AcarsText: return "Text";
    case AircraftReport::PositionReport: return "Position report";
    case AircraftReport::OooiEvent: return "OOOI";
    case AircraftReport::Loadsheet: return "Loadsheet";
    case AircraftReport::FlightPlan: return "Flight plan";
    case AircraftReport::Clearance: return "Clearance";
    case AircraftReport::Cpdlc: return "CPDLC";
    case AircraftReport::Logon: return "Logon";
    case AircraftReport::PerformanceReport: return "Performance";
    default: return "";
    }
}

// Collect notable documents - flight plans, clearances, loadsheets, logons,
// OOOI events - against the flight they belong to. Plain ACARS messages would
// flood the log so they are not collected, and CPDLC goes to the ATC log.
void AircraftTracker::addDocument(TrackedAircraft *item, TrackedFlight *flight, const AircraftReport& report)
{
    switch (report.m_documentKind)
    {
    case AircraftReport::Loadsheet:
    case AircraftReport::FlightPlan:
    case AircraftReport::Clearance:
    case AircraftReport::Logon:
    case AircraftReport::OooiEvent:
        break;
    default:
        return;
    }
    if (report.m_documentText.isEmpty()) {
        return;
    }
    // Retransmissions produce identical documents back to back
    if (!flight->m_documents.isEmpty()
        && (flight->m_documents.last()->m_kind == (int) report.m_documentKind)
        && (flight->m_documents.last()->m_text == report.m_documentText)) {
        return;
    }

    TrackedDocument *doc = new TrackedDocument();
    doc->m_kind = report.m_documentKind;
    doc->m_title = report.m_documentTitle.isEmpty() ? documentKindName(report.m_documentKind) : report.m_documentTitle;
    doc->m_text = report.m_documentText;
    doc->m_received = report.m_received;
    flight->m_documents.append(doc);

    m_pendingDocuments.append(documentEvent(item, flight, doc));
    m_dirtyFlights.insert(flight);
    m_dbDirty = true;
}

// Log of controller-pilot data link messages across all channels. Only the
// concise ATC message text (e.g. "WILCO") is kept, not the full decode.
QString AircraftTracker::weatherKindName(int kind)
{
    switch (kind)
    {
    case AircraftReport::Metar:  return "METAR";
    case AircraftReport::Taf:    return "TAF";
    case AircraftReport::Atis:   return "D-ATIS";
    case AircraftReport::Twip:   return "TWIP";
    case AircraftReport::Notam:  return "NOTAM";
    case AircraftReport::Pirep:  return "PIREP";
    case AircraftReport::Sigmet: return "SIGMET";
    default:                     return "Weather";
    }
}

// Weather reaches us through whichever aircraft asked for it, but it describes an
// airport, so it is logged on its own rather than against that aircraft.
void AircraftTracker::addWeatherReport(TrackedAircraft *item, const AircraftReport& report)
{
    if ((report.m_weatherKind == AircraftReport::WeatherNone)
        || report.m_weatherText.isEmpty()) {
        return;
    }

    WeatherEvent e;
    e.m_received = report.m_received;
    e.m_airport = report.m_weatherAirport;
    e.m_kind = weatherKindName(report.m_weatherKind);
    e.m_text = report.m_weatherText;
    e.m_summary = QString(e.m_text).replace('\n', " ").simplified();
    e.m_from = !item->m_flight.isEmpty() ? item->m_flight : regFor(item);

    // The same METAR is sent to every aircraft that asks, so the identical report
    // arrives again and again. Only a genuinely new one is worth a row.
    //
    // This used to scan the whole log, comparing the full text of every entry against
    // every arrival - and since the log only grows, the cost per report grew with it, on
    // the tracker thread, for exactly the traffic that produces the most repeats.
    const size_t key = qHashMulti(0, e.m_airport, e.m_kind, e.m_text);
    for (auto it = m_weatherIndex.constFind(key);
         (it != m_weatherIndex.constEnd()) && (it.key() == key); ++it)
    {
        // value() rather than at(): should the log ever be cleared without the index,
        // a stale position then simply fails to match and the report is logged again,
        // where at() would be undefined behaviour
        const WeatherEvent seen = m_weatherLog.value(it.value());
        if ((seen.m_airport == e.m_airport) && (seen.m_kind == e.m_kind)
            && (seen.m_text == e.m_text)) {
            return;
        }
    }

    m_weatherIndex.insert(key, m_weatherLog.size());
    m_weatherLog.append(e);
    m_pendingWeather.append(e);
}

void AircraftTracker::addAtcMessage(TrackedAircraft *item, TrackedFlight *flight, const AircraftReport& report)
{
    if ((report.m_documentKind != AircraftReport::Cpdlc) || report.m_atc.isEmpty()) {
        return;
    }

    QString flightName = !flight->m_flight.isEmpty() ? flight->m_flight : regFor(item);

    AtcEvent e;
    e.m_aircraftId = item->m_id;
    e.m_flightId = flight->m_id;
    e.m_received = report.m_received;
    e.m_protocol = documentKindName(report.m_documentKind);
    e.m_uplink = report.m_uplink;
    e.m_from = report.m_uplink ? report.m_station : flightName;
    e.m_to = report.m_uplink ? flightName : report.m_station;
    e.m_message = QString(report.m_atc).replace('\n', "; ");
    e.m_tooltip = report.m_atc;
    e.m_mapName = item->m_mapName.isEmpty() ? mapItemName(item) : item->m_mapName;

    // Retransmissions produce identical messages back to back
    if (!m_atcLog.isEmpty()
        && (m_atcLog.last().m_message == e.m_message)
        && (m_atcLog.last().m_from == e.m_from)
        && (m_atcLog.last().m_to == e.m_to)) {
        return;
    }

    m_atcLog.append(e);
    m_pendingAtc.append(e);
    m_dbDirty = true;
}

// Speech and command notifications for aircraft matching the user's rules.
// Each rule fires at most once per aircraft. Speech is passed to the GUI;
// commands run here, so they work in server mode too.
void AircraftTracker::checkNotifications(TrackedAircraft *item, const AircraftReport& report)
{
    for (int i = 0; i < m_settings.m_notificationSettings.size(); i++)
    {
        if (item->m_notifiedRules.contains(i)) {
            continue;
        }
        const AircraftSettings::NotificationSettings *rule = m_settings.m_notificationSettings[i].data();

        QString match;
        switch (rule->m_matchColumn)
        {
        case AircraftSettings::MATCH_ICAO:
            if (item->m_icao) {
                match = QString("%1").arg(item->m_icao, 6, 16, QChar('0')).toUpper();
            }
            break;
        case AircraftSettings::MATCH_REG:
            match = item->m_registration;
            break;
        case AircraftSettings::MATCH_FLIGHT:
            match = item->m_flight;
            break;
        case AircraftSettings::MATCH_TYPE:
            if (m_aircraftInfo && m_aircraftInfo->contains(item->m_registration)) {
                match = m_aircraftInfo->value(item->m_registration)->m_type;
            }
            break;
        default:
            break;
        }

        if (!match.isEmpty()
            && rule->m_regularExpression.isValid()
            && rule->m_regularExpression.match(match).hasMatch())
        {
            item->m_notifiedRules.insert(i);
            if (!rule->m_speech.isEmpty()) {
                emit speechNotification(subAircraftString(item, report, rule->m_speech));
            }
            if (!rule->m_command.isEmpty()) {
                commandNotification(subAircraftString(item, report, rule->m_command));
            }
        }
    }
}

QString AircraftTracker::subAircraftString(TrackedAircraft *item, const AircraftReport& report, const QString& string) const
{
    QString s = string;
    s = s.replace("${icao}", item->m_icao ? QString("%1").arg(item->m_icao, 6, 16, QChar('0')).toUpper() : QString());
    s = s.replace("${reg}", item->m_registration);
    s = s.replace("${flight}", item->m_flight);
    QString type;
    if (m_aircraftInfo && m_aircraftInfo->contains(item->m_registration)) {
        type = m_aircraftInfo->value(item->m_registration)->m_type;
    }
    s = s.replace("${type}", type);
    s = s.replace("${protocol}", protocolName(report.m_protocol));
    return s;
}

void AircraftTracker::commandNotification(const QString& command)
{
#if QT_CONFIG(process)
    QStringList allArgs = QProcess::splitCommand(command);
    if (allArgs.size() > 0)
    {
        QString program = allArgs[0];
        allArgs.pop_front();
        QProcess::startDetached(program, allArgs);
    }
#else
    qWarning() << "AircraftTracker::commandNotification: QProcess not supported. Can't run: " << command;
#endif
}


// The all time figures outlive the session that set them, so they travel in the settings
// as an opaque blob - a versioned stream rather than a settings key each, so that adding
// a statistic later does not need a new key and cannot collide with one.
//
// The session side is deliberately not written: it is defined by the run it describes.
// Nor is the all time aircraft count, which the database holds - see AircraftStatistics.
//
// Version 1 held the records alone. It still reads, and its records become the all time
// records, which is what they always were. Version 3 added where each record was set;
// records read from an older blob simply have no position, and are not drawn on the Map.
QByteArray AircraftTracker::serializeStatistics(const AircraftStatistics& statistics)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    stream.setVersion(QDataStream::Qt_5_15);

    stream << (quint32) 3;      // format version
    stream << (quint32) AircraftReport::ProtocolCount;

    auto put = [&stream](const AircraftStatistics::Record& r)
    {
        stream << r.m_valid << r.m_value << r.m_aircraft << r.m_when;
        // Added in version 3
        stream << r.m_positionValid << r.m_latitude << r.m_longitude
               << r.m_altitudeValid << r.m_altitudeFt;
    };

    const AircraftStatistics::Scope& all = statistics.m_allTime;
    for (int i = 0; i < AircraftReport::ProtocolCount; i++) {
        put(all.m_maxRange[i]);
    }
    put(all.m_fastest);
    put(all.m_highest);
    stream << all.m_maxConcurrent << all.m_maxConcurrentWhen;

    // Added in version 2
    stream << all.m_totalMessages;
    for (int i = 0; i < AircraftReport::ProtocolCount; i++) {
        stream << all.m_messagesByProtocol[i];
    }
    stream << all.m_seconds << statistics.m_firstStart;
    return data;
}

AircraftStatistics AircraftTracker::deserializeStatistics(const QByteArray& data)
{
    AircraftStatistics statistics;
    if (data.isEmpty()) {
        return statistics;
    }

    QDataStream stream(data);
    stream.setVersion(QDataStream::Qt_5_15);

    quint32 version = 0;
    quint32 protocols = 0;
    stream >> version >> protocols;
    if ((version < 1) || (version > 3) || (stream.status() != QDataStream::Ok)) {
        return statistics;      // Written by something else, or truncated: start clean
    }

    auto get = [&stream, version](AircraftStatistics::Record& r)
    {
        stream >> r.m_valid >> r.m_value >> r.m_aircraft >> r.m_when;
        if (version >= 3)
        {
            stream >> r.m_positionValid >> r.m_latitude >> r.m_longitude
                   >> r.m_altitudeValid >> r.m_altitudeFt;
        }
    };

    AircraftStatistics::Scope& all = statistics.m_allTime;

    // Read what was written even if the protocol list has grown since; records for
    // protocols that did not exist then simply stay unset
    for (quint32 i = 0; i < protocols; i++)
    {
        AircraftStatistics::Record r;
        get(r);
        if (i < (quint32) AircraftReport::ProtocolCount) {
            all.m_maxRange[i] = r;
        }
    }
    get(all.m_fastest);
    get(all.m_highest);
    stream >> all.m_maxConcurrent >> all.m_maxConcurrentWhen;

    if (version >= 2)
    {
        stream >> all.m_totalMessages;
        for (quint32 i = 0; i < protocols; i++)
        {
            quint64 n = 0;
            stream >> n;
            if (i < (quint32) AircraftReport::ProtocolCount) {
                all.m_messagesByProtocol[i] = n;
            }
        }
        stream >> all.m_seconds >> statistics.m_firstStart;
    }

    if (stream.status() != QDataStream::Ok) {
        return AircraftStatistics();
    }
    return statistics;
}


// A 3D model for the map, for aircraft the ADS-B demodulator is not providing one for.
//
// This is the same matcher ADS-B uses - it moved to sdrbase so both can share it - and it
// works from the model name, manufacturer and operator in the OpenSky database, which is
// reached through the registration. So an aircraft heard only on ACARS, HFDL or Aero now
// gets a model too, which is most of the oceanic traffic ADS-B cannot see at all.
//
// Chosen ONCE and kept. The livery is picked at random where there is a choice, so asking
// again on every report would change the aircraft's appearance as it flew.
void AircraftTracker::update3DModel(TrackedAircraft *item)
{
    if (!item->m_model3D.isEmpty()) {
        return;
    }

    const AircraftInformation *info = nullptr;
    if (m_aircraftInfo && !item->m_registration.isEmpty())
    {
        auto it = m_aircraftInfo->find(item->m_registration);
        if (it != m_aircraftInfo->end()) {
            info = it.value();
        }
    }

    QString model;
    float altitudeOffset = 0.0f;
    float labelOffset = 0.0f;
    bool matched = info && Aircraft3DModels::instance().modelFor(
        info, m_settings.m_favourLivery, false, model, altitudeOffset, labelOffset);

    if (!matched)
    {
        // Only a fraction of types have a model of their own, and an aircraft we know
        // nothing about has none either. Rather than leave it as a flat billboard among
        // modelled aircraft, stand in something of about the right shape and size
        matched = Aircraft3DModels::instance().defaultModel(
            info ? info->m_type : QString(), model, altitudeOffset, labelOffset);
    }

    if (matched)
    {
        item->m_model3D = model;
        item->m_modelAltitudeOffset = altitudeOffset;
        item->m_labelAltitudeOffset = labelOffset;
        item->m_modelIsOurs = true;
    }
}

// Where the receiver was for a report that arrived on this device set.
//
// A device can know where it is - a remote SDR reports the server's position - and that
// is the antenna that actually heard the aircraft, so it is preferred over My Position.
// getDevicePosition() leaves its arguments alone and returns false when the device has
// none, which makes the fallback a single call.
//
// It is cached because that helper builds an entire device report and parses it as JSON:
// far too expensive per message, but nothing here moves quickly, so a periodic refresh
// picks up a device that has just learned where it is.
QGeoCoordinate AircraftTracker::receiverPosition(int deviceSetIndex)
{
    const QDateTime now = QDateTime::currentDateTime();
    if (!m_devicePositionsRefreshed.isValid()
        || (m_devicePositionsRefreshed.secsTo(now) > 60))
    {
        m_devicePositions.clear();
        m_devicePositionsRefreshed = now;
    }

    auto cached = m_devicePositions.find(deviceSetIndex);
    if (cached != m_devicePositions.end()) {
        return cached.value();
    }

    float latitude = MainCore::instance()->getSettings().getLatitude();
    float longitude = MainCore::instance()->getSettings().getLongitude();
    float altitude = MainCore::instance()->getSettings().getAltitude();

    if (deviceSetIndex >= 0) {
        ChannelWebAPIUtils::getDevicePosition((unsigned int) deviceSetIndex, latitude, longitude, altitude);
    }

    QGeoCoordinate position(latitude, longitude, altitude);
    m_devicePositions.insert(deviceSetIndex, position);
    return position;
}

// Records, counters and the concurrent aircraft window
void AircraftTracker::updateStatistics(const AircraftReport& report, TrackedAircraft *item,
                                      bool rangeValid, float rangeKm,
                                      bool altitudeValid, float altitudeFt)
{
    m_statisticsDirty = true;

    const QString who = !item->m_registration.isEmpty() ? item->m_registration
                      : (!item->m_flight.isEmpty() ? item->m_flight
                         : QString("%1").arg(item->m_icao, 6, 16, QChar('0')).toUpper());
    const QDateTime when = report.m_received.isValid() ? report.m_received
                                                       : QDateTime::currentDateTime();

    // rangeValid/rangeKm describe the position THIS report carried - see the note where
    // they are worked out. A report that carried none measures nothing, which is what
    // keeps ADS-B distances from being credited to HFDL's text-only reports.
    //
    // The position stored with a record is the one THIS report carried, for the same
    // reason: it is the fix the range was measured from, so it is where the record was
    // set. Exactly 0N 0E is a zeroed field rather than a place, as it is on ingest.
    const bool positionValid = report.m_positionValid
        && ((report.m_latitude != 0.0f) || (report.m_longitude != 0.0f));

    m_statistics.m_session.update(report.m_protocol, rangeValid, rangeKm,
        item->m_speedValid, item->m_speedKts, altitudeValid, altitudeFt, who, when,
        positionValid, report.m_latitude, report.m_longitude);
    m_statistics.m_allTime.update(report.m_protocol, rangeValid, rangeKm,
        item->m_speedValid, item->m_speedKts, altitudeValid, altitudeFt, who, when,
        positionValid, report.m_latitude, report.m_longitude);

    // First time this airframe has been heard in this session - which is not the same as
    // the first time it was tracked, because the tracked list is restored from the
    // database at startup
    if (!m_sessionHeard.contains(item->m_id))
    {
        m_sessionHeard.insert(item->m_id);
        m_statistics.m_session.m_distinctAircraft = m_sessionHeard.size();
    }

    // Counted into the all time figure separately, and only once it can be identified
    // well enough to recognise again on another day
    if (!m_sessionRecorded.contains(item->m_id))
    {
        int countDelta = 0;
        if (recordSeen(item, countDelta) >= 0)
        {
            m_sessionRecorded.insert(item->m_id);
            if (countDelta != 0)
            {
                m_statistics.m_allTime.m_distinctAircraft =
                    std::max(0, m_statistics.m_allTime.m_distinctAircraft + countDelta);
                m_statisticsDirty = true;
            }
        }
    }
}

// The most aircraft heard within any 15 minute window. Counted over what has actually
// been heard recently rather than over the tracked list, which holds aircraft for far
// longer than that and would only ever report the retention period's total.
void AircraftTracker::updateConcurrent(const QDateTime& now)
{
    int concurrent = 0;
    for (const TrackedAircraft *item : m_aircraft)
    {
        if (item->m_lastSeen.isValid()
            && (item->m_lastSeen.secsTo(now) <= AircraftStatistics::ConcurrentWindowMins * 60)) {
            concurrent++;
        }
    }
    for (AircraftStatistics::Scope *scope : { &m_statistics.m_session, &m_statistics.m_allTime })
    {
        if (concurrent > scope->m_maxConcurrent)
        {
            scope->m_maxConcurrent = concurrent;
            scope->m_maxConcurrentWhen = now;
            m_statisticsDirty = true;
        }
    }
}

// Everything a rule does, in the order the rules are held. Two lists with the same
// signature address the same aircraft in the same way, whatever else has changed in the
// settings, so the record of which have already fired still means what it says.
QString AircraftTracker::notificationSignature(const AircraftSettings& settings)
{
    QStringList parts;
    for (const auto& rule : settings.m_notificationSettings)
    {
        if (rule) {
            parts.append(QString("%1\x1f%2\x1f%3\x1f%4").arg(rule->m_matchColumn)
                .arg(rule->m_regExp).arg(rule->m_speech).arg(rule->m_command));
        }
    }
    return parts.join(QChar(0x1e));
}

QString AircraftTracker::protocolName(int protocol) const
{
    switch (protocol)
    {
    case AircraftReport::ADSB: return "ADS-B";
    case AircraftReport::ACARS: return "ACARS";
    case AircraftReport::VDL2: return "VDL2";
    case AircraftReport::HFDL: return "HFDL";
    case AircraftReport::AERO: return "Aero";
    default: return "?";
    }
}

// E.g: {"ADS-B", "ACARS 131.725", "HFDL 5.720"}. ADS-B has no frequency - it is
// always 1090 MHz.
QStringList AircraftTracker::sourcesList(const QHash<QPair<int, qint64>, int>& sources) const
{
    QStringList parts;
    QList<QPair<int, qint64>> keys = sources.keys();
    std::sort(keys.begin(), keys.end());
    for (const auto& key : keys)
    {
        QString part = protocolName(key.first);
        if ((key.second != 0) && (key.first != (int) AircraftReport::ADSB)) {
            part += QString(" %1").arg(key.second / 1e6, 0, 'f', 3);
        }
        if (!parts.contains(part)) {
            parts.append(part);
        }
    }
    return parts;
}

QString AircraftTracker::sourcesText(const QHash<QPair<int, qint64>, int>& sources) const
{
    return sourcesList(sources).join("; ");
}

// Name on the Map: registration when known, otherwise flight, otherwise the ICAO
// address in hex, so successive reports update one item
// How a flight is written on the Map: either as flown (BAW123) or as ATC says it
// (SPEEDBIRD 123), which is what is heard on the radio
QString AircraftTracker::flightLabel(const QString& flight) const
{
    if (!m_settings.m_atcCallsigns || (flight.size() < 3)) {
        return flight;
    }

    const Airline *airline = Airline::getByICAO(flight.left(3));

    if (airline && !airline->m_callsign.isEmpty()) {
        return QString("%1 %2").arg(airline->m_callsign, flight.mid(3));
    }
    return flight;
}

QString AircraftTracker::mapItemName(const TrackedAircraft *item) const
{
    if (!item->m_registration.isEmpty()) {
        return item->m_registration;
    }
    if (!item->m_flight.isEmpty()) {
        return item->m_flight;
    }
    if (item->m_icao) {
        return QString("%1").arg(item->m_icao, 6, 16, QChar('0')).toUpper();
    }
    return QString();
}

// ATC style Map label, like the ADS-B demodulator's: identity, route, flight
// level, then speed and type
QString AircraftTracker::atcLabel(const TrackedAircraft *item) const
{
    QStringList lines;
    lines.append(!item->m_flight.isEmpty() ? flightLabel(item->m_flight) : mapItemName(item));
    if (!item->m_departure.isEmpty() && !item->m_arrival.isEmpty()) {
        lines.append(QString("%1-%2").arg(item->m_departure).arg(item->m_arrival));
    }
    if (item->m_altitudeValid && item->m_positionValid)
    {
        QChar c = item->m_altitudeFt >= 6000.0f ? 'F' : 'A';
        lines.append(QString("%1%2").arg(c).arg((int) (item->m_altitudeFt / 100.0f)));
    }
    QStringList row;
    if (item->m_speedValid) {
        row.append(QString("G%1").arg((int) item->m_speedKts));
    }
    if (m_aircraftInfo && m_aircraftInfo->contains(item->m_registration))
    {
        const QString& type = m_aircraftInfo->value(item->m_registration)->m_type;
        if (!type.isEmpty()) {
            row.append(type);
        }
    }
    if (!row.isEmpty()) {
        lines.append(row.join(" "));
    }
    return lines.join("<br>");
}

// Map icon by aircraft type, reusing the ADS-B demodulator's icon set (its qrc
// resources are process wide)
QString AircraftTracker::aircraftImage(const TrackedAircraft *item) const
{
    return aircraftImage(item->m_registration, item->m_model3D);
}

QString AircraftTracker::aircraftImage(const QString& registration, const QString& model3D) const
{
    static const QStringList fourEngineTypes = {
        "A388", "B741", "B742", "B743", "B744", "B748", "B74R", "B74S", "BLCF",
        "A342", "A343", "A345", "A346", "C17", "K35R"};
    static const QStringList heavyTwinTypes = {
        "B772", "B773", "B77L", "B77W", "B788", "B789", "B78X",
        "A332", "A333", "A338", "A339", "A359", "A35K",
        "B762", "B763", "B764", "MD11", "DC10"};
    static const QStringList smallTypes = {
        "GLF4", "GLF5", "GLF6", "GLEX", "GL5T", "CL60", "C25A", "C25B", "C25C",
        "C525", "C550", "C560", "C56X", "C680", "C68A", "C750", "E50P", "E55P",
        "E545", "E550", "F2TH", "F900", "FA7X", "FA8X", "LJ35", "LJ45", "LJ60",
        "PC24", "PRM1", "HDJT"};

    // The livery icons are named after the 3D models they were rendered from
    // (e.g. BB_Airbus_png/A320/A320_BAW.gltf -> A320_BAW.png), so an aircraft's icon
    // follows its matched model's basename. Like the plain icons, they are the ADS-B
    // demodulator's resources, so may not exist in every build
    if (m_settings.m_useLiveryIcons && !model3D.isEmpty())
    {
        QString image = QString("liveries/%1.png").arg(QFileInfo(model3D).completeBaseName());
        if (QFile::exists(":/map/" + image)) {
            return image;
        }
    }

    QString name = "aircraft_large.png";
    if (m_aircraftInfo && m_aircraftInfo->contains(registration))
    {
        const QString& type = m_aircraftInfo->value(registration)->m_type;
        if (fourEngineTypes.contains(type)) {
            name = "aircraft_heavy_4engine.png";
        } else if (heavyTwinTypes.contains(type)) {
            name = "aircraft_heavy_2engine.png";
        } else if (smallTypes.contains(type)) {
            name = "aircraft_small.png";
        }
    }
    // No model match - use a white version of the plain icon, so the style stays
    // consistent with the livery icons rather than dropping back to black
    if (m_settings.m_useLiveryIcons && QFile::exists(":/map/liveries/" + name)) {
        return "liveries/" + name;
    }
    return name;
}

// A position is timestamped when it was measured, an altitude when IT was measured,
// and on ADS-B those are different messages - a Mode S altitude reply carries no
// position. Pairing a fix with whatever altitude arrived most recently therefore states
// a height the aircraft had at some other moment. With the two most recent altitude
// observations bracketing the fix, the right value is a linear interpolation between
// them; outside that, the nearest is the best available and is used unchanged.
float AircraftTracker::altitudeAt(const TrackedAircraft *item, const QDateTime& when, float fallbackFt)
{
    if (!when.isValid() || !item->m_altitudeDateTime.isValid()
        || !item->m_prevAltitudeDateTime.isValid())
    {
        return fallbackFt;      // Nothing timestamped to work from
    }
    const qint64 span = item->m_prevAltitudeDateTime.msecsTo(item->m_altitudeDateTime);
    const qint64 offset = item->m_prevAltitudeDateTime.msecsTo(when);
    if ((span <= 0) || (offset < 0) || (offset > span)) {
        return fallbackFt;      // Not between the two, so nothing to interpolate
    }
    const float t = (float) offset / (float) span;
    return item->m_prevAltitudeFt + t * (item->m_altitudeFt - item->m_prevAltitudeFt);
}

// How long a position stays believable, as a CZML availability interval. Without one an
// entity is available for ever, and on the 3D map that means its model and every texture
// the model carries stay resident until something explicitly deletes it - which was
// enough to exhaust GPU texture memory and lose the WebGL context on a busy sky.
//
// The window has to match how often the protocol that gave us the position actually
// reports. ADS-B is about once a second, so the demodulator's 61 seconds is right for
// it; a position that came from ACARS, VDL-2 or HFDL can be the only one for many
// minutes, and expiring it that fast would make the aircraft flicker.
// How long the position we hold stays believable. ADS-B reports about once a second,
// so a position a minute old means we have stopped hearing the aircraft; a position
// from ACARS, VDL-2 or HFDL may be the only one for many minutes and expiring it as
// fast would leave the aircraft off the map almost always.
int AircraftTracker::positionTimeoutSecs(const TrackedAircraft *item) const
{
    const int mins = (item->m_positionRank == PositionAdsb) ? m_settings.m_adsbPositionMins
                                                            : m_settings.m_acarsPositionMins;
    // The extra second is the demodulator's, for CesiumGS/cesium#12426
    return (mins * 60) + 1;
}

bool AircraftTracker::positionStale(const TrackedAircraft *item, const QDateTime& now) const
{
    return item->m_positionDateTime.isValid()
        && (item->m_positionDateTime.secsTo(now) > positionTimeoutSecs(item));
}

QDateTime AircraftTracker::mapAvailableUntil(const TrackedAircraft *item) const
{
    if (!item->m_positionDateTime.isValid()) {
        return QDateTime();
    }
    return item->m_positionDateTime.addSecs(positionTimeoutSecs(item));
}

// The whole collated track, so it survives gaps between protocols. Each
// coordinate must carry its time - the Map dereferences it unconditionally.
// Selecting one of an aircraft's past flights draws that flight's track instead of the
// one it is on now. The icon is left where the aircraft actually is - the track is
// history, the aircraft is not
const AircraftTracker::TrackedFlight *AircraftTracker::trackFlight(const TrackedAircraft *item) const
{
    if (m_watchedFlight != 0)
    {
        for (const TrackedFlight *flight : item->m_flights)
        {
            if (flight->m_id == m_watchedFlight) {
                return flight;
            }
        }
    }
    return item->m_currentFlight;
}

QList<SWGSDRangel::SWGMapCoordinate *> *AircraftTracker::buildTrack(const TrackedFlight *flight) const
{
    QList<SWGSDRangel::SWGMapCoordinate *> *track = new QList<SWGSDRangel::SWGMapCoordinate *>();
    if (!flight) {
        return track;   // An empty list, not null: null means "unchanged" to the Map
    }
    for (int i = 0; i < flight->m_track.size(); i++)
    {
        SWGSDRangel::SWGMapCoordinate *coord = new SWGSDRangel::SWGMapCoordinate();
        coord->setLatitude(flight->m_track[i].latitude());
        coord->setLongitude(flight->m_track[i].longitude());
        coord->setAltitude(flight->m_track[i].altitude());
        // WithMs, not Qt::ISODate: that is whole seconds, and two positions in the
        // same second then reach the Map with identical times - which the Map's
        // Whittaker-Eilers smoother divides by, giving inf and then nan
        coord->setDateTime(new QString(flight->m_trackTimes[i].toString(Qt::ISODateWithMs)));
        track->append(coord);
    }
    return track;
}

void AircraftTracker::sendToMap(TrackedAircraft *item)
{
    // The Map shows what we are currently hearing. Archived aircraft are taken off it
    // by setActive(), and must not be put back by any other path - nor may an aircraft
    // whose position has gone stale, which keeps sending messages that carry no new one
    if (!item->m_active || positionStale(item, QDateTime::currentDateTime())) {
        return;
    }

    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_feature, "mapitems", mapPipes);

    if (mapPipes.size() == 0) {
        return;
    }

    // When the name upgrades (a logon revealed the airframe), the old item is
    // removed and the full track continues under the new name
    QString name = mapItemName(item);
    if (name.isEmpty()) {
        return;
    }
    if (!item->m_mapName.isEmpty() && (item->m_mapName != name)) {
        removeFromMap(item->m_mapName);
    }
    // Rebuilding the whole track for every report is expensive, so it is only
    // sent when it has changed; the Map keeps its stored track otherwise
    bool sendTrack = item->m_mapTrackChanged || (item->m_mapName != name);
    item->m_mapName = name;

    const AircraftInformation *info =
        (m_aircraftInfo && m_aircraftInfo->contains(item->m_registration))
            ? m_aircraftInfo->value(item->m_registration) : nullptr;

    QStringList text;
    // The same picture header the ADS-B demodulator puts on its popups - a side view of
    // the type in the operator's livery, or failing that the airline's logo, beside the
    // country flag. An aircraft heard only on ACARS then reads the same as one heard on
    // ADS-B, which is the point: the popup should not say which protocol found it.
    if (info)
    {
        const QString sideview = AircraftInformation::resourcePathToURL(
            AircraftInformation::getSideviewIconPath(info->m_registration,
                                                     info->m_operatorICAO, info->m_type));
        const QString airline = AircraftInformation::resourcePathToURL(
            AircraftInformation::getAirlineIconPath(info->m_operatorICAO));
        const QString country = info->getFlag();
        const QString flag = country.isEmpty()
            ? QString() : AircraftInformation::getFlagIconURL(country);

        if (!flag.isEmpty() && !sideview.isEmpty()) {
            text.append(QString("<table width=100%><tr><td><img src=%1 width=85 height=20>"
                                "<td><img src=%2 align=right></table>").arg(sideview).arg(flag));
        } else if (!flag.isEmpty() && !airline.isEmpty()) {
            text.append(QString("<table width=100%><tr><td><img src=%1>"
                                "<td><img src=%2 align=right></table>").arg(airline).arg(flag));
        } else if (!flag.isEmpty()) {
            text.append(QString("<img src=%1>").arg(flag));
        } else if (!sideview.isEmpty()) {
            text.append(QString("<img src=%1>").arg(sideview));
        } else if (!airline.isEmpty()) {
            text.append(QString("<img src=%1>").arg(airline));
        }
    }
    if (!item->m_registration.isEmpty()) {
        text.append(QString("Reg: %1").arg(item->m_registration));
    }
    if (!item->m_flight.isEmpty()) {
        text.append(QString("Flight: %1").arg(item->m_flight));
    }
    if (info)
    {
        QString type = info->m_model.isEmpty() ? info->m_type
            : (info->m_type.isEmpty() ? info->m_model
                : QString("%1 (%2)").arg(info->m_model).arg(info->m_type));
        if (!type.isEmpty()) {
            text.append(QString("Type: %1").arg(type));
        }
    }
    if (!item->m_departure.isEmpty()) {
        text.append(QString("From: %1").arg(item->m_departure));
    }
    if (!item->m_arrival.isEmpty()) {
        text.append(QString("To: %1").arg(item->m_arrival));
    }
    if (!item->m_route.isEmpty()) {
        text.append(QString("Route: %1").arg(item->m_route));
    }
    // The flight's list, not the airframe's: the Map is about what is being heard
    // now, and an airframe's all time list names protocols and frequencies it was heard
    // on weeks ago and is not being heard on at all
    QStringList sources = item->m_currentFlight
        ? sourcesList(item->m_currentFlight->m_sources) : QStringList();
    if (sources.size() == 1)
    {
        text.append(QString("Heard on: %1").arg(sources[0]));
    }
    else if (!sources.isEmpty())
    {
        text.append("Heard on:");
        text.append(sources);
    }
    // The most recent decoded message used to go on the end. An ACARS message is often
    // twenty lines of free text, which made the popup unreadable and pushed everything
    // above it off screen. It is in the feature's own Documents tab, which is where a
    // whole message belongs.

    for (const auto& pipe : mapPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        SWGSDRangel::SWGMapItem *swgMapItem = new SWGSDRangel::SWGMapItem();
        swgMapItem->setName(new QString(name));
        swgMapItem->setLatitude(item->m_latitude);
        swgMapItem->setLongitude(item->m_longitude);
        swgMapItem->setAltitude(item->m_altitudeValid ? Units::feetToMetres(item->m_altitudeFt) : 0.0f);
        swgMapItem->setPositionDateTime(new QString(item->m_positionDateTime.toString(Qt::ISODateWithMs)));
        const QDateTime until = mapAvailableUntil(item);
        if (until.isValid()) {
            swgMapItem->setAvailableUntil(new QString(until.toString(Qt::ISODateWithMs)));
        }
        if (item->m_altitudeDateTime.isValid()) {
            swgMapItem->setAltitudeDateTime(new QString(item->m_altitudeDateTime.toString(Qt::ISODateWithMs)));
        }
        swgMapItem->setImage(new QString(QString("qrc:///map/%1").arg(aircraftImage(item))));
        if (item->m_headingValid) {
            swgMapItem->setImageRotation((int) item->m_heading);
        }
        swgMapItem->setText(new QString(text.join("\n")));
        if (m_settings.m_atcLabels) {
            swgMapItem->setLabel(new QString(atcLabel(item)));
        } else {
            swgMapItem->setLabel(new QString(item->m_flight.isEmpty() ? name : item->m_flight));
        }
        if (sendTrack) {
            swgMapItem->setTrack(buildTrack(trackFlight(item)));
        }

        // Keep using the 3D model a pass-through provided, so the 3D map shows
        // the model rather than falling back to a flat billboard of the icon
        if (!item->m_model3D.isEmpty())
        {
            swgMapItem->setModel(new QString(item->m_model3D));
            // Without these the model sits half underground: 0,0,0 is the middle of the
            // model rather than the bottom of its undercarriage. They were never sent
            // from here, so even a pass-through model was at the wrong height whenever
            // this feature re-sent it.
            swgMapItem->setModelAltitudeOffset(item->m_modelAltitudeOffset);
            swgMapItem->setLabelAltitudeOffset(item->m_labelAltitudeOffset);
            if (item->m_headingValid)
            {
                swgMapItem->setOrientation(1);
                swgMapItem->setHeading(item->m_heading);
            }
        }

        MainCore::MsgMapItem *msg = MainCore::MsgMapItem::create(m_feature, swgMapItem);
        messageQueue->push(msg);
    }
    item->m_mapTrackChanged = false;
}

// Forward a fully-built map item from a demodulator (the ADS-B pass-through, which
// carries the 3D model and full aircraft state for the Map's PFD) to the Map,
// renamed to this feature's naming and enriched with the collated track and route
// facts. Takes ownership of the item.
void AircraftTracker::forwardMapItem(TrackedAircraft *item, SWGSDRangel::SWGMapItem *swgMapItem)
{
    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_feature, "mapitems", mapPipes);

    QString name = mapItemName(item);
    if ((mapPipes.size() == 0) || name.isEmpty()
        || positionStale(item, QDateTime::currentDateTime()))
    {
        delete swgMapItem;
        return;
    }
    if (!item->m_mapName.isEmpty() && (item->m_mapName != name)) {
        removeFromMap(item->m_mapName);
    }
    // Rebuilding the whole track for every report is expensive, so it is only
    // sent when it has changed; the Map keeps its stored track otherwise
    bool sendTrack = item->m_mapTrackChanged || (item->m_mapName != name);

    // On the 3D map an entity shows either its model or a flat billboard of the
    // 2D icon. When the pass-through brings a model to an aircraft first drawn
    // without one, the item is removed and re-added so the stale billboard goes.
    QString *model = swgMapItem->getModel();
    bool hasModel = (model != nullptr) && !model->isEmpty();
    if (hasModel)
    {
        if (!item->m_mapModelForm && (item->m_mapName == name))
        {
            removeFromMap(name);
            sendTrack = true;
        }
        item->m_model3D = *model;
        // ... and what goes with it. Without these the item built here on the next quiet
        // period, and the row written to the database, carry a zero offset or the offsets
        // of whichever model this one replaced - so the aircraft sits at the wrong height
        // as soon as the pass-through stops.
        item->m_modelAltitudeOffset = swgMapItem->getModelAltitudeOffset();
        item->m_labelAltitudeOffset = swgMapItem->getLabelAltitudeOffset();
        item->m_modelIsOurs = false;
        item->m_mapModelForm = true;
    }
    item->m_mapName = name;
    item->m_mapResync = false;
    m_mapResyncPending.remove(item);

    if (swgMapItem->getName()) {
        *swgMapItem->getName() = name;
    } else {
        swgMapItem->setName(new QString(name));
    }

    // Both paths describe the same entity, so they must agree on how long it lives
    const QDateTime until = mapAvailableUntil(item);
    if (until.isValid())
    {
        if (swgMapItem->getAvailableUntil()) {
            *swgMapItem->getAvailableUntil() = until.toString(Qt::ISODateWithMs);
        } else {
            swgMapItem->setAvailableUntil(new QString(until.toString(Qt::ISODateWithMs)));
        }
    }

    // Add what other protocols have revealed to the popup
    QStringList extra;
    QString text = swgMapItem->getText() ? *swgMapItem->getText() : QString();
    if (!item->m_departure.isEmpty() && !text.contains(item->m_departure)) {
        extra.append(QString("From: %1").arg(item->m_departure));
    }
    if (!item->m_arrival.isEmpty() && !text.contains(item->m_arrival)) {
        extra.append(QString("To: %1").arg(item->m_arrival));
    }
    if (!item->m_route.isEmpty() && !text.contains(item->m_route)) {
        extra.append(QString("Route: %1").arg(item->m_route));
    }
    // The flight's list, for the same reason as in sendToMap()
    QStringList sources = item->m_currentFlight
        ? sourcesList(item->m_currentFlight->m_sources) : QStringList();
    if (sources.size() == 1)
    {
        extra.append(QString("Heard on: %1").arg(sources[0]));
    }
    else if (!sources.isEmpty())
    {
        extra.append("Heard on:");
        extra.append(sources);
    }
    if (swgMapItem->getText()) {
        swgMapItem->getText()->append("\n" + extra.join("\n"));
    } else {
        swgMapItem->setText(new QString(extra.join("\n")));
    }

    // The label is always ours, not the demodulator's: its own ATC labels
    // setting would otherwise override this feature's on every update
    QString label = m_settings.m_atcLabels ? atcLabel(item)
        : (item->m_flight.isEmpty() ? name : item->m_flight);
    if (swgMapItem->getLabel()) {
        *swgMapItem->getLabel() = label;
    } else {
        swgMapItem->setLabel(new QString(label));
    }

    if (sendTrack)
    {
        swgMapItem->setTrack(buildTrack(trackFlight(item)));
        item->m_mapTrackChanged = false;
    }

    // Each message takes ownership of its item, so additional Map features get a
    // copy
    QList<SWGSDRangel::SWGMapItem *> items;
    items.append(swgMapItem);
    for (int i = 1; i < mapPipes.size(); i++)
    {
        SWGSDRangel::SWGMapItem *copy = new SWGSDRangel::SWGMapItem();
        QJsonObject *json = swgMapItem->asJsonObject();
        copy->fromJsonObject(*json);
        delete json;
        items.append(copy);
    }
    for (int i = 0; i < mapPipes.size(); i++)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(mapPipes[i]->m_element);
        MainCore::MsgMapItem *msg = MainCore::MsgMapItem::create(m_feature, items[i]);
        messageQueue->push(msg);
    }
}

// The furthest an aircraft has ever been heard, drawn at the position it was in when
// it set the record. Unlike everything else this feature puts on the Map this is not an
// aircraft that is flying now - it marks a place - so it is a fixed position with no
// track and no expiry, and it stays there until the record is beaten, the records are
// reset, or the option is turned off.
//
// Records set before version 3 of the statistics blob have no position stored, so the
// best record that CAN be drawn is used rather than the best overall - otherwise
// upgrading would leave the option doing nothing until the old record was beaten.
const char *AircraftTracker::RecordMapItemName = "Max range record";

void AircraftTracker::updateRecordMapItem()
{
    const AircraftStatistics::Record *best = nullptr;
    int bestProtocol = -1;

    if (m_settings.m_displayMaxRangeOnMap)
    {
        // The record, not the furthest record that happens to be placeable. Skipping the
        // unplaceable ones put a 301 km ACARS marker on the Map while the statistics
        // table said the record was 7281 km on HFDL - two different answers to the same
        // question, which is worse than no marker at all.
        for (int i = 0; i < AircraftReport::ProtocolCount; i++)
        {
            const AircraftStatistics::Record& r = m_statistics.m_allTime.m_maxRange[i];
            if (r.m_valid && (!best || (r.m_value > best->m_value)))
            {
                best = &r;
                bestProtocol = i;
            }
        }
    }

    // A record set before the position was stored alongside it has nowhere to be drawn.
    // Nothing is drawn until it is beaten, rather than something that contradicts the
    // table - said once, since this is checked several times a second.
    if (best && !best->m_positionValid)
    {
        if (!m_recordUnplaceableWarned)
        {
            qDebug() << "AircraftTracker: the maximum range record" << best->m_value
                     << "km, set by" << best->m_aircraft
                     << "on" << protocolName(bestProtocol)
                     << "- was set before positions were kept with records, so it cannot"
                     << "be drawn on the Map. It will appear once the record is beaten.";
            m_recordUnplaceableWarned = true;
        }
        best = nullptr;
    }
    else if (best) {
        m_recordUnplaceableWarned = false;
    }

    if (!best)
    {
        if (m_recordOnMap)
        {
            removeFromMap(RecordMapItemName);
            m_recordOnMap = false;
            m_recordMapKey.clear();
        }
        return;
    }

    // flush() runs three times a second, and the record changes perhaps twice a day
    const QString key = QString("%1|%2|%3|%4|%5")
        .arg(best->m_aircraft)
        .arg(best->m_value, 0, 'f', 3)
        .arg(best->m_latitude, 0, 'f', 5)
        .arg(best->m_longitude, 0, 'f', 5)
        .arg(bestProtocol);
    if (m_recordOnMap && (key == m_recordMapKey)) {
        return;
    }

    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_feature, "mapitems", mapPipes);
    if (mapPipes.isEmpty()) {
        return;         // No Map open: try again when there is one
    }
    m_recordMapKey = key;
    m_recordOnMap = true;

    // The record names the aircraft as its registration where it had one, otherwise its
    // flight number - both of which the key map is built on, so the airframe can be found
    // again where it is still tracked and its own icon used, livery and all. Where it is
    // not, the type still comes from the registration, and the icon is the plain one in
    // white rather than the black it used to be - which is the same fallback a tracked
    // aircraft with no model match gets.
    const TrackedAircraft *holder = m_byKey.value(regKey(best->m_aircraft), nullptr);
    if (!holder) {
        holder = m_byKey.value(flightKey(best->m_aircraft), nullptr);
    }
    const QString recordImage = holder ? aircraftImage(holder)
                                       : aircraftImage(best->m_aircraft, QString());

    QStringList text;
    text.append(QString("<b>Maximum range record</b>"));
    text.append(QString("Aircraft: %1").arg(best->m_aircraft));
    text.append(QString("Range: %1 km").arg(best->m_value, 0, 'f', 1));
    text.append(QString("Protocol: %1").arg(protocolName(bestProtocol)));
    if (best->m_altitudeValid) {
        text.append(QString("Altitude: %1 ft").arg((int) std::round(best->m_altitudeFt)));
    }
    if (best->m_when.isValid()) {
        text.append(QString("When: %1").arg(best->m_when.toString("yyyy-MM-dd hh:mm:ss")));
    }

    for (const auto& pipe : mapPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        SWGSDRangel::SWGMapItem *swgMapItem = new SWGSDRangel::SWGMapItem();
        swgMapItem->setName(new QString(RecordMapItemName));
        swgMapItem->setLatitude(best->m_latitude);
        swgMapItem->setLongitude(best->m_longitude);
        swgMapItem->setAltitude(best->m_altitudeValid
                                ? Units::feetToMetres(best->m_altitudeFt) : 0.0f);
        swgMapItem->setFixedPosition(1);
        swgMapItem->setImage(new QString(QString("qrc:///map/%1").arg(recordImage)));
        swgMapItem->setLabel(new QString(QString("%1 %2 km")
            .arg(best->m_aircraft).arg(best->m_value, 0, 'f', 0)));
        swgMapItem->setText(new QString(text.join("\n")));
        MainCore::MsgMapItem *msg = MainCore::MsgMapItem::create(m_feature, swgMapItem);
        messageQueue->push(msg);
    }
}

// Place the points of a filed route, dropping any that cannot be placed.
//
// A position the message itself gave is used in preference to anything else: a route
// insert places most of its fixes, which settles which of two identically named fixes
// was meant and covers fixes no database here holds. Failing that the name is looked up
// - airports are four characters, waypoints five, VOR and NDB idents three, and an
// oceanic position seven (5530N, i.e. 55 North 030 West), the same rules the ACARS
// demodulator applied to an oceanic clearance, since a route is written the same way.
//
// EVERY coordinate is given an altitude. QGeoCoordinate's two argument constructor
// leaves altitude as qQNaN() while still reporting isValid(), and Qt writes a non-finite
// double into the Map's CZML as null, which Cesium rejects outright - taking down the
// whole 3D map rather than misplacing one route.
void AircraftTracker::resolveRoute(QList<AircraftReport::RouteWaypoint>& points,
                                   QList<QGeoCoordinate>& coords) const
{
    // A route names the airways between its fixes as well as the fixes: "BPK Q295 PAAVO
    // M604 GIVPO". An airway is a letter or two and then digits, which no fix ident is,
    // so this is what keeps Q295 from being looked up as an airport and drawn as one.
    static const QRegularExpression airwayRe("^[A-Z]{1,2}[0-9]{1,4}$");
    // 5530N is 55 North 030 West
    static const QRegularExpression oceanicRe("^([0-9][0-9])([NS])([0-9]?[0-9][0-9])([EW])$");

    QMutableListIterator<AircraftReport::RouteWaypoint> i(points);
    while (i.hasNext())
    {
        const AircraftReport::RouteWaypoint& point = i.next();
        const QString name = point.m_name;

        if (point.m_positionValid)
        {
            coords.append(QGeoCoordinate(point.m_latitude, point.m_longitude, 0.0));
            continue;
        }

        bool found = false;
        if (!airwayRe.match(name).hasMatch())
        {
            if ((name.size() == 4) && m_airports)
            {
                const AirportInformation *airport = m_airports->value(name, nullptr);
                if (airport)
                {
                    found = true;
                    coords.append(QGeoCoordinate(airport->m_latitude, airport->m_longitude, 0.0));
                }
            }
            else if (name.size() == 5)
            {
                const Waypoint *waypoint = Waypoints::findWayPoint(name);
                if (waypoint)
                {
                    found = true;
                    coords.append(QGeoCoordinate(waypoint->m_latitude, waypoint->m_longitude, 0.0));
                }
            }
            else if ((name.size() == 3) && m_navAids)
            {
                for (int j = 0; j < m_navAids->size(); j++)
                {
                    if (m_navAids->at(j)->m_ident == name)
                    {
                        const NavAid *navAid = m_navAids->at(j);
                        found = true;
                        coords.append(QGeoCoordinate(navAid->m_latitude, navAid->m_longitude, 0.0));
                        break;      // Idents are not globally unique; take the first
                    }
                }
            }
            else if (name.size() == 7)
            {
                const QRegularExpressionMatch match = oceanicRe.match(name);
                if (match.hasMatch())
                {
                    float latitude = match.captured(1).toInt();
                    if (match.captured(2) == "S") {
                        latitude = -latitude;
                    }
                    float longitude = match.captured(3).toInt();
                    if (match.captured(4) == "W") {
                        longitude = -longitude;
                    }
                    found = true;
                    coords.append(QGeoCoordinate(latitude, longitude, 0.0));
                }
            }
        }

        if (!found) {
            i.remove();
        }
    }
}

// A route the aircraft has been given - a filed flight plan or an oceanic clearance -
// drawn on the Map as a line through the waypoints with a marker at each one. This used
// to live in the ACARS demodulator, for clearances only; it is here now so that both
// kinds are drawn the same way, and so that a route revealed by any protocol is drawn once
// however many demodulators heard it.
//
// The kind is part of every item name, so a flight with both a plan and a clearance
// shows both: the Map keys its items on the name, and identical names would have one
// quietly replacing the other.
//
// Not removed when the flight is archived, which is how clearances have always behaved:
// the route is a record of where the aircraft was going, and that stays interesting
// after it has landed.
void AircraftTracker::sendRouteToMap(TrackedFlight *flight, TrackedFlight::FiledRoute& route,
                                     const QString& kind)
{
    if (route.m_waypoints.isEmpty()) {
        route.m_pending = false;
        return;
    }

    // Nothing can be drawn until the flight has a name, since that is what the Map keys
    // its items on - and on most protocols the route arrives before the identity does,
    // an oceanic clearance naming the flight in a message the registration follows.
    // This used to be tested AFTER the route was marked drawn, so a route filed before
    // the identity was known was recorded as drawn and then never sent at all.
    const QString name = flight->m_flight.isEmpty()
        ? (flight->m_aircraft ? flight->m_aircraft->m_registration : QString()) : flight->m_flight;
    // Both of the reasons a route cannot go out yet are things nothing tells the flight
    // about when they change - the Map being opened, the identity arriving - so the
    // route is remembered and retried from the flush timer. Without that, opening the
    // Map after a clearance had been received left the route undrawn until another
    // route-bearing message happened along, which on an oceanic flight can be an hour.
    if (name.isEmpty())
    {
        route.m_pending = true;
        m_routesPending.insert(flight->m_id);
        return;
    }

    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_feature, "mapitems", mapPipes);
    if (mapPipes.isEmpty())
    {
        route.m_pending = true;
        m_routesPending.insert(flight->m_id);
        return;
    }
    route.m_pending = false;

    // The route field is the waypoints alone, so the airports it runs between are added
    // here - which is what makes the drawn line start and finish somewhere recognisable
    QList<AircraftReport::RouteWaypoint> points;
    // Not "name": that is the flight's, just above, and shadowing it here would read
    // as though the route were being built out of it
    auto named = [](const QString& waypoint)
    {
        AircraftReport::RouteWaypoint point;
        point.m_name = waypoint;
        return point;
    };
    if (flight->m_departure.size() == 4) {
        points.append(named(flight->m_departure));
    }
    // The message's own positions where it gave them, its names alone where it did not
    if (route.m_points.isEmpty())
    {
        for (const QString& waypoint : route.m_waypoints.split(" ", Qt::SkipEmptyParts)) {
            points.append(named(waypoint));
        }
    }
    else
    {
        points.append(route.m_points);
    }
    if (flight->m_arrival.size() == 4) {
        points.append(named(flight->m_arrival));
    }

    QList<QGeoCoordinate> coords;
    resolveRoute(points, coords);
    QStringList names;
    for (const AircraftReport::RouteWaypoint& point : points) {
        names.append(point.m_name);
    }
    // One resolvable point is not a route, and drawing a line through it draws nothing.
    // Not pending: only a fuller message can change this, and that calls back in here.
    if (coords.size() < 2)
    {
        // A route that has been re-filed onto waypoints we cannot place is still a
        // different route from the one on the Map, so what is drawn there is now wrong.
        // Returning without saying so left the previous line and its markers up.
        removeDrawnRoute(route);
        return;
    }

    // What would actually be drawn, rather than the waypoint list it came from. Keying
    // on the waypoints alone meant a route was drawn once and then never corrected,
    // where everything else about it improves after the first filing: the departure and
    // arrival airports that bookend the line arrive in a later message, a waypoint that
    // could not be placed at first is placed once the aircraft names it with its
    // position, and the text is fuller once the whole message has been reassembled.
    QStringList signature;
    signature.append(name);
    signature.append(flight->m_departure);
    signature.append(flight->m_arrival);
    signature.append(route.m_text);
    for (int i = 0; i < names.size(); i++)
    {
        signature.append(QString("%1@%2,%3").arg(names[i])
            .arg(coords[i].latitude(), 0, 'f', 5).arg(coords[i].longitude(), 0, 'f', 5));
    }
    const QString drawn = signature.join(QChar(0x1f));
    if (drawn == route.m_drawn) {
        return;
    }

    // The Map keys its items on the name, and both the flight's name and the list of
    // waypoints can change between drawings - a registration gives way to the callsign
    // once it is known, and a re-filed route can be shorter than the one before it. What
    // was drawn last time and will not be drawn this time has to be taken off by name,
    // or the old line stays beside the new one and dropped waypoints stay on the map for
    // the rest of the session.
    const QString lineName = QString("%1 %2").arg(name).arg(kind);
    QStringList waypointNames;
    for (const QString& waypoint : names) {
        waypointNames.append(QString("%1 %2 %3").arg(name).arg(kind).arg(waypoint));
    }
    if (!route.m_drawnLine.isEmpty() && (route.m_drawnLine != lineName)) {
        removeFromMap(route.m_drawnLine);
    }
    for (const QString& previous : route.m_drawnWaypoints)
    {
        if (!waypointNames.contains(previous)) {
            removeFromMap(previous);
        }
    }

    QStringList text;
    text.append(kind);
    text.append(QString("Flight: %1").arg(name));
    if (!flight->m_departure.isEmpty() && !flight->m_arrival.isEmpty()) {
        text.append(QString("Route: %1 to %2").arg(flight->m_departure).arg(flight->m_arrival));
    }
    text.append("Waypoints: " + names.join(" "));
    // The message itself, which is where the clearance number, entry time, flight level
    // and Mach are. The demodulator used to repeat those on every waypoint; one copy on
    // the route says the same thing without covering the map in it.
    if (!route.m_text.isEmpty())
    {
        text.append(QString());
        text.append(route.m_text);
    }

    for (const auto& pipe : mapPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);

        SWGSDRangel::SWGMapItem *swgMapItem = new SWGSDRangel::SWGMapItem();
        swgMapItem->setName(new QString(lineName));
        swgMapItem->setLabel(new QString(name));
        swgMapItem->setText(new QString(text.join("\n")));
        swgMapItem->setLatitude(coords[0].latitude());
        swgMapItem->setLongitude(coords[0].longitude());
        swgMapItem->setAltitude(coords[0].altitude());
        swgMapItem->setImage(new QString("none"));
        swgMapItem->setImageRotation(0);
        swgMapItem->setFixedPosition(true);
        swgMapItem->setAltitudeReference(3);    // 1 - CLAMP_TO_GROUND, 3 - CLIP_TO_GROUND

        QList<SWGSDRangel::SWGMapCoordinate *> *line = new QList<SWGSDRangel::SWGMapCoordinate *>();
        for (const auto& coord : coords)
        {
            SWGSDRangel::SWGMapCoordinate *c = new SWGSDRangel::SWGMapCoordinate();
            c->setLatitude(coord.latitude());
            c->setLongitude(coord.longitude());
            c->setAltitude(coord.altitude());
            line->append(c);
        }
        swgMapItem->setCoordinates(line);
        swgMapItem->setType(3);                 // Line

        MainCore::MsgMapItem *msg = MainCore::MsgMapItem::create(m_feature, swgMapItem);
        messageQueue->push(msg);

        // The waypoints themselves. The Map does not draw its own waypoint layer across
        // the whole globe, and shows it only when zoomed well in, so the points of a
        // route have to be sent explicitly to be visible with the line.
        for (int i = 0; i < names.size(); i++)
        {
            swgMapItem = new SWGSDRangel::SWGMapItem();
            // Not the name the Map uses for the waypoint itself, which would replace it
            swgMapItem->setName(new QString(waypointNames[i]));
            swgMapItem->setLabel(new QString(names[i]));

            QStringList waypointText;
            waypointText.append(QString("%1 waypoint").arg(kind));
            waypointText.append(QString("Flight: %1").arg(name));
            waypointText.append(QString("Waypoint: %1").arg(names[i]));
            swgMapItem->setText(new QString(waypointText.join("\n")));

            swgMapItem->setLatitude(coords[i].latitude());
            swgMapItem->setLongitude(coords[i].longitude());
            swgMapItem->setAltitude(coords[i].altitude());
            swgMapItem->setImage(new QString("waypoint.png"));
            swgMapItem->setImageRotation(0);
            swgMapItem->setFixedPosition(true);
            swgMapItem->setAltitudeReference(3);

            msg = MainCore::MsgMapItem::create(m_feature, swgMapItem);
            messageQueue->push(msg);
        }
    }

    // Only once it has gone out. Marking it before anything had been sent is what let a
    // route be recorded as drawn and then dropped.
    route.m_drawn = drawn;
    route.m_drawnLine = lineName;
    route.m_drawnWaypoints = waypointNames;
    route.m_pending = false;
    if (!flight->m_flightPlan.m_pending && !flight->m_clearance.m_pending) {
        m_routesPending.remove(flight->m_id);
    }
}

// Take a route off the Map, by the names it was drawn under. Only what was actually
// emitted is removed, so this cannot disturb another route that happens to share a name.
void AircraftTracker::removeDrawnRoute(TrackedFlight::FiledRoute& route)
{
    if (route.m_drawnLine.isEmpty() && route.m_drawnWaypoints.isEmpty()) {
        return;
    }
    if (!route.m_drawnLine.isEmpty()) {
        removeFromMap(route.m_drawnLine);
    }
    for (const QString& waypoint : route.m_drawnWaypoints) {
        removeFromMap(waypoint);
    }
    route.m_drawnLine.clear();
    route.m_drawnWaypoints.clear();
    route.m_drawn.clear();
}

// Everything this feature draws is sent once and then only when it changes, which is
// what keeps a busy sky affordable - but it means a Map opened later, or closed and
// opened again, starts from nothing and is only ever told about changes. So when the
// number of Maps listening goes up, everything is offered again: the tracks, the routes
// and the record marker. Aircraft positions need no help, being re-sent as they arrive.
void AircraftTracker::checkMapConsumers()
{
    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_feature, "mapitems", mapPipes);

    QSet<unsigned int> consumers;
    bool arrived = false;
    for (const ObjectPipe *pipe : mapPipes)
    {
        consumers.insert(pipe->m_pipeId);
        if (!m_mapConsumers.contains(pipe->m_pipeId)) {
            arrived = true;
        }
    }
    m_mapConsumers = consumers;
    if (!arrived) {
        return;                         // The same Maps, which have what they need
    }

    for (TrackedAircraft *item : m_aircraft)
    {
        if (item->m_active)
        {
            item->m_mapTrackChanged = true;
            item->m_mapName.clear();    // Draw it again rather than assuming it is there
            // Held until it has actually been sent. A pass-through aircraft whose ADS-B
            // has gone quiet is skipped by flush() for a minute after its last item, and
            // the dirty set is cleared either way - so without this the aircraft would
            // simply never reach the new Map.
            item->m_mapResync = true;
            m_mapResyncPending.insert(item);
            m_dirtyAircraft.insert(item);
        }

        // Routes are drawn for archived aircraft too - they are a record of where the
        // aircraft was going, and are deliberately left up - so unlike the aircraft
        // itself they are re-sent whether or not it is still being heard
        for (TrackedFlight *flight : item->m_flights)
        {
            for (TrackedFlight::FiledRoute *route : { &flight->m_flightPlan, &flight->m_clearance })
            {
                if (!route->m_waypoints.isEmpty())
                {
                    // Not removeDrawnRoute(): the items are still on the Maps that
                    // already had them, and re-sending replaces them by name
                    route->m_drawn.clear();
                    route->m_pending = true;
                    m_routesPending.insert(flight->m_id);
                }
            }
        }
    }

    // Sent again for the same reason, and by the same means: forget what was last drawn
    m_recordMapKey.clear();
    m_recordOnMap = false;
}

// A route held back for want of a Map to draw it on, or a name to draw it under. Neither
// is signalled, so this is polled - but only past the two tests that are cheap, so the
// expensive part, resolving each waypoint against the databases, is reached only when the
// route could actually be delivered.
void AircraftTracker::retryRoutes()
{
    if (m_routesPending.isEmpty()) {
        return;
    }

    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_feature, "mapitems", mapPipes);
    if (mapPipes.isEmpty()) {
        return;
    }

    const QSet<quint64> pending = m_routesPending;
    for (quint64 id : pending)
    {
        TrackedFlight *flight = flightById(id);
        if (!flight)
        {
            m_routesPending.remove(id);     // Deleted, or merged into another flight
            continue;
        }
        sendRouteToMap(flight, flight->m_flightPlan, "Flight plan");
        sendRouteToMap(flight, flight->m_clearance, "Oceanic clearance");
        if (!flight->m_flightPlan.m_pending && !flight->m_clearance.m_pending) {
            m_routesPending.remove(id);
        }
    }
}

void AircraftTracker::removeFromMap(const QString& name)
{
    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_feature, "mapitems", mapPipes);

    for (const auto& pipe : mapPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        SWGSDRangel::SWGMapItem *swgMapItem = new SWGSDRangel::SWGMapItem();
        swgMapItem->setName(new QString(name));
        swgMapItem->setImage(new QString(""));  // An empty image removes the item
        MainCore::MsgMapItem *msg = MainCore::MsgMapItem::create(m_feature, swgMapItem);
        messageQueue->push(msg);
    }
}
