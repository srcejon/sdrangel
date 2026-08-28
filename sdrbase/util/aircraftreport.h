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

#ifndef INCLUDE_UTIL_AIRCRAFTREPORT_H
#define INCLUDE_UTIL_AIRCRAFTREPORT_H

#include <QString>
#include <QDateTime>

namespace SWGSDRangel {
    class SWGMapItem;
}

// One sighting of an aircraft by a demodulator, passed from channels (ADS-B and
// ACARS family demodulators) to the Aircraft feature over the "aircraftreport"
// message pipe. Every field is optional except the source description - a report
// carries whichever identities, position and document the received frame had.
struct AircraftReport
{
    // New protocols go on the END: the Aircraft feature keys a per-source map and a
    // message rate chart on this value
    enum Protocol {
        ADSB,
        ACARS,          // Plain VHF ACARS
        VDL2,
        HFDL,
        AERO,           // Inmarsat Classic Aero (aviation SATCOM)
        ProtocolCount
    };

    enum DocumentKind {
        NoDocument,
        AcarsText,      // A plain ACARS message
        PositionReport,
        OooiEvent,      // Out/off/on/in event
        Loadsheet,
        FlightPlan,
        Clearance,      // Oceanic/departure clearance
        Cpdlc,
        Logon,          // Link management: logons, logoffs
        PerformanceReport
    };

    // Source
    Protocol m_protocol = ACARS;
    qint64 m_frequency = 0;         // Channel centre frequency in Hz, 0 if unknown
    int m_deviceSetIndex = -1;
    int m_channelIndex = -1;

    // Identity - whichever the frame carried
    quint32 m_icao = 0;             // 24 bit ICAO address, 0 if unknown
    QString m_registration;
    QString m_flight;               // Flight number / callsign as transmitted

    // Position and kinematics
    bool m_positionValid = false;
    float m_latitude = 0.0f;
    float m_longitude = 0.0f;
    bool m_altitudeValid = false;
    float m_altitudeFt = 0.0f;
    bool m_headingValid = false;
    float m_heading = 0.0f;         // Degrees
    bool m_speedValid = false;
    float m_speedKts = 0.0f;
    QDateTime m_positionDateTime;   // The report's own time when it differs from reception
    // When the altitude was observed. On ADS-B this is often NOT the position's time:
    // a Mode S altitude reply carries no position, so the two are measured separately.
    // Invalid on bearers that report a single fix, where the position's time serves.
    QDateTime m_altitudeDateTime;

    QDateTime m_received;

    // The OOOI event this report carried, if any. Out of gate, take off, landing and on
    // gate are the four times that describe a flight, and they arrive as separate ACARS
    // messages on labels 13 to 16.
    enum OooiEventKind { OooiNone = 0, OooiOut = 13, OooiOff, OooiOn, OooiIn };
    int m_oooiEvent = OooiNone;
    QDateTime m_oooiTime;       //!< When the event happened, not when it was reported

    // Weather this report carried. Aircraft ask for it and the ground sends it, so it
    // arrives through ACARS like everything else - but it describes an AIRPORT rather
    // than the aircraft that happened to request it, which is why it is kept separately.
    enum WeatherKind {
        WeatherNone = 0,
        Metar,          //!< Observation
        Taf,            //!< Forecast
        Atis,           //!< D-ATIS
        Twip,           //!< Terminal weather information for pilots
        Notam,
        Pirep,          //!< Pilot report
        Sigmet,         //!< Significant meteorological information, and AIRMET
        WeatherOther    //!< Recognised as weather, not as any of the above
    };
    int m_weatherKind = WeatherNone;
    QString m_weatherAirport;   //!< ICAO code the report is about, when it names one
    QString m_weatherText;      //!< The report itself, as sent

    // Decoded content
    DocumentKind m_documentKind = NoDocument;
    QString m_documentTitle;
    QString m_documentText;         // Full multi-line decode
    QString m_label;                // ACARS label or frame type
    bool m_uplink = false;          // true when ground to air
    QString m_atc;                  // Concise ATC message text (e.g. "WILCO") for CPDLC
    QString m_station;              // Ground station / ATC facility, when known

    // Route facts the message revealed (OOOI events, flight plans, clearances)
    QString m_departure;
    QString m_arrival;
    QString m_route;
    // One waypoint of m_route. A route insert gives the position of most of its fixes,
    // and using those beats looking the names up: it places fixes no database here
    // holds, and settles which of two identically named fixes was meant.
    struct RouteWaypoint {
        QString m_name;
        bool m_positionValid = false;
        float m_latitude = 0.0f;
        float m_longitude = 0.0f;
    };
    // Same order and length as the space separated names in m_route, and empty when the
    // route came from somewhere that names waypoints without placing them
    QList<RouteWaypoint> m_routeWaypoints;

    // Phase 2: a fully-built map item from the ADS-B demodulator, passed through
    // so its PFD state reaches the Map losslessly. Ownership transfers with the
    // report; nullptr for reports that do not carry one.
    SWGSDRangel::SWGMapItem *m_mapItem = nullptr;
};

#endif // INCLUDE_UTIL_AIRCRAFTREPORT_H
