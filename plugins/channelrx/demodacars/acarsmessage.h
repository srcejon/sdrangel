///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2022 Jon Beniston, M7RCE                                        //
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

#ifndef INCLUDE_ACARSMESSGAE_H
#define INCLUDE_ACARSMESSGAE_H

#include <QString>
#include <QDate>
#include <QTime>
#include <QMetaType>

#define ASCII_SOH 0x01
#define ASCII_STX 0x02
#define ASCII_ETX 0x03
#define ASCII_ACK 0x06
#define ASCII_NAK 0x15
#define ASCII_ETB 0x17
#define ASCII_DEL 0x7f

// Shortest legal ARINC 618 block: SOH mode addr(7) ack label(2) blockid ETX BCS(2) DEL.
// STX is present only when message text follows.
#define ACARS_MIN_BYTES 17

// Sanity bound on an altitude recovered from a fixed-width position report, used to
// reject a field that has been read at the wrong offset. Airliners sending these do
// not cruise above FL430
#define ACARS_MAX_REPORTED_ALTITUDE_FT 50000

// See ARINC 618
class AcarsMessage {
public:
    QDateTime m_dateTime;   // Date & time message was received
    QString m_mode;
    QString m_address;
    QChar m_ack;
    QString m_label;
    QChar m_blockId;
    QString m_text;
    QString m_originator; // Message originator (1-9,A-Z) - for downlinks
    QString m_messageNumber; // Two digit message number (0-99) - for downlinks
    QString m_blockSequence; // Block sequence for multiblock messages (A-Z)
    QString m_flight; // Flight number - for downlinks
    QString m_subLabel;
    bool m_multiBlock; // Indicates if this message has additional following blocks
    bool m_uplink;  // Indicates if uplink or downlink
    AcarsMessage(const QByteArray &message);
    virtual bool decode(const QString &message) { return true; }
    virtual QString toString();
    // Aircraft position carried in the message, for the table and Map. altitudeFt is
    // -1 when the message has a position but no altitude.
    virtual bool getPosition(float& latitude, float& longitude, int& altitudeFt) const
    {
        (void) latitude;
        (void) longitude;
        (void) altitudeFt;
        return false;
    }
    static AcarsMessage *decode(const QByteArray &message);
protected:
    static AcarsMessage *decode(const QByteArray &message, const QString &label, const QString &text, bool uplink);
    static bool isUplink(QChar blockId);
};

// ARINC622 - Character-Oriented Air Traffic Service (ATS) Applications
// Application data embedded in ACARS text field
class ARINC622 {
public:
    QString m_atcCenter;
    QString m_imi;
    QString m_aircraft;
    QString m_app;
    int m_crc;
    QString m_do219; // IMI == "AT1"
    bool m_valid;
    ARINC622() :
        m_valid(false)
    {}
    bool decode(const QString &message, bool uplink);
    QString toString();
};

class AcarsDataTransceiverAutoTune : public AcarsMessage {
public:
    int m_frequencyHz;
    int m_precision;
    AcarsDataTransceiverAutoTune(const QByteArray &message);
    bool decode(const QString &text) override;

    virtual QString toString() override;
};


class AcarsGroundUTCRequest : public AcarsMessage {
public:
    QDate m_date;
    int m_day;
    QTime m_time;
    AcarsGroundUTCRequest(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsOceanicClearanceResponse : public AcarsMessage {
public:
    QTime m_messageTime;
    QDate m_messageDate;
    QString m_atcCenter;
    int m_clearanceNumber = 0;
    QString m_aircraft;
    QString m_destination;
    QString m_via;
    QString m_routeId;
    QString m_route;
    QString m_entryPoint;
    QTime m_entryTime;
    float m_mach = 0.0f;
    int m_flightLevel = 0;
    QString m_freeText;
    AcarsOceanicClearanceResponse(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsDepartureClearanceResponse : public AcarsMessage {
public:
    QTime m_messageTime;
    QDate m_messageDate;
    QString m_departureAirport;
    int m_clearanceNumber = 0;
    QString m_flight;
    QString m_destination;
    QString m_runway;
    QString m_sid;
    QString m_squawk;
    QString m_departureTimeType;
    QString m_departureTime;
    QString m_atis;
    QString m_freeText;
    AcarsDepartureClearanceResponse(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsFlightSystemsMessage : public AcarsMessage {
public:
    QTime m_messageTime;
    QDate m_messageDate;
    QString m_departureAirport;
    QString m_flight;
    QString m_freeText;
    AcarsFlightSystemsMessage(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsATISReport: public AcarsMessage {
public:
    QString m_airport;
    QString m_indicator;
    QString m_atisId;
    QTime m_time;
    QString m_atisInformation;
    AcarsATISReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsATSFacilitiesNotification : public AcarsMessage {
public:
    QString m_atcCenter;
    QString m_flight;
    QString m_aircraftRegistration;
    QString m_aircraftICAO;
    QTime m_time;
    QString m_mti;  // Message text identifier
    AcarsATSFacilitiesNotification(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsRequestOceanicClearance : public AcarsMessage {
public:
    QString m_avionicsIndicator;
    QString m_aircraft;
    QString m_entryPoint;
    QTime m_entryTime;
    float m_mach = 0.0f;
    int m_flightLevel = 0;
    QString m_freeText;
    AcarsRequestOceanicClearance(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsOceanicClearanceReadback : public AcarsMessage {
public:
    QTime m_messageTime;
    QDate m_messageDate;
    QString m_atcCenter;
    int m_clearanceNumber = 0;
    QString m_aircraft;
    QString m_destination;
    QString m_via;
    QString m_routeId;
    QString m_route;
    QString m_entryPoint;
    QTime m_entryTime;
    float m_mach = 0.0f;
    int m_flightLevel = 0;
    QString m_freeText;
    AcarsOceanicClearanceReadback(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsRequestATISReport : public AcarsMessage {
public:
    QString m_avionicsIndicator;
    QString m_airport;
    QString m_arrivalDepartureIndicator;
    AcarsRequestATISReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsWeatherRequest : public AcarsMessage {
public:
    QStringList m_airports;
    QString m_reportType;       // MET report code ("SA" = METAR) or the WXRQ /TYP value
    QString m_registration;     // WXRQ form only
    QString m_origin;           // WXRQ form only
    QString m_destination;      // WXRQ form only
    AcarsWeatherRequest(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsFlightStatus : public AcarsMessage {
public:
    QString m_departureAirport;
    QString m_arrivalAirport;
    bool m_positionValid = false;
    float m_latitude = 0.0f;
    float m_longitude = 0.0f;
    int m_altitudeFt = -1;          // -1 when not reported
    int m_fuelOnBoardKg = -1;       // Units differ by layout, so held in kg
    int m_fuelUsedKg = -1;
    bool m_temperatureValid = false;
    int m_temperatureC = 0;         // Static air temperature
    int m_windSpeedKts = -1;
    int m_windDirectionDeg = -1;
    int m_headingDeg = -1;          // Compact layout only
    int m_trackDeg = -1;
    int m_speedKts = -1;
    QTime m_eta;
    QTime m_time;
    QString m_additionalData;       // Undecoded middle fields, when present
    AcarsFlightStatus(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
    bool getPosition(float& latitude, float& longitude, int& altitudeFt) const override;
};

// Position reports in the related airline formats used on labels 10, 12, 16, 22, 24 and 44
class AcarsPositionReport : public AcarsMessage {
public:
    bool m_positionValid = false;
    float m_latitude = 0.0f;
    float m_longitude = 0.0f;
    int m_altitudeFt = -1;
    QTime m_time;
    QTime m_eta;
    QString m_reportFlight;         // Flight number within the report (label 24)
    QString m_reportType;           // POS/ETA/IN/OFF/ON (label 44)
    QString m_departureAirport;     // Label 44
    QString m_arrivalAirport;
    int m_month = -1;
    int m_day = -1;
    float m_fuelTons = -1.0f;
    AcarsPositionReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
    bool getPosition(float& latitude, float& longitude, int& altitudeFt) const override;
};

// Airline operational reports on label 80 (3X01 POSRPT / OPNORM / INRANG formats)
class AcarsOpsReport : public AcarsMessage {
public:
    QString m_reportType;           // POSRPT, OPNORM or INRANG
    QString m_reportFlight;
    int m_day = -1;
    QString m_departureAirport;
    QString m_arrivalAirport;
    QString m_tail;
    bool m_positionValid = false;
    float m_latitude = 0.0f;
    float m_longitude = 0.0f;
    int m_altitudeFt = -1;
    QString m_nextWaypoint;
    int m_headingDeg = -1;
    int m_machX1000 = -1;
    int m_speedKts = -1;            // TAS or SPD
    bool m_temperatureValid = false;
    int m_temperatureC = 0;
    QString m_fuelOnBoard;          // As reported; units vary by airline
    QTime m_time;                   // From /UTC
    QTime m_eta;
    AcarsOpsReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
    bool getPosition(float& latitude, float& longitude, int& altitudeFt) const override;
};

// OOOI event reports on labels 13 to 18 (out of gate, takeoff, landing, in gate and
// the post reports), in the /13 OUT EVENT / KSFO KEWR ... format
class AcarsEventReport : public AcarsMessage {
public:
    int m_event = 0;                // The label as an int, 13 to 18
    QString m_eventDescription;     // From the message, e.g. "OUT EVENT"
    QString m_departureAirport;
    QString m_arrivalAirport;
    int m_day = -1;
    QTime m_eventTime;
    int m_fuelOnBoard = -1;         // In 100kg
    bool m_positionValid = false;
    float m_latitude = 0.0f;
    float m_longitude = 0.0f;
    AcarsEventReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
    bool getPosition(float& latitude, float& longitude, int& altitudeFt) const override;
};

class AcarsATCCommunications : public AcarsMessage {
public:
    ARINC622 m_622;
    AcarsATCCommunications(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsMessageFromSubsystem : public AcarsMessage {
public:
    ARINC622 m_622;
    // FMS reports (e.g. #M1BPOS position, #M1BFPN flight plan) when the text is not
    // an ARINC 622 ATS message
    QString m_fmsSublabel;
    bool m_positionValid = false;
    float m_latitude = 0.0f;
    float m_longitude = 0.0f;
    int m_altitudeFt = -1;
    QString m_lastWaypoint;
    QTime m_timeOver;               // Time over the last waypoint
    QString m_nextWaypoint;
    QTime m_etaNext;                // ETA at the next waypoint
    QString m_nextPlus1Waypoint;
    bool m_temperatureValid = false;
    int m_temperatureC = 0;
    QString m_posAdditionalData;
    QString m_fpnDeparture;
    QString m_fpnArrival;
    QString m_fpnRoute;         //!< Waypoints and airways, as a flight plan is written
    QString m_fpnRunway;        //!< Departure runway, with the arrival's in brackets
    QString m_fpnSid;           //!< Departure procedure
    QString m_fpnStar;          //!< Arrival procedure
    QString m_fpnApproach;      //!< Approach and its transition
    QString m_fpnAlternate;     //!< Alternate destination, from the /RA: block
    //!< One waypoint of m_fpnRoute. Deliberately not a QGeoCoordinate: this header is
    //!< compiled into the test harness, which does not link Qt Positioning.
    struct FpnWaypoint {
        QString m_name;
        bool m_hasPosition = false;
        float m_latitude = 0.0f;
        float m_longitude = 0.0f;
    };
    //!< Same order and length as the names in m_fpnRoute. A route insert gives the
    //!< position of most of its fixes, which places fixes no database here holds and
    //!< settles which of two identically named fixes was meant.
    QList<FpnWaypoint> m_fpnWaypoints;
    AcarsMessageFromSubsystem(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
    bool getPosition(float& latitude, float& longitude, int& altitudeFt) const override;
};

class AcarsOutReport : public AcarsMessage {
public:
    QString m_departure;
    QString m_destination;
    int m_outTimeHours;
    int m_outTimeMinutes;
    int m_fuel;
    QString m_boardedFuel;
    QString m_freeText;
    AcarsOutReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsOffReport : public AcarsMessage {
public:
    QString m_departure;
    QString m_destination;
    int m_offTimeHours;
    int m_offTimeMinutes;
    QString m_freeText;
    AcarsOffReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsOnReport : public AcarsMessage {
public:
    QString m_departure;
    QString m_destination;
    int m_onTimeHours;
    int m_onTimeMinutes;
    QString m_freeText;
    AcarsOnReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsInReport : public AcarsMessage {
public:
    QString m_departure;
    QString m_destination;
    int m_inTimeHours;
    int m_inTimeMinutes;
    int m_fuel;
    QChar m_captainFirstOfficerId;
    QChar m_landingCategory;
    QString m_freeText;
    AcarsInReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsOutReturnInReport : public AcarsMessage {
public:
    QString m_departure;
    QString m_destination;
    int m_outTimeHours;
    int m_outTimeMinutes;
    int m_returnInTimeHours;
    int m_returnInTimeMinutes;
    int m_fuel;
    QString m_freeText;
    AcarsOutReturnInReport(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

class AcarsSquitter : public AcarsMessage {
public:
    QString m_version;
    QString m_serviceProvider;
    QString m_iataStationId;
    QString m_icaoStationId;
    QString m_stationNumber;
    QString m_latitude;
    QString m_longitude;
    QStringList m_alternateService;
    QStringList m_alternateFrequency;
    QString m_freeText;
    AcarsSquitter(const QByteArray &message);
    bool decode(const QString &text) override;
    virtual QString toString() override;
};

Q_DECLARE_METATYPE(AcarsMessage *)

#endif // INCLUDE_ACARSMESSGAE_H
