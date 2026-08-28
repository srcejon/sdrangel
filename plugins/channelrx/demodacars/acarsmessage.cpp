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

#include "acarsmessage.h"

#include <cmath>

#include <QDebug>
#include <QRegularExpression>

#include "do219.h"

// Imbedded Message Identifiers - ARINC 622: Table 2-1
const QHash<QString, QString> m_imis = {
    {"AD1", "Automatic dependent surveillance (ARINC 745-1)"},
    {"ADS", "Automatic dependent surveillance (ARINC 745-2)"},
    {"DIS", "Automatic dependent surveillance (ARINC 745-1/2)"},
    {"AT0", "Air traffic control communication (Preceeding DO-219)"},
    {"AT1", "Air traffic control communication"},     // DO-219 / CPCLD
    {"CR1", "Air traffic control communication - Connect request"},
    {"CC1", "Air traffic control communication - Connect confirm"},
    {"DR1", "Air traffic control communication - Disconnect request"}
};

#include "util/ourairportsdb.h"

QSharedPointer<const QHash<QString, AirportInformation *>> m_airportInfo;

static QString icaoAirportName(const QString &icao)
{
    if (m_airportInfo)
    {
        if (m_airportInfo->contains(icao)) {
            return m_airportInfo->value(icao)->m_name;
        }
    }
    return "";
}

static QString icaoAndAirportName(const QString &icao)
{
    QString airportName = icaoAirportName(icao);
    if (airportName.isEmpty()) {
        return icao;
    } else {
        return QString("%1 (%2)").arg(icao).arg(airportName);
    }
}

AcarsMessage::AcarsMessage(const QByteArray &message)
{
    // Every field below is read at a fixed offset, so a block shorter than the smallest
    // legal one would index off the end - and QString/QByteArray operator[] only asserts
    // in a debug build, so a release build reads whatever is there instead of failing.
    // decode() checks the length before it builds one of these, but nothing obliged it
    // to: the test harness constructs an AcarsFlightStatus from an empty QByteArray to
    // exercise decode() on its own. A short block leaves every field empty.
    if (message.size() < ACARS_MIN_BYTES) {
        return;
    }

    // Mode is destination of message
    // 2 = All ground stations / aircraft - Other characters are specific station / aircraft
    // and country specific
    m_mode = QString("%1").arg(message[1]);

    // Address is aircraft registration. Prefixed with '.'s if registration is shorter than field width
    m_address = QString(message.mid(2, 7));
    while (m_address.startsWith(".")) {
        m_address = m_address.mid(1); // Remove prefix
    }

    m_ack = message[9];

    // Label (type of message)
    m_label = message.mid(10, 2);

    // Block ID indicates if downlink (0-9) or uplink (A-Z or NUL)
    m_blockId = message[12];
    m_uplink = isUplink(m_blockId);

    // Extract text
    qsizetype textSize = message.length() - 14 - 1 - 2 - 1;
    if (textSize >= 10)
    {
        m_text = message.mid(14, textSize);

        if (!m_uplink)
        {
            // Extract downlink fields
            m_originator = m_text[0];
            m_messageNumber = m_text.mid(1, 2);
            m_blockSequence = m_text[3];
            m_flight = m_text.mid(4, 6);
            // Remove extracted fields from text
            m_text = m_text.mid(10);
            // Is there a sublabel? (Typically for H1 label)
            if ((m_text.size() >= 3) && (m_text[0] == '#'))
            {
                m_subLabel = m_text.mid(1, 2);
                m_text = m_text.mid(3);
            }
        }
        else
        {
            if (m_text.startsWith("- #"))
            {
                m_subLabel = m_text.mid(3, 2);
                m_text = m_text.mid(5);
            }
        }
    }

    m_multiBlock = message[message.length() - 4] == ASCII_ETB;
}

QString AcarsMessage::toString()
{
    return "";   // Return nothing, as we don't know how to decode this message
}

AcarsMessage *AcarsMessage::decode(const QByteArray &message)
{
    // A no-text block is 17 bytes and has ETX directly after the block identifier. STX is
    // present only when text follows (ARINC 618 Appendix B).
    const bool noText = (message.size() == 17) && ((message[13] & 0x7f) == ASCII_ETX);
    const bool hasText = (message.size() >= 18) && ((message[13] & 0x7f) == ASCII_STX);

    if ((noText || hasText)
        && (message[0] == (char)ASCII_SOH)
        && (message[message.size()-1] == (char)ASCII_DEL))
    {
        // Remove parity bit from MSB
        QByteArray ascii = message;
        for (int i = 0; i < message.length(); i++) {
            ascii[i] = ascii[i] & 0x7f;
        }

        // Extract label (type of message)
        QString label(ascii.mid(10, 2));

        // Extract block identifier to determine if uplink or downlink
        QChar blockId(ascii[12]);
        bool uplink = isUplink(blockId);

        // Extract message text
        int start = 14;
        if (!uplink) {
            start += 10; // Skip msg num and flight
        }
        
        QString text;
        qsizetype textSize = message.length() - start - 1 - 2 - 1;
        if (textSize > 0) {
            text = ascii.mid(start, textSize);
        }

        return decode(ascii, label, text, uplink);
    }
    else
    {
        return nullptr;
    }
}

AcarsMessage *AcarsMessage::decode(const QByteArray &message, const QString &label, const QString &text, bool uplink)
{
    AcarsMessage *acarsMessage;
    if (label == ":;") {
        acarsMessage = new AcarsDataTransceiverAutoTune(message);
    } else if (label == "52") {
        acarsMessage = new AcarsGroundUTCRequest(message);
    } else if (label == "5U") {
        acarsMessage = new AcarsWeatherRequest(message);
    } else if ((label == "10") || (label == "12") || (label == "22") || (label == "24") || (label == "44")) {
        acarsMessage = new AcarsPositionReport(message);
    } else if (label == "80") {
        acarsMessage = new AcarsOpsReport(message);
    } else if ((label == "13") || (label == "14") || (label == "17") || (label == "18")) {
        acarsMessage = new AcarsEventReport(message);
    } else if (label == "15") {
        // Label 15 carries both FST01 flight status and /15 ON EVENT landing reports
        if (text.startsWith("/15")) {
            acarsMessage = new AcarsEventReport(message);
        } else {
            acarsMessage = new AcarsFlightStatus(message);
        }
    } else if (label == "16") {
        // Label 16 carries both N space position reports and /16 IN EVENT gate reports
        if (text.startsWith("/16")) {
            acarsMessage = new AcarsEventReport(message);
        } else {
            acarsMessage = new AcarsPositionReport(message);
        }
    } else if (label == "A0") {
        acarsMessage = new AcarsATSFacilitiesNotification(message);
    } else if (label == "A1") {
        acarsMessage = new AcarsOceanicClearanceResponse(message);
    } else if (label == "A3") {
        acarsMessage = new AcarsDepartureClearanceResponse(message);
    } else if (label == "A4") {
        acarsMessage = new AcarsFlightSystemsMessage(message);
    } else if (label == "A9") {
        acarsMessage = new AcarsATISReport(message);
    } else if (label == "B0") {
        acarsMessage = new AcarsATSFacilitiesNotification(message);
    } else if (label == "B1") {
        acarsMessage = new AcarsRequestOceanicClearance(message);
    } else if (label == "B2") {
        acarsMessage = new AcarsOceanicClearanceReadback(message);
    } else if (label == "B9") {
        acarsMessage = new AcarsRequestATISReport(message);
    } else if (label == "AA") {
        acarsMessage = new AcarsATCCommunications(message);
    } else if (label == "BA") {
        acarsMessage = new AcarsATCCommunications(message);
    } else if (label == "H1") {
        acarsMessage = new AcarsMessageFromSubsystem(message);
    } else if (label == "QP") {
        acarsMessage = new AcarsOutReport(message);
    } else if (label == "QQ") {
        acarsMessage = new AcarsOffReport(message);
    } else if (label == "QR") {
        acarsMessage = new AcarsOnReport(message);
    } else if (label == "QS") {
        acarsMessage = new AcarsInReport(message);
    } else if (label == "QT") {
        acarsMessage = new AcarsOutReturnInReport(message);
    } else if (label == "SQ") {
        acarsMessage = new AcarsSquitter(message);
    } else {
        acarsMessage = new AcarsMessage(message);
    }
    acarsMessage->decode(text);
    return acarsMessage;
}

// Block ID is 0-9 for downlink and A-Z for uplink
bool AcarsMessage::isUplink(QChar blockId)
{
    return !blockId.isDigit();
}

AcarsDataTransceiverAutoTune::AcarsDataTransceiverAutoTune(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsDataTransceiverAutoTune::decode(const QString &text)
{
    if (text.size() == 6)
    {
        m_frequencyHz = text.toInt() * 1000;
        m_precision = 3;
        return true;
    }
    else if (text.size() == 9)
    {
        m_frequencyHz = text.toInt();
        m_precision = 6;
        return true;
    }
    else
    {
        qDebug() << "AcarsDataTransceiverAutoTune: Can't decode: " << text;
        m_frequencyHz = -1;
        m_precision = 0;
        return false;
    }
}

QString AcarsDataTransceiverAutoTune::toString()
{
    if (m_frequencyHz != -1) {
        return QString("<b>Tune to:</b> %1MHz").arg(m_frequencyHz/1000000.0, 0, 'f', m_precision);
    } else {
        return "";
    }
}

AcarsGroundUTCRequest::AcarsGroundUTCRequest(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsGroundUTCRequest::decode(const QString &text)
{
    if (m_uplink)
    {
        QRegularExpression re(R"((\d\d)(\d\d)(\d\d)(\d)(\d\d)(\d\d)(\d\d))");
        QRegularExpressionMatch match = re.match(text);
        if (match.hasMatch())
        {
            m_date = QDate(match.captured(1).toInt() + 2000, match.captured(2).toInt(), match.captured(3).toInt());
            m_day = match.captured(4).toInt();
            m_time = QTime(match.captured(5).toInt(), match.captured(6).toInt(), match.captured(7).toInt());
            return true;
        }
        else
        {
            qDebug() << "AcarsGroundUTCRequest: Failed to match " << text;
            return false;
        }
    }
    else
    {
        return true;
    }
}

QString AcarsGroundUTCRequest::toString()
{
    if (m_uplink && m_date.isValid() && m_time.isValid())
    {
        const QStringList days = {"", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
        return QString("<b>UTC:</b> %1 %2 %3")
                    .arg(m_date.toString("yyyy-MM-dd"))
                    .arg(days[m_day])
                    .arg(m_time.toString());
    }
    else
    {
        return "";
    }
}

// See Table C-2 in Appendix C of ARINC 623
AcarsOceanicClearanceResponse::AcarsOceanicClearanceResponse(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsOceanicClearanceResponse::decode(const QString &text)
{
    // \r\n not always positioned as spec, so just replace with spaces
    QRegularExpression re(R"(CLX (\d\d)(\d\d) (\d\d)(\d\d)(\d\d) (\w\w\w\w) CLRNCE (\d\d\d) (\w*) CLRD TO (\w\w\w\w) VIA (\w*) (\w* \w*) ([A-Za-z0-9/ ]*)FM (\w*)\/(\d\d)(\d\d) MNTN F(\d\d\d) M(\d\d\d))");
    QRegularExpressionMatch match = re.match(text.simplified());
    if (match.hasMatch())
    {
        m_messageTime = QTime(match.captured(1).toInt(), match.captured(2).toInt());
        m_messageDate = QDate(match.captured(3).toInt() + 2000, match.captured(4).toInt(), match.captured(5).toInt());
        m_atcCenter = match.captured(6);
        m_clearanceNumber = match.captured(7).toInt();
        m_aircraft = match.captured(8);
        m_destination = match.captured(9);
        m_via = match.captured(10);
        m_routeId = match.captured(11);
        m_route = match.captured(12).simplified();
        m_entryPoint = match.captured(13);
        m_entryTime = QTime(match.captured(14).toInt(), match.captured(15).toInt());
        m_flightLevel = match.captured(16).toInt();
        m_mach = match.captured(17).toInt() / 100.0f;

        // TODO: ATC RECLEARANCE

/*        if (match.lastCapturedIndex() >= 9) {
            m_freeText = match.captured(9).trimmed();
        }*/
        return true;
    }
    else
    {
        qDebug() << "AcarsOceanicClearanceReadback: Failed to match " << text;
        return false;
    }
}

QString AcarsOceanicClearanceResponse::toString()
{
    if (m_messageTime.isValid())
    {
        QString s = QString("<b>Time:</b> %1<br><b>Date:</b> %2<br><b>ATC center:</b> %3<br><b>Clearance number:</b> %4<br><b>Aircraft:</b> %5<br><b>Destination:</b> %6<br><b>Via:</b> %7<br><b>Route Id:</b> %8<br><b>Route:</b> %9<br><b>Entry point:</b> %10<br><b>Entry time:</b> %11<br><b>Flight level:</b> %12<br><b>Mach:</b> %13")
                    .arg(m_messageTime.toString("hh:mm"))
                    .arg(m_messageDate.toString())
                    .arg(icaoAndAirportName(m_atcCenter))
                    .arg(m_clearanceNumber)
                    .arg(m_aircraft)
                    .arg(icaoAndAirportName(m_destination))
                    .arg(m_via)
                    .arg(m_routeId)
                    .arg(m_route)
                    .arg(m_entryPoint)
                    .arg(m_entryTime.toString("hh:mm"))
                    .arg(m_flightLevel)
                    .arg(m_mach)
                    ;
        if (!m_freeText.isEmpty()) {
            s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
        }
        return s;
    }
    else
    {
        return "";
    }
}

// See Table D-2 in Appendix D of ARINC 623
AcarsDepartureClearanceResponse::AcarsDepartureClearanceResponse(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsDepartureClearanceResponse::decode(const QString &text)
{
    // \r\n not always positioned as spec, so just replace with spaces
    QRegularExpression re(R"(CLD (\d\d)(\d\d) (\d\d)(\d\d)(\d\d) (\w\w\w\w) PDC (\d\d\d) ([A-Za-z0-9]*) CLRD TO (\w\w\w\w) OFF (\w*) VIA (\w*) SQUAWK (\d\d\d\d) (ADT|MDI) (\d*) ATIS (\w) ([\w ]*)?)");
    QRegularExpressionMatch match = re.match(text.simplified());
    if (match.hasMatch())
    {
        m_messageTime = QTime(match.captured(1).toInt(), match.captured(2).toInt());
        m_messageDate = QDate(match.captured(3).toInt() + 2000, match.captured(4).toInt(), match.captured(5).toInt());
        m_departureAirport = match.captured(6);
        m_clearanceNumber = match.captured(7).toInt();
        m_flight = match.captured(8);
        m_destination = match.captured(9);
        m_runway = match.captured(10);
        m_sid = match.captured(11);
        m_squawk = match.captured(12);
        m_departureTimeType = match.captured(13); // ADT or MDI
        // Departure time can be mss hhmm or hhmm/hhmm
        m_departureTime = match.captured(14);
        m_atis = match.captured(15);
        if (match.lastCapturedIndex() >= 16) {
            m_freeText = match.captured(16).trimmed();
        }
        return true;
    }
    else
    {
        qDebug() << "AcarsDepartureClearanceResponse: Failed to match " << text;
        return false;
    }
}

QString AcarsDepartureClearanceResponse::toString()
{
    if (m_messageTime.isValid())
    {
        QString s = QString("<b>Time:</b> %1<br><b>Date:</b> %2<br><b>Departure airport:</b> %3<br><b>Clearance number:</b> %4<br><b>Flight:</b> %5<br><b>Destination:</b> %6<br><b>Runway:</b> %7<br><b>SID:</b> %8<br><b>Squawk:</b> %9<br><b>Departure time:</b> %10 %11<br><b>ATIS:</b> %12")
                        .arg(m_messageTime.toString("hh:mm"))
                        .arg(m_messageDate.toString())
                        .arg(icaoAndAirportName(m_departureAirport))
                        .arg(m_clearanceNumber)
                        .arg(m_flight)
                        .arg(icaoAndAirportName(m_destination))
                        .arg(m_runway)
                        .arg(m_sid)
                        .arg(m_squawk)
                        .arg(m_departureTimeType)
                        .arg(m_departureTime)
                        .arg(m_atis)
                        ;
        if (!m_freeText.isEmpty()) {
            s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
        }
        return s;
    }
    else
    {
        return "";
    }
}

// See Attachment 9 in ARINC 623
AcarsFlightSystemsMessage::AcarsFlightSystemsMessage(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsFlightSystemsMessage::decode(const QString &text)
{
    QRegularExpression re(R"(FSM (\d\d)(\d\d) (\d\d)(\d\d)(\d\d) (\w\w\w\w) (\w*) ([\w\s]*)?)");
    QRegularExpressionMatch match = re.match(text.simplified());
    if (match.hasMatch())
    {
        m_messageTime = QTime(match.captured(1).toInt(), match.captured(2).toInt());
        m_messageDate = QDate(match.captured(3).toInt() + 2000, match.captured(4).toInt(), match.captured(5).toInt());
        m_departureAirport = match.captured(6);
        m_flight = match.captured(7);
        if (match.lastCapturedIndex() >= 8) {
            m_freeText = match.captured(8).trimmed(); // Sometimes last 4 digits are CRC, but not always
        }
        return true;
    }
    else
    {
        qDebug() << "AcarsFlightSystemsMessage: Failed to match " << text;
        return false;
    }
}

QString AcarsFlightSystemsMessage::toString()
{
    QString s;
    if (m_messageTime.isValid())
    {
        s.append(QString("<b>Time %1<br><b>Date:</b> %2<br><b>Departure airport:</b> %3<br><b>Flight:</b> %4")
                    .arg(m_messageTime.toString("hh:mm"))
                    .arg(m_messageDate.toString())
                    .arg(icaoAndAirportName(m_departureAirport))
                    .arg(m_flight));
        if (!m_freeText.isEmpty()) {
            s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
        }
    }
    return s;
}

// Appendix B in ARINC 623
AcarsATISReport::AcarsATISReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsATISReport::decode(const QString &text)
{
    QRegularExpression re(R"(TI2\/(\w\w\w\w)\s+(\w\w\w)\s+ATIS\s+(\w)\s+(\d\d)(\d\d)Z\s+([\s\S]*))");
    QRegularExpressionMatch match = re.match(text);
    if (match.hasMatch())
    {
        m_airport = match.captured(1);
        m_indicator = match.captured(2);
        m_atisId = match.captured(3);
        m_time = QTime(match.captured(4).toInt(), match.captured(5).toInt());
        m_atisInformation = match.captured(6);
        return true;
    }
    else
    {
        qDebug() << "AcarsATISReport: Failed to match " << text;
        return false;
    }
}

QString AcarsATISReport::toString()
{
    if (!m_airport.isEmpty())
    {
        // These are the only 3 in ARINC 623, but EGKK uses COM
        const QHash<QString, QString> indicators = {
            {"ARR", "Arrival"},
            {"DEP", "Departure"},
            {"ENR", "Enroute/VOLMET"}
        };
        QString indicator = m_indicator;
        if (indicators.contains(indicator)) {
            indicator = indicators[indicator];
        }
        QString s = QString("<b>Airport:</b> %1<br><b>Arrival/departure indicator:</b> %2<br><b>ATIS Id:</b> %3<br><b>Time:</b> %4")
                            .arg(icaoAndAirportName(m_airport))
                            .arg(indicator)
                            .arg(m_atisId)
                            .arg(m_time.toString("hh:mm"));
        s.append("\n");
        s.append(m_atisInformation);
        if (m_multiBlock)
        {
            s.append("");
        }
        return s;
    }
    else
    {
        return "";
    }
}

// See section 3 in ARINC 622
AcarsATSFacilitiesNotification::AcarsATSFacilitiesNotification(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsATSFacilitiesNotification::decode(const QString &text)
{
    QRegularExpression re(R"(\/(\w*)\.AFN\/FMH(\w*),([\.\-\w]*),(\w*)(,(\d\d)(\d\d)(\d\d))?)");
    QRegularExpressionMatch match = re.match(text);
    if (match.hasMatch())
    {
        m_atcCenter = match.captured(1);
        m_flight = match.captured(2);
        m_aircraftRegistration = match.captured(3);
        while (m_aircraftRegistration.startsWith(".")) {
            m_aircraftRegistration = m_aircraftRegistration.mid(1); // Remove prefix
        }
        m_aircraftICAO = match.captured(4);         // 24-bit ICAO hex id
        if (match.lastCapturedIndex() >= 7) {
            m_time = QTime(match.captured(5).toInt(), match.captured(6).toInt(), match.captured(7).toInt());
        }

        int offset = match.capturedStart(0) + match.captured(0).length() + 1; // Skip matched string and /
        m_mti = text.mid(offset, 3);
        return true;
    }
    else
    {
        qDebug() << "AcarsATSFacilitiesNotification: Failed to match " << text;
        return false;
    }
}

QString AcarsATSFacilitiesNotification::toString()
{
    QString s = QString("<b>ATC center:</b> %1<br><b>Flight:</b> %2<br><b>Aircraft:</b> %3")
                    .arg(icaoAndAirportName(m_atcCenter))
                    .arg(m_flight)
                    .arg(m_aircraftRegistration);
    if (!m_aircraftICAO.isEmpty()) {
        s.append(QString("<br><b>Aircraft ICAO:</b> %1").arg(m_aircraftICAO));
    }
    if (m_time.isValid()) {
        s.append(QString("<br><b>Time:</b> %1").arg(m_time.toString("hh:mm:ss")));
    }
    if (!m_mti.isEmpty())
    {
        const QHash<QString, QString> mtis = {
            {"FPO", "AFN contact"},
            {"FAK", "AFN acknowledge"},
            {"FCA", "AFN contact advisory"},
            {"FRP", "AFN response"},
            {"FCP", "AFN complete"},
        };
        s.append(QString("<br><b>MTI:</b> %1 (%2)").arg(m_mti).arg(mtis[m_mti]));
    }
    return s;
}

AcarsRequestOceanicClearance::AcarsRequestOceanicClearance(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsRequestOceanicClearance::decode(const QString &text)
{
    // Mostly start OC1/RCL, but occasionally just RCL
    QRegularExpression re(R"(RCL (\d\d\d)\ ([\w ]*)-([\w ]*)\/(\d\d)(\d\d) M(\d\d\d)F(\d\d\d)( -RMK\/(.*))?)");
    QRegularExpressionMatch match = re.match(text.simplified());
    if (match.hasMatch())
    {
        m_avionicsIndicator = match.captured(1);
        m_aircraft = match.captured(2);
        m_entryPoint = match.captured(3);
        m_entryTime = QTime(match.captured(4).toInt(), match.captured(5).toInt());
        m_mach =  match.captured(6).toInt() / 100.0f;
        m_flightLevel = match.captured(7).toInt();
        if (match.lastCapturedIndex() >= 9) {
            m_freeText = match.captured(9).trimmed();
        }
        return true;
    }
    else
    {
        qDebug() << "AcarsRequestOceanicClearance: Failed to match " << text;
        return false;
    }
}

QString AcarsRequestOceanicClearance::toString()
{
    QString s = QString("<b>Avionics rows:</b> %1<br><b>Aircraft:</b> %2<br><b>Entry point:</b> %3<br><b>Time:</b> %4<br><b>Mach:</b> %5<br><b>Flight level:</b> %6")
                    .arg(m_avionicsIndicator)
                    .arg(m_aircraft)
                    .arg(m_entryPoint)
                    .arg(m_entryTime.toString("hh:mm"))
                    .arg(m_mach, 0, 'f', 2)
                    .arg(m_flightLevel);
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}

AcarsOceanicClearanceReadback::AcarsOceanicClearanceReadback(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsOceanicClearanceReadback::decode(const QString &text)
{
    // \r<br><b> not always positioned as spec, so just replace with spaces
    // Mostly messages start OC1/CLA, but occasionally just CLA
    QRegularExpression re(R"(CLA (\d\d)(\d\d) (\d\d)(\d\d)(\d\d) (\w\w\w\w) CLRNCE (\d\d\d) (\w*) CLRD TO (\w\w\w\w) VIA (\w*) (\w* \w*) ([A-Za-z0-9/ ]*)FM (\w*)\/(\d\d)(\d\d) MNTN F(\d\d\d) M(\d\d\d))");
    QRegularExpressionMatch match = re.match(text.simplified());
    if (match.hasMatch())
    {
        m_messageTime = QTime(match.captured(1).toInt(), match.captured(2).toInt());
        m_messageDate = QDate(match.captured(3).toInt() + 2000, match.captured(4).toInt(), match.captured(5).toInt());
        m_atcCenter = match.captured(6);
        m_clearanceNumber = match.captured(7).toInt();
        m_aircraft = match.captured(8);
        m_destination = match.captured(9);
        m_via = match.captured(10);
        m_routeId = match.captured(11);
        m_route = match.captured(12).simplified();
        m_entryPoint = match.captured(13);
        m_entryTime = QTime(match.captured(14).toInt(), match.captured(15).toInt());
        m_flightLevel = match.captured(16).toInt();
        m_mach = match.captured(17).toInt() / 100.0f;

        // TODO: ATC RECLEARANCE

/*        if (match.lastCapturedIndex() >= 9) {
            m_freeText = match.captured(9).trimmed();
        }*/
        return true;
    }
    else
    {
        qDebug() << "AcarsOceanicClearanceReadback: Failed to match " << text;
        return false;
    }
}

QString AcarsOceanicClearanceReadback::toString()
{
    QString s = QString("<b>Time:</b> %1<br><b>Date:</b> %2<br><b>ATC center:</b> %3<br><b>Clearance number:</b> %4<br><b>Aircraft:</b> %5<br><b>Destination:</b> %6<br><b>Via:</b> %7<br><b>Route Id:</b> %8<br><b>Route:</b> %9<br><b>Entry point:</b> %10<br><b>Entry time:</b> %11<br><b>Flight level:</b> %12<br><b>Mach:</b> %13")
                    .arg(m_messageTime.toString("hh:mm"))
                    .arg(m_messageDate.toString())
                    .arg(icaoAndAirportName(m_atcCenter))
                    .arg(m_clearanceNumber)
                    .arg(m_aircraft)
                    .arg(icaoAndAirportName(m_destination))
                    .arg(m_via)
                    .arg(m_routeId)
                    .arg(m_route)
                    .arg(m_entryPoint)
                    .arg(m_entryTime.toString("hh:mm"))
                    .arg(m_flightLevel)
                    .arg(m_mach)
                    ;
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}

AcarsRequestATISReport::AcarsRequestATISReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsRequestATISReport::decode(const QString &text)
{
    int imiIdx = text.indexOf("TI2/");
    if (imiIdx != -1)
    {
        m_avionicsIndicator = text.mid(imiIdx + 4, 3);
        m_airport = text.mid(imiIdx + 7, 4).trimmed();
        m_arrivalDepartureIndicator = text.mid(imiIdx + 11, 1);
        return true;
    }
    else if (!text.contains('/'))
    {
        m_avionicsIndicator = text.mid(0, 3);
        m_airport = text.mid(3, 4).trimmed();
        m_arrivalDepartureIndicator = text.mid(7, 1);
        return true;
    }
    else
    {
        qDebug() << "AcarsRequestATISReport: Failed to match " << text;
        return false;
    }
}

QString AcarsRequestATISReport::toString()
{
    const QHash<QString, QString> indicators = {
        {"A", "Arrival ATIS"},
        {"D", "Departure ATIS"},
        {"C", "Arrival with automatic update"},
        {"E", "Automatic enroute information service (AEIS) or VOLMET"},
        {"T", "Terminate automatic update of ATIS"}
    };
    return QString("<b>Avionics rows:</b> %1<br><b>Airport:</b> %2<br><b>Arrival/departure indicator:</b> %3")
                .arg(m_avionicsIndicator)
                .arg(icaoAndAirportName(m_airport))
                .arg(indicators[m_arrivalDepartureIndicator]);
}

AcarsWeatherRequest::AcarsWeatherRequest(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsWeatherRequest::decode(const QString &text)
{
    if (text.contains("WXRQ"))
    {
        // E.g: 01 WXRQ   47EZ/20 LGRP/EGGP .G-JZDE /TYP 1/STA EGGP/STA EGCC/STA EGNX
        QRegularExpression staRe(R"(\/STA\s+([A-Z0-9]{4}))");
        QRegularExpressionMatchIterator it = staRe.globalMatch(text);
        while (it.hasNext()) {
            m_airports.append(it.next().captured(1));
        }
        QRegularExpression typRe(R"(\/TYP\s+(\w+))");
        QRegularExpressionMatch typMatch = typRe.match(text);
        if (typMatch.hasMatch()) {
            m_reportType = typMatch.captured(1);
        }
        // Flight route and registration: LGRP/EGGP .G-JZDE
        QRegularExpression routeRe(R"(([A-Z]{4})\/([A-Z]{4})\s+\.([A-Z0-9\-]+))");
        QRegularExpressionMatch routeMatch = routeRe.match(text);
        if (routeMatch.hasMatch())
        {
            m_origin = routeMatch.captured(1);
            m_destination = routeMatch.captured(2);
            m_registration = routeMatch.captured(3);
        }
        if (!m_airports.isEmpty()) {
            return true;
        }
    }
    else
    {
        // E.g: LFLL LFPG LFSG EGKK-SA - a list of airports then the MET report code wanted
        QRegularExpression re(R"(^([A-Z0-9]{4}(?: [A-Z0-9]{4})*)-([A-Z]{2})$)");
        QRegularExpressionMatch match = re.match(text.simplified());
        if (match.hasMatch())
        {
            m_airports = match.captured(1).split(' ');
            m_reportType = match.captured(2);
            return true;
        }
    }
    qDebug() << "AcarsWeatherRequest: Failed to match " << text;
    return false;
}

QString AcarsWeatherRequest::toString()
{
    if (m_airports.isEmpty()) {
        return "";
    }
    // WMO MET report type codes as used in ARINC 620 weather requests
    const QHash<QString, QString> reportTypes = {
        {"SA", "METAR"},
        {"SP", "SPECI"},
        {"FT", "TAF"},
        {"FC", "Short TAF"},
        {"FA", "Area forecast"},
        {"WS", "SIGMET"},
        {"NO", "NOTAM"}
    };
    QStringList lines;
    if (!m_registration.isEmpty()) {
        lines.append(QString("<b>Aircraft:</b> %1").arg(m_registration));
    }
    if (!m_origin.isEmpty()) {
        lines.append(QString("<b>Origin:</b> %1").arg(icaoAndAirportName(m_origin)));
    }
    if (!m_destination.isEmpty()) {
        lines.append(QString("<b>Destination:</b> %1").arg(icaoAndAirportName(m_destination)));
    }
    if (!m_reportType.isEmpty())
    {
        QString type = m_reportType;
        if (reportTypes.contains(type)) {
            type = QString("%1 (%2)").arg(reportTypes[type]).arg(m_reportType);
        }
        lines.append(QString("<b>Report type:</b> %1").arg(type));
    }
    for (const QString &airport : m_airports) {
        lines.append(QString("<b>Airport:</b> %1").arg(icaoAndAirportName(airport)));
    }
    return lines.join("<br>");
}

AcarsFlightStatus::AcarsFlightStatus(const QByteArray &message) :
    AcarsMessage(message)
{
}

// Flight status report, as used by British Airways on label 15. The format is not
// publicly documented; the field identification here is inferred from off-air
// messages and consistent with the examples at
// https://github.com/airframesio/acars-message-documentation/blob/main/research/15.md
//
// After FST01 come the departure and arrival airports and the position, then a
// fixed-width block - altitude in flight levels, fuel on board, fuel used and static
// air temperature - and finally wind, heading, track and ground speed.
//
// The position is NOT a fixed width. Latitude carries three or four decimal places
// (five or six digits) and longitude three or four (six or seven), and which is used
// varies by aircraft rather than by value, so trailing digits alone cannot say where
// the longitude ends. What pins it is the temperature: it always sits eleven
// characters past the position and is five characters ending in 'C', or five spaces
// when not reported. Only one longitude width puts it there. Across 54 off-air
// messages spanning three field layouts that resolved every one unambiguously.
//
// Assuming seven digits, as this did until 2026-08-26, shifts every field after the
// position one place left for the aircraft that send six. That read altitudes of
// 68000 and 77000 ft, fuel of 62.7 t on an E190 that cannot carry 13, and dropped the
// sign from the temperature - all from messages whose positions still looked right,
// which is why it went unnoticed. E.g. G-LCYJ out of London City:
//   FST01EGLCLEIB N51057 E000313 168 0627 0081 M010C 023 223 203 201 352  1244 1037
//                                   FL168 6270kg 810kg -10C  23kt/223  hdg/trk  352kt
//
// The two layouts differ in more than the position width. The compact one zero-pads
// its fields, signs the temperature adjacently, and counts fuel in 10 kg; the padded
// one right-aligns in spaces and counts fuel in 100 kg. E.g:
//   FST01EHAMEGLLN51659E0001153 92  30  19     23246      230  13431335
//   FST01KLAXEGLLN513366W0014766140 1691219 M10C025091098097344  14031345
// the second being a 747 from Los Angeles reporting 121900 kg burnt, which only makes
// sense at 100 kg a count.
bool AcarsFlightStatus::decode(const QString &text)
{
    // Departure and arrival airports, then latitude and longitude as degrees with a
    // variable number of decimal places
    static const QRegularExpression re(
        R"(^FST01([A-Z0-9]{4})([A-Z0-9]{4})([NS])(\d{4,6})([EW])(\d{6,}))");
    QRegularExpressionMatch match = re.match(text);
    if (!match.hasMatch())
    {
        qDebug() << "AcarsFlightStatus: Failed to match " << text;
        return false;
    }
    m_departureAirport = match.captured(1);
    m_arrivalAirport = match.captured(2);

    // Everything from the hemisphere character on, which is where the offsets below
    // are measured from
    const QString body = text.mid(13);
    const int latDigits = match.captured(4).size();

    // Find the longitude width by looking for the temperature where each candidate
    // would put it. Five spaces means the aircraft did not report one.
    static const QRegularExpression tempRe(R"(^(\s{5}|\s*[MP]?\s*(\d{1,3})C)$)");
    int lonDigits = 0;
    bool ambiguous = false;
    for (int candidate = 6; candidate <= 7; candidate++)
    {
        const int width = 2 + latDigits + candidate;
        if (body.size() < width + 16) {
            continue;
        }
        if (tempRe.match(body.mid(width + 11, 5)).hasMatch())
        {
            ambiguous = lonDigits != 0;
            lonDigits = candidate;
        }
    }

    // The position itself is usable either way - the two candidates differ by metres -
    // so report it even when the layout is unrecognised, but leave the fields whose
    // meaning depends on the width unset rather than guess at them
    const bool widthKnown = (lonDigits != 0) && !ambiguous;
    if (!widthKnown)
    {
        qDebug() << "AcarsFlightStatus: Could not locate the field boundary in " << text;
        lonDigits = 7;
    }

    QString latDigitStr = match.captured(4);
    m_latitude = (latDigitStr.left(2) + "." + latDigitStr.mid(2)).toFloat();
    if (match.captured(3) == "S") {
        m_latitude = -m_latitude;
    }
    QString lonDigitStr = match.captured(6).left(lonDigits);
    m_longitude = (lonDigitStr.left(3) + "." + lonDigitStr.mid(3)).toFloat();
    if (match.captured(5) == "W") {
        m_longitude = -m_longitude;
    }
    m_positionValid = true;
    if (!widthKnown) {
        return true;
    }

    // Fixed width fields: altitude in flight levels, then fuel on board and fuel used
    const bool compact = (lonDigits == 6);
    const int fuelUnitKg = compact ? 10 : 100;
    QString rest = body.mid(2 + latDigits + lonDigits);
    auto takeField = [&rest](int width)
    {
        QString field = rest.left(width).trimmed();
        rest = rest.mid(width);
        return field;
    };
    bool ok;
    int value = takeField(3).toInt(&ok);
    // Guard against a layout we have not seen putting something else here. No airliner
    // sending these reports cruises above FL430
    if (ok && (value * 100 <= ACARS_MAX_REPORTED_ALTITUDE_FT)) {
        m_altitudeFt = value * 100;
    }
    value = takeField(4).toInt(&ok);
    if (ok) {
        m_fuelOnBoardKg = value * fuelUnitKg;
    }
    value = takeField(4).toInt(&ok);
    if (ok) {
        m_fuelUsedKg = value * fuelUnitKg;
    }

    // Static air temperature, e.g. "M 58C" is -58C and "P018C" is +18C
    QString tempField = takeField(5);
    QRegularExpressionMatch tempMatch = tempRe.match(tempField);
    if (tempMatch.hasMatch() && !tempMatch.captured(2).isEmpty())
    {
        m_temperatureC = tempMatch.captured(2).toInt();
        if (tempField.contains('M')) {
            m_temperatureC = -m_temperatureC;
        }
        m_temperatureValid = true;
    }

    // The trailing eight digits are the ETA then the message time
    QString middle = rest.trimmed();
    static const QRegularExpression timesRe(R"((\d\d)(\d\d)(\d\d)(\d\d)$)");
    QRegularExpressionMatch timesMatch = timesRe.match(middle);
    if (timesMatch.hasMatch())
    {
        m_eta = QTime(timesMatch.captured(1).toInt(), timesMatch.captured(2).toInt());
        m_time = QTime(timesMatch.captured(3).toInt(), timesMatch.captured(4).toInt());
        middle = middle.left(middle.size() - 8).trimmed();
    }

    // Wind speed and direction, then heading, track and ground speed. The compact
    // layout packs all five as three digits each; the padded one gives two digits to
    // the wind speed and leaves heading and track blank
    static const QRegularExpression compactRe(R"(^(\d{3})(\d{3})(\d{3})(\d{3})(\d{3})$)");
    static const QRegularExpression paddedRe(R"(^(\d{1,2})(\d{3})\s+(\d{1,3})$)");
    QRegularExpressionMatch windMatch = compactRe.match(middle);
    if (compact && windMatch.hasMatch() && (windMatch.captured(2).toInt() <= 360))
    {
        m_windSpeedKts = windMatch.captured(1).toInt();
        m_windDirectionDeg = windMatch.captured(2).toInt();
        m_headingDeg = windMatch.captured(3).toInt();
        m_trackDeg = windMatch.captured(4).toInt();
        m_speedKts = windMatch.captured(5).toInt();
    }
    else if ((windMatch = paddedRe.match(middle)).hasMatch()
             && (windMatch.captured(2).toInt() <= 360))
    {
        m_windSpeedKts = windMatch.captured(1).toInt();
        m_windDirectionDeg = windMatch.captured(2).toInt();
        m_speedKts = windMatch.captured(3).toInt();
    }
    else if (!middle.isEmpty())
    {
        m_additionalData = middle;
    }
    return true;
}

QString AcarsFlightStatus::toString()
{
    if (m_departureAirport.isEmpty()) {
        return "";
    }
    QStringList lines;
    lines.append(QString("<b>Departure:</b> %1").arg(icaoAndAirportName(m_departureAirport)));
    lines.append(QString("<b>Arrival:</b> %1").arg(icaoAndAirportName(m_arrivalAirport)));
    lines.append(QString("<b>Position:</b> %1%2 %3%4")
                    .arg(std::abs(m_latitude), 0, 'f', 4)
                    .arg(m_latitude < 0.0f ? "S" : "N")
                    .arg(std::abs(m_longitude), 0, 'f', 4)
                    .arg(m_longitude < 0.0f ? "W" : "E"));
    if (m_altitudeFt >= 0) {
        lines.append(QString("<b>Altitude:</b> %1 ft").arg(m_altitudeFt));
    }
    if (m_fuelOnBoardKg >= 0) {
        lines.append(QString("<b>Fuel on board:</b> %1 t").arg(m_fuelOnBoardKg / 1000.0, 0, 'f', 2));
    }
    if (m_fuelUsedKg >= 0) {
        lines.append(QString("<b>Fuel used:</b> %1 t").arg(m_fuelUsedKg / 1000.0, 0, 'f', 2));
    }
    if (m_temperatureValid) {
        lines.append(QString("<b>Temperature:</b> %1%2C").arg(m_temperatureC).arg(QChar(0xB0)));
    }
    if (m_windSpeedKts >= 0) {
        lines.append(QString("<b>Wind:</b> %1%2 %3 kts").arg(m_windDirectionDeg).arg(QChar(0xB0)).arg(m_windSpeedKts));
    }
    if (m_headingDeg >= 0) {
        lines.append(QString("<b>Heading:</b> %1%2").arg(m_headingDeg).arg(QChar(0xB0)));
    }
    if (m_trackDeg >= 0) {
        lines.append(QString("<b>Track:</b> %1%2").arg(m_trackDeg).arg(QChar(0xB0)));
    }
    if (m_speedKts >= 0) {
        lines.append(QString("<b>Ground speed:</b> %1 kts").arg(m_speedKts));
    }
    if (m_eta.isValid()) {
        lines.append(QString("<b>ETA:</b> %1").arg(m_eta.toString("hh:mm")));
    }
    if (m_time.isValid()) {
        lines.append(QString("<b>Time:</b> %1").arg(m_time.toString("hh:mm")));
    }
    if (!m_additionalData.isEmpty()) {
        lines.append(QString("<b>Additional data:</b> %1").arg(m_additionalData));
    }
    return lines.join("<br>");
}

bool AcarsFlightStatus::getPosition(float& latitude, float& longitude, int& altitudeFt) const
{
    if (!m_positionValid) {
        return false;
    }
    latitude = m_latitude;
    longitude = m_longitude;
    altitudeFt = m_altitudeFt;
    return true;
}

static QString positionString(float latitude, float longitude)
{
    return QString("%1%2 %3%4")
                .arg(std::abs(latitude), 0, 'f', 4)
                .arg(latitude < 0.0f ? "S" : "N")
                .arg(std::abs(longitude), 0, 'f', 4)
                .arg(longitude < 0.0f ? "W" : "E");
}

AcarsPositionReport::AcarsPositionReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

// A family of related airline position report formats, one per label. Formats
// cross-checked against airframes.io's acars-decoder-typescript test vectors.
bool AcarsPositionReport::decode(const QString &text)
{
    if ((m_label == "12") || (m_label == "16"))
    {
        // E.g: N 42.150,W121.187,39000,161859, 109,.C-GWSO,1742   (12)
        //      N 44.203,W 86.546,31965,6, 290                     (16)
        //      N 28.177/W 96.055                                  (16)
        QRegularExpression re(R"(^([NS])\s*(\d{1,2}\.\d+)\s*[,/]\s*([EW])\s*(\d{1,3}\.\d+)(?:,\s*(\d+))?)");
        QRegularExpressionMatch match = re.match(text);
        if (match.hasMatch())
        {
            m_latitude = match.captured(2).toFloat() * (match.captured(1) == "S" ? -1.0f : 1.0f);
            m_longitude = match.captured(4).toFloat() * (match.captured(3) == "W" ? -1.0f : 1.0f);
            m_positionValid = true;
            if (!match.captured(5).isEmpty()) {
                m_altitudeFt = match.captured(5).toInt();
            }
            if (m_label == "12")
            {
                // The fourth field is the time as HHMMSS
                QStringList fields = text.split(',');
                if (fields.size() >= 4)
                {
                    QString t = fields[3].trimmed();
                    if (t.size() == 6) {
                        m_time = QTime(t.mid(0, 2).toInt(), t.mid(2, 2).toInt(), t.mid(4, 2).toInt());
                    }
                }
            }
            return true;
        }
    }
    else if (m_label == "10")
    {
        // E.g: POS082150, N 3885,W 7841,---,308,26922,  51,22290, 529,  19,-225,6
        // The time is HHMMSS; latitude and longitude are degrees x100; the eighth
        // field is the altitude in feet
        QRegularExpression re(R"(^POS(\d{6}),\s*([NS])\s*(\d+),\s*([EW])\s*(\d+),)");
        QRegularExpressionMatch match = re.match(text);
        if (match.hasMatch())
        {
            m_time = QTime(match.captured(1).mid(0, 2).toInt(),
                           match.captured(1).mid(2, 2).toInt(),
                           match.captured(1).mid(4, 2).toInt());
            m_latitude = match.captured(3).toFloat() / 100.0f * (match.captured(2) == "S" ? -1.0f : 1.0f);
            m_longitude = match.captured(5).toFloat() / 100.0f * (match.captured(4) == "W" ? -1.0f : 1.0f);
            m_positionValid = true;
            QStringList fields = text.split(',');
            if (fields.size() >= 8)
            {
                bool ok;
                int alt = fields[7].trimmed().toInt(&ok);
                if (ok) {
                    m_altitudeFt = alt;
                }
            }
            return true;
        }
    }
    else if (m_label == "22")
    {
        // E.g: N 370824W 760010,-------,194936,30418, ,      , ,M 42,27335  42, 107,
        // Latitude and longitude are degrees x10000; the third field is the time as
        // HHMMSS and the fourth the altitude in feet
        QRegularExpression re(R"(^([NS])\s*(\d{6})\s*([EW])\s*(\d{6}),)");
        QRegularExpressionMatch match = re.match(text);
        if (match.hasMatch())
        {
            m_latitude = match.captured(2).toFloat() / 10000.0f * (match.captured(1) == "S" ? -1.0f : 1.0f);
            m_longitude = match.captured(4).toFloat() / 10000.0f * (match.captured(3) == "W" ? -1.0f : 1.0f);
            m_positionValid = true;
            QStringList fields = text.split(',');
            if ((fields.size() >= 3) && (fields[2].trimmed().size() == 6))
            {
                QString t = fields[2].trimmed();
                m_time = QTime(t.mid(0, 2).toInt(), t.mid(2, 2).toInt(), t.mid(4, 2).toInt());
            }
            if (fields.size() >= 4)
            {
                bool ok;
                int alt = fields[3].trimmed().toInt(&ok);
                if (ok) {
                    m_altitudeFt = alt;
                }
            }
            return true;
        }
    }
    else if (m_label == "44")
    {
        // E.g: POS02,N38171W077507,319,KJFK,KUZA,0926,0245,0327,004.6
        // Coordinates are degrees then minutes x10 (38 degrees 17.1 minutes);
        // altitude is in flight levels; then departure and arrival airports, the
        // date as MMDD, the time and ETA as HHMM, and fuel in tonnes
        QRegularExpression re(R"(^(?:00)?(POS|ETA|IN|OFF|ON)(\d{2}),([NS])(\d\d)(\d{3})([EW])(\d{3})(\d{3}),(\d{1,3}),(\w{4}),(\w{4}),(\d{4}),(\d{4}),(\d{4}),(\d+\.?\d*))");
        QRegularExpressionMatch match = re.match(text);
        if (match.hasMatch())
        {
            m_reportType = match.captured(1);
            m_latitude = match.captured(4).toFloat() + match.captured(5).toFloat() / 10.0f / 60.0f;
            if (match.captured(3) == "S") {
                m_latitude = -m_latitude;
            }
            m_longitude = match.captured(7).toFloat() + match.captured(8).toFloat() / 10.0f / 60.0f;
            if (match.captured(6) == "W") {
                m_longitude = -m_longitude;
            }
            m_positionValid = true;
            m_altitudeFt = match.captured(9).toInt() * 100;
            m_departureAirport = match.captured(10);
            m_arrivalAirport = match.captured(11);
            m_month = match.captured(12).mid(0, 2).toInt();
            m_day = match.captured(12).mid(2, 2).toInt();
            m_time = QTime(match.captured(13).mid(0, 2).toInt(), match.captured(13).mid(2, 2).toInt());
            m_eta = QTime(match.captured(14).mid(0, 2).toInt(), match.captured(14).mid(2, 2).toInt());
            m_fuelTons = match.captured(15).toFloat();
            return true;
        }
    }
    else if (m_label == "24")
    {
        // E.g: /241710/1021/04WM/34962/N53.13/E001.33/3374/1056/
        // Day and time, then flight, altitude in feet, position and ETA
        QRegularExpression re(R"(^/24(\d{4})/(\d+)/(\w+)/(\d+)/([NS])(\d{1,2}\.\d+)/([EW])(\d{1,3}\.\d+)/(\d+)/(\d{4})/?)");
        QRegularExpressionMatch match = re.match(text);
        if (match.hasMatch())
        {
            m_time = QTime(match.captured(1).mid(0, 2).toInt(), match.captured(1).mid(2, 2).toInt());
            m_reportFlight = match.captured(3);
            m_altitudeFt = match.captured(4).toInt();
            m_latitude = match.captured(6).toFloat() * (match.captured(5) == "S" ? -1.0f : 1.0f);
            m_longitude = match.captured(8).toFloat() * (match.captured(7) == "W" ? -1.0f : 1.0f);
            m_positionValid = true;
            m_eta = QTime(match.captured(10).mid(0, 2).toInt(), match.captured(10).mid(2, 2).toInt());
            return true;
        }
    }
    qDebug() << "AcarsPositionReport: Failed to match label" << m_label << text;
    return false;
}

QString AcarsPositionReport::toString()
{
    if (!m_positionValid) {
        return "";
    }
    const QHash<QString, QString> reportTypes = {
        {"POS", "Position report"},
        {"ETA", "ETA report"},
        {"IN", "In gate report"},
        {"OFF", "Takeoff report"},
        {"ON", "Landing report"}
    };
    QStringList lines;
    if (!m_reportType.isEmpty()) {
        lines.append(QString("<b>Report:</b> %1").arg(reportTypes.value(m_reportType, m_reportType)));
    }
    if (!m_reportFlight.isEmpty()) {
        lines.append(QString("<b>Flight:</b> %1").arg(m_reportFlight));
    }
    if (!m_departureAirport.isEmpty()) {
        lines.append(QString("<b>Departure:</b> %1").arg(icaoAndAirportName(m_departureAirport)));
    }
    if (!m_arrivalAirport.isEmpty()) {
        lines.append(QString("<b>Arrival:</b> %1").arg(icaoAndAirportName(m_arrivalAirport)));
    }
    lines.append(QString("<b>Position:</b> %1").arg(positionString(m_latitude, m_longitude)));
    if (m_altitudeFt >= 0) {
        lines.append(QString("<b>Altitude:</b> %1 ft").arg(m_altitudeFt));
    }
    if (m_day >= 0) {
        lines.append(QString("<b>Date:</b> %1/%2").arg(m_day, 2, 10, QChar('0')).arg(m_month, 2, 10, QChar('0')));
    }
    if (m_time.isValid()) {
        lines.append(QString("<b>Time:</b> %1").arg(m_time.toString(m_time.second() ? "hh:mm:ss" : "hh:mm")));
    }
    if (m_eta.isValid()) {
        lines.append(QString("<b>ETA:</b> %1").arg(m_eta.toString("hh:mm")));
    }
    if (m_fuelTons >= 0.0f) {
        lines.append(QString("<b>Fuel on board:</b> %1 t").arg(m_fuelTons, 0, 'f', 1));
    }
    return lines.join("<br>");
}

bool AcarsPositionReport::getPosition(float& latitude, float& longitude, int& altitudeFt) const
{
    if (!m_positionValid) {
        return false;
    }
    latitude = m_latitude;
    longitude = m_longitude;
    altitudeFt = m_altitudeFt;
    return true;
}

AcarsEventReport::AcarsEventReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

// OOOI event reports: /13 OUT EVENT      / KSFO KEWR 12 231445/TIME 2314
// with optional additional lines such as /LOC N373703,W1222251 (degrees, minutes,
// seconds), /LOC +38.9603,-077.4595 (decimal) and /FOB 0215 (fuel, 100kg)
bool AcarsEventReport::decode(const QString &text)
{
    QRegularExpression re(R"(^/1([3-8])\s+(.+?)/\s*([A-Z0-9]{4})\s+([A-Z0-9]{4})\s+(\d{1,2})\s+(\d{6}))");
    QRegularExpressionMatch match = re.match(text);
    if (!match.hasMatch())
    {
        qDebug() << "AcarsEventReport: Failed to match " << text;
        return false;
    }
    m_event = 10 + match.captured(1).toInt();
    m_eventDescription = match.captured(2).trimmed();
    m_departureAirport = match.captured(3);
    m_arrivalAirport = match.captured(4);
    m_day = match.captured(5).toInt();
    QString t = match.captured(6);
    m_eventTime = QTime(t.mid(0, 2).toInt(), t.mid(2, 2).toInt(), t.mid(4, 2).toInt());

    QRegularExpression fobRe(R"(/FOB\s+(\d+))");
    QRegularExpressionMatch fobMatch = fobRe.match(text);
    if (fobMatch.hasMatch()) {
        m_fuelOnBoard = fobMatch.captured(1).toInt();
    }

    // Position in degrees/minutes/seconds, e.g. N373703,W1222251
    QRegularExpression locDmsRe(R"(/LOC\s+([NS])(\d\d)(\d\d)(\d\d),\s*([EW])(\d{2,3})(\d\d)(\d\d))");
    QRegularExpressionMatch locMatch = locDmsRe.match(text);
    if (locMatch.hasMatch())
    {
        m_latitude = locMatch.captured(2).toFloat() + locMatch.captured(3).toFloat() / 60.0f + locMatch.captured(4).toFloat() / 3600.0f;
        if (locMatch.captured(1) == "S") {
            m_latitude = -m_latitude;
        }
        m_longitude = locMatch.captured(6).toFloat() + locMatch.captured(7).toFloat() / 60.0f + locMatch.captured(8).toFloat() / 3600.0f;
        if (locMatch.captured(5) == "W") {
            m_longitude = -m_longitude;
        }
        m_positionValid = true;
    }
    else
    {
        // Or decimal, e.g. +38.9603,-077.4595
        QRegularExpression locDecRe(R"(/LOC\s+([+-]\d{1,2}\.\d+),\s*([+-]\d{1,3}\.\d+))");
        locMatch = locDecRe.match(text);
        if (locMatch.hasMatch())
        {
            m_latitude = locMatch.captured(1).toFloat();
            m_longitude = locMatch.captured(2).toFloat();
            m_positionValid = true;
        }
    }
    return true;
}

QString AcarsEventReport::toString()
{
    if (m_eventDescription.isEmpty()) {
        return "";
    }
    const QHash<int, QString> events = {
        {13, "Out of gate"},
        {14, "Takeoff"},
        {15, "Landing"},
        {16, "In gate"},
        {17, "Post report"},
        {18, "Post times report"}
    };
    QStringList lines;
    lines.append(QString("<b>Event:</b> %1 (%2)").arg(events[m_event]).arg(m_eventDescription));
    lines.append(QString("<b>Departure:</b> %1").arg(icaoAndAirportName(m_departureAirport)));
    lines.append(QString("<b>Arrival:</b> %1").arg(icaoAndAirportName(m_arrivalAirport)));
    if (m_eventTime.isValid()) {
        lines.append(QString("<b>Time:</b> Day %1 %2").arg(m_day).arg(m_eventTime.toString("hh:mm:ss")));
    }
    if (m_fuelOnBoard >= 0) {
        lines.append(QString("<b>Fuel on board:</b> %1 t").arg(m_fuelOnBoard / 10.0, 0, 'f', 1));
    }
    if (m_positionValid) {
        lines.append(QString("<b>Position:</b> %1").arg(positionString(m_latitude, m_longitude)));
    }
    return lines.join("<br>");
}

bool AcarsEventReport::getPosition(float& latitude, float& longitude, int& altitudeFt) const
{
    if (!m_positionValid) {
        return false;
    }
    latitude = m_latitude;
    longitude = m_longitude;
    altitudeFt = -1;
    return true;
}

AcarsOpsReport::AcarsOpsReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

// Coordinates in label 80 reports appear as N29395 (degrees x1000) or N3539.2
// (degrees then decimal minutes)
static bool parseOpsCoordinate(const QString& s, float& value)
{
    QRegularExpression plainRe(R"(^([NSEW])(\d+)$)");
    QRegularExpressionMatch match = plainRe.match(s);
    if (match.hasMatch())
    {
        int degDigits = match.captured(2).size() - 3;
        value = match.captured(2).left(degDigits).toFloat()
                + match.captured(2).mid(degDigits).toFloat() / 1000.0f;
        if ((match.captured(1) == "S") || (match.captured(1) == "W")) {
            value = -value;
        }
        return true;
    }
    QRegularExpression minRe(R"(^([NSEW])(\d+)(\d\d\.\d)$)");
    match = minRe.match(s);
    if (match.hasMatch())
    {
        value = match.captured(2).toFloat() + match.captured(3).toFloat() / 60.0f;
        if ((match.captured(1) == "S") || (match.captured(1) == "W")) {
            value = -value;
        }
        return true;
    }
    return false;
}

// Airline operational reports, e.g:
// 3N01 POSRPT 5891/04 KIAH/MMGL .XA-VOI
// /POS N29395W095133/ALT +15608/MCH 558/FOB 0100/ETA 0410
// with OPNORM and INRANG variants, and optional /FL /TAS /SAT /HDG /NWYP /SPD /UTC
// keyed fields. Formats per airframes.io's decoder test vectors.
bool AcarsOpsReport::decode(const QString &text)
{
    QRegularExpression hdrRe(R"(^3\w0\d\s+(POSRPT|OPNORM|INRANG)\s+(\w+)/(\d{1,2})\s+([A-Z0-9]{4})/([A-Z0-9]{4})\s+\.([A-Z0-9-]+))");
    QRegularExpressionMatch match = hdrRe.match(text);
    if (!match.hasMatch())
    {
        qDebug() << "AcarsOpsReport: Failed to match " << text;
        return false;
    }
    m_reportType = match.captured(1);
    m_reportFlight = match.captured(2);
    m_day = match.captured(3).toInt();
    m_departureAirport = match.captured(4);
    m_arrivalAirport = match.captured(5);
    m_tail = match.captured(6);

    // Keyed fields anywhere in the remainder
    auto keyed = [&text](const char *pattern) -> QString
    {
        QRegularExpression re{QString(pattern)};
        QRegularExpressionMatch m = re.match(text);
        return m.hasMatch() ? m.captured(1) : QString();
    };

    QString pos = keyed(R"(/POS\s+([NS][\d.]+)\s?([EW][\d.]+))");
    if (!pos.isEmpty())
    {
        QRegularExpression posRe(R"(/POS\s+([NS][\d.]+)\s?([EW][\d.]+))");
        QRegularExpressionMatch posMatch = posRe.match(text);
        float lat, lon;
        if (parseOpsCoordinate(posMatch.captured(1), lat) && parseOpsCoordinate(posMatch.captured(2), lon))
        {
            m_latitude = lat;
            m_longitude = lon;
            m_positionValid = true;
        }
    }
    QString alt = keyed(R"(/ALT\s+\+?(\d+))");
    if (!alt.isEmpty()) {
        m_altitudeFt = alt.toInt();
    }
    QString fl = keyed(R"(/FL\s+(\d+))");
    if (!fl.isEmpty()) {
        m_altitudeFt = fl.toInt() * 100;
    }
    m_nextWaypoint = keyed(R"(/NWYP\s+(\w+))");
    QString hdg = keyed(R"(/HDG\s+(\d+))");
    if (!hdg.isEmpty()) {
        m_headingDeg = hdg.toInt();
    }
    QString mch = keyed(R"(/MCH\s+(\d+))");
    if (!mch.isEmpty()) {
        m_machX1000 = mch.toInt();
    }
    QString spd = keyed(R"(/(?:TAS|SPD)\s+(\d+))");
    if (!spd.isEmpty()) {
        m_speedKts = spd.toInt();
    }
    QString sat = keyed(R"(/SAT\s+([+-]?\d+))");
    if (!sat.isEmpty())
    {
        m_temperatureC = sat.toInt();
        m_temperatureValid = true;
    }
    m_fuelOnBoard = keyed(R"(/FOB\s+([N\d.]+))");
    QString utc = keyed(R"(/UTC\s+(\d{6}))");
    if (!utc.isEmpty()) {
        m_time = QTime(utc.mid(0, 2).toInt(), utc.mid(2, 2).toInt(), utc.mid(4, 2).toInt());
    }
    QString eta = keyed(R"(/ETA\s+(\d\d):?(\d\d))");
    if (!eta.isEmpty())
    {
        QRegularExpression etaRe(R"(/ETA\s+(\d\d):?(\d\d))");
        QRegularExpressionMatch etaMatch = etaRe.match(text);
        m_eta = QTime(etaMatch.captured(1).toInt(), etaMatch.captured(2).toInt());
    }
    return true;
}

QString AcarsOpsReport::toString()
{
    if (m_reportType.isEmpty()) {
        return "";
    }
    const QHash<QString, QString> reportTypes = {
        {"POSRPT", "Position report"},
        {"OPNORM", "Operations normal"},
        {"INRANG", "In range"}
    };
    QStringList lines;
    lines.append(QString("<b>Report:</b> %1").arg(reportTypes.value(m_reportType, m_reportType)));
    lines.append(QString("<b>Flight:</b> %1").arg(m_reportFlight));
    lines.append(QString("<b>Aircraft:</b> %1").arg(m_tail));
    lines.append(QString("<b>Departure:</b> %1").arg(icaoAndAirportName(m_departureAirport)));
    lines.append(QString("<b>Arrival:</b> %1").arg(icaoAndAirportName(m_arrivalAirport)));
    if (m_positionValid) {
        lines.append(QString("<b>Position:</b> %1").arg(positionString(m_latitude, m_longitude)));
    }
    if (m_altitudeFt >= 0) {
        lines.append(QString("<b>Altitude:</b> %1 ft").arg(m_altitudeFt));
    }
    if (!m_nextWaypoint.isEmpty()) {
        lines.append(QString("<b>Next waypoint:</b> %1").arg(m_nextWaypoint));
    }
    if (m_headingDeg >= 0) {
        lines.append(QString("<b>Heading:</b> %1%2").arg(m_headingDeg).arg(QChar(0xB0)));
    }
    if (m_machX1000 >= 0) {
        lines.append(QString("<b>Mach:</b> %1").arg(m_machX1000 / 1000.0, 0, 'f', 3));
    }
    if (m_speedKts >= 0) {
        lines.append(QString("<b>Speed:</b> %1 kts").arg(m_speedKts));
    }
    if (m_temperatureValid) {
        lines.append(QString("<b>Temperature:</b> %1%2C").arg(m_temperatureC).arg(QChar(0xB0)));
    }
    if (!m_fuelOnBoard.isEmpty()) {
        lines.append(QString("<b>Fuel on board:</b> %1").arg(m_fuelOnBoard));
    }
    if (m_time.isValid()) {
        lines.append(QString("<b>Time:</b> Day %1 %2").arg(m_day).arg(m_time.toString("hh:mm:ss")));
    }
    if (m_eta.isValid()) {
        lines.append(QString("<b>ETA:</b> %1").arg(m_eta.toString("hh:mm")));
    }
    return lines.join("<br>");
}

bool AcarsOpsReport::getPosition(float& latitude, float& longitude, int& altitudeFt) const
{
    if (!m_positionValid) {
        return false;
    }
    latitude = m_latitude;
    longitude = m_longitude;
    altitudeFt = m_altitudeFt;
    return true;
}

bool ARINC622::decode(const QString &text, bool uplink)
{
    QRegularExpression re(R"(\/(?:\w\w )?(\w*)\.(\w\w\d)([\.\w\-]{7})(.*)(\w\w\w\w))");
    QRegularExpressionMatch match = re.match(text.simplified());
    if (match.hasMatch())
    {
        m_atcCenter = match.captured(1);
        m_imi = match.captured(2);
        m_aircraft = match.captured(3);
        while (m_aircraft.startsWith(".")) {
            m_aircraft = m_aircraft.mid(1); // Remove "." prefix
        }
        // m_aircraft should be checked against ACARS registration field to ensure they match
        m_app = match.captured(4);
        if ((m_imi == "AT1") || (m_imi == "CR1"))
        {
            DO219Decoder decoder;
            m_do219 = decoder.decode(m_app, uplink);
        }

        m_crc = match.captured(5).toInt(nullptr, 16);
        m_valid = true;
        return true;
    }
    else
    {
        return false;
    }
}

QString ARINC622::toString()
{
    if (m_valid)
    {
        QString s = QString("<b>ATC Center:</b> %1<br><b>IMI:</b> %2<br><b>Aircraft:</b> %3<br><b>CRC:</b> %4")
                    .arg(m_atcCenter)
                    .arg(m_imis[m_imi])
                    .arg(m_aircraft)
                    .arg(QString::number(m_crc, 16));

        if (!m_do219.isEmpty()) {
            s.append(QString("<br><b>CPDLC decode:</b> %1").arg(m_do219));
        } else  if (!m_app.isEmpty()) {
            s.append(QString("<br><b>CPDLC data:</b> %1").arg(m_app));
        }
        return s;
    }
    else
    {
        return "";
    }
}

AcarsATCCommunications::AcarsATCCommunications(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsATCCommunications::decode(const QString &text)
{
    return m_622.decode(text, m_uplink);
}

QString AcarsATCCommunications::toString()
{
    return m_622.toString();
}

AcarsMessageFromSubsystem::AcarsMessageFromSubsystem(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsMessageFromSubsystem::decode(const QString &text)
{
    if (m_622.decode(text, m_uplink)) {
        return true;
    }

    // Not an ARINC 622 ATS message: try the FMS report formats, which start with a
    // #<sublabel> prefix such as #M1B (FMC), e.g:
    // #M1BPOSN43312W123174,EASON,215754,370,EBINY,220601,ELENN,M48,02216,185/TS215754,092122
    // #M1BFPN/RP:DA:KSFO:AA:KEWR:R:...
    // Field meanings per airframes.io's acars-message-documentation research
    QString t = text;
    QRegularExpression subRe(R"(^#(\w{2}[A-Z]?))");
    QRegularExpressionMatch subMatch = subRe.match(t);
    if (subMatch.hasMatch())
    {
        m_fmsSublabel = subMatch.captured(1);
        t = t.mid(subMatch.capturedLength());
    }
    else if (!t.startsWith("POS") && !t.startsWith("FPN/")
             && (t.mid(1).startsWith("POS") || t.mid(1).startsWith("FPN/")))
    {
        // The header parse strips a #xx sublabel from the text, which can leave just
        // the supplementary character (the B of #M1B) before the report type - this
        // is the form the multipart assembler's combined text takes
        m_fmsSublabel = m_subLabel + t.left(1);
        t = t.mid(1);
    }

    // A route insert - the flight plan the FMS is flying - as ARINC 702A writes it:
    // colon separated fields in an /RP: block, appended to a position report or sent on
    // its own as an FPN message. The /RA: alternate block has the same shape.
    //
    //   /RP:DA:EGLL:AA:ENGM:R:27L(01R):D:BPK7G:A:RIPA3L:AP:ILSY01R.VABLPU..BPK,
    //   N51450W000064.Q295.PAAVO..PAAVO,N51518E000513.M604.GIVPO..LARGA,N54518E004092
    //
    // DA is the departure airport, AA the arrival, R the departure runway with the
    // arrival's in brackets, D the SID, A the STAR and AP the lateral path. In the path
    // "." separates a fix from the airway to the next one and ".." is a direct leg, and
    // a fix may be followed by its position. R used to be read as the route, which is
    // why a plan only ever plotted as its two airports: it is the runway.
    auto extractFlightPlan = [this](const QString& s)
    {
        // Read within the /RP: block where there is one, so the /RA: alternate block's
        // own DA and AA are not taken for this flight's
        auto block = [](const QString& text, const QString& tag) -> QString
        {
            const int start = text.indexOf(tag + ":");
            if (start < 0) {
                return QString();
            }
            const int end = text.indexOf('/', start);
            return (end >= 0) ? text.mid(start, end - start) : text.mid(start);
        };
        auto field = [](const QString& text, const char *tag) -> QString
        {
            if (text.isEmpty()) {
                return QString();
            }
            const QRegularExpression re(QString(":%1:([^:/]*)").arg(tag));
            const QRegularExpressionMatch match = re.match(text);
            return match.hasMatch() ? match.captured(1).trimmed() : QString();
        };

        const QString rpBlock = block(s, "RP");
        const QString rp = rpBlock.isEmpty() ? s : rpBlock;

        m_fpnDeparture = field(rp, "DA");
        m_fpnArrival = field(rp, "AA");
        m_fpnRunway = field(rp, "R");
        m_fpnSid = field(rp, "D");
        m_fpnStar = field(rp, "A");
        m_fpnAlternate = field(block(s, "RA"), "AA");

        QString path = field(rp, "AP");
        if (path.isEmpty()) {
            path = field(rp, "F");
        }
        // N51450 is 51 degrees 45.0 minutes north and W000064 is 0 degrees 6.4 minutes
        // west: three digits of minutes of which the last is tenths, after two digits of
        // degrees for latitude and three for longitude
        static const QRegularExpression coordRe(R"(^([NS])(\d{2})(\d{3})([EW])(\d{3})(\d{3})$)");
        const QStringList tokens = path.split('.', Qt::SkipEmptyParts);

        // Everything before the first fix that carries a position is the approach and
        // its transition: the path proper is the part that is placed. Where nothing is
        // placed there is nothing to separate them by, and it is all route.
        int first = 0;
        while ((first < tokens.size())
               && !coordRe.match(tokens[first].section(',', 1, 1).trimmed()).hasMatch()) {
            first++;
        }
        if (first >= tokens.size()) {
            first = 0;
        } else if (first > 0) {
            m_fpnApproach = QStringList(tokens.mid(0, first)).join(" ");
        }

        for (int i = first; i < tokens.size(); i++)
        {
            const QString name = tokens[i].section(',', 0, 0).trimmed();
            if (name.isEmpty()) {
                continue;
            }
            const QRegularExpressionMatch coord =
                coordRe.match(tokens[i].section(',', 1, 1).trimmed());

            FpnWaypoint waypoint;
            waypoint.m_name = name;
            if (coord.hasMatch())
            {
                waypoint.m_hasPosition = true;
                waypoint.m_latitude = (coord.captured(2).toFloat()
                    + coord.captured(3).toFloat() / 600.0f) * (coord.captured(1) == "S" ? -1.0f : 1.0f);
                waypoint.m_longitude = (coord.captured(5).toFloat()
                    + coord.captured(6).toFloat() / 600.0f) * (coord.captured(4) == "W" ? -1.0f : 1.0f);
            }

            // A fix is usually given twice - once as the airway's far end and again
            // with its position - so a name that has just repeated replaces rather than
            // extends, keeping whichever of the two placed it
            if (!m_fpnWaypoints.isEmpty() && (m_fpnWaypoints.last().m_name == name))
            {
                if (waypoint.m_hasPosition) {
                    m_fpnWaypoints.last() = waypoint;
                }
                continue;
            }
            m_fpnWaypoints.append(waypoint);
        }

        QStringList names;
        for (const FpnWaypoint& waypoint : m_fpnWaypoints) {
            names.append(waypoint.m_name);
        }
        m_fpnRoute = names.join(" ");

        // Older or airline specific forms write the route as free text after the
        // arrival airport instead of as a path
        if (m_fpnRoute.isEmpty() && !m_fpnArrival.isEmpty())
        {
            static const QRegularExpression tailRe(R"(:AA:\w{4}\.+(.+)$)");
            const QRegularExpressionMatch tailMatch = tailRe.match(s);
            if (tailMatch.hasMatch()) {
                m_fpnRoute = tailMatch.captured(1).trimmed();
            }
        }
    };

    // Position report: position, last waypoint and time over it, altitude in flight
    // levels, next waypoint and its ETA, the waypoint after, and the static air
    // temperature. Latitude and longitude are degrees x1000.
    QRegularExpression posRe(R"(^POS([NS])(\d{4,5})([EW])(\d{5,6}),([^,]*),(\d{6}),(\d+),([^,]*),(\d{6}),([^,]*),([MP])\s?(\d+)(.*)$)");
    QRegularExpressionMatch posMatch = posRe.match(t);
    if (posMatch.hasMatch())
    {
        m_latitude = posMatch.captured(2).toFloat() / 1000.0f * (posMatch.captured(1) == "S" ? -1.0f : 1.0f);
        m_longitude = posMatch.captured(4).toFloat() / 1000.0f * (posMatch.captured(3) == "W" ? -1.0f : 1.0f);
        m_positionValid = true;
        m_lastWaypoint = posMatch.captured(5);
        QString ts = posMatch.captured(6);
        m_timeOver = QTime(ts.mid(0, 2).toInt(), ts.mid(2, 2).toInt(), ts.mid(4, 2).toInt());
        m_altitudeFt = posMatch.captured(7).toInt() * 100;
        m_nextWaypoint = posMatch.captured(8);
        ts = posMatch.captured(9);
        m_etaNext = QTime(ts.mid(0, 2).toInt(), ts.mid(2, 2).toInt(), ts.mid(4, 2).toInt());
        m_nextPlus1Waypoint = posMatch.captured(10);
        m_temperatureC = posMatch.captured(12).toInt();
        if (posMatch.captured(11) == "M") {
            m_temperatureC = -m_temperatureC;
        }
        m_temperatureValid = true;
        QString additional = posMatch.captured(13);
        if (additional.startsWith(",")) {
            additional = additional.mid(1);
        }
        m_posAdditionalData = additional.trimmed();
        // A position report can carry a route insert after the report fields; it is
        // decoded into the flight plan fields above, so the several hundred characters
        // it occupies are dropped from the raw remainder rather than shown twice. Cut
        // out just those blocks: what follows them is a different subsystem's data.
        extractFlightPlan(m_posAdditionalData);
        static const QRegularExpression insertRe(R"(/R[PIA]:[^/]*)");
        m_posAdditionalData.remove(insertRe);
        m_posAdditionalData = m_posAdditionalData.trimmed();
        return true;
    }

    // Flight plan
    if (t.startsWith("FPN/"))
    {
        extractFlightPlan(t);
        return !m_fpnDeparture.isEmpty() || !m_fpnArrival.isEmpty() || !m_fpnRoute.isEmpty()
            || !m_fpnRunway.isEmpty() || !m_fpnSid.isEmpty() || !m_fpnStar.isEmpty();
    }

    return false;
}

QString AcarsMessageFromSubsystem::toString()
{
    if (m_622.m_valid) {
        return m_622.toString();
    }
    QStringList lines;
    bool hasFpn = !m_fpnDeparture.isEmpty() || !m_fpnArrival.isEmpty() || !m_fpnRoute.isEmpty()
                || !m_fpnRunway.isEmpty() || !m_fpnSid.isEmpty() || !m_fpnStar.isEmpty()
                || !m_fpnApproach.isEmpty() || !m_fpnAlternate.isEmpty();
    if (m_positionValid)
    {
        lines.append(QString("<b>Position:</b> %1").arg(positionString(m_latitude, m_longitude)));
        if (m_altitudeFt >= 0) {
            lines.append(QString("<b>Altitude:</b> %1 ft").arg(m_altitudeFt));
        }
        if (!m_lastWaypoint.isEmpty()) {
            lines.append(QString("<b>Last waypoint:</b> %1 at %2").arg(m_lastWaypoint).arg(m_timeOver.toString("hh:mm:ss")));
        }
        if (!m_nextWaypoint.isEmpty()) {
            lines.append(QString("<b>Next waypoint:</b> %1 ETA %2").arg(m_nextWaypoint).arg(m_etaNext.toString("hh:mm:ss")));
        }
        if (!m_nextPlus1Waypoint.isEmpty()) {
            lines.append(QString("<b>Then:</b> %1").arg(m_nextPlus1Waypoint));
        }
        if (m_temperatureValid) {
            lines.append(QString("<b>Temperature:</b> %1%2C").arg(m_temperatureC).arg(QChar(0xB0)));
        }
    }
    if (hasFpn)
    {
        // In the order the flight is flown, rather than the order the fields happen
        // to appear in the message
        if (!m_fpnDeparture.isEmpty()) {
            lines.append(QString("<b>Departure:</b> %1").arg(icaoAndAirportName(m_fpnDeparture)));
        }
        if (!m_fpnRunway.isEmpty()) {
            lines.append(QString("<b>Runway:</b> %1").arg(m_fpnRunway));
        }
        if (!m_fpnSid.isEmpty()) {
            lines.append(QString("<b>SID:</b> %1").arg(m_fpnSid));
        }
        if (!m_fpnRoute.isEmpty()) {
            lines.append(QString("<b>Route:</b> %1").arg(m_fpnRoute));
        }
        if (!m_fpnStar.isEmpty()) {
            lines.append(QString("<b>STAR:</b> %1").arg(m_fpnStar));
        }
        if (!m_fpnApproach.isEmpty()) {
            lines.append(QString("<b>Approach:</b> %1").arg(m_fpnApproach));
        }
        if (!m_fpnArrival.isEmpty()) {
            lines.append(QString("<b>Arrival:</b> %1").arg(icaoAndAirportName(m_fpnArrival)));
        }
        if (!m_fpnAlternate.isEmpty()) {
            lines.append(QString("<b>Alternate:</b> %1").arg(icaoAndAirportName(m_fpnAlternate)));
        }
    }
    if (m_positionValid && !m_posAdditionalData.isEmpty()) {
        lines.append(QString("<b>Additional data:</b> %1").arg(m_posAdditionalData));
    }
    if (lines.isEmpty()) {
        return "";
    }
    if (!m_fmsSublabel.isEmpty()) {
        lines.prepend(QString("<b>Subsystem:</b> %1").arg(m_fmsSublabel));
    }
    return lines.join("<br>");
}

bool AcarsMessageFromSubsystem::getPosition(float& latitude, float& longitude, int& altitudeFt) const
{
    if (!m_positionValid) {
        return false;
    }
    latitude = m_latitude;
    longitude = m_longitude;
    altitudeFt = m_altitudeFt;
    return true;
}

AcarsOutReport::AcarsOutReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsOutReport::decode(const QString &text)
{
    m_departure = text.mid(0, 4);
    m_destination = text.mid(4, 4);
    m_outTimeHours = text.mid(8, 2).toInt();
    m_outTimeMinutes = text.mid(10, 2).toInt();
    m_fuel = text.mid(12, 4).toInt();
    m_boardedFuel = text.mid(16, 4); // Can be -----
    m_freeText = text.mid(20, text.length() - 1).trimmed();
    return true;
}

QString AcarsOutReport::toString()
{
    QString s = QString("<b>Departure:</b> %1<br><b><b>Destination:</b> %2<br><b><b>Out time:</b> %3:%4<br><b><b>Fuel:</b> %5<br><b><b>Boarded fuel:</b> %6")
                    .arg(icaoAndAirportName(m_departure))
                    .arg(icaoAndAirportName(m_destination))
                    .arg(m_outTimeHours, 2, 10, QChar('0'))
                    .arg(m_outTimeMinutes, 2, 10, QChar('0'))
                    .arg(m_fuel)
                    .arg(m_boardedFuel);
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}

AcarsOffReport::AcarsOffReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsOffReport::decode(const QString &text)
{
    m_departure = text.mid(0, 4);
    m_destination = text.mid(4, 4);
    m_offTimeHours = text.mid(8, 2).toInt();
    m_offTimeMinutes = text.mid(10, 2).toInt();
    m_freeText = text.mid(12, text.length() - 1).trimmed();
    return true;
}

QString AcarsOffReport::toString()
{
    QString s = QString("<b>Departure:</b> %1<br><b>Destination:</b> %2<br><b>Off time:</b> %3:%4")
                    .arg(icaoAndAirportName(m_departure))
                    .arg(icaoAndAirportName(m_destination))
                    .arg(m_offTimeHours, 2, 10, QChar('0'))
                    .arg(m_offTimeMinutes, 2, 10, QChar('0'));
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}

AcarsOnReport::AcarsOnReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsOnReport::decode(const QString &text)
{
    m_departure = text.mid(0, 4);
    m_destination = text.mid(4, 4);
    m_onTimeHours = text.mid(8, 2).toInt();
    m_onTimeMinutes = text.mid(10, 2).toInt();
    m_freeText = text.mid(12, text.length() - 1).trimmed();
    return true;
}

QString AcarsOnReport::toString()
{
    QString s = QString("<b>Departure:</b> %1<br><b>Destination:</b> %2<br><b>On time:</b> %3:%4")
                    .arg(icaoAndAirportName(m_departure))
                    .arg(icaoAndAirportName(m_destination))
                    .arg(m_onTimeHours, 2, 10, QChar('0'))
                    .arg(m_onTimeMinutes, 2, 10, QChar('0'));
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}

AcarsInReport::AcarsInReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsInReport::decode(const QString &text)
{
    m_departure = text.mid(0, 4);
    m_destination = text.mid(4, 4);
    m_inTimeHours = text.mid(8, 2).toInt();
    m_inTimeMinutes = text.mid(10, 2).toInt();
    m_fuel = text.mid(12, 4).toInt();
    m_captainFirstOfficerId = text[16];
    m_landingCategory = text[17];
    m_freeText = text.mid(18, text.length() - 1).trimmed();
    return true;
}

QString AcarsInReport::toString()
{
    QString s = QString("<b>Departure:</b> %1<br><b>Destination:</b> %2<br><b>In time:</b> %3:%4<br><b>Fuel:</b> %5<br>Id: %6<br><b>Landing category:</b> %7")
                    .arg(icaoAndAirportName(m_departure))
                    .arg(icaoAndAirportName(m_destination))
                    .arg(m_inTimeHours, 2, 10, QChar('0'))
                    .arg(m_inTimeMinutes, 2, 10, QChar('0'))
                    .arg(m_fuel)
                    .arg(m_captainFirstOfficerId)
                    .arg(m_landingCategory);
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}

AcarsOutReturnInReport::AcarsOutReturnInReport(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsOutReturnInReport::decode(const QString &text)
{
    m_departure = text.mid(0, 4);
    m_destination = text.mid(4, 4);
    m_outTimeHours = text.mid(8, 2).toInt();
    m_outTimeMinutes = text.mid(10, 2).toInt();
    m_returnInTimeHours = text.mid(12, 2).toInt();
    m_returnInTimeMinutes = text.mid(14, 2).toInt();
    m_fuel = text.mid(16, 4).toInt();
    m_freeText = text.mid(20, text.length() - 1).trimmed();
    return true;
}

QString AcarsOutReturnInReport::toString()
{
    QString s = QString("<b>Departure:</b> %1<br><b>Destination:</b> %2<br><b>Out time:</b> %3:%4<br><b>Return in time:</b> %5:%6<br><b>Fuel:</b> %7")
                    .arg(icaoAndAirportName(m_departure))
                    .arg(icaoAndAirportName(m_destination))
                    .arg(m_outTimeHours, 2, 10, QChar('0'))
                    .arg(m_outTimeMinutes, 2, 10, QChar('0'))
                    .arg(m_returnInTimeHours, 2, 10, QChar('0'))
                    .arg(m_returnInTimeMinutes, 2, 10, QChar('0'))
                    .arg(m_fuel);
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}

// ARINC 620
AcarsSquitter::AcarsSquitter(const QByteArray &message) :
    AcarsMessage(message)
{
}

bool AcarsSquitter::decode(const QString &text)
{
    if (m_uplink)
    {
        m_version = text.mid(0, 2);
        m_serviceProvider = text.mid(2, 2);
        if (m_version == "00")
        {
            m_freeText = text.mid(4);
        }
        else
        {
            m_iataStationId = text.mid(4, 3);
            m_icaoStationId = text.mid(7, 4);
            m_stationNumber = text.mid(11, 1);
            if (m_version == "01")
            {
                m_freeText = text.mid(12);
            }
            else
            {
                m_latitude = text.mid(12, 5);
                m_longitude = text.mid(17, 6);
                int idx = 23;
                while ((text[idx] != '/') && (idx < text.length()))
                {
                    if ((text[idx] == 'V') || (text[idx] == 'A') || (text[idx] == 'B'))
                    {
                        m_alternateService.append(text[idx]);
                        m_alternateFrequency.append(QString("%1.%2kHz")
                                                            .arg(text.mid(idx+1,3))
                                                            .arg(text.mid(idx+4,3)));
                        idx += 7;
                    }
                    else
                    {
                        break;
                    }
                }
                idx++;
                m_freeText = text.mid(idx);
            }
        }
    }
    return true;
}

QString AcarsSquitter::toString()
{
    const QHash<QString, QString> service = {
        {"A", "VDL Mode 2 ATN Only"},
        {"B", "VDL Mode 2 AOA and ATN"},
        {"V", "VDL Mode 2 AOA Only"},
    };
    const QHash<QString, QString> serviceProviders = {
        {"XA", "Rockwell Collins IMS"},
        {"XS", "SITA VHF"},
        {"AS", "Honeywell"},
    };
    QString serviceProvider = m_serviceProvider;
    if (serviceProviders.contains(m_serviceProvider)) {
        serviceProvider = QString("%1 (%2)").arg(m_serviceProvider).arg(serviceProviders.value(m_serviceProvider));
    }
    QString s = QString("<b>Version:</b> %1<br><b>Service provider:</b> %2")
                    .arg(m_version)
                    .arg(serviceProvider);
    if (m_version != "00")
    {
        s.append(QString("<br><b>IATA station Id:</b> %1<br><b>ICAO station Id:</b> %2<br><b>Station number:</b> %3")
                        .arg(m_iataStationId)
                        .arg(icaoAndAirportName(m_icaoStationId))
                        .arg(m_stationNumber));
        if (m_version != "01")
        {
            s.append(QString("<br><b>Latitude:</b> %1<br><b>Longitude:</b> %2")
                            .arg(m_latitude)
                            .arg(m_longitude));
            for (int i = 0; i < m_alternateService.size(); i++)
            {
                s.append(QString("<br><b>Alternate service:</b> %1 - %2")
                            .arg(service[m_alternateService[i]])
                            .arg(m_alternateFrequency[i]));
            }
        }
    }
    if (!m_freeText.isEmpty()) {
        s.append(QString("<br><b>Free text:</b> %1").arg(m_freeText));
    }
    return s;
}
