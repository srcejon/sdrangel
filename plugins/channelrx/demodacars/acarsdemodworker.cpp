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

#include <mutex>

#include <QMutex>
#include <QMutexLocker>

#include <QDebug>
#include <QCoreApplication>
#include <QJsonObject>
#include <QJsonDocument>
#include <QHostInfo>

#include "dsp/dspcommands.h"
#include "util/db.h"
#include "util/units.h"
#include "maincore.h"

#include "SWGMapItem.h"

#ifdef _MSC_VER
// For libacars/asn1/asn_system.h
#define ASSUMESTDTYPES
#define	ssize_t		SSIZE_T
#endif

#include <libacars/asn1/FANSATCDownlinkMessage.h>
#include <libacars/asn1/FANSATCDownlinkMsgElementId.h>
#include <libacars/asn1/FANSATCDownlinkMsgElementIdSequence.h>
#include <libacars/asn1/FANSLatitudeLongitude.h>
#include <libacars/asn1/FANSPositionCurrent.h>
#include <libacars/asn1/FANSPositionReport.h>
#include <libacars/libacars.h>
#include <libacars/acars.h>
#include <libacars/vstring.h>
#include <libacars/adsc.h>
#include <libacars/cpdlc.h>
#include <libacars/arinc.h>
#include <libacars/reassembly.h>

#ifdef _MSC_VER
// asn_system.h maps these C99 functions to pre-C99 MSVC CRT names; the macros
// break later standard headers (e.g. <random> uses std::ilogb)
#undef isnan
#undef finite
#undef copysign
#undef ilogb
#endif

#include "acarsdemod.h"
#include "acarsdemodworker.h"
#include "acarstextformat.h"

// Airport information used by AcarsMessage decoders (file-scope in
// acarsmessage.cpp); initialised in startWork()
extern QSharedPointer<const QHash<QString, AirportInformation *>> m_airportInfo;

MESSAGE_CLASS_DEFINITION(AcarsDemodWorker::MsgConfigureWorker, Message)

// Convert CPDCL altitude in to feet
static int cpdlcAltitude(FANSAltitude_t altitude)
{
	switch(altitude.present)
    {
	case FANSAltitude_PR_altitudeQNH:
		return altitude.choice.altitudeQNH * 10;
	case FANSAltitude_PR_altitudeQNHMeters:
		return Units::metresToIntegerFeet(altitude.choice.altitudeQNHMeters);
	case FANSAltitude_PR_altitudeQFE:
		return altitude.choice.altitudeQFE * 10;
	case FANSAltitude_PR_altitudeQFEMeters:
		return Units::metresToIntegerFeet(altitude.choice.altitudeQFEMeters);
	case FANSAltitude_PR_altitudeGNSSFeet:
		return altitude.choice.altitudeGNSSFeet;
	case FANSAltitude_PR_altitudeGNSSMeters:
		return Units::metresToIntegerFeet(altitude.choice.altitudeGNSSMeters);
	case FANSAltitude_PR_altitudeFlightLevel:
		return altitude.choice.altitudeFlightLevel * 100;
	case FANSAltitude_PR_altitudeFlightLevelMetric:
		return Units::metresToIntegerFeet(altitude.choice.altitudeFlightLevelMetric * 10.0);
	case FANSAltitude_PR_NOTHING:        /* No components present */
	default:
        return -1;
	}
}

static double cpdlcCoordinate(long degrees, long *tenthsofminutes)
{
	double result = (double)degrees;
	if(tenthsofminutes != NULL) {
		result += (double)(*tenthsofminutes) / 10.0f / 60.0f;
	}
	return result;
}

// Extract lat, lon, alt from FANSPositionReport
static bool cpdlcExtractPosition(FANSPositionReport_t rpt, double &latitude, double &longitude, int &altitude)
{
	// FANSPositionCurrent_t can contain various types of position data (fix name, navaid, etc).
	// We want only latitude/longitude coordinates.
	FANSPositionCurrent_t pos = rpt.positioncurrent;
	if(pos.present != FANSPosition_PR_latitudeLongitude) {
		return false;
	}
	FANSLatitudeLongitude_t latlon = pos.choice.latitudeLongitude;
	latitude = cpdlcCoordinate(latlon.latitude.latitudeDegrees, latlon.latitude.minutesLatLon);
	if(latlon.latitude.latitudeDirection == FANSLatitudeDirection_south) {
		latitude = -latitude;
	}
	longitude = cpdlcCoordinate(latlon.longitude.longitudeDegrees, latlon.longitude.minutesLatLon);
	if(latlon.longitude.longitudeDirection == FANSLongitudeDirection_west) {
		longitude = -longitude;
	}
	altitude = cpdlcAltitude(rpt.altitude);

    return true;
}

AcarsDemodWorker::AcarsDemodWorker(AcarsDemod *acarsDemod) :
    m_acarsDemod(acarsDemod)
{
    qRegisterMetaType<AcarsRowEvent>("AcarsRowEvent");
    qRegisterMetaType<AcarsAssemblyEvent>("AcarsAssemblyEvent");

    connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &AcarsDemodWorker::handleInputMessages);
}

AcarsDemodWorker::~AcarsDemodWorker()
{
    if (m_logFile.isOpen())
    {
        m_logStream.flush();
        m_logFile.close();
    }
    if (m_reasmCtx) {
        la_reasm_ctx_destroy(m_reasmCtx);
    }
    m_multipartAssembler.clear();
}

// Runs on the worker thread once it starts
void AcarsDemodWorker::startWork()
{
    // Sockets must be created on the thread that uses them
    m_udpSocket = new QUdpSocket(this);
    m_feedSocket = new QUdpSocket(this);
    m_feedAirframesTcpSocket = new QTcpSocket(this);
    m_feedAvdelphiTcpSocket = new QTcpSocket(this);

    m_reasmCtx = la_reasm_ctx_new();

    // Airport information for the AcarsMessage decoders, plus the user-defined label
    // names.
    //
    // All three of these are PROCESS WIDE - m_airportInfo is a global and the two label
    // hashes are static members - while startWork() runs on every demodulator's OWN
    // worker thread, because it is connected to QThread::started. An unguarded
    // check-then-assign therefore races as soon as two channels are created close
    // together: QSharedPointer assignment is not atomic, and nor is QHash insertion, so
    // the failure mode is a corrupted hash or a double free rather than merely doing the
    // work twice. The base label hashes are initialised at file scope, so only these
    // additions need guarding.
    static std::once_flag sharedDataInit;
    std::call_once(sharedDataInit, []()
    {
        m_airportInfo = OurAirportsDB::getAirportsByIdent();

        // Add user-defined labels
        for (int i = '1'; i <= '4'; i++)
        {
            for (char c = '0'; c <= '~'; c++)
            {
                QString label;
                label.append(QChar(i));
                label.append(QChar(c));
                m_labels.insert(label, "User defined");
                m_subLabels.insert(label, "User defined");
            }
        }
    });

    m_aircraftInfo = OsnDB::getAircraftInformationByReg();
    m_aircraftInfoByIcao = OsnDB::getAircraftInformation();

    loadVdl2Gs();
}

// A different RF channel or protocol: nothing partially reassembled from the old one may
// combine with new traffic. That means ALL THREE reassembly layers, not just ATN - the
// ACARS multipart assembler holds partial ETB chains, and libacars keeps its own
// reassembly context for fragmented messages. Clearing only the ATN decoder left the
// other two to splice a half message from one frequency onto traffic from another.
void AcarsDemodWorker::resetReassembly()
{
    m_atnDecoder.clear();
    m_multipartAssembler.clear();
    if (m_reasmCtx)
    {
        la_reasm_ctx_destroy(m_reasmCtx);
        m_reasmCtx = la_reasm_ctx_new();
    }
}

void AcarsDemodWorker::handleInputMessages()
{
    Message* message;
    while ((message = m_inputMessageQueue.pop()))
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void AcarsDemodWorker::forwardToGui(Message *message)
{
    MessageQueue *guiQueue = m_acarsDemod->getMessageQueueToGUI();
    if (guiQueue) {
        guiQueue->push(message);
    } else {
        delete message;
    }
}

bool AcarsDemodWorker::handleMessage(const Message& message)
{
    if (AcarsDemod::MsgProtocolChange::match(message))
    {
        const AcarsDemod::MsgProtocolChange& protocol = (const AcarsDemod::MsgProtocolChange&) message;
        // Arrives in stream order with the packets, so everything after it really did
        // come off this protocol. The sink also emits this on a FORCED settings apply, which
        // happens at startup and on preset load without the mode necessarily changing, so
        // only a genuine change discards what is part way through reassembly.
        if (m_protocolMode != protocol.getMode())
        {
            m_protocolMode = protocol.getMode();
            resetReassembly();
        }
        return true;
    }
    if (MainCore::MsgPacket::match(message))
    {
        MainCore::MsgPacket& report = (MainCore::MsgPacket&) message;
        processPacket(report.getPacket(), report.getDateTime());

        // Forward to APRS and other packet features
        QList<ObjectPipe*> packetsPipes;
        MainCore::instance()->getMessagePipes().getMessagePipes(m_acarsDemod, "packets", packetsPipes);

        for (const auto& pipe : packetsPipes)
        {
            MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
            MainCore::MsgPacket *msg = new MainCore::MsgPacket(report);
            messageQueue->push(msg);
        }

        // Forward via UDP
        if (m_settings.m_udpEnabled && m_udpSocket)
        {
            m_udpSocket->writeDatagram(report.getPacket().data(), report.getPacket().size(),
                                       QHostAddress(m_settings.m_udpAddress), m_settings.m_udpPort);
        }

        // Write to log file
        if (m_logFile.isOpen())
        {
            QByteArray bytes = report.getPacket();
            // One ISO 8601 column, to match the table. It is unambiguous, it sorts
            // chronologically as text - which the locale date string did not - and it
            // keeps the milliseconds the two locale strings threw away
            m_logStream << report.getDateTime().toString(Qt::ISODateWithMs) << ","
                << bytes.toHex()
                << "\n";
        }

        return true;
    }
    else if (AcarsDemod::MsgVdl2Frame::match(message))
    {
        AcarsDemod::MsgVdl2Frame& report = (AcarsDemod::MsgVdl2Frame&) message;
        processVdl2Frame(report.getFrame(), report.getDateTime());
        return true;
    }
    else if (AcarsDemod::MsgAeroFrame::match(message))
    {
        AcarsDemod::MsgAeroFrame& report = (AcarsDemod::MsgAeroFrame&) message;
        processAeroFrame(report.getFrame(), report.getDateTime());
        return true;
    }
    else if (AcarsDemod::MsgHfdlFrame::match(message))
    {
        AcarsDemod::MsgHfdlFrame& report = (AcarsDemod::MsgHfdlFrame&) message;
        processHfdlFrame(report.getFrame(), report.getDateTime());
        return true;
    }
    else if (MsgConfigureWorker::match(message))
    {
        const MsgConfigureWorker& cfg = (const MsgConfigureWorker&) message;
        applySettings(cfg.getSettings(), cfg.getForce());
        return true;
    }
    else if (DSPSignalNotification::match(message))
    {
        const DSPSignalNotification& notif = (const DSPSignalNotification&) message;
        if (m_centerFrequency != notif.getCenterFrequency()) {
            resetReassembly();
        }
        m_centerFrequency = notif.getCenterFrequency();
        return true;
    }
    return false;
}

void AcarsDemodWorker::applySettings(const AcarsDemodSettings& settings, bool force)
{
    if ((settings.m_inputFrequencyOffset != m_settings.m_inputFrequencyOffset)
        || (settings.m_mode != m_settings.m_mode)
        || ((settings.m_mode == AcarsDemodSettings::Aero)
            && (settings.m_aeroChannel != m_settings.m_aeroChannel))) {
        resetReassembly();
    }

    if ((settings.m_logEnabled != m_settings.m_logEnabled)
        || (settings.m_logFilename != m_settings.m_logFilename)
        || force)
    {
        if (m_logFile.isOpen())
        {
            m_logStream.flush();
            m_logFile.close();
        }
        if (settings.m_logEnabled && !settings.m_logFilename.isEmpty())
        {
            m_logFile.setFileName(settings.m_logFilename);
            if (m_logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
            {
                qDebug() << "AcarsDemodWorker::applySettings - Logging to: " << settings.m_logFilename;
                bool newFile = m_logFile.size() == 0;
                m_logStream.setDevice(&m_logFile);
                if (newFile)
                {
                    // Write header
                    m_logStream << "DateTime,Data\n";
                }
            }
            else
            {
                qDebug() << "AcarsDemodWorker::applySettings - Unable to open log file: " << settings.m_logFilename;
            }
        }
    }

    m_settings = settings;

    if (m_feedAirframesTcpSocket) {
        updateFeedConnections();
    }
}

// Process an ACARS message from any of the three modes
void AcarsDemodWorker::processPacket(const QByteArray& packet, QDateTime dateTime)
{
    processAcarsMessage(packet, dateTime);
}

// Whether anything is going to look at what we format.
//
// The channel's GUI message queue is the framework's own test for this, and it is null in
// a server build or for a channel created headlessly. Everything the demodulator exists to
// DO - the UDP forward, the log, the aggregator feeds, the aircraft reports and map items -
// happens either way; what is skipped is only the rendering and the strings kept for the
// decode view, which in a headless run are built, stored and never read.
bool AcarsDemodWorker::hasGui() const
{
    return m_acarsDemod && m_acarsDemod->getMessageQueueToGUI();
}

void AcarsDemodWorker::emitAssemblyState(const QSharedPointer<AcarsMultipartAssembler::Assembly>& assembly)
{
    // The whole of this exists to populate the decode view
    if (!hasGui()) {
        return;
    }

    AcarsAssemblyEvent e;
    e.m_assemblyId = assembly->m_id;
    e.m_isMultipart = assembly->isMultipart();
    e.m_uplink = assembly->m_uplink;
    e.m_statusText = AcarsMultipartAssembler::statusText(assembly->m_status);
    e.m_uniquePartCount = assembly->m_uniquePartCount;
    e.m_duplicateCount = assembly->m_duplicateCount;
    e.m_missingSequences = assembly->m_missingSequences;

    // The "Combined message" section for the decode view: the libacars decode
    // of the complete assembly, our own decoder's text, or the raw combination
    if (!assembly->m_libAcarsDecodedText.isEmpty())
    {
        QString libAcarsDecode = assembly->m_libAcarsDecodedText.toHtmlEscaped();
        libAcarsDecode.replace("\n", "<br>");
        e.m_combinedHtmlBody = libAcarsDecode;
    }
    else if (!assembly->m_decodedText.isEmpty())
    {
        QString builtInDecode = assembly->m_decodedText;
        builtInDecode.replace("\r", "<br>");
        builtInDecode.replace("\n", "<br>");
        e.m_combinedHtmlBody = builtInDecode;
    }
    else
    {
        QString combinedText = assembly->m_combinedText.toHtmlEscaped();
        combinedText.replace("\n", "<br>");
        e.m_combinedHtmlBody = combinedText;
    }

    emit assemblyUpdated(e);
}

// Decode one ACARS message: multipart assembly, our decoders, libacars (ADS-C,
// CPDLC, ARINC 622), aircraft reports, aggregator feeds and the display event
// ACARS labels are two characters and some of them are control codes - the general
// response in demand mode is underscore followed by DEL. Left as-is they render as a
// missing-glyph box, which reads as a decode failure when it is nothing of the sort.
// Substituting the Unicode Control Pictures block shows what the character actually is.
// Display only: the raw label still does the table lookup and still goes out on the feeds.
static QString printableLabel(const QString& label)
{
    QString out;
    for (QChar c : label)
    {
        const ushort u = c.unicode();
        if (u == 0x7F) {
            out.append(QChar(0x2421));          // DEL
        } else if (u < 0x20) {
            out.append(QChar(0x2400 + u));      // NUL through US
        } else {
            out.append(c);
        }
    }
    return out;
}

// The event carries a day of the month and a UTC time but no month or year, so the
// reception time supplies those. Searching a few days either side rather than assuming
// the same month is what makes it work across a month boundary - an OUT at 23:55 on the
// 31st is commonly reported a few minutes later, on the 1st.
static QDateTime oooiDateTime(const QDateTime& received, int day, const QTime& time)
{
    if (!time.isValid() || !received.isValid()) {
        return QDateTime();
    }
    const QDate base = received.toUTC().date();
    if ((day < 1) || (day > 31)) {
        return QDateTime(base, time, Qt::UTC);
    }
    for (int offset = 0; offset <= 2; offset++)
    {
        for (int sign = 1; sign >= -1; sign -= 2)
        {
            const QDate d = base.addDays(sign * offset);
            if (d.day() == day) {
                return QDateTime(d, time, Qt::UTC);
            }
            if (offset == 0) {
                break;
            }
        }
    }
    return QDateTime(base, time, Qt::UTC);
}

// Recognise a weather report inside an ACARS message, and say which airport it is about.
//
// There is no single weather label. Aircraft request weather on 5U and ATIS on 5D/B9, and
// the reply comes back on whatever label the airline's system uses - very often plain
// text. So the label narrows it and the CONTENT decides, which is also what makes a METAR
// forwarded inside a free-text message findable.
//
// Each form is recognised by the shape the WMO and ICAO formats guarantee:
//   METAR  ICAO ddhhmmZ ...            observation, sometimes with a METAR/SPECI prefix
//   TAF    TAF [AMD|COR] ICAO ddhhmmZ  forecast
//   NOTAM  a series beginning with an exclamation mark or the word NOTAM
//   PIREP  UA/ or UUA/ followed by /OV
//   SIGMET ICAO SIGMET|AIRMET n VALID
static int classifyWeather(const QString& label, const QString& text,
                           QString& airport, QString& body)
{
    static const QRegularExpression metarRe(
        R"((?:^|\n)\s*(?:(?:METAR|SPECI)\s+)?([A-Z]{4})\s+\d{6}Z\s)");
    static const QRegularExpression tafRe(
        R"((?:^|\n)\s*TAF(?:\s+(?:AMD|COR|RTD))?\s+([A-Z]{4})\s)");
    static const QRegularExpression sigmetRe(
        R"((?:^|\n)\s*([A-Z]{4})\s+(?:SIGMET|AIRMET)\s)");
    static const QRegularExpression notamRe(
        R"((?:^|\n)\s*(?:NOTAM|!\s*([A-Z]{3,4})))");
    static const QRegularExpression pirepRe(
        R"((?:^|\n)\s*(?:UA|UUA)/.*?/OV\s+([A-Z]{3,4}))");
    static const QRegularExpression atisRe(
        R"(([A-Z]{4})\s+(?:ARR|DEP|ARR/DEP)?\s*ATIS\s+([A-Z])\b)");

    body = text.trimmed();
    if (body.isEmpty()) {
        return AircraftReport::WeatherNone;
    }

    QRegularExpressionMatch m = tafRe.match(body);
    if (m.hasMatch())
    {
        airport = m.captured(1);
        return AircraftReport::Taf;
    }
    m = metarRe.match(body);
    if (m.hasMatch())
    {
        airport = m.captured(1);
        return AircraftReport::Metar;
    }
    m = sigmetRe.match(body);
    if (m.hasMatch())
    {
        airport = m.captured(1);
        return AircraftReport::Sigmet;
    }
    m = pirepRe.match(body);
    if (m.hasMatch())
    {
        airport = m.captured(1);
        return AircraftReport::Pirep;
    }
    m = notamRe.match(body);
    if (m.hasMatch())
    {
        airport = m.captured(1);
        return AircraftReport::Notam;
    }
    m = atisRe.match(body);
    if (m.hasMatch())
    {
        airport = m.captured(1);
        return AircraftReport::Atis;
    }

    // Nothing matched on content. The label still says what the message was FOR, so a
    // terminal weather report is kept as weather even when its body is a format we do
    // not parse - better an unclassified row than a missing one.
    if ((label == "AB") || (label == "BB")) {
        return AircraftReport::Twip;
    }
    if ((label == "A9") || (label == "5D") || (label == "B9")) {
        return AircraftReport::Atis;
    }
    if ((label == "5U") || (label == "WO")) {
        return AircraftReport::WeatherOther;
    }
    return AircraftReport::WeatherNone;
}

void AcarsDemodWorker::processAcarsMessage(QByteArray messageBytes, QDateTime received)
{
    // FIXME: Occasionally last byte is ff instead of 7f - correct, otherwise libacars wont parse
    // Should be fixed in sink
    if (messageBytes[messageBytes.size()-1] == (char)0xff) {
        messageBytes[messageBytes.size()-1] = 0x7f;
    }

    // Decode the message
    AcarsMessage *message = AcarsMessage::decode(messageBytes);
    if (!message)
    {
        qDebug() << "AcarsDemodWorker::processAcarsMessage: Unsupported ACARS message: " << messageBytes;
        return;
    }

    // Owned here and freed on every exit path. It used to be appended to m_messages and
    // kept for the life of the channel, because the multipart assembler identified parts
    // by the message's ADDRESS - so every message ever decoded had to stay allocated for
    // those addresses to remain unique. The assembler now uses an id instead, and nothing
    // outside this function holds the pointer: the row and assembly events carry copies.
    QScopedPointer<AcarsMessage> messageOwner(message);

    message->m_dateTime = received;

    AcarsMultipartAssembler::Part multipartPart;
    multipartPart.m_sourceId = ++m_nextSourceId;
    multipartPart.m_received = received;
    multipartPart.m_uplink = message->m_uplink;
    multipartPart.m_address = message->m_address;
    multipartPart.m_label = message->m_label;
    multipartPart.m_subLabel = message->m_subLabel;
    multipartPart.m_blockId = message->m_blockId;
    multipartPart.m_originator = message->m_originator;
    multipartPart.m_messageNumber = message->m_messageNumber;
    if (!message->m_blockSequence.isEmpty()) {
        multipartPart.m_blockSequence = message->m_blockSequence[0];
    }
    multipartPart.m_text = message->m_text;
    multipartPart.m_more = message->m_multiBlock;

    AcarsMultipartAssembler::Result multipart = m_multipartAssembler.add(multipartPart);
    if (multipart.m_assembly)
    {
        // Do not offer a partial or gap-collapsed payload to an application decoder.
        // Once complete, earlier table rows get the same decode through the assembly.
        if (multipart.m_assembly->m_status == AcarsMultipartAssembler::Complete)
        {
            message->decode(multipart.m_assembly->m_combinedText);
            if (hasGui()) {
                multipart.m_assembly->m_decodedText = message->toString();
            }
        }
        else
        {
            multipart.m_assembly->m_decodedText.clear();
            multipart.m_assembly->m_libAcarsDecodedText.clear();
        }
    }

    feedMessage(*message);

    AcarsRowEvent e;
    e.m_frameType = 0;
    e.m_received = received;
    e.m_uplink = message->m_uplink;

    // Record the aircraft as active for the GUI's chart. Only downlinks show
    // the aircraft itself transmitting.
    if (!message->m_uplink && !message->m_address.isEmpty()) {
        e.m_chartAircraftId = message->m_address;
    }

    // Mode is destination of message
    // 2 = All ground stations / aircraft - Other characters are specific station / aircraft
    // and country specific (the GUI derives the country decode)
    e.m_mode = QString("%1").arg(message->m_mode);

    e.m_address = message->m_address;

    // ACK or NACK - Or block ID it corresponds to
    if (message->m_ack == (char)ASCII_ACK) {
        e.m_ack = "ACK";
    } else if (message->m_ack == (char)ASCII_NAK) {
        e.m_ack = "NAK";
    } else {
        e.m_ack = QString("%1").arg(message->m_ack);
    }

    // Label (type of message)
    QString label = printableLabel(message->m_label);
    if (!message->m_subLabel.isEmpty())
    {
        label.append('/');
        label.append(message->m_subLabel);
        e.m_labelDecode = m_subLabels[message->m_subLabel];
    }
    else
    {
        e.m_labelDecode = m_labels.value(message->m_label);
    }
    // An ACARS block reaches here from every protocol - the VDL-2, HFDL and Aero
    // receivers all reassemble one and push it down this same path - so the link it
    // came off is m_protocolMode, tracked in stream order, and not simply "ACARS"
    switch (m_protocolMode)
    {
    case AcarsDemodSettings::VDL2:
        e.m_protocol = "VDL-2";
        e.m_bitRate = 31500;        // 10500 symbols/s of D8PSK, three bits each
        break;
    case AcarsDemodSettings::HFDL:
        e.m_protocol = "HFDL";
        e.m_bitRate = 0;            // Per frame, and not carried on this path
        break;
    case AcarsDemodSettings::Aero:
        e.m_protocol = "Aero";
        e.m_bitRate = AcarsAeroReceiver::submodeRate(m_settings.m_aeroChannel);
        break;
    default:
        e.m_protocol = "ACARS";
        e.m_bitRate = 2400;
        break;
    }
    e.m_label = label;

    // Block ID
    e.m_blockId = QString("%1").arg(message->m_blockId);

    e.m_originator = message->m_originator;
    e.m_originatorDecode = m_originators[message->m_originator];
    e.m_messageNumber = message->m_messageNumber;
    e.m_blockSequence = message->m_blockSequence;
    e.m_flight = message->m_flight;

    // One sighting for the Aircraft feature, filled in as the message decodes
    // and sent at the end
    AircraftReport report;
    report.m_received = received;
    report.m_flight = message->m_flight;
    report.m_label = label;
    report.m_uplink = message->m_uplink;
    report.m_documentKind = AircraftReport::AcarsText;
    if (message->m_text.contains("LOADSHEET"))
    {
        report.m_documentKind = AircraftReport::Loadsheet;
        report.m_documentTitle = "Loadsheet";
    }
    // The address is only a registration when it is a real one - HFDL/VDL-2
    // link identities show as "AC n"/"GS n"/"Unidentified" forms
    if (!message->m_address.isEmpty() && (message->m_address != "Unidentified")
        && !message->m_address.startsWith("AC ") && !message->m_address.startsWith("GS ")) {
        report.m_registration = message->m_address;
    }

    // Get UNIX time message was received
    struct timeval rxTimeVal;
    rxTimeVal.tv_sec = received.toSecsSinceEpoch();
    rxTimeVal.tv_usec = 0;

    // Tell libacars which protocol this came off. It changes the reassembly rules, and
    // until now nothing set it at all, so HFDL and VDL-2 were both being parsed as
    // plain VHF ACARS. There is no VDL-2 constant because VDL-2 *is* a VHF protocol.
    //
    // The protocol comes from m_protocolMode, stamped by the sink into the packet queue, NOT
    // from m_settings - the two are configured through independent queues, so a packet
    // demodulated under the old mode can arrive after the worker's settings have moved on.
    //
    // la_config_set_int writes a PROCESS WIDE setting with no per-context alternative in
    // the API, so two ACARS demodulators in different modes would otherwise race between
    // the set and the parse. The lock below spans both, which serialises parsing across
    // every ACARS demodulator in the process - acceptable because a parse is microseconds,
    // and the alternative is messages silently reassembled under another channel's rules.
    static QMutex protocolMutex;

    la_proto_node *node;
    {
    QMutexLocker protocolLock(&protocolMutex);

    switch (m_protocolMode)
    {
    case AcarsDemodSettings::HFDL:
        la_config_set_int("acars_bearer", LA_ACARS_BEARER_HFDL);
        break;
    case AcarsDemodSettings::Aero:
        la_config_set_int("acars_bearer", LA_ACARS_BEARER_SATCOM);
        break;
    default:
        la_config_set_int("acars_bearer", LA_ACARS_BEARER_VHF);
        break;
    }

    // Decode message with libacars
    node = la_acars_parse_and_reassemble((uint8_t *)messageBytes.data()+1, messageBytes.length() - 1,
                                         message->m_uplink ? LA_MSG_DIR_GND2AIR : LA_MSG_DIR_AIR2GND,
                                         m_reasmCtx, rxTimeVal);
    }   // protocol lock released: it only has to span the set and the parse
    if (node != nullptr)
    {
        // Decode binary message data using libacars (FANS 1/A ADS-C & CPDLC, MAIM, Media Advisory)
        if (node->next)
        {
            la_vstring *vstr = la_proto_tree_format_text(nullptr, node->next);
            QString full = QString(vstr->str).trimmed();
            // Column: flattened to one line; the full multi-line decode is kept
            // for the decode view and Map popups
            e.m_textDecode = acarsDecodeToColumn(full);
            e.m_fullDecode = full;
            if (multipart.m_assembly
                && (multipart.m_assembly->m_status == AcarsMultipartAssembler::Complete))
            {
                if (hasGui()) {
                    multipart.m_assembly->m_libAcarsDecodedText = vstr->str;
                }
            }
            e.m_useLibacars = true;
            la_vstring_destroy(vstr, true);
        }
        else
        {
            // Our decoder for stuff libacars doesn't support, such as B1/B2/QQ
            QString full = acarsDecodeToPlain(message->toString());
            e.m_textDecode = acarsDecodeToColumn(full);
            e.m_fullDecode = full;
            e.m_useLibacars = false;
        }

        // ARINC 622 decode
        la_proto_node *arincNode = la_proto_tree_find_arinc(node);
        if (arincNode)
        {
            la_arinc_msg *arinc = (la_arinc_msg *)arincNode->data;
            if (arinc)
            {
                // Use our decode if libacars doesn't support the IMI. E.g. TI2
                if (arinc->imi == ARINC_MSG_UNKNOWN)
                {
                    QString full = acarsDecodeToPlain(message->toString());
                    e.m_textDecode = acarsDecodeToColumn(full);
                    e.m_fullDecode = full;
                    e.m_useLibacars = false;
                }

                // Extract ATC comms
                if ((arinc->imi == ARINC_MSG_AT1) || (arinc->imi == ARINC_MSG_CR1))
                {
                    // Use our decoder rather than libacars for ATC, as it expands messages, rather than displays in hierarchical form
                    AcarsATCCommunications *atcCommunications = dynamic_cast<AcarsATCCommunications *>(message);
                    if (atcCommunications && !atcCommunications->m_622.m_do219.isEmpty()) {
                        e.m_atc = atcCommunications->m_622.m_do219;
                    }
                    AcarsMessageFromSubsystem *sub = dynamic_cast<AcarsMessageFromSubsystem *>(message);
                    if (sub && !sub->m_622.m_do219.isEmpty()) {
                        e.m_atc = sub->m_622.m_do219;
                    }
                    report.m_documentKind = AircraftReport::Cpdlc;
                    report.m_documentTitle = "CPDLC";
                    if (!e.m_atc.isEmpty())
                    {
                        report.m_documentText = e.m_atc;
                        report.m_atc = e.m_atc;
                    }
                    if (strlen(arinc->gs_addr) > 0) {
                        report.m_station = QString(arinc->gs_addr).trimmed();
                    }
                }
            }
        }

        // Look for ADS-C data. The tag numbers are direction dependent - in a
        // downlink these are position reports, but in an uplink the same tags
        // are contract requests, whose data is not a report struct - so only
        // extract positions from downlinks.
        la_proto_node *adscNode = la_proto_tree_find_adsc(node);
        if (adscNode && !message->m_uplink)
        {
            la_adsc_msg_t *adsc = (la_adsc_msg_t *)adscNode->data;
            if (adsc && !adsc->err)
            {
            	la_list *l = adsc->tag_list;
            	la_adsc_basic_report_t *rpt = NULL;
            	while(l != NULL)
                {
            		la_adsc_tag_t *tag_struct = (la_adsc_tag_t *)l->data;
            		uint8_t t = tag_struct->tag;
            		// Look for a Basic Report
            		if(t == 7 || t == 9 || t == 10 || t == 18 || t == 19 || t == 20)
                    {
            			rpt = (la_adsc_basic_report_t *)tag_struct->data;
                        e.m_hasPosition = true;
                        e.m_hasAltitude = true;
                        e.m_latitude = rpt->lat;
                        e.m_longitude = rpt->lon;
                        e.m_altitudeFt = rpt->alt;
                        // Position for the Aircraft feature
                        report.m_positionValid = true;
                        report.m_latitude = rpt->lat;
                        report.m_longitude = rpt->lon;
                        report.m_altitudeValid = true;
                        report.m_altitudeFt = rpt->alt;
                        report.m_documentKind = AircraftReport::PositionReport;
            			break;
            		}
            		l = la_list_next(l);
                }
            }
        }

        // Look for position in ADS Position report / CPDLC data
        la_proto_node *cpdlcNode = la_proto_tree_find_cpdlc(node);
        if (cpdlcNode)
        {
            la_cpdlc_msg *cpdlc = (la_cpdlc_msg *)cpdlcNode->data;
            if (cpdlc && !cpdlc->err)
            {
                if (!strcmp(cpdlc->asn_type->name, "FANSATCDownlinkMessage"))
                {
                    FANSATCDownlinkMessage_t *dm = (FANSATCDownlinkMessage_t *)cpdlc->data;
                    if (dm)
                    {
                        bool positionValid = false;
                        double latitude, longitude;
                        int altitudeFt;
                        if(dm->aTCDownlinkmsgelementid.present == FANSATCDownlinkMsgElementId_PR_dM48PositionReport)
                        {
                    		positionValid = cpdlcExtractPosition(dm->aTCDownlinkmsgelementid.choice.dM48PositionReport, latitude, longitude, altitudeFt);
                        }
                        else
                        {
                            FANSATCDownlinkMsgElementIdSequence_t *dmeid_seq = dm->aTCdownlinkmsgelementid_seqOf;
                            if(dmeid_seq != NULL)
                            {
                                for(int i = 0; i < dmeid_seq->list.count; i++)
                                {
                                    FANSATCDownlinkMsgElementId_t *dmeid_ptr = (FANSATCDownlinkMsgElementId_t *)dmeid_seq->list.array[i];
                                    if (dmeid_ptr != NULL && dmeid_ptr->present == FANSATCDownlinkMsgElementId_PR_dM48PositionReport)
                                    {
                                        positionValid = cpdlcExtractPosition(dmeid_ptr->choice.dM48PositionReport, latitude, longitude, altitudeFt);
                                        break;
                                    }
                                }
                            }
                        }
                        if (positionValid)
                        {
                            e.m_hasPosition = true;
                            e.m_hasAltitude = true;
                            e.m_latitude = latitude;
                            e.m_longitude = longitude;
                            e.m_altitudeFt = altitudeFt;
                            // Position for the Aircraft feature
                            report.m_positionValid = true;
                            report.m_latitude = latitude;
                            report.m_longitude = longitude;
                            report.m_altitudeValid = true;
                            report.m_altitudeFt = altitudeFt;
                            report.m_documentKind = AircraftReport::Cpdlc;
                        }
                    }
                }
            }
        }

    	la_proto_tree_destroy(node);
	}
    else
    {
        e.m_textDecode = "Failed to la_acars_parse_and_reassemble";
    }

    // Weather. An ATIS report has already been parsed and knows its own airport; every
    // other form is recognised from the text.
    {
        QString wxAirport, wxBody;
        if (AcarsATISReport *atis = dynamic_cast<AcarsATISReport *>(message))
        {
            report.m_weatherKind = AircraftReport::Atis;
            report.m_weatherAirport = atis->m_airport;
            report.m_weatherText = atis->m_atisInformation.isEmpty()
                                 ? message->m_text : atis->m_atisInformation;
        }
        else
        {
            const int kind = classifyWeather(message->m_label, message->m_text,
                                             wxAirport, wxBody);
            if (kind != AircraftReport::WeatherNone)
            {
                report.m_weatherKind = kind;
                report.m_weatherAirport = wxAirport;
                report.m_weatherText = wxBody;
            }
        }
    }

    // Route facts for the Aircraft feature: departure/arrival airports and
    // flight plan or oceanic clearance routes, from whichever decoder saw them
    {
        QString dep, arr, route;
        if (AcarsFlightStatus *fs = dynamic_cast<AcarsFlightStatus *>(message))
        {
            dep = fs->m_departureAirport;
            arr = fs->m_arrivalAirport;
        }
        else if (AcarsEventReport *ev = dynamic_cast<AcarsEventReport *>(message))
        {
            dep = ev->m_departureAirport;
            arr = ev->m_arrivalAirport;
            report.m_documentKind = AircraftReport::OooiEvent;

            // Labels 13 to 16 are the four events proper; 17 and 18 are follow-up
            // reports of the same ones and carry no new time
            if ((ev->m_event >= AircraftReport::OooiOut)
                && (ev->m_event <= AircraftReport::OooiIn))
            {
                report.m_oooiEvent = ev->m_event;
                report.m_oooiTime = oooiDateTime(report.m_received, ev->m_day, ev->m_eventTime);
            }
        }
        else if (AcarsPositionReport *pr = dynamic_cast<AcarsPositionReport *>(message))
        {
            dep = pr->m_departureAirport;
            arr = pr->m_arrivalAirport;
        }
        else if (AcarsOpsReport *ops = dynamic_cast<AcarsOpsReport *>(message))
        {
            dep = ops->m_departureAirport;
            arr = ops->m_arrivalAirport;
        }
        else if (AcarsMessageFromSubsystem *sub = dynamic_cast<AcarsMessageFromSubsystem *>(message))
        {
            dep = sub->m_fpnDeparture;
            arr = sub->m_fpnArrival;
            route = sub->m_fpnRoute;
            for (const auto& waypoint : sub->m_fpnWaypoints)
            {
                AircraftReport::RouteWaypoint point;
                point.m_name = waypoint.m_name;
                point.m_positionValid = waypoint.m_hasPosition;
                point.m_latitude = waypoint.m_latitude;
                point.m_longitude = waypoint.m_longitude;
                report.m_routeWaypoints.append(point);
            }
            if (!route.isEmpty())
            {
                report.m_documentKind = AircraftReport::FlightPlan;
                report.m_documentTitle = "Flight plan";
            }
        }
        else if (AcarsOceanicClearanceReadback *ocr = dynamic_cast<AcarsOceanicClearanceReadback *>(message))
        {
            arr = ocr->m_destination;
            route = QStringList({ocr->m_entryPoint, ocr->m_route}).join(" ").trimmed();
            report.m_documentKind = AircraftReport::Clearance;
            report.m_documentTitle = "Oceanic clearance readback";
        }
        else if (AcarsOceanicClearanceResponse *oc = dynamic_cast<AcarsOceanicClearanceResponse *>(message))
        {
            arr = oc->m_destination;
            route = QStringList({oc->m_entryPoint, oc->m_route}).join(" ").trimmed();
            report.m_documentKind = AircraftReport::Clearance;
            report.m_documentTitle = "Oceanic clearance";
        }
        report.m_departure = dep;
        report.m_arrival = arr;
        report.m_route = route;
    }

    // Aircraft position from any decoded message that carries one (flight
    // status, position reports, OOOI events, FMS reports)
    float posLatitude, posLongitude;
    int posAltitudeFt;
    if (message->getPosition(posLatitude, posLongitude, posAltitudeFt))
    {
        e.m_hasPosition = true;
        e.m_latitude = posLatitude;
        e.m_longitude = posLongitude;
        if (posAltitudeFt >= 0)
        {
            e.m_hasAltitude = true;
            e.m_altitudeFt = posAltitudeFt;
        }
        // Position for the Aircraft feature
        report.m_positionValid = true;
        report.m_latitude = posLatitude;
        report.m_longitude = posLongitude;
        if (posAltitudeFt >= 0)
        {
            report.m_altitudeValid = true;
            report.m_altitudeFt = posAltitudeFt;
        }
    }

    // The clearance route reaches the Map through the Aircraft feature, which draws
    // flight plans and clearances alike - see AircraftTracker::sendRouteToMap(). The
    // route itself went out above as report.m_route, and the destination as m_arrival.

    e.m_text = message->m_text;
    e.m_hex = QString(messageBytes.toHex());

    // Report the sighting to the Aircraft feature: the expanded ATC text when
    // set above, otherwise the full multi-line decode, otherwise the raw text
    if (report.m_documentText.isEmpty()) {
        report.m_documentText = e.m_fullDecode;
    }
    if (report.m_documentText.isEmpty()) {
        report.m_documentText = message->m_text;
    }
    if (!report.m_registration.isEmpty() || !report.m_flight.isEmpty()) {
        sendAircraftReport(report);
    }

    // Precompute the decode-view HTML for non-multipart rows, mirroring the
    // view's old fallback chain
    if (!e.m_fullDecode.isEmpty() && hasGui())
    {
        e.m_viewDecodeHtml = acarsDecodeToHtml(e.m_fullDecode);
    }
    else if (!e.m_textDecode.isEmpty() && !e.m_useLibacars)
    {
        QString html = message->toString();
        html.replace("\r", "<br>");
        e.m_viewDecodeHtml = html;
    }
    else if (e.m_textDecode.isEmpty())
    {
        e.m_viewDecodeHtml = message->m_text;
    }
    else
    {
        e.m_viewDecodeHtml = e.m_textDecode;
    }

    // Multipart rows share their assembly id; the assembly state event lets the
    // decode view show the evolving combined message for every part's row
    if (multipart.m_assembly)
    {
        e.m_assemblyId = multipart.m_assembly->m_id;
        e.m_partNumber = multipart.m_partNumber;
    }

    emit rowReady(e);

    if (multipart.m_assembly) {
        emitAssemblyState(multipart.m_assembly);
    }
}

// Feed a received ACARS message to the enabled aggregators, as one acarsdec-style
// JSON object per UDP datagram - the format both airframes.io (port 5550) and
// avdelphi.com (vdlm2dec JSON, port 5556) ingest. Messages from all three modes
// pass through here, since VDL-2 and HFDL ACARS arrive on the same path.
void AcarsDemodWorker::feedMessage(const AcarsMessage& message)
{
    if (!m_settings.m_feedEnabled) {
        return;
    }

    QJsonObject json;
    json.insert("timestamp", message.m_dateTime.toMSecsSinceEpoch() / 1000.0);
    if (!m_settings.m_feedStationId.isEmpty()) {
        json.insert("station_id", m_settings.m_feedStationId);
    }
    json.insert("channel", 0);
    json.insert("freq", (m_centerFrequency + m_settings.m_inputFrequencyOffset) / 1e6);
    json.insert("level", CalcDb::dbPower(m_acarsDemod->getMagSq()));
    json.insert("error", 0);
    json.insert("mode", message.m_mode);
    json.insert("label", message.m_label);
    json.insert("block_id", QString(message.m_blockId));
    if ((message.m_ack == QChar(ASCII_NAK)) || message.m_ack.isNull()) {
        json.insert("ack", false);
    } else {
        json.insert("ack", QString(message.m_ack));
    }
    json.insert("tail", message.m_address);
    if (!message.m_flight.isEmpty()) {
        json.insert("flight", message.m_flight);
    }
    QString msgNo = message.m_originator + message.m_messageNumber + message.m_blockSequence;
    if (!msgNo.isEmpty()) {
        json.insert("msgno", msgNo);
    }
    json.insert("text", message.m_text);
    json.insert("end", !message.m_multiBlock);
    // Identify the client, as the standalone decoders do - aggregators use this to
    // recognise the source
    QJsonObject app;
    app.insert("name", "SDRangel");
    app.insert("ver", QCoreApplication::applicationVersion());
    json.insert("app", app);

    QByteArray datagram = QJsonDocument(json).toJson(QJsonDocument::Compact);
    datagram.append('\n');

    // UDP: host names are resolved once and cached; the cache invalidates when the
    // setting changes, so a blocking DNS lookup happens at most once per host
    auto sendUdp = [this, &datagram](const QString& host, int port, QHostAddress& cache, QString& cachedHost)
    {
        if (host.isEmpty()) {
            return;
        }
        if (host != cachedHost)
        {
            cachedHost = host;
            cache = QHostAddress(host);
            if (cache.isNull())
            {
                QHostInfo info = QHostInfo::fromName(host);
                cache = info.addresses().isEmpty() ? QHostAddress() : info.addresses().first();
                if (cache.isNull()) {
                    qWarning() << "AcarsDemodWorker: feed cannot resolve host" << host;
                } else {
                    qDebug() << "AcarsDemodWorker: feed sending UDP to" << host
                             << "(" << cache.toString() << ") :" << port;
                }
            }
        }
        if (!cache.isNull()) {
            m_feedSocket->writeDatagram(datagram, cache, port);
        }
    };

    // TCP, as the airframes.io station wizard assigns: a newline-delimited JSON
    // stream, exactly what vdlm2dec/acarsdec write in their -j network modes. The
    // connection is made on demand and re-made after a drop or a settings change;
    // a message arriving while unconnected starts the connect and is itself
    // dropped, which matches how the standalone feeder clients behave.
    auto sendTcp = [&datagram](QTcpSocket& socket, const QString& host, int port, const QString& service)
    {
        if (host.isEmpty()) {
            return;
        }
        bool wrongPeer = (socket.peerName() != host) || (socket.peerPort() != port);
        if ((socket.state() == QAbstractSocket::ConnectedState) && !wrongPeer)
        {
            qint64 written = socket.write(datagram);
            qDebug() << "AcarsDemodWorker: feed sent" << written << "bytes to" << service;
        }
        else if ((socket.state() == QAbstractSocket::UnconnectedState) || wrongPeer)
        {
            qDebug() << "AcarsDemodWorker: feed not connected to" << service
                     << "- reconnecting, message dropped";
            socket.abort();
            socket.connectToHost(host, port);
        }
        else
        {
            qDebug() << "AcarsDemodWorker: feed still connecting to" << service
                     << "(state" << socket.state() << ") - message dropped";
        }
    };

    if (m_settings.m_feedAirframes)
    {
        if (m_settings.m_feedAirframesTcp) {
            sendTcp(*m_feedAirframesTcpSocket, m_settings.m_feedAirframesHost,
                m_settings.m_feedAirframesPort, "airframes.io");
        } else {
            sendUdp(m_settings.m_feedAirframesHost, m_settings.m_feedAirframesPort,
                m_feedAirframesAddr, m_feedAirframesResolvedHost);
        }
    }
    if (m_settings.m_feedAvdelphi)
    {
        if (m_settings.m_feedAvdelphiTcp) {
            sendTcp(*m_feedAvdelphiTcpSocket, m_settings.m_feedAvdelphiHost,
                m_settings.m_feedAvdelphiPort, "avdelphi.com");
        } else {
            sendUdp(m_settings.m_feedAvdelphiHost, m_settings.m_feedAvdelphiPort,
                m_feedAvdelphiAddr, m_feedAvdelphiResolvedHost);
        }
    }
}

// Open (or close) the aggregator connections to match the settings. Connections are
// also re-made on demand when a message needs sending, but connecting eagerly here
// means enabling the feed gives immediate feedback in the log rather than silently
// waiting for the first ACARS message.
void AcarsDemodWorker::updateFeedConnections()
{
    auto update = [this](QTcpSocket& socket, bool enabled, bool tcp, const QString& host, int port)
    {
        bool want = m_settings.m_feedEnabled && enabled && tcp && !host.isEmpty();
        bool wrongPeer = (socket.peerName() != host) || (socket.peerPort() != port);
        if (want)
        {
            if ((socket.state() == QAbstractSocket::UnconnectedState) || wrongPeer)
            {
                qDebug() << "AcarsDemodWorker: feed connecting to" << host << ":" << port;
                socket.abort();
                socket.connectToHost(host, port);
            }
        }
        else if (socket.state() != QAbstractSocket::UnconnectedState)
        {
            socket.abort();
        }
    };
    update(*m_feedAirframesTcpSocket, m_settings.m_feedAirframes, m_settings.m_feedAirframesTcp,
        m_settings.m_feedAirframesHost, m_settings.m_feedAirframesPort);
    update(*m_feedAvdelphiTcpSocket, m_settings.m_feedAvdelphi, m_settings.m_feedAvdelphiTcp,
        m_settings.m_feedAvdelphiHost, m_settings.m_feedAvdelphiPort);
}

// Send one sighting of an aircraft to any Aircraft features listening on the
// "aircraftreport" pipe. The caller fills in the identity, position and decoded
// content; the source description is filled in here from the channel's tuning.
void AcarsDemodWorker::sendAircraftReport(AircraftReport& report)
{
    QList<ObjectPipe*> pipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_acarsDemod, "aircraftreport", pipes);

    if (pipes.size() == 0) {
        return;
    }

    report.m_protocol = AcarsDemodSettings::protocolForMode(m_settings.m_mode);
    report.m_frequency = m_centerFrequency + m_settings.m_inputFrequencyOffset;
    report.m_deviceSetIndex = m_acarsDemod->getDeviceSetIndex();
    report.m_channelIndex = m_acarsDemod->getIndexInDeviceSet();

    for (const auto& pipe : pipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        messageQueue->push(MainCore::MsgAircraftReport::create(m_acarsDemod, report));
    }
}

// Ground stations (HFDL squitters, VDL-2 GSIFs) go straight to the Map as fixed
// antennas; aircraft go via the Aircraft feature
void AcarsDemodWorker::sendGroundStationToMap(const QString& name, float latitude, float longitude, const QString& text, QDateTime dateTime)
{
    QList<ObjectPipe*> mapPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_acarsDemod, "mapitems", mapPipes);

    for (const auto& pipe : mapPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        SWGSDRangel::SWGMapItem *swgMapItem = new SWGSDRangel::SWGMapItem();
        swgMapItem->setName(new QString(name));
        swgMapItem->setLatitude(latitude);
        swgMapItem->setLongitude(longitude);
        swgMapItem->setAltitude(0.0f);
        swgMapItem->setPositionDateTime(new QString(dateTime.toString(Qt::ISODateWithMs)));
        swgMapItem->setFixedPosition(true);
        // The Map feature's antenna icon, as used for its own beacons
        swgMapItem->setImage(new QString(QString("qrc:///map/map/antenna.png")));
        swgMapItem->setText(new QString(text));
        swgMapItem->setLabel(new QString(name));
        swgMapItem->setOrientation(0);
        MainCore::MsgMapItem *msg = MainCore::MsgMapItem::create(m_acarsDemod, swgMapItem);
        messageQueue->push(msg);
    }
}

QHash<QString, QString> AcarsDemodWorker::m_labels = {
    {":;", "Data transceiver auto-tune"},
    {"_j", "Reserved"},
    {":}", "OA to AOA autotune"},
    {"_\x7f", "General response (Demand mode)"},
    {"00", "Hijack situation report"},
    // 10 to 4~ are user defined so will vary with airline - added in constructor
    {"51", "Ground GMT request"},
    {"52", "Ground UTC request"},
    {"54", "Voice contact request"},
    {"57", "Alternate aircrew initiated position report"},
    {"5D", "ATIS request"},
    {"5P", "Temporary suspension"},
    {"5R", "Aircrew initiated position report"},
    {"5U", "Weather request"},
    {"5V", "VDL switch advisory"},
    {"5Y", "Aircrew revision to previous ETA / diversion"},
    {"5Z", "Airline designated downlink"},
    {"7A", "Aircrew initiated engine data / take off thrust report"},
    {"7B", "Aircrew entered miscellaneous message"},
    {"80", "Aircrew-addressed downlinks"},
    {"81", "Aircrew-addressed downlinks"},
    {"82", "Aircrew-addressed downlinks"},
    {"83", "Aircrew-addressed downlinks"},
    {"84", "Aircrew-addressed downlinks"},
    {"85", "Aircrew-addressed downlinks"},
    {"86", "Aircrew-addressed downlinks"},
    {"87", "Aircrew-addressed downlinks"},
    {"88", "Aircrew-addressed downlinks"},
    {"89", "Aircrew-addressed downlinks"},
    {"8A", "Aircrew-addressed downlinks"},
    {"8B", "Aircrew-addressed downlinks"},
    {"8C", "Aircrew-addressed downlinks"},
    {"8D", "Aircrew-addressed downlinks"},
    {"8E", "Aircrew-addressed downlinks"},
    {"8F", "Aircrew-addressed downlinks"},
    {"99", "Emergency message"},
    {"A1", "Oceanic clearance"},
    {"A2", "Unassigned"},
    {"A3", "Departure clearance"},
    {"A4", "Flight systems message"},
    {"A5", "Unassigned"},
    {"A6", "Request ADS reports"},
    {"A7", "Free text from ATC"},
    {"A8", "Deliver departure slot"},
    {"A9", "ATIS report"},
    {"A0", "ATS facilities notification"},
    {"AA", "ATC communications"},
    {"AB", "Terminal weather information"},
    {"AC", "Pushback clearance"},
    {"AD", "Expected taxi clearance"},
    {"AE", "Unassigned"},
    {"AF", "CPC command response"},
    {"AF", "Unassigned"},
    // B0-BZ ATS (Air Traffic Service) Messages - see ARINC 623
    {"B1", "Request Oceanic clearance"},
    {"B2", "Oceanic clearance readback"},
    {"B3", "Request departure clearance"},
    {"B4", "Departure clearance readback"},
    {"B5", "Waypoint position report"},
    {"B6", "Provide ADS report"},
    {"B7", "Free text to ATC"},
    {"B8", "Request departure slot"},
    {"B9", "Request ATIS report"},
    {"B0", "ATS facilities notification"},
    {"BA", "ATC communications"},
    {"BB", "Terminal weather information for pilots"},
    {"BC", "Pushback clearance request"},
    {"BD", "Expected taxi clearance request"},
    {"BE", "CPC aircraft log-on/log-off request"},
    {"BF", "CPC WILCO/UNABLE response"},
    {"BG", "Unassigned"},
    {"C0", "Uplink to all cockpit printers"},
    {"C1", "Uplink to cockpit printer No.1"},
    {"C2", "Uplink to cockpit printer No.2"},
    {"C3", "Uplink to cockpit printer No.3"},
    {"CA", "Printer Status = Error"},
    {"CB", "Printer Status = Busy"},
    {"CC", "Printer Status = Local test mode"},
    {"CD", "Printer Status = Out of paper"},
    {"CE", "Printer Status = Buffer overrun"},
    {"CF", "Printer Status = Reserved"},
    {"DI", "De-icing"},
    {"DL", "Data loading"},
    {"E1", "Internet email message"},
    {"E2", "Internet email message / DSP service"},
    {"EL", "Left Engine monitoring unit messages"},
    {"ER", "Right engine monitoring unit messages"},
    {"F3", "Dedicated transceiver advisory"},
    {"H1", "Message to/from terminal"}, // avionic or airborne subsystem - See table C-2A for sub-label
    {"H2", "Meteorological report"},
    {"H3", "Icing report"},
    {"H4", "Meteorological configuration report"},
    {"HF", "HFDL messages"},                         // Not in ARINC 620
    {"HX", "Undelivered uplink report"},
    {"KB", "Loopback response"},
    {"LB", "Cabin e-logbook"},
    {"LC", "Cabin e-logbook"},
    {"LS", "Techincal (cockpit) e-logbook"},
    {"LT", "Technical (cockpit) e-logbook"},
    {"M1", "IATA departure message"},               // Not in ARINC 620
    {"M2", "IATA arrival message"},                 // Not in ARINC 620
    {"M3", "IATA return to ramp message"},          // Not in ARINC 620
    {"M4", "IATA return from airborne message"},    // Not in ARINC 620
    // P0-P6 and PA-PC are protected using ACARS Message Security
    {"Q0", "Link test"},
    {"Q1", "Departure / Arrival report"},
    {"Q2", "ETA report"},
    {"Q3", "Clock update advisory"},
    {"Q4", "Voice circuit busy"},
    {"Q5", "Unable to deliver uplink messages"},
    {"Q6", "Voice to data channel changeover advisory"},
    {"Q7", "Delay message"},
    {"QA", "Out / Fuel report"},
    {"QB", "Off report"},
    {"QC", "On report"},
    {"QD", "In / Fuel report"},
    {"QE", "Out / Fuel destination report"},
    {"QF", "Off / Fuel destination report"},
    {"QG", "Out / Return in report"},
    {"QH", "Out report"},
    {"QK", "Landing report"},
    {"QL", "Arrival report"},
    {"QM", "Arrival information report"},
    {"QN", "Diversion report"},
    {"QP", "Out report"},
    {"QQ", "Off report"},
    {"QR", "On report"},
    {"QS", "In report"},
    {"QT", "Out/return in report"},
    {"QV", "Autotune reject"},
    {"QX", "Intercept/unable to process"},
    {"RA", "Command / response uplink"},
    {"RB", "Command / response downlink"},
    {"RE", "Refuel - Administrative and general purpose"},
    {"RF", "Refuel - CG targeting"},
    {"S1", "VHF network statistics report"},
    {"S2", "VHF network performance report"},
    {"S3", "LRU configuration report"},
    {"SA", "Media advisory"},
    {"SQ", "Squitter"},
    {"TE", "Turbulence event report"},
    {"UP", "Message acknowledgement"},              // Not in ARINC 620
    // VA-VZ are supplier defined messages
    {"WP", "Potable water remote pre-selection downlinks"},
    // X1-X9 are DSP (Datalink Service Provider) defined
    {"X1", "DSP defined"},
    {"X2", "DSP defined"},
    {"X3", "DSP defined"},
    {"X4", "DSP defined"},
    {"X5", "DSP defined"},
    {"X6", "DSP defined"},
    {"X7", "DSP defined"},
    {"X8", "DSP defined"},
    {"X9", "DSP defined"}
};

QHash<QString, QString> AcarsDemodWorker::m_subLabels = {
    // 10 to 4~ are user defined so will vary with airline - added in constructor
    {"A1", "ADS Unit (ADSU), left"},            // ARINC 622
    {"A2", "ADS Unit (ADSU), right"},           // ARINC 622
    {"AD", "ADS Unit (ADSU), selected"},        // ARINC 622
    {"CF", "Central fault display"},
    {"DF", "Digital flight data acquisition unit"},
    {"EC", "Engine display system"},
    {"EI", "Engine indicating system"},
    {"F1", "Electronic flight bag #1"},
    {"F1", "Electronic flight bag #2"},
    {"H1", "HF data radio, left"},
    {"H2", "HF data radio, right"},
    {"HD", "HF data radio, selected"},
    {"M1", "Flight management computer (FMC), left"},
    {"M2", "Flight management computer (FMC), right"},
    {"M3", "Flight management computer (FMC), center"},
    {"MD", "Flight management computer (FMC), selected"},
    {"OA", "Onboard airport navigation system"},
    {"PS", "Keyboard/Display unit"},
    {"S1", "Satellite data unit (SDU), left"},
    {"S2", "Satellite data unit (SDU), right"},
    {"SD", "Satellite data unit (SDU), selected"},
    {"T0", "All cabin terminals"},
    {"T1", "Cabin terminal 1"},
    {"T2", "Cabin terminal 2"},
    {"T3", "Cabin terminal 3"},
    {"T4", "Cabin terminal 4"},
    {"T5", "Cabin terminal"},
    {"T6", "Cabin terminal"},
    {"T7", "Cabin terminal"},
    {"T8", "Cabin terminal"},
    {"TA", "TCAS/Traffic computer"},
    {"WO", "Weather observation report"},
};

QHash<QString, QString> AcarsDemodWorker::m_originators = {
    {"1", "Cabin terminal 1"},
    {"2", "Cabin terminal 2"},
    {"3", "Cabin terminal 3"},
    {"4", "Cabin terminal 4"},
    {"5", "User terminal"},
    {"6", "User terminal"},
    {"7", "User terminal"},
    {"8", "User terminal"},
    {"A", "TCAS"},
    {"C", "CFDIU"},
    {"D", "DFDAU"},
    {"E", "EICAS/ECAM/EFIS"},
    {"F", "FMC"},
    {"J", "ATSU/ADSU"},
    {"L", "CMU (ATS)"},
    {"M", "CMD (AOC)"},
    {"O", "OAT"},
    {"Q", "SDU"},
    {"S", "System control"},
    {"T", "HF data radio"},
    {"U", "User defined"},
    {"Y", "EFB, left"},
    {"Z", "EFB, right"}
};

// The vendored ground station list is Windows-1252, not UTF-8. A degree sign there is
// the single byte 0xB0, which is not valid UTF-8, so reading it as UTF-8 turned every
// one in to a replacement character - and coordRe below looks for U+00B0, so it never
// matched and NO ground station ever got a position out of the file. Upstream could
// change, so take UTF-8 when the bytes really are UTF-8 and fall back when they are not.
static QString decodeVdl2Gs(const QByteArray& raw)
{
    const QString utf8 = QString::fromUtf8(raw);
    if (!utf8.contains(QChar::ReplacementCharacter)) {
        return utf8;
    }

    // Windows-1252 is Latin-1 except for 0x80-0x9F, where Latin-1 has C1 control codes.
    // Qt 6 dropped QTextCodec, so that range is mapped by hand. The list uses 0x96, an
    // en dash, as well as 0xB0.
    static const ushort cp1252[32] = {
        0x20AC, 0x0081, 0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021,
        0x02C6, 0x2030, 0x0160, 0x2039, 0x0152, 0x008D, 0x017D, 0x008F,
        0x0090, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014,
        0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0x009D, 0x017E, 0x0178
    };
    QString text = QString::fromLatin1(raw);
    for (int i = 0; i < text.size(); i++)
    {
        const ushort u = text.at(i).unicode();
        if ((u >= 0x80) && (u <= 0x9F)) {
            text[i] = QChar(cp1252[u - 0x80]);
        }
    }
    return text;
}

// The community VDL-2 ground station list (vendored from
// https://sdrangel.org/downloads/VDL2_Ground_Stations.txt), mapping 24-bit
// addresses to names, cities and (where listed) coordinates
void AcarsDemodWorker::loadVdl2Gs()
{
    QFile file(":/vdl2gs.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "AcarsDemodWorker::loadVdl2Gs: Failed to open vdl2gs.txt";
        return;
    }
    QRegularExpression lineRe(R"(^([0-9A-Fa-f]{6}) \[([^\]]*)\] \[([^\]]*)\])");
    QRegularExpression coordRe(QString::fromUtf8(R"((\d+)\x{00b0}(\d+)'([NS]) (\d+)\x{00b0}(\d+)'([EW]))"));
    const QStringList lines = decodeVdl2Gs(file.readAll()).split('\n');
    for (const QString& line : lines)
    {
        QRegularExpressionMatch match = lineRe.match(line);
        if (!match.hasMatch()) {
            continue;
        }
        Vdl2GsInfo gs;
        QString name = match.captured(2);
        gs.m_city = match.captured(3);
        QRegularExpressionMatch coord = coordRe.match(name);
        if (coord.hasMatch())
        {
            gs.m_hasPosition = true;
            gs.m_latitude = (coord.captured(1).toFloat() + coord.captured(2).toFloat() / 60.0f)
                            * (coord.captured(3) == "S" ? -1.0f : 1.0f);
            gs.m_longitude = (coord.captured(4).toFloat() + coord.captured(5).toFloat() / 60.0f)
                             * (coord.captured(6) == "W" ? -1.0f : 1.0f);
            name = name.left(coord.capturedStart()).trimmed();
        }
        // Strip a trailing frequency from the name (e.g. "KJFK Kennedy Intl 136.100")
        name.remove(QRegularExpression(R"(\s+\d{3}\.\d{3}\s*$)"));
        gs.m_name = name.trimmed();
        m_vdl2Gs.insert(match.captured(1).toUInt(nullptr, 16), gs);
    }
    qDebug() << "AcarsDemodWorker::loadVdl2Gs: loaded" << m_vdl2Gs.size() << "ground stations";
}

// Decode a VDL-2 AVLC frame that is not an ACARS message (ACARS messages arrive
// via processAcarsMessage like VHF ACARS): the frame summary, the ATN subnetwork
// for information frames and the XID/GSIF link management parameters
void AcarsDemodWorker::processVdl2Frame(const AcarsVdl2Receiver::Frame& frame, QDateTime received)
{
    AcarsRowEvent e;
    e.m_frameType = 1;
    e.m_received = received;
    e.m_uplink = !frame.m_fromAircraft;

    // Record the transmitting aircraft as active for the GUI's chart
    if (frame.m_fromAircraft) {
        e.m_chartAircraftId = QString("%1").arg(frame.m_srcAddress, 6, 16, QChar('0')).toUpper();
    }

    QString type(frame.m_frameType);

    // Supervisory and link management responses flood the table during data
    // transfers; the GUI hides them by default
    static const QStringList noInfoTypes = {"RR", "RNR", "REJ", "SREJ", "DM", "DISC", "UA", "FRMR", "U"};
    e.m_noInfo = noInfoTypes.contains(type);

    // The aircraft side of the link in the Registration column: the registration looked
    // up from the 24-bit ICAO address via the OpenSky database, or the address itself in
    // hex when it is not in the database
    auto addressText = [this](uint32_t address, int type)
    {
        QString hex = QString("%1").arg(address, 6, 16, QChar('0')).toUpper();
        switch (type)
        {
        case 1:                                      // Aircraft
            if (m_aircraftInfoByIcao && m_aircraftInfoByIcao->contains((int) address))
            {
                QString registration = m_aircraftInfoByIcao->value((int) address)->m_registration;
                if (!registration.isEmpty()) {
                    return registration;
                }
            }
            return hex;
        case 4:
        case 5:                                      // Ground station: name from the community list
            if (m_vdl2Gs.contains(address)) {
                return QString("GS %1 (%2)").arg(hex).arg(m_vdl2Gs.value(address).m_name);
            }
            return QString("GS %1").arg(hex);
        case 7: return QString("All");
        default: return hex;
        }
    };

    // A known ground station's coordinates fill the position columns, arming the
    // Map checkbox (which places it as a fixed antenna, like HFDL squitters)
    {
        uint32_t gsAddress = frame.m_fromAircraft ? frame.m_dstAddress : frame.m_srcAddress;
        int gsType = frame.m_fromAircraft ? frame.m_dstType : frame.m_srcType;
        if (((gsType == 4) || (gsType == 5)) && m_vdl2Gs.contains(gsAddress))
        {
            const Vdl2GsInfo& gs = m_vdl2Gs.value(gsAddress);
            if (gs.m_hasPosition)
            {
                e.m_hasPosition = true;
                e.m_latitude = gs.m_latitude;
                e.m_longitude = gs.m_longitude;
            }
        }
    }
    if (frame.m_fromAircraft) {
        e.m_address = addressText(frame.m_srcAddress, frame.m_srcType);
    } else {
        e.m_address = addressText(frame.m_dstAddress, frame.m_dstType);
    }

    e.m_protocol = "VDL-2";
    e.m_bitRate = 31500;
    e.m_label = type;
    e.m_labelDecode = avlcFrameTypeName(frame.m_frameType);

    QString text = QString("%1 %2 %3")
        .arg(addressText(frame.m_srcAddress, frame.m_srcType))
        .arg(QChar(0x2192)) // Right arrow
        .arg(addressText(frame.m_dstAddress, frame.m_dstType));

    // One sighting for the Aircraft feature: the aircraft side of the link,
    // identified by its 24-bit ICAO address
    AircraftReport report;
    report.m_received = received;
    report.m_uplink = !frame.m_fromAircraft;
    {
        uint32_t acAddress = frame.m_fromAircraft ? frame.m_srcAddress : frame.m_dstAddress;
        int acType = frame.m_fromAircraft ? frame.m_srcType : frame.m_dstType;
        if (acType == 1)
        {
            report.m_icao = acAddress;
            if (m_aircraftInfoByIcao && m_aircraftInfoByIcao->contains((int) acAddress)) {
                report.m_registration = m_aircraftInfoByIcao->value((int) acAddress)->m_registration;
            }
        }
        uint32_t gsAddress = frame.m_fromAircraft ? frame.m_dstAddress : frame.m_srcAddress;
        int gsType = frame.m_fromAircraft ? frame.m_dstType : frame.m_srcType;
        if ((gsType == 4) || (gsType == 5))
        {
            // Short form: the airport code from the ground station list rather
            // than the full name/location, falling back to the hex address
            if (m_vdl2Gs.contains(gsAddress) && !m_vdl2Gs.value(gsAddress).m_name.isEmpty()) {
                report.m_station = QString("GS %1").arg(m_vdl2Gs.value(gsAddress).m_name.section(' ', 0, 0));
            } else {
                report.m_station = QString("GS %1").arg(QString("%1").arg(gsAddress, 6, 16, QChar('0')).toUpper());
            }

            // The Mode columns say which ground station the frame used, in the same form
            // the Message column shows it. The community list names them "EGLC London
            // City UK", of which the first word is the ICAO code.
            e.m_mode = QString("%1").arg(gsAddress, 6, 16, QChar('0')).toUpper();
            const Vdl2GsInfo& gs = m_vdl2Gs.value(gsAddress);
            e.m_modeDecode = gs.m_name.section(' ', 0, 0);
            e.m_gsHasPosition = gs.m_hasPosition;
            e.m_gsLatitude = gs.m_latitude;
            e.m_gsLongitude = gs.m_longitude;
        }
    }

    // Information frames carry the ATN subnetwork: X.25, CLNP, COTP and the ICAO
    // applications (CM logons, CPDLC, ADS-C v2). XID frames carry link management
    // parameters, including the GSIF ground station broadcasts.
    bool isInfoFrame = (type == "I") || (type == "UI");
    if ((isInfoFrame || (type == "XID")) && (frame.m_infoLength > 0))
    {
        QByteArray info((const char *) frame.m_bytes.data() + frame.m_infoOffset, frame.m_infoLength);
        Vdl2AtnDecoder::Result atn;

        if (isInfoFrame)
        {
            atn = m_atnDecoder.decode(info, frame.m_srcAddress, frame.m_dstAddress, frame.m_fromAircraft);
        }
        else
        {
            bool pf = (frame.m_bytes[8] >> 4) & 0x1; // Control field P/F bit
            atn = m_atnDecoder.decodeXid(info, frame.m_srcStatus, pf);
        }

        text.append(QString(" %1").arg(atn.m_summary));

        if (!atn.m_decoded.isEmpty())
        {
            e.m_textDecode = QString(atn.m_decoded).replace('\n', "; ");
            e.m_fullDecode = atn.m_decoded;
            e.m_viewDecodeHtml = acarsDecodeToHtml(atn.m_decoded);
        }
        // Left blank for CPDLC messages with no message data (e.g. an ACSE release
        // wrapping an empty PDU)
        if (atn.m_isCpdlc && !atn.m_atc.isEmpty()) {
            e.m_atc = atn.m_atc;
        }
        if (atn.m_hasPosition)
        {
            // From an XID aircraft (or ground station) location parameter. The resolution
            // is coarse: 0.1 degrees and 1000 ft.
            e.m_hasPosition = true;
            e.m_latitude = atn.m_latitude;
            e.m_longitude = atn.m_longitude;
            if (atn.m_hasAltitude)
            {
                e.m_hasAltitude = true;
                e.m_altitudeFt = atn.m_altitudeFt;
            }
            // A downlinked location is the aircraft's own position
            if (frame.m_fromAircraft)
            {
                report.m_positionValid = true;
                report.m_latitude = atn.m_latitude;
                report.m_longitude = atn.m_longitude;
                if (atn.m_hasAltitude)
                {
                    report.m_altitudeValid = true;
                    report.m_altitudeFt = atn.m_altitudeFt;
                }
            }
        }
        report.m_documentText = atn.m_decoded;
        if (atn.m_isCpdlc)
        {
            report.m_documentKind = AircraftReport::Cpdlc;
            report.m_atc = atn.m_atc;
        }
        else if (type == "XID")
        {
            report.m_documentKind = AircraftReport::Logon;
        }
    }
    else if (frame.m_infoLength > 0)
    {
        text.append(QString(" (%1 bytes)").arg(frame.m_infoLength));
    }
    e.m_text = text.trimmed();

    QByteArray bytes((const char *) frame.m_bytes.data(), (int) frame.m_bytes.size());
    e.m_hex = bytes.toHex();

    // Report identified aircraft to the Aircraft feature. ACARS-over-AVLC frames
    // are reported from processAcarsMessage instead, with their decoded content.
    if (report.m_icao)
    {
        report.m_label = type;
        sendAircraftReport(report);
    }

    emit rowReady(e);
}

// Decode an HFDL LPDU or SPDU that is not an ACARS message: logons, squitters
// and the embedded HFNPDU performance/frequency reports
// One Aero signal unit that is not an ACARS message: log-on and log-off exchanges,
// channel control and assignment, acknowledgements and the system table broadcasts.
// ACARS blocks never come here - they are reassembled inside the receiver and pushed as
// MainCore::MsgPacket, so they take the same path as every other protocol's.
void AcarsDemodWorker::processAeroFrame(const AcarsAeroReceiver::Frame& frame, QDateTime received)
{
    AcarsRowEvent e;
    e.m_frameType = 3;
    e.m_received = received;
    e.m_uplink = frame.m_uplink;

    AircraftReport report;
    report.m_received = received;
    report.m_uplink = frame.m_uplink;

    // The AES ID is the aircraft's ICAO 24 bit address, so the same registration lookup
    // the HFDL and VDL-2 paths use applies directly. (Confirmed off air: AES 78016E
    // resolves to B-HNF, which the ACARS block in the same recording also names.)
    QString address;
    if (frame.m_aesId)
    {
        QString hex = QString("%1").arg(frame.m_aesId, 6, 16, QChar('0')).toUpper();
        address = hex;
        report.m_icao = frame.m_aesId;
        if (m_aircraftInfoByIcao && m_aircraftInfoByIcao->contains((int) frame.m_aesId))
        {
            QString registration = m_aircraftInfoByIcao->value((int) frame.m_aesId)->m_registration;
            if (!registration.isEmpty())
            {
                address = registration;
                report.m_registration = registration;
            }
        }
        // Only a downlink proves the aircraft transmitted; a ground station addressing
        // an aircraft still counts as a sighting but not as an active transmitter for
        // the GUI's aircraft-seen chart
        if (!frame.m_uplink) {
            e.m_chartAircraftId = hex;
        }
    }
    e.m_address = address;

    static const char *channelNames[] = { "P", "R", "T" };
    const char *channel = ((frame.m_channel >= 0) && (frame.m_channel <= AcarsAeroReceiver::ChannelT))
                        ? channelNames[frame.m_channel] : "?";
    e.m_protocol = QString("Aero %1").arg(channel);
    e.m_bitRate = frame.m_bitRate;
    // The signal unit type octet is the label; what it means is the decode
    e.m_label = frame.m_bytes.empty() ? QString()
              : QString("%1").arg(frame.m_bytes[0], 2, 16, QChar('0')).toUpper();
    e.m_labelDecode = QString::fromStdString(frame.m_type);

    // A signal unit with neither address is a broadcast - the system and EIRP tables
    // describe the satellite and its beams rather than talking to any one aircraft. It
    // used to render as "GES 00 -> AES", which is two placeholders and a zero pretending
    // to be a conversation
    // As hexadecimal, which is how the Message column has always shown it. The decode
    // names the satellite and region rather than the station - see gesName().
    if (frame.m_gesId)
    {
        e.m_mode = QString("%1").arg(frame.m_gesId, 2, 16, QChar('0')).toUpper();
        const char *ges = AcarsAeroReceiver::gesName(frame.m_gesId);
        e.m_modeDecode = ges ? QString(ges) : QString();
    }

    if (!frame.m_aesId && !frame.m_gesId)
    {
        e.m_text = "Broadcast";
    }
    else
    {
        QString gesText = QString("GES %1").arg(frame.m_gesId, 2, 16, QChar('0')).toUpper();
        if (frame.m_uplink) {
            e.m_text = QString("%1 %2 %3").arg(gesText).arg(QChar(0x2192)).arg(address.isEmpty() ? QString("AES") : address);
        } else {
            e.m_text = QString("%1 %2 %3").arg(address.isEmpty() ? QString("AES") : address).arg(QChar(0x2192)).arg(gesText);
        }
    }

    QStringList decode;
    const std::vector<uint8_t>& b = frame.m_bytes;
    uint8_t type = b.empty() ? 0 : b[0];

    // Satellite housekeeping, which the GUI hides by default. Classified here rather
    // than from the "Broadcast" text above, because a broadcast is a consequence of
    // carrying no address and not the same question as carrying no information
    e.m_noInfo = AcarsAeroReceiver::suIsNoInfo(type);

    switch (type)
    {
    case AcarsAeroReceiver::SuCChannelDistress:
    case AcarsAeroReceiver::SuCChannelFlightSafety:
    case AcarsAeroReceiver::SuCChannelOtherSafety:
    case AcarsAeroReceiver::SuCChannelNonSafety:
        // A voice channel assignment names the pair of L band frequencies the call will
        // use, as 15 bit channel numbers with bit 15 flagging a spot beam
        if (b.size() >= 10)
        {
            int rxChannel = ((b[6] & 0x7F) << 8) | b[7];
            int txChannel = ((b[8] & 0x7F) << 8) | b[9];
            decode.append(QString("Receive %1 MHz (%2)")
                .arg(AcarsAeroReceiver::cChannelReceiveMHz(rxChannel), 0, 'f', 4)
                .arg((b[6] & 0x80) ? "spot beam" : "global beam"));
            decode.append(QString("Transmit %1 MHz (%2)")
                .arg(AcarsAeroReceiver::cChannelTransmitMHz(txChannel), 0, 'f', 4)
                .arg((b[8] & 0x80) ? "spot beam" : "global beam"));
        }
        break;

    case AcarsAeroReceiver::SuLogOnRequest:
    case AcarsAeroReceiver::SuLogOnConfirm:
    case AcarsAeroReceiver::SuLogOffRequest:
    case AcarsAeroReceiver::SuLogOnReject:
    case AcarsAeroReceiver::SuLogOnInterrogation:
    case AcarsAeroReceiver::SuLogOnOffAcknowledge:
    case AcarsAeroReceiver::SuLogOnPrompt:
    case AcarsAeroReceiver::SuDataChannelReassignment:
        report.m_documentKind = AircraftReport::Logon;
        break;

    default:
        break;
    }

    if (frame.m_gesId) {
        report.m_station = QString("GES %1").arg(frame.m_gesId, 2, 16, QChar('0')).toUpper();
    }

    e.m_textDecode = decode.join("; ");
    e.m_fullDecode = decode.join("\n");
    // m_fullDecode is functional - it becomes AircraftReport::m_documentText - but the
    // HTML rendering of it is only ever read by the decode view
    if (hasGui()) {
        e.m_viewDecodeHtml = acarsDecodeToHtml(e.m_fullDecode);
    }

    QByteArray bytes((const char *) b.data(), (int) b.size());
    e.m_hex = bytes.toHex();

    // Report identified aircraft to the Aircraft feature. Signal units that name no
    // aircraft - system tables, channel control - are shown in the table but are not
    // sightings of anything.
    if (report.m_icao || !report.m_registration.isEmpty())
    {
        report.m_label = QString::fromStdString(frame.m_type);
        report.m_documentText = e.m_fullDecode;
        sendAircraftReport(report);
    }

    emit rowReady(e);
}

void AcarsDemodWorker::processHfdlFrame(const AcarsHfdlReceiver::Frame& frame, QDateTime received)
{
    AcarsRowEvent e;
    e.m_frameType = 2;
    e.m_received = received;
    e.m_uplink = frame.m_uplink;

    // Logon LPDUs carry the aircraft's 24 bit ICAO address, which gives the
    // registration via the OpenSky database and feeds the aircraft-seen chart
    uint32_t icao = 0;
    static const QList<uint8_t> logonTypes = { 0x2F, 0x3F, 0x4F, 0x5F, 0x8F, 0x9F, 0xBF };
    if ((frame.m_bytes.size() >= 4) && logonTypes.contains(frame.m_bytes[0])) {
        icao = AcarsHfdlReceiver::parseIcao(frame.m_bytes.data() + 1);
    }
    if (icao && !frame.m_uplink) {
        e.m_chartAircraftId = QString("%1").arg(icao, 6, 16, QChar('0')).toUpper();
    }

    // The 8-bit HFDL aircraft ID is assigned by the ground station at logon; 255
    // means no ID assigned yet - the aircraft is logging on
    auto acText = [](uint32_t id)
    {
        return id == 255 ? QString("Unidentified") : QString("AC %1").arg(id);
    };

    // One sighting for the Aircraft feature, filled in as the frame decodes. Only
    // frames that identify the airframe (an ICAO address from a logon) or carry a
    // flight identity are reported - the 8-bit "AC n" link IDs are not durable
    // identities an aircraft can be collated under.
    AircraftReport report;
    report.m_received = received;
    report.m_uplink = frame.m_uplink;

    // Registration from the ICAO address when a logon carries one, otherwise the
    // 8-bit HFDL aircraft/ground station IDs from the MPDU header
    QString address;
    if (icao)
    {
        QString hex = QString("%1").arg(icao, 6, 16, QChar('0')).toUpper();
        address = hex;
        report.m_icao = icao;
        report.m_documentKind = AircraftReport::Logon;
        if (m_aircraftInfoByIcao && m_aircraftInfoByIcao->contains((int) icao))
        {
            QString registration = m_aircraftInfoByIcao->value((int) icao)->m_registration;
            if (!registration.isEmpty())
            {
                address = registration;
                report.m_registration = registration;
            }
        }
    }
    else if (frame.m_type == "Squitter")
    {
        address = QString("GS %1").arg(frame.m_srcId);
    }
    else if (frame.m_uplink)
    {
        address = acText(frame.m_dstId);
    }
    else
    {
        address = acText(frame.m_srcId);
    }
    e.m_address = address;

    // Logon requests carry the flight identity as 6 characters at offset 6, space
    // padded for shorter callsigns. The following byte belongs to a varying field
    // that can happen to be printable (it showed up as bogus trailing characters,
    // often lowercase), so only callsign characters are accepted and the first
    // non-callsign byte ends the field.
    if ((frame.m_bytes.size() >= 12) && icao)
    {
        QString flight;
        for (int i = 6; i < 12; i++)
        {
            char ch = (char) frame.m_bytes[i];
            if (((ch >= 'A') && (ch <= 'Z')) || ((ch >= '0') && (ch <= '9')) || (ch == ' ')) {
                flight.append(ch);
            } else {
                break;
            }
        }
        e.m_flight = flight.trimmed();
        report.m_flight = flight.trimmed();
    }

    e.m_protocol = "HFDL";
    e.m_bitRate = frame.m_bitRate;
    if (frame.m_isSquitter)
    {
        // A squitter is not an LPDU and has no type octet to put in the label
        e.m_label = "Squitter";
        e.m_labelDecode = "Ground station frequency broadcast";
    }
    else
    {
        e.m_label = QString("%1").arg(frame.m_typeId, 2, 16, QChar('0')).toUpper();
        e.m_labelDecode = QString::fromStdString(frame.m_type);
    }

    // The ground station side of the link, decoded from the system table ID
    uint32_t gsId = (frame.m_uplink || (frame.m_type == "Squitter")) ? frame.m_srcId : frame.m_dstId;
    const char *gsStation = AcarsHfdlReceiver::gsName(gsId);
    if (gsStation) {
        report.m_station = QString(gsStation).section(',', 0, 0);
    } else if (gsId) {
        report.m_station = QString("GS %1").arg(gsId);
    }

    // The station is what the Mode columns are for. gsName gives "Shannon, Ireland";
    // the town alone is what fits, and the country is in the Message column anyway.
    if (gsId)
    {
        e.m_mode = QString::number(gsId);
        e.m_modeDecode = gsStation ? QString(gsStation).section(',', 0, 0) : QString();

        double gsLatitude, gsLongitude;
        if (AcarsHfdlReceiver::gsPosition(gsId, gsLatitude, gsLongitude))
        {
            e.m_gsHasPosition = true;
            e.m_gsLatitude = (float) gsLatitude;
            e.m_gsLongitude = (float) gsLongitude;
        }
    }

    QString text;
    QString decode;
    if (frame.m_type == "Squitter")
    {
        text = QString("Squitter from GS %1").arg(frame.m_srcId);
        decode = QString("From %1").arg(gsStation ? gsStation : QString("GS %1").arg(gsId));

        // The squitter's position is the transmitting ground station itself, from the
        // system table: filling the position columns arms the Map checkbox, and the
        // "GS" address makes it a fixed antenna icon rather than an aircraft
        double gsLat, gsLon;
        if (AcarsHfdlReceiver::gsPosition(frame.m_srcId, gsLat, gsLon))
        {
            e.m_hasPosition = true;
            e.m_latitude = gsLat;
            e.m_longitude = gsLon;
        }

        AcarsHfdlReceiver::SquitterInfo sq;
        if (AcarsHfdlReceiver::parseSquitter(frame.m_bytes, sq))
        {
            decode.append(QString("; frame %1.%2; systable v%3")
                .arg(sq.m_frameIndex).arg(sq.m_frameOffset).arg(sq.m_systableVersion));
            const char *note = AcarsHfdlReceiver::squitterChangeNote(sq.m_changeNote);
            if (note) {
                decode.append(QString("; %1").arg(note));
            }
            if (sq.m_rls) {
                decode.append("; RLS");
            }
            // The transmitting station's status plus up to two neighbours; the
            // frequency sets are bitmasks into each station's system table
            // frequency list, decoded here to kHz
            for (const auto& gs : sq.m_gs)
            {
                if (gs.m_id == 0) {
                    continue;
                }
                QStringList freqs;
                int freqCount = 0;
                const uint16_t *freqList = AcarsHfdlReceiver::gsFrequencies(gs.m_id, &freqCount);
                for (int bit = 0; bit < 20; bit++)
                {
                    if ((gs.m_freqsInUse >> bit) & 1)
                    {
                        if (freqList && (bit < freqCount)) {
                            freqs.append(QString::number(freqList[bit]));
                        } else {
                            freqs.append(QString("bit%1").arg(bit));
                        }
                    }
                }
                decode.append(QString("; GS %1%2 on %3")
                    .arg(gs.m_id)
                    .arg(gs.m_utcSync ? "" : " (no UTC)")
                    .arg(freqs.isEmpty() ? "no frequencies" : freqs.join(", ") + " kHz"));
            }
            // Each station's announced frequencies for the GUI's ground station
            // table dialog - one squitter reports up to three stations
            for (const auto& gs : sq.m_gs)
            {
                if (gs.m_id == 0) {
                    continue;
                }
                int freqCount = 0;
                const uint16_t *freqList = AcarsHfdlReceiver::gsFrequencies(gs.m_id, &freqCount);
                QList<int> heard;
                for (int bit = 0; (bit < 20) && freqList; bit++)
                {
                    if (((gs.m_freqsInUse >> bit) & 1) && (bit < freqCount)) {
                        heard.append(freqList[bit]);
                    }
                }
                e.m_gsHeardIds.append((int) gs.m_id);
                e.m_gsHeardFreqs.append(heard);
            }
        }
    }
    else if (frame.m_uplink)
    {
        text = QString("GS %1 %2 %3").arg(frame.m_srcId).arg(QChar(0x2192)).arg(acText(frame.m_dstId));
        decode = QString("From %1").arg(gsStation ? gsStation : QString("GS %1").arg(gsId));
    }
    else
    {
        text = QString("%1 %2 GS %3").arg(acText(frame.m_srcId)).arg(QChar(0x2192)).arg(frame.m_dstId);
        decode = QString("To %1").arg(gsStation ? gsStation : QString("GS %1").arg(gsId));
    }

    // Embedded HFNPDUs: performance data (FF D1) in unnumbered data LPDUs and
    // frequency data (FF D5) inside logon requests and resumes. Both carry the
    // flight identity and the aircraft's position, which fills the position
    // columns and arms the Map checkbox.
    if (frame.m_type != "Squitter")
    {
        // A ground station's frequency-list bitmask rendered as kHz, as for squitters
        auto freqMaskText = [](uint32_t gsId, uint32_t mask)
        {
            QStringList freqs;
            int freqCount = 0;
            const uint16_t *freqList = AcarsHfdlReceiver::gsFrequencies(gsId, &freqCount);
            for (int bit = 0; bit < 20; bit++)
            {
                if ((mask >> bit) & 1)
                {
                    if (freqList && (bit < freqCount)) {
                        freqs.append(QString::number(freqList[bit]));
                    } else {
                        freqs.append(QString("bit%1").arg(bit));
                    }
                }
            }
            return freqs.isEmpty() ? QString("none") : freqs.join(", ") + " kHz";
        };
        auto positionText = [](double latitude, double longitude, int utcSeconds)
        {
            return QString("%1%2 %3%4 at %5z")
                .arg(std::abs(latitude), 0, 'f', 4).arg(latitude < 0.0 ? "S" : "N")
                .arg(std::abs(longitude), 0, 'f', 4).arg(longitude < 0.0 ? "W" : "E")
                .arg(QTime(0, 0).addSecs(utcSeconds).toString("hh:mm:ss"));
        };
        auto setFlightAndPosition = [&](const std::string& flightId, double latitude, double longitude, int utcSeconds)
        {
            QString flight = QString::fromStdString(flightId).trimmed();
            if (!flight.isEmpty() && e.m_flight.isEmpty()) {
                e.m_flight = flight;
            }
            if (!flight.isEmpty() && report.m_flight.isEmpty()) {
                report.m_flight = flight;
            }
            report.m_documentKind = AircraftReport::PerformanceReport;
            // Aircraft without a position feed report the all-ones sentinel, which
            // decodes as 180/180 - any latitude beyond 90 marks it unavailable
            if (std::abs(latitude) > 90.0) {
                return;
            }
            e.m_hasPosition = true;
            e.m_latitude = latitude;
            e.m_longitude = longitude;

            // Aircraft dump their STORED performance log too - one record per past
            // ground station association, hours old (seen live: a burst carrying
            // reports from three flight legs back). Only a report timestamped close
            // to now is the aircraft's current position, so only that goes to the
            // Map; stale reports still fill the row's position columns. "Now" is the
            // wall clock OR the newest report time seen, so replayed recordings
            // (whose reports are all hours old against the wall clock but mutually
            // consistent) still reach the Map.
            auto circAge = [](int a, int b)
            {
                int d = a - b;
                while (d > 43200) { d -= 86400; }
                while (d < -43200) { d += 86400; }
                return d;
            };
            int nowSeconds = received.toUTC().time().msecsSinceStartOfDay() / 1000;
            bool fresh = std::abs(circAge(nowSeconds, utcSeconds)) <= 600;
            if (!fresh && (m_hfdlNewestReportSecs >= 0)) {
                // Not older than 10 minutes behind the newest report seen
                fresh = circAge(utcSeconds, m_hfdlNewestReportSecs) >= -600;
            }
            if ((m_hfdlNewestReportSecs < 0)
                || (circAge(utcSeconds, m_hfdlNewestReportSecs) > 0)) {
                m_hfdlNewestReportSecs = utcSeconds;
            }
            if (!fresh) {
                return;
            }

            // A fresh position goes into the report for the Aircraft feature,
            // stamped with the report's own UTC time of day mapped to the nearest
            // day around reception
            report.m_positionValid = true;
            report.m_latitude = (float) latitude;
            report.m_longitude = (float) longitude;
            QDateTime dayStart = received.toUTC();
            dayStart.setTime(QTime(0, 0));
            QDateTime posTime = dayStart.addSecs(utcSeconds);
            const qint64 halfDay = 12 * 3600;
            if (posTime.secsTo(received.toUTC()) > halfDay) {
                posTime = posTime.addDays(1);
            } else if (received.toUTC().secsTo(posTime) > halfDay) {
                posTime = posTime.addDays(-1);
            }
            report.m_positionDateTime = posTime;
        };

        AcarsHfdlReceiver::PerfDataInfo perf;
        AcarsHfdlReceiver::FreqDataInfo freqData;
        if (AcarsHfdlReceiver::parsePerfData(frame.m_bytes, perf))
        {
            decode.append(QString("; Performance data: %1 %2; v%3 leg %4; GS %5 freq %6")
                .arg(QString::fromStdString(perf.m_flightId).trimmed())
                .arg(positionText(perf.m_latitude, perf.m_longitude, perf.m_utcSeconds))
                .arg(perf.m_version).arg(perf.m_flightLeg)
                .arg(perf.m_gsId).arg(perf.m_freqId));
            decode.append(QString("; freq searches %1/%2; HF off %3/%4 s (prev/cur)")
                .arg(perf.m_prevFreqSearches).arg(perf.m_curFreqSearches)
                .arg(perf.m_prevHfDisabledSecs).arg(perf.m_curHfDisabledSecs));
            decode.append(QString("; MPDUs rx %1/%2/%3/%4 errs %5/%6/%7/%8, tx %9/%10/%11/%12, delivered %13/%14/%15/%16 (1800/1200/600/300 bps)")
                .arg(perf.m_mpdusRx[0]).arg(perf.m_mpdusRx[1]).arg(perf.m_mpdusRx[2]).arg(perf.m_mpdusRx[3])
                .arg(perf.m_mpdusRxErrs[0]).arg(perf.m_mpdusRxErrs[1]).arg(perf.m_mpdusRxErrs[2]).arg(perf.m_mpdusRxErrs[3])
                .arg(perf.m_mpdusTx[0]).arg(perf.m_mpdusTx[1]).arg(perf.m_mpdusTx[2]).arg(perf.m_mpdusTx[3])
                .arg(perf.m_mpdusDelivered[0]).arg(perf.m_mpdusDelivered[1]).arg(perf.m_mpdusDelivered[2]).arg(perf.m_mpdusDelivered[3]));
            decode.append(QString("; SPDUs rx %1 errs %2; freq change %3")
                .arg(perf.m_spdusRx).arg(perf.m_spdusRxErrs).arg(perf.m_freqChangeCode));
            setFlightAndPosition(perf.m_flightId, perf.m_latitude, perf.m_longitude, perf.m_utcSeconds);
        }
        else if (AcarsHfdlReceiver::parseFreqData(frame.m_bytes, freqData))
        {
            decode.append(QString("; Frequency data: %1 %2")
                .arg(QString::fromStdString(freqData.m_flightId).trimmed())
                .arg(positionText(freqData.m_latitude, freqData.m_longitude, freqData.m_utcSeconds)));
            for (const auto& pf : freqData.m_gs)
            {
                if (pf.m_gsId == 0) {
                    continue;
                }
                const char *name = AcarsHfdlReceiver::gsName(pf.m_gsId);
                decode.append(QString("; GS %1%2: propagating %3, tuned %4")
                    .arg(pf.m_gsId)
                    .arg(name ? QString(" (%1)").arg(QString(name).section(',', 0, 0)) : QString())
                    .arg(freqMaskText(pf.m_gsId, pf.m_propagating))
                    .arg(freqMaskText(pf.m_gsId, pf.m_tuned)));
            }
            setFlightAndPosition(freqData.m_flightId, freqData.m_latitude, freqData.m_longitude, freqData.m_utcSeconds);
        }
    }

    e.m_text = text;
    e.m_textDecode = decode;
    // The column keeps the single semicolon-separated line; the decode view and the
    // Map popup get the multi-line form
    e.m_fullDecode = QString(decode).replace("; ", "\n");
    e.m_viewDecodeHtml = acarsDecodeToHtml(e.m_fullDecode);

    QByteArray bytes((const char *) frame.m_bytes.data(), (int) frame.m_bytes.size());
    e.m_hex = bytes.toHex();

    // Report identified aircraft to the Aircraft feature
    if (report.m_icao || !report.m_registration.isEmpty() || !report.m_flight.isEmpty())
    {
        report.m_label = QString::fromStdString(frame.m_type);
        report.m_documentText = e.m_fullDecode;
        sendAircraftReport(report);
    }

    emit rowReady(e);
}
