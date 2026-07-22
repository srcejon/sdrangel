///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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

#include <cmath>
#include <limits>

#include <QDebug>

#include "SWGChannelSettings.h"
#include "SWGMeteorSettings.h"
#include "SWGWorkspaceInfo.h"

#include "device/deviceapi.h"
#include "dsp/dspcommands.h"

#include "meteor.h"

MESSAGE_CLASS_DEFINITION(Meteor::MsgConfigureMeteor, Message)
MESSAGE_CLASS_DEFINITION(Meteor::MsgCameraMeteorDetected, Message)

const char * const Meteor::m_channelIdURI = "sdrangel.channel.meteor";
const char * const Meteor::m_channelId = "Meteor";

namespace {
    constexpr int CameraEventExpirySeconds = 10 * 60;
}

Meteor::Meteor(DeviceAPI *deviceAPI) :
    ChannelAPI(m_channelIdURI, ChannelAPI::StreamSingleSink),
    m_deviceAPI(deviceAPI),
    m_spectrumVis(SDR_RX_SCALEF),
    m_headSpectrumVis(SDR_RX_SCALEF),
    m_basebandSampleRate(0),
    m_centerFrequency(0),
    m_eventSourceHandler(
        QStringList({QStringLiteral("sdrangel.feature.camera")}),
        QStringList({QStringLiteral("event")}),
        QStringLiteral("F"))
{
    setObjectName(m_channelId);

    m_basebandSink = new MeteorBaseband();
    m_basebandSink->setSpectrumSink(&m_spectrumVis);
    m_basebandSink->setChannel(this);
    m_basebandSink->moveToThread(&m_thread);

    applySettings(m_settings, QStringList(), true);

    m_deviceAPI->addChannelSink(this);
    m_deviceAPI->addChannelSinkAPI(this);

    QObject::connect(
        this,
        &ChannelAPI::indexInDeviceSetChanged,
        this,
        &Meteor::handleIndexInDeviceSetChanged
    );

    QObject::connect(
        &m_eventSourceHandler,
        &AvailableChannelOrFeatureHandler::messageEnqueued,
        this,
        &Meteor::eventMessageEnqueued,
        Qt::QueuedConnection
    );
    m_cameraEventExpiryTimer.setInterval(60 * 1000);
    connect(&m_cameraEventExpiryTimer, &QTimer::timeout, this, &Meteor::pruneCameraMeteorEvents);
    m_cameraEventExpiryTimer.start();
    m_eventSourceHandler.scanAvailableChannelsAndFeatures();
}

Meteor::~Meteor()
{
    qDebug("Meteor::~Meteor");

    m_deviceAPI->removeChannelSinkAPI(this);
    m_deviceAPI->removeChannelSink(this, true);

    if (m_basebandSink->isRunning()) {
        stop();
    }

    delete m_basebandSink;
}

void Meteor::setDeviceAPI(DeviceAPI *deviceAPI)
{
    if (deviceAPI != m_deviceAPI)
    {
        m_deviceAPI->removeChannelSinkAPI(this);
        m_deviceAPI->removeChannelSink(this, false);
        m_deviceAPI = deviceAPI;
        m_deviceAPI->addChannelSink(this);
        m_deviceAPI->addChannelSinkAPI(this);
    }
}

void Meteor::setMessageQueueToGUI(MessageQueue *queue)
{
    ChannelAPI::setMessageQueueToGUI(queue);
    m_basebandSink->setMessageQueueToGUI(queue);
    m_basebandSink->setSpectrumSinks(&m_spectrumVis, queue ? &m_headSpectrumVis : nullptr);
}

uint32_t Meteor::getNumberOfDeviceStreams() const
{
    return m_deviceAPI->getNbSourceStreams();
}

void Meteor::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end, bool firstOfBurst)
{
    (void) firstOfBurst;
    m_basebandSink->feed(begin, end);
}

void Meteor::start()
{
    qDebug("Meteor::start");

    m_basebandSink->reset();
    m_basebandSink->startWork();
    m_thread.start();

    DSPSignalNotification *dspMsg = new DSPSignalNotification(m_basebandSampleRate, m_centerFrequency);
    m_basebandSink->getInputMessageQueue()->push(dspMsg);

    MeteorBaseband::MsgConfigureMeteorBaseband *msg =
        MeteorBaseband::MsgConfigureMeteorBaseband::create(m_settings, QStringList(), true);
    m_basebandSink->getInputMessageQueue()->push(msg);

    if (getMessageQueueToGUI()) {
        getMessageQueueToGUI()->push(new DSPSignalNotification(m_basebandSampleRate, m_centerFrequency));
    }
}

void Meteor::stop()
{
    qDebug("Meteor::stop");
    m_basebandSink->stopWork();
    m_thread.quit();
    m_thread.wait();
}

bool Meteor::handleMessage(const Message& cmd)
{
    if (MsgConfigureMeteor::match(cmd))
    {
        MsgConfigureMeteor& cfg = (MsgConfigureMeteor&) cmd;
        qDebug() << "Meteor::handleMessage: MsgConfigureMeteor";
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());

        return true;
    }
    else if (DSPSignalNotification::match(cmd))
    {
        DSPSignalNotification& notif = (DSPSignalNotification&) cmd;
        m_basebandSampleRate = notif.getSampleRate();
        m_centerFrequency = notif.getCenterFrequency();
        qDebug() << "Meteor::handleMessage: DSPSignalNotification";

        m_basebandSink->getInputMessageQueue()->push(new DSPSignalNotification(notif));

        if (getMessageQueueToGUI()) {
            getMessageQueueToGUI()->push(new DSPSignalNotification(notif));
        }

        return true;
    }
    else if (MainCore::MsgEvent::match(cmd))
    {
        handleEvent((const MainCore::MsgEvent&) cmd);
        return true;
    }
    else
    {
        return false;
    }
}

void Meteor::eventMessageEnqueued(MessageQueue *messageQueue)
{
    handleEventMessageQueue(messageQueue);
}

void Meteor::handleEventMessageQueue(MessageQueue *messageQueue)
{
    Message *message;

    while ((message = messageQueue->pop()) != nullptr)
    {
        if (MainCore::MsgEvent::match(*message)) {
            handleEvent((const MainCore::MsgEvent&) *message);
        }

        delete message;
    }
}

void Meteor::handleEvent(const MainCore::MsgEvent& eventMessage)
{
    if (!isMeteorObjectEvent(eventMessage)) {
        return;
    }

    QDateTime eventTime = eventMessage.getDateTime();

    if (!eventTime.isValid()) {
        eventTime = QDateTime::currentDateTimeUtc();
    }

    const QObject *source = eventMessage.getPipeSource();

    if (eventMessage.getEvent() == MainCore::MsgEvent::CameraObjectDetectedEvent)
    {
        pruneCameraMeteorEvents();

        if (!m_cameraMeteorEvents.contains(source))
        {
            const QMap<QString, QString> fields = parseEventDataFields(eventMessage.getData());
            CameraMeteorEvent cameraEvent;
            cameraEvent.m_startTimeUtc = eventTime.toUTC();
            cameraEvent.m_receivedTimeUtc = QDateTime::currentDateTimeUtc();
            cameraEvent.m_hasMagnitude = parseEventDoubleField(fields, QStringLiteral("magnitude"), cameraEvent.m_magnitude);
            cameraEvent.m_hasFlux = parseEventDoubleField(fields, QStringLiteral("flux"), cameraEvent.m_flux);
            m_cameraMeteorEvents.insert(source, cameraEvent);

            if (source && !m_cameraEventSources.contains(source))
            {
                m_cameraEventSources.insert(source);
                connect(
                    const_cast<QObject *>(source),
                    &QObject::destroyed,
                    this,
                    [this, source]() {
                        m_cameraMeteorEvents.remove(source);
                        m_cameraEventSources.remove(source);
                    });
            }
        }

        return;
    }

    if (eventMessage.getEvent() != MainCore::MsgEvent::CameraObjectLostEvent) {
        return;
    }

    const auto startIt = m_cameraMeteorEvents.find(source);

    if (startIt == m_cameraMeteorEvents.end()) {
        return;
    }

    const CameraMeteorEvent cameraEvent = startIt.value();
    m_cameraMeteorEvents.erase(startIt);

    const qint64 durationMSecs = cameraEvent.m_startTimeUtc.msecsTo(eventTime);

    if (durationMSecs < 0) {
        return;
    }

    if (getMessageQueueToGUI())
    {
        getMessageQueueToGUI()->push(MsgCameraMeteorDetected::create(
            cameraEvent.m_startTimeUtc,
            (double) durationMSecs / 1000.0,
            cameraEvent.m_hasMagnitude,
            cameraEvent.m_magnitude,
            cameraEvent.m_hasFlux,
            cameraEvent.m_flux));
    }
}

void Meteor::pruneCameraMeteorEvents()
{
    const QDateTime cutoffUtc = QDateTime::currentDateTimeUtc().addSecs(-CameraEventExpirySeconds);

    for (auto it = m_cameraMeteorEvents.begin(); it != m_cameraMeteorEvents.end();)
    {
        if (!it->m_receivedTimeUtc.isValid() || (it->m_receivedTimeUtc < cutoffUtc)) {
            it = m_cameraMeteorEvents.erase(it);
        } else {
            ++it;
        }
    }
}

QMap<QString, QString> Meteor::parseEventDataFields(const QString& data)
{
    QMap<QString, QString> fields;
    const QStringList entries = data.split(',', Qt::SkipEmptyParts);

    for (const QString& entry : entries)
    {
        const int separator = entry.indexOf('=');

        if (separator <= 0) {
            continue;
        }

        const QString name = entry.left(separator).trimmed().toLower();
        QString value = entry.mid(separator + 1).trimmed();

        if (((value.startsWith(QLatin1Char('"')) && value.endsWith(QLatin1Char('"')))
            || (value.startsWith(QLatin1Char('\'')) && value.endsWith(QLatin1Char('\''))))
            && (value.size() >= 2))
        {
            value = value.mid(1, value.size() - 2);
        }

        if (!name.isEmpty()) {
            fields.insert(name, value);
        }
    }

    return fields;
}

bool Meteor::parseEventDoubleField(const QMap<QString, QString>& fields, const QString& name, double& value)
{
    const QString text = fields.value(name).trimmed();

    if (text.isEmpty()) {
        return false;
    }

    bool ok = false;
    value = text.toDouble(&ok);
    return ok;
}

bool Meteor::isMeteorObjectEvent(const MainCore::MsgEvent& eventMessage)
{
    if ((eventMessage.getEvent() != MainCore::MsgEvent::CameraObjectDetectedEvent)
        && (eventMessage.getEvent() != MainCore::MsgEvent::CameraObjectLostEvent))
    {
        return false;
    }

    const QMap<QString, QString> fields = parseEventDataFields(eventMessage.getData());
    QString objectClass = fields.value(QStringLiteral("class"));

    if (objectClass.isEmpty()) {
        objectClass = fields.value(QStringLiteral("name"));
    }

    if (objectClass.isEmpty()) {
        objectClass = fields.value(QStringLiteral("label"));
    }

    return objectClass.compare(QStringLiteral("meteor"), Qt::CaseInsensitive) == 0;
}

void Meteor::setCenterFrequency(qint64 frequency)
{
    MeteorSettings settings = m_settings;
    settings.m_inputFrequencyOffset = frequency;
    settings.m_frequency = m_centerFrequency + settings.m_inputFrequencyOffset;
    applySettings(settings, {"inputFrequencyOffset", "frequency"}, false);

    if (m_guiMessageQueue)
    {
        MsgConfigureMeteor *msgToGUI = MsgConfigureMeteor::create(settings, {"inputFrequencyOffset", "frequency"}, false);
        m_guiMessageQueue->push(msgToGUI);
    }
}

void Meteor::applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force)
{
    qDebug() << "Meteor::applySettings:" << settings.getDebugString(settingsKeys, force)
             << " force:" << force;

    if (settingsKeys.contains("streamIndex"))
    {
        if (m_deviceAPI->getSampleMIMO())
        {
            m_deviceAPI->removeChannelSinkAPI(this);
            m_deviceAPI->removeChannelSink(this, m_settings.m_streamIndex);
            m_deviceAPI->addChannelSink(this, settings.m_streamIndex);
            m_deviceAPI->addChannelSinkAPI(this);
            m_settings.m_streamIndex = settings.m_streamIndex;
            emit streamIndexChanged(settings.m_streamIndex);
        }
    }

    MeteorBaseband::MsgConfigureMeteorBaseband *msg =
        MeteorBaseband::MsgConfigureMeteorBaseband::create(settings, settingsKeys, force);
    m_basebandSink->getInputMessageQueue()->push(msg);

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

QByteArray Meteor::serialize() const
{
    return m_settings.serialize();
}

bool Meteor::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        MsgConfigureMeteor *msg = MsgConfigureMeteor::create(m_settings, QStringList(), true);
        m_inputMessageQueue.push(msg);
        return true;
    }
    else
    {
        m_settings.resetToDefaults();
        MsgConfigureMeteor *msg = MsgConfigureMeteor::create(m_settings, QStringList(), true);
        m_inputMessageQueue.push(msg);
        return false;
    }
}

int Meteor::webapiSettingsGet(
        SWGSDRangel::SWGChannelSettings& response,
        QString& errorMessage)
{
    (void) errorMessage;
    response.setMeteorSettings(new SWGSDRangel::SWGMeteorSettings());
    response.getMeteorSettings()->init();
    webapiFormatChannelSettings(response, m_settings);
    return 200;
}

int Meteor::webapiSettingsPutPatch(
        bool force,
        const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response,
        QString& errorMessage)
{
    MeteorSettings settings = m_settings;

    if (!webapiUpdateChannelSettings(settings, channelSettingsKeys, response, errorMessage)) {
        return 400;
    }

    if (channelSettingsKeys.contains("streamIndex")
        && ((uint32_t) settings.m_streamIndex >= getNumberOfDeviceStreams()))
    {
        errorMessage = QString("streamIndex must be less than the device stream count (%1)")
            .arg(getNumberOfDeviceStreams());
        return 400;
    }

    MsgConfigureMeteor *msg = MsgConfigureMeteor::create(settings, channelSettingsKeys, force);
    m_inputMessageQueue.push(msg);

    if (m_guiMessageQueue)
    {
        MsgConfigureMeteor *msgToGUI = MsgConfigureMeteor::create(settings, channelSettingsKeys, force);
        m_guiMessageQueue->push(msgToGUI);
    }

    webapiFormatChannelSettings(response, settings);
    return 200;
}

int Meteor::webapiWorkspaceGet(
        SWGSDRangel::SWGWorkspaceInfo& response,
        QString& errorMessage)
{
    (void) errorMessage;
    response.setIndex(m_settings.m_workspaceIndex);
    return 200;
}

void Meteor::webapiFormatChannelSettings(
        SWGSDRangel::SWGChannelSettings& response,
        const MeteorSettings& settings)
{
    SWGSDRangel::SWGMeteorSettings *swg = response.getMeteorSettings();

    swg->setInputFrequencyOffset(settings.m_inputFrequencyOffset);
    swg->setFrequencyMode((int) settings.m_frequencyMode);
    swg->setFrequency(settings.m_frequency);
    swg->setChannelSampleRate(settings.m_channelSampleRate);
    swg->setPowerLpfCutoff(settings.m_powerLPFCutoff);
    swg->setDetectionThresholdDb(settings.m_detectionThresholdDB);
    swg->setMinDurationMs(settings.m_minDurationMS);
    swg->setMaxDurationMs(settings.m_maxDurationMS);
    swg->setMaxFrequencyDrift(settings.m_maxFrequencyDrift);
    swg->setDetectionsTableColumnHidden(settings.m_detectionsTableColumnHidden);
    swg->setDetectionBoxPaddingPixels(settings.m_detectionBoxPaddingPixels);
    swg->setDetectionLabelMode((int) settings.m_detectionLabelMode);
    swg->setTransmitterLatitude(settings.m_transmitterLatitude);
    swg->setTransmitterLongitude(settings.m_transmitterLongitude);
    swg->setAntennaAzimuth(settings.m_antennaAzimuth);
    swg->setAntennaElevation(settings.m_antennaElevation);
    swg->setAntennaBeamwidth(settings.m_antennaBeamwidth);

    if (swg->getRotator()) {
        *swg->getRotator() = settings.m_rotator;
    } else {
        swg->setRotator(new QString(settings.m_rotator));
    }

    swg->setRgbColor(settings.m_rgbColor);

    if (swg->getTitle()) {
        *swg->getTitle() = settings.m_title;
    } else {
        swg->setTitle(new QString(settings.m_title));
    }

    swg->setStreamIndex(settings.m_streamIndex);
    swg->setWorkspaceIndex(settings.m_workspaceIndex);
    swg->setHidden(settings.m_hidden ? 1 : 0);

    if (settings.m_channelMarker)
    {
        if (swg->getChannelMarker()) {
            settings.m_channelMarker->formatTo(swg->getChannelMarker());
        } else {
            SWGSDRangel::SWGChannelMarker *swgChannelMarker = new SWGSDRangel::SWGChannelMarker();
            settings.m_channelMarker->formatTo(swgChannelMarker);
            swg->setChannelMarker(swgChannelMarker);
        }
    }

    if (settings.m_spectrumGUI)
    {
        if (swg->getSpectrumGui()) {
            settings.m_spectrumGUI->formatTo(swg->getSpectrumGui());
        } else {
            SWGSDRangel::SWGGLSpectrum *swgSpectrumGUI = new SWGSDRangel::SWGGLSpectrum();
            settings.m_spectrumGUI->formatTo(swgSpectrumGUI);
            swg->setSpectrumGui(swgSpectrumGUI);
        }
    }

    if (settings.m_rollupState)
    {
        if (swg->getRollupState()) {
            settings.m_rollupState->formatTo(swg->getRollupState());
        } else {
            SWGSDRangel::SWGRollupState *swgRollupState = new SWGSDRangel::SWGRollupState();
            settings.m_rollupState->formatTo(swgRollupState);
            swg->setRollupState(swgRollupState);
        }
    }
}

bool Meteor::webapiUpdateChannelSettings(
        MeteorSettings& settings,
        const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response,
        QString& errorMessage)
{
    SWGSDRangel::SWGMeteorSettings *swg = response.getMeteorSettings();

    if (!swg)
    {
        errorMessage = "Missing MeteorSettings object";
        return false;
    }

    MeteorSettings updatedSettings = settings;

    if (channelSettingsKeys.contains("inputFrequencyOffset")) {
        const qint64 offset = swg->getInputFrequencyOffset();

        if ((offset < std::numeric_limits<qint32>::min())
            || (offset > std::numeric_limits<qint32>::max()))
        {
            errorMessage = "inputFrequencyOffset is outside the supported 32-bit range";
            return false;
        }

        updatedSettings.m_inputFrequencyOffset = (qint32) offset;
    }
    if (channelSettingsKeys.contains("frequencyMode")) {
        updatedSettings.m_frequencyMode = (MeteorSettings::FrequencyMode) swg->getFrequencyMode();
    }
    if (channelSettingsKeys.contains("frequency")) {
        updatedSettings.m_frequency = swg->getFrequency();
    }
    if (channelSettingsKeys.contains("channelSampleRate")) {
        updatedSettings.m_channelSampleRate = swg->getChannelSampleRate();
    }
    if (channelSettingsKeys.contains("powerLPFCutoff")) {
        updatedSettings.m_powerLPFCutoff = swg->getPowerLpfCutoff();
    }
    if (channelSettingsKeys.contains("detectionThresholdDB")) {
        updatedSettings.m_detectionThresholdDB = swg->getDetectionThresholdDb();
    }
    if (channelSettingsKeys.contains("minDurationMS")) {
        updatedSettings.m_minDurationMS = swg->getMinDurationMs();
    }
    if (channelSettingsKeys.contains("maxDurationMS")) {
        updatedSettings.m_maxDurationMS = swg->getMaxDurationMs();
    }
    if (channelSettingsKeys.contains("maxFrequencyDrift")) {
        updatedSettings.m_maxFrequencyDrift = swg->getMaxFrequencyDrift();
    }
    if (channelSettingsKeys.contains("detectionsTableColumnHidden")) {
        const qint32 columnMask = swg->getDetectionsTableColumnHidden();

        if (columnMask < 0)
        {
            errorMessage = "detectionsTableColumnHidden must not be negative";
            return false;
        }

        updatedSettings.m_detectionsTableColumnHidden = (quint32) columnMask;
    }
    if (channelSettingsKeys.contains("detectionBoxPaddingPixels")) {
        updatedSettings.m_detectionBoxPaddingPixels = swg->getDetectionBoxPaddingPixels();
    }
    if (channelSettingsKeys.contains("detectionLabelMode")) {
        updatedSettings.m_detectionLabelMode = (MeteorSettings::DetectionLabelMode) swg->getDetectionLabelMode();
    }
    if (channelSettingsKeys.contains("transmitterLatitude")) {
        updatedSettings.m_transmitterLatitude = swg->getTransmitterLatitude();
    }
    if (channelSettingsKeys.contains("transmitterLongitude")) {
        updatedSettings.m_transmitterLongitude = swg->getTransmitterLongitude();
    }
    if (channelSettingsKeys.contains("antennaAzimuth")) {
        updatedSettings.m_antennaAzimuth = swg->getAntennaAzimuth();
    }
    if (channelSettingsKeys.contains("antennaElevation")) {
        updatedSettings.m_antennaElevation = swg->getAntennaElevation();
    }
    if (channelSettingsKeys.contains("antennaBeamwidth")) {
        updatedSettings.m_antennaBeamwidth = swg->getAntennaBeamwidth();
    }
    if (channelSettingsKeys.contains("rotator"))
    {
        if (!swg->getRotator())
        {
            errorMessage = "rotator must not be null";
            return false;
        }

        updatedSettings.m_rotator = *swg->getRotator();
    }
    if (channelSettingsKeys.contains("rgbColor")) {
        updatedSettings.m_rgbColor = (quint32) swg->getRgbColor();
    }
    if (channelSettingsKeys.contains("title"))
    {
        if (!swg->getTitle())
        {
            errorMessage = "title must not be null";
            return false;
        }

        updatedSettings.m_title = *swg->getTitle();
    }
    if (channelSettingsKeys.contains("streamIndex")) {
        updatedSettings.m_streamIndex = swg->getStreamIndex();
    }
    if (channelSettingsKeys.contains("workspaceIndex")) {
        updatedSettings.m_workspaceIndex = swg->getWorkspaceIndex();
    }
    if (channelSettingsKeys.contains("hidden"))
    {
        if ((swg->getHidden() != 0) && (swg->getHidden() != 1))
        {
            errorMessage = "hidden must be 0 or 1";
            return false;
        }

        updatedSettings.m_hidden = swg->getHidden() != 0;
    }

    if (!validateChannelSettings(updatedSettings, errorMessage)) {
        return false;
    }

    if (settings.m_channelMarker && channelSettingsKeys.contains("channelMarker") && !swg->getChannelMarker())
    {
        errorMessage = "channelMarker must not be null";
        return false;
    }
    if (settings.m_spectrumGUI && channelSettingsKeys.contains("spectrumGUI") && !swg->getSpectrumGui())
    {
        errorMessage = "spectrumGUI must not be null";
        return false;
    }
    if (settings.m_rollupState && channelSettingsKeys.contains("rollupState") && !swg->getRollupState())
    {
        errorMessage = "rollupState must not be null";
        return false;
    }

    settings = updatedSettings;

    if (settings.m_channelMarker && channelSettingsKeys.contains("channelMarker")) {
        settings.m_channelMarker->updateFrom(channelSettingsKeys, swg->getChannelMarker());
    }
    if (settings.m_spectrumGUI && channelSettingsKeys.contains("spectrumGUI")) {
        settings.m_spectrumGUI->updateFrom(channelSettingsKeys, swg->getSpectrumGui());
    }
    if (settings.m_rollupState && channelSettingsKeys.contains("rollupState")) {
        settings.m_rollupState->updateFrom(channelSettingsKeys, swg->getRollupState());
    }

    return true;
}

bool Meteor::validateChannelSettings(const MeteorSettings& settings, QString& errorMessage)
{
    if ((settings.m_frequencyMode != MeteorSettings::Offset)
        && (settings.m_frequencyMode != MeteorSettings::Absolute))
    {
        errorMessage = "frequencyMode must be 0 (offset) or 1 (absolute)";
        return false;
    }
    if (settings.m_frequency < 0)
    {
        errorMessage = "frequency must not be negative";
        return false;
    }
    if (!MeteorSettings::isSupportedSampleRate(settings.m_channelSampleRate))
    {
        errorMessage = "channelSampleRate must be 100, 300, 1000, or 3000";
        return false;
    }

    const float maxPowerLPFCutoff = settings.m_channelSampleRate * 0.45f;

    if (!std::isfinite(settings.m_powerLPFCutoff)
        || (settings.m_powerLPFCutoff < 0.1f)
        || (settings.m_powerLPFCutoff > maxPowerLPFCutoff))
    {
        errorMessage = QString("powerLPFCutoff must be finite and between 0.1 and %1 Hz")
            .arg(maxPowerLPFCutoff);
        return false;
    }
    if (!std::isfinite(settings.m_detectionThresholdDB)
        || (settings.m_detectionThresholdDB < 1.0f)
        || (settings.m_detectionThresholdDB > 60.0f))
    {
        errorMessage = "detectionThresholdDB must be finite and between 1.0 and 60.0 dB";
        return false;
    }
    if ((settings.m_minDurationMS < 1) || (settings.m_minDurationMS > 10000))
    {
        errorMessage = "minDurationMS must be between 1 and 10000";
        return false;
    }
    if ((settings.m_maxDurationMS < 1) || (settings.m_maxDurationMS > 60000))
    {
        errorMessage = "maxDurationMS must be between 1 and 60000";
        return false;
    }
    if (settings.m_minDurationMS > settings.m_maxDurationMS)
    {
        errorMessage = "minDurationMS must not exceed maxDurationMS";
        return false;
    }
    if (!std::isfinite(settings.m_maxFrequencyDrift)
        || (settings.m_maxFrequencyDrift < 0.0f)
        || (settings.m_maxFrequencyDrift > 1500.0f))
    {
        errorMessage = "maxFrequencyDrift must be finite and between 0.0 and 1500.0 Hz";
        return false;
    }
    if ((settings.m_detectionBoxPaddingPixels < 0) || (settings.m_detectionBoxPaddingPixels > 100))
    {
        errorMessage = "detectionBoxPaddingPixels must be between 0 and 100";
        return false;
    }
    if ((settings.m_detectionLabelMode < MeteorSettings::DetectionLabelNone)
        || (settings.m_detectionLabelMode > MeteorSettings::DetectionLabelRight))
    {
        errorMessage = "detectionLabelMode must be 0 (none), 1 (top), or 2 (right)";
        return false;
    }
    if (!std::isfinite(settings.m_transmitterLatitude)
        || (settings.m_transmitterLatitude < -90.0)
        || (settings.m_transmitterLatitude > 90.0))
    {
        errorMessage = "transmitterLatitude must be finite and between -90.0 and 90.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_transmitterLongitude)
        || (settings.m_transmitterLongitude < -180.0)
        || (settings.m_transmitterLongitude > 180.0))
    {
        errorMessage = "transmitterLongitude must be finite and between -180.0 and 180.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_receiverLatitude)
        || (settings.m_receiverLatitude < -90.0)
        || (settings.m_receiverLatitude > 90.0))
    {
        errorMessage = "receiverLatitude must be finite and between -90.0 and 90.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_receiverLongitude)
        || (settings.m_receiverLongitude < -180.0)
        || (settings.m_receiverLongitude > 180.0))
    {
        errorMessage = "receiverLongitude must be finite and between -180.0 and 180.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_transmitterAzimuth)
        || (settings.m_transmitterAzimuth < 0.0f)
        || (settings.m_transmitterAzimuth > 360.0f))
    {
        errorMessage = "transmitterAzimuth must be finite and between 0.0 and 360.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_transmitterElevation)
        || (settings.m_transmitterElevation < -90.0f)
        || (settings.m_transmitterElevation > 90.0f))
    {
        errorMessage = "transmitterElevation must be finite and between -90.0 and 90.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_transmitterBeamwidth)
        || (settings.m_transmitterBeamwidth < 0.0f)
        || (settings.m_transmitterBeamwidth > 360.0f))
    {
        errorMessage = "transmitterBeamwidth must be finite and between 0.0 and 360.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_transmitterHPBW)
        || (settings.m_transmitterHPBW < 0.0f)
        || (settings.m_transmitterHPBW > 180.0f))
    {
        errorMessage = "transmitterHPBW must be finite and between 0.0 and 180.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_antennaAzimuth)
        || (settings.m_antennaAzimuth < 0.0f)
        || (settings.m_antennaAzimuth > 360.0f))
    {
        errorMessage = "antennaAzimuth must be finite and between 0.0 and 360.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_antennaElevation)
        || (settings.m_antennaElevation < -90.0f)
        || (settings.m_antennaElevation > 90.0f))
    {
        errorMessage = "antennaElevation must be finite and between -90.0 and 90.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_antennaBeamwidth)
        || (settings.m_antennaBeamwidth < 0.0f)
        || (settings.m_antennaBeamwidth > 360.0f))
    {
        errorMessage = "antennaBeamwidth must be finite and between 0.0 and 360.0 degrees";
        return false;
    }
    if (!std::isfinite(settings.m_mapMaxAltitudeKM)
        || (settings.m_mapMaxAltitudeKM < 1.0f)
        || (settings.m_mapMaxAltitudeKM > 50000.0f))
    {
        errorMessage = "mapMaxAltitudeKM must be finite and between 1.0 and 50000.0 km";
        return false;
    }
    if (!settings.m_rotator.isEmpty())
    {
        const QStringList rotatorParts = settings.m_rotator.split(QLatin1Char(':'));
        bool featureSetOK = false;
        bool featureOK = false;

        if (rotatorParts.size() == 2)
        {
            const int featureSetIndex = rotatorParts.at(0).toInt(&featureSetOK);
            const int featureIndex = rotatorParts.at(1).toInt(&featureOK);
            featureSetOK = featureSetOK && (featureSetIndex >= 0);
            featureOK = featureOK && (featureIndex >= 0);
        }

        if (!featureSetOK || !featureOK)
        {
            errorMessage = "rotator must be empty or '<featureSetIndex>:<featureIndex>'";
            return false;
        }
    }
    if (settings.m_streamIndex < 0)
    {
        errorMessage = "streamIndex must not be negative";
        return false;
    }
    if (settings.m_workspaceIndex < 0)
    {
        errorMessage = "workspaceIndex must not be negative";
        return false;
    }

    return true;
}

void Meteor::handleIndexInDeviceSetChanged(int index)
{
    if (index < 0) {
        return;
    }

    QString fifoLabel = QString("%1 [%2:%3]")
        .arg(m_channelId)
        .arg(m_deviceAPI->getDeviceSetIndex())
        .arg(index);
    m_basebandSink->setFifoLabel(fifoLabel);
}
