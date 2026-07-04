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

Meteor::Meteor(DeviceAPI *deviceAPI) :
    ChannelAPI(m_channelIdURI, ChannelAPI::StreamSingleSink),
    m_deviceAPI(deviceAPI),
    m_spectrumVis(SDR_RX_SCALEF),
    m_basebandSampleRate(0),
    m_centerFrequency(0),
    m_eventSourceHandler(
        QStringList({QStringLiteral("sdrangel.feature.camera")}),
        QStringList({QStringLiteral("event")}),
        QStringLiteral("F"))
{
    setObjectName(m_channelId);

    m_basebandSink = new MeteorBaseband();
    m_basebandSink->setScopeSink(&m_scopeVis);
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
        if (!m_cameraMeteorStartTimes.contains(source)) {
            m_cameraMeteorStartTimes.insert(source, eventTime);
        }

        return;
    }

    if (eventMessage.getEvent() != MainCore::MsgEvent::CameraObjectLostEvent) {
        return;
    }

    const auto startIt = m_cameraMeteorStartTimes.find(source);

    if (startIt == m_cameraMeteorStartTimes.end()) {
        return;
    }

    const QDateTime startTime = startIt.value();
    m_cameraMeteorStartTimes.erase(startIt);

    const qint64 durationMSecs = startTime.msecsTo(eventTime);

    if (durationMSecs < 0) {
        return;
    }

    if (getMessageQueueToGUI())
    {
        getMessageQueueToGUI()->push(MsgCameraMeteorDetected::create(
            startTime.toUTC(),
            (double) durationMSecs / 1000.0));
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
    (void) errorMessage;
    MeteorSettings settings = m_settings;
    webapiUpdateChannelSettings(settings, channelSettingsKeys, response);

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

    if (settings.m_scopeGUI)
    {
        if (swg->getScopeGui()) {
            settings.m_scopeGUI->formatTo(swg->getScopeGui());
        } else {
            SWGSDRangel::SWGGLScope *swgScopeGUI = new SWGSDRangel::SWGGLScope();
            settings.m_scopeGUI->formatTo(swgScopeGUI);
            swg->setScopeGui(swgScopeGUI);
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

void Meteor::webapiUpdateChannelSettings(
        MeteorSettings& settings,
        const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response)
{
    SWGSDRangel::SWGMeteorSettings *swg = response.getMeteorSettings();

    if (channelSettingsKeys.contains("inputFrequencyOffset")) {
        settings.m_inputFrequencyOffset = swg->getInputFrequencyOffset();
    }
    if (channelSettingsKeys.contains("frequencyMode")) {
        settings.m_frequencyMode = (MeteorSettings::FrequencyMode) swg->getFrequencyMode();
    }
    if (channelSettingsKeys.contains("frequency")) {
        settings.m_frequency = swg->getFrequency();
    }
    if (channelSettingsKeys.contains("channelSampleRate")) {
        settings.m_channelSampleRate = swg->getChannelSampleRate();
    }
    if (channelSettingsKeys.contains("powerLPFCutoff")) {
        settings.m_powerLPFCutoff = swg->getPowerLpfCutoff();
    }
    if (channelSettingsKeys.contains("detectionThresholdDB")) {
        settings.m_detectionThresholdDB = swg->getDetectionThresholdDb();
    }
    if (channelSettingsKeys.contains("minDurationMS")) {
        settings.m_minDurationMS = swg->getMinDurationMs();
    }
    if (channelSettingsKeys.contains("maxDurationMS")) {
        settings.m_maxDurationMS = swg->getMaxDurationMs();
    }
    if (channelSettingsKeys.contains("maxFrequencyDrift")) {
        settings.m_maxFrequencyDrift = swg->getMaxFrequencyDrift();
    }
    if (channelSettingsKeys.contains("detectionsTableColumnHidden")) {
        settings.m_detectionsTableColumnHidden = swg->getDetectionsTableColumnHidden();
    }
    if (channelSettingsKeys.contains("detectionBoxPaddingPixels")) {
        settings.m_detectionBoxPaddingPixels = swg->getDetectionBoxPaddingPixels();
    }
    if (channelSettingsKeys.contains("rgbColor")) {
        settings.m_rgbColor = swg->getRgbColor();
    }
    if (channelSettingsKeys.contains("title")) {
        settings.m_title = *swg->getTitle();
    }
    if (channelSettingsKeys.contains("streamIndex")) {
        settings.m_streamIndex = swg->getStreamIndex();
    }
    if (channelSettingsKeys.contains("workspaceIndex")) {
        settings.m_workspaceIndex = swg->getWorkspaceIndex();
    }
    if (channelSettingsKeys.contains("hidden")) {
        settings.m_hidden = swg->getHidden() != 0;
    }
    if (settings.m_channelMarker && channelSettingsKeys.contains("channelMarker")) {
        settings.m_channelMarker->updateFrom(channelSettingsKeys, swg->getChannelMarker());
    }
    if (settings.m_spectrumGUI && channelSettingsKeys.contains("spectrumGUI")) {
        settings.m_spectrumGUI->updateFrom(channelSettingsKeys, swg->getSpectrumGui());
    }
    if (settings.m_scopeGUI && channelSettingsKeys.contains("scopeGUI")) {
        settings.m_scopeGUI->updateFrom(channelSettingsKeys, swg->getScopeGui());
    }
    if (settings.m_rollupState && channelSettingsKeys.contains("rollupState")) {
        settings.m_rollupState->updateFrom(channelSettingsKeys, swg->getRollupState());
    }
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
