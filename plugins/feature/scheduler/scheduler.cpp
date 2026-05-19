///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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
#include <QProcess>
#include <QRegularExpression>

#include "SWGDeviceState.h"
#include "SWGFeatureSettings.h"
#include "SWGSchedulerDeviceSetAction.h"
#include "SWGSchedulerFeatureAction.h"
#include "SWGSchedulerRule.h"
#include "SWGSchedulerSettings.h"

#include "channel/channelwebapiutils.h"
#include "device/deviceset.h"
#include "feature/featureset.h"
#include "feature/featurewebapiutils.h"
#include "maincore.h"
#include "settings/serializable.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"
#include "util/messagequeue.h"

#include "scheduler.h"

MESSAGE_CLASS_DEFINITION(Scheduler::MsgConfigureScheduler, Message)

const char* const Scheduler::m_featureIdURI = "sdrangel.feature.scheduler";
const char* const Scheduler::m_featureId = "Scheduler";

namespace
{
QDateTime schedulerDateTimeFromString(const QString *text)
{
    if (!text) {
        return QDateTime();
    }

    QDateTime dateTime = QDateTime::fromString(*text, Qt::ISODateWithMs);

    if (!dateTime.isValid()) {
        dateTime = QDateTime::fromString(*text, Qt::ISODate);
    }

    return dateTime;
}

QString schedulerDateTimeToString(const QDateTime& dateTime)
{
    return dateTime.isValid() ? dateTime.toString(Qt::ISODateWithMs) : QString();
}

bool parseFrequency(const QString& text, double& frequencyInHz)
{
    const QRegularExpression re(
        QStringLiteral("^\\s*([+-]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)\\s*([kKmMgG]?)(?:[hH][zZ])?\\s*$"));
    const QRegularExpressionMatch match = re.match(text);

    if (!match.hasMatch()) {
        return false;
    }

    bool ok = false;
    double value = match.captured(1).toDouble(&ok);
    if (!ok) {
        return false;
    }

    const QString unit = match.captured(2).toLower();
    if (unit == QStringLiteral("k")) {
        value *= 1e3;
    } else if (unit == QStringLiteral("m")) {
        value *= 1e6;
    } else if (unit == QStringLiteral("g")) {
        value *= 1e9;
    }

    frequencyInHz = value;
    return true;
}

bool patchDeviceSetting(int deviceSetIndex, const SchedulerSettings::SettingValue& setting)
{
    switch (setting.m_type)
    {
    case SchedulerSettings::SettingInteger:
        return ChannelWebAPIUtils::patchDeviceSetting(deviceSetIndex, setting.m_name, setting.m_value.toInt());
    case SchedulerSettings::SettingDouble:
        return ChannelWebAPIUtils::patchDeviceSetting(deviceSetIndex, setting.m_name, setting.m_value.toDouble());
    case SchedulerSettings::SettingString:
    default:
        return ChannelWebAPIUtils::patchDeviceSetting(deviceSetIndex, setting.m_name, setting.m_value);
    }
}

bool patchChannelSetting(int deviceSetIndex, int channelIndex, const SchedulerSettings::SettingValue& setting)
{
    switch (setting.m_type)
    {
    case SchedulerSettings::SettingInteger:
        return ChannelWebAPIUtils::patchChannelSetting(deviceSetIndex, channelIndex, setting.m_name, setting.m_value.toInt());
    case SchedulerSettings::SettingDouble:
        return ChannelWebAPIUtils::patchChannelSetting(deviceSetIndex, channelIndex, setting.m_name, setting.m_value.toDouble());
    case SchedulerSettings::SettingString:
    default:
        return ChannelWebAPIUtils::patchChannelSetting(deviceSetIndex, channelIndex, setting.m_name, setting.m_value);
    }
}

bool patchFeatureSetting(int featureSetIndex, int featureIndex, const SchedulerSettings::SettingValue& setting)
{
    switch (setting.m_type)
    {
    case SchedulerSettings::SettingInteger:
        return ChannelWebAPIUtils::patchFeatureSetting(featureSetIndex, featureIndex, setting.m_name, setting.m_value.toInt());
    case SchedulerSettings::SettingDouble:
        return ChannelWebAPIUtils::patchFeatureSetting(featureSetIndex, featureIndex, setting.m_name, setting.m_value.toDouble());
    case SchedulerSettings::SettingString:
    default:
        return ChannelWebAPIUtils::patchFeatureSetting(featureSetIndex, featureIndex, setting.m_name, setting.m_value);
    }
}
}

Scheduler::Scheduler(WebAPIAdapterInterface *webAPIAdapterInterface) :
    Feature(m_featureIdURI, webAPIAdapterInterface),
    m_eventSourceHandler(QStringList(), QStringList({QStringLiteral("event")}), QStringLiteral("RTMF"))
#ifdef QT_TEXTTOSPEECH_FOUND
    , m_speech(new QTextToSpeech(this))
#endif
{
    qDebug("Scheduler::Scheduler: webAPIAdapterInterface: %p", webAPIAdapterInterface);
    setObjectName(m_featureId);
    m_state = StRunning;

    connect(&m_timer, &QTimer::timeout, this, &Scheduler::processDueRules);
    m_timer.start(1000);

    connect(
        &m_eventSourceHandler,
        &AvailableChannelOrFeatureHandler::messageEnqueued,
        this,
        &Scheduler::eventMessageEnqueued,
        Qt::QueuedConnection
    );
    m_eventSourceHandler.scanAvailableChannelsAndFeatures();
}

Scheduler::~Scheduler()
{
}

bool Scheduler::handleMessage(const Message& cmd)
{
    if (MsgConfigureScheduler::match(cmd))
    {
        const MsgConfigureScheduler& cfg = (const MsgConfigureScheduler&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }

    return false;
}

QByteArray Scheduler::serialize() const
{
    return m_settings.serialize();
}

bool Scheduler::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        m_inputMessageQueue.push(MsgConfigureScheduler::create(m_settings, QStringList(), true));
        return true;
    }

    m_settings.resetToDefaults();
    m_inputMessageQueue.push(MsgConfigureScheduler::create(m_settings, QStringList(), true));
    return false;
}

int Scheduler::webapiRun(bool run, SWGSDRangel::SWGDeviceState& response, QString& errorMessage)
{
    (void) run;
    (void) errorMessage;
    response.setState(new QString(QStringLiteral("running")));
    return 200;
}

int Scheduler::webapiSettingsGet(
        SWGSDRangel::SWGFeatureSettings& response,
        QString& errorMessage)
{
    (void) errorMessage;
    response.setSchedulerSettings(new SWGSDRangel::SWGSchedulerSettings());
    response.getSchedulerSettings()->init();
    webapiFormatFeatureSettings(response, m_settings);
    return 200;
}

int Scheduler::webapiSettingsPutPatch(
        bool force,
        const QStringList& featureSettingsKeys,
        SWGSDRangel::SWGFeatureSettings& response,
        QString& errorMessage)
{
    (void) errorMessage;
    SchedulerSettings settings = m_settings;
    webapiUpdateFeatureSettings(settings, featureSettingsKeys, response);

    MsgConfigureScheduler *msg = MsgConfigureScheduler::create(settings, featureSettingsKeys, force);
    m_inputMessageQueue.push(msg);

    if (m_guiMessageQueue)
    {
        MsgConfigureScheduler *msgToGUI = MsgConfigureScheduler::create(settings, featureSettingsKeys, force);
        m_guiMessageQueue->push(msgToGUI);
    }

    webapiFormatFeatureSettings(response, settings);
    return 200;
}

void Scheduler::webapiFormatFeatureSettings(
        SWGSDRangel::SWGFeatureSettings& response,
        const SchedulerSettings& settings)
{
    SWGSDRangel::SWGSchedulerSettings *swg = response.getSchedulerSettings();

    if (swg->getTitle()) {
        *swg->getTitle() = settings.m_title;
    } else {
        swg->setTitle(new QString(settings.m_title));
    }

    swg->setRgbColor(settings.m_rgbColor);
    swg->setWorkspaceIndex(settings.m_workspaceIndex);

    QList<SWGSDRangel::SWGSchedulerRule*> *swgRules = new QList<SWGSDRangel::SWGSchedulerRule*>();

    for (const SchedulerSettings::ScheduleRule& rule : settings.m_rules)
    {
        SWGSDRangel::SWGSchedulerRule *swgRule = new SWGSDRangel::SWGSchedulerRule();
        swgRule->init();
        swgRule->setId(new QString(rule.m_id));
        swgRule->setName(new QString(rule.m_name));
        swgRule->setEnabled(rule.m_enabled ? 1 : 0);
        swgRule->setTriggerType((int) rule.m_triggerType);
        swgRule->setTime(new QString(schedulerDateTimeToString(rule.m_time)));
        swgRule->setRecurrence((int) rule.m_recurrence);
        swgRule->setEventType(rule.m_eventType);
        swgRule->setEventSourceId(new QString(rule.m_eventSourceId));
        swgRule->setEventDataRegex(new QString(rule.m_eventDataRegex));
        swgRule->setEventDelay(rule.m_eventDelay);
        swgRule->setEventDelayUnit((int) rule.m_eventDelayUnit);
        swgRule->setCommand(new QString(rule.m_command));
        swgRule->setSpeech(new QString(rule.m_speech));

        QList<SWGSDRangel::SWGSchedulerDeviceSetAction*> *swgDeviceActions =
            new QList<SWGSDRangel::SWGSchedulerDeviceSetAction*>();
        for (const SchedulerSettings::DeviceSetAction& action : rule.m_deviceSetActions)
        {
            SWGSDRangel::SWGSchedulerDeviceSetAction *swgAction =
                new SWGSDRangel::SWGSchedulerDeviceSetAction();
            swgAction->init();
            swgAction->setDeviceSetIndex(action.m_deviceSetIndex);
            swgAction->setDeviceSetId(new QString(action.m_deviceSetId));
            swgAction->setPresetGroup(new QString(action.m_presetGroup));
            swgAction->setPresetFrequency((qint64) action.m_presetFrequency);
            swgAction->setPresetDescription(new QString(action.m_presetDescription));
            swgAction->setAcquisitionAction((int) action.m_acquisitionAction);
            swgAction->setFileSinkAction((int) action.m_fileSinkAction);
            double centerFrequency = 0.0;
            swgAction->setOverrideCenterFrequency(parseFrequency(action.m_centerFrequency, centerFrequency) ? 1 : 0);
            swgAction->setCenterFrequency((qint64) qRound64(centerFrequency));
            swgDeviceActions->append(swgAction);
        }
        swgRule->setDeviceSetActions(swgDeviceActions);

        QList<SWGSDRangel::SWGSchedulerFeatureAction*> *swgFeatureActions =
            new QList<SWGSDRangel::SWGSchedulerFeatureAction*>();
        for (const SchedulerSettings::FeatureAction& action : rule.m_featureActions)
        {
            SWGSDRangel::SWGSchedulerFeatureAction *swgAction =
                new SWGSDRangel::SWGSchedulerFeatureAction();
            swgAction->init();
            swgAction->setFeatureSetIndex(action.m_featureSetIndex);
            swgAction->setFeatureIndex(action.m_featureIndex);
            swgAction->setFeatureId(new QString(action.m_featureId));
            swgAction->setAction((int) action.m_action);
            swgFeatureActions->append(swgAction);
        }
        swgRule->setFeatureActions(swgFeatureActions);
        swgRule->setLastRun(new QString(schedulerDateTimeToString(rule.m_lastRun)));
        swgRules->append(swgRule);
    }

    swg->setRules(swgRules);

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

void Scheduler::webapiUpdateFeatureSettings(
        SchedulerSettings& settings,
        const QStringList& featureSettingsKeys,
        SWGSDRangel::SWGFeatureSettings& response)
{
    SWGSDRangel::SWGSchedulerSettings *swg = response.getSchedulerSettings();

    if (featureSettingsKeys.contains("title")) {
        settings.m_title = *swg->getTitle();
    }
    if (featureSettingsKeys.contains("rgbColor")) {
        settings.m_rgbColor = swg->getRgbColor();
    }
    if (featureSettingsKeys.contains("workspaceIndex")) {
        settings.m_workspaceIndex = swg->getWorkspaceIndex();
    }
    if (featureSettingsKeys.contains("rules"))
    {
        settings.m_rules.clear();

        if (swg->getRules())
        {
            for (auto *swgRule : *swg->getRules())
            {
                if (!swgRule) {
                    continue;
                }

                SchedulerSettings::ScheduleRule rule;
                rule.m_id = swgRule->getId() ? *swgRule->getId() : SchedulerSettings::newRuleId();
                if (rule.m_id.isEmpty()) {
                    rule.m_id = SchedulerSettings::newRuleId();
                }
                rule.m_name = swgRule->getName() ? *swgRule->getName() : QString();
                rule.m_enabled = swgRule->getEnabled() != 0;
                rule.m_triggerType = (SchedulerSettings::TriggerType) swgRule->getTriggerType();
                rule.m_time = schedulerDateTimeFromString(swgRule->getTime());
                rule.m_recurrence = (SchedulerSettings::Recurrence) swgRule->getRecurrence();
                rule.m_eventType = swgRule->getEventType();
                rule.m_eventSourceId = swgRule->getEventSourceId() ? *swgRule->getEventSourceId() : QString();
                rule.m_eventDataRegex = swgRule->getEventDataRegex() ? *swgRule->getEventDataRegex() : QString();
                rule.m_eventDelay = swgRule->getEventDelay();
                rule.m_eventDelayUnit = (SchedulerSettings::DelayUnit) swgRule->getEventDelayUnit();
                rule.m_command = swgRule->getCommand() ? *swgRule->getCommand() : QString();
                rule.m_speech = swgRule->getSpeech() ? *swgRule->getSpeech() : QString();

                if (swgRule->getDeviceSetActions())
                {
                    for (auto *swgAction : *swgRule->getDeviceSetActions())
                    {
                        if (!swgAction) {
                            continue;
                        }

                        SchedulerSettings::DeviceSetAction action;
                        action.m_deviceSetIndex = swgAction->getDeviceSetIndex();
                        action.m_deviceSetId = swgAction->getDeviceSetId() ? *swgAction->getDeviceSetId() : QString();
                        action.m_presetGroup = swgAction->getPresetGroup() ? *swgAction->getPresetGroup() : QString();
                        action.m_presetFrequency = (quint64) swgAction->getPresetFrequency();
                        action.m_presetDescription = swgAction->getPresetDescription() ? *swgAction->getPresetDescription() : QString();
                        action.m_acquisitionAction = (SchedulerSettings::RunAction) swgAction->getAcquisitionAction();
                        action.m_fileSinkAction = (SchedulerSettings::RunAction) swgAction->getFileSinkAction();
                        action.m_centerFrequency = swgAction->getOverrideCenterFrequency() != 0
                            ? QString::number(swgAction->getCenterFrequency())
                            : QString();
                        rule.m_deviceSetActions.append(action);
                    }
                }

                if (swgRule->getFeatureActions())
                {
                    for (auto *swgAction : *swgRule->getFeatureActions())
                    {
                        if (!swgAction) {
                            continue;
                        }

                        SchedulerSettings::FeatureAction action;
                        action.m_featureSetIndex = swgAction->getFeatureSetIndex();
                        action.m_featureIndex = swgAction->getFeatureIndex();
                        action.m_featureId = swgAction->getFeatureId() ? *swgAction->getFeatureId() : QString();
                        action.m_action = (SchedulerSettings::RunAction) swgAction->getAction();
                        rule.m_featureActions.append(action);
                    }
                }

                rule.m_lastRun = schedulerDateTimeFromString(swgRule->getLastRun());
                settings.m_rules.append(rule);
            }
        }
    }
    if (settings.m_rollupState && featureSettingsKeys.contains("rollupState")) {
        settings.m_rollupState->updateFrom(featureSettingsKeys, swg->getRollupState());
    }
}

QStringList Scheduler::eventTypeNames()
{
    return QStringList({
        QStringLiteral("Satellite AOS"),
        QStringLiteral("Satellite LOS"),
        QStringLiteral("ADS-B Aircraft Detected"),
        QStringLiteral("ADS-B Aircraft Lost"),
        QStringLiteral("ADS-B Notification"),
        QStringLiteral("AIS Ship Detected"),
        QStringLiteral("AIS Ship Lost"),
        QStringLiteral("Star Rise"),
        QStringLiteral("Star Set"),
        QStringLiteral("Meteor Scatter"),
        QStringLiteral("Camera Object Detected"),
        QStringLiteral("Camera Object Lost"),
        QStringLiteral("Camera Motion Detected"),
        QStringLiteral("Camera Motion Stopped"),
        QStringLiteral("Camera Object In View"),
        QStringLiteral("Camera Object Out Of View"),
        QStringLiteral("Packet Received"),
        QStringLiteral("Squelch Open"),
        QStringLiteral("Squelch Closed"),
        QStringLiteral("CTCSS Frequency"),
        QStringLiteral("DCS Code"),
        });
}

QString Scheduler::eventTypeName(int eventType)
{
    const QStringList names = eventTypeNames();
    if ((eventType >= 0) && (eventType < names.size())) {
        return names[eventType];
    } else {
        return QStringLiteral("Event %1").arg(eventType);
    }
}

void Scheduler::applySettings(const SchedulerSettings& settings, const QStringList& settingsKeys, bool force)
{
    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (force || settingsKeys.contains("rules")) {
        rebuildNextRuns();
    }
}

void Scheduler::rebuildNextRuns()
{
    m_nextRuns.clear();
    const QDateTime now = QDateTime::currentDateTime();

    for (const SchedulerSettings::ScheduleRule& rule : m_settings.m_rules) {
        updateNextRun(rule, now);
    }
}

void Scheduler::updateNextRun(const SchedulerSettings::ScheduleRule& rule, const QDateTime& after)
{
    if ((rule.m_triggerType == SchedulerSettings::TriggerTime) && rule.m_enabled) {
        m_nextRuns[rule.m_id] = SchedulerSettings::nextDateTime(rule, after);
    } else {
        m_nextRuns.remove(rule.m_id);
    }
}

void Scheduler::processDueRules()
{
    const QDateTime now = QDateTime::currentDateTime();
    bool updated = false;

    for (SchedulerSettings::ScheduleRule& rule : m_settings.m_rules)
    {
        if (!rule.m_enabled || (rule.m_triggerType != SchedulerSettings::TriggerTime)) {
            continue;
        }

        if (!m_nextRuns.contains(rule.m_id)) {
            updateNextRun(rule, now);
        }

        const QDateTime nextRun = m_nextRuns.value(rule.m_id);
        if (nextRun.isValid() && (nextRun <= now))
        {
            ExecutionContext context;
            context.m_trigger = QStringLiteral("time");
            context.m_dateTime = nextRun;
            context.m_eventType = -1;
            executeRule(rule, context);
            updateNextRun(rule, now.addSecs(1));
            updated = true;
        }
    }

    if (updated) {
        notifyGUI(QStringList({QStringLiteral("rules")}));
    }
}

void Scheduler::eventMessageEnqueued(MessageQueue *messageQueue)
{
    handleEventMessageQueue(messageQueue);
}

void Scheduler::handleEventMessageQueue(MessageQueue *messageQueue)
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

void Scheduler::handleEvent(const MainCore::MsgEvent& eventMessage)
{
    QString sourceId;

    for (const SchedulerSettings::ScheduleRule& rule : m_settings.m_rules)
    {
        if (!ruleMatchesEvent(rule, eventMessage, sourceId)) {
            continue;
        }

        ExecutionContext context;
        context.m_trigger = QStringLiteral("event");
        context.m_dateTime = eventMessage.getDateTime();
        context.m_eventType = static_cast<int>(eventMessage.getEvent());
        context.m_eventName = eventTypeName(context.m_eventType);
        context.m_source = sourceId;
        context.m_data = eventMessage.getData();

        const QString ruleId = rule.m_id;
        const int delayMs = SchedulerSettings::delaySeconds(rule) * 1000;

        if (delayMs > 0)
        {
            QTimer::singleShot(delayMs, this, [this, ruleId, context]() {
                executeRuleById(ruleId, context);
            });
        }
        else
        {
            executeRuleById(ruleId, context);
        }
    }
}

bool Scheduler::ruleMatchesEvent(const SchedulerSettings::ScheduleRule& rule, const MainCore::MsgEvent& eventMessage, QString& sourceId) const
{
    if (!rule.m_enabled || (rule.m_triggerType != SchedulerSettings::TriggerEvent)) {
        return false;
    }

    if (rule.m_eventType != static_cast<int>(eventMessage.getEvent())) {
        return false;
    }

    sourceId = sourceIdForObject(eventMessage.getPipeSource());

    if (!rule.m_eventSourceId.isEmpty() && (rule.m_eventSourceId != sourceId)) {
        return false;
    }

    if (!rule.m_eventDataRegex.isEmpty())
    {
        const QRegularExpression re(rule.m_eventDataRegex);
        if (!re.isValid()) {
            return false;
        }

        if (!re.match(eventMessage.getData()).hasMatch()) {
            return false;
        }
    }

    return true;
}

void Scheduler::executeRuleById(const QString& ruleId, const ExecutionContext& context)
{
    bool updated = false;

    for (SchedulerSettings::ScheduleRule& rule : m_settings.m_rules)
    {
        if (rule.m_id == ruleId)
        {
            if (rule.m_enabled) {
                executeRule(rule, context);
                updated = true;
            }
            break;
        }
    }

    if (updated) {
        notifyGUI(QStringList({QStringLiteral("rules")}));
    }
}

void Scheduler::executeRule(SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context)
{
    qDebug() << "Scheduler::executeRule:" << rule.m_name << context.m_trigger;
    rule.m_lastRun = QDateTime::currentDateTime();
    executeRuleActions(rule, context);
}

void Scheduler::executeRuleActions(const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context)
{
    const QList<SchedulerSettings::DeviceSetAction> deviceActions = rule.m_deviceSetActions;
    const QList<SchedulerSettings::ChannelAction> channelActions = rule.m_channelActions;
    const QList<SchedulerSettings::FeatureAction> featureActions = rule.m_featureActions;

    auto loadPreset = [](int deviceSetIndex, const QString& presetGroup, quint64 presetFrequency, const QString& presetDescription) {
        if (presetGroup.isEmpty()) {
            return;
        }

        MainCore *mainCore = MainCore::instance();
        const std::vector<DeviceSet*>& deviceSets = mainCore->getDeviceSets();

        if ((deviceSetIndex < 0) || (deviceSetIndex >= (int) deviceSets.size()))
        {
            qWarning() << "Scheduler::executeRuleActions: no device set" << deviceSetIndex;
            return;
        }

        const DeviceSet *deviceSet = deviceSets[deviceSetIndex];
        QString presetType;

        if (deviceSet->m_deviceSourceEngine != nullptr) {
            presetType = QStringLiteral("R");
        } else if (deviceSet->m_deviceSinkEngine != nullptr) {
            presetType = QStringLiteral("T");
        } else if (deviceSet->m_deviceMIMOEngine != nullptr) {
            presetType = QStringLiteral("M");
        }

        const Preset *preset = mainCore->getSettings().getPreset(
            presetGroup,
            presetFrequency,
            presetDescription,
            presetType);

        if (preset)
        {
            mainCore->getMainMessageQueue()->push(MainCore::MsgLoadPreset::create(preset, deviceSetIndex));
        }
        else
        {
            qWarning() << "Scheduler::executeRuleActions: unable to find preset"
                       << presetGroup << presetFrequency << presetDescription;
        }
    };

    for (const SchedulerSettings::DeviceSetAction& action : deviceActions)
    {
        loadPreset(action.m_deviceSetIndex, action.m_presetGroup, action.m_presetFrequency, action.m_presetDescription);
    }

    QTimer::singleShot(1000, this, [this, rule, context, deviceActions, channelActions, featureActions]() {
        executeDeviceActions(deviceActions);
        executeChannelActions(channelActions);
        executeFeatureActions(featureActions);
        executeCommand(rule.m_command, rule, context);
        saySpeech(rule.m_speech, rule, context);

        const int durationSeconds = SchedulerSettings::durationSeconds(rule);
        if (durationSeconds > 0)
        {
            const int durationMs = qMin(durationSeconds, 2147483) * 1000;
            QTimer::singleShot(durationMs, this, [this, deviceActions, channelActions, featureActions]() {
                executeDeviceDurationStops(deviceActions);
                executeChannelDurationStops(channelActions);
                executeFeatureDurationStops(featureActions);
            });
        }
    });
}

void Scheduler::executeDeviceActions(const QList<SchedulerSettings::DeviceSetAction>& actions)
{
    for (const SchedulerSettings::DeviceSetAction& action : actions)
    {
        double centerFrequency = 0.0;
        if (!action.m_centerFrequency.isEmpty() && parseFrequency(action.m_centerFrequency, centerFrequency))
        {
            qDebug() << "Scheduler::executeDeviceActions: set center frequency"
                     << centerFrequency << "on device set" << action.m_deviceSetIndex;
            ChannelWebAPIUtils::setCenterFrequency(action.m_deviceSetIndex, centerFrequency);
        }
        else if (!action.m_centerFrequency.isEmpty())
        {
            qWarning() << "Scheduler::executeDeviceActions: invalid center frequency"
                       << action.m_centerFrequency << "on device set" << action.m_deviceSetIndex;
        }
    }

    for (const SchedulerSettings::DeviceSetAction& action : actions)
    {
        if (action.m_acquisitionAction == SchedulerSettings::ActionStart) {
            ChannelWebAPIUtils::run(action.m_deviceSetIndex);
        } else if (action.m_acquisitionAction == SchedulerSettings::ActionStop) {
            ChannelWebAPIUtils::stop(action.m_deviceSetIndex);
        }

        for (const SchedulerSettings::SettingValue& setting : action.m_settings)
        {
            patchDeviceSetting(action.m_deviceSetIndex, setting);
        }
    }

    for (const SchedulerSettings::DeviceSetAction& action : actions)
    {
        if (action.m_fileSinkAction == SchedulerSettings::ActionStart) {
            ChannelWebAPIUtils::startStopFileSinks(action.m_deviceSetIndex, true);
        } else if (action.m_fileSinkAction == SchedulerSettings::ActionStop) {
            ChannelWebAPIUtils::startStopFileSinks(action.m_deviceSetIndex, false);
        }
    }
}

void Scheduler::executeChannelActions(const QList<SchedulerSettings::ChannelAction>& actions)
{
    for (const SchedulerSettings::ChannelAction& action : actions)
    {
        switch (action.m_action)
        {
        case SchedulerSettings::ActionFileSinkRecordStart:
            ChannelWebAPIUtils::fileSinkRecord(action.m_deviceSetIndex, action.m_channelIndex, true);
            break;
        case SchedulerSettings::ActionFileSinkRecordStop:
            ChannelWebAPIUtils::fileSinkRecord(action.m_deviceSetIndex, action.m_channelIndex, false);
            break;
        case SchedulerSettings::ActionSigMFRecordStart:
            ChannelWebAPIUtils::sigMFRecord(action.m_deviceSetIndex, action.m_channelIndex, true);
            break;
        case SchedulerSettings::ActionSigMFRecordStop:
            ChannelWebAPIUtils::sigMFRecord(action.m_deviceSetIndex, action.m_channelIndex, false);
            break;
        case SchedulerSettings::ActionRTTYTransmit:
            ChannelWebAPIUtils::rttyModTransmit(action.m_deviceSetIndex, action.m_channelIndex, action.m_text);
            break;
        case SchedulerSettings::ActionPSK31Transmit:
            ChannelWebAPIUtils::psk31ModTransmit(action.m_deviceSetIndex, action.m_channelIndex, action.m_text);
            break;
        case SchedulerSettings::ActionPacketTransmit:
            ChannelWebAPIUtils::packetModTransmit(
                action.m_deviceSetIndex,
                action.m_channelIndex,
                action.m_callsign,
                action.m_to,
                action.m_via,
                action.m_data);
            break;
        case SchedulerSettings::ActionIEEE_802_15_4Transmit:
            ChannelWebAPIUtils::ieee_802_15_4Transmit(action.m_deviceSetIndex, action.m_channelIndex, action.m_data);
            break;
        case SchedulerSettings::ActionAISTransmit:
            ChannelWebAPIUtils::aisModTransmit(action.m_deviceSetIndex, action.m_channelIndex, action.m_data);
            break;
        case SchedulerSettings::ActionFreqScannerRun:
            ChannelWebAPIUtils::freqScannerRun(action.m_deviceSetIndex, action.m_channelIndex, true);
            break;
        case SchedulerSettings::ActionFreqScannerStop:
            ChannelWebAPIUtils::freqScannerRun(action.m_deviceSetIndex, action.m_channelIndex, false);
            break;
        case SchedulerSettings::ActionRadioAstronomyStart:
            ChannelWebAPIUtils::radioAstronomyStart(action.m_deviceSetIndex, action.m_channelIndex);
            break;
        case SchedulerSettings::ActionApplySetting:
            for (const SchedulerSettings::SettingValue& setting : action.m_settings) {
                patchChannelSetting(action.m_deviceSetIndex, action.m_channelIndex, setting);
            }
            break;
        default:
            break;
        }
    }
}

void Scheduler::executeFeatureActions(const QList<SchedulerSettings::FeatureAction>& actions)
{
    for (const SchedulerSettings::FeatureAction& action : actions)
    {
        if (action.m_action == SchedulerSettings::ActionStart) {
            FeatureWebAPIUtils::run(action.m_featureSetIndex, action.m_featureIndex);
        } else if (action.m_action == SchedulerSettings::ActionStop) {
            FeatureWebAPIUtils::stop(action.m_featureSetIndex, action.m_featureIndex);
        } else if ((action.m_action == SchedulerSettings::ActionCameraSaveImage)
            && (action.m_featureId == "sdrangel.feature.camera"))
        {
            FeatureWebAPIUtils::cameraSaveImage(
                action.m_cameraFilename,
                action.m_cameraRecordMode,
                action.m_cameraImageCount,
                action.m_featureSetIndex,
                action.m_featureIndex);
        } else if ((action.m_action == SchedulerSettings::ActionCameraRecordVideo)
            && (action.m_featureId == "sdrangel.feature.camera"))
        {
            FeatureWebAPIUtils::cameraRecordVideo(
                action.m_cameraFilename,
                action.m_cameraRecordMode,
                action.m_cameraVideoDuration,
                action.m_featureSetIndex,
                action.m_featureIndex);
        } else if ((action.m_action == SchedulerSettings::ActionMapFind)
            && (action.m_featureId == "sdrangel.feature.map"))
        {
            FeatureWebAPIUtils::mapFind(
                action.m_findTarget,
                action.m_featureSetIndex,
                action.m_featureIndex);
        } else if ((action.m_action == SchedulerSettings::ActionMapFind)
            && (action.m_featureId == "sdrangel.feature.skymap"))
        {
            FeatureWebAPIUtils::skyMapFind(
                action.m_findTarget,
                action.m_featureSetIndex,
                action.m_featureIndex);
        } else if (action.m_action == SchedulerSettings::ActionApplySetting)
        {
            for (const SchedulerSettings::SettingValue& setting : action.m_settings) {
                patchFeatureSetting(action.m_featureSetIndex, action.m_featureIndex, setting);
            }
        }
    }
}

void Scheduler::executeDeviceDurationStops(const QList<SchedulerSettings::DeviceSetAction>& actions)
{
    for (const SchedulerSettings::DeviceSetAction& action : actions)
    {
        if (action.m_acquisitionAction == SchedulerSettings::ActionStart) {
            ChannelWebAPIUtils::stop(action.m_deviceSetIndex);
        }

        if (action.m_fileSinkAction == SchedulerSettings::ActionStart) {
            ChannelWebAPIUtils::startStopFileSinks(action.m_deviceSetIndex, false);
        }
    }
}

void Scheduler::executeChannelDurationStops(const QList<SchedulerSettings::ChannelAction>& actions)
{
    for (const SchedulerSettings::ChannelAction& action : actions)
    {
        switch (action.m_action)
        {
        case SchedulerSettings::ActionFileSinkRecordStart:
            ChannelWebAPIUtils::fileSinkRecord(action.m_deviceSetIndex, action.m_channelIndex, false);
            break;
        case SchedulerSettings::ActionSigMFRecordStart:
            ChannelWebAPIUtils::sigMFRecord(action.m_deviceSetIndex, action.m_channelIndex, false);
            break;
        case SchedulerSettings::ActionFreqScannerRun:
            ChannelWebAPIUtils::freqScannerRun(action.m_deviceSetIndex, action.m_channelIndex, false);
            break;
        default:
            break;
        }
    }
}

void Scheduler::executeFeatureDurationStops(const QList<SchedulerSettings::FeatureAction>& actions)
{
    for (const SchedulerSettings::FeatureAction& action : actions)
    {
        if (action.m_action == SchedulerSettings::ActionStart) {
            FeatureWebAPIUtils::stop(action.m_featureSetIndex, action.m_featureIndex);
        }
    }
}

void Scheduler::executeCommand(const QString& command, const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context)
{
    if (command.isEmpty()) {
        return;
    }

#if QT_CONFIG(process)
    QStringList allArgs = QProcess::splitCommand(substitute(command, rule, context));
    if (allArgs.isEmpty()) {
        return;
    }

    const QString program = allArgs.takeFirst();
    QProcess::startDetached(program, allArgs);
#else
    qWarning() << "Scheduler::executeCommand: QProcess not supported. Can't run:" << command;
#endif
}

void Scheduler::saySpeech(const QString& speech, const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context)
{
    if (speech.isEmpty()) {
        return;
    }

#ifdef QT_TEXTTOSPEECH_FOUND
    m_speech->say(substitute(speech, rule, context));
#else
    qWarning() << "Scheduler::saySpeech: TextToSpeech not supported. Unable to say" << speech;
#endif
}

QString Scheduler::substitute(const QString& text, const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context) const
{
    QString result = text;
    result.replace(QStringLiteral("${rule}"), rule.m_name);
    result.replace(QStringLiteral("${trigger}"), context.m_trigger);
    result.replace(QStringLiteral("${dateTime}"), context.m_dateTime.toString(Qt::ISODateWithMs));
    result.replace(QStringLiteral("${event}"), context.m_eventName);
    result.replace(QStringLiteral("${source}"), context.m_source);
    result.replace(QStringLiteral("${data}"), context.m_data);
    return result;
}

QString Scheduler::sourceIdForObject(const QObject *object) const
{
    const AvailableChannelOrFeatureList& entries = m_eventSourceHandler.getAvailableChannelOrFeatureList();

    for (const AvailableChannelOrFeature& entry : entries)
    {
        if (entry.m_object == object) {
            return entry.getLongId();
        }
    }

    return object ? object->objectName() : QString();
}

void Scheduler::notifyGUI(const QStringList& settingsKeys)
{
    if (m_guiMessageQueue) {
        m_guiMessageQueue->push(MsgConfigureScheduler::create(m_settings, settingsKeys, false));
    }
}
