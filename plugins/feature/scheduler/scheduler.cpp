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

#include "channel/channelwebapiutils.h"
#include "device/deviceset.h"
#include "feature/featureset.h"
#include "feature/featurewebapiutils.h"
#include "maincore.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"
#include "util/messagequeue.h"

#include "scheduler.h"

MESSAGE_CLASS_DEFINITION(Scheduler::MsgConfigureScheduler, Message)

const char* const Scheduler::m_featureIdURI = "sdrangel.feature.scheduler";
const char* const Scheduler::m_featureId = "Scheduler";

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

QStringList Scheduler::eventTypeNames()
{
    return QStringList({
        QStringLiteral("Satellite AOS"),
        QStringLiteral("Satellite LOS"),
        QStringLiteral("ADSB Aircraft Detected"),
        QStringLiteral("ADSB Aircraft Lost"),
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
        QStringLiteral("Packet Received")
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

    for (const SchedulerSettings::DeviceSetAction& action : deviceActions)
    {
        if (action.m_presetGroup.isEmpty()) {
            continue;
        }

        MainCore *mainCore = MainCore::instance();
        const std::vector<DeviceSet*>& deviceSets = mainCore->getDeviceSets();

        if ((action.m_deviceSetIndex < 0) || (action.m_deviceSetIndex >= (int) deviceSets.size()))
        {
            qWarning() << "Scheduler::executeRuleActions: no device set" << action.m_deviceSetIndex;
            continue;
        }

        const DeviceSet *deviceSet = deviceSets[action.m_deviceSetIndex];
        QString presetType;

        if (deviceSet->m_deviceSourceEngine != nullptr) {
            presetType = QStringLiteral("R");
        } else if (deviceSet->m_deviceSinkEngine != nullptr) {
            presetType = QStringLiteral("T");
        } else if (deviceSet->m_deviceMIMOEngine != nullptr) {
            presetType = QStringLiteral("M");
        }

        const Preset *preset = mainCore->getSettings().getPreset(
            action.m_presetGroup,
            action.m_presetFrequency,
            action.m_presetDescription,
            presetType);

        if (preset)
        {
            mainCore->getMainMessageQueue()->push(MainCore::MsgLoadPreset::create(preset, action.m_deviceSetIndex));
        }
        else
        {
            qWarning() << "Scheduler::executeRuleActions: unable to find preset"
                       << action.m_presetGroup << action.m_presetFrequency << action.m_presetDescription;
        }
    }

    QTimer::singleShot(1000, this, [this, rule, context, deviceActions]() {
        executeDeviceActions(deviceActions);
        executeFeatureActions(rule.m_featureActions);
        executeCommand(rule.m_command, rule, context);
        saySpeech(rule.m_speech, rule, context);
    });
}

void Scheduler::executeDeviceActions(const QList<SchedulerSettings::DeviceSetAction>& actions)
{
    for (const SchedulerSettings::DeviceSetAction& action : actions)
    {
        if (action.m_overrideCenterFrequency)
        {
            qDebug() << "Scheduler::executeDeviceActions: set center frequency"
                     << action.m_centerFrequency << "on device set" << action.m_deviceSetIndex;
            ChannelWebAPIUtils::setCenterFrequency(action.m_deviceSetIndex, action.m_centerFrequency);
        }
    }

    for (const SchedulerSettings::DeviceSetAction& action : actions)
    {
        if (action.m_acquisitionAction == SchedulerSettings::ActionStart) {
            ChannelWebAPIUtils::run(action.m_deviceSetIndex);
        } else if (action.m_acquisitionAction == SchedulerSettings::ActionStop) {
            ChannelWebAPIUtils::stop(action.m_deviceSetIndex);
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

void Scheduler::executeFeatureActions(const QList<SchedulerSettings::FeatureAction>& actions)
{
    for (const SchedulerSettings::FeatureAction& action : actions)
    {
        if (action.m_action == SchedulerSettings::ActionStart) {
            FeatureWebAPIUtils::run(action.m_featureSetIndex, action.m_featureIndex);
        } else if (action.m_action == SchedulerSettings::ActionStop) {
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
