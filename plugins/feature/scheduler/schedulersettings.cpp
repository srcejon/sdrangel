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

#include <QDataStream>
#include <QIODevice>
#include <QUuid>

#include "settings/serializable.h"
#include "util/simpleserializer.h"

#include "schedulersettings.h"

namespace
{
void writeFeatureActionExtras(QDataStream& out, const QList<SchedulerSettings::ScheduleRule>& rules)
{
    out << quint32(1);
    out << rules.size();

    for (const SchedulerSettings::ScheduleRule& rule : rules)
    {
        out << rule.m_id;
        out << rule.m_featureActions.size();

        for (const SchedulerSettings::FeatureAction& action : rule.m_featureActions)
        {
            out << action.m_cameraFilename;
            out << action.m_cameraRecordMode;
            out << action.m_cameraImageCount;
            out << action.m_cameraVideoDuration;
            out << action.m_findTarget;
        }
    }
}

void readFeatureActionExtras(QDataStream& in, QList<SchedulerSettings::ScheduleRule>& rules)
{
    quint32 version = 0;
    int ruleCount = 0;

    in >> version;
    if (version != 1) {
        return;
    }

    in >> ruleCount;

    for (int ruleIndex = 0; ruleIndex < ruleCount; ++ruleIndex)
    {
        QString ruleId;
        int actionCount = 0;

        in >> ruleId;
        in >> actionCount;

        SchedulerSettings::ScheduleRule *rule = nullptr;
        for (SchedulerSettings::ScheduleRule& candidate : rules)
        {
            if (candidate.m_id == ruleId)
            {
                rule = &candidate;
                break;
            }
        }

        for (int actionIndex = 0; actionIndex < actionCount; ++actionIndex)
        {
            QString cameraFilename;
            int cameraRecordMode = 0;
            int cameraImageCount = 1;
            int cameraVideoDuration = 0;
            QString findTarget;

            in >> cameraFilename;
            in >> cameraRecordMode;
            in >> cameraImageCount;
            in >> cameraVideoDuration;
            in >> findTarget;

            if (rule && (actionIndex >= 0) && (actionIndex < rule->m_featureActions.size()))
            {
                SchedulerSettings::FeatureAction& action = rule->m_featureActions[actionIndex];
                action.m_cameraFilename = cameraFilename;
                action.m_cameraRecordMode = cameraRecordMode;
                action.m_cameraImageCount = cameraImageCount;
                action.m_cameraVideoDuration = cameraVideoDuration;
                action.m_findTarget = findTarget;
            }
        }
    }
}

QDateTime monthlyDateTime(const QDate& baseDate, const QTime& time, int monthOffset)
{
    const QDate monthDate = QDate(baseDate.year(), baseDate.month(), 1).addMonths(monthOffset);
    const int day = qMin(baseDate.day(), monthDate.daysInMonth());
    return QDateTime(QDate(monthDate.year(), monthDate.month(), day), time);
}
}

QDataStream& operator<<(QDataStream& out, const SchedulerSettings::DeviceSetAction& action)
{
    out << action.m_deviceSetIndex;
    out << action.m_deviceSetId;
    out << action.m_presetGroup;
    out << action.m_presetFrequency;
    out << action.m_presetDescription;
    out << static_cast<qint32>(action.m_acquisitionAction);
    out << static_cast<qint32>(action.m_fileSinkAction);
    out << action.m_overrideCenterFrequency;
    out << action.m_centerFrequency;
    return out;
}

QDataStream& operator>>(QDataStream& in, SchedulerSettings::DeviceSetAction& action)
{
    qint32 acquisitionAction;
    qint32 fileSinkAction;

    in >> action.m_deviceSetIndex;
    in >> action.m_deviceSetId;
    in >> action.m_presetGroup;
    in >> action.m_presetFrequency;
    in >> action.m_presetDescription;
    in >> acquisitionAction;
    in >> fileSinkAction;
    in >> action.m_overrideCenterFrequency;
    in >> action.m_centerFrequency;

    action.m_acquisitionAction = static_cast<SchedulerSettings::RunAction>(acquisitionAction);
    action.m_fileSinkAction = static_cast<SchedulerSettings::RunAction>(fileSinkAction);
    return in;
}

QDataStream& operator<<(QDataStream& out, const SchedulerSettings::FeatureAction& action)
{
    out << action.m_featureSetIndex;
    out << action.m_featureIndex;
    out << action.m_featureId;
    out << static_cast<qint32>(action.m_action);
    return out;
}

QDataStream& operator>>(QDataStream& in, SchedulerSettings::FeatureAction& action)
{
    qint32 runAction;

    in >> action.m_featureSetIndex;
    in >> action.m_featureIndex;
    in >> action.m_featureId;
    in >> runAction;

    action.m_action = static_cast<SchedulerSettings::RunAction>(runAction);
    return in;
}

QDataStream& operator<<(QDataStream& out, const SchedulerSettings::ChannelAction& action)
{
    out << action.m_deviceSetIndex;
    out << action.m_deviceSetId;
    out << action.m_channelIndex;
    out << action.m_channelId;
    out << static_cast<qint32>(action.m_action);
    out << action.m_text;
    out << action.m_callsign;
    out << action.m_to;
    out << action.m_via;
    out << action.m_data;
    return out;
}

QDataStream& operator>>(QDataStream& in, SchedulerSettings::ChannelAction& action)
{
    qint32 runAction;

    in >> action.m_deviceSetIndex;
    in >> action.m_deviceSetId;
    in >> action.m_channelIndex;
    in >> action.m_channelId;
    in >> runAction;
    in >> action.m_text;
    in >> action.m_callsign;
    in >> action.m_to;
    in >> action.m_via;
    in >> action.m_data;

    action.m_action = static_cast<SchedulerSettings::RunAction>(runAction);
    return in;
}

QDataStream& operator<<(QDataStream& out, const SchedulerSettings::ScheduleRule& rule)
{
    out << rule.m_id;
    out << rule.m_name;
    out << rule.m_enabled;
    out << static_cast<qint32>(rule.m_triggerType);
    out << rule.m_time;
    out << static_cast<qint32>(rule.m_recurrence);
    out << rule.m_eventType;
    out << rule.m_eventSourceId;
    out << rule.m_eventDataRegex;
    out << rule.m_eventDelay;
    out << static_cast<qint32>(rule.m_eventDelayUnit);
    out << rule.m_command;
    out << rule.m_speech;
    out << rule.m_deviceSetActions;
    out << rule.m_channelActions;
    out << rule.m_featureActions;
    out << rule.m_lastRun;
    return out;
}

QDataStream& operator>>(QDataStream& in, SchedulerSettings::ScheduleRule& rule)
{
    qint32 triggerType;
    qint32 recurrence;
    qint32 delayUnit;

    in >> rule.m_id;
    in >> rule.m_name;
    in >> rule.m_enabled;
    in >> triggerType;
    in >> rule.m_time;
    in >> recurrence;
    in >> rule.m_eventType;
    in >> rule.m_eventSourceId;
    in >> rule.m_eventDataRegex;
    in >> rule.m_eventDelay;
    in >> delayUnit;
    in >> rule.m_command;
    in >> rule.m_speech;
    in >> rule.m_deviceSetActions;
    in >> rule.m_channelActions;
    in >> rule.m_featureActions;
    in >> rule.m_lastRun;

    rule.m_triggerType = static_cast<SchedulerSettings::TriggerType>(triggerType);
    rule.m_recurrence = static_cast<SchedulerSettings::Recurrence>(recurrence);
    rule.m_eventDelayUnit = static_cast<SchedulerSettings::DelayUnit>(delayUnit);

    if (rule.m_id.isEmpty()) {
        rule.m_id = SchedulerSettings::newRuleId();
    }

    return in;
}

SchedulerSettings::DeviceSetAction::DeviceSetAction() :
    m_deviceSetIndex(0),
    m_presetFrequency(0),
    m_acquisitionAction(ActionNoChange),
    m_fileSinkAction(ActionNoChange),
    m_overrideCenterFrequency(false),
    m_centerFrequency(0)
{
}

SchedulerSettings::FeatureAction::FeatureAction() :
    m_featureSetIndex(0),
    m_featureIndex(0),
    m_action(ActionStart),
    m_cameraRecordMode(0),
    m_cameraImageCount(1),
    m_cameraVideoDuration(0)
{
}

SchedulerSettings::ChannelAction::ChannelAction() :
    m_deviceSetIndex(0),
    m_channelIndex(0),
    m_action(ActionFileSinkRecordStart)
{
}

SchedulerSettings::ScheduleRule::ScheduleRule() :
    m_id(SchedulerSettings::newRuleId()),
    m_name("New rule"),
    m_enabled(true),
    m_triggerType(TriggerTime),
    m_time(QDateTime::currentDateTime().addSecs(60)),
    m_recurrence(RecurrenceOnce),
    m_eventType(0),
    m_eventDelay(0),
    m_eventDelayUnit(DelaySeconds)
{
}

SchedulerSettings::SchedulerSettings() :
    m_rollupState(nullptr)
{
    resetToDefaults();
}

void SchedulerSettings::resetToDefaults()
{
    m_title = "Scheduler";
    m_workspaceIndex = -1;
    m_geometryBytes.clear();
    m_rgbColor = QColor(229, 156, 64).rgb();
    m_rules.clear();
}

QByteArray SchedulerSettings::serialize() const
{
    SimpleSerializer s(1);
    QByteArray rulesBlob;
    QByteArray featureActionExtrasBlob;
    QDataStream rulesStream(&rulesBlob, QIODevice::WriteOnly);
    QDataStream featureActionExtrasStream(&featureActionExtrasBlob, QIODevice::WriteOnly);

    rulesStream << quint32(2);
    rulesStream << m_rules;
    writeFeatureActionExtras(featureActionExtrasStream, m_rules);

    s.writeString(1, m_title);
    s.writeS32(2, m_workspaceIndex);
    s.writeBlob(3, m_geometryBytes);
    if (m_rollupState) {
        s.writeBlob(4, m_rollupState->serialize());
    }
    s.writeU32(5, m_rgbColor);
    s.writeBlob(6, rulesBlob);
    s.writeBlob(7, featureActionExtrasBlob);

    return s.final();
}

bool SchedulerSettings::deserialize(const QByteArray& data)
{
    SimpleDeserializer d(data);
    QByteArray blob;
    uint32_t rgb;

    if (!d.isValid())
    {
        resetToDefaults();
        return false;
    }

    if (d.getVersion() == 1)
    {
        d.readString(1, &m_title, "Scheduler");
        d.readS32(2, &m_workspaceIndex, -1);
        d.readBlob(3, &m_geometryBytes);
        if (m_rollupState)
        {
            d.readBlob(4, &blob);
            m_rollupState->deserialize(blob);
        }
        d.readU32(5, &rgb, QColor(229, 156, 64).rgb());
        m_rgbColor = rgb;

        d.readBlob(6, &blob);
        m_rules.clear();

        if (!blob.isEmpty())
        {
            QDataStream rulesStream(blob);
            quint32 rulesVersion = 0;
            rulesStream >> rulesVersion;

            if (rulesVersion == 2) {
                rulesStream >> m_rules;
            }
        }

        d.readBlob(7, &blob);
        if (!blob.isEmpty())
        {
            QDataStream featureActionExtrasStream(blob);
            readFeatureActionExtras(featureActionExtrasStream, m_rules);
        }

        return true;
    }

    resetToDefaults();
    return false;
}

void SchedulerSettings::applySettings(const QStringList& settingsKeys, const SchedulerSettings& settings)
{
    if (settingsKeys.contains("title")) {
        m_title = settings.m_title;
    }
    if (settingsKeys.contains("workspaceIndex")) {
        m_workspaceIndex = settings.m_workspaceIndex;
    }
    if (settingsKeys.contains("geometryBytes")) {
        m_geometryBytes = settings.m_geometryBytes;
    }
    if (settingsKeys.contains("rgbColor")) {
        m_rgbColor = settings.m_rgbColor;
    }
    if (settingsKeys.contains("rules")) {
        m_rules = settings.m_rules;
    }
}

QString SchedulerSettings::newRuleId()
{
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

QDateTime SchedulerSettings::nextDateTime(const ScheduleRule& rule, const QDateTime& after)
{
    if (!rule.m_time.isValid()) {
        return QDateTime();
    }

    if (rule.m_recurrence == RecurrenceOnce) {
        return rule.m_time > after ? rule.m_time : QDateTime();
    }

    if (rule.m_recurrence == RecurrenceDaily)
    {
        qint64 days = rule.m_time.daysTo(after);
        if (days < 0) {
            days = 0;
        }
        QDateTime candidate = rule.m_time.addDays(days);
        while (candidate <= after) {
            candidate = candidate.addDays(1);
        }
        return candidate;
    }

    if (rule.m_recurrence == RecurrenceWeekly)
    {
        qint64 days = rule.m_time.daysTo(after);
        if (days < 0) {
            days = 0;
        }
        QDateTime candidate = rule.m_time.addDays((days / 7) * 7);
        while (candidate <= after) {
            candidate = candidate.addDays(7);
        }
        return candidate;
    }

    if (rule.m_recurrence == RecurrenceMonthly)
    {
        const QDate baseDate = rule.m_time.date();
        const QDate afterDate = after.date();
        int months = (afterDate.year() - baseDate.year()) * 12 + (afterDate.month() - baseDate.month());
        if (months < 0) {
            months = 0;
        }

        for (int offset = qMax(0, months - 1); offset < months + 24; ++offset)
        {
            QDateTime candidate = monthlyDateTime(baseDate, rule.m_time.time(), offset);
            if (candidate > after) {
                return candidate;
            }
        }
    }

    return QDateTime();
}

int SchedulerSettings::delaySeconds(const ScheduleRule& rule)
{
    const int delay = qMax(0, rule.m_eventDelay);
    return rule.m_eventDelayUnit == DelayMinutes ? delay * 60 : delay;
}
