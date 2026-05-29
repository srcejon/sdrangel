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

int SchedulerSettings::weekdayBit(const QDate& date)
{
    return 1 << (date.dayOfWeek() - 1);
}

bool SchedulerSettings::weekdayMaskMatches(int weekdayMask, const QDate& date)
{
    return (weekdayMask & weekdayBit(date)) != 0;
}

bool SchedulerSettings::isAfterDateUntil(const SchedulerSettings::ScheduleRule& rule, const QDateTime& candidate)
{
    return rule.m_dateUntil.isValid() && candidate.isValid() && (candidate.date() > rule.m_dateUntil);
}

QDateTime SchedulerSettings::monthlyDateTime(const QDate& baseDate, const QTime& time, int monthOffset)
{
    const QDate monthDate = QDate(baseDate.year(), baseDate.month(), 1).addMonths(monthOffset);
    const int day = qMin(baseDate.day(), monthDate.daysInMonth());
    return QDateTime(QDate(monthDate.year(), monthDate.month(), day), time);
}

QDataStream& operator<<(QDataStream& out, const SchedulerSettings::SettingValue& setting)
{
    out << setting.m_name;
    out << setting.m_value;
    out << static_cast<qint32>(setting.m_type);
    return out;
}

QDataStream& operator>>(QDataStream& in, SchedulerSettings::SettingValue& setting)
{
    qint32 type;

    in >> setting.m_name;
    in >> setting.m_value;
    in >> type;

    setting.m_type = static_cast<SchedulerSettings::SettingValueType>(type);
    return in;
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
    out << action.m_centerFrequency;
    out << action.m_settings;
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
    in >> action.m_centerFrequency;
    in >> action.m_settings;

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
    out << action.m_cameraFilename;
    out << action.m_cameraRecordRawFits;
    out << action.m_cameraRecordCalibratedMedia;
    out << action.m_cameraRecordPostProcessedMedia;
    out << action.m_cameraImageCount;
    out << action.m_cameraVideoDuration;
    out << action.m_findTarget;
    out << action.m_settings;
    return out;
}

QDataStream& operator>>(QDataStream& in, SchedulerSettings::FeatureAction& action)
{
    qint32 runAction;

    in >> action.m_featureSetIndex;
    in >> action.m_featureIndex;
    in >> action.m_featureId;
    in >> runAction;
    in >> action.m_cameraFilename;
    in >> action.m_cameraRecordRawFits;
    in >> action.m_cameraRecordCalibratedMedia;
    in >> action.m_cameraRecordPostProcessedMedia;
    in >> action.m_cameraImageCount;
    in >> action.m_cameraVideoDuration;
    in >> action.m_findTarget;
    in >> action.m_settings;

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
    out << action.m_settings;
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
    in >> action.m_settings;

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
    out << rule.m_dateUntil;
    out << static_cast<qint32>(rule.m_recurrence);
    out << rule.m_weekdayMask;
    out << rule.m_eventType;
    out << rule.m_eventSourceId;
    out << rule.m_eventDataRegex;
    out << rule.m_eventDelay;
    out << static_cast<qint32>(rule.m_eventDelayUnit);
    out << rule.m_duration;
    out << static_cast<qint32>(rule.m_durationUnit);
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
    qint32 durationUnit;

    in >> rule.m_id;
    in >> rule.m_name;
    in >> rule.m_enabled;
    in >> triggerType;
    in >> rule.m_time;
    in >> rule.m_dateUntil;
    in >> recurrence;
    in >> rule.m_weekdayMask;
    in >> rule.m_eventType;
    in >> rule.m_eventSourceId;
    in >> rule.m_eventDataRegex;
    in >> rule.m_eventDelay;
    in >> delayUnit;
    in >> rule.m_duration;
    in >> durationUnit;
    in >> rule.m_command;
    in >> rule.m_speech;
    in >> rule.m_deviceSetActions;
    in >> rule.m_channelActions;
    in >> rule.m_featureActions;
    in >> rule.m_lastRun;

    rule.m_triggerType = static_cast<SchedulerSettings::TriggerType>(triggerType);
    rule.m_recurrence = static_cast<SchedulerSettings::Recurrence>(recurrence);
    rule.m_eventDelayUnit = static_cast<SchedulerSettings::DelayUnit>(delayUnit);
    rule.m_durationUnit = static_cast<SchedulerSettings::DelayUnit>(durationUnit);

    if (rule.m_id.isEmpty()) {
        rule.m_id = SchedulerSettings::newRuleId();
    }

    return in;
}

SchedulerSettings::SettingValue::SettingValue() :
    m_type(SettingString)
{
}

SchedulerSettings::DeviceSetAction::DeviceSetAction() :
    m_deviceSetIndex(0),
    m_presetFrequency(0),
    m_acquisitionAction(ActionNoChange),
    m_fileSinkAction(ActionNoChange)
{
}

SchedulerSettings::FeatureAction::FeatureAction() :
    m_featureSetIndex(0),
    m_featureIndex(0),
    m_action(ActionStart),
    m_cameraRecordRawFits(false),
    m_cameraRecordCalibratedMedia(true),
    m_cameraRecordPostProcessedMedia(false),
    m_cameraImageCount(1),
    m_cameraVideoDuration(0)
{
}

SchedulerSettings::ChannelAction::ChannelAction() :
    m_deviceSetIndex(0),
    m_channelIndex(0),
    m_action(ActionNoChange)
{
}

SchedulerSettings::ScheduleRule::ScheduleRule() :
    m_id(SchedulerSettings::newRuleId()),
    m_name("New rule"),
    m_enabled(true),
    m_triggerType(TriggerTime),
    m_time(QDateTime::currentDateTime().addSecs(60)),
    m_dateUntil(),
    m_recurrence(RecurrenceOnce),
    m_weekdayMask(DefaultWeekdayMask),
    m_eventType(0),
    m_eventDelay(0),
    m_eventDelayUnit(DelaySeconds),
    m_duration(0),
    m_durationUnit(DelaySeconds)
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
    QDataStream rulesStream(&rulesBlob, QIODevice::WriteOnly);

    rulesStream << m_rules;

    s.writeString(1, m_title);
    s.writeS32(2, m_workspaceIndex);
    s.writeBlob(3, m_geometryBytes);
    if (m_rollupState) {
        s.writeBlob(4, m_rollupState->serialize());
    }
    s.writeU32(5, m_rgbColor);
    s.writeBlob(6, rulesBlob);

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
            rulesStream >> m_rules;
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
        return (rule.m_time > after) && !isAfterDateUntil(rule, rule.m_time) ? rule.m_time : QDateTime();
    }

    if (rule.m_recurrence == RecurrenceDaily)
    {
        if (rule.m_weekdayMask == 0) {
            return QDateTime();
        }

        qint64 days = rule.m_time.daysTo(after);
        if (days < 0) {
            days = 0;
        }
        QDateTime candidate = rule.m_time.addDays(days);
        while ((candidate <= after) || !weekdayMaskMatches(rule.m_weekdayMask, candidate.date())) {
            candidate = candidate.addDays(1);
            if (isAfterDateUntil(rule, candidate)) {
                return QDateTime();
            }
        }
        return isAfterDateUntil(rule, candidate) ? QDateTime() : candidate;
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
            if (isAfterDateUntil(rule, candidate)) {
                return QDateTime();
            }
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

int SchedulerSettings::durationSeconds(const ScheduleRule& rule)
{
    const int duration = qMax(0, rule.m_duration);
    return rule.m_durationUnit == DelayMinutes ? duration * 60 : duration;
}
