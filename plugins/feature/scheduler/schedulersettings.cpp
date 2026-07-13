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
#include <limits>

#include "settings/serializable.h"
#include "util/astronomy.h"
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

QDate SchedulerSettings::monthlyDate(const QDate& baseDate, int monthOffset)
{
    const QDate monthDate = QDate(baseDate.year(), baseDate.month(), 1).addMonths(monthOffset);
    const int day = qMin(baseDate.day(), monthDate.daysInMonth());
    return QDate(monthDate.year(), monthDate.month(), day);
}

QDateTime SchedulerSettings::scheduledDateTime(
    TriggerType triggerType,
    const QDate& date,
    const QTime& time,
    double latitude,
    double longitude)
{
    if (triggerType == TriggerTime) {
        return QDateTime(date, time);
    }

    QDateTime sunrise;
    QDateTime sunset;
    Astronomy::sunrise(date, latitude, longitude, sunrise, sunset);
    const QDateTime result = triggerType == TriggerSunrise ? sunrise : sunset;

    // The astronomy calculation can produce NaN at latitudes with no rise/set.
    if (!result.isValid() || (qAbs(date.daysTo(result.toLocalTime().date())) > 1)) {
        return QDateTime();
    }

    return result;
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

namespace {

QByteArray serializeRuleV2(const SchedulerSettings::ScheduleRule& rule)
{
    QByteArray blob;
    QDataStream out(&blob, QIODevice::WriteOnly);

    out << static_cast<qint32>(1);
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
    out << rule.m_eventCount;
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

    return blob;
}

bool deserializeRuleV2(const QByteArray& blob, SchedulerSettings::ScheduleRule& rule)
{
    QDataStream in(blob);
    qint32 version;
    qint32 triggerType;
    qint32 recurrence;
    qint32 delayUnit;
    qint32 durationUnit;

    in >> version;
    if (version != 1) {
        return false;
    }

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
    in >> rule.m_eventCount;
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
    rule.m_eventCount = qMax(1, rule.m_eventCount);

    if (rule.m_id.isEmpty()) {
        rule.m_id = SchedulerSettings::newRuleId();
    }

    return in.status() == QDataStream::Ok;
}

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
    m_eventCount(1),
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
    SimpleSerializer s(2);
    QByteArray rulesBlob;
    QDataStream rulesStream(&rulesBlob, QIODevice::WriteOnly);

    rulesStream << static_cast<qint32>(m_rules.size());
    for (const ScheduleRule& rule : m_rules) {
        rulesStream << serializeRuleV2(rule);
    }

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

        for (ScheduleRule& rule : m_rules) {
            rule.m_eventCount = qMax(1, rule.m_eventCount);
        }

        return true;
    }

    if (d.getVersion() == 2)
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
            qint32 ruleCount = 0;

            rulesStream >> ruleCount;
            for (qint32 i = 0; i < ruleCount; ++i)
            {
                QByteArray ruleBlob;
                ScheduleRule rule;

                rulesStream >> ruleBlob;
                if (deserializeRuleV2(ruleBlob, rule)) {
                    m_rules.append(rule);
                }
            }
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

bool SchedulerSettings::isTimeTrigger(TriggerType triggerType)
{
    return (triggerType == TriggerTime) ||
        (triggerType == TriggerSunrise) ||
        (triggerType == TriggerSunset);
}

QDateTime SchedulerSettings::nextDateTime(
    const ScheduleRule& rule,
    const QDateTime& after,
    double latitude,
    double longitude)
{
    if (!rule.m_time.isValid() || !isTimeTrigger(rule.m_triggerType)) {
        return QDateTime();
    }

    if (rule.m_recurrence == RecurrenceOnce)
    {
        const QDateTime candidate = scheduledDateTime(
            rule.m_triggerType,
            rule.m_time.date(),
            rule.m_time.time(),
            latitude,
            longitude);
        return (candidate > after) && !isAfterDateUntil(rule, candidate) ? candidate : QDateTime();
    }

    if (rule.m_recurrence == RecurrenceDaily)
    {
        if (rule.m_weekdayMask == 0) {
            return QDateTime();
        }

        QDate candidateDate = after.date() < rule.m_time.date() ? rule.m_time.date() : after.date();

        for (int dayOffset = 0; dayOffset < 370; ++dayOffset)
        {
            if (rule.m_dateUntil.isValid() && (candidateDate > rule.m_dateUntil)) {
                return QDateTime();
            }

            if (weekdayMaskMatches(rule.m_weekdayMask, candidateDate))
            {
                const QDateTime candidate = scheduledDateTime(
                    rule.m_triggerType,
                    candidateDate,
                    rule.m_time.time(),
                    latitude,
                    longitude);
                if (candidate.isValid() && (candidate > after)) {
                    return candidate;
                }
            }

            candidateDate = candidateDate.addDays(1);
        }

        return QDateTime();
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
            const QDate candidateDate = monthlyDate(baseDate, offset);
            const QDateTime candidate = scheduledDateTime(
                rule.m_triggerType,
                candidateDate,
                rule.m_time.time(),
                latitude,
                longitude);
            if (isAfterDateUntil(rule, candidate)) {
                return QDateTime();
            }
            if (candidate.isValid() && (candidate > after)) {
                return candidate;
            }
        }
    }

    return QDateTime();
}

QDateTime SchedulerSettings::nextSunset(const QDateTime& after, double latitude, double longitude)
{
    QDate date = after.date();

    for (int dayOffset = 0; dayOffset < 370; ++dayOffset)
    {
        const QDateTime sunset = scheduledDateTime(
            TriggerSunset,
            date,
            QTime(),
            latitude,
            longitude);
        if (sunset.isValid() && (sunset > after)) {
            return sunset;
        }

        date = date.addDays(1);
    }

    return QDateTime();
}

int SchedulerSettings::delaySeconds(const ScheduleRule& rule)
{
    qint64 delay = qMax<qint64>(0, rule.m_eventDelay);

    if (rule.m_eventDelayUnit == DelayMinutes) {
        delay *= 60;
    } else if (rule.m_eventDelayUnit == DelayHours) {
        delay *= 60 * 60;
    }

    return static_cast<int>(qMin(delay, static_cast<qint64>(std::numeric_limits<int>::max())));
}

int SchedulerSettings::durationSeconds(const ScheduleRule& rule)
{
    if (rule.m_durationUnit == DelayUntilSunset) {
        return 0;
    }

    qint64 duration = qMax<qint64>(0, rule.m_duration);

    if (rule.m_durationUnit == DelayMinutes) {
        duration *= 60;
    } else if (rule.m_durationUnit == DelayHours) {
        duration *= 60 * 60;
    }

    return static_cast<int>(qMin(duration, static_cast<qint64>(std::numeric_limits<int>::max())));
}
