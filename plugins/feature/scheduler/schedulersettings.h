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

#ifndef INCLUDE_FEATURE_SCHEDULERSETTINGS_H_
#define INCLUDE_FEATURE_SCHEDULERSETTINGS_H_

#include <QByteArray>
#include <QColor>
#include <QDateTime>
#include <QList>
#include <QString>
#include <QStringList>

class Serializable;

struct SchedulerSettings
{
    enum TriggerType
    {
        TriggerTime = 0,
        TriggerEvent = 1
    };

    enum Recurrence
    {
        RecurrenceOnce = 0,
        RecurrenceDaily,
        RecurrenceWeekly,
        RecurrenceMonthly
    };

    enum RunAction
    {
        ActionNoChange = 0,
        ActionStart,
        ActionStop
    };

    enum DelayUnit
    {
        DelaySeconds = 0,
        DelayMinutes
    };

    struct DeviceSetAction
    {
        int m_deviceSetIndex;
        QString m_deviceSetId;
        QString m_presetGroup;
        quint64 m_presetFrequency;
        QString m_presetDescription;
        RunAction m_acquisitionAction;
        RunAction m_fileSinkAction;
        bool m_overrideCenterFrequency;
        quint64 m_centerFrequency;

        DeviceSetAction();
    };

    struct FeatureAction
    {
        int m_featureSetIndex;
        int m_featureIndex;
        QString m_featureId;
        RunAction m_action;

        FeatureAction();
    };

    struct ScheduleRule
    {
        QString m_id;
        QString m_name;
        bool m_enabled;
        TriggerType m_triggerType;
        QDateTime m_time;
        Recurrence m_recurrence;
        int m_eventType;
        QString m_eventSourceId;
        QString m_eventDataRegex;
        int m_eventDelay;
        DelayUnit m_eventDelayUnit;
        QString m_command;
        QString m_speech;
        QList<DeviceSetAction> m_deviceSetActions;
        QList<FeatureAction> m_featureActions;
        QDateTime m_lastRun;

        ScheduleRule();
    };

    QString m_title;
    int m_workspaceIndex;
    QByteArray m_geometryBytes;
    Serializable *m_rollupState;
    quint32 m_rgbColor;
    QList<ScheduleRule> m_rules;

    SchedulerSettings();
    ~SchedulerSettings() = default;

    void resetToDefaults();
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void applySettings(const QStringList& settingsKeys, const SchedulerSettings& settings);
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }

    static QString newRuleId();
    static QDateTime nextDateTime(const ScheduleRule& rule, const QDateTime& after);
    static int delaySeconds(const ScheduleRule& rule);
};

#endif // INCLUDE_FEATURE_SCHEDULERSETTINGS_H_
