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
        RecurrenceMonthly
    };

    enum RunAction
    {
        ActionNoChange = 0,
        ActionStart,
        ActionStop,
        ActionCameraSaveImage,
        ActionCameraRecordVideo,
        ActionMapFind,
        ActionFileSinkRecordStart,
        ActionFileSinkRecordStop,
        ActionSigMFRecordStart,
        ActionSigMFRecordStop,
        ActionRTTYTransmit,
        ActionPSK31Transmit,
        ActionPacketTransmit,
        ActionIEEE_802_15_4Transmit,
        ActionAISTransmit,
        ActionFreqScannerRun,
        ActionFreqScannerStop,
        ActionRadioAstronomyStart,
        ActionApplySetting
    };

    enum DelayUnit
    {
        DelaySeconds = 0,
        DelayMinutes
    };

    enum SettingValueType
    {
        SettingString = 0,
        SettingInteger,
        SettingDouble
    };

    struct SettingValue
    {
        QString m_name;
        QString m_value;
        SettingValueType m_type;

        SettingValue();
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
        QString m_centerFrequency;
        QList<SettingValue> m_settings;

        DeviceSetAction();
    };

    struct FeatureAction
    {
        int m_featureSetIndex;
        int m_featureIndex;
        QString m_featureId;
        RunAction m_action;
        QString m_cameraFilename;
        bool m_cameraRecordRawFits;
        bool m_cameraRecordCalibratedMedia;
        bool m_cameraRecordPostProcessedMedia;
        int m_cameraImageCount;
        int m_cameraVideoDuration;
        QString m_findTarget;
        QList<SettingValue> m_settings;

        FeatureAction();
    };

    struct ChannelAction
    {
        int m_deviceSetIndex;
        QString m_deviceSetId;
        int m_channelIndex;
        QString m_channelId;
        RunAction m_action;
        QString m_text;
        QString m_callsign;
        QString m_to;
        QString m_via;
        QString m_data;
        QList<SettingValue> m_settings;

        ChannelAction();
    };

    struct ScheduleRule
    {
        QString m_id;
        QString m_name;
        bool m_enabled;
        TriggerType m_triggerType;
        QDateTime m_time;
        QDate m_dateUntil;
        Recurrence m_recurrence;
        int m_weekdayMask;
        int m_eventType;
        QString m_eventSourceId;
        QString m_eventDataRegex;
        int m_eventDelay;
        DelayUnit m_eventDelayUnit;
        int m_duration;
        DelayUnit m_durationUnit;
        QString m_command;
        QString m_speech;
        QList<DeviceSetAction> m_deviceSetActions;
        QList<ChannelAction> m_channelActions;
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
    static int durationSeconds(const ScheduleRule& rule);

private:
    static constexpr int DefaultWeekdayMask = 0x7f;
    static int weekdayBit(const QDate& date);
    static bool weekdayMaskMatches(int weekdayMask, const QDate& date);
    static bool isAfterDateUntil(const ScheduleRule& rule, const QDateTime& candidate);
    static QDateTime monthlyDateTime(const QDate& baseDate, const QTime& time, int monthOffset);
};

#endif // INCLUDE_FEATURE_SCHEDULERSETTINGS_H_
