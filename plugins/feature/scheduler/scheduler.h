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

#ifndef INCLUDE_FEATURE_SCHEDULER_H_
#define INCLUDE_FEATURE_SCHEDULER_H_

#include <QMap>
#include <QTimer>
#include <QByteArray>
#include <functional>

#ifdef QT_TEXTTOSPEECH_FOUND
#include <QTextToSpeech>
#endif

#include "availablechannelorfeaturehandler.h"
#include "feature/feature.h"
#include "maincore.h"
#include "util/message.h"

#include "schedulersettings.h"

class MessageQueue;
class WebAPIAdapterInterface;

namespace SWGSDRangel {
    class SWGDeviceState;
    class SWGFeatureSettings;
}

class Scheduler : public Feature
{
    Q_OBJECT
public:
    class MsgConfigureScheduler : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const SchedulerSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureScheduler* create(const SchedulerSettings& settings, const QStringList& settingsKeys, bool force)
        {
            return new MsgConfigureScheduler(settings, settingsKeys, force);
        }

    private:
        SchedulerSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;

        MsgConfigureScheduler(const SchedulerSettings& settings, const QStringList& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    Scheduler(WebAPIAdapterInterface *webAPIAdapterInterface);
    ~Scheduler() override;

    void destroy() override { delete this; }
    bool handleMessage(const Message& cmd) override;

    void getIdentifier(QString& id) const override { id = objectName(); }
    QString getIdentifier() const override { return objectName(); }
    void getTitle(QString& title) const override { title = m_settings.m_title; }

    QByteArray serialize() const override;
    bool deserialize(const QByteArray& data) override;

    int webapiRun(bool run, SWGSDRangel::SWGDeviceState& response, QString& errorMessage) override;
    int webapiSettingsGet(
            SWGSDRangel::SWGFeatureSettings& response,
            QString& errorMessage) override;
    int webapiSettingsPutPatch(
            bool force,
            const QStringList& featureSettingsKeys,
            SWGSDRangel::SWGFeatureSettings& response,
            QString& errorMessage) override;

    static void webapiFormatFeatureSettings(
            SWGSDRangel::SWGFeatureSettings& response,
            const SchedulerSettings& settings);
    static void webapiUpdateFeatureSettings(
            SchedulerSettings& settings,
            const QStringList& featureSettingsKeys,
            SWGSDRangel::SWGFeatureSettings& response);

    static QStringList eventTypeNames();
    static QString eventTypeName(int eventType);
    static const char* const m_featureIdURI;
    static const char* const m_featureId;

private:
    struct ExecutionContext
    {
        QString m_trigger;
        QDateTime m_dateTime;
        int m_eventType;
        QString m_eventName;
        QString m_source;
        QString m_data;
        QMap<QString, QString> m_dataFields;
    };

    SchedulerSettings m_settings;
    QTimer m_timer;
    QMap<QString, QDateTime> m_nextRuns;
    QMap<QString, int> m_eventMatchCounts;
    AvailableChannelOrFeatureHandler m_eventSourceHandler;
#ifdef QT_TEXTTOSPEECH_FOUND
    QTextToSpeech *m_speech;
#endif

    void applySettings(const SchedulerSettings& settings, const QStringList& settingsKeys, bool force = false);
    void rebuildNextRuns();
    void updateNextRun(const SchedulerSettings::ScheduleRule& rule, const QDateTime& after);
    void processDueRules();
    void handleEventMessageQueue(MessageQueue *messageQueue);
    void handleEvent(const MainCore::MsgEvent& eventMessage);
    bool ruleMatchesEvent(const SchedulerSettings::ScheduleRule& rule, const MainCore::MsgEvent& eventMessage, QString& sourceId) const;
    void executeRuleById(const QString& ruleId, const ExecutionContext& context);
    void executeRule(SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context);
    void executeRuleActions(const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context);
    void executeDurationStopsByRuleId(const QString& ruleId, const QByteArray& ruleState);
    void executeDeviceActions(
        const QList<SchedulerSettings::DeviceSetAction>& actions,
        const QString& ruleId,
        const QByteArray& ruleState,
        std::function<void()> completion);
    void executeFileSinkActions(
        const QList<SchedulerSettings::DeviceSetAction>& actions,
        int actionIndex,
        int waitCount,
        const QString& ruleId,
        const QByteArray& ruleState,
        std::function<void()> completion);
    void executeChannelActions(const QList<SchedulerSettings::ChannelAction>& actions);
    void executeFeatureActions(const QList<SchedulerSettings::FeatureAction>& actions);
    void executeDeviceDurationStops(const QList<SchedulerSettings::DeviceSetAction>& actions);
    void executeChannelDurationStops(const QList<SchedulerSettings::ChannelAction>& actions);
    void executeFeatureDurationStops(const QList<SchedulerSettings::FeatureAction>& actions);
    static QDateTime schedulerDateTimeFromString(const QString *text);
    static QString schedulerDateTimeToString(const QDateTime& dateTime);
    static QByteArray ruleState(const SchedulerSettings::ScheduleRule& rule);
    bool isRuleCurrentAndEnabled(const QString& ruleId, const QByteArray& state) const;
    static bool parseFrequency(const QString& text, double& frequencyInHz);
    static QMap<QString, QString> parseEventDataFields(const QString& data);
    static bool patchDeviceSetting(int deviceSetIndex, const SchedulerSettings::SettingValue& setting);
    static bool patchChannelSetting(int deviceSetIndex, int channelIndex, const SchedulerSettings::SettingValue& setting);
    static bool patchFeatureSetting(int featureSetIndex, int featureIndex, const SchedulerSettings::SettingValue& setting);
    void executeCommand(const QString& command, const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context);
    void saySpeech(const QString& speech, const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context);
    QString substitute(const QString& text, const SchedulerSettings::ScheduleRule& rule, const ExecutionContext& context) const;
    QString sourceIdForObject(const QObject *object) const;
    void notifyGUI(const QStringList& settingsKeys);

private slots:
    void eventMessageEnqueued(MessageQueue *messageQueue);
};

#endif // INCLUDE_FEATURE_SCHEDULER_H_
