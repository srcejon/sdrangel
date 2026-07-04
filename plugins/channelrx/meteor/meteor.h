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

#ifndef INCLUDE_METEOR_H
#define INCLUDE_METEOR_H

#include <QDateTime>
#include <QMap>
#include <QThread>

#include "availablechannelorfeaturehandler.h"
#include "channel/channelapi.h"
#include "dsp/basebandsamplesink.h"
#include "dsp/scopevis.h"
#include "dsp/spectrumvis.h"
#include "maincore.h"
#include "util/message.h"

#include "meteorbaseband.h"
#include "meteorsettings.h"

class DeviceAPI;
class MessageQueue;

namespace SWGSDRangel {
    class SWGChannelSettings;
    class SWGWorkspaceInfo;
}

class Meteor : public BasebandSampleSink, public ChannelAPI {
public:
    class MsgConfigureMeteor : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const MeteorSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureMeteor* create(const MeteorSettings& settings, const QStringList& settingsKeys, bool force)
        {
            return new MsgConfigureMeteor(settings, settingsKeys, force);
        }

    private:
        MeteorSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;

        MsgConfigureMeteor(const MeteorSettings& settings, const QStringList& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        {}
    };

    class MsgCameraMeteorDetected : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QDateTime& getDateTimeUtc() const { return m_dateTimeUtc; }
        double getDurationS() const { return m_durationS; }

        static MsgCameraMeteorDetected* create(const QDateTime& dateTimeUtc, double durationS)
        {
            return new MsgCameraMeteorDetected(dateTimeUtc, durationS);
        }

    private:
        QDateTime m_dateTimeUtc;
        double m_durationS;

        MsgCameraMeteorDetected(const QDateTime& dateTimeUtc, double durationS) :
            Message(),
            m_dateTimeUtc(dateTimeUtc),
            m_durationS(durationS)
        {}
    };

    Meteor(DeviceAPI *deviceAPI);
    virtual ~Meteor();
    virtual void destroy() { delete this; }
    virtual void setDeviceAPI(DeviceAPI *deviceAPI);
    virtual DeviceAPI *getDeviceAPI() { return m_deviceAPI; }
    virtual void setMessageQueueToGUI(MessageQueue *queue);

    SpectrumVis *getSpectrumVis() { return &m_spectrumVis; }
    ScopeVis *getScopeVis() { return &m_scopeVis; }
    int getChannelSampleRate() const { return m_basebandSink->getChannelSampleRate(); }

    using BasebandSampleSink::feed;
    virtual void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end, bool po);
    virtual void start();
    virtual void stop();
    virtual void pushMessage(Message *msg) { m_inputMessageQueue.push(msg); }
    virtual QString getSinkName() { return objectName(); }

    virtual void getIdentifier(QString& id) { id = objectName(); }
    virtual QString getIdentifier() const { return objectName(); }
    virtual const QString& getURI() const { return getName(); }
    virtual void getTitle(QString& title) { title = m_settings.m_title; }
    virtual qint64 getCenterFrequency() const { return m_settings.m_inputFrequencyOffset; }
    virtual void setCenterFrequency(qint64 frequency);

    virtual QByteArray serialize() const;
    virtual bool deserialize(const QByteArray& data);

    virtual int webapiSettingsGet(
            SWGSDRangel::SWGChannelSettings& response,
            QString& errorMessage);

    virtual int webapiSettingsPutPatch(
            bool force,
            const QStringList& channelSettingsKeys,
            SWGSDRangel::SWGChannelSettings& response,
            QString& errorMessage);

    virtual int getNbSinkStreams() const { return 1; }
    virtual int getNbSourceStreams() const { return 0; }
    virtual int getStreamIndex() const { return m_settings.m_streamIndex; }
    uint32_t getNumberOfDeviceStreams() const;

    virtual qint64 getStreamCenterFrequency(int streamIndex, bool sinkElseSource) const
    {
        (void) streamIndex;
        (void) sinkElseSource;
        return m_settings.m_inputFrequencyOffset;
    }

    virtual int webapiWorkspaceGet(
            SWGSDRangel::SWGWorkspaceInfo& response,
            QString& errorMessage);

    static void webapiFormatChannelSettings(
            SWGSDRangel::SWGChannelSettings& response,
            const MeteorSettings& settings);

    static void webapiUpdateChannelSettings(
            MeteorSettings& settings,
            const QStringList& channelSettingsKeys,
            SWGSDRangel::SWGChannelSettings& response);

    static const char * const m_channelIdURI;
    static const char * const m_channelId;

private:
    DeviceAPI *m_deviceAPI;
    QThread m_thread;
    MeteorBaseband* m_basebandSink;
    MeteorSettings m_settings;
    SpectrumVis m_spectrumVis;
    ScopeVis m_scopeVis;
    int m_basebandSampleRate;
    qint64 m_centerFrequency;
    AvailableChannelOrFeatureHandler m_eventSourceHandler;
    QMap<const QObject*, QDateTime> m_cameraMeteorStartTimes;

    virtual bool handleMessage(const Message& cmd);
    void applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force = false);
    void handleEventMessageQueue(MessageQueue *messageQueue);
    void handleEvent(const MainCore::MsgEvent& eventMessage);
    static QMap<QString, QString> parseEventDataFields(const QString& data);
    static bool isMeteorObjectEvent(const MainCore::MsgEvent& eventMessage);

private slots:
    void eventMessageEnqueued(MessageQueue *messageQueue);
    void handleIndexInDeviceSetChanged(int index);
};

#endif // INCLUDE_METEOR_H
