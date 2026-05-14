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

#ifndef INCLUDE_METEORBASEBAND_H
#define INCLUDE_METEORBASEBAND_H

#include <QObject>
#include <QRecursiveMutex>

#include "dsp/samplesinkfifo.h"
#include "util/message.h"
#include "util/messagequeue.h"

#include "meteordemodsink.h"

class ChannelAPI;
class DownChannelizer;
class MessageQueue;
class ScopeVis;
class SpectrumVis;

class MeteorBaseband : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureMeteorBaseband : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const MeteorSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureMeteorBaseband* create(const MeteorSettings& settings, const QStringList& settingsKeys, bool force)
        {
            return new MsgConfigureMeteorBaseband(settings, settingsKeys, force);
        }

    private:
        MeteorSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;

        MsgConfigureMeteorBaseband(const MeteorSettings& settings, const QStringList& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        {}
    };

    MeteorBaseband();
    ~MeteorBaseband();
    void reset();
    void startWork();
    void stopWork();
    void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setBasebandSampleRate(int sampleRate);
    int getChannelSampleRate() const;
    void setChannel(ChannelAPI *channel);
    void setScopeSink(ScopeVis* scopeSink);
    void setSpectrumSink(SpectrumVis* spectrumSink) { m_spectrumVis = spectrumSink; m_sink.setSpectrumSink(spectrumSink); }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_sink.setMessageQueueToGUI(messageQueue); }
    bool isRunning() const { return m_running; }
    void setFifoLabel(const QString& label) { m_sampleFifo.setLabel(label); }

private:
    SampleSinkFifo m_sampleFifo;
    DownChannelizer *m_channelizer;
    MeteorDemodSink m_sink;
    MessageQueue m_inputMessageQueue;
    MeteorSettings m_settings;
    SpectrumVis *m_spectrumVis;
    bool m_running;
    QRecursiveMutex m_mutex;

    bool handleMessage(const Message& cmd);
    void applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force = false);
    void notifyVisualSampleRate();

private slots:
    void handleInputMessages();
    void handleData();
};

#endif // INCLUDE_METEORBASEBAND_H
