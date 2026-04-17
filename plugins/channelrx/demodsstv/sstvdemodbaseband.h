///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Edouard Griffiths, F4EXB                                   //
// Copyright (C) 2021-2026 Jon Beniston, M7RCE <jon@beniston.com>                //
// Some code by Copilot / Claude Sonnet                                          //
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

#ifndef INCLUDE_SSTVDEMODBASEBAND_H
#define INCLUDE_SSTVDEMODBASEBAND_H

#include <QObject>
#include <QRecursiveMutex>

#include "dsp/samplesinkfifo.h"
#include "util/message.h"
#include "util/messagequeue.h"

#include "sstvdemodsink.h"

class DownChannelizer;
class SSTVDemod;

class SSTVDemodBaseband : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureSSTVDemodBaseband : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const SSTVDemodSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureSSTVDemodBaseband* create(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force)
        {
            return new MsgConfigureSSTVDemodBaseband(settingsKeys, settings, force);
        }

    private:
        SSTVDemodSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;

        MsgConfigureSSTVDemodBaseband(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        {}
    };

    SSTVDemodBaseband();
    ~SSTVDemodBaseband();
    void reset();
    void startWork();
    void stopWork();
    void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void getMagSqLevels(double& avg, double& peak, int& nbSamples) {
        m_sink.getMagSqLevels(avg, peak, nbSamples);
    }
    void setMessageQueueToChannel(MessageQueue *messageQueue) { m_sink.setMessageQueueToChannel(messageQueue); }
    void setBasebandSampleRate(int sampleRate);
    double getMagSq() const { return m_sink.getMagSq(); }
    bool isRunning() const { return m_running; }
    void setFifoLabel(const QString& label) { m_sampleFifo.setLabel(label); }
    void resetDecoder() { m_sink.resetDecoder(); }

private:
    SampleSinkFifo m_sampleFifo;
    DownChannelizer *m_channelizer;
    SSTVDemodSink m_sink;
    MessageQueue m_inputMessageQueue;
    SSTVDemodSettings m_settings;
    bool m_running;
    QRecursiveMutex m_mutex;

    bool handleMessage(const Message& cmd);
    void applySettings(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force = false);

private slots:
    void handleInputMessages();
    void handleData();
};

#endif // INCLUDE_SSTVDEMODBASEBAND_H
