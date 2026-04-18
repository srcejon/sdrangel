///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#ifndef PLUGINS_CHANNELTX_MODSSTV_SSTVMODBASEBAND_H_
#define PLUGINS_CHANNELTX_MODSSTV_SSTVMODBASEBAND_H_

#include <QObject>
#include <QRecursiveMutex>
#include <QImage>

#include "dsp/samplesourcefifo.h"
#include "util/message.h"
#include "util/messagequeue.h"

#include "sstvmodsource.h"
#include "sstvmodsettings.h"

class UpChannelizer;
class ChannelAPI;

class SSTVModBaseband : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureSSTVModBaseband : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        const SSTVModSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureSSTVModBaseband* create(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force)
        {
            return new MsgConfigureSSTVModBaseband(settingsKeys, settings, force);
        }
    private:
        SSTVModSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;
        MsgConfigureSSTVModBaseband(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force) :
            Message(), m_settings(settings), m_settingsKeys(settingsKeys), m_force(force)
        {}
    };

    class MsgStartStop : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        bool getStart() const { return m_start; }
        static MsgStartStop* create(bool start) { return new MsgStartStop(start); }
    private:
        bool m_start;
        explicit MsgStartStop(bool start) : Message(), m_start(start) {}
    };

    SSTVModBaseband();
    ~SSTVModBaseband();
    void reset();
    void pull(const SampleVector::iterator& begin, unsigned int nbSamples);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    double getMagSq() const { return m_source.getMagSq(); }
    int getChannelSampleRate() const;
    void setChannel(ChannelAPI *channel);

signals:
    void levelChanged(qreal rmsLevel, qreal peakLevel, int numSamples);
    void transmitComplete();

private:
    SampleSourceFifo m_sampleFifo;
    UpChannelizer *m_channelizer;
    SSTVModSource m_source;
    MessageQueue m_inputMessageQueue;
    SSTVModSettings m_settings;
    QRecursiveMutex m_mutex;

    void processFifo(SampleVector& data, unsigned int iBegin, unsigned int iEnd);
    bool handleMessage(const Message& cmd);
    void applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force = false);

private slots:
    void handleInputMessages();
    void handleData();
    void onTransmitComplete();
};

#endif // PLUGINS_CHANNELTX_MODSSTV_SSTVMODBASEBAND_H_
