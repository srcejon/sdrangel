///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020 Jon Beniston, M7RCE                                        //
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

#ifndef INCLUDE_ADSBDEMODSINKWORKER_H
#define INCLUDE_ADSBDEMODSINKWORKER_H

#include <QObject>
#include <QThread>

#include "dsp/dsptypes.h"
#include "util/crc.h"
#include "util/message.h"
#include "util/messagequeue.h"
#include "adsbdemodstats.h"
#include "adsbdemodsettings.h"

class ADSBDemodSink;

class ADSBDemodSinkWorker : public QThread {
    Q_OBJECT
public:

    class MsgConfigureADSBDemodSinkWorker : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const ADSBDemodSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureADSBDemodSinkWorker* create(const ADSBDemodSettings& settings, const QStringList& settingsKeys, bool force)
        {
            return new MsgConfigureADSBDemodSinkWorker(settings, settingsKeys, force);
        }

    private:
        ADSBDemodSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;

        MsgConfigureADSBDemodSinkWorker(const ADSBDemodSettings& settings, const QStringList& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    ADSBDemodSinkWorker(ADSBDemodSink *sink) :
        m_sink(sink),
        m_demodStats(),
        m_correlationThresholdLinear(0.02f),
        m_crc()
    {
    }
    void run() override;
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }

private:
    void handleInputMessages();
    MessageQueue m_inputMessageQueue;
    ADSBDemodSettings m_settings;
    ADSBDemodSink *m_sink;
    ADSBDemodStats m_demodStats;
    Real m_correlationThresholdLinear;
    static const Real m_correlationScale;
    crcadsb m_crc;                      //!< Have as member to avoid recomputing LUT
    QHash<unsigned, qint64> m_icaos;         //!< ICAO addresses that have been received and msecsSinceEpoch last heard
    QDateTime m_lastClear;

    struct RXRecord {
        QByteArray m_data;
        int m_firstIndex;
        unsigned short m_preamble;
        Real m_preambleCorrelation;
        Real m_correlationOnes;
        QDateTime m_dateTime;
        unsigned m_crc;
    };

    QHash<int, QList<RXRecord>> m_modeSOnlyIcaos;

    QDateTime rxDateTime(int firstIdx, int readBuffer) const;
    void handleModeS(unsigned char *data, int bytes, unsigned icao, int df, int firstIndex, unsigned short preamble, Real preambleCorrelation, Real correlationOnes, const QDateTime& dateTime, unsigned crc);
    void forwardFrame(const unsigned char *data, int size, float preambleCorrelation, float correlationOnes, const QDateTime& dateTime, unsigned crc);
    bool icaoHeardRecently(unsigned icao, const QDateTime &dateTime);
    static bool icaoValid(unsigned icao);
    static bool modeSValid(unsigned char *data, unsigned df);

};

#endif // INCLUDE_ADSBDEMODSINKWORKER_H
