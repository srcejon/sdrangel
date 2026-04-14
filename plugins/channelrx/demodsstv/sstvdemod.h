///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015-2018 Edouard Griffiths, F4EXB.                             //
// Copyright (C) 2021 Jon Beniston, M7RCE                                        //
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

#ifndef INCLUDE_SSTVDEMOD_H
#define INCLUDE_SSTVDEMOD_H

#include <QNetworkRequest>
#include <QThread>
#include <QImage>

#include "dsp/basebandsamplesink.h"
#include "channel/channelapi.h"
#include "util/message.h"

#include "sstvdemodbaseband.h"
#include "sstvdemodsettings.h"

class QNetworkAccessManager;
class QNetworkReply;
class DeviceAPI;

class SSTVDemod : public BasebandSampleSink, public ChannelAPI {
public:
    /** Sent from GUI or API to configure the demodulator */
    class MsgConfigureSSTVDemod : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const SSTVDemodSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureSSTVDemod* create(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force)
        {
            return new MsgConfigureSSTVDemod(settingsKeys, settings, force);
        }

    private:
        SSTVDemodSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;

        MsgConfigureSSTVDemod(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        {}
    };

    /** Sent from sink to GUI when a pair of SSTV scan lines has been decoded */
    class MsgImage : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }
        int getLineIndex() const { return m_lineIndex; }

        static MsgImage* create(const QImage& image, int lineIndex)
        {
            return new MsgImage(image, lineIndex);
        }

    private:
        QImage m_image;
        int m_lineIndex;

        MsgImage(const QImage& image, int lineIndex) :
            Message(),
            m_image(image),
            m_lineIndex(lineIndex)
        {}
    };

    /** Sent from GUI to reset the decoder and clear the image */
    class MsgResetDecoder : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgResetDecoder* create()
        {
            return new MsgResetDecoder();
        }

    private:
        MsgResetDecoder() :
            Message()
        {}
    };

    SSTVDemod(DeviceAPI *deviceAPI);
    virtual ~SSTVDemod();
    virtual void destroy() { delete this; }
    virtual void setDeviceAPI(DeviceAPI *deviceAPI);
    virtual DeviceAPI *getDeviceAPI() { return m_deviceAPI; }

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

    virtual int getNbSinkStreams() const { return 1; }
    virtual int getNbSourceStreams() const { return 0; }
    virtual int getStreamIndex() const { return m_settings.m_streamIndex; }

    virtual qint64 getStreamCenterFrequency(int streamIndex, bool sinkElseSource) const
    {
        (void) streamIndex;
        (void) sinkElseSource;
        return 0;
    }

    virtual int webapiSettingsGet(
            SWGSDRangel::SWGChannelSettings& response,
            QString& errorMessage);

    virtual int webapiWorkspaceGet(
            SWGSDRangel::SWGWorkspaceInfo& response,
            QString& errorMessage);

    virtual int webapiSettingsPutPatch(
            bool force,
            const QStringList& channelSettingsKeys,
            SWGSDRangel::SWGChannelSettings& response,
            QString& errorMessage);

    double getMagSq() const { return m_basebandSink->getMagSq(); }

    void getMagSqLevels(double& avg, double& peak, int& nbSamples) {
        m_basebandSink->getMagSqLevels(avg, peak, nbSamples);
    }

    uint32_t getNumberOfDeviceStreams() const;

    static const char * const m_channelIdURI;
    static const char * const m_channelId;

private:
    DeviceAPI *m_deviceAPI;
    QThread m_thread;
    SSTVDemodBaseband *m_basebandSink;
    SSTVDemodSettings m_settings;
    int m_basebandSampleRate;
    qint64 m_centerFrequency;

    QNetworkAccessManager *m_networkManager;
    QNetworkRequest m_networkRequest;

    virtual bool handleMessage(const Message& cmd);
    void applySettings(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force = false);
    void webapiReverseSendSettings(const QList<QString>& channelSettingsKeys, const SSTVDemodSettings& settings, bool force);

private slots:
    void networkManagerFinished(QNetworkReply *reply);
    void handleIndexInDeviceSetChanged(int index);
};

#endif // INCLUDE_SSTVDEMOD_H
