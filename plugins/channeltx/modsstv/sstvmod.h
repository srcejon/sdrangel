///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#ifndef PLUGINS_CHANNELTX_MODSSTV_SSTVMOD_H_
#define PLUGINS_CHANNELTX_MODSSTV_SSTVMOD_H_

#include <QRecursiveMutex>
#include <QNetworkRequest>
#include <QImage>

#include "dsp/basebandsamplesource.h"
#include "channel/channelapi.h"
#include "util/message.h"

#include "sstvmodsettings.h"

class QNetworkAccessManager;
class QNetworkReply;
class QThread;
class DeviceAPI;
class SSTVModBaseband;
class ObjectPipe;

class SSTVMod : public BasebandSampleSource, public ChannelAPI
{
public:
    class MsgConfigureSSTVMod : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        const SSTVModSettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }
        static MsgConfigureSSTVMod* create(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force)
        {
            return new MsgConfigureSSTVMod(settingsKeys, settings, force);
        }
    private:
        SSTVModSettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;
        MsgConfigureSSTVMod(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force) :
            Message(), m_settings(settings), m_settingsKeys(settingsKeys), m_force(force)
        {}
    };

    /** Start or stop the SSTV transmission. */
    class MsgStartStop : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        bool getStart() const { return m_start; }
        static MsgStartStop* create(bool start) { return new MsgStartStop(start); }
    private:
        bool m_start;
        explicit MsgStartStop(bool start) : Message(), m_start(start) {}
    };

    /** Load an image from a path or a QImage. */
    class MsgLoadImage : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        const QImage& getImage() const { return m_image; }
        static MsgLoadImage* create(const QImage& image) { return new MsgLoadImage(image); }
    private:
        QImage m_image;
        explicit MsgLoadImage(const QImage& image) : Message(), m_image(image) {}
    };

    /** Sent to the GUI when transmission completes. */
    class MsgReportTransmitComplete : public Message {
        MESSAGE_CLASS_DECLARATION
    public:
        static MsgReportTransmitComplete* create() { return new MsgReportTransmitComplete(); }
    private:
        MsgReportTransmitComplete() : Message() {}
    };

    explicit SSTVMod(DeviceAPI *deviceAPI);
    ~SSTVMod() final;
    void destroy() final { delete this; }
    void setDeviceAPI(DeviceAPI *deviceAPI) final;
    DeviceAPI *getDeviceAPI() final { return m_deviceAPI; }

    void start() final;
    void stop() final;
    void pull(SampleVector::iterator& begin, unsigned int nbSamples) final;
    void pushMessage(Message *msg) final { m_inputMessageQueue.push(msg); }
    QString getSourceName() final { return objectName(); }

    void getIdentifier(QString& id) final { id = objectName(); }
    QString getIdentifier() const final { return objectName(); }
    void getTitle(QString& title) final { title = m_settings.m_title; }
    qint64 getCenterFrequency() const final { return m_settings.m_inputFrequencyOffset; }
    void setCenterFrequency(qint64 frequency) final;

    QByteArray serialize() const final;
    bool deserialize(const QByteArray& data) final;

    int getNbSinkStreams() const final { return 1; }
    int getNbSourceStreams() const final { return 0; }
    int getStreamIndex() const final { return m_settings.m_streamIndex; }

    qint64 getStreamCenterFrequency(int streamIndex, bool sinkElseSource) const final
    {
        (void) streamIndex;
        (void) sinkElseSource;
        return m_settings.m_inputFrequencyOffset;
    }

    int webapiSettingsGet(SWGSDRangel::SWGChannelSettings& response, QString& errorMessage) final;
    int webapiWorkspaceGet(SWGSDRangel::SWGWorkspaceInfo& response, QString& errorMessage) final;
    int webapiSettingsPutPatch(bool force, const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response, QString& errorMessage) final;

    static void webapiFormatChannelSettings(SWGSDRangel::SWGChannelSettings& response, const SSTVModSettings& settings);
    static void webapiUpdateChannelSettings(SSTVModSettings& settings, const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response);

    double getMagSq() const;
    uint32_t getNumberOfDeviceStreams() const;

    static const char* const m_channelIdURI;
    static const char* const m_channelId;

private:
    DeviceAPI*       m_deviceAPI;
    QThread*         m_thread = nullptr;
    bool             m_running = false;
    SSTVModBaseband* m_basebandSource = nullptr;
    SSTVModSettings  m_settings;
    int              m_basebandSampleRate = 0;
    qint64           m_centerFrequency = 0;

    QRecursiveMutex  m_settingsMutex;
    QNetworkAccessManager *m_networkManager;
    QNetworkRequest  m_networkRequest;

    bool handleMessage(const Message& cmd) final;
    void applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force = false);
    void webapiReverseSendSettings(const QList<QString>& channelSettingsKeys, const SSTVModSettings& settings, bool force);
    void sendChannelSettings(const QList<ObjectPipe*>& pipes, const QList<QString>& channelSettingsKeys,
        const SSTVModSettings& settings, bool force);

private slots:
    void networkManagerFinished(QNetworkReply *reply) const;
    void onTransmitComplete();
};

#endif // PLUGINS_CHANNELTX_MODSSTV_SSTVMOD_H_
