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

#include "acarsdemod.h"

#include <QTime>
#include <QDebug>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QBuffer>
#include <QThread>

#include <stdio.h>
#include <complex.h>

#include "SWGChannelSettings.h"
//#include "SWGACARSDemodSettings.h"
#include "SWGChannelReport.h"
#include "SWGMapItem.h"

#include "dsp/dspengine.h"
#include "dsp/dspcommands.h"
#include "device/deviceapi.h"
#include "feature/feature.h"
#include "util/db.h"
#include "maincore.h"

#include "acarsdemodworker.h"

MESSAGE_CLASS_DEFINITION(AcarsDemod::MsgConfigureAcarsDemod, Message)
MESSAGE_CLASS_DEFINITION(AcarsDemod::MsgVdl2Frame, Message)
MESSAGE_CLASS_DEFINITION(AcarsDemod::MsgHfdlFrame, Message)
MESSAGE_CLASS_DEFINITION(AcarsDemod::MsgAeroFrame, Message)
MESSAGE_CLASS_DEFINITION(AcarsDemod::MsgProtocolChange, Message)

const char * const AcarsDemod::m_channelIdURI = "sdrangel.channel.acarsdemod";
const char * const AcarsDemod::m_channelId = "ACARSDemod";

AcarsDemod::AcarsDemod(DeviceAPI *deviceAPI) :
        ChannelAPI(m_channelIdURI, ChannelAPI::StreamSingleSink),
        m_deviceAPI(deviceAPI),
        m_basebandSampleRate(0)
{
    setObjectName(m_channelId);

    m_basebandSink = new AcarsDemodBaseband(this);
    m_basebandSink->setFifoLabel(QString("%1 [%2:%3]")
        .arg(m_channelId)
        .arg(m_deviceAPI->getDeviceSetIndex())
        .arg(getIndexInDeviceSet())
    );
    m_basebandSink->setChannel(this);
    m_basebandSink->moveToThread(&m_thread);

    // Decoded frames are processed by the worker on its own thread, for the
    // channel's lifetime, in both GUI and server builds
    m_workerThread = new QThread();
    m_worker = new AcarsDemodWorker(this);
    m_worker->moveToThread(m_workerThread);
    QObject::connect(m_workerThread, &QThread::started, m_worker, &AcarsDemodWorker::startWork);
    QObject::connect(m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_workerThread, &QThread::finished, m_workerThread, &QThread::deleteLater);
    m_workerThread->start();

    // The sink sends its frames straight to the worker
    m_basebandSink->setMessageQueueToChannel(m_worker->getInputMessageQueue());

    applySettings(m_settings, true);

    m_deviceAPI->addChannelSink(this);
    m_deviceAPI->addChannelSinkAPI(this);

    m_networkManager = new QNetworkAccessManager();
    QObject::connect(
        m_networkManager,
        &QNetworkAccessManager::finished,
        this,
        &AcarsDemod::networkManagerFinished
    );
}

AcarsDemod::~AcarsDemod()
{
    qDebug("AcarsDemod::~AcarsDemod");
    QObject::disconnect(
        m_networkManager,
        &QNetworkAccessManager::finished,
        this,
        &AcarsDemod::networkManagerFinished
    );
    delete m_networkManager;
    m_deviceAPI->removeChannelSinkAPI(this);
    m_deviceAPI->removeChannelSink(this, true);

    if (m_basebandSink->isRunning()) {
        stop();
    }

    delete m_basebandSink;

    // The sink is gone, so nothing pushes to the worker any more
    m_workerThread->quit();
    m_workerThread->wait();
    m_worker = nullptr;
    m_workerThread = nullptr;
}

void AcarsDemod::setDeviceAPI(DeviceAPI *deviceAPI)
{
    if (deviceAPI != m_deviceAPI)
    {
        m_deviceAPI->removeChannelSinkAPI(this);
        m_deviceAPI->removeChannelSink(this, false);
        m_deviceAPI = deviceAPI;
        m_deviceAPI->addChannelSink(this);
        m_deviceAPI->addChannelSinkAPI(this);
    }
}

uint32_t AcarsDemod::getNumberOfDeviceStreams() const
{
    return m_deviceAPI->getNbSourceStreams();
}

void AcarsDemod::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end, bool firstOfBurst)
{
    (void) firstOfBurst;
    m_basebandSink->feed(begin, end);
}

void AcarsDemod::start()
{
    qDebug("AcarsDemod::start");

    m_basebandSink->reset();
    m_basebandSink->startWork();
    m_thread.start();

    DSPSignalNotification *dspMsg = new DSPSignalNotification(m_basebandSampleRate, m_centerFrequency);
    m_basebandSink->getInputMessageQueue()->push(dspMsg);

    AcarsDemodBaseband::MsgConfigureAcarsDemodBaseband *msg = AcarsDemodBaseband::MsgConfigureAcarsDemodBaseband::create(m_settings, true);
    m_basebandSink->getInputMessageQueue()->push(msg);
}

void AcarsDemod::stop()
{
    qDebug("AcarsDemod::stop");
    m_basebandSink->stopWork();
    m_thread.quit();
    m_thread.wait();
}

bool AcarsDemod::handleMessage(const Message& cmd)
{
    if (MsgConfigureAcarsDemod::match(cmd))
    {
        MsgConfigureAcarsDemod& cfg = (MsgConfigureAcarsDemod&) cmd;
        qDebug() << "AcarsDemod::handleMessage: MsgConfigureAcarsDemod";
        applySettings(cfg.getSettings(), cfg.getForce());

        return true;
    }
    else if (DSPSignalNotification::match(cmd))
    {
        DSPSignalNotification& notif = (DSPSignalNotification&) cmd;
        m_basebandSampleRate = notif.getSampleRate();
        m_centerFrequency = notif.getCenterFrequency();
        // Forward to the sink
        DSPSignalNotification* rep = new DSPSignalNotification(notif); // make a copy
        qDebug() << "AcarsDemod::handleMessage: DSPSignalNotification";
        m_basebandSink->getInputMessageQueue()->push(rep);
        // Forward to the worker, which needs the centre frequency for its outputs
        if (m_worker) {
            m_worker->getInputMessageQueue()->push(new DSPSignalNotification(notif));
        }
        // Forward to GUI if any
        if (m_guiMessageQueue)
        {
            rep = new DSPSignalNotification(notif);
            m_guiMessageQueue->push(rep);
        }

        return true;
    }
    else
    {
        return false;
    }
}

void AcarsDemod::setCenterFrequency(qint64 frequency)
{
    AcarsDemodSettings settings = m_settings;
    settings.m_inputFrequencyOffset = frequency;
    applySettings(settings, false);

    if (m_guiMessageQueue) // forward to GUI if any
    {
        MsgConfigureAcarsDemod *msgToGUI = MsgConfigureAcarsDemod::create(settings, false);
        m_guiMessageQueue->push(msgToGUI);
    }
}

void AcarsDemod::applySettings(const AcarsDemodSettings& settings, bool force)
{
    qDebug() << "AcarsDemod::applySettings:"
            << " m_logEnabled: " << settings.m_logEnabled
            << " m_logFilename: " << settings.m_logFilename
            << " m_streamIndex: " << settings.m_streamIndex
            << " m_useReverseAPI: " << settings.m_useReverseAPI
            << " m_reverseAPIAddress: " << settings.m_reverseAPIAddress
            << " m_reverseAPIPort: " << settings.m_reverseAPIPort
            << " m_reverseAPIDeviceIndex: " << settings.m_reverseAPIDeviceIndex
            << " m_reverseAPIChannelIndex: " << settings.m_reverseAPIChannelIndex
            << " force: " << force;

    QList<QString> reverseAPIKeys;

    if ((settings.m_inputFrequencyOffset != m_settings.m_inputFrequencyOffset) || force) {
        reverseAPIKeys.append("inputFrequencyOffset");
    }
    if ((settings.m_rfBandwidth != m_settings.m_rfBandwidth) || force) {
        reverseAPIKeys.append("rfBandwidth");
    }
    if ((settings.m_udpEnabled != m_settings.m_udpEnabled) || force) {
        reverseAPIKeys.append("udpEnabled");
    }
    if ((settings.m_udpAddress != m_settings.m_udpAddress) || force) {
        reverseAPIKeys.append("udpAddress");
    }
    if ((settings.m_udpPort != m_settings.m_udpPort) || force) {
        reverseAPIKeys.append("udpPort");
    }
    if ((settings.m_logFilename != m_settings.m_logFilename) || force) {
        reverseAPIKeys.append("logFilename");
    }
    if ((settings.m_logEnabled != m_settings.m_logEnabled) || force) {
        reverseAPIKeys.append("logEnabled");
    }
    if (m_settings.m_streamIndex != settings.m_streamIndex)
    {
        if (m_deviceAPI->getSampleMIMO()) // change of stream is possible for MIMO devices only
        {
            m_deviceAPI->removeChannelSinkAPI(this);
            m_deviceAPI->removeChannelSink(this, m_settings.m_streamIndex);
            m_deviceAPI->addChannelSink(this, settings.m_streamIndex);
            m_deviceAPI->addChannelSinkAPI(this);
        }

        reverseAPIKeys.append("streamIndex");
    }

    AcarsDemodBaseband::MsgConfigureAcarsDemodBaseband *msg = AcarsDemodBaseband::MsgConfigureAcarsDemodBaseband::create(settings, force);
    m_basebandSink->getInputMessageQueue()->push(msg);

    if (settings.m_useReverseAPI)
    {
        bool fullUpdate = ((m_settings.m_useReverseAPI != settings.m_useReverseAPI) && settings.m_useReverseAPI) ||
                (m_settings.m_reverseAPIAddress != settings.m_reverseAPIAddress) ||
                (m_settings.m_reverseAPIPort != settings.m_reverseAPIPort) ||
                (m_settings.m_reverseAPIDeviceIndex != settings.m_reverseAPIDeviceIndex) ||
                (m_settings.m_reverseAPIChannelIndex != settings.m_reverseAPIChannelIndex);
        webapiReverseSendSettings(reverseAPIKeys, settings, fullUpdate || force);
    }

    // The worker handles the outputs (UDP, log, pipes) and, in later phases,
    // the higher layer decode
    if (m_worker) {
        m_worker->getInputMessageQueue()->push(AcarsDemodWorker::MsgConfigureWorker::create(settings, force));
    }

    m_settings = settings;
}

QByteArray AcarsDemod::serialize() const
{
    return m_settings.serialize();
}

bool AcarsDemod::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        MsgConfigureAcarsDemod *msg = MsgConfigureAcarsDemod::create(m_settings, true);
        m_inputMessageQueue.push(msg);
        return true;
    }
    else
    {
        m_settings.resetToDefaults();
        MsgConfigureAcarsDemod *msg = MsgConfigureAcarsDemod::create(m_settings, true);
        m_inputMessageQueue.push(msg);
        return false;
    }
}

int AcarsDemod::webapiSettingsGet(
        SWGSDRangel::SWGChannelSettings& response,
        QString& errorMessage)
{
    (void) errorMessage;
  /*  response.setAcarsDemodSettings(new SWGSDRangel::SWGACARSDemodSettings());
    response.getAcarsDemodSettings()->init();
    webapiFormatChannelSettings(response, m_settings);  */
    return 200;
}

int AcarsDemod::webapiSettingsPutPatch(
        bool force,
        const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response,
        QString& errorMessage)
{
  /*  (void) errorMessage;
    AcarsDemodSettings settings = m_settings;
    webapiUpdateChannelSettings(settings, channelSettingsKeys, response);

    MsgConfigureAcarsDemod *msg = MsgConfigureAcarsDemod::create(settings, force);
    m_inputMessageQueue.push(msg);

    qDebug("AcarsDemod::webapiSettingsPutPatch: forward to GUI: %p", m_guiMessageQueue);
    if (m_guiMessageQueue) // forward to GUI if any
    {
        MsgConfigureAcarsDemod *msgToGUI = MsgConfigureAcarsDemod::create(settings, force);
        m_guiMessageQueue->push(msgToGUI);
    }

    webapiFormatChannelSettings(response, settings);
               */
    return 200;
}

void AcarsDemod::webapiUpdateChannelSettings(
        AcarsDemodSettings& settings,
        const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response)
{
   /* if (channelSettingsKeys.contains("inputFrequencyOffset")) {
        settings.m_inputFrequencyOffset = response.getAcarsDemodSettings()->getInputFrequencyOffset();
    }
    if (channelSettingsKeys.contains("fmDeviation")) {
        settings.m_fmDeviation = response.getAcarsDemodSettings()->getFmDeviation();
    }
    if (channelSettingsKeys.contains("rfBandwidth")) {
        settings.m_rfBandwidth = response.getAcarsDemodSettings()->getRfBandwidth();
    }
    if (channelSettingsKeys.contains("udpEnabled")) {
        settings.m_udpEnabled = response.getAcarsDemodSettings()->getUdpEnabled();
    }
    if (channelSettingsKeys.contains("udpAddress")) {
        settings.m_udpAddress = *response.getAcarsDemodSettings()->getUdpAddress();
    }
    if (channelSettingsKeys.contains("udpPort")) {
        settings.m_udpPort = response.getAcarsDemodSettings()->getUdpPort();
    }
    if (channelSettingsKeys.contains("rgbColor")) {
        settings.m_rgbColor = response.getAcarsDemodSettings()->getRgbColor();
    }
    if (channelSettingsKeys.contains("title")) {
        settings.m_title = *response.getAcarsDemodSettings()->getTitle();
    }
    if (channelSettingsKeys.contains("streamIndex")) {
        settings.m_streamIndex = response.getAcarsDemodSettings()->getStreamIndex();
    }
    if (channelSettingsKeys.contains("useReverseAPI")) {
        settings.m_useReverseAPI = response.getAcarsDemodSettings()->getUseReverseApi() != 0;
    }
    if (channelSettingsKeys.contains("reverseAPIAddress")) {
        settings.m_reverseAPIAddress = *response.getAcarsDemodSettings()->getReverseApiAddress();
    }
    if (channelSettingsKeys.contains("reverseAPIPort")) {
        settings.m_reverseAPIPort = response.getAcarsDemodSettings()->getReverseApiPort();
    }
    if (channelSettingsKeys.contains("reverseAPIDeviceIndex")) {
        settings.m_reverseAPIDeviceIndex = response.getAcarsDemodSettings()->getReverseApiDeviceIndex();
    }
    if (channelSettingsKeys.contains("reverseAPIChannelIndex")) {
        settings.m_reverseAPIChannelIndex = response.getAcarsDemodSettings()->getReverseApiChannelIndex();
    }        */
}

void AcarsDemod::webapiFormatChannelSettings(SWGSDRangel::SWGChannelSettings& response, const AcarsDemodSettings& settings)
{
    /*response.getAcarsDemodSettings()->setFmDeviation(settings.m_fmDeviation);
    response.getAcarsDemodSettings()->setInputFrequencyOffset(settings.m_inputFrequencyOffset);
    response.getAcarsDemodSettings()->setRfBandwidth(settings.m_rfBandwidth);
    response.getAcarsDemodSettings()->setUdpEnabled(settings.m_udpEnabled);
    response.getAcarsDemodSettings()->setUdpAddress(new QString(settings.m_udpAddress));
    response.getAcarsDemodSettings()->setUdpPort(settings.m_udpPort);

    response.getAcarsDemodSettings()->setRgbColor(settings.m_rgbColor);
    if (response.getAcarsDemodSettings()->getTitle()) {
        *response.getAcarsDemodSettings()->getTitle() = settings.m_title;
    } else {
        response.getAcarsDemodSettings()->setTitle(new QString(settings.m_title));
    }

    response.getAcarsDemodSettings()->setStreamIndex(settings.m_streamIndex);
    response.getAcarsDemodSettings()->setUseReverseApi(settings.m_useReverseAPI ? 1 : 0);

    if (response.getAcarsDemodSettings()->getReverseApiAddress()) {
        *response.getAcarsDemodSettings()->getReverseApiAddress() = settings.m_reverseAPIAddress;
    } else {
        response.getAcarsDemodSettings()->setReverseApiAddress(new QString(settings.m_reverseAPIAddress));
    }

    response.getAcarsDemodSettings()->setReverseApiPort(settings.m_reverseAPIPort);
    response.getAcarsDemodSettings()->setReverseApiDeviceIndex(settings.m_reverseAPIDeviceIndex);
    response.getAcarsDemodSettings()->setReverseApiChannelIndex(settings.m_reverseAPIChannelIndex);   */
}

void AcarsDemod::webapiReverseSendSettings(QList<QString>& channelSettingsKeys, const AcarsDemodSettings& settings, bool force)
{
   /* SWGSDRangel::SWGChannelSettings *swgChannelSettings = new SWGSDRangel::SWGChannelSettings();
    webapiFormatChannelSettings(channelSettingsKeys, swgChannelSettings, settings, force);

    QString channelSettingsURL = QString("http://%1:%2/sdrangel/deviceset/%3/channel/%4/settings")
            .arg(settings.m_reverseAPIAddress)
            .arg(settings.m_reverseAPIPort)
            .arg(settings.m_reverseAPIDeviceIndex)
            .arg(settings.m_reverseAPIChannelIndex);
    m_networkRequest.setUrl(QUrl(channelSettingsURL));
    m_networkRequest.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QBuffer *buffer = new QBuffer();
    buffer->open((QBuffer::ReadWrite));
    buffer->write(swgChannelSettings->asJson().toUtf8());
    buffer->seek(0);

    // Always use PATCH to avoid passing reverse API settings
    QNetworkReply *reply = m_networkManager->sendCustomRequest(m_networkRequest, "PATCH", buffer);
    buffer->setParent(reply);

    delete swgChannelSettings; */
}

void AcarsDemod::webapiFormatChannelSettings(
        QList<QString>& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings *swgChannelSettings,
        const AcarsDemodSettings& settings,
        bool force
)
{
  /*  swgChannelSettings->setDirection(0); // Single sink (Rx)
    swgChannelSettings->setOriginatorChannelIndex(getIndexInDeviceSet());
    swgChannelSettings->setOriginatorDeviceSetIndex(getDeviceSetIndex());
    swgChannelSettings->setChannelType(new QString("AcarsDemod"));
    swgChannelSettings->setAcarsDemodSettings(new SWGSDRangel::SWGACARSDemodSettings());
    SWGSDRangel::SWGACARSDemodSettings *sWGACARSDemodSettings = swgChannelSettings->getAcarsDemodSettings();

    // transfer data that has been modified. When force is on transfer all data except reverse API data

    if (channelSettingsKeys.contains("inputFrequencyOffset") || force) {
        sWGACARSDemodSettings->setInputFrequencyOffset(settings.m_inputFrequencyOffset);
    }
    if (channelSettingsKeys.contains("rfBandwidth") || force) {
        sWGACARSDemodSettings->setRfBandwidth(settings.m_rfBandwidth);
    }
    if (channelSettingsKeys.contains("udpEnabled") || force) {
        sWGACARSDemodSettings->setUdpEnabled(settings.m_udpEnabled);
    }
    if (channelSettingsKeys.contains("udpAddress") || force) {
        sWGACARSDemodSettings->setUdpAddress(new QString(settings.m_udpAddress));
    }
    if (channelSettingsKeys.contains("udpPort") || force) {
        sWGACARSDemodSettings->setUdpPort(settings.m_udpPort);
    }
    if (channelSettingsKeys.contains("rgbColor") || force) {
        sWGACARSDemodSettings->setRgbColor(settings.m_rgbColor);
    }
    if (channelSettingsKeys.contains("title") || force) {
        sWGACARSDemodSettings->setTitle(new QString(settings.m_title));
    }
    if (channelSettingsKeys.contains("streamIndex") || force) {
        sWGACARSDemodSettings->setStreamIndex(settings.m_streamIndex);
    }            */
}

void AcarsDemod::networkManagerFinished(QNetworkReply *reply)
{
    QNetworkReply::NetworkError replyError = reply->error();

    if (replyError)
    {
        qWarning() << "AcarsDemod::networkManagerFinished:"
                << " error(" << (int) replyError
                << "): " << replyError
                << ": " << reply->errorString();
    }
    else
    {
        QString answer = reply->readAll();
        answer.chop(1); // remove last \n
        qDebug("AcarsDemod::networkManagerFinished: reply:\n%s", answer.toStdString().c_str());
    }

    reply->deleteLater();
}
