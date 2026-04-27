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

#ifndef INCLUDE_FEATURE_CAMERAWORKER_H_
#define INCLUDE_FEATURE_CAMERAWORKER_H_

#include <QObject>
#include <QHash>
#include <QSize>
#include <QSet>
#include <QTimer>
#include <QImage>
#include <QDateTime>
#include <QRecursiveMutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/dnn/dnn.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "audio/audiofifo.h"
#include "availabledevicehandler.h"
#include "camerasettings.h"

class QNetworkAccessManager;
class CameraPostProcessor;

class CameraWorker : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraWorker : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraWorker* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraWorker(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraWorker(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    class MsgStartStop : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool getStartStop() const { return m_startStop; }

        static MsgStartStop* create(bool startStop)
        {
            return new MsgStartStop(startStop);
        }

    private:
        bool m_startStop;

        MsgStartStop(bool startStop) :
            Message(),
            m_startStop(startStop)
        { }
    };

    class MsgRefreshCameraList : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgRefreshCameraList* create()
        {
            return new MsgRefreshCameraList();
        }

    private:
        MsgRefreshCameraList() : Message() {}
    };

    class MsgReportCameraList : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QStringList& getCameraIds() const { return m_cameraIds; }

        static MsgReportCameraList* create(const QStringList& cameraIds)
        {
            return new MsgReportCameraList(cameraIds);
        }

    private:
        QStringList m_cameraIds;

        MsgReportCameraList(const QStringList& cameraIds) :
            Message(),
            m_cameraIds(cameraIds)
        { }
    };

    class MsgReportAlpacaCameraInfo : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        int getMaxBinX() const { return m_maxBinX; }
        int getMaxBinY() const { return m_maxBinY; }
        const QStringList& getGains() const { return m_gains; }
        int getGainMin() const { return m_gainMin; }
        int getGainMax() const { return m_gainMax; }
        const QStringList& getOffsets() const { return m_offsets; }
        int getOffsetMin() const { return m_offsetMin; }
        int getOffsetMax() const { return m_offsetMax; }
        const QStringList& getReadoutModes() const { return m_readoutModes; }
        const QString& getSensorName() const { return m_sensorName; }
        int getSensorType() const { return m_sensorType; }
        double getPixelSizeX() const { return m_pixelSizeX; }
        double getPixelSizeY() const { return m_pixelSizeY; }
        int getCameraSizeX() const { return m_cameraSizeX; }
        int getCameraSizeY() const { return m_cameraSizeY; }
        double getCcdTemperature() const { return m_ccdTemperature; }
        bool isCcdTemperatureValid() const { return m_ccdTemperatureValid; }

        static MsgReportAlpacaCameraInfo* create(
            int maxBinX, int maxBinY,
            const QStringList& gains, int gainMin, int gainMax,
            const QStringList& offsets, int offsetMin, int offsetMax,
            const QStringList& readoutModes,
            const QString& sensorName, int sensorType,
            double pixelSizeX, double pixelSizeY,
            int cameraSizeX, int cameraSizeY,
            double ccdTemperature, bool ccdTemperatureValid)
        {
            return new MsgReportAlpacaCameraInfo(
                maxBinX, maxBinY, gains, gainMin, gainMax, offsets, offsetMin, offsetMax,
                readoutModes, sensorName, sensorType, pixelSizeX, pixelSizeY,
                cameraSizeX, cameraSizeY, ccdTemperature, ccdTemperatureValid);
        }

    private:
        int m_maxBinX;
        int m_maxBinY;
        QStringList m_gains;
        int m_gainMin;
        int m_gainMax;
        QStringList m_offsets;
        int m_offsetMin;
        int m_offsetMax;
        QStringList m_readoutModes;
        QString m_sensorName;
        int m_sensorType;
        double m_pixelSizeX;
        double m_pixelSizeY;
        int m_cameraSizeX;
        int m_cameraSizeY;
        double m_ccdTemperature;
        bool m_ccdTemperatureValid;

        MsgReportAlpacaCameraInfo(
            int maxBinX, int maxBinY,
            const QStringList& gains, int gainMin, int gainMax,
            const QStringList& offsets, int offsetMin, int offsetMax,
            const QStringList& readoutModes,
            const QString& sensorName, int sensorType,
            double pixelSizeX, double pixelSizeY,
            int cameraSizeX, int cameraSizeY,
            double ccdTemperature, bool ccdTemperatureValid) :
            Message(),
            m_maxBinX(maxBinX),
            m_maxBinY(maxBinY),
            m_gains(gains),
            m_gainMin(gainMin),
            m_gainMax(gainMax),
            m_offsets(offsets),
            m_offsetMin(offsetMin),
            m_offsetMax(offsetMax),
            m_readoutModes(readoutModes),
            m_sensorName(sensorName),
            m_sensorType(sensorType),
            m_pixelSizeX(pixelSizeX),
            m_pixelSizeY(pixelSizeY),
            m_cameraSizeX(cameraSizeX),
            m_cameraSizeY(cameraSizeY),
            m_ccdTemperature(ccdTemperature),
            m_ccdTemperatureValid(ccdTemperatureValid)
        { }
    };

    // Sent when the set of available spectrum-view devices changes
    class MsgReportAvailableDevices : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QStringList& getDeviceLongIds() const { return m_deviceLongIds; }

        static MsgReportAvailableDevices* create(const QStringList& deviceLongIds)
        {
            return new MsgReportAvailableDevices(deviceLongIds);
        }

    private:
        QStringList m_deviceLongIds;

        MsgReportAvailableDevices(const QStringList& deviceLongIds) :
            Message(),
            m_deviceLongIds(deviceLongIds)
        { }
    };

    // Sent periodically to update live status fields (camerastate, ccdtemperature)
    class MsgReportAlpacaStatus : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        int getCameraState() const { return m_cameraState; }
        double getCcdTemperature() const { return m_ccdTemperature; }
        bool isCcdTemperatureValid() const { return m_ccdTemperatureValid; }

        static MsgReportAlpacaStatus* create(int cameraState, double ccdTemperature, bool ccdTemperatureValid)
        {
            return new MsgReportAlpacaStatus(cameraState, ccdTemperature, ccdTemperatureValid);
        }

    private:
        int m_cameraState;
        double m_ccdTemperature;
        bool m_ccdTemperatureValid;

        MsgReportAlpacaStatus(int cameraState, double ccdTemperature, bool ccdTemperatureValid) :
            Message(),
            m_cameraState(cameraState),
            m_ccdTemperature(ccdTemperature),
            m_ccdTemperatureValid(ccdTemperatureValid)
        { }
    };

    CameraWorker();
    ~CameraWorker();

    void startWork();
    void stopWork();
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }
    void setPostProcessorInputMessageQueue(MessageQueue *messageQueue) { m_postProcessorInputMessageQueue = messageQueue; }

private:
    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_postProcessorInputMessageQueue;
    QRecursiveMutex m_mutex;
    CameraSettings m_settings;
    AvailableDeviceHandler m_availableDeviceHandler;
    AvailableDeviceList m_availableDevices;
    bool m_capturing;
    bool m_capturingAudio;
    QTimer m_captureTimer;
    QNetworkAccessManager *m_networkManager;
    bool m_alpacaFrameRequestPending;
    quint32 m_alpacaClientId;
    quint32 m_alpacaClientTransactionId;
    int m_alpacaSensorType;          // 0=Mono, 1=Colour, 2=RGGB, 3=CMYG, 4=CMYG2, 5=LRGB
    bool m_alpacaImageBytesSupported; // true = try ImageBytes binary protocol; false = use JSON
    QTimer m_statusTimer;   // polls camerastate + ccdtemperature every 2 s
    QObject *m_spectrumPipeSource; ///< Cached pointer to the DeviceAPI of the selected spectrum device

    // Audio pass-through (Qt camera only)
    AudioFifo m_captureAudioFifo;  ///< Receives captured microphone samples from AudioDeviceManager
    AudioFifo m_outputAudioFifo;   ///< Feeds samples to AudioDeviceManager for playback
    QVector<quint8> m_audioTransferBuffer; ///< Scratch buffer for the capture→output copy

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void startCapture();
    void stopCapture();
    QImage createPlaceholderFrame() const;
    QString buildAlpacaBaseUrl() const;
    QImage parseAlpacaImageArray(const QByteArray& payload) const;
    QImage parseAlpacaImageBytes(const QByteArray& payload) const;
    QImage renderRawPixelArray(const QVector<QVector<int>>& raw, int width, int height) const;

    void alpacaStartExposure();
    void alpacaCheckImageReady();
    void alpacaFetchImageArray();
    void alpacaQueryCameraCapabilities();
    void alpacaSetCameraParams();
    void alpacaPollStatus();

private slots:
    void handleInputMessages();
    void handleDeviceMessageQueue(MessageQueue* messageQueue);
    void captureTick();
    void statusTick();
    void onAvailableDevicesChanged(const QStringList& renameFrom, const QStringList& renameTo,
                                   const QStringList& removed, const QStringList& added);
    void onCaptureAudioDataReady(); ///< Called when microphone samples are available in m_captureAudioFifo
};

#endif // INCLUDE_FEATURE_CAMERAWORKER_H_
