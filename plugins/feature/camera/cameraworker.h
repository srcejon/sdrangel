///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
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
#include <QByteArray>
#include <QRecursiveMutex>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/dnn/dnn.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "availabledevicehandler.h"
#include "cameraalpacacontroller.h"
#include "cameraasicontroller.h"
#include "camerainfo.h"
#include "cameramediaplaybackcontroller.h"
#include "camerapipelineframe.h"
#include "cameraqtaudiocontroller.h"
#include "camerasettings.h"

class QNetworkAccessManager;
class QNetworkReply;
class QUrl;
class CameraPostProcessor;
class CameraFramePreprocessor;
class CameraFinder;

class CameraWorker : public QObject
{
    Q_OBJECT
public:
    class MsgReportAlpacaFilterWheelInfo : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QStringList& getNames() const { return m_names; }
        int getPosition() const { return m_position; }

        static MsgReportAlpacaFilterWheelInfo* create(const QStringList& names, int position)
        {
            return new MsgReportAlpacaFilterWheelInfo(names, position);
        }

    private:
        QStringList m_names;
        int m_position;

        MsgReportAlpacaFilterWheelInfo(const QStringList& names, int position) :
            Message(),
            m_names(names),
            m_position(position)
        { }
    };

    class MsgStartStop : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool getStartStop() const { return m_startStop; }
        quint64 getCaptureEpoch() const { return m_captureEpoch; }

        static MsgStartStop* create(bool startStop, quint64 captureEpoch = 0)
        {
            return new MsgStartStop(startStop, captureEpoch);
        }

    private:
        bool m_startStop;
        quint64 m_captureEpoch;

        MsgStartStop(bool startStop, quint64 captureEpoch) :
            Message(),
            m_startStop(startStop),
            m_captureEpoch(captureEpoch)
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

    class MsgStartAutoFocus : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgStartAutoFocus* create()
        {
            return new MsgStartAutoFocus();
        }

    private:
        MsgStartAutoFocus() : Message() {}
    };

    class MsgVideoFileControl : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        enum Action
        {
            Play,
            Pause,
            Restart,
            StepBack,
            StepForward,
            Seek
        };

        Action getAction() const { return m_action; }
        qint64 getPositionMs() const { return m_positionMs; }

        static MsgVideoFileControl* create(Action action, qint64 positionMs = -1)
        {
            return new MsgVideoFileControl(action, positionMs);
        }

    private:
        Action m_action;
        qint64 m_positionMs;

        MsgVideoFileControl(Action action, qint64 positionMs) :
            Message(),
            m_action(action),
            m_positionMs(positionMs)
        { }
    };

    class MsgReportVideoFilePlayback : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        qint64 getPositionMs() const { return m_positionMs; }
        qint64 getDurationMs() const { return m_durationMs; }
        bool isPlaying() const { return m_playing; }
        bool isOpen() const { return m_open; }

        static MsgReportVideoFilePlayback* create(qint64 positionMs, qint64 durationMs, bool playing, bool open)
        {
            return new MsgReportVideoFilePlayback(positionMs, durationMs, playing, open);
        }

    private:
        qint64 m_positionMs;
        qint64 m_durationMs;
        bool m_playing;
        bool m_open;

        MsgReportVideoFilePlayback(qint64 positionMs, qint64 durationMs, bool playing, bool open) :
            Message(),
            m_positionMs(positionMs),
            m_durationMs(durationMs),
            m_playing(playing),
            m_open(open)
        { }
    };

    class MsgReportCameraList : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QList<CameraInfo>& getCameras() const { return m_cameras; }

        static MsgReportCameraList* create(const QList<CameraInfo>& cameras)
        {
            return new MsgReportCameraList(cameras);
        }

    private:
        QList<CameraInfo> m_cameras;

        MsgReportCameraList(const QList<CameraInfo>& cameras) :
            Message(),
            m_cameras(cameras)
        { }
    };

    class MsgReportAlpacaDeviceList : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QList<AlpacaDeviceInfo>& getFocusers() const { return m_focusers; }
        const QList<AlpacaDeviceInfo>& getFilterWheels() const { return m_filterWheels; }

        static MsgReportAlpacaDeviceList* create(const QList<AlpacaDeviceInfo>& focusers, const QList<AlpacaDeviceInfo>& filterWheels)
        {
            return new MsgReportAlpacaDeviceList(focusers, filterWheels);
        }

    private:
        QList<AlpacaDeviceInfo> m_focusers;
        QList<AlpacaDeviceInfo> m_filterWheels;

        MsgReportAlpacaDeviceList(const QList<AlpacaDeviceInfo>& focusers, const QList<AlpacaDeviceInfo>& filterWheels) :
            Message(),
            m_focusers(focusers),
            m_filterWheels(filterWheels)
        { }
    };

    class MsgReportAlpacaCameraInfo : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QString& getName() const { return m_name; }
        const QString& getDescription() const { return m_description; }
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
        double getExposureMinMs() const { return m_exposureMinMs; }
        double getExposureMaxMs() const { return m_exposureMaxMs; }
        double getExposureResolutionMs() const { return m_exposureResolutionMs; }

        static MsgReportAlpacaCameraInfo* create(
            const QString& name, const QString& description,
            int maxBinX, int maxBinY,
            const QStringList& gains, int gainMin, int gainMax,
            const QStringList& offsets, int offsetMin, int offsetMax,
            const QStringList& readoutModes,
            const QString& sensorName, int sensorType,
            double pixelSizeX, double pixelSizeY,
            int cameraSizeX, int cameraSizeY,
            double ccdTemperature, bool ccdTemperatureValid,
            double exposureMinMs, double exposureMaxMs, double exposureResolutionMs)
        {
            return new MsgReportAlpacaCameraInfo(
                name, description, maxBinX, maxBinY, gains, gainMin, gainMax, offsets, offsetMin, offsetMax,
                readoutModes, sensorName, sensorType, pixelSizeX, pixelSizeY,
                cameraSizeX, cameraSizeY, ccdTemperature, ccdTemperatureValid,
                exposureMinMs, exposureMaxMs, exposureResolutionMs);
        }

    private:
        QString m_name;
        QString m_description;
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
        double m_exposureMinMs;
        double m_exposureMaxMs;
        double m_exposureResolutionMs;

        MsgReportAlpacaCameraInfo(
            const QString& name, const QString& description,
            int maxBinX, int maxBinY,
            const QStringList& gains, int gainMin, int gainMax,
            const QStringList& offsets, int offsetMin, int offsetMax,
            const QStringList& readoutModes,
            const QString& sensorName, int sensorType,
            double pixelSizeX, double pixelSizeY,
            int cameraSizeX, int cameraSizeY,
            double ccdTemperature, bool ccdTemperatureValid,
            double exposureMinMs, double exposureMaxMs, double exposureResolutionMs) :
            Message(),
            m_name(name),
            m_description(description),
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
            m_ccdTemperatureValid(ccdTemperatureValid),
            m_exposureMinMs(exposureMinMs),
            m_exposureMaxMs(exposureMaxMs),
            m_exposureResolutionMs(exposureResolutionMs)
        { }
    };

    class MsgReportAsiCameraInfo : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QString& getName() const { return m_name; }
        int getMaxBinX() const { return m_maxBinX; }
        int getMaxBinY() const { return m_maxBinY; }
        int getGainMin() const { return m_gainMin; }
        int getGainMax() const { return m_gainMax; }
        int getOffsetMin() const { return m_offsetMin; }
        int getOffsetMax() const { return m_offsetMax; }
        int getCameraSizeX() const { return m_cameraSizeX; }
        int getCameraSizeY() const { return m_cameraSizeY; }
        double getPixelSizeUm() const { return m_pixelSizeUm; }
        int getBitDepth() const { return m_bitDepth; }
        bool isColor() const { return m_isColor; }
        double getExposureMinMs() const { return m_exposureMinMs; }
        double getExposureMaxMs() const { return m_exposureMaxMs; }
        bool isCoolerSupported() const { return m_coolerSupported; }
        bool isCoolerOn() const { return m_coolerOn; }
        bool isTargetTempSupported() const { return m_targetTempSupported; }
        int getTargetTempMin() const { return m_targetTempMin; }
        int getTargetTempMax() const { return m_targetTempMax; }
        int getTargetTemp() const { return m_targetTemp; }
        bool isUsbBandwidthSupported() const { return m_usbBandwidthSupported; }
        int getUsbBandwidthMin() const { return m_usbBandwidthMin; }
        int getUsbBandwidthMax() const { return m_usbBandwidthMax; }
        int getUsbBandwidth() const { return m_usbBandwidth; }
        bool isHighSpeedModeSupported() const { return m_highSpeedModeSupported; }
        bool isHighSpeedMode() const { return m_highSpeedMode; }
        bool isRgb24Supported() const { return m_rgb24Supported; }
        bool isRaw16Supported() const { return m_raw16Supported; }
        bool isRaw8Supported() const { return m_raw8Supported; }

        static MsgReportAsiCameraInfo* create(const QString& name, int maxBinX, int maxBinY,
            int gainMin, int gainMax, int offsetMin, int offsetMax,
            int cameraSizeX, int cameraSizeY, double pixelSizeUm, int bitDepth, bool isColor,
            double exposureMinMs, double exposureMaxMs,
            bool coolerSupported, bool coolerOn,
            bool targetTempSupported, int targetTempMin, int targetTempMax, int targetTemp,
            bool usbBandwidthSupported, int usbBandwidthMin, int usbBandwidthMax, int usbBandwidth,
            bool highSpeedModeSupported, bool highSpeedMode,
            bool rgb24Supported, bool raw16Supported, bool raw8Supported)
        {
            return new MsgReportAsiCameraInfo(name, maxBinX, maxBinY, gainMin, gainMax, offsetMin, offsetMax,
                cameraSizeX, cameraSizeY, pixelSizeUm, bitDepth, isColor, exposureMinMs, exposureMaxMs,
                coolerSupported, coolerOn, targetTempSupported, targetTempMin, targetTempMax, targetTemp,
                usbBandwidthSupported, usbBandwidthMin, usbBandwidthMax, usbBandwidth,
                highSpeedModeSupported, highSpeedMode, rgb24Supported, raw16Supported, raw8Supported);
        }

    private:
        QString m_name;
        int m_maxBinX;
        int m_maxBinY;
        int m_gainMin;
        int m_gainMax;
        int m_offsetMin;
        int m_offsetMax;
        int m_cameraSizeX;
        int m_cameraSizeY;
        double m_pixelSizeUm;
        int m_bitDepth;
        bool m_isColor;
        double m_exposureMinMs;
        double m_exposureMaxMs;
        bool m_coolerSupported;
        bool m_coolerOn;
        bool m_targetTempSupported;
        int m_targetTempMin;
        int m_targetTempMax;
        int m_targetTemp;
        bool m_usbBandwidthSupported;
        int m_usbBandwidthMin;
        int m_usbBandwidthMax;
        int m_usbBandwidth;
        bool m_highSpeedModeSupported;
        bool m_highSpeedMode;
        bool m_rgb24Supported;
        bool m_raw16Supported;
        bool m_raw8Supported;

        MsgReportAsiCameraInfo(const QString& name, int maxBinX, int maxBinY,
            int gainMin, int gainMax, int offsetMin, int offsetMax,
            int cameraSizeX, int cameraSizeY, double pixelSizeUm, int bitDepth, bool isColor,
            double exposureMinMs, double exposureMaxMs,
            bool coolerSupported, bool coolerOn,
            bool targetTempSupported, int targetTempMin, int targetTempMax, int targetTemp,
            bool usbBandwidthSupported, int usbBandwidthMin, int usbBandwidthMax, int usbBandwidth,
            bool highSpeedModeSupported, bool highSpeedMode,
            bool rgb24Supported, bool raw16Supported, bool raw8Supported) :
            Message(),
            m_name(name),
            m_maxBinX(maxBinX),
            m_maxBinY(maxBinY),
            m_gainMin(gainMin),
            m_gainMax(gainMax),
            m_offsetMin(offsetMin),
            m_offsetMax(offsetMax),
            m_cameraSizeX(cameraSizeX),
            m_cameraSizeY(cameraSizeY),
            m_pixelSizeUm(pixelSizeUm),
            m_bitDepth(bitDepth),
            m_isColor(isColor),
            m_exposureMinMs(exposureMinMs),
            m_exposureMaxMs(exposureMaxMs),
            m_coolerSupported(coolerSupported),
            m_coolerOn(coolerOn),
            m_targetTempSupported(targetTempSupported),
            m_targetTempMin(targetTempMin),
            m_targetTempMax(targetTempMax),
            m_targetTemp(targetTemp),
            m_usbBandwidthSupported(usbBandwidthSupported),
            m_usbBandwidthMin(usbBandwidthMin),
            m_usbBandwidthMax(usbBandwidthMax),
            m_usbBandwidth(usbBandwidth),
            m_highSpeedModeSupported(highSpeedModeSupported),
            m_highSpeedMode(highSpeedMode),
            m_rgb24Supported(rgb24Supported),
            m_raw16Supported(raw16Supported),
            m_raw8Supported(raw8Supported)
        {}
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

    class MsgReportAutoExposureGain : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        double getExposureTimeMs() const { return m_exposureTimeMs; }
        int getGain() const { return m_gain; }
        double getMeasuredBrightness() const { return m_measuredBrightness; }
        double getSaturatedFraction() const { return m_saturatedFraction; }

        static MsgReportAutoExposureGain* create(double exposureTimeMs, int gain, double measuredBrightness, double saturatedFraction)
        {
            return new MsgReportAutoExposureGain(exposureTimeMs, gain, measuredBrightness, saturatedFraction);
        }

    private:
        double m_exposureTimeMs;
        int m_gain;
        double m_measuredBrightness;
        double m_saturatedFraction;

        MsgReportAutoExposureGain(double exposureTimeMs, int gain, double measuredBrightness, double saturatedFraction) :
            Message(),
            m_exposureTimeMs(exposureTimeMs),
            m_gain(gain),
            m_measuredBrightness(measuredBrightness),
            m_saturatedFraction(saturatedFraction)
        { }
    };

    class MsgReportAutoFocus : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QString& getStatus() const { return m_status; }
        bool isActive() const { return m_active; }
        int getPosition() const { return m_position; }
        double getScore() const { return m_score; }
        int getStepIndex() const { return m_stepIndex; }
        int getStepCount() const { return m_stepCount; }

        static MsgReportAutoFocus* create(const QString& status, bool active, int position, double score, int stepIndex, int stepCount)
        {
            return new MsgReportAutoFocus(status, active, position, score, stepIndex, stepCount);
        }

    private:
        QString m_status;
        bool m_active;
        int m_position;
        double m_score;
        int m_stepIndex;
        int m_stepCount;

        MsgReportAutoFocus(const QString& status, bool active, int position, double score, int stepIndex, int stepCount) :
            Message(),
            m_status(status),
            m_active(active),
            m_position(position),
            m_score(score),
            m_stepIndex(stepIndex),
            m_stepCount(stepCount)
        { }
    };

    // Sent periodically to update live status fields (camerastate, ccdtemperature)
    class MsgReportAlpacaStatus : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        int getCameraState() const { return m_cameraState; }
        double getCcdTemperature() const { return m_ccdTemperature; }
        bool isCcdTemperatureValid() const { return m_ccdTemperatureValid; }
        qint64 getCaptureTimeMs() const { return m_captureTimeMs; }
        const QString& getReceiveImageFormat() const { return m_receiveImageFormat; }
        int getLastErrorNumber() const { return m_lastErrorNumber; }
        const QString& getLastErrorMessage() const { return m_lastErrorMessage; }

        static MsgReportAlpacaStatus* create(int cameraState, double ccdTemperature, bool ccdTemperatureValid, qint64 captureTimeMs,
                                              const QString& receiveImageFormat,
                                              int lastErrorNumber, const QString& lastErrorMessage)
        {
            return new MsgReportAlpacaStatus(cameraState, ccdTemperature, ccdTemperatureValid, captureTimeMs,
                 receiveImageFormat, lastErrorNumber, lastErrorMessage);
        }

    private:
        int m_cameraState;
        double m_ccdTemperature;
        bool m_ccdTemperatureValid;
        qint64 m_captureTimeMs;
        QString m_receiveImageFormat;
        int m_lastErrorNumber;
        QString m_lastErrorMessage;

        MsgReportAlpacaStatus(int cameraState, double ccdTemperature, bool ccdTemperatureValid, qint64 captureTimeMs,
                               const QString& receiveImageFormat,
                               int lastErrorNumber, const QString& lastErrorMessage) :
            Message(),
            m_cameraState(cameraState),
            m_ccdTemperature(ccdTemperature),
            m_ccdTemperatureValid(ccdTemperatureValid),
            m_captureTimeMs(captureTimeMs),
            m_receiveImageFormat(receiveImageFormat),
            m_lastErrorNumber(lastErrorNumber),
            m_lastErrorMessage(lastErrorMessage)
        { }
    };

    CameraWorker();
    ~CameraWorker();

    void startWork();
    void stopWork();
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setMessageQueueToGUI(MessageQueue *messageQueue);
    void setMessageQueueToFeature(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }
    void setFramePreprocessor(CameraFramePreprocessor *framePreprocessor) { m_framePreprocessor = framePreprocessor; }
    void setPostProcessorInputMessageQueue(MessageQueue *messageQueue) { m_postProcessorInputMessageQueue = messageQueue; }
    void setRecorderInputMessageQueue(MessageQueue *messageQueue) { m_qtAudio.setRecordingMessageQueue(messageQueue); }

private:
    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_msgQueueToFeature;
    CameraFramePreprocessor *m_framePreprocessor;
    MessageQueue *m_postProcessorInputMessageQueue;
    QRecursiveMutex m_mutex;
    CameraSettings m_settings;
    AvailableDeviceHandler m_availableDeviceHandler;
    AvailableDeviceList m_availableDevices;
    QSet<QString> m_reportedFeatureErrorKeys;
    bool m_capturing;
    quint64 m_captureEpoch;
    QTimer m_captureTimer;
    QNetworkAccessManager *m_networkManager;
    CameraFinder *m_cameraFinder;
    int m_stackFrameIndex;
    int m_hdrExposureIndex;
    struct AutoExposureState {
        bool m_valid = false;
        double m_brightness = 0.0;
        double m_saturatedFraction = 0.0;
        int m_settleFramesRemaining = 0;
        int m_saturatedFrames = 0;
        int m_adjustDirection = 0;
        int m_adjustDirectionFrames = 0;
        quint64 m_debugFrameCounter = 0;
    } m_autoExposure;
    struct AutoFocusState {
        bool m_active = false;
        bool m_movePending = false;
        int m_originalPosition = 0;
        int m_bestPosition = 0;
        double m_bestScore = -1.0;
        int m_currentIndex = -1;
        int m_settleFramesRemaining = 0;
        QVector<int> m_positions;
    } m_autoFocus;
    CameraAlpacaController m_alpaca;
    CameraQtAudioController m_qtAudio;
    // Declared AFTER m_qtAudio so it destructs FIRST: the decode thread it owns can
    // submit audio to m_qtAudio during teardown, so the audio controller must
    // outlive the playback controller.
    CameraMediaPlaybackController m_playback;
    QTimer m_statusTimer;   // polls camerastate + ccdtemperature
    QObject *m_spectrumPipeSource; ///< Cached pointer to the DeviceAPI of the selected spectrum device
#ifdef ASICAMERA_FOUND
    CameraAsiController m_asi;
#endif

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void reportAvailableDevicesToGUI() const;
    void reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage);
    bool isHdrBracketingActive() const;
    void resetHdrBracketState();
    int currentStackBurstFrameCount() const;
    int currentStackBurstIndex() const;
    int currentHdrExposureCount() const;
    int currentHdrExposureIndex() const;
    double currentCaptureExposureTimeMs() const;
    void advanceStackBurstState();
    void advanceHdrBracketState();
    bool useStackIntervalCadence() const;
    int captureTimerIntervalMs() const;
    void scheduleNextCaptureAfterFrame();
    void scheduleNextCaptureAfterFailure();
    void populateFrameExposureMetadata(CameraPipelineFrame& frame) const;
    void maybeAdjustAutoExposureGain(const CameraPipelineFrame& frame);
    bool measureAutoExposureGain(const QImage& image, double& measuredBrightness, double& saturatedFraction) const;
    void reportAutoExposureGainToGUI(double measuredBrightness, double saturatedFraction) const;
    void startAutoFocus();
    void cancelAutoFocus(const QString& status = QString());
    void moveAutoFocusToNextPosition();
    void sampleAutoFocusFrame(const CameraPipelineFrame& frame);
    double measureAutoFocusScore(const QImage& image) const;
    void reportAutoFocusToGUI(const QString& status, bool active, int position, double score, int stepIndex, int stepCount) const;
    void startCapture();
    void stopCapture();
    QImage createPlaceholderFrame() const;
    void reportAlpacaCameraInfoToGUI(const CameraAlpacaController::CapabilitiesReport& report);
    void reportAlpacaStatusToGUI(int cameraState = -1, double ccdTemperature = NAN, bool ccdTemperatureValid = false);
    static const int m_alpacaStatusPollIntervalMs = 2000;
    static const int m_alpacaImageReadyPollIntervalMs = 100;

    void startControllerExposure();
    void abortControllerExposure();
    void checkControllerImageReady();
    void checkControllerCameraStateForImageReady();
    void fetchControllerImage();
    void applyControllerCameraParams();
    void pollControllerStatus();
    void disconnectControllerCamera(const CameraSettings& settings);
    void setControllerCameraConnected(bool connected, std::function<void()> continuation = {});
    void setControllerFocuserConnected(bool connected, std::function<void()> continuation = {});
    void moveControllerFocuser();
    void setControllerFilterWheelConnected(bool connected, std::function<void()> continuation = {});
    void queryControllerFilterWheelPosition(std::function<void(int)> continuation);
    void waitForControllerFilterWheelPosition(int retriesRemaining);
    void queryControllerFilterWheelInfo();
    void setControllerFilterWheelPosition();
    void bootstrapControllerCamera(std::function<void()> continuation = {});
    void resetAlpacaConnectionState();
    void resetAlpacaFocuserConnectionState();
    void resetAlpacaFilterWheelConnectionState();
#ifdef ASICAMERA_FOUND
    void asiQueryCameraCapabilities();
    bool asiOpenCamera();
    void asiCloseCamera();
    bool asiApplyCameraSettings();
    bool asiCaptureExposureFrame();
    void asiCaptureVideoFrame();
    void asiPollStatus();
    void asiCaptureTick();
    bool useAsiContinuousVideoCadence() const;
    void scheduleNextAsiVideoCapture(int delayMs = 0);
    void invalidateAsiSettings();
#endif

private slots:
    void handleInputMessages();
    void handleDeviceMessageQueue(MessageQueue* messageQueue);
    void captureTick();
    void statusTick();
    void onAvailableDevicesChanged(const QStringList& renameFrom, const QStringList& renameTo,
                                   const QStringList& removed, const QStringList& added);
    // Outbound media-playback reports/errors from CameraMediaPlaybackController.
    void onPlaybackReport(qint64 positionMs, qint64 durationMs, bool playing, bool open);
    void onPlaybackError(const QString& errorKey, const QString& title, const QString& errorMessage);
};

#endif // INCLUDE_FEATURE_CAMERAWORKER_H_
