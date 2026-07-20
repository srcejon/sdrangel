///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERATHERMALPROCESSOR_H_
#define INCLUDE_FEATURE_CAMERATHERMALPROCESSOR_H_

#include <memory>

#include <QObject>

#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"
#include "camerathermaldecoder.h"

class CameraFrameAligner;

class CameraThermalProcessor : public QObject
{
    Q_OBJECT
public:
    CameraThermalProcessor();
    ~CameraThermalProcessor() override;

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraFrameAligner *stage) { m_nextStage = stage; }

private:
    MessageQueue m_inputMessageQueue;
    CameraSettings m_settings;
    CameraFrameAligner *m_nextStage = nullptr;
    std::unique_ptr<CameraThermalDecoder> m_decoder;
    double m_smoothedLowC = 0.0;
    double m_smoothedHighC = 0.0;
    bool m_haveSmoothedRange = false;
    bool m_captureActive = false;
    quint64 m_captureEpoch = 0;
    bool m_rawDumpAttempted = false;
    QString m_lastRawSignature;
    CameraPipelineFramePtr m_lastThermalInput;

    bool handleMessage(const Message& message);
    void applySettings(const CameraSettings& settings, const QList<QString>& keys, bool force);
    bool processThermalFrame(CameraPipelineFrame& frame);
    QImage colorize(const cv::Mat& temperatureC, double lowC, double highC) const;
    void updateStatistics(CameraPipelineFrame& frame) const;
    void forward(const CameraPipelineFramePtr& frame);

private slots:
    void handleInputMessages();
};

#endif // INCLUDE_FEATURE_CAMERATHERMALPROCESSOR_H_
