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

#include <algorithm>

#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QMutableHashIterator>
#include <QPainter>
#include <QProcess>
#include <QSharedPointer>
#include <QTextDocument>
#include <QTextStream>
#include <QTimer>
#include <QVector>

#include "maincore.h"
#include "channel/channelwebapiutils.h"
#include "device/deviceset.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"
#include "util/profiler.h"
#include "camerapostprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgConfigureCameraPostProcessor, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSpectrumFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportSaveVideoState, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgCaptureActive, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgDownloadProgress, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgDownloadComplete, Message)

namespace {

QString substituteObjectClass(QString text, const QString& className)
{
    text.replace(QStringLiteral("${class}"), className);
    return text;
}

class ProtoReader
{
public:
    ProtoReader(const uchar *data, int size) :
        m_data(data),
        m_size(size),
        m_pos(0)
    {}

    bool atEnd() const { return m_pos >= m_size; }

    bool readVarint(quint64& value)
    {
        value = 0;
        int shift = 0;

        while (m_pos < m_size && shift < 64)
        {
            const quint8 byte = m_data[m_pos++];
            value |= static_cast<quint64>(byte & 0x7f) << shift;

            if ((byte & 0x80) == 0) {
                return true;
            }

            shift += 7;
        }

        return false;
    }

    bool readSubMessage(ProtoReader& subMessage)
    {
        quint64 length = 0;

        if (!readVarint(length) || (length > static_cast<quint64>(m_size - m_pos))) {
            return false;
        }

        subMessage = ProtoReader(m_data + m_pos, static_cast<int>(length));
        m_pos += static_cast<int>(length);
        return true;
    }

    bool skipField(int wireType)
    {
        switch (wireType)
        {
        case 0:
        {
            quint64 value = 0;
            return readVarint(value);
        }
        case 1:
            return skipBytes(8);
        case 2:
        {
            quint64 length = 0;
            return readVarint(length) && skipBytes(static_cast<int>(length));
        }
        case 5:
            return skipBytes(4);
        default:
            return false;
        }
    }

private:
    bool skipBytes(int count)
    {
        if ((count < 0) || (count > (m_size - m_pos))) {
            return false;
        }

        m_pos += count;
        return true;
    }

    const uchar *m_data;
    int m_size;
    int m_pos;
};

bool parseOnnxTensorShape(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 1) && (wireType == 2))
        {
            ProtoReader dimReader(nullptr, 0);

            if (!reader.readSubMessage(dimReader)) {
                return false;
            }

            int dimValue = 0;
            bool hasDimValue = false;

            while (!dimReader.atEnd())
            {
                quint64 dimTag = 0;

                if (!dimReader.readVarint(dimTag)) {
                    return false;
                }

                const int dimFieldNumber = static_cast<int>(dimTag >> 3);
                const int dimWireType = static_cast<int>(dimTag & 0x7);

                if ((dimFieldNumber == 1) && (dimWireType == 0))
                {
                    quint64 value = 0;

                    if (!dimReader.readVarint(value)) {
                        return false;
                    }

                    dimValue = static_cast<int>(value);
                    hasDimValue = true;
                }
                else if (!dimReader.skipField(dimWireType))
                {
                    return false;
                }
            }

            dims.append(hasDimValue ? dimValue : 0);
        }
        else if (!reader.skipField(wireType))
        {
            return false;
        }
    }

    return true;
}

bool parseOnnxTensorType(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 2) && (wireType == 2))
        {
            ProtoReader shapeReader(nullptr, 0);
            return reader.readSubMessage(shapeReader) && parseOnnxTensorShape(shapeReader, dims);
        }

        if (!reader.skipField(wireType)) {
            return false;
        }
    }

    return false;
}

bool parseOnnxValueInfo(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 2) && (wireType == 2))
        {
            ProtoReader typeReader(nullptr, 0);

            if (!reader.readSubMessage(typeReader)) {
                return false;
            }

            while (!typeReader.atEnd())
            {
                quint64 typeTag = 0;

                if (!typeReader.readVarint(typeTag)) {
                    return false;
                }

                const int typeFieldNumber = static_cast<int>(typeTag >> 3);
                const int typeWireType = static_cast<int>(typeTag & 0x7);

                if ((typeFieldNumber == 1) && (typeWireType == 2))
                {
                    ProtoReader tensorTypeReader(nullptr, 0);
                    return typeReader.readSubMessage(tensorTypeReader) && parseOnnxTensorType(tensorTypeReader, dims);
                }

                if (!typeReader.skipField(typeWireType)) {
                    return false;
                }
            }
        }
        else if (!reader.skipField(wireType))
        {
            return false;
        }
    }

    return false;
}

cv::Size readOnnxInputSize(const QString& modelPath)
{
    QFile modelFile(modelPath);

    if (!modelFile.open(QIODevice::ReadOnly)) {
        return cv::Size();
    }

    const QByteArray modelData = modelFile.readAll();
    ProtoReader modelReader(reinterpret_cast<const uchar *>(modelData.constData()), modelData.size());

    while (!modelReader.atEnd())
    {
        quint64 tag = 0;

        if (!modelReader.readVarint(tag)) {
            break;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 7) && (wireType == 2))
        {
            ProtoReader graphReader(nullptr, 0);

            if (!modelReader.readSubMessage(graphReader)) {
                break;
            }

            while (!graphReader.atEnd())
            {
                quint64 graphTag = 0;

                if (!graphReader.readVarint(graphTag)) {
                    return cv::Size();
                }

                const int graphFieldNumber = static_cast<int>(graphTag >> 3);
                const int graphWireType = static_cast<int>(graphTag & 0x7);

                if ((graphFieldNumber == 11) && (graphWireType == 2))
                {
                    ProtoReader inputReader(nullptr, 0);
                    QVector<int> dims;

                    if (!graphReader.readSubMessage(inputReader) || !parseOnnxValueInfo(inputReader, dims)) {
                        continue;
                    }

                    if (dims.size() >= 4)
                    {
                        const int width = dims.at(dims.size() - 1);
                        const int height = dims.at(dims.size() - 2);

                        if ((width > 0) && (height > 0)) {
                            return cv::Size(width, height);
                        }
                    }
                }
                else if (!graphReader.skipField(graphWireType))
                {
                    return cv::Size();
                }
            }
        }
        else if (!modelReader.skipField(wireType))
        {
            break;
        }
    }

    return cv::Size();
}

}

CameraPostProcessor::CameraPostProcessor() :
    m_msgQueueToGUI(nullptr),
    m_captureActive(false),
    m_imageSaved(false),
    m_yoloInputSize(640, 640),
    m_autoWhiteBalanceGains(1.0, 1.0, 1.0),
    m_autoWhiteBalanceInitialized(false),
#ifdef QT_TEXTTOSPEECH_FOUND
    m_speech(new QTextToSpeech(this)),
#endif
    m_dlm(this)
{
    connect(&m_dlm, &HttpDownloadManager::downloadComplete, this, &CameraPostProcessor::downloadComplete);
}

CameraPostProcessor::~CameraPostProcessor()
{
    disconnect(&m_dlm, &HttpDownloadManager::downloadComplete, this, &CameraPostProcessor::downloadComplete);
    stopWork();
    m_inputMessageQueue.clear();
}

void CameraPostProcessor::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraPostProcessor::handleInputMessages);
    handleInputMessages();
}

void CameraPostProcessor::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraPostProcessor::handleInputMessages);

    if (m_videoWriter.isOpened()) {
        m_videoWriter.release();
    }
}

void CameraPostProcessor::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
        // Post processing can be slow, so drop image frames if the queue gets too large
        if (m_inputMessageQueue.size() > 20)
        {
            while (m_inputMessageQueue.size() > 0)
            {
                message = m_inputMessageQueue.pop();
                if (MsgProcessFrame::match(*message))
                {
                    qDebug() << "CameraPostProcessor: Dropping frame to catch up";
                    delete message;
                } else
                {
                    if (handleMessage(*message)) {
                        delete message;
                    }
                }
            }
        }
    }
}

bool CameraPostProcessor::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraPostProcessor::match(cmd))
    {
        MsgConfigureCameraPostProcessor& cfg = (MsgConfigureCameraPostProcessor&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        MsgProcessFrame& frameMsg = (MsgProcessFrame&) cmd;
        processNewFrame(frameMsg.getImage());
        return true;
    }
    else if (MsgSpectrumFrame::match(cmd))
    {
        MsgSpectrumFrame& frameMsg = (MsgSpectrumFrame&) cmd;
        m_spectrumViewImage = frameMsg.getImage();
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        MsgCaptureActive& activeMsg = (MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();

        if (m_captureActive)
        {
            m_imageSaved = false;
            m_previousRawFrame = QImage();
            m_autoWhiteBalanceGains = cv::Vec3d(1.0, 1.0, 1.0);
            m_autoWhiteBalanceInitialized = false;
        }
        else if (m_videoWriter.isOpened())
        {
            m_videoWriter.release();
        }

        return true;
    }

    return false;
}

void CameraPostProcessor::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraPostProcessor::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    static const QStringList kPostProcessingKeys = {
        "postProcessWhiteBalanceMode",
        "postProcessWhiteBalanceRedGain",
        "postProcessWhiteBalanceGreenGain",
        "postProcessWhiteBalanceBlueGain",
        "saturation", "gamma", "gaussianBlur", "medianBlur", "sharpen", "sobelEdge", "flipX", "flipY",
        "brightness", "contrast", "invertColors", "overlayDateTime", "dateTimeColor",
        "dateTimeFormat", "dateTimePosX", "dateTimePosY",
        "overlayText", "overlayTextString", "overlayTextColor",
        "overlayTextFontFamily", "overlayTextFontScale", "overlayTextPosX", "overlayTextPosY",
        "diffMask", "diffThreshold", "dilationSize", "diffMaskHistoryFrames", "diffMaskCloseSize", "overlayFontFamily", "overlayFontScale",
        "motionDetect", "motionBoxColor", "minContourArea",
        "overlaySpectrum", "spectrumDevice", "spectrumOffsetX", "spectrumOffsetY", "spectrumScale",
        "yoloEnabled", "yoloModelPath", "yoloLabelsPath", "yoloConfThreshold", "yoloNmsThreshold", "yoloBoxColor"
    };
    const bool postProcessChanged = force || std::any_of(kPostProcessingKeys.cbegin(), kPostProcessingKeys.cend(),
        [&settingsKeys](const QString& k) { return settingsKeys.contains(k); });
    const bool sourceChanged = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged)
    {
        m_lastRawFrame = QImage();
        m_previousRawFrame = QImage();
        m_diffMaskHistory.clear();
        m_autoWhiteBalanceGains = cv::Vec3d(1.0, 1.0, 1.0);
        m_autoWhiteBalanceInitialized = false;
    }

    if (force
        || settingsKeys.contains("postProcessWhiteBalanceMode")
        || settingsKeys.contains("postProcessWhiteBalanceRedGain")
        || settingsKeys.contains("postProcessWhiteBalanceGreenGain")
        || settingsKeys.contains("postProcessWhiteBalanceBlueGain"))
    {
        m_autoWhiteBalanceGains = cv::Vec3d(1.0, 1.0, 1.0);
        m_autoWhiteBalanceInitialized = false;
    }

    if (force
        || settingsKeys.contains("diffMask")
        || settingsKeys.contains("diffThreshold")
        || settingsKeys.contains("dilationSize")
        || settingsKeys.contains("diffMaskHistoryFrames")
        || settingsKeys.contains("diffMaskCloseSize"))
    {
        m_diffMaskHistory.clear();
    }

    if ((force && !m_settings.m_diffMask) || (settingsKeys.contains("diffMask") && !m_settings.m_diffMask)) {
        m_previousRawFrame = QImage();
    }

    if ((force && !m_settings.m_motionDetect) || (settingsKeys.contains("motionDetect") && !m_settings.m_motionDetect)) {
        m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractorMOG2>();
    }

    if (settingsKeys.contains("yoloModelPath") || (force && m_yoloLoadedModelPath != m_settings.m_yoloModelPath))
    {
        m_yoloNet = cv::dnn::Net();
        m_yoloLoadedModelPath.clear();
    }

    if (settingsKeys.contains("yoloLabelsPath") || (force && m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath))
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();
        // Download even if YOLO not enabled, for use in the GUI
        if (m_settings.m_yoloLabelsPath.startsWith("http://") || m_settings.m_yoloLabelsPath.startsWith("https://")) {
            download(m_settings.m_yoloLabelsPath, "onnx");
        }
    }

    if ((force && !m_settings.m_yoloEnabled)
        || settingsKeys.contains("yoloEnabled")
        || settingsKeys.contains("yoloLabelsPath")
        || settingsKeys.contains("objectDeviceSettings"))
    {
        m_detectedObjectClasses.clear();
        m_pendingDisappearDeadlines.clear();
    }

    if (force || settingsKeys.contains("spectrumDevice")) {
        m_spectrumViewImage = QImage();
    }

    if (settingsKeys.contains("saveVideo") || settingsKeys.contains("videoFileName"))
    {
        if (m_videoWriter.isOpened()) {
            m_videoWriter.release();
        }
    }

    if (m_settings.m_yoloEnabled && (m_settings.m_yoloModelPath.startsWith("http://") || m_settings.m_yoloModelPath.startsWith("https://"))) {
        download(m_settings.m_yoloModelPath, "onnx");
    }

    if (postProcessChanged && !m_lastRawFrame.isNull()) {
        const QImage processed = applyPostProcessing(m_lastRawFrame);
        reportFrameToGUI(processed);
    }
}

void CameraPostProcessor::processNewFrame(const QImage& image)
{
    if (image.isNull()) {
        return;
    }

    m_captureDateTime = QDateTime::currentDateTime();
    const QImage processed = applyPostProcessing(image);

    m_previousRawFrame = m_lastRawFrame;
    m_lastRawFrame = image;

    reportFrameToGUI(processed);

    if (m_captureActive && m_settings.m_saveImage && !m_imageSaved && !m_settings.m_imageFileName.isEmpty())
    {
        QFileInfo fileInfo(m_settings.m_imageFileName);
        QString filename = fileInfo.path() + "/" + fileInfo.baseName() + "." + QDateTime::currentDateTimeUtc().toString("yyyy-MM-ddTHH_mm_ss_zzz") + "." + fileInfo.suffix();
        qDebug() << "CameraPostProcessor: Saving image to" << filename;
        const QImage& frameToSave = m_settings.m_videoPostProcess ? processed : image;
        frameToSave.save(filename);
        m_imageSaved = true;
    }

    if (m_captureActive && m_settings.m_saveVideo && !m_settings.m_videoFileName.isEmpty())
    {
        if (!m_videoWriter.isOpened())
        {
            QFileInfo fileInfo(m_settings.m_videoFileName);
            QString filename = fileInfo.path() + "/" + fileInfo.baseName() + "." + QDateTime::currentDateTimeUtc().toString("yyyy-MM-ddTHH_mm_ss_zzz") + "." + fileInfo.suffix();
            qDebug() << "CameraPostProcessor: Saving video to" << filename;

            const QImage& frameForSize = m_settings.m_videoPostProcess ? processed : image;
            const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            m_videoWriter.open(
                filename.toStdString(),
                fourcc,
                m_settings.getCaptureFrameRate(),
                cv::Size(frameForSize.width(), frameForSize.height()),
                true);
        }

        if (m_videoWriter.isOpened())
        {
            const QImage& frameToWrite = m_settings.m_videoPostProcess ? processed : image;
            const QImage rgb = frameToWrite.convertToFormat(QImage::Format_RGB888);
            cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                        const_cast<uchar*>(rgb.bits()),
                        static_cast<size_t>(rgb.bytesPerLine()));
            cv::Mat bgrMat;
            cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
            m_videoWriter.write(bgrMat);
        }
    }
}

void CameraPostProcessor::reportFrameToGUI(const QImage& image)
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportFrame::create(image));
    }
}

QImage CameraPostProcessor::applyPostProcessing(const QImage& input)
{
    PROFILER_START();

    const bool needsSpectrumOverlay = m_settings.m_overlaySpectrum && !m_spectrumViewImage.isNull();
    const bool needsWhiteBalance = m_settings.m_postProcessWhiteBalanceMode != 0;
    const bool needsSaturation = std::abs(m_settings.m_saturation - 1.0) > 1e-4;
    const bool needsGamma = std::abs(m_settings.m_gamma - 1.0) > 1e-4;
    const bool needsGaussianBlur = m_settings.m_gaussianBlur > 0;
    const bool needsMedianBlur = m_settings.m_medianBlur > 0;
    const bool needsSharpen = m_settings.m_sharpen > 1e-4;
    const bool needsSobelEdge = m_settings.m_sobelEdge > 1e-4;
    const bool needsFlip = m_settings.m_flipX || m_settings.m_flipY;
    const bool needsBrightContrast = (m_settings.m_brightness != 0.0 || m_settings.m_contrast != 1.0);
    QTextDocument overlayTextDocument;
    overlayTextDocument.setHtml(m_settings.m_overlayTextString);
    const bool needsTextOverlay = m_settings.m_overlayText && !overlayTextDocument.toPlainText().trimmed().isEmpty();
    const bool needsAny = needsWhiteBalance
        || needsSaturation
        || needsGamma
        || needsGaussianBlur
        || needsMedianBlur
        || needsSharpen
        || needsSobelEdge
        || needsFlip
        || needsBrightContrast
        || m_settings.m_invertColors
        || m_settings.m_overlayDateTime
        || needsTextOverlay
        || (m_settings.m_diffMask && !m_previousRawFrame.isNull())
        || m_settings.m_motionDetect
        || (m_settings.m_yoloEnabled && !m_settings.m_yoloModelPath.isEmpty())
        || needsSpectrumOverlay;

    if (!needsAny) {
        return input;
    }

    const QImage rgb = input.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    if (needsWhiteBalance)
    {
        cv::Vec3d gains(
            m_settings.m_postProcessWhiteBalanceBlueGain,
            m_settings.m_postProcessWhiteBalanceGreenGain,
            m_settings.m_postProcessWhiteBalanceRedGain);

        if (m_settings.m_postProcessWhiteBalanceMode == 1)
        {
            const cv::Scalar means = cv::mean(bgrMat);
            const double blueMean = std::max(1.0, means[0]);
            const double greenMean = std::max(1.0, means[1]);
            const double redMean = std::max(1.0, means[2]);
            const double targetMean = (blueMean + greenMean + redMean) / 3.0;

            cv::Vec3d targetGains(
                qBound(0.25, targetMean / blueMean, 4.0),
                qBound(0.25, targetMean / greenMean, 4.0),
                qBound(0.25, targetMean / redMean, 4.0));

            if (!m_autoWhiteBalanceInitialized)
            {
                m_autoWhiteBalanceGains = targetGains;
                m_autoWhiteBalanceInitialized = true;
            }
            else
            {
                static constexpr double kAutoWhiteBalanceSmoothing = 0.2;
                m_autoWhiteBalanceGains =
                    (1.0 - kAutoWhiteBalanceSmoothing) * m_autoWhiteBalanceGains
                    + kAutoWhiteBalanceSmoothing * targetGains;
            }

            gains = m_autoWhiteBalanceGains;
        }

        std::vector<cv::Mat> channels;
        cv::split(bgrMat, channels);
        channels[0].convertTo(channels[0], -1, gains[0], 0.0);
        channels[1].convertTo(channels[1], -1, gains[1], 0.0);
        channels[2].convertTo(channels[2], -1, gains[2], 0.0);
        cv::merge(channels, bgrMat);
    }

    if (needsSaturation)
    {
        cv::Mat hsvMat;
        cv::cvtColor(bgrMat, hsvMat, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> hsvChannels;
        cv::split(hsvMat, hsvChannels);
        hsvChannels[1].convertTo(hsvChannels[1], -1, m_settings.m_saturation, 0.0);
        cv::merge(hsvChannels, hsvMat);
        cv::cvtColor(hsvMat, bgrMat, cv::COLOR_HSV2BGR);
    }

    if (needsGamma)
    {
        cv::Mat lut(1, 256, CV_8U);
        uchar* lutData = lut.ptr<uchar>();
        for (int i = 0; i < 256; ++i) {
            lutData[i] = static_cast<uchar>(qBound(0, static_cast<int>(std::pow(i / 255.0, m_settings.m_gamma) * 255.0 + 0.5), 255));
        }
        cv::LUT(bgrMat, lut, bgrMat);
    }

    if (needsGaussianBlur)
    {
        const int kernelSize = 2 * m_settings.m_gaussianBlur + 1;
        cv::GaussianBlur(bgrMat, bgrMat, cv::Size(kernelSize, kernelSize), 0.0);
    }

    if (needsMedianBlur)
    {
        const int kernelSize = 2 * m_settings.m_medianBlur + 1;
        cv::medianBlur(bgrMat, bgrMat, kernelSize);
    }

    if (needsSharpen)
    {
        cv::Mat blurred;
        cv::GaussianBlur(bgrMat, blurred, cv::Size(0, 0), 1.0);
        cv::addWeighted(bgrMat, 1.0 + m_settings.m_sharpen, blurred, -m_settings.m_sharpen, 0.0, bgrMat);
    }

    if (needsSobelEdge)
    {
        cv::Mat grayMat;
        cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);

        cv::Mat gradX;
        cv::Mat gradY;
        cv::Sobel(grayMat, gradX, CV_16S, 1, 0, 3);
        cv::Sobel(grayMat, gradY, CV_16S, 0, 1, 3);

        cv::Mat absGradX;
        cv::Mat absGradY;
        cv::convertScaleAbs(gradX, absGradX);
        cv::convertScaleAbs(gradY, absGradY);

        cv::Mat edgesGray;
        cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0.0, edgesGray);

        cv::Mat edgesBgr;
        cv::cvtColor(edgesGray, edgesBgr, cv::COLOR_GRAY2BGR);
        cv::addWeighted(bgrMat, 1.0, edgesBgr, m_settings.m_sobelEdge, 0.0, bgrMat);
    }

    if (needsFlip)
    {
        const int flipCode = m_settings.m_flipX && m_settings.m_flipY ? -1 : (m_settings.m_flipX ? 1 : 0);
        cv::flip(bgrMat, bgrMat, flipCode);
    }

    if (needsBrightContrast)
    {
        cv::Mat adjusted;
        cv::convertScaleAbs(bgrMat, adjusted, m_settings.m_contrast, m_settings.m_brightness);
        bgrMat = adjusted;
    }

    if (m_settings.m_invertColors) {
        cv::bitwise_not(bgrMat, bgrMat);
    }

    if (m_settings.m_diffMask && !m_previousRawFrame.isNull()
        && m_previousRawFrame.width() == input.width()
        && m_previousRawFrame.height() == input.height())
    {
        const QImage prevRgb = m_previousRawFrame.convertToFormat(QImage::Format_RGB888);
        cv::Mat prevMat(prevRgb.height(), prevRgb.width(), CV_8UC3,
                        const_cast<uchar*>(prevRgb.bits()),
                        static_cast<size_t>(prevRgb.bytesPerLine()));
        cv::Mat prevBgr;
        cv::cvtColor(prevMat, prevBgr, cv::COLOR_RGB2BGR);

        cv::Mat gray, prevGray, diff, mask;
        cv::cvtColor(bgrMat, gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(prevBgr, prevGray, cv::COLOR_BGR2GRAY);
        cv::absdiff(gray, prevGray, diff);
        cv::threshold(diff, mask, m_settings.m_diffThreshold, 255, cv::THRESH_BINARY);

        cv::Mat dilationKernel;
        if (m_settings.m_dilationSize > 0)
        {
            const int ksize = 2 * m_settings.m_dilationSize + 1;
            dilationKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
            cv::dilate(mask, mask, dilationKernel);
        }

        if (!m_diffMaskHistory.empty() &&
            (m_diffMaskHistory.front().size() != mask.size() || m_diffMaskHistory.front().type() != mask.type()))
        {
            m_diffMaskHistory.clear();
        }

        m_diffMaskHistory.push_back(mask.clone());

        const size_t historyFrames = static_cast<size_t>(std::max(1, m_settings.m_diffMaskHistoryFrames));
        while (m_diffMaskHistory.size() > historyFrames) {
            m_diffMaskHistory.pop_front();
        }

        cv::Mat combinedMask = m_diffMaskHistory.front().clone();
        for (size_t i = 1; i < m_diffMaskHistory.size(); ++i) {
            cv::bitwise_or(combinedMask, m_diffMaskHistory[i], combinedMask);
        }
        if (m_settings.m_diffMaskCloseSize > 0) {
            const int closeKsize = 2 * m_settings.m_diffMaskCloseSize + 1;
            const cv::Mat closeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closeKsize, closeKsize));
            cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, closeKernel);
        }

        cv::Mat result = cv::Mat::zeros(bgrMat.size(), bgrMat.type());
        cv::bitwise_and(bgrMat, bgrMat, result, combinedMask);
        bgrMat = result;
    }

    if (m_settings.m_motionDetect)
    {
        if (!m_bgSubtractor) {
            m_bgSubtractor = cv::createBackgroundSubtractorMOG2();
        }

        cv::Mat fgMask;
        m_bgSubtractor->apply(bgrMat, fgMask);
        cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        const QColor& bc = m_settings.m_motionBoxColor;
        const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
        for (const auto& contour : contours)
        {
            if (cv::contourArea(contour) >= static_cast<double>(m_settings.m_minContourArea)) {
                cv::rectangle(bgrMat, cv::boundingRect(contour), boxColor, 2);
            }
        }
    }

    if (m_settings.m_yoloEnabled && !m_settings.m_yoloModelPath.isEmpty()) {
        runYoloDetections(bgrMat);
    }

    if (needsSpectrumOverlay)
    {
        QImage specSrc = m_spectrumViewImage;
        if (qAbs(m_settings.m_spectrumScale - 1.0) > 1e-4)
        {
            const int sw = static_cast<int>(specSrc.width() * m_settings.m_spectrumScale);
            const int sh = static_cast<int>(specSrc.height() * m_settings.m_spectrumScale);
            if (sw > 0 && sh > 0) {
                specSrc = specSrc.scaled(sw, sh, Qt::KeepAspectRatio, Qt::SmoothTransformation);
            }
        }

        const QImage specRgb = specSrc.convertToFormat(QImage::Format_RGBA8888);
        const int dstW = bgrMat.cols;
        const int dstH = bgrMat.rows;
        const int ox = m_settings.m_spectrumOffsetX;
        const int oy = m_settings.m_spectrumOffsetY;
        const int sw = specRgb.width();
        const int sh = specRgb.height();

        const int srcX0 = std::max(0, -ox);
        const int srcY0 = std::max(0, -oy);
        const int srcX1 = std::min(sw, dstW - ox);
        const int srcY1 = std::min(sh, dstH - oy);

        for (int sy = srcY0; sy < srcY1; ++sy)
        {
            const uchar* srcRow = specRgb.constScanLine(sy);
            const int dy = oy + sy;
            uchar* dstRow = bgrMat.ptr<uchar>(dy);

            for (int sx = srcX0; sx < srcX1; ++sx)
            {
                const int srcPx = sx * 4;
                const uchar alpha = srcRow[srcPx + 3];
                if (alpha == 0) {
                    continue;
                }
                const int dx = (ox + sx) * 3;
                if (alpha == 255)
                {
                    dstRow[dx] = srcRow[srcPx + 2];
                    dstRow[dx + 1] = srcRow[srcPx + 1];
                    dstRow[dx + 2] = srcRow[srcPx];
                }
                else
                {
                    const int a = alpha;
                    const int invA = 255 - a;
                    dstRow[dx] = static_cast<uchar>((srcRow[srcPx + 2] * a + dstRow[dx] * invA) / 255);
                    dstRow[dx + 1] = static_cast<uchar>((srcRow[srcPx + 1] * a + dstRow[dx + 1] * invA) / 255);
                    dstRow[dx + 2] = static_cast<uchar>((srcRow[srcPx] * a + dstRow[dx + 2] * invA) / 255);
                }
            }
        }
    }

    cv::cvtColor(bgrMat, bgrMat, cv::COLOR_BGR2RGB);
    const QImage rawResult(bgrMat.data, bgrMat.cols, bgrMat.rows,
                           static_cast<qsizetype>(bgrMat.step[0]),
                           QImage::Format_RGB888);
    QImage result = rawResult.copy();

    if (m_settings.m_overlayDateTime)
    {
        const QString fmt = m_settings.m_dateTimeFormat.isEmpty()
                            ? QStringLiteral("yyyy-MM-dd hh:mm:ss")
                            : m_settings.m_dateTimeFormat;
        const QString text = m_captureDateTime.toString(fmt);
        QFont font;
        if (!m_settings.m_overlayFontFamily.isEmpty()) {
            font.setFamily(m_settings.m_overlayFontFamily);
        }
        font.setPointSizeF(m_settings.m_overlayFontScale);
        const QFontMetrics fm(font);
        QPainter painter(&result);
        painter.setRenderHint(QPainter::TextAntialiasing);
        painter.setFont(font);
        painter.setPen(m_settings.m_dateTimeColor);
        const int x = m_settings.m_dateTimePosX;
        const int y = m_settings.m_dateTimePosY + fm.ascent();
        painter.drawText(x, y, text);
    }

    if (needsTextOverlay)
    {
        QFont font;
        if (!m_settings.m_overlayTextFontFamily.isEmpty()) {
            font.setFamily(m_settings.m_overlayTextFontFamily);
        }
        font.setPointSizeF(m_settings.m_overlayTextFontScale);
        overlayTextDocument.setDefaultFont(font);
        overlayTextDocument.setDefaultStyleSheet(QStringLiteral("* { color: %1; }").arg(m_settings.m_overlayTextColor.name()));
        overlayTextDocument.setHtml(QString("<div>%1</div>").arg(m_settings.m_overlayTextString));

        const int x = std::max(0, m_settings.m_overlayTextPosX);
        const qreal maxTextWidth = std::max(1, result.width() - x);
        overlayTextDocument.setTextWidth(maxTextWidth);

        QPainter painter(&result);
        painter.setRenderHint(QPainter::TextAntialiasing);
        painter.save();
        painter.translate(x, m_settings.m_overlayTextPosY);
        overlayTextDocument.drawContents(&painter);
        painter.restore();
    }

    PROFILER_STOP("CameraPostProcessor::applyPostProcessing");
    return result;
}

void CameraPostProcessor::runYoloDetections(cv::Mat& bgrMat)
{
    PROFILER_START();

    const QDateTime detectionTime = m_captureDateTime.isValid() ? m_captureDateTime : QDateTime::currentDateTime();

    if (m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath)
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();

        if (!m_settings.m_yoloLabelsPath.isEmpty() && !(m_settings.m_yoloLabelsPath.startsWith("http://") || m_settings.m_yoloLabelsPath.startsWith("https://")))
        {
            QFile f(m_settings.m_yoloLabelsPath);
            if (f.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QTextStream ts(&f);
                while (!ts.atEnd())
                {
                    const QString line = ts.readLine().trimmed();
                    if (!line.isEmpty()) {
                        m_yoloLabels.append(line);
                    }
                }
                m_yoloLoadedLabelsPath = m_settings.m_yoloLabelsPath;
            }
            else
            {
                qWarning() << "CameraPostProcessor::runYoloDetections: cannot open labels file:" << m_settings.m_yoloLabelsPath;
            }
        }
    }

    if (m_yoloLoadedModelPath != m_settings.m_yoloModelPath)
    {
        m_yoloNet = cv::dnn::Net();
        m_yoloInputSize = cv::Size(640, 640);
        m_yoloLoadedModelPath.clear();

        if (!m_settings.m_yoloModelPath.isEmpty() && !(m_settings.m_yoloModelPath.startsWith("http://") || m_settings.m_yoloModelPath.startsWith("https://")))
        {
            try
            {
                QString localFile = CameraSettings::urlToFilename(m_settings.m_yoloModelPath, "onnx");

                m_yoloNet = cv::dnn::readNetFromONNX(localFile.toStdString());

                const cv::Size modelInputSize = readOnnxInputSize(localFile);

                if ((modelInputSize.width > 0) && (modelInputSize.height > 0))
                {
                    m_yoloInputSize = modelInputSize;
                }
                else
                {
                    qWarning() << "CameraPostProcessor::runYoloDetections: unable to read model input size, using fallback 640x640 for"
                                << localFile;
                }

                m_yoloLoadedModelPath = m_settings.m_yoloModelPath;
                qDebug() << "CameraPostProcessor::runYoloDetections: loaded model" << localFile
                            << "with input size" << m_yoloInputSize.width << "x" << m_yoloInputSize.height;
            }
            catch (const cv::Exception& e)
            {
                qWarning() << "CameraPostProcessor::runYoloDetections: failed to load model:" << e.what();
                return;
            }
        }
        else
        {
            return;
        }
    }

    if (m_yoloNet.empty()) {
        return;
    }

    // Don't bother with OpenCL
    // Execution time in ms from Profiler
    // Model | CPU  | OPENCL_FP16 | CUDA | CUDA_FP16
    // 26n   | 60   | 270         | 22   |
    // 26s   | 119  |             | 54   |
    // 26m   | 267  | 720         | 60   |
    // 26l   | 370  |             | 70   |
    // 26x   | 550  | 1482        | 82   |
    if (m_settings.m_yoloDnnTarget == CameraSettings::CUDA)
    {
        m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else if (m_settings.m_yoloDnnTarget == CameraSettings::CUDA_FP16)
    {
        m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    const float scaleX = static_cast<float>(bgrMat.cols) / m_yoloInputSize.width;
    const float scaleY = static_cast<float>(bgrMat.rows) / m_yoloInputSize.height;

    cv::Mat blob;
    cv::dnn::blobFromImage(bgrMat, blob, 1.0 / 255.0, m_yoloInputSize, cv::Scalar(), true, false);
    m_yoloNet.setInput(blob);

    std::vector<cv::Mat> outputs;
    try
    {
        m_yoloNet.forward(outputs, m_yoloNet.getUnconnectedOutLayersNames());
    }
    catch (const cv::Exception& e)
    {
        qWarning() << "CameraPostProcessor::runYoloDetections: inference failed:" << e.what();
        return;
    }
    m_yoloNet.setInput(blob);

    if (outputs.empty()) {
        processObjectDetections(QSet<QString>(), detectionTime);
        return;
    }

    cv::Mat det = outputs[0];
    if (det.dims == 3) {
        det = det.reshape(1, det.size[1]);
    }

    const float confThresh = static_cast<float>(m_settings.m_yoloConfThreshold);
    const float nmsThresh = static_cast<float>(m_settings.m_yoloNmsThreshold);
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classIds;
    QSet<QString> currentDetectedClasses;
    const bool isV8Style = (det.rows < det.cols);

    if (isV8Style)
    {
        const int numAnchors = det.cols;
        const int numClasses = det.rows - 4;
        if (numClasses <= 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            float bestScore = 0.0f;
            int bestClass = 0;
            for (int c = 0; c < numClasses; ++c)
            {
                const float s = det.at<float>(4 + c, a);
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = det.at<float>(0, a) * scaleX;
            const float cy = det.at<float>(1, a) * scaleY;
            const float w = det.at<float>(2, a) * scaleX;
            const float h = det.at<float>(3, a) * scaleY;

            boxes.push_back(cv::Rect(static_cast<int>(cx - w / 2.0f), static_cast<int>(cy - h / 2.0f), static_cast<int>(w), static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }
    else
    {
        const int numAnchors = det.rows;
        const int numClasses = det.cols - 5;
        if (numClasses < 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            const float objectness = det.at<float>(a, 4);
            if (objectness < confThresh) {
                continue;
            }

            float bestScore = 0.0f;
            int bestClass = 0;
            for (int c = 0; c < numClasses; ++c)
            {
                const float s = objectness * det.at<float>(a, 5 + c);
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = det.at<float>(a, 0) * scaleX;
            const float cy = det.at<float>(a, 1) * scaleY;
            const float w = det.at<float>(a, 2) * scaleX;
            const float h = det.at<float>(a, 3) * scaleY;

            boxes.push_back(cv::Rect(static_cast<int>(cx - w / 2.0f), static_cast<int>(cy - h / 2.0f), static_cast<int>(w), static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, confThresh, nmsThresh, indices);

    const QColor& bc = m_settings.m_yoloBoxColor;
    const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
    const cv::Scalar textBg(0, 0, 0);

    for (int idx : indices)
    {
        const cv::Rect& box = boxes[idx];
        cv::rectangle(bgrMat, box, boxColor, 2);

        QString label;
        if (!m_yoloLabels.isEmpty() && classIds[idx] < m_yoloLabels.size()) {
            label = m_yoloLabels[classIds[idx]];
        } else {
            label = QStringLiteral("cls%1").arg(classIds[idx]);
        }
        currentDetectedClasses.insert(label);
        label += QStringLiteral(" %1%").arg(static_cast<int>(scores[idx] * 100.0f + 0.5f));

        const std::string labelStd = label.toStdString();
        int baseLine = 0;
        const cv::Size textSize = cv::getTextSize(labelStd, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        const int labelY = std::max(box.y, textSize.height + 2);
        cv::rectangle(bgrMat,
                      cv::Point(box.x, labelY - textSize.height - 2),
                      cv::Point(box.x + textSize.width, labelY + baseLine),
                      textBg, cv::FILLED);
        cv::putText(bgrMat, labelStd, cv::Point(box.x, labelY), cv::FONT_HERSHEY_SIMPLEX, 0.5, boxColor, 1, cv::LINE_AA);
    }

    PROFILER_STOP("YOLO");
    processObjectDetections(currentDetectedClasses, detectionTime);
}

void CameraPostProcessor::processObjectDetections(const QSet<QString>& currentDetectedClasses, const QDateTime& now)
{
    for (const QString& className : currentDetectedClasses)
    {
        m_pendingDisappearDeadlines.remove(className);

        if (!m_detectedObjectClasses.contains(className))
        {
            m_detectedObjectClasses.insert(className);
            applyObjectDetectedSettings(className);
        }
    }

    for (const QString& className : m_detectedObjectClasses)
    {
        if (!currentDetectedClasses.contains(className) && !m_pendingDisappearDeadlines.contains(className)) {
            m_pendingDisappearDeadlines.insert(className, now.addMSecs(static_cast<qint64>(m_settings.m_yoloDisappearDebounce * 1000.0)));
        }
    }

    QMutableHashIterator<QString, QDateTime> it(m_pendingDisappearDeadlines);
    while (it.hasNext())
    {
        it.next();

        if (currentDetectedClasses.contains(it.key())) {
            it.remove();
            continue;
        }

        if (it.value() <= now)
        {
            m_detectedObjectClasses.remove(it.key());
            applyObjectDisappearedSettings(it.key());
            it.remove();
        }
    }
}

bool CameraPostProcessor::shouldRecordVideoForDetectedObjects() const
{
    for (const QString& className : m_detectedObjectClasses)
    {
        QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
        if (deviceSettingsList == nullptr) {
            continue;
        }

        for (CameraSettings::ObjectDeviceSettings *devSettings : *deviceSettingsList)
        {
            if (devSettings && devSettings->m_recordVideo) {
                return true;
            }
        }
    }

    return false;
}

void CameraPostProcessor::setVideoRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveVideo == enabled) {
        return;
    }

    m_settings.m_saveVideo = enabled;

    if (!enabled && m_videoWriter.isOpened()) {
        m_videoWriter.release();
    }

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportSaveVideoState::create(enabled));
    }
}

void CameraPostProcessor::executeCommand(const QString& command, const QString& className)
{
    if (command.isEmpty()) {
        return;
    }

#if QT_CONFIG(process)
    const QString cmd = substituteObjectClass(command, className);
    QStringList allArgs = QProcess::splitCommand(cmd);

    if (allArgs.isEmpty()) {
        return;
    }

    qDebug() << "CameraPostProcessor::executeCommand: Executing:" << allArgs;
    const QString program = allArgs.takeFirst();
    QProcess::startDetached(program, allArgs);
#else
    qWarning() << "CameraPostProcessor::executeCommand: QProcess not supported. Can't run:" << command;
    (void) className;
#endif
}

void CameraPostProcessor::saySpeech(const QString& speech, const QString& className)
{
    if (speech.isEmpty()) {
        return;
    }

    const QString expandedSpeech = substituteObjectClass(speech, className);

#ifdef QT_TEXTTOSPEECH_FOUND
    m_speech->say(expandedSpeech);
#else
    qWarning() << "CameraPostProcessor::saySpeech: TextToSpeech not supported. Unable to say" << expandedSpeech;
#endif
}

void CameraPostProcessor::applyObjectDetectedSettings(const QString& className)
{
    if (!m_settings.m_objectDeviceSettings.contains(className)) {
        return;
    }

    QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
    if (deviceSettingsList == nullptr) {
        return;
    }

    MainCore *mainCore = MainCore::instance();
    const MainSettings& mainSettings = mainCore->getSettings();
    const std::vector<DeviceSet*>& deviceSets = mainCore->getDeviceSets();

    for (int i = 0; i < deviceSettingsList->size(); ++i)
    {
        CameraSettings::ObjectDeviceSettings *devSettings = deviceSettingsList->at(i);
        if (devSettings == nullptr) {
            continue;
        }

        if (devSettings->m_deviceSetIndex < 0 || devSettings->m_deviceSetIndex >= static_cast<int>(deviceSets.size()))
        {
            qWarning() << "CameraPostProcessor::applyObjectDetectedSettings: device set at"
                       << devSettings->m_deviceSetIndex << "does not exist";
            continue;
        }

        if (!devSettings->m_presetGroup.isEmpty())
        {
            const DeviceSet *deviceSet = deviceSets[devSettings->m_deviceSetIndex];
            QString presetType;
            if (deviceSet->m_deviceSourceEngine != nullptr) {
                presetType = "R";
            } else if (deviceSet->m_deviceSinkEngine != nullptr) {
                presetType = "T";
            } else if (deviceSet->m_deviceMIMOEngine != nullptr) {
                presetType = "M";
            }

            const Preset *preset = mainSettings.getPreset(
                devSettings->m_presetGroup,
                devSettings->m_presetFrequency,
                devSettings->m_presetDescription,
                presetType);

            if (preset != nullptr)
            {
                qDebug() << "CameraPostProcessor::applyObjectDetectedSettings: loading preset"
                         << preset->getDescription() << "for class" << className
                         << "to device set" << devSettings->m_deviceSetIndex;
                mainCore->getMainMessageQueue()->push(
                    MainCore::MsgLoadPreset::create(preset, devSettings->m_deviceSetIndex));
            }
            else
            {
                qWarning() << "CameraPostProcessor::applyObjectDetectedSettings: unable to get preset"
                           << devSettings->m_presetGroup
                           << devSettings->m_presetFrequency
                           << devSettings->m_presetDescription;
            }
        }

        if (devSettings->m_recordVideo) {
            setVideoRecordingEnabled(true);
        }
    }

    QTimer::singleShot(1000, this, [this, deviceSettingsList, className]()
    {
        for (int i = 0; i < deviceSettingsList->size(); ++i)
        {
            CameraSettings::ObjectDeviceSettings *devSettings = deviceSettingsList->at(i);
            if (devSettings == nullptr) {
                continue;
            }

            if (devSettings->m_startOnDetect) {
                ChannelWebAPIUtils::run(devSettings->m_deviceSetIndex);
            }

            if (devSettings->m_startStopFileSink) {
                ChannelWebAPIUtils::startStopFileSinks(devSettings->m_deviceSetIndex, true);
            }

            if (!devSettings->m_detectCommand.isEmpty()) {
                executeCommand(devSettings->m_detectCommand, className);
            }

            if (!devSettings->m_detectSpeech.isEmpty()) {
                saySpeech(devSettings->m_detectSpeech, className);
            }
        }
    });
}

void CameraPostProcessor::applyObjectDisappearedSettings(const QString& className)
{
    if (!m_settings.m_objectDeviceSettings.contains(className)) {
        return;
    }

    QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
    if (deviceSettingsList == nullptr) {
        return;
    }

    for (int i = 0; i < deviceSettingsList->size(); ++i)
    {
        CameraSettings::ObjectDeviceSettings *devSettings = deviceSettingsList->at(i);
        if (devSettings == nullptr) {
            continue;
        }

        if (devSettings->m_startStopFileSink) {
            ChannelWebAPIUtils::startStopFileSinks(devSettings->m_deviceSetIndex, false);
        }

        if (devSettings->m_stopOnDisappear) {
            ChannelWebAPIUtils::stop(devSettings->m_deviceSetIndex);
        }

        if (!devSettings->m_disappearCommand.isEmpty()) {
            executeCommand(devSettings->m_disappearCommand, className);
        }

        if (!devSettings->m_disappearSpeech.isEmpty()) {
            saySpeech(devSettings->m_disappearSpeech, className);
        }
    }

    if (!shouldRecordVideoForDetectedObjects()) {
        setVideoRecordingEnabled(false);
    }
}

void CameraPostProcessor::download(const QString& filename, const QString& destSubDir)
{
    QDir downloadDir = QDir(HttpDownloadManager::downloadDir());
    QString destDirPath = downloadDir.path() + "/" + destSubDir ;
    QDir destDir(destDirPath);

    if (!destDir.exists())
    {
        if (!downloadDir.mkdir(destSubDir)) {
            qWarning() << "Failed to make directory" << (downloadDir.path() + "/" + destSubDir);
        }
    }
    if (destDir.exists())
    {
        QUrl url(filename);
        QString localFilename = CameraSettings::urlToFilename(filename, destSubDir);

        if (!QFileInfo::exists(localFilename))
        {
            if (!m_pendingDownloads.contains(localFilename))
            {
                qDebug() << "Downloading from" << url << "to" << localFilename;
                m_pendingDownloads.append(localFilename);
                QNetworkReply *reply = m_dlm.download(url, localFilename);
                connect(reply, &QNetworkReply::downloadProgress, this, [this, filename, localFilename](qint64 bytesRead, qint64 totalBytes) {
                    if (m_msgQueueToGUI) {
                        m_msgQueueToGUI->push(MsgDownloadProgress::create(filename, localFilename, bytesRead, totalBytes));
                    }
                });
            }
        }
        else
        {
            // File already downloaded
            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgDownloadComplete::create(filename, localFilename, true, ""));
            }
        }
    }
}

void CameraPostProcessor::downloadComplete(const QString &filename, bool success, const QString &url, const QString &errorMessage)
{
    (void) success;
    (void) url;
    (void) errorMessage;

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgDownloadComplete::create(url, filename, success, errorMessage));
    }

    m_completedDownloads.append(filename);
    if (m_completedDownloads == m_pendingDownloads)
    {
        m_pendingDownloads.clear();
        m_completedDownloads.clear();
    }
}
