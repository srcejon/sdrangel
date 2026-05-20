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

#include "cameraalpacacontroller.h"

#include <algorithm>
#include <cstring>
#include <limits>

#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaEnum>
#include <QNetworkReply>
#include <QRandomGenerator>
#include <QtEndian>

CameraAlpacaController::CameraAlpacaController() :
    m_frameRequestPending(false),
    m_clientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_clientTransactionId(1),
    m_sensorType(0),
    m_cameraSizeX(0),
    m_cameraSizeY(0),
    m_bayerOffsetX(0),
    m_bayerOffsetY(0),
    m_imageBytesSupported(true),
    m_lastErrorNumber(0),
    m_lastErrorMessage(),
    m_lastReceiveImageFormat(),
    m_connected(false),
    m_connectionPending(false),
    m_focuserConnected(false),
    m_focuserConnectionPending(false),
    m_filterWheelConnected(false),
    m_filterWheelConnectionPending(false),
    m_bootstrapPending(false),
    m_paramsInitialized(false),
    m_exposureSeenActive(false),
    m_lastBinX(0),
    m_lastBinY(0),
    m_lastNumX(0),
    m_lastNumY(0),
    m_lastEffectiveNumX(-1),
    m_lastEffectiveNumY(-1),
    m_lastStartX(0),
    m_lastStartY(0),
    m_lastGain(-1),
    m_lastOffset(-1),
    m_lastReadoutMode(0),
    m_exposureMinMs(0.001),
    m_exposureMaxMs(60000.0),
    m_lastCaptureTimeMs(-1)
{
}

void CameraAlpacaController::resetConnectionState()
{
    m_connected = false;
    m_connectionPending = false;
    m_pendingConnectedContinuations.clear();
    m_bootstrapPending = false;
    m_pendingBootstrapContinuations.clear();
    setLastError(0, QString());
}

void CameraAlpacaController::resetFocuserConnectionState()
{
    m_focuserConnected = false;
    m_focuserConnectionPending = false;
    m_pendingFocuserConnectedContinuations.clear();
}

void CameraAlpacaController::resetFilterWheelConnectionState()
{
    m_filterWheelConnected = false;
    m_filterWheelConnectionPending = false;
    m_pendingFilterWheelConnectedContinuations.clear();
}

void CameraAlpacaController::setLastError(int errorNumber, const QString& errorMessage)
{
    m_lastErrorNumber = errorNumber;
    m_lastErrorMessage = errorMessage;
}

void CameraAlpacaController::resetCaptureState()
{
    m_lastCaptureTimeMs = -1;
    m_captureTimer.invalidate();
    m_paramsInitialized = false;
}

bool CameraAlpacaController::parseErrorPayload(const QByteArray& payload, int& errorNumber, QString& errorMessage)
{
    errorNumber = 0;
    errorMessage.clear();

    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (!doc.isObject()) {
        return false;
    }

    const QJsonObject root = doc.object();

    if (!root.contains(QStringLiteral("ErrorNumber")) && !root.contains(QStringLiteral("ErrorMessage"))) {
        return false;
    }

    errorNumber = root.value(QStringLiteral("ErrorNumber")).toInt(0);
    errorMessage = root.value(QStringLiteral("ErrorMessage")).toString();
    return true;
}

bool CameraAlpacaController::parseImageBytesError(const QByteArray& payload, int& errorNumber)
{
    errorNumber = 0;

    if (payload.size() < 8) {
        return false;
    }

    const char *data = payload.constData();
    const qint32 metadataVersion = qFromLittleEndian<qint32>(data);

    if (metadataVersion != 1) {
        return false;
    }

    errorNumber = qFromLittleEndian<qint32>(data + 4);
    return true;
}

bool CameraAlpacaController::isOptionalCapabilityPath(const QString& path)
{
    const QString prop = path.section('/', -1).toLower();
    return (prop == QStringLiteral("gains"))
        || (prop == QStringLiteral("gainmin"))
        || (prop == QStringLiteral("gainmax"))
        || (prop == QStringLiteral("offsets"))
        || (prop == QStringLiteral("offsetmin"))
        || (prop == QStringLiteral("offsetmax"))
        || (prop == QStringLiteral("readoutmodes"))
        || (prop == QStringLiteral("ccdtemperature"));
}

QString CameraAlpacaController::baseUrl(const CameraSettings& settings)
{
    return QString("http://%1:%2")
        .arg(settings.m_alpacaHost)
        .arg(settings.m_alpacaPort);
}

QString CameraAlpacaController::focuserBaseUrl(const CameraSettings& settings)
{
    return QString("http://%1:%2")
        .arg(settings.m_alpacaFocuserHost)
        .arg(settings.m_alpacaFocuserPort);
}

QString CameraAlpacaController::filterWheelBaseUrl(const CameraSettings& settings)
{
    return QString("http://%1:%2")
        .arg(settings.m_alpacaFilterWheelHost)
        .arg(settings.m_alpacaFilterWheelPort);
}

QString CameraAlpacaController::transportError(QNetworkReply *reply)
{
    if (reply->error() == QNetworkReply::NoError) {
        return "";
    } else {
        return QString("%1 %2").arg(QMetaEnum::fromType<QNetworkReply::NetworkError>().valueToKey(reply->error())).arg(reply->errorString());
    }
}

QImage CameraAlpacaController::renderGrayscaleRaw(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit)
{
    int minValue = std::numeric_limits<int>::max();
    int maxValue = std::numeric_limits<int>::min();

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            minValue = std::min(minValue, raw[x][y]);
            maxValue = std::max(maxValue, raw[x][y]);
        }
    }

    const int range = maxValue - minValue;
    if (use16Bit)
    {
        const int uniformGray = (range == 0) ? (minValue > 0 ? 32768 : 0) : 0;
        QImage image(width, height, QImage::Format_Grayscale16);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const int value = (range > 0)
                    ? qBound(0, static_cast<int>(((raw[x][y] - minValue) * 65535.0) / range), 65535)
                    : uniformGray;
                reinterpret_cast<quint16*>(image.scanLine(y))[x] = static_cast<quint16>(value);
            }
        }
        return image;
    }

    const int uniformGray = (range == 0) ? (minValue > 0 ? 128 : 0) : 0;
    QImage image(width, height, QImage::Format_Grayscale8);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            const int value = (range > 0)
                ? qBound(0, static_cast<int>(((raw[x][y] - minValue) * 255.0) / range), 255)
                : uniformGray;
            image.scanLine(y)[x] = static_cast<uchar>(value);
        }
    }

    return image;
}

QImage CameraAlpacaController::parseImageArray(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(payload);
    if (!doc.isObject()) {
        return fallbackImage;
    }

    const QJsonObject root = doc.object();

    const int errorNumber = root.value("ErrorNumber").toInt(0);
    if (errorNumber != 0) {
        qDebug() << "CameraAlpacaController::parseImageArray: Alpaca error" << errorNumber
                 << root.value("ErrorMessage").toString();
        return fallbackImage;
    }

    const int rank = root.value("Rank").toInt(0);
    const QJsonArray value = root.value("Value").toArray();

    if (value.isEmpty()) {
        return fallbackImage;
    }

    // Alpaca imagearray layout:
    //   Rank 2 (monochrome or Bayer): Value[column][row]
    //   Rank 3 (colour):              Value[plane][column][row], plane 0=R, 1=G, 2=B
    if (rank == 2)
    {
        const int width = value.size();
        if (width == 0) {
            return fallbackImage;
        }
        const QJsonArray firstCol = value[0].toArray();
        const int height = firstCol.size();
        if (height == 0) {
            return fallbackImage;
        }

        // First pass: find minimum and maximum pixel values for black-level correction and scaling.
        int minVal = std::numeric_limits<int>::max();
        int maxVal = std::numeric_limits<int>::min();
        for (const QJsonValue& col : value) {
            for (const QJsonValue& pix : col.toArray()) {
                const int v = pix.toInt(0);
                if (v < minVal) { minVal = v; }
                if (v > maxVal) { maxVal = v; }
            }
        }
        const bool use16Bit = (minVal < 0) || (maxVal > 255);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageArray rank2 %1")
                .arg(use16Bit ? QStringLiteral("16-bit") : QStringLiteral("8-bit"));
        }

        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            const QJsonArray col = value[x].toArray();
            for (int y = 0; y < height; ++y) {
                raw[x][y] = col[y].toInt(0);
            }
        }

        // Bayer demosaicing for sensorType 2 (RGGB), 3 (CMYG), 4 (CMYG2), 5 (LRGB)
        // sensorType 0 = Monochrome, 1 = Colour (handled by rank 3 normally)
        return renderRawPixelArray(raw, width, height, use16Bit, bayerPattern);
    }
    else if (rank == 3)
    {
        if (value.size() < 3) {
            return fallbackImage;
        }
        const QJsonArray planeR = value[0].toArray();
        const QJsonArray planeG = value[1].toArray();
        const QJsonArray planeB = value[2].toArray();

        const int width = planeR.size();
        if (width == 0) {
            return fallbackImage;
        }
        const int height = planeR[0].toArray().size();
        if (height == 0) {
            return fallbackImage;
        }

        // First pass: find minimum and maximum pixel values for black-level correction and scaling.
        int minVal = std::numeric_limits<int>::max();
        int maxVal = std::numeric_limits<int>::min();
        for (const QJsonArray* plane : {&planeR, &planeG, &planeB}) {
            for (const QJsonValue& col : *plane) {
                for (const QJsonValue& pix : col.toArray()) {
                    const int v = pix.toInt(0);
                    if (v < minVal) { minVal = v; }
                    if (v > maxVal) { maxVal = v; }
                }
            }
        }
        const int range3 = maxVal - minVal;
        const bool use16Bit = (minVal < 0) || (maxVal > 255);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageArray rank3 %1")
                .arg(use16Bit ? QStringLiteral("16-bit") : QStringLiteral("8-bit"));
        }
        const double scale = (range3 > 0) ? ((use16Bit ? 65535.0 : 255.0) / range3) : 0.0;
        const int uniformGray3 = (range3 == 0) ? (minVal > 0 ? (use16Bit ? 32768 : 128) : 0) : 0;

        QImage image(width, height, use16Bit ? QImage::Format_RGBA64 : QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            const QJsonArray colR = planeR[x].toArray();
            const QJsonArray colG = planeG[x].toArray();
            const QJsonArray colB = planeB[x].toArray();
            for (int y = 0; y < height; ++y) {
                const int maxComponent = use16Bit ? 65535 : 255;
                const int r = (range3 > 0) ? qBound(0, static_cast<int>((colR[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;
                const int g = (range3 > 0) ? qBound(0, static_cast<int>((colG[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;
                const int b = (range3 > 0) ? qBound(0, static_cast<int>((colB[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;

                if (use16Bit) {
                    reinterpret_cast<QRgba64*>(image.scanLine(y))[x] = qRgba64(r, g, b, 65535);
                } else {
                    reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
                }
            }
        }
        return image;
    }

    return fallbackImage;
}
QImage CameraAlpacaController::renderRawPixelArray(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    int minValue = std::numeric_limits<int>::max();
    int maxValue = std::numeric_limits<int>::min();

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            minValue = std::min(minValue, raw[x][y]);
            maxValue = std::max(maxValue, raw[x][y]);
        }
    }

    if (m_sensorType == 2)
    {
        const int phaseX = ((m_bayerOffsetX % 2) + 2) % 2;
        const int phaseY = ((m_bayerOffsetY % 2) + 2) % 2;

        if (bayerPattern)
        {
            if ((phaseX == 0) && (phaseY == 0)) {
                *bayerPattern = CameraPipelineFrame::BayerRGGB;
            } else if ((phaseX == 1) && (phaseY == 0)) {
                *bayerPattern = CameraPipelineFrame::BayerGRBG;
            } else if ((phaseX == 0) && (phaseY == 1)) {
                *bayerPattern = CameraPipelineFrame::BayerGBRG;
            } else {
                *bayerPattern = CameraPipelineFrame::BayerBGGR;
            }
        }

        return renderGrayscaleRaw(raw, width, height, use16Bit);
    }

    // Monochrome (sensorType 0 or 1 returning rank 2, or unsupported Bayer types 3-5)
    return renderGrayscaleRaw(raw, width, height, use16Bit);
}

// Alpaca ImageBytes binary format (ASCOM Alpaca spec):
// https://ascom-standards.org/api/?urls.primaryName=ASCOM%20Alpaca%20Device%20API#/Camera/get__device_type___device_number__imagearray
//
//   Byte  0- 3: MetaDataVersion  (int32 LE) â€” must be 1
//   Byte  4- 7: ErrorNumber      (int32 LE) â€” 0 = success
//   Byte  8-11: ClientTransactionID (int32 LE)
//   Byte 12-15: ServerTransactionID (int32 LE)
//   Byte 16-19: DataStart        (int32 LE) â€” byte offset to pixel data, typically 44
//   Byte 20-23: ImageElementType (int32 LE) â€” original ADU element type (ASCOM ImageArrayElementTypes enum)
//   Byte 24-27: TransmissionElementType (int32 LE) â€” wire type (same enum)
//   Byte 28-31: Rank             (int32 LE) â€” 2 or 3
//   Byte 32-35: Dimension1       (int32 LE) â€” rank2: width; rank3: number of planes
//   Byte 36-39: Dimension2       (int32 LE) â€” rank2: height; rank3: width
//   Byte 40-43: Dimension3       (int32 LE) â€” rank2: unused (0); rank3: height
//
// ASCOM ImageArrayElementTypes enum:
//   1=Int16, 2=Int32, 3=Double, 4=Single, 5=UInt64, 6=Byte, 7=Int64, 8=UInt16, 9=UInt32
//
// Pixel data is column-major within each rank-2 plane: pixel[x][y] at index (x*height + y)
// For rank 3: pixel[plane][x][y] at index (plane*width*height + x*height + y)
QImage CameraAlpacaController::parseImageBytes(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

    static constexpr int    kHeaderSize         = 44;
    // ASCOM ImageArrayElementTypes enum values
    static constexpr qint32 kElementTypeInt16   = 1;
    static constexpr qint32 kElementTypeInt32   = 2;
    static constexpr qint32 kElementTypeDouble  = 3;
    static constexpr qint32 kElementTypeSingle  = 4;
    static constexpr qint32 kElementTypeByte    = 6;
    static constexpr qint32 kElementTypeUInt16  = 8;

    if (payload.size() < kHeaderSize) {
        qDebug() << "CameraAlpacaController::parseImageBytes: payload too small" << payload.size();
        return fallbackImage;
    }

    const char* hdr = payload.constData();

    const qint32 metadataVersion  = qFromLittleEndian<qint32>(hdr + 0);
    const qint32 errorNumber      = qFromLittleEndian<qint32>(hdr + 4);
    // clientTransactionID        = qFromLittleEndian<qint32>(hdr + 8);  // informational
    // serverTransactionID        = qFromLittleEndian<qint32>(hdr + 12); // informational
    const qint32 dataStart        = qFromLittleEndian<qint32>(hdr + 16);
    const qint32 imageElementType = qFromLittleEndian<qint32>(hdr + 20);
    const qint32 transmissionType = qFromLittleEndian<qint32>(hdr + 24);
    const qint32 rank             = qFromLittleEndian<qint32>(hdr + 28);
    const qint32 dim1             = qFromLittleEndian<qint32>(hdr + 32);
    const qint32 dim2             = qFromLittleEndian<qint32>(hdr + 36);
    const qint32 dim3             = qFromLittleEndian<qint32>(hdr + 40);

    if (metadataVersion != 1) {
        qDebug() << "CameraAlpacaController::parseImageBytes: unsupported MetaDataVersion" << metadataVersion;
        return fallbackImage;
    }

    if (errorNumber != 0) {
        qDebug() << "CameraAlpacaController::parseImageBytes: Alpaca error" << errorNumber;
        return fallbackImage;
    }

    if (dataStart < kHeaderSize || dataStart > payload.size()) {
        qDebug() << "CameraAlpacaController::parseImageBytes: invalid DataStart" << dataStart;
        return fallbackImage;
    }

    int elementSize = 0;
    switch (transmissionType) {
        case kElementTypeInt16:  elementSize = 2; break;
        case kElementTypeInt32:  elementSize = 4; break;
        case kElementTypeDouble: elementSize = 8; break;
        case kElementTypeSingle: elementSize = 4; break;
        case kElementTypeByte:   elementSize = 1; break;
        case kElementTypeUInt16: elementSize = 2; break;
      default:
            qDebug() << "CameraAlpacaController::parseImageBytes: unknown TransmissionElementType" << transmissionType;
            return fallbackImage;
    }

    auto elementTypeName = [](qint32 elementType) -> QString {
        switch (elementType) {
        case kElementTypeInt16: return QStringLiteral("Int16");
        case kElementTypeInt32: return QStringLiteral("Int32");
        case kElementTypeDouble: return QStringLiteral("Double");
        case kElementTypeSingle: return QStringLiteral("Single");
        case kElementTypeByte: return QStringLiteral("Byte");
        case kElementTypeUInt16: return QStringLiteral("UInt16");
        default: return QStringLiteral("Unknown");
        }
    };

    const char*   pixels         = payload.constData() + dataStart;
    const qsizetype pixelDataLen = payload.size() - dataStart;

    // Read one pixel value as double (any supported element type) at a given byte offset.
    // Using double avoids overflow during the min/max scan and subsequent scaling.
    auto readPixelAsDouble = [&](qsizetype byteOffset) -> double {
        if (byteOffset + elementSize > pixelDataLen) {
            return 0.0;
        }
        const char* p = pixels + byteOffset;
        switch (transmissionType) {
            case kElementTypeInt16:
                return static_cast<double>(qFromLittleEndian<qint16>(p));
            case kElementTypeInt32:
                return static_cast<double>(qFromLittleEndian<qint32>(p));
            case kElementTypeDouble: {
                // Use memcpy + assume little-endian host (x86/x64/ARM â€” all Qt-supported platforms).
                // Qt does not provide qFromLittleEndian<double> for all versions.
                double v;
                std::memcpy(&v, p, sizeof(v));
                return v;
            }
            case kElementTypeSingle: {
                float v;
                std::memcpy(&v, p, sizeof(v));
                return static_cast<double>(v);
            }
            case kElementTypeByte:
                return static_cast<double>(static_cast<quint8>(*p));
            case kElementTypeUInt16:
                return static_cast<double>(qFromLittleEndian<quint16>(p));
            default:
                return 0.0;
        }
    };

    if (rank == 2)
    {
        const int width  = static_cast<int>(dim1);
        const int height = static_cast<int>(dim2);
        if (width <= 0 || height <= 0) {
            return fallbackImage;
        }
        const qsizetype required = static_cast<qsizetype>(width) * height * elementSize;
        if (pixelDataLen < required) {
            qDebug() << "CameraAlpacaController::parseImageBytes: insufficient pixel data for rank 2:"
                     << pixelDataLen << "<" << required;
            return fallbackImage;
        }

        // First pass: min/max for black-level correction and linear scaling to 8-bit
        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const double v = readPixelAsDouble(static_cast<qsizetype>(x * height + y) * elementSize);
                if (v < minVal) { minVal = v; }
                if (v > maxVal) { maxVal = v; }
            }
        }
        const bool use16Bit = (imageElementType == kElementTypeInt16)
                           || (imageElementType == kElementTypeUInt16)
                           || (transmissionType == kElementTypeInt16)
                           || (transmissionType == kElementTypeUInt16)
                           || (minVal < 0.0)
                           || (maxVal > 255.0);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageBytes rank2 %1/%2%3")
                .arg(elementTypeName(imageElementType),
                     elementTypeName(transmissionType),
                     use16Bit ? QStringLiteral(" 16-bit") : QString());
        }

        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const double v = readPixelAsDouble(static_cast<qsizetype>(x * height + y) * elementSize);
                raw[x][y] = qRound(v);
            }
        }

        return renderRawPixelArray(raw, width, height, use16Bit, bayerPattern);
    }
    else if (rank == 3)
    {
        const int planes = static_cast<int>(dim1);
        const int width  = static_cast<int>(dim2);
        const int height = static_cast<int>(dim3);
        if (planes < 3 || width <= 0 || height <= 0) {
            return fallbackImage;
        }
        const qsizetype required = static_cast<qsizetype>(planes) * width * height * elementSize;
        if (pixelDataLen < required) {
            qDebug() << "CameraAlpacaController::parseImageBytes: insufficient pixel data for rank 3:"
                     << pixelDataLen << "<" << required;
            return fallbackImage;
        }

        auto pixelAt = [&](int plane, int x, int y) -> double {
            return readPixelAsDouble(static_cast<qsizetype>(plane * width * height + x * height + y) * elementSize);
        };

        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();
        for (int p = 0; p < 3; ++p) {
            for (int x = 0; x < width; ++x) {
                for (int y = 0; y < height; ++y) {
                    const double v = pixelAt(p, x, y);
                    if (v < minVal) { minVal = v; }
                    if (v > maxVal) { maxVal = v; }
                }
            }
        }
        const double range3 = maxVal - minVal;
        const bool use16Bit = (imageElementType == kElementTypeInt16)
                           || (imageElementType == kElementTypeUInt16)
                           || (transmissionType == kElementTypeInt16)
                           || (transmissionType == kElementTypeUInt16)
                           || (minVal < 0.0)
                           || (maxVal > 255.0);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageBytes rank3 %1/%2%3")
                .arg(elementTypeName(imageElementType),
                     elementTypeName(transmissionType),
                     use16Bit ? QStringLiteral(" 16-bit") : QString());
        }
        const double scale = (range3 > 0.0) ? ((use16Bit ? 65535.0 : 255.0) / range3) : 0.0;
        const int uniformGray3 = (range3 == 0.0) ? (minVal > 0.0 ? (use16Bit ? 32768 : 128) : 0) : 0;

        QImage image(width, height, use16Bit ? QImage::Format_RGBA64 : QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const int maxComponent = use16Bit ? 65535 : 255;
                const int r = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(0, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;
                const int g = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(1, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;
                const int b = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(2, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;

                if (use16Bit) {
                    reinterpret_cast<QRgba64*>(image.scanLine(y))[x] = qRgba64(r, g, b, 65535);
                } else {
                    reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
                }
            }
        }
        return image;
    }

    return fallbackImage;
}
