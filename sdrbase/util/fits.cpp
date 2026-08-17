///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021-2022 Jon Beniston, M7RCE <jon@beniston.com>                //
// Copyright (C) 2022 Edouard Griffiths, F4EXB <f4exb06@gmail.com>               //
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
#include <cmath>

#include <QtGlobal>
#include <QDateTime>
#include <QFile>
#include <QRegularExpression>
#include <QDebug>
#include <QResource>

#include "fits.h"

namespace {

QByteArray fitsCard(const QString& keyword, const QString& value, const QString& comment = QString())
{
    QString card = keyword.leftJustified(8, QLatin1Char(' '));
    if (!value.isEmpty())
    {
        card += QStringLiteral("= ");
        card += value.rightJustified(20, QLatin1Char(' '));
        if (!comment.isEmpty()) {
            card += QStringLiteral(" / ") + comment;
        }
    }

    return card.leftJustified(80, QLatin1Char(' ')).left(80).toLatin1();
}

QString fitsStringValue(const QString& value)
{
    QString escaped = value;
    escaped.replace(QLatin1Char('\''), QStringLiteral("''"));
    return QStringLiteral("'%1'").arg(escaped).leftJustified(20, QLatin1Char(' '));
}

QString fitsVariantValue(const QVariant& value)
{
    switch (value.type())
    {
    case QVariant::Bool:
        return value.toBool() ? QStringLiteral("T") : QStringLiteral("F");
    case QVariant::Int:
    case QVariant::LongLong:
    case QVariant::UInt:
    case QVariant::ULongLong:
        return value.toString();
    case QVariant::Double:
        return QString::number(value.toDouble(), 'g', 15);
    case QVariant::DateTime:
        return fitsStringValue(value.toDateTime().toUTC().toString(Qt::ISODateWithMs));
    default:
        return fitsStringValue(value.toString());
    }
}

void appendPadding(QByteArray& data)
{
    const int remainder = data.size() % 2880;
    if (remainder != 0) {
        data.append(QByteArray(2880 - remainder, '\0'));
    }
}

bool isValidFitsKeyword(const QString& keyword)
{
    static const QRegularExpression re(QStringLiteral("^[A-Z0-9_-]{1,8}$"));
    return re.match(keyword).hasMatch();
}

QVariant parseFitsValue(const QString& field)
{
    QString value = field.trimmed();
    if (value.startsWith(QLatin1Char('\'')))
    {
        QString text;
        for (int i = 1; i < value.size(); ++i)
        {
            if (value.at(i) != QLatin1Char('\''))
            {
                text.append(value.at(i));
                continue;
            }
            if ((i + 1 < value.size()) && (value.at(i + 1) == QLatin1Char('\'')))
            {
                text.append(QLatin1Char('\''));
                ++i;
                continue;
            }
            break;
        }
        return text;
    }

    const int commentIndex = value.indexOf(QLatin1Char('/'));
    if (commentIndex >= 0) {
        value = value.left(commentIndex).trimmed();
    }
    if (value == QLatin1String("T")) {
        return true;
    }
    if (value == QLatin1String("F")) {
        return false;
    }

    bool ok = false;
    const qlonglong integerValue = value.toLongLong(&ok);
    if (ok) {
        return integerValue;
    }

    QString floatingValue = value;
    floatingValue.replace(QLatin1Char('D'), QLatin1Char('E'));
    floatingValue.replace(QLatin1Char('d'), QLatin1Char('E'));
    const double number = floatingValue.toDouble(&ok);
    return ok ? QVariant(number) : QVariant(value);
}

}

FITS::FITS(QString resourceName) :
    m_valid(false)
{
    int m_headerSize = 2880;
    qint64 m_fileSize;

    QResource m_res(resourceName);
    if (m_res.isValid())
    {
    #if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
        m_data = m_res.uncompressedData();
        m_fileSize = m_res.uncompressedSize();
    #else
        m_data = QByteArray::fromRawData((const char *)m_res.data(), m_res.size());
        if (m_res.isCompressed()) {
            m_data = qUncompress(m_data);
        }
        m_fileSize = m_res.size();
    #endif
    }
    else
    {
        QFile file(resourceName);

        if (!file.open(QIODevice::ReadOnly))
        {
            qWarning() << "FITS:" << resourceName << "is not a valid resource and could not be opened as a file";
            return;
        }

        m_data = file.readAll();
        m_fileSize = m_data.size();
    }

    int hLen = std::min((qint64)m_headerSize * 3, m_fileSize);   // Could possibly be bigger
    QByteArray headerBytes = m_data.left(hLen);
    QString header = QString::fromLatin1(headerBytes);
    QRegularExpression widthRE("NAXIS1 *= *([0-9]+)");
    QRegularExpression heightRE("NAXIS2 *= *([0-9]+)");
    QRegularExpression bitsPerPixelRE("BITPIX *= *(-?[0-9]+)");
    QRegularExpression bzeroRE("BZERO *= *([0-9]+)");
    QRegularExpression bscaleRE("BSCALE *= *(-?[0-9]+(.[0-9]+)?)");
    QRegularExpression buintRE("BUNIT *= *\\'([A-Z ]+)\\'");
    QRegularExpression cdelt1RE("CDELT1 *= *(-?[0-9]+(.[0-9]+)?)");
    QRegularExpression cdelt2RE("CDELT2 *= *(-?[0-9]+(.[0-9]+)?)");
    QRegularExpression endRE("END {77}");
    QRegularExpressionMatch match;

    match = widthRE.match(header);
    if (match.hasMatch())
        m_width = match.capturedTexts()[1].toInt();
    else
    {
        qWarning() << "FITS: NAXIS1 missing";
        return;
    }

    match = heightRE.match(header);
    if (match.hasMatch())
        m_height = match.capturedTexts()[1].toInt();
    else
    {
        qWarning() << "FITS: NAXIS2 missing";
        return;
    }

    match = bitsPerPixelRE.match(header);
    if (match.hasMatch())
        m_bitsPerPixel = match.capturedTexts()[1].toInt();
    else
    {
        qWarning() << "FITS: BITPIX missing";
        return;
    }

    m_bytesPerPixel = abs(m_bitsPerPixel)/8;
    match = bzeroRE.match(header);
    if (match.hasMatch())
        m_bzero = match.capturedTexts()[1].toInt();
    else
        m_bzero = 0;

    match = bscaleRE.match(header);
    if (match.hasMatch())
        m_bscale = match.capturedTexts()[1].toDouble();
    else
        m_bscale = 1.0;

    match = cdelt1RE.match(header);
    if (match.hasMatch())
        m_cdelta1 = match.capturedTexts()[1].toDouble();
    else
        m_cdelta1 = 0.0;

    match = cdelt2RE.match(header);
    if (match.hasMatch())
        m_cdelta2 = match.capturedTexts()[1].toDouble();
    else
        m_cdelta2 = 0.0;

    match = buintRE.match(header);
    if (match.hasMatch())
    {
        m_buint = match.capturedTexts()[1].trimmed();
        if (m_buint.contains("MILLI"))
            m_uintScale = 0.001f;
        else
            m_uintScale = 1.0f;
    }
    else
        m_uintScale = 1.0f;

    match = endRE.match(header);
    int endIdx = match.capturedStart(0);
    if (!match.hasMatch())
    {
        qWarning() << "FITS: END missing";
        return;
    }

    for (int cardOffset = 0; cardOffset < endIdx; cardOffset += 80)
    {
        const QString card = header.mid(cardOffset, 80);
        const QString keyword = card.left(8).trimmed();
        if (!keyword.isEmpty() && (card.size() > 9) && (card.at(8) == QLatin1Char('='))) {
            m_headers.insert(keyword, parseFitsValue(card.mid(10)));
        }
    }
    m_dataStart = ((endIdx + m_headerSize) / m_headerSize) * m_headerSize;
    m_valid = true;
}

bool FITS::saveImage(const QString& fileName,
                     const QByteArray& imageData,
                     int width,
                     int height,
                     int bitsPerPixel,
                     const QVariantMap& headers,
                     QString *errorMessage)
{
    return saveImage(fileName, imageData, width, height, bitsPerPixel, 1, headers, errorMessage);
}

bool FITS::saveImage(const QString& fileName,
                     const QByteArray& imageData,
                     int width,
                     int height,
                     int bitsPerPixel,
                     int channels,
                     const QVariantMap& headers,
                     QString *errorMessage)
{
    if ((width <= 0) || (height <= 0))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Invalid image dimensions");
        }
        return false;
    }

    if ((bitsPerPixel != 8) && (bitsPerPixel != 16))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Only 8-bit and 16-bit FITS image saving is supported");
        }
        return false;
    }

    if ((channels != 1) && (channels != 3))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Only 1-channel and 3-channel FITS image saving is supported");
        }
        return false;
    }

    const int bytesPerPixel = bitsPerPixel / 8;
    const qsizetype expectedSize = static_cast<qsizetype>(width) * static_cast<qsizetype>(height) * bytesPerPixel * channels;
    if (imageData.size() < expectedSize)
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Image data is smaller than the specified FITS dimensions");
        }
        return false;
    }

    QByteArray header;
    header += fitsCard(QStringLiteral("SIMPLE"), QStringLiteral("T"));
    header += fitsCard(QStringLiteral("BITPIX"), QString::number(bitsPerPixel));
    header += fitsCard(QStringLiteral("NAXIS"), QString::number(channels == 1 ? 2 : 3));
    header += fitsCard(QStringLiteral("NAXIS1"), QString::number(width));
    header += fitsCard(QStringLiteral("NAXIS2"), QString::number(height));
    if (channels > 1) {
        header += fitsCard(QStringLiteral("NAXIS3"), QString::number(channels));
    }
    if (bitsPerPixel == 16)
    {
        header += fitsCard(QStringLiteral("BZERO"), QStringLiteral("32768"));
        header += fitsCard(QStringLiteral("BSCALE"), QStringLiteral("1"));
    }

    for (auto it = headers.cbegin(); it != headers.cend(); ++it)
    {
        const QString keyword = it.key().trimmed().toUpper();
        if (keyword.isEmpty()
            || (keyword == QLatin1String("SIMPLE"))
            || (keyword == QLatin1String("BITPIX"))
            || (keyword == QLatin1String("NAXIS"))
            || (keyword == QLatin1String("NAXIS1"))
            || (keyword == QLatin1String("NAXIS2"))
            || (keyword == QLatin1String("NAXIS3"))
            || !isValidFitsKeyword(keyword))
        {
            continue;
        }
        header += fitsCard(keyword, fitsVariantValue(it.value()));
    }
    header += fitsCard(QStringLiteral("END"), QString());
    appendPadding(header);

    QByteArray data;
    data.reserve(static_cast<int>(expectedSize));
    const uchar *src = reinterpret_cast<const uchar*>(imageData.constData());
    const int rowBytes = width * bytesPerPixel;
    const qsizetype planeBytes = static_cast<qsizetype>(height) * rowBytes;

    if (bitsPerPixel == 8)
    {
        for (int channel = 0; channel < channels; ++channel)
        {
            const uchar *plane = src + (static_cast<qsizetype>(channel) * planeBytes);
            for (int y = height - 1; y >= 0; --y) {
                data.append(reinterpret_cast<const char*>(plane + (y * rowBytes)), rowBytes);
            }
        }
    }
    else
    {
        for (int channel = 0; channel < channels; ++channel)
        {
            const uchar *plane = src + (static_cast<qsizetype>(channel) * planeBytes);
            for (int y = height - 1; y >= 0; --y)
            {
                const uchar *row = plane + (y * rowBytes);
                for (int x = 0; x < rowBytes; x += 2)
                {
                    const quint16 unsignedValue = static_cast<quint16>(row[x] | (static_cast<quint16>(row[x + 1]) << 8));
                    const quint16 storedValue = static_cast<quint16>(static_cast<qint32>(unsignedValue) - 32768);
                    data.append(static_cast<char>((storedValue >> 8) & 0xff));
                    data.append(static_cast<char>(storedValue & 0xff));
                }
            }
        }
    }
    appendPadding(data);

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        if (errorMessage) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    if ((file.write(header) != header.size()) || (file.write(data) != data.size()))
    {
        if (errorMessage) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    return true;
}

float FITS::value(int x, int y) const
{
    int offset = m_dataStart + (m_height-1-y) * m_width * m_bytesPerPixel + x * m_bytesPerPixel;
    const uchar *data = (const uchar *)m_data.data();
    // Big-endian
    quint64 v = 0;
    for (int i = m_bytesPerPixel - 1; i >= 0; i--) {
        v += static_cast<quint64>(data[offset++]) << (i*8);
    }
    if (m_bitsPerPixel > 0)
    {
        // FITS BITPIX=8 is unsigned. Larger integer BITPIX values are signed,
        // with unsigned sensor data represented through BZERO/BSCALE.
        switch (m_bytesPerPixel)
        {
        case 1:
            return v * m_bscale + m_bzero;
        case 2:
            return static_cast<qint16>(static_cast<quint16>(v)) * m_bscale + m_bzero;
        case 4:
            return static_cast<qint32>(static_cast<quint32>(v)) * m_bscale + m_bzero;
        case 8:
            return static_cast<qint64>(v) * m_bscale + m_bzero;
        default:
            return v * m_bscale + m_bzero;
        }
    }
    else
    {
        // Type-punning via unions apparently undefined behaviour in C++
        uint32_t i = (uint32_t)v;
        float f;
        memcpy(&f, &i, sizeof(f));
        return f;
    }
}

float FITS::scaledValue(int x, int y) const
{
    float v = value(x, y);
    return v * m_uintScale;
}

int FITS::mod(int a, int b) const
{
    return a - b * floor(a/(double)b);
}

float FITS::scaledWrappedValue(int x, int y) const
{
    float v = value(mod(x, m_width), mod(y, m_height));
    return v * m_uintScale;
}
