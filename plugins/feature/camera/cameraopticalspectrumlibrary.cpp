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
#include <cmath>
#include <cstring>
#include <limits>

#include <QStringList>
#include <QUrl>

#include <zlib.h>

#include "cameraopticalspectrumlibrary.h"

namespace {

// The Pickles library file names encode class, subclass and luminosity class, but not
// uniformly: some cover a range of subclasses (b12iii = B1-B2 III, b57v = B5-B7 V) and
// m10iii is subclass 10 rather than a 1-0 range, so the values are tabulated explicitly
// rather than parsed out of the key. Ranges are entered as their midpoint.
// Metallicity: 'r' prefix = metal rich, 'w' prefix = metal weak.
struct TemplateDef
{
    const char* m_key;
    char m_class;
    double m_subClass;
    const char* m_lum;
    int m_metallicity;
};

const TemplateDef kTemplateDefs[] = {
    // O
    {"o5v", 'O', 5, "V", 0}, {"o8iii", 'O', 8, "III", 0}, {"o9v", 'O', 9, "V", 0},
    // B
    {"b0i", 'B', 0, "I", 0}, {"b0v", 'B', 0, "V", 0}, {"b1i", 'B', 1, "I", 0},
    {"b1v", 'B', 1, "V", 0}, {"b12iii", 'B', 1.5, "III", 0}, {"b2ii", 'B', 2, "II", 0},
    {"b2iv", 'B', 2, "IV", 0}, {"b3i", 'B', 3, "I", 0}, {"b3iii", 'B', 3, "III", 0},
    {"b3v", 'B', 3, "V", 0}, {"b5i", 'B', 5, "I", 0}, {"b5ii", 'B', 5, "II", 0},
    {"b5iii", 'B', 5, "III", 0}, {"b57v", 'B', 6, "V", 0}, {"b6iv", 'B', 6, "IV", 0},
    {"b8i", 'B', 8, "I", 0}, {"b8v", 'B', 8, "V", 0}, {"b9iii", 'B', 9, "III", 0},
    {"b9v", 'B', 9, "V", 0},
    // A
    {"a0i", 'A', 0, "I", 0}, {"a0iii", 'A', 0, "III", 0}, {"a0iv", 'A', 0, "IV", 0},
    {"a0v", 'A', 0, "V", 0}, {"a2i", 'A', 2, "I", 0}, {"a2v", 'A', 2, "V", 0},
    {"a3iii", 'A', 3, "III", 0}, {"a3v", 'A', 3, "V", 0}, {"a47iv", 'A', 5.5, "IV", 0},
    {"a5iii", 'A', 5, "III", 0}, {"a5v", 'A', 5, "V", 0}, {"a7iii", 'A', 7, "III", 0},
    {"a7v", 'A', 7, "V", 0},
    // F
    {"f0i", 'F', 0, "I", 0}, {"f0ii", 'F', 0, "II", 0}, {"f0iii", 'F', 0, "III", 0},
    {"f02iv", 'F', 1, "IV", 0}, {"f0v", 'F', 0, "V", 0}, {"f2ii", 'F', 2, "II", 0},
    {"f2iii", 'F', 2, "III", 0}, {"f2v", 'F', 2, "V", 0}, {"f5i", 'F', 5, "I", 0},
    {"f5iii", 'F', 5, "III", 0}, {"f5iv", 'F', 5, "IV", 0}, {"f5v", 'F', 5, "V", 0},
    {"f6v", 'F', 6, "V", 0}, {"f8i", 'F', 8, "I", 0}, {"f8iv", 'F', 8, "IV", 0},
    {"f8v", 'F', 8, "V", 0},
    // G
    {"g0i", 'G', 0, "I", 0}, {"g0iii", 'G', 0, "III", 0}, {"g0iv", 'G', 0, "IV", 0},
    {"g0v", 'G', 0, "V", 0}, {"g2i", 'G', 2, "I", 0}, {"g2iv", 'G', 2, "IV", 0},
    {"g2v", 'G', 2, "V", 0}, {"g5i", 'G', 5, "I", 0}, {"g5ii", 'G', 5, "II", 0},
    {"g5iii", 'G', 5, "III", 0}, {"g5iv", 'G', 5, "IV", 0}, {"g5v", 'G', 5, "V", 0},
    {"g8i", 'G', 8, "I", 0}, {"g8iii", 'G', 8, "III", 0}, {"g8iv", 'G', 8, "IV", 0},
    {"g8v", 'G', 8, "V", 0},
    // K
    {"k01ii", 'K', 0.5, "II", 0}, {"k0iii", 'K', 0, "III", 0}, {"k0iv", 'K', 0, "IV", 0},
    {"k0v", 'K', 0, "V", 0}, {"k1iii", 'K', 1, "III", 0}, {"k1iv", 'K', 1, "IV", 0},
    {"k2i", 'K', 2, "I", 0}, {"k2iii", 'K', 2, "III", 0}, {"k2v", 'K', 2, "V", 0},
    {"k34ii", 'K', 3.5, "II", 0}, {"k3i", 'K', 3, "I", 0}, {"k3iii", 'K', 3, "III", 0},
    {"k3iv", 'K', 3, "IV", 0}, {"k3v", 'K', 3, "V", 0}, {"k4i", 'K', 4, "I", 0},
    {"k4iii", 'K', 4, "III", 0}, {"k4v", 'K', 4, "V", 0}, {"k5iii", 'K', 5, "III", 0},
    {"k5v", 'K', 5, "V", 0}, {"k7v", 'K', 7, "V", 0},
    // M
    {"m0iii", 'M', 0, "III", 0}, {"m0v", 'M', 0, "V", 0}, {"m10iii", 'M', 10, "III", 0},
    {"m1iii", 'M', 1, "III", 0}, {"m1v", 'M', 1, "V", 0}, {"m2i", 'M', 2, "I", 0},
    {"m2iii", 'M', 2, "III", 0}, {"m2p5v", 'M', 2.5, "V", 0}, {"m2v", 'M', 2, "V", 0},
    {"m3ii", 'M', 3, "II", 0}, {"m3iii", 'M', 3, "III", 0}, {"m3v", 'M', 3, "V", 0},
    {"m4iii", 'M', 4, "III", 0}, {"m4v", 'M', 4, "V", 0}, {"m5iii", 'M', 5, "III", 0},
    {"m5v", 'M', 5, "V", 0}, {"m6iii", 'M', 6, "III", 0}, {"m6v", 'M', 6, "V", 0},
    {"m7iii", 'M', 7, "III", 0}, {"m8iii", 'M', 8, "III", 0}, {"m9iii", 'M', 9, "III", 0},
    // Metal rich
    {"rf6v", 'F', 6, "V", 1}, {"rf8v", 'F', 8, "V", 1}, {"rg0v", 'G', 0, "V", 1},
    {"rg5iii", 'G', 5, "III", 1}, {"rg5v", 'G', 5, "V", 1}, {"rk0iii", 'K', 0, "III", 1},
    {"rk0v", 'K', 0, "V", 1}, {"rk1iii", 'K', 1, "III", 1}, {"rk2iii", 'K', 2, "III", 1},
    {"rk3iii", 'K', 3, "III", 1}, {"rk4iii", 'K', 4, "III", 1}, {"rk5iii", 'K', 5, "III", 1},
    // Metal weak
    {"wf5v", 'F', 5, "V", -1}, {"wf8v", 'F', 8, "V", -1}, {"wg0v", 'G', 0, "V", -1},
    {"wg5iii", 'G', 5, "III", -1}, {"wg5v", 'G', 5, "V", -1}, {"wg8iii", 'G', 8, "III", -1},
    {"wk0iii", 'K', 0, "III", -1}, {"wk1iii", 'K', 1, "III", -1}, {"wk2iii", 'K', 2, "III", -1},
    {"wk3iii", 'K', 3, "III", -1}, {"wk4iii", 'K', 4, "III", -1},
};

int luminosityOrder(const QString& lum)
{
    if (lum == QLatin1String("I")) {
        return 1;
    }
    if (lum == QLatin1String("II")) {
        return 2;
    }
    if (lum == QLatin1String("III")) {
        return 3;
    }
    if (lum == QLatin1String("IV")) {
        return 4;
    }
    return 5; // V
}

QString metallicitySuffix(int metallicity)
{
    if (metallicity > 0) {
        return QStringLiteral(" (metal rich)");
    }
    if (metallicity < 0) {
        return QStringLiteral(" (metal weak)");
    }
    return QString();
}

QString formatSubClass(double subClass)
{
    // 2.5 -> "2.5", 2.0 -> "2"
    return (std::abs(subClass - std::round(subClass)) < 1e-6)
        ? QString::number(static_cast<int>(std::lround(subClass)))
        : QString::number(subClass, 'f', 1);
}

} // namespace

const QVector<CameraOpticalSpectrumTemplate>& CameraOpticalSpectrumLibrary::templates()
{
    static const QVector<CameraOpticalSpectrumTemplate> allTemplates = []() {
        QVector<CameraOpticalSpectrumTemplate> list;
        for (const TemplateDef& def : kTemplateDefs)
        {
            CameraOpticalSpectrumTemplate entry;
            entry.m_key = QString::fromLatin1(def.m_key);
            entry.m_class = QChar::fromLatin1(def.m_class);
            entry.m_subClass = def.m_subClass;
            entry.m_lum = QString::fromLatin1(def.m_lum);
            entry.m_metallicity = def.m_metallicity;
            entry.m_name = QStringLiteral("%1%2 %3%4")
                .arg(entry.m_class)
                .arg(formatSubClass(entry.m_subClass))
                .arg(entry.m_lum)
                .arg(metallicitySuffix(entry.m_metallicity));
            list.append(entry);
        }
        return list;
    }();
    return allTemplates;
}

const CameraOpticalSpectrumTemplate* CameraOpticalSpectrumLibrary::findTemplate(const QString& key)
{
    for (const CameraOpticalSpectrumTemplate& entry : templates())
    {
        if (entry.m_key == key) {
            return &entry;
        }
    }
    return nullptr;
}

CameraOpticalSpectrumType CameraOpticalSpectrumLibrary::parseSpectralType(const QString& spectralType)
{
    CameraOpticalSpectrumType parsed;
    const QString text = spectralType.trimmed();

    // An MK type always leads with its Harvard class letter, so only the first character
    // is considered. Scanning for the first O/B/A/F/G/K/M anywhere would misread the
    // degenerate and subdwarf types that embed one ("DA2" -> A2, "sdB" -> B).
    if (text.isEmpty() || !QStringLiteral("OBAFGKM").contains(text.at(0))) {
        return parsed;
    }
    parsed.m_class = text.at(0);

    // Numeric subclass immediately after the class letter (may be fractional, e.g. K1.5)
    int i = 1;
    QString number;
    while ((i < text.size()) && (text.at(i).isDigit() || (text.at(i) == '.')))
    {
        number.append(text.at(i));
        i++;
    }
    bool ok = false;
    const double subClass = number.toDouble(&ok);
    parsed.m_subClass = ok ? subClass : 0.0;

    // Luminosity class after the subclass. Longest match first so "III" is not read as
    // "II", and "IV" not as "I". Anything trailing (Ia/Iab/Va) is a refinement we ignore.
    parsed.m_lum = QStringLiteral("V");
    static const QStringList kLumClasses = {
        QStringLiteral("III"), QStringLiteral("IV"), QStringLiteral("II"),
        QStringLiteral("VI"), QStringLiteral("V"), QStringLiteral("I")
    };
    for (int j = i; j < text.size(); j++)
    {
        bool matched = false;
        for (const QString& lum : kLumClasses)
        {
            if (text.mid(j, lum.size()) == lum)
            {
                // VI is a subdwarf; treat as a dwarf since the library has no VI templates
                parsed.m_lum = (lum == QLatin1String("VI")) ? QStringLiteral("V") : lum;
                matched = true;
                break;
            }
        }
        if (matched) {
            break;
        }
    }

    parsed.m_valid = true;
    return parsed;
}

QString CameraOpticalSpectrumLibrary::matchTemplate(const QString& spectralType)
{
    const CameraOpticalSpectrumType parsed = parseSpectralType(spectralType);
    if (!parsed.m_valid) {
        return QString();
    }

    // Closest template of the same class: luminosity class dominates the score so a
    // giant is never matched to a dwarf just because its subclass is closer.
    QString bestKey;
    double bestScore = std::numeric_limits<double>::max();
    for (const CameraOpticalSpectrumTemplate& entry : templates())
    {
        if ((entry.m_class != parsed.m_class) || (entry.m_metallicity != 0)) {
            continue;
        }
        const double lumDistance = std::abs(luminosityOrder(entry.m_lum) - luminosityOrder(parsed.m_lum));
        const double score = 10.0 * lumDistance + std::abs(entry.m_subClass - parsed.m_subClass);
        if (score < bestScore)
        {
            bestScore = score;
            bestKey = entry.m_key;
        }
    }
    return bestKey;
}

QString CameraOpticalSpectrumLibrary::templateUrl(const QString& key)
{
    return QStringLiteral("https://cdsarc.cds.unistra.fr/ftp/J/PASP/110/863/%1.dat.gz").arg(key);
}

const QVector<CameraOpticalSpectrumEmissionTemplate>& CameraOpticalSpectrumLibrary::emissionTemplates()
{
    // SDSS DR7 spectroscopic cross-correlation templates (spSpec format). Rest-frame
    // spectra; the dialog applies the redshift setting when overlaying them.
    static const QVector<CameraOpticalSpectrumEmissionTemplate> templates = {
        {QStringLiteral("qso"), QStringLiteral("QSO composite (SDSS)"), QStringLiteral("spDR2-029.fit")},
        {QStringLiteral("qsolum"), QStringLiteral("QSO high luminosity (SDSS)"), QStringLiteral("spDR2-032.fit")},
        {QStringLiteral("galaxy-early"), QStringLiteral("Galaxy, early type (SDSS)"), QStringLiteral("spDR2-023.fit")},
        {QStringLiteral("galaxy-late"), QStringLiteral("Galaxy, late type (SDSS)"), QStringLiteral("spDR2-027.fit")},
        {QStringLiteral("galaxy-lrg"), QStringLiteral("Luminous red galaxy (SDSS)"), QStringLiteral("spDR2-028.fit")},
    };
    return templates;
}

bool CameraOpticalSpectrumLibrary::isEmissionTemplate(const QString& key)
{
    for (const CameraOpticalSpectrumEmissionTemplate& entry : emissionTemplates())
    {
        if (entry.m_key == key) {
            return true;
        }
    }
    return false;
}

QString CameraOpticalSpectrumLibrary::emissionTemplateUrl(const QString& key)
{
    for (const CameraOpticalSpectrumEmissionTemplate& entry : emissionTemplates())
    {
        if (entry.m_key == key) {
            return QStringLiteral("https://classic.sdss.org/dr7/algorithms/spectemplates/%1").arg(entry.m_file);
        }
    }
    return QString();
}

QString CameraOpticalSpectrumLibrary::templateDisplayName(const QString& key)
{
    if (const CameraOpticalSpectrumTemplate* stellar = findTemplate(key)) {
        return stellar->m_name;
    }
    for (const CameraOpticalSpectrumEmissionTemplate& entry : emissionTemplates())
    {
        if (entry.m_key == key) {
            return entry.m_name;
        }
    }
    return QString();
}

QVector<QPointF> CameraOpticalSpectrumLibrary::parseSdssTemplateFits(const QByteArray& fits)
{
    QVector<QPointF> points;
    constexpr int kBlockSize = 2880;
    constexpr int kCardSize = 80;
    if (fits.size() < kBlockSize) {
        return points;
    }

    // Parse the primary header cards until END; values we need are numeric
    int bitpix = 0;
    qint64 axis1 = 0;
    double coeff0 = std::numeric_limits<double>::quiet_NaN();
    double coeff1 = std::numeric_limits<double>::quiet_NaN();
    int dataOffset = -1;
    for (int block = 0; (dataOffset < 0) && ((block + 1) * kBlockSize <= fits.size()); block++)
    {
        for (int card = 0; card < kBlockSize / kCardSize; card++)
        {
            const QByteArray line = fits.mid(block * kBlockSize + card * kCardSize, kCardSize);
            const QByteArray keyword = line.left(8).trimmed();
            if (keyword == "END")
            {
                dataOffset = (block + 1) * kBlockSize;
                break;
            }
            const int eq = line.indexOf('=');
            if (eq < 0) {
                continue;
            }
            QByteArray value = line.mid(eq + 1);
            const int slash = value.indexOf('/');
            if (slash >= 0) {
                value = value.left(slash);
            }
            bool ok = false;
            const double numeric = value.trimmed().toDouble(&ok);
            if (!ok) {
                continue;
            }
            if (keyword == "BITPIX") {
                bitpix = static_cast<int>(numeric);
            } else if (keyword == "NAXIS1") {
                axis1 = static_cast<qint64>(numeric);
            } else if ((keyword == "COEFF0") || (keyword == "CRVAL1")) {
                if (std::isnan(coeff0)) {
                    coeff0 = numeric;
                }
            } else if ((keyword == "COEFF1") || (keyword == "CD1_1")) {
                if (std::isnan(coeff1)) {
                    coeff1 = numeric;
                }
            }
        }
    }

    if ((dataOffset < 0) || (bitpix != -32) || (axis1 <= 0) || std::isnan(coeff0) || std::isnan(coeff1)
        || (fits.size() < dataOffset + axis1 * 4)) {
        return points;
    }

    // Flux is row 0: axis1 big-endian IEEE floats. Downsample for display; the native
    // sampling (~10k points) is far finer than the chart needs.
    const int stride = qMax(1, static_cast<int>(axis1 / 2048));
    const uchar* data = reinterpret_cast<const uchar*>(fits.constData()) + dataOffset;
    points.reserve(static_cast<int>(axis1 / stride) + 1);
    for (qint64 i = 0; i < axis1; i += stride)
    {
        const uchar* p = data + 4 * i;
        quint32 raw = (quint32(p[0]) << 24) | (quint32(p[1]) << 16) | (quint32(p[2]) << 8) | quint32(p[3]);
        float flux = 0.0f;
        std::memcpy(&flux, &raw, sizeof(flux));
        if (!std::isfinite(flux)) {
            continue;
        }
        const double angstrom = std::pow(10.0, coeff0 + coeff1 * static_cast<double>(i));
        points.append(QPointF(angstrom / 10.0, flux));
    }
    return points;
}

QVector<QPointF> CameraOpticalSpectrumLibrary::parseExportedSpectrumCsv(const QByteArray& csv)
{
    QVector<QPointF> points;
    const QList<QByteArray> lines = csv.split('\n');
    if (lines.size() < 2) {
        return points;
    }

    const QList<QByteArray> header = lines.first().trimmed().split(',');
    const int wavelengthColumn = header.indexOf("wavelength_nm");
    int valueColumn = header.indexOf("luminance_corrected");
    if (valueColumn < 0) {
        valueColumn = header.indexOf("luminance");
    }
    if ((wavelengthColumn < 0) || (valueColumn < 0)) {
        return points;
    }

    for (int i = 1; i < lines.size(); i++)
    {
        const QList<QByteArray> fields = lines.at(i).trimmed().split(',');
        if (fields.size() <= qMax(wavelengthColumn, valueColumn)) {
            continue;
        }
        bool okNm = false;
        bool okValue = false;
        const double nm = fields.at(wavelengthColumn).toDouble(&okNm);
        const double value = fields.at(valueColumn).toDouble(&okValue);
        if (okNm && okValue) {
            points.append(QPointF(nm, value));
        }
    }
    std::sort(points.begin(), points.end(), [](const QPointF& a, const QPointF& b) { return a.x() < b.x(); });
    return points;
}

QString CameraOpticalSpectrumLibrary::simbadLookupUrl(const QString& objectName)
{
    const QString query = QStringLiteral(
        "SELECT main_id, sp_type FROM basic JOIN ident ON oidref = oid WHERE id = '%1'")
        .arg(QString(objectName).replace('\'', QStringLiteral("''")));
    return QStringLiteral("https://simbad.cds.unistra.fr/simbad/sim-tap/sync?request=doQuery&lang=adql&format=csv&query=%1")
        .arg(QString::fromLatin1(QUrl::toPercentEncoding(query)));
}

bool CameraOpticalSpectrumLibrary::parseSimbadResponse(const QByteArray& csv, QString& mainId, QString& spectralType)
{
    const QStringList lines = QString::fromUtf8(csv).split('\n', Qt::SkipEmptyParts);
    if (lines.size() < 2) {
        return false;
    }

    // Data row: main_id,sp_type - both may be quoted and may contain commas
    const QString row = lines.at(1).trimmed();
    QStringList fields;
    QString field;
    bool inQuotes = false;
    for (int i = 0; i < row.size(); i++)
    {
        const QChar c = row.at(i);
        if (c == '"')
        {
            // Doubled quotes inside a quoted field are an escaped quote
            if (inQuotes && (i + 1 < row.size()) && (row.at(i + 1) == '"'))
            {
                field.append('"');
                i++;
            }
            else
            {
                inQuotes = !inQuotes;
            }
        }
        else if ((c == ',') && !inQuotes)
        {
            fields.append(field);
            field.clear();
        }
        else
        {
            field.append(c);
        }
    }
    fields.append(field);

    if (fields.size() < 2) {
        return false;
    }
    mainId = fields.at(0).trimmed();
    spectralType = fields.at(1).trimmed();
    return !spectralType.isEmpty();
}

namespace {

// Linear interpolation on an x-ascending point list; NaN outside the range
double interpolateAt(const QVector<QPointF>& points, double x)
{
    if (points.isEmpty() || (x < points.first().x()) || (x > points.last().x())) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    int lo = 0;
    int hi = points.size() - 1;
    while (hi - lo > 1)
    {
        const int mid = (lo + hi) / 2;
        if (points[mid].x() <= x) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const double span = points[hi].x() - points[lo].x();
    if (span <= 0.0) {
        return points[lo].y();
    }
    const double frac = (x - points[lo].x()) / span;
    return points[lo].y() + frac * (points[hi].y() - points[lo].y());
}

} // namespace

QVector<QPointF> CameraOpticalSpectrumLibrary::computeInstrumentResponse(
    const QVector<double>& wavelengths,
    const QVector<float>& observed,
    const QVector<QPointF>& reference,
    const QVector<double>& maskedWavelengths,
    double maskHalfWidthNm,
    double smoothingWidthNm,
    double* rawPeakOut)
{
    if (rawPeakOut) {
        *rawPeakOut = 0.0;
    }
    if ((wavelengths.size() != observed.size()) || (wavelengths.size() < 2) || reference.isEmpty()) {
        return QVector<QPointF>();
    }

    double referencePeak = 0.0;
    for (const QPointF& point : reference) {
        referencePeak = std::max(referencePeak, point.y());
    }
    if (referencePeak <= 0.0) {
        return QVector<QPointF>();
    }

    // Effective mask half-widths: where masked lines crowd together (the Balmer series
    // converging towards its limit in the blue), fixed-width masks merge into one block
    // that swallows the whole region, leaving no continuum anchors and no response there
    // at all. Cap each line's mask at 40% of the gap to its neighbours so a sliver of
    // continuum always survives between adjacent lines; the heavy smoothing that follows
    // tolerates the resulting narrower masks.
    QVector<double> maskCentres = maskedWavelengths;
    std::sort(maskCentres.begin(), maskCentres.end());
    // Deduplicate near-identical centres: a duplicated line would otherwise cap both
    // masks to 40% of a ~zero gap, silently unmasking the line entirely
    maskCentres.erase(std::unique(maskCentres.begin(), maskCentres.end(),
        [](double a, double b) { return std::abs(a - b) < 0.05; }), maskCentres.end());
    QVector<double> maskHalfWidths(maskCentres.size(), maskHalfWidthNm);
    for (int i = 0; i < maskCentres.size(); i++)
    {
        if (i > 0) {
            maskHalfWidths[i] = std::min(maskHalfWidths[i], 0.4 * (maskCentres[i] - maskCentres[i - 1]));
        }
        if (i < maskCentres.size() - 1) {
            maskHalfWidths[i] = std::min(maskHalfWidths[i], 0.4 * (maskCentres[i + 1] - maskCentres[i]));
        }
    }

    // Raw per-sample ratios, excluding masked lines, weak-reference regions and
    // non-positive observations
    QVector<QPointF> ratios;
    ratios.reserve(wavelengths.size());
    for (int i = 0; i < wavelengths.size(); i++)
    {
        const double nm = wavelengths[i];
        const double observedValue = observed[i];
        if (observedValue <= 0.0) {
            continue;
        }
        bool masked = false;
        for (int m = 0; m < maskCentres.size(); m++)
        {
            if (std::abs(nm - maskCentres[m]) <= maskHalfWidths[m])
            {
                masked = true;
                break;
            }
        }
        if (masked) {
            continue;
        }
        const double referenceValue = interpolateAt(reference, nm);
        // Avoid dividing by the reference's faint tails, which explode the ratio
        if (std::isnan(referenceValue) || (referenceValue < 0.02 * referencePeak)) {
            continue;
        }
        ratios.append(QPointF(nm, observedValue / referenceValue));
    }

    std::sort(ratios.begin(), ratios.end(), [](const QPointF& a, const QPointF& b) { return a.x() < b.x(); });

    // Require a meaningful overlap before claiming a response
    if ((ratios.size() < 20) || (ratios.last().x() - ratios.first().x() < 50.0)) {
        return QVector<QPointF>();
    }

    // Resample to a 1 nm grid (interpolating across the masked gaps), then boxcar smooth
    const int gridStart = static_cast<int>(std::ceil(ratios.first().x()));
    const int gridEnd = static_cast<int>(std::floor(ratios.last().x()));
    if (gridEnd - gridStart < 50) {
        return QVector<QPointF>();
    }
    QVector<double> grid(gridEnd - gridStart + 1);
    for (int i = 0; i < grid.size(); i++) {
        grid[i] = interpolateAt(ratios, gridStart + i);
    }

    const int half = std::max(1, static_cast<int>(std::lround(smoothingWidthNm / 2.0)));
    const int gridSize = static_cast<int>(grid.size());
    QVector<QPointF> response(gridSize);
    double peak = 0.0;
    for (int i = 0; i < gridSize; i++)
    {
        // The window shrinks SYMMETRICALLY at the grid edges. Simply clamping it makes
        // the average one-sided there, which biases the response high on a falling edge
        // (and low on a rising one) and bends the corrected spectrum down/up at its ends.
        const int sideHalf = std::min(half, std::min(i, gridSize - 1 - i));
        const int lo = i - sideHalf;
        const int hi = i + sideHalf;
        double sum = 0.0;
        for (int j = lo; j <= hi; j++) {
            sum += grid[j];
        }
        const double value = sum / (hi - lo + 1);
        response[i] = QPointF(gridStart + i, value);
        peak = std::max(peak, value);
    }
    if (peak <= 0.0) {
        return QVector<QPointF>();
    }
    if (rawPeakOut) {
        *rawPeakOut = peak;
    }
    for (QPointF& point : response) {
        point.setY(point.y() / peak);
    }
    return response;
}

double CameraOpticalSpectrumLibrary::responseAt(const QVector<QPointF>& response, double nm)
{
    const double value = interpolateAt(response, nm);
    if (!std::isnan(value)) {
        return value;
    }
    // The response grid is trimmed to whole nanometres inside the captured range, so a
    // spectrum can extend a fraction of a nanometre beyond it at each end; without a
    // tolerance those samples read "no coverage" and get blanked, notching the ends of
    // the corrected spectrum down to zero. Hold the end value over a small margin.
    constexpr double kEdgeToleranceNm = 2.0;
    if (!response.isEmpty())
    {
        if ((nm < response.first().x()) && (response.first().x() - nm <= kEdgeToleranceNm)) {
            return response.first().y();
        }
        if ((nm > response.last().x()) && (nm - response.last().x() <= kEdgeToleranceNm)) {
            return response.last().y();
        }
    }
    return 0.0;
}

QByteArray CameraOpticalSpectrumLibrary::gunzip(const QByteArray& compressed)
{
    if (compressed.isEmpty()) {
        return QByteArray();
    }

    z_stream stream;
    std::memset(&stream, 0, sizeof(stream));
    stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(compressed.constData()));
    stream.avail_in = static_cast<uInt>(compressed.size());

    // 16 + MAX_WBITS selects gzip (rather than zlib) framing
    if (inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK) {
        return QByteArray();
    }

    QByteArray uncompressed;
    char buffer[32768];
    int status = Z_OK;
    do
    {
        stream.next_out = reinterpret_cast<Bytef*>(buffer);
        stream.avail_out = sizeof(buffer);
        status = inflate(&stream, Z_NO_FLUSH);
        if ((status != Z_OK) && (status != Z_STREAM_END))
        {
            inflateEnd(&stream);
            return QByteArray();
        }
        uncompressed.append(buffer, sizeof(buffer) - static_cast<int>(stream.avail_out));
    } while (status != Z_STREAM_END);

    inflateEnd(&stream);
    return uncompressed;
}

QVector<QPointF> CameraOpticalSpectrumLibrary::parseSpectrumData(const QByteArray& data)
{
    QVector<QPointF> points;
    const QList<QByteArray> lines = data.split('\n');
    points.reserve(lines.size());
    for (const QByteArray& line : lines)
    {
        const QByteArray trimmed = line.trimmed();
        if (trimmed.isEmpty() || trimmed.startsWith('#')) {
            continue;
        }
        // Columns: wavelength (Angstrom), normalised flux, standard deviation, components
        const QList<QByteArray> fields = trimmed.simplified().split(' ');
        if (fields.size() < 2) {
            continue;
        }
        bool okWavelength = false;
        bool okFlux = false;
        const double angstrom = fields.at(0).toDouble(&okWavelength);
        const double flux = fields.at(1).toDouble(&okFlux);
        if (!okWavelength || !okFlux || (angstrom <= 0.0)) {
            continue;
        }
        points.append(QPointF(angstrom / 10.0, flux));
    }
    return points;
}
