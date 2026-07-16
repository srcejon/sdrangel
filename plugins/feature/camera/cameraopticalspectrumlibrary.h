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

#ifndef INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMLIBRARY_H_
#define INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMLIBRARY_H_

#include <QPointF>
#include <QString>
#include <QVector>

/// One reference spectrum template from the Pickles (1998) stellar spectral flux library
struct CameraOpticalSpectrumTemplate
{
    QString m_key;      ///< Catalog file base name, e.g. "a0v"; also the settings value
    QString m_name;     ///< Display name, e.g. "A0 V"
    QChar m_class;      ///< Harvard class letter: O B A F G K M
    double m_subClass;  ///< Numeric subclass, e.g. 0 for A0; 2.5 for M2.5
    QString m_lum;      ///< Luminosity class: I II III IV V
    int m_metallicity;  ///< -1 metal-weak, 0 solar, +1 metal-rich
};

/// A spectral type parsed out of a SIMBAD sp_type string
struct CameraOpticalSpectrumType
{
    bool m_valid = false;
    QChar m_class;
    double m_subClass = 0.0;
    QString m_lum;
};

/// A non-stellar reference spectrum template (SDSS cross-correlation templates)
struct CameraOpticalSpectrumEmissionTemplate
{
    QString m_key;  ///< Settings key, e.g. "qso"; distinct from all Pickles keys
    QString m_name; ///< Display name
    QString m_file; ///< File name at the SDSS template archive
};

/**
 * \brief Reference stellar spectra from the Pickles (1998) library, hosted by CDS/VizieR.
 *
 * Provides the template catalogue, a parser for SIMBAD spectral-type strings, matching of
 * a spectral type to the closest template, and a parser for the downloaded data files.
 * Pure data handling with no GUI or network dependency so it can be unit tested.
 *
 * Reference: Pickles A.J., 1998, PASP 110, 863; catalogue J/PASP/110/863.
 * Spectra are normalised to 1.0 at 555.6 nm, so they carry the shape of the spectrum
 * rather than an absolute flux.
 */
class CameraOpticalSpectrumLibrary
{
public:
    /// All 131 templates, ordered by class then subclass then luminosity class
    static const QVector<CameraOpticalSpectrumTemplate>& templates();

    /// Look up a template by its key; returns nullptr if unknown
    static const CameraOpticalSpectrumTemplate* findTemplate(const QString& key);

    /// Parses a SIMBAD sp_type string (e.g. "A0V", "K1.5IIIFe-0.5", "M1-M2Ia-Iab").
    /// A missing luminosity class defaults to V.
    static CameraOpticalSpectrumType parseSpectralType(const QString& spectralType);

    /// Closest solar-abundance template to the given SIMBAD spectral type; empty if unmatched
    static QString matchTemplate(const QString& spectralType);

    /// URL of the gzipped data file for a template key
    static QString templateUrl(const QString& key);

    /// Non-stellar (QSO/galaxy) templates from the SDSS DR7 spectral template archive
    static const QVector<CameraOpticalSpectrumEmissionTemplate>& emissionTemplates();

    /// True when the key names an emission-object template rather than a Pickles one
    static bool isEmissionTemplate(const QString& key);

    /// URL of the FITS file for an emission-object template key
    static QString emissionTemplateUrl(const QString& key);

    /// Display name for any template key, stellar or emission; empty if unknown
    static QString templateDisplayName(const QString& key);

    /// Parses an SDSS spSpec-format spectral template FITS file (BITPIX -32, flux in
    /// row 0, log-linear wavelength from COEFF0/COEFF1 in Angstroms) into
    /// (wavelength nm, flux) points, downsampled for display.
    static QVector<QPointF> parseSdssTemplateFits(const QByteArray& fits);

    /// Parses a CSV previously written by the spectrum dialog's Export (columns
    /// wavelength_nm and luminance_corrected/luminance) into (nm, value) points.
    /// Empty when the file has no wavelength calibration.
    static QVector<QPointF> parseExportedSpectrumCsv(const QByteArray& csv);

    /// SIMBAD TAP URL resolving an object name to its main id and spectral type (CSV)
    static QString simbadLookupUrl(const QString& objectName);

    /// Extracts main_id and sp_type from a SIMBAD CSV response; returns false if not found
    static bool parseSimbadResponse(const QByteArray& csv, QString& mainId, QString& spectralType);

    /// Parses a decompressed Pickles data file into (wavelength nm, normalised flux) points
    static QVector<QPointF> parseSpectrumData(const QByteArray& data);

    /// Decompresses a gzip (.gz) blob; returns empty on failure
    static QByteArray gunzip(const QByteArray& compressed);

    /**
     * Computes the relative instrument response (sensor QE x Bayer filters x grating
     * blaze x optics x atmosphere) by dividing an observed, wavelength-calibrated
     * spectrum of a reference star by its library template. Wavelengths within
     * maskHalfWidthNm of a masked line (the star's absorption lines and telluric bands)
     * are excluded and interpolated across, and the ratio is resampled to a 1 nm grid
     * and smoothed over smoothingWidthNm - the response is physically smooth, so heavy
     * smoothing is correct and suppresses noise and residual line structure.
     *
     * @return (wavelength nm, response) normalised to a peak of 1.0; empty if the
     *         overlap is too small to be usable. rawPeakOut (optional) receives the
     *         pre-normalisation peak, so per-channel responses can be rescaled to a
     *         shared normalisation that preserves the relative channel sensitivities.
     */
    static QVector<QPointF> computeInstrumentResponse(
        const QVector<double>& wavelengths,
        const QVector<float>& observed,
        const QVector<QPointF>& reference,
        const QVector<double>& maskedWavelengths,
        double maskHalfWidthNm,
        double smoothingWidthNm,
        double* rawPeakOut = nullptr);

    /// Linearly interpolated response at the given wavelength; 0 outside the curve's range
    static double responseAt(const QVector<QPointF>& response, double nm);
};

#endif // INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMLIBRARY_H_
