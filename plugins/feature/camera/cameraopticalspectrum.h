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

#ifndef INCLUDE_FEATURE_CAMERAOPTICALSPECTRUM_H_
#define INCLUDE_FEATURE_CAMERAOPTICALSPECTRUM_H_

#include <QImage>
#include <QString>
#include <QVector>

/**
 * \brief 1D optical spectrum extracted from the detection RoI of a frame.
 *
 * Produced by CameraOpticalSpectrumExtractor while the optical spectrum dialog is open and
 * attached to CameraPipelineFrame::m_opticalSpectrumData for display in the GUI. Profiles
 * are in the pixel domain (index = offset along the dispersion axis from m_axisOrigin);
 * wavelength calibration is applied in the GUI so recalibration does not require new
 * frames.
 *
 * \note isValid() requires a non-empty luminance profile; the per-channel profiles
 *       always have the same length as m_luminance when valid.
 */
struct CameraOpticalSpectrumData
{
    QVector<float> m_luminance;  ///< Background-subtracted aperture sums per position along the dispersion axis
    QVector<float> m_red;        ///< Red channel profile
    QVector<float> m_green;      ///< Green channel profile
    QVector<float> m_blue;       ///< Blue channel profile
    QVector<float> m_background; ///< Luminance background that was subtracted (aperture-scaled); empty when no subtraction
    int m_axisOrigin = 0;        ///< Image-space X (or Y when m_verticalAxis) of profile index 0
    bool m_verticalAxis = false; ///< Dispersion axis runs along image Y (RoI taller than wide)
    float m_zeroOrderPx = -1.0f; ///< Auto-detected zero-order centroid in profile-index units; -1 = not found
    /// Background subtraction was requested but skipped because the source fills the RoI,
    /// leaving no off-trace rows to estimate a background from (e.g. a discharge tube whose
    /// emission lines span the whole frame). Subtracting would have removed the signal itself.
    bool m_backgroundUnavailable = false;
    /// Background subtraction was requested but skipped because the RoI leaves too few rows
    /// beyond the aperture and its guard gap to estimate a background from - the RoI needs
    /// sky margins above/below the trace.
    bool m_backgroundInsufficientRows = false;
    float m_saturatedFraction = 0.0f; ///< Fraction of aperture pixels with a clipped channel

    bool isValid() const { return !m_luminance.isEmpty(); }
};

/// A named reference wavelength shown as an overlay line on the spectrum chart
struct CameraOpticalSpectrumRefLine
{
    QString m_label;
    double m_nm;                 ///< Rest wavelength
    bool m_terrestrial = false;  ///< Line originates in Earth's atmosphere, so is never redshifted
};

/// A selectable set of reference lines (e.g. the Balmer series)
struct CameraOpticalSpectrumRefLineSet
{
    QString m_key;  ///< Stable key used in the opticalSpectrumRefLines setting
    QString m_name; ///< Display name
    QVector<CameraOpticalSpectrumRefLine> m_lines;
    bool m_terrestrial = false; ///< Set originates in Earth's atmosphere (telluric, aurora/airglow)
};

/// A detected spectral feature (emission peak or absorption dip) in a profile
struct CameraOpticalSpectrumFeature
{
    int m_index = 0;           ///< Sample index of the extremum
    bool m_emission = false;   ///< Peak above the continuum (true) or dip below it (false)
    float m_prominence = 0.0f; ///< |value - continuum| at the extremum
};

/// Result of measuring a spectral line feature at a position in a profile
struct CameraOpticalSpectrumLineMeasurement
{
    bool m_valid = false;
    bool m_emission = false;
    double m_centreIndex = 0.0;            ///< Extremum position in samples
    double m_continuum = 0.0;              ///< Local continuum level
    double m_fwhmSamples = 0.0;            ///< Full width at half maximum, in samples
    double m_equivalentWidthSamples = 0.0; ///< Equivalent width in samples (positive for both emission and absorption)
};

/**
 * \brief Wavelength calibration solved from clicked reference points.
 *
 * The model is nm = dispersion * sign * (pixel - zeroOrderPx) with sign = +1 when the
 * red end lies at increasing pixel coordinate. Produced by
 * CameraOpticalSpectrumExtractor::calibrateOnePoint / calibrateTwoPoint.
 */
struct CameraOpticalSpectrumCalibration
{
    bool m_valid = false;
    double m_dispersion = 0.0;  ///< nm/pixel, always positive when valid
    bool m_redPositive = true;  ///< Wavelength increases with increasing pixel coordinate
    double m_zeroOrderPx = 0.0; ///< Zero-order position along the dispersion axis in image pixels
};

/**
 * \brief Extracts a 1D spectrum (intensity profile along the dispersion axis) from an image RoI.
 *
 * For slitless (objective diffraction grating) star spectroscopy: the user places the
 * detection RoI around the first-order spectrum trace; the extractor collapses the RoI
 * to a per-column intensity profile by summing an aperture of rows centred on the trace
 * and subtracting a per-column sky background estimated from the rows outside the
 * aperture. The dispersion axis is taken along the RoI's long side.
 *
 * Stateless; safe to call from any pipeline thread.
 */
class CameraOpticalSpectrumExtractor
{
public:
    /**
     * @param image Source frame (any QImage format; 8 and 16 bit-per-channel supported).
     * @param roiX/roiY/roiWidth/roiHeight Detection RoI; width/height <= 0 = full frame
     *        (same semantics as the detection stages).
     * @param apertureRows Number of rows summed across the trace; <= 0 = full RoI height
     *        (background subtraction is then unavailable).
     * @param backgroundSubtract Subtract the per-column median of the rows outside the
     *        aperture (scaled by the aperture height).
     */
    static CameraOpticalSpectrumData extract(
        const QImage& image,
        int roiX,
        int roiY,
        int roiWidth,
        int roiHeight,
        int apertureRows,
        bool backgroundSubtract);

    /// True when the red end of the spectrum lies at increasing profile index (from the
    /// red-vs-blue channel intensity centroids). Meaningful only for colour spectra.
    static bool autoDirectionRedPositive(const CameraOpticalSpectrumData& data);

    /// True when the red/blue centroid separation is large enough for
    /// autoDirectionRedPositive to be trusted. On faint or near-grey data the centroids
    /// are a coin flip frame to frame; callers should latch the last decisive answer
    /// rather than let the wavelength axis flip with the noise.
    static bool autoDirectionDecisive(const CameraOpticalSpectrumData& data, double minSeparationSamples = 2.0);

    /// Approximate perceived colour of monochromatic light of the given wavelength
    /// (Bruton's piecewise-linear approximation), scaled by intensity (0..1) with a
    /// display gamma. Returns black outside the visible range (~380-780 nm).
    static QRgb wavelengthToColour(double nm, double intensity = 1.0);

    /// Dispersion and direction from one identified feature and a known zero-order
    /// position (all pixel values are along-axis image coordinates).
    static CameraOpticalSpectrumCalibration calibrateOnePoint(double pixel, double nm, double zeroOrderPixel);

    /// Dispersion, direction and zero-order position from two identified features.
    static CameraOpticalSpectrumCalibration calibrateTwoPoint(double pixel1, double nm1, double pixel2, double nm2);

    /// The built-in reference line sets (Balmer, He I, Na/Ca, telluric O2). User lines
    /// form an additional set with key "custom" (see parseCustomLines).
    static const QVector<CameraOpticalSpectrumRefLineSet>& referenceLineSets();

    /// Parses the opticalSpectrumCustomLines setting ("label:nm;label:nm").
    static QVector<CameraOpticalSpectrumRefLine> parseCustomLines(const QString& customLines);

    /// Resolves the opticalSpectrumRefLines selection to concrete lines. Each
    /// comma-separated token is either a set key ("balmer" = the whole set) or
    /// "key:label" for an individual line; unknown tokens are ignored.
    static QVector<CameraOpticalSpectrumRefLine> selectedReferenceLines(const QString& refLines, const QString& customLines);

    /// Index of the strongest feature (largest deviation from the window median)
    /// within +/- window samples of index; returns index itself if nothing stands out.
    static int snapToFeature(const QVector<float>& profile, int index, int window = 15);

    /// Measures the spectral line at/near the given sample: local continuum, FWHM from
    /// the interpolated half-maximum crossings, and equivalent width integrated between
    /// the continuum crossings. Invalid when no significant feature is present.
    static CameraOpticalSpectrumLineMeasurement measureLine(const QVector<float>& profile, int index);

    /// Detects significant emission/absorption features: extrema of the residual against
    /// a moving-median continuum exceeding 5x the noise, strongest first.
    static QVector<CameraOpticalSpectrumFeature> detectFeatures(const QVector<float>& profile, int maxFeatures = 12);
};

#endif // INCLUDE_FEATURE_CAMERAOPTICALSPECTRUM_H_
