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

#ifndef INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMDIALOG_H_
#define INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMDIALOG_H_

#include <deque>
#include <functional>

#include <QDialog>
#include <QTimer>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>

#include "cameraopticalspectrum.h"
#include "camerasettings.h"

class ButtonSwitch;
class QComboBox;
class QDoubleSpinBox;
class QGraphicsSimpleTextItem;
class QLabel;
class QNetworkAccessManager;
class QNetworkReply;
class QPushButton;
class QRubberBand;
class QSpinBox;

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

/**
 * \brief Dialog that plots the optical spectrum extracted from the detection RoI.
 *
 * Displays intensity vs wavelength (or vs pixel when uncalibrated) with selectable
 * reference-line overlays (Balmer series, He I, Na/Ca, telluric O2 and custom lines).
 * The pipeline supplies pixel-domain profiles (CameraOpticalSpectrumData); wavelength
 * calibration, smoothing, frame averaging and normalisation are applied here so
 * changing them re-renders without waiting for a new frame.
 *
 * The dialog edits the opticalSpectrum* fields of the CameraSettings it is given and
 * emits settingsChanged() so the owner can propagate them to the pipeline.
 */
class CameraOpticalSpectrumDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CameraOpticalSpectrumDialog(CameraSettings& settings, const CameraOpticalSpectrumData& spectrumData, QWidget* parent = nullptr);
    void updateSpectrum(const CameraOpticalSpectrumData& spectrumData);

signals:
    void settingsChanged(const QStringList& settingsKeys);

protected:
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    bool event(QEvent* event) override;

private:
    void applySettingChanged(const QString& settingsKey);
    void updateChart();
    [[nodiscard]] QAbstractAxis* activeAxisY() const;
    [[nodiscard]] double axisYMax() const;
    void ensureAxisY();
    void addFittedSeries(const QVector<QPointF>& points, const QString& name, const QColor& colour,
        Qt::PenStyle penStyle, double minX, double maxX, double normFactor, double xScale, double plotFloor, double& maxY);
    void updateFeatureAnnotations(const QVector<double>& xValues, double plotFloor);
    void positionFeatureLabels();
    void loadOverlay();
    void clearOverlay();
    void updateStrips();
    void updateColourStrip();
    void updateImageStrip();
    /// Renders a one-row strip aligned with the chart's plot area; colourAt is called per
    /// pixel with the axis value and the (fractional) profile index it maps to.
    void renderStrip(QLabel* strip, const std::function<QRgb(double axisValue, double index)>& colourAt);
    [[nodiscard]] static float interpolate(const QVector<float>& profile, double index);
    void updateReferenceLines(double minY, double maxY);
    void positionReferenceLineLabels();
    void clearReferenceLines();
    [[nodiscard]] QPointF chartValueAt(const QPointF& viewportPos) const;
    void applyZoomRect(const QRect& viewportRect);
    void cancelDrag();
    void zoomOut();
    void resetZoom();
    void updateMarker();
    void renderOneMarker(double& markerPixel, bool secondMarker, double plotFloor);
    void positionMarkerLabel();
    void positionOneMarkerLabel(QScatterSeries* series, QGraphicsSimpleTextItem* label);
    void updateRedshiftVelocityLabel();
    [[nodiscard]] QVector<float> displayProfile(const QVector<float>& profile) const;
    [[nodiscard]] double wavelengthAt(int profileIndex) const;
    [[nodiscard]] bool isCalibrated() const;
    [[nodiscard]] bool directionRedPositive() const;
    [[nodiscard]] double zeroOrderImagePosition() const;
    void exportCsv();
    void captureInstrumentResponse();
    void loadResponseFile();
    void selectResponseFile();
    void updateResponseControls();
    [[nodiscard]] QString responseFilePath() const;
    [[nodiscard]] static QString defaultResponseFilePath();
    void openReferenceDialog();
    void loadReferenceTemplate(const QString& key);
    void handleReferenceDownload(QNetworkReply* reply);
    void updateReferenceLabel(const QString& status = QString());
    [[nodiscard]] static QString referenceCachePath(const QString& key);
    void setCalibrationMode(bool active);
    void handleCalibrationClick(const QPointF& viewportPos);
    void finishCalibration();
    void applyCalibration(const CameraOpticalSpectrumCalibration& calibration, bool setZeroOrder);
    [[nodiscard]] double pixelFromViewportX(double viewportX) const;

    CameraSettings& m_settings;
    CameraOpticalSpectrumData m_spectrumData;
    std::deque<CameraOpticalSpectrumData> m_averageHistory;
    bool m_autoRedPositive = true;      ///< Latched auto direction; only updated when the centroids are decisive
    double m_zeroOrderSmoothedPx = -1.0; ///< Smoothed auto zero-order position, so the wavelength axis does not jitter with noise
    QTimer m_chartThrottleTimer;        ///< Limits frame-driven chart rebuilds; settings changes still redraw immediately
    bool m_chartUpdatePending = false;
    bool m_inUpdateChart = false;       ///< Suppresses redundant strip renders from plotAreaChanged during updateChart

    // Rubber-band zoom. The zoom range persists across frame updates (which reset the
    // axes every frame) until Reset; Zoom out steps back towards the full view.
    QRubberBand* m_rubberBand = nullptr;
    QPoint m_dragStartVp;               ///< Drag start in viewport coordinates
    bool m_dragging = false;
    bool m_zoomed = false;
    double m_zoomXMin = 0.0;
    double m_zoomXMax = 0.0;
    double m_zoomYMin = 0.0;
    double m_zoomYMax = 0.0;
    double m_fullXMin = 0.0;             ///< Full data extent, cached each updateChart for zoom-out clamping
    double m_fullXMax = 0.0;
    double m_fullYMin = 0.0;
    double m_fullYMax = 0.0;

    QChart* m_chart;
    QChartView* m_chartView;
    QValueAxis* m_axisX;
    QValueAxis* m_axisY;
    QLogValueAxis* m_axisYLog;
    QVector<QLineSeries*> m_referenceLineSeries;
    QVector<QGraphicsSimpleTextItem*> m_referenceLineLabels;
    QVector<double> m_referenceLineWavelengths;
    double m_markerPixel = -1.0;        ///< Marker A position along the dispersion axis in image pixels; -1 = none
    double m_markerPixelB = -1.0;       ///< Marker B (Ctrl+click) position; -1 = none
    QScatterSeries* m_markerSeries = nullptr;
    QScatterSeries* m_markerSeriesB = nullptr;
    QGraphicsSimpleTextItem* m_markerLabel = nullptr;
    QGraphicsSimpleTextItem* m_markerLabelB = nullptr;
    QScatterSeries* m_featureSeries = nullptr;          ///< Auto-identified feature markers
    QVector<QGraphicsSimpleTextItem*> m_featureLabels;
    QVector<QPointF> m_featureAnchors;                  ///< Chart coordinates the feature labels anchor to
    QVector<QPointF> m_overlayPoints;   ///< Loaded comparison spectrum: wavelength nm vs value
    QString m_overlayName;
    double m_displayNormFactor = 1.0;   ///< Normalisation factor applied to the plotted profiles
    QLabel* m_colourStrip;
    QLabel* m_imageStrip;
    QVector<float> m_displayLuminance; ///< Averaged+smoothed luminance profile as displayed, for the colour strip
    QVector<float> m_displayRed;       ///< Averaged+smoothed R/G/B profiles as displayed, for the image strip
    QVector<float> m_displayGreen;
    QVector<float> m_displayBlue;
    QVector<double> m_displayXValues;  ///< X axis value per profile index as displayed
    // Response correction blanks (zeroes) samples outside the captured response's
    // coverage; feature detection and line measurement must stay inside the covered
    // range or the hard blank->signal step at its edges reads as a strong feature.
    int m_displayValidFirst = 0;       ///< First profile index with response coverage
    int m_displayValidLast = -1;       ///< Last profile index with response coverage; full range when no correction

    QPushButton* m_referenceButton;
    QLabel* m_referenceLabel;
    QLabel* m_warningLabel;
    QStringList m_dataWarnings; ///< Extraction warnings from the latest frame, shown with any display warnings
    QNetworkAccessManager* m_referenceNetworkManager = nullptr;
    QVector<QPointF> m_referencePoints; ///< Loaded template: wavelength nm vs normalised flux
    QString m_referenceLoadedKey;       ///< Template key m_referencePoints was loaded from
    QVector<QPointF> m_responsePoints;  ///< Captured instrument response (luminance): wavelength nm vs relative response
    QVector<QPointF> m_responseRed;     ///< Per-channel responses, normalised jointly so relative sensitivities survive
    QVector<QPointF> m_responseGreen;
    QVector<QPointF> m_responseBlue;
    QVector<float> m_displayLuminanceRaw; ///< Luminance before response correction, for response capture
    QVector<float> m_displayRedRaw;       ///< Channels before response correction, for per-channel capture
    QVector<float> m_displayGreenRaw;
    QVector<float> m_displayBlueRaw;
    QPushButton* m_captureResponseButton;
    ButtonSwitch* m_applyResponseCheck;
    QPushButton* m_responseFileButton;
    QPushButton* m_calibrateButton;
    QVector<QPointF> m_calibrationClicks; ///< x = along-axis image pixel, y = wavelength in nm
    ButtonSwitch* m_zeroOrderAutoCheck;
    QDoubleSpinBox* m_zeroOrderSpin;
    QDoubleSpinBox* m_dispersionSpin;
    QComboBox* m_directionCombo;
    QSpinBox* m_apertureSpin;
    ButtonSwitch* m_backgroundSubCheck;
    QSpinBox* m_smoothingSpin;
    QSpinBox* m_averageFramesSpin;
    ButtonSwitch* m_normalizeCheck;
    ButtonSwitch* m_logCheck;
    ButtonSwitch* m_identifyCheck;
    ButtonSwitch* m_colourStripCheck;
    ButtonSwitch* m_imageStripCheck;
    QLabel* m_overlayLabel;
    QPushButton* m_overlayClearButton;
    QPushButton* m_zoomOutButton = nullptr;
    QPushButton* m_zoomResetButton = nullptr;
    QDoubleSpinBox* m_redshiftSpin;
    QLabel* m_redshiftVelocityLabel;
    ButtonSwitch* m_luminanceCheck;
    ButtonSwitch* m_redCheck;
    ButtonSwitch* m_greenCheck;
    ButtonSwitch* m_blueCheck;

    bool m_updatingControls = false;
};

#endif // INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMDIALOG_H_
