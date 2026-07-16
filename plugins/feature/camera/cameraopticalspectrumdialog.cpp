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

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QGraphicsSimpleTextItem>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QDir>
#include <QMessageBox>
#include <QMouseEvent>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRegularExpression>
#include <QPixmap>
#include <QScopedValueRollback>
#include <QStandardPaths>
#include <QPushButton>
#include <QRubberBand>
#include <QResizeEvent>
#include <QSpinBox>
#include <QTextStream>
#include <QVBoxLayout>
#include <QtCharts/QLegendMarker>

#include "gui/dialogpositioner.h"

#include "cameraopticalspectrumdialog.h"
#include "cameraopticalspectrumlibrary.h"
#include "cameraopticalspectrumlinesdialog.h"
#include "cameraopticalspectrumreferencedialog.h"

namespace {

constexpr int kColourStripHeight = 20;
constexpr double kVisibleMinNm = 380.0;
constexpr double kVisibleMaxNm = 780.0;
// Display gamma; lifts faint detail in the strips so absorption features stay visible
constexpr double kStripGamma = 0.8;
// CDS puts browser-like user agents behind a proof-of-work anti-bot challenge, which a
// plain request cannot answer; identifying as SDRangel is passed straight through.
const QByteArray kUserAgent = QByteArrayLiteral("SDRangel (+https://github.com/f4exb/sdrangel)");

} // namespace

CameraOpticalSpectrumDialog::CameraOpticalSpectrumDialog(CameraSettings& settings, const CameraOpticalSpectrumData& spectrumData, QWidget* parent)
    : QDialog(parent),
      m_settings(settings),
      m_chart(new QChart()),
      m_chartView(new QChartView(m_chart, this)),
      m_axisX(new QValueAxis()),
      m_axisY(new QValueAxis()),
      m_axisYLog(new QLogValueAxis())
{
    setWindowTitle(tr("Optical spectrum"));
    resize(800, 520);
    setModal(false);
    new DialogPositioner(this, true);

    m_chart->setTheme(QChart::ChartThemeDark);
    m_chart->legend()->setVisible(true);

    m_axisX->setTitleText(tr("Pixel"));
    m_axisY->setTitleText(tr("Intensity"));
    m_axisYLog->setTitleText(tr("Intensity"));
    m_axisYLog->setBase(10.0);
    m_axisYLog->setLabelFormat(QStringLiteral("%g"));
    // Minor gridlines keep a sub-decade range readable (a snug continuum spectrum may
    // span less than one power of ten)
    m_axisYLog->setMinorTickCount(8);
    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    m_chartView->setRenderHint(QPainter::Antialiasing);
    m_chartView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    connect(m_chart, &QChart::plotAreaChanged, this, [this]() {
        positionReferenceLineLabels();
        positionMarkerLabel();
        // updateChart renders the strips itself once it finishes
        if (!m_inUpdateChart) {
            updateStrips();
        }
    });

    // Frame-driven chart rebuilds are throttled: QtCharts series churn at full camera
    // rate costs real CPU, and the eye cannot follow it anyway. Data (averaging
    // history) still updates every frame; only the redraw is limited.
    m_chartThrottleTimer.setSingleShot(true);
    m_chartThrottleTimer.setInterval(100);
    connect(&m_chartThrottleTimer, &QTimer::timeout, this, [this]() {
        if (m_chartUpdatePending)
        {
            m_chartUpdatePending = false;
            updateChart();
            m_chartThrottleTimer.start();
        }
    });
    m_chartView->setToolTip(tr("Drag a rectangle: zoom in (use Zoom out / Reset zoom below).\n"
        "Click: place a readout marker showing wavelength, intensity and line measurements (FWHM, equivalent width).\n"
        "Shift+click: snap the marker to the nearest spectral feature.\n"
        "Ctrl+click: place a second marker, showing the differences from the first.\n"
        "Right-click: remove the markers. While calibrating, clicks pick the reference points instead."));

    // Strip showing the colours actually extracted from the image along the dispersion
    // axis - effectively the observed spectrum, useful for lining up reference lines.
    m_imageStrip = new QLabel(this);
    m_imageStrip->setFixedHeight(kColourStripHeight);
    m_imageStrip->setToolTip(tr("Colours extracted from the image along the dispersion axis (the observed spectrum),\n"
        "scaled to the brightest sample. Use it to line the reference lines up with the real features."));

    // Strip showing the colour of each wavelength, with brightness following the plotted
    // luminance. Both strips are aligned with the chart's plot area.
    m_colourStrip = new QLabel(this);
    m_colourStrip->setFixedHeight(kColourStripHeight);
    m_colourStrip->setToolTip(tr("Approximate colour of each wavelength, with brightness following the plotted luminance.\nGrey outside the visible range or when uncalibrated."));

    // Calibration controls
    m_zeroOrderAutoCheck = new QCheckBox(tr("Auto zero order"), this);
    m_zeroOrderAutoCheck->setToolTip(tr("Locate the zero-order (undispersed) source image as the strongest peak in the RoI.\nUntick to enter its pixel position manually (it may lie outside the RoI)."));
    m_zeroOrderSpin = new QDoubleSpinBox(this);
    m_zeroOrderSpin->setRange(-100000.0, 100000.0);
    m_zeroOrderSpin->setDecimals(1);
    m_zeroOrderSpin->setSuffix(tr(" px"));
    m_zeroOrderSpin->setToolTip(tr("Zero-order position along the dispersion axis in image pixels"));
    m_dispersionSpin = new QDoubleSpinBox(this);
    m_dispersionSpin->setRange(0.0, 100.0);
    m_dispersionSpin->setDecimals(4);
    m_dispersionSpin->setSingleStep(0.01);
    m_dispersionSpin->setSuffix(tr(" nm/px"));
    m_dispersionSpin->setSpecialValueText(tr("Uncalibrated"));
    m_dispersionSpin->setToolTip(tr("Dispersion of the grating at the sensor. 0 = uncalibrated (X axis in pixels).\nCalibrate from a known line: dispersion = wavelength / distance from zero order in pixels."));
    m_directionCombo = new QComboBox(this);
    m_directionCombo->addItem(tr("Auto (colour)"));
    m_directionCombo->addItem(tr("Red towards +pixels"));
    m_directionCombo->addItem(tr("Red towards -pixels"));
    m_directionCombo->setToolTip(tr("Which way wavelength increases along the dispersion axis.\nAuto compares the red and blue channel centroids."));
    m_calibrateButton = new QPushButton(tr("Calibrate..."), this);
    m_calibrateButton->setCheckable(true);
    m_calibrateButton->setToolTip(tr("Calculate the dispersion by clicking reference points on the chart.\n"
        "Click one identified feature and enter its wavelength to calibrate against the zero-order position,\n"
        "or click two features to also solve for the zero-order position and direction."));
    connect(m_calibrateButton, &QPushButton::toggled, this, [this](bool checked) {
        if (checked) {
            setCalibrationMode(true);
        } else if (m_calibrationClicks.size() == 1) {
            finishCalibration();
        } else {
            setCalibrationMode(false);
        }
    });

    // Extraction controls
    m_apertureSpin = new QSpinBox(this);
    m_apertureSpin->setRange(0, 256);
    m_apertureSpin->setSpecialValueText(tr("All"));
    m_apertureSpin->setToolTip(tr("Rows summed across the spectrum trace (centred on it automatically).\n0 sums the whole RoI height (background subtraction is then unavailable)."));
    m_backgroundSubCheck = new QCheckBox(tr("Subtract background"), this);
    m_backgroundSubCheck->setToolTip(tr("Subtract the per-column sky background estimated from the RoI rows outside the aperture.\n"
        "Skipped automatically when the source fills the RoI (such as a discharge tube whose lines span the frame),\n"
        "as those rows carry the signal rather than a background."));
    m_warningLabel = new QLabel(this);
    m_warningLabel->setStyleSheet(QStringLiteral("color: rgb(240, 170, 60);"));
    m_warningLabel->setWordWrap(true);
    m_smoothingSpin = new QSpinBox(this);
    m_smoothingSpin->setRange(1, 99);
    m_smoothingSpin->setSpecialValueText(tr("Off"));
    m_smoothingSpin->setToolTip(tr("Moving-average width in samples applied to the displayed profile"));
    m_averageFramesSpin = new QSpinBox(this);
    m_averageFramesSpin->setRange(1, 100);
    m_averageFramesSpin->setSpecialValueText(tr("Off"));
    m_averageFramesSpin->setToolTip(tr("Average the displayed profile over this many frames to reduce noise"));
    m_normalizeCheck = new QCheckBox(tr("Normalise"), this);
    m_normalizeCheck->setToolTip(tr("Scale the displayed profile to a peak of 1.0"));
    m_logCheck = new QCheckBox(tr("Log"), this);
    m_logCheck->setToolTip(tr("Plot intensity on a logarithmic axis, keeping faint features legible next to bright ones"));
    m_identifyCheck = new QCheckBox(tr("Identify"), this);
    m_identifyCheck->setToolTip(tr("Detect significant emission/absorption features and label those matching known reference lines\n(redshift is applied to source lines). Strong unmatched features are labelled with their wavelength.\nRequires wavelength calibration."));
    m_imageStripCheck = new QCheckBox(tr("Image strip"), this);
    m_imageStripCheck->setToolTip(tr("Show a strip below the chart with the colours extracted from the image (the observed spectrum)"));
    m_colourStripCheck = new QCheckBox(tr("Wavelength strip"), this);
    m_colourStripCheck->setToolTip(tr("Show a strip below the chart with the colour of each wavelength, brightness following the plotted luminance"));

    // Redshift applied to the source reference lines
    m_redshiftSpin = new QDoubleSpinBox(this);
    m_redshiftSpin->setRange(-0.9, 10.0);
    m_redshiftSpin->setDecimals(4);
    m_redshiftSpin->setSingleStep(0.001);
    m_redshiftSpin->setToolTip(tr("Redshift z applied to the source reference lines, which are displayed at rest wavelength x (1 + z).\n"
        "Adjust until the lines match the observed features to estimate the source's redshift.\n"
        "Redshifted lines are tinted orange, blueshifted lines blue.\n"
        "Terrestrial lines (telluric O2/H2O and aurora/airglow) are never shifted, so they stay anchored as a check.\n"
        "Negative values model blueshift. The equivalent recession velocity is shown alongside."));
    m_redshiftVelocityLabel = new QLabel(this);
    m_redshiftVelocityLabel->setToolTip(tr("Relativistic recession velocity equivalent to the entered redshift"));

    // Channel selection
    m_luminanceCheck = new QCheckBox(tr("Luminance"), this);
    m_redCheck = new QCheckBox(tr("R"), this);
    m_greenCheck = new QCheckBox(tr("G"), this);
    m_blueCheck = new QCheckBox(tr("B"), this);
    m_luminanceCheck->setChecked(true);

    m_updatingControls = true;
    m_zeroOrderAutoCheck->setChecked(m_settings.m_opticalSpectrumZeroOrderAuto);
    m_zeroOrderSpin->setValue(m_settings.m_opticalSpectrumZeroOrderX);
    m_zeroOrderSpin->setEnabled(!m_settings.m_opticalSpectrumZeroOrderAuto);
    m_dispersionSpin->setValue(m_settings.m_opticalSpectrumDispersion);
    m_directionCombo->setCurrentIndex(static_cast<int>(m_settings.m_opticalSpectrumDirection));
    m_apertureSpin->setValue(m_settings.m_opticalSpectrumApertureRows);
    m_backgroundSubCheck->setChecked(m_settings.m_opticalSpectrumBackgroundSub);
    m_smoothingSpin->setValue(m_settings.m_opticalSpectrumSmoothing);
    m_averageFramesSpin->setValue(m_settings.m_opticalSpectrumAverageFrames);
    m_normalizeCheck->setChecked(m_settings.m_opticalSpectrumNormalize);
    m_logCheck->setChecked(m_settings.m_opticalSpectrumLogY);
    m_identifyCheck->setChecked(m_settings.m_opticalSpectrumAutoIdentify);
    m_colourStripCheck->setChecked(m_settings.m_opticalSpectrumColourStrip);
    m_colourStrip->setVisible(m_settings.m_opticalSpectrumColourStrip);
    m_imageStripCheck->setChecked(m_settings.m_opticalSpectrumImageStrip);
    m_imageStrip->setVisible(m_settings.m_opticalSpectrumImageStrip);
    m_redshiftSpin->setValue(m_settings.m_opticalSpectrumRedshift);
    m_updatingControls = false;
    updateRedshiftVelocityLabel();

    connect(m_zeroOrderAutoCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumZeroOrderAuto = checked;
        m_zeroOrderSpin->setEnabled(!checked);
        applySettingChanged(QStringLiteral("opticalSpectrumZeroOrderAuto"));
    });
    connect(m_zeroOrderSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        m_settings.m_opticalSpectrumZeroOrderX = value;
        applySettingChanged(QStringLiteral("opticalSpectrumZeroOrderX"));
    });
    connect(m_dispersionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        m_settings.m_opticalSpectrumDispersion = value;
        applySettingChanged(QStringLiteral("opticalSpectrumDispersion"));
    });
    connect(m_directionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        m_settings.m_opticalSpectrumDirection = static_cast<CameraSettings::OpticalSpectrumDirection>(index);
        applySettingChanged(QStringLiteral("opticalSpectrumDirection"));
    });
    connect(m_apertureSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        m_settings.m_opticalSpectrumApertureRows = value;
        // Frames already in the averaging history were extracted with the old
        // aperture; do not blend the two extraction configurations together
        m_averageHistory.clear();
        applySettingChanged(QStringLiteral("opticalSpectrumApertureRows"));
    });
    connect(m_backgroundSubCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumBackgroundSub = checked;
        m_averageHistory.clear();
        applySettingChanged(QStringLiteral("opticalSpectrumBackgroundSub"));
    });
    connect(m_smoothingSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        m_settings.m_opticalSpectrumSmoothing = value;
        applySettingChanged(QStringLiteral("opticalSpectrumSmoothing"));
    });
    connect(m_averageFramesSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        m_settings.m_opticalSpectrumAverageFrames = value;
        applySettingChanged(QStringLiteral("opticalSpectrumAverageFrames"));
    });
    connect(m_normalizeCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumNormalize = checked;
        applySettingChanged(QStringLiteral("opticalSpectrumNormalize"));
    });
    connect(m_logCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumLogY = checked;
        applySettingChanged(QStringLiteral("opticalSpectrumLogY"));
    });
    connect(m_identifyCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumAutoIdentify = checked;
        applySettingChanged(QStringLiteral("opticalSpectrumAutoIdentify"));
    });
    connect(m_colourStripCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumColourStrip = checked;
        m_colourStrip->setVisible(checked);
        applySettingChanged(QStringLiteral("opticalSpectrumColourStrip"));
    });
    connect(m_imageStripCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumImageStrip = checked;
        m_imageStrip->setVisible(checked);
        applySettingChanged(QStringLiteral("opticalSpectrumImageStrip"));
    });
    connect(m_redshiftSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        m_settings.m_opticalSpectrumRedshift = value;
        updateRedshiftVelocityLabel();
        applySettingChanged(QStringLiteral("opticalSpectrumRedshift"));
    });
    const auto displayOnlyChanged = [this]() { updateChart(); };
    connect(m_luminanceCheck, &QCheckBox::toggled, this, displayOnlyChanged);
    connect(m_redCheck, &QCheckBox::toggled, this, displayOnlyChanged);
    connect(m_greenCheck, &QCheckBox::toggled, this, displayOnlyChanged);
    connect(m_blueCheck, &QCheckBox::toggled, this, displayOnlyChanged);

    auto* calibrationLayout = new QHBoxLayout();
    calibrationLayout->addWidget(m_zeroOrderAutoCheck);
    calibrationLayout->addWidget(m_zeroOrderSpin);
    calibrationLayout->addWidget(new QLabel(tr("Dispersion"), this));
    calibrationLayout->addWidget(m_dispersionSpin);
    calibrationLayout->addWidget(new QLabel(tr("Direction"), this));
    calibrationLayout->addWidget(m_directionCombo);
    calibrationLayout->addWidget(m_calibrateButton);
    calibrationLayout->addStretch();

    auto* extractionLayout = new QHBoxLayout();
    extractionLayout->addWidget(new QLabel(tr("Aperture"), this));
    extractionLayout->addWidget(m_apertureSpin);
    extractionLayout->addWidget(m_backgroundSubCheck);
    extractionLayout->addWidget(new QLabel(tr("Smooth"), this));
    extractionLayout->addWidget(m_smoothingSpin);
    extractionLayout->addWidget(new QLabel(tr("Average"), this));
    extractionLayout->addWidget(m_averageFramesSpin);
    extractionLayout->addWidget(m_normalizeCheck);
    extractionLayout->addWidget(m_logCheck);
    extractionLayout->addWidget(m_identifyCheck);
    extractionLayout->addWidget(m_imageStripCheck);
    extractionLayout->addWidget(m_colourStripCheck);
    extractionLayout->addStretch();

    auto* displayLayout = new QHBoxLayout();
    displayLayout->addWidget(m_luminanceCheck);
    displayLayout->addWidget(m_redCheck);
    displayLayout->addWidget(m_greenCheck);
    displayLayout->addWidget(m_blueCheck);
    displayLayout->addSpacing(16);
    displayLayout->addWidget(new QLabel(tr("Redshift z"), this));
    displayLayout->addWidget(m_redshiftSpin);
    displayLayout->addWidget(m_redshiftVelocityLabel);
    auto* linesButton = new QPushButton(tr("Lines..."), this);
    linesButton->setToolTip(tr("Select which reference lines to overlay on the chart"));
    connect(linesButton, &QPushButton::clicked, this, [this]() {
        CameraOpticalSpectrumLinesDialog dialog(m_settings.m_opticalSpectrumRefLines, m_settings.m_opticalSpectrumCustomLines, this);
        // Apply custom-line edits before selection so a re-render sees the new lines
        connect(&dialog, &CameraOpticalSpectrumLinesDialog::customLinesChanged, this, [this](const QString& customLines) {
            m_settings.m_opticalSpectrumCustomLines = customLines;
            applySettingChanged(QStringLiteral("opticalSpectrumCustomLines"));
        });
        connect(&dialog, &CameraOpticalSpectrumLinesDialog::selectionChanged, this, [this](const QString& refLines) {
            m_settings.m_opticalSpectrumRefLines = refLines;
            applySettingChanged(QStringLiteral("opticalSpectrumRefLines"));
        });
        dialog.exec();
    });
    displayLayout->addWidget(linesButton);

    m_referenceButton = new QPushButton(tr("Reference..."), this);
    m_referenceButton->setToolTip(tr("Overlay a reference spectrum for a star or spectral type, downloaded from the Pickles library"));
    connect(m_referenceButton, &QPushButton::clicked, this, [this]() { openReferenceDialog(); });
    m_referenceLabel = new QLabel(this);
    displayLayout->addWidget(m_referenceButton);
    displayLayout->addWidget(m_referenceLabel, 1);

    // Instrument response: capture from a reference star, then divide it out. The
    // tooltips (set in updateResponseControls) name the active response file.
    m_captureResponseButton = new QPushButton(tr("Capture response"), this);
    connect(m_captureResponseButton, &QPushButton::clicked, this, [this]() { captureInstrumentResponse(); });
    m_applyResponseCheck = new QCheckBox(tr("Apply response"), this);
    connect(m_applyResponseCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_settings.m_opticalSpectrumApplyResponse = checked;
        applySettingChanged(QStringLiteral("opticalSpectrumApplyResponse"));
    });
    m_responseFileButton = new QPushButton(tr("..."), this);
    m_responseFileButton->setMaximumWidth(28);
    connect(m_responseFileButton, &QPushButton::clicked, this, [this]() { selectResponseFile(); });
    displayLayout->addWidget(m_captureResponseButton);
    displayLayout->addWidget(m_applyResponseCheck);
    displayLayout->addWidget(m_responseFileButton);
    displayLayout->addStretch();

    auto* exportButton = new QPushButton(tr("Export CSV..."), this);
    connect(exportButton, &QPushButton::clicked, this, [this]() { exportCsv(); });
    auto* overlayButton = new QPushButton(tr("Load overlay..."), this);
    overlayButton->setToolTip(tr("Overlay a previously exported spectrum CSV for comparison.\nThe file must have been exported with the wavelength scale calibrated."));
    connect(overlayButton, &QPushButton::clicked, this, [this]() { loadOverlay(); });
    m_overlayClearButton = new QPushButton(tr("\303\227"), this);
    m_overlayClearButton->setMaximumWidth(24);
    m_overlayClearButton->setToolTip(tr("Remove the loaded overlay spectrum"));
    m_overlayClearButton->setVisible(false);
    connect(m_overlayClearButton, &QPushButton::clicked, this, [this]() { clearOverlay(); });
    m_overlayLabel = new QLabel(this);
    m_zoomOutButton = new QPushButton(tr("Zoom out"), this);
    m_zoomOutButton->setToolTip(tr("Zoom out one step. Drag a rectangle on the chart to zoom in."));
    m_zoomOutButton->setEnabled(false);
    connect(m_zoomOutButton, &QPushButton::clicked, this, [this]() { zoomOut(); });
    m_zoomResetButton = new QPushButton(tr("Reset zoom"), this);
    m_zoomResetButton->setToolTip(tr("Restore the full view"));
    m_zoomResetButton->setEnabled(false);
    connect(m_zoomResetButton, &QPushButton::clicked, this, [this]() { resetZoom(); });
    auto* closeButton = new QPushButton(tr("Close"), this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::close);

    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(exportButton);
    buttonLayout->addWidget(overlayButton);
    buttonLayout->addWidget(m_overlayClearButton);
    buttonLayout->addWidget(m_overlayLabel);
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_zoomOutButton);
    buttonLayout->addWidget(m_zoomResetButton);
    buttonLayout->addWidget(closeButton);

    auto* layout = new QVBoxLayout();
    layout->addLayout(calibrationLayout);
    layout->addLayout(extractionLayout);
    layout->addWidget(m_warningLabel);
    layout->addWidget(m_chartView, 1);
    layout->addWidget(m_imageStrip);
    layout->addWidget(m_colourStrip);
    layout->addLayout(displayLayout);
    layout->addLayout(buttonLayout);
    setLayout(layout);

    m_chartView->viewport()->installEventFilter(this);

    loadResponseFile();
    m_updatingControls = true;
    m_applyResponseCheck->setChecked(m_settings.m_opticalSpectrumApplyResponse);
    m_updatingControls = false;
    updateResponseControls();

    updateReferenceLabel();
    if (!m_settings.m_opticalSpectrumReferenceTemplate.isEmpty()) {
        loadReferenceTemplate(m_settings.m_opticalSpectrumReferenceTemplate);
    }

    updateSpectrum(spectrumData);
}

QString CameraOpticalSpectrumDialog::referenceCachePath(const QString& key)
{
    const QString baseDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    const QString extension = CameraOpticalSpectrumLibrary::isEmissionTemplate(key)
        ? QStringLiteral("fit")
        : QStringLiteral("dat.gz");
    return QDir(baseDir).filePath(QStringLiteral("camera/pickles/%1.%2").arg(key, extension));
}

void CameraOpticalSpectrumDialog::updateReferenceLabel(const QString& status)
{
    if (!status.isEmpty())
    {
        m_referenceLabel->setText(status);
        return;
    }
    m_referenceLabel->setText(CameraOpticalSpectrumLibrary::templateDisplayName(m_settings.m_opticalSpectrumReferenceTemplate));
}

QString CameraOpticalSpectrumDialog::defaultResponseFilePath()
{
    const QString baseDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    return QDir(baseDir).filePath(QStringLiteral("camera/spectrum-response.csv"));
}

QString CameraOpticalSpectrumDialog::responseFilePath() const
{
    return m_settings.m_opticalSpectrumResponseFile.isEmpty()
        ? defaultResponseFilePath()
        : m_settings.m_opticalSpectrumResponseFile;
}

void CameraOpticalSpectrumDialog::selectResponseFile()
{
    // Save-style dialog without the overwrite prompt: an existing file is selected to
    // load, a new name creates the file on the next capture. Nothing is overwritten here.
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Response file"),
        responseFilePath(),
        tr("CSV files (*.csv)"),
        nullptr,
        QFileDialog::DontConfirmOverwrite);
    if (fileName.isEmpty() || (fileName == responseFilePath())) {
        return;
    }

    // Store the default path as empty so the setting stays portable
    m_settings.m_opticalSpectrumResponseFile = (fileName == defaultResponseFilePath()) ? QString() : fileName;
    loadResponseFile();
    updateResponseControls();
    applySettingChanged(QStringLiteral("opticalSpectrumResponseFile"));
}

void CameraOpticalSpectrumDialog::updateResponseControls()
{
    const QString filePath = responseFilePath();
    const QString fileName = QFileInfo(filePath).fileName();
    m_applyResponseCheck->setEnabled(!m_responsePoints.isEmpty());
    m_responseFileButton->setToolTip(tr("Select the instrument response file. Keep separate files for different\n"
        "cameras or conditions (e.g. vega-asi294.csv, tube-bench.csv).\n"
        "Current: %1%2").arg(filePath, m_responsePoints.isEmpty() ? tr(" (no response captured yet)") : QString()));
    m_applyResponseCheck->setToolTip(tr("Divide the displayed luminance by the captured instrument response,\n"
        "correcting the continuum shape for the sensor, Bayer filters and grating.\nUsing %1").arg(fileName));
    m_captureResponseButton->setToolTip(tr("Measure the instrument response (sensor QE x Bayer filters x grating blaze) by dividing the\n"
        "current spectrum by the selected reference template. Point the camera at the reference star,\n"
        "calibrate the wavelength scale, select the star's spectral type with Reference..., then capture.\n"
        "The star's absorption lines and the telluric bands are masked and the curve heavily smoothed.\n"
        "Saves to %1").arg(fileName));
}

void CameraOpticalSpectrumDialog::loadResponseFile()
{
    m_responsePoints.clear();
    m_responseRed.clear();
    m_responseGreen.clear();
    m_responseBlue.clear();
    QFile file(responseFilePath());
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return;
    }
    while (!file.atEnd())
    {
        const QList<QByteArray> fields = file.readLine().trimmed().split(',');
        if (fields.size() < 2) {
            continue;
        }
        bool okNm = false;
        bool okValue = false;
        const double nm = fields.at(0).toDouble(&okNm);
        const double value = fields.at(1).toDouble(&okValue);
        if (!okNm || !okValue) {
            continue;
        }
        m_responsePoints.append(QPointF(nm, value));

        // Per-channel columns (files from older versions carry luminance only). A zero
        // means "outside this channel's usable range" and is dropped so responseAt
        // reports no coverage there rather than interpolating through zero.
        if (fields.size() >= 5)
        {
            bool okR = false;
            bool okG = false;
            bool okB = false;
            const double r = fields.at(2).toDouble(&okR);
            const double g = fields.at(3).toDouble(&okG);
            const double b = fields.at(4).toDouble(&okB);
            if (okR && (r > 0.0)) {
                m_responseRed.append(QPointF(nm, r));
            }
            if (okG && (g > 0.0)) {
                m_responseGreen.append(QPointF(nm, g));
            }
            if (okB && (b > 0.0)) {
                m_responseBlue.append(QPointF(nm, b));
            }
        }
    }
    // responseAt() interpolates by binary search, which requires ascending wavelengths;
    // user-selected files may be hand-edited or concatenated
    const auto byWavelength = [](const QPointF& a, const QPointF& b) { return a.x() < b.x(); };
    std::sort(m_responsePoints.begin(), m_responsePoints.end(), byWavelength);
    std::sort(m_responseRed.begin(), m_responseRed.end(), byWavelength);
    std::sort(m_responseGreen.begin(), m_responseGreen.end(), byWavelength);
    std::sort(m_responseBlue.begin(), m_responseBlue.end(), byWavelength);
}

void CameraOpticalSpectrumDialog::captureInstrumentResponse()
{
    if (!isCalibrated())
    {
        QMessageBox::information(this, tr("Capture response"),
            tr("Calibrate the wavelength scale first - the response is a function of wavelength."));
        return;
    }
    if (m_referencePoints.isEmpty())
    {
        QMessageBox::information(this, tr("Capture response"),
            tr("Select the reference star's spectral type with Reference... first,\nso there is a template to divide the observed spectrum by."));
        return;
    }
    if (m_displayLuminanceRaw.size() < 2)
    {
        QMessageBox::information(this, tr("Capture response"), tr("No spectrum data to capture from."));
        return;
    }

    // Mask the reference star's hydrogen lines and the telluric bands, plus every
    // currently selected source/custom line, so none of them dent the smooth response
    // curve (a K-type reference's Ca/Na lines would imprint otherwise). Duplicates are
    // removed inside computeInstrumentResponse.
    QVector<double> maskedWavelengths;
    for (const CameraOpticalSpectrumRefLineSet& set : CameraOpticalSpectrumExtractor::referenceLineSets())
    {
        if ((set.m_key == QLatin1String("balmer")) || set.m_terrestrial)
        {
            for (const CameraOpticalSpectrumRefLine& line : set.m_lines) {
                maskedWavelengths.append(line.m_nm);
            }
        }
    }
    for (const CameraOpticalSpectrumRefLine& line : CameraOpticalSpectrumExtractor::selectedReferenceLines(
        m_settings.m_opticalSpectrumRefLines, m_settings.m_opticalSpectrumCustomLines)) {
        maskedWavelengths.append(line.m_nm);
    }

    const QVector<QPointF> response = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        m_displayXValues, m_displayLuminanceRaw, m_referencePoints, maskedWavelengths, 12.0, 20.0);
    if (response.isEmpty())
    {
        QMessageBox::warning(this, tr("Capture response"),
            tr("Could not measure a response - the observed spectrum and the template overlap too little.\n"
               "Check the wavelength calibration, and that the spectrum is bright and unsaturated."));
        return;
    }

    // Per-channel responses, rescaled to a shared normalisation (largest channel peak
    // = 1) so the relative channel sensitivities are preserved: correcting each
    // channel by these recovers the true hue, not a re-white-balanced one.
    double peakR = 0.0;
    double peakG = 0.0;
    double peakB = 0.0;
    QVector<QPointF> responseR = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        m_displayXValues, m_displayRedRaw, m_referencePoints, maskedWavelengths, 12.0, 20.0, &peakR);
    QVector<QPointF> responseG = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        m_displayXValues, m_displayGreenRaw, m_referencePoints, maskedWavelengths, 12.0, 20.0, &peakG);
    QVector<QPointF> responseB = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        m_displayXValues, m_displayBlueRaw, m_referencePoints, maskedWavelengths, 12.0, 20.0, &peakB);
    const double maxPeak = qMax(peakR, qMax(peakG, peakB));
    if (maxPeak > 0.0)
    {
        const auto rescale = [maxPeak](QVector<QPointF>& response, double peak) {
            const double factor = peak / maxPeak;
            for (QPointF& point : response) {
                point.setY(point.y() * factor);
            }
        };
        rescale(responseR, peakR);
        rescale(responseG, peakG);
        rescale(responseB, peakB);
    }

    const QString filePath = responseFilePath();
    QDir().mkpath(QFileInfo(filePath).absolutePath());
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        // Channel curves are written on the luminance curve's wavelength grid
        QTextStream out(&file);
        out << "wavelength_nm,response,response_r,response_g,response_b\n";
        for (const QPointF& point : response)
        {
            out << point.x() << ',' << QString::number(point.y(), 'f', 5)
                << ',' << QString::number(CameraOpticalSpectrumLibrary::responseAt(responseR, point.x()), 'f', 5)
                << ',' << QString::number(CameraOpticalSpectrumLibrary::responseAt(responseG, point.x()), 'f', 5)
                << ',' << QString::number(CameraOpticalSpectrumLibrary::responseAt(responseB, point.x()), 'f', 5)
                << '\n';
        }
    }

    m_responsePoints = response;
    m_responseRed = responseR;
    m_responseGreen = responseG;
    m_responseBlue = responseB;
    updateResponseControls();
    m_applyResponseCheck->setChecked(true); // also sets the setting via the toggled handler
    updateChart();
}

void CameraOpticalSpectrumDialog::openReferenceDialog()
{
    CameraOpticalSpectrumReferenceDialog dialog(m_settings.m_opticalSpectrumReferenceTemplate, this);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    const QString key = dialog.selectedTemplate();
    if (key == m_settings.m_opticalSpectrumReferenceTemplate) {
        return;
    }

    m_settings.m_opticalSpectrumReferenceTemplate = key;
    applySettingChanged(QStringLiteral("opticalSpectrumReferenceTemplate"));

    if (key.isEmpty())
    {
        m_referencePoints.clear();
        m_referenceLoadedKey.clear();
        updateReferenceLabel();
        updateChart();
    }
    else
    {
        loadReferenceTemplate(key);
    }
}

namespace {

// Stellar (Pickles) templates are gzipped ASCII; emission-object (SDSS) templates are FITS
QVector<QPointF> parseTemplateBytes(const QString& key, const QByteArray& bytes)
{
    if (CameraOpticalSpectrumLibrary::isEmissionTemplate(key)) {
        return CameraOpticalSpectrumLibrary::parseSdssTemplateFits(bytes);
    }
    return CameraOpticalSpectrumLibrary::parseSpectrumData(CameraOpticalSpectrumLibrary::gunzip(bytes));
}

} // namespace

void CameraOpticalSpectrumDialog::loadReferenceTemplate(const QString& key)
{
    if (key.isEmpty() || (key == m_referenceLoadedKey)) {
        return;
    }
    if (!CameraOpticalSpectrumLibrary::findTemplate(key) && !CameraOpticalSpectrumLibrary::isEmissionTemplate(key)) {
        return;
    }

    // Templates are small and never change, so a downloaded one is cached for good
    const QString cachePath = referenceCachePath(key);
    QFile cacheFile(cachePath);
    if (cacheFile.exists() && cacheFile.open(QIODevice::ReadOnly))
    {
        const QByteArray bytes = cacheFile.readAll();
        cacheFile.close();
        const QVector<QPointF> points = parseTemplateBytes(key, bytes);
        if (!points.isEmpty())
        {
            m_referencePoints = points;
            m_referenceLoadedKey = key;
            updateReferenceLabel();
            updateChart();
            return;
        }
        // Cached file is unusable (truncated or corrupt); fall through and re-download
    }

    if (!m_referenceNetworkManager)
    {
        m_referenceNetworkManager = new QNetworkAccessManager(this);
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
        // Without a timeout a hung connection leaves "Downloading..." shown forever
        m_referenceNetworkManager->setTransferTimeout(15000);
#endif
        connect(m_referenceNetworkManager, &QNetworkAccessManager::finished, this, [this](QNetworkReply* reply) {
            handleReferenceDownload(reply);
        });
    }

    m_referenceDownloadKey = key;
    updateReferenceLabel(tr("Downloading %1...").arg(key));
    const QString url = CameraOpticalSpectrumLibrary::isEmissionTemplate(key)
        ? CameraOpticalSpectrumLibrary::emissionTemplateUrl(key)
        : CameraOpticalSpectrumLibrary::templateUrl(key);
    QNetworkRequest request{QUrl(url)};
    request.setHeader(QNetworkRequest::UserAgentHeader, kUserAgent);
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    m_referenceNetworkManager->get(request);
}

void CameraOpticalSpectrumDialog::handleReferenceDownload(QNetworkReply* reply)
{
    reply->deleteLater();
    const QString key = m_referenceDownloadKey;
    m_referenceDownloadKey.clear();

    if (reply->error() != QNetworkReply::NoError)
    {
        updateReferenceLabel(tr("Download failed: %1").arg(reply->errorString()));
        return;
    }

    const QByteArray bytes = reply->readAll();
    const QVector<QPointF> points = parseTemplateBytes(key, bytes);
    if (points.isEmpty())
    {
        updateReferenceLabel(tr("Could not read the downloaded reference spectrum"));
        return;
    }

    const QString cachePath = referenceCachePath(key);
    QDir().mkpath(QFileInfo(cachePath).absolutePath());
    QFile cacheFile(cachePath);
    if (cacheFile.open(QIODevice::WriteOnly))
    {
        cacheFile.write(bytes);
        cacheFile.close();
    }

    m_referencePoints = points;
    m_referenceLoadedKey = key;
    updateReferenceLabel();
    updateChart();
}

void CameraOpticalSpectrumDialog::updateSpectrum(const CameraOpticalSpectrumData& spectrumData)
{
    if (spectrumData.isValid())
    {
        // Warn rather than silently mislead: a source filling the RoI leaves no rows to
        // measure a background from, and heavy clipping destroys the colour information.
        m_dataWarnings.clear();
        if (spectrumData.m_backgroundUnavailable) {
            m_dataWarnings.append(tr("no background rows (source fills the RoI) - subtraction skipped"));
        }
        if (spectrumData.m_backgroundInsufficientRows) {
            m_dataWarnings.append(tr("RoI too small for background estimation - enlarge it to include sky above/below the trace (subtraction skipped)"));
        }
        if (spectrumData.m_saturatedFraction > 0.02f) {
            m_dataWarnings.append(tr("%1% of the aperture is saturated - reduce exposure")
                .arg(spectrumData.m_saturatedFraction * 100.0f, 0, 'f', 0));
        }

        // Frame-averaging history: restart whenever profiles would no longer be
        // sample-aligned - a length change (RoI resize), a moved RoI origin, or an axis
        // flip. Averaging misaligned profiles smears two sky positions together.
        if (!m_averageHistory.empty()
            && ((m_averageHistory.back().m_luminance.size() != spectrumData.m_luminance.size())
                || (m_averageHistory.back().m_axisOrigin != spectrumData.m_axisOrigin)
                || (m_averageHistory.back().m_verticalAxis != spectrumData.m_verticalAxis))) {
            m_averageHistory.clear();
        }
        m_averageHistory.push_back(spectrumData);
        while (static_cast<int>(m_averageHistory.size()) > m_averageFramesSpin->maximum()) {
            m_averageHistory.pop_front();
        }
        m_spectrumData = spectrumData;

        // Latch the auto direction: only adopt a new answer when the red/blue centroid
        // separation is decisive, so a faint or grey source cannot flip the wavelength
        // axis frame to frame.
        if (CameraOpticalSpectrumExtractor::autoDirectionDecisive(spectrumData)) {
            m_autoRedPositive = CameraOpticalSpectrumExtractor::autoDirectionRedPositive(spectrumData);
        }

        // Smooth the auto zero-order position so per-frame centroid noise does not
        // jitter the wavelength axis; a large jump (the star moved) snaps immediately.
        if (spectrumData.m_zeroOrderPx >= 0.0f)
        {
            const double detected = spectrumData.m_zeroOrderPx;
            if ((m_zeroOrderSmoothedPx < 0.0) || (std::abs(detected - m_zeroOrderSmoothedPx) > 5.0)) {
                m_zeroOrderSmoothedPx = detected;
            } else {
                m_zeroOrderSmoothedPx = 0.75 * m_zeroOrderSmoothedPx + 0.25 * detected;
            }
        }
        else
        {
            m_zeroOrderSmoothedPx = -1.0;
        }
    }

    // Frame-driven redraws are throttled; the newest data always renders eventually
    // because a skipped update leaves the pending flag for the timer to pick up
    if (m_chartThrottleTimer.isActive())
    {
        m_chartUpdatePending = true;
    }
    else
    {
        updateChart();
        m_chartThrottleTimer.start();
    }
}

void CameraOpticalSpectrumDialog::applySettingChanged(const QString& settingsKey)
{
    if (m_updatingControls) {
        return;
    }
    emit settingsChanged({settingsKey});
    updateChart();
}

bool CameraOpticalSpectrumDialog::isCalibrated() const
{
    return m_settings.m_opticalSpectrumDispersion > 0.0;
}

bool CameraOpticalSpectrumDialog::directionRedPositive() const
{
    switch (m_settings.m_opticalSpectrumDirection)
    {
    case CameraSettings::OpticalSpectrumDirectionNormal:
        return true;
    case CameraSettings::OpticalSpectrumDirectionFlipped:
        return false;
    case CameraSettings::OpticalSpectrumDirectionAuto:
    default:
        return m_autoRedPositive; // latched in updateSpectrum when the centroids are decisive
    }
}

double CameraOpticalSpectrumDialog::zeroOrderImagePosition() const
{
    if (m_settings.m_opticalSpectrumZeroOrderAuto && (m_zeroOrderSmoothedPx >= 0.0)) {
        return m_spectrumData.m_axisOrigin + m_zeroOrderSmoothedPx;
    }
    return m_settings.m_opticalSpectrumZeroOrderX;
}

double CameraOpticalSpectrumDialog::wavelengthAt(int profileIndex) const
{
    const double pixel = m_spectrumData.m_axisOrigin + profileIndex;
    const double sign = directionRedPositive() ? 1.0 : -1.0;
    return m_settings.m_opticalSpectrumDispersion * sign * (pixel - zeroOrderImagePosition());
}

QVector<float> CameraOpticalSpectrumDialog::displayProfile(const QVector<float>& profile) const
{
    // Frame averaging is applied by the caller (profiles passed in are already
    // averaged); this applies the moving-average smoothing.
    const int window = m_settings.m_opticalSpectrumSmoothing;
    if ((window <= 1) || profile.isEmpty()) {
        return profile;
    }
    QVector<float> smoothed(profile.size());
    const int half = window / 2;
    for (int i = 0; i < profile.size(); i++)
    {
        const int lo = qMax(0, i - half);
        const int hi = qMin(profile.size() - 1, i + half);
        double sum = 0.0;
        for (int j = lo; j <= hi; j++) {
            sum += profile[j];
        }
        smoothed[i] = static_cast<float>(sum / (hi - lo + 1));
    }
    return smoothed;
}

void CameraOpticalSpectrumDialog::clearReferenceLines()
{
    // The series themselves are deleted by QChart::removeAllSeries; only reset the
    // pointers and delete the labels, which the chart does not own.
    m_referenceLineSeries.clear();
    m_referenceLineWavelengths.clear();
    qDeleteAll(m_referenceLineLabels);
    m_referenceLineLabels.clear();
}

void CameraOpticalSpectrumDialog::updateChart()
{
    const QScopedValueRollback<bool> inUpdateGuard(m_inUpdateChart, true);
    clearReferenceLines();
    m_chart->removeAllSeries(); // also deletes the marker and feature series
    m_markerSeries = nullptr;
    m_markerSeriesB = nullptr;
    m_featureSeries = nullptr;
    delete m_markerLabel;
    m_markerLabel = nullptr;
    delete m_markerLabelB;
    m_markerLabelB = nullptr;
    qDeleteAll(m_featureLabels);
    m_featureLabels.clear();
    m_featureAnchors.clear();
    ensureAxisY();

    if (!m_spectrumData.isValid())
    {
        m_axisX->setTitleText(tr("Pixel"));
        m_axisX->setRange(0.0, 1.0);
        m_axisY->setRange(0.0, 1.0);
        m_axisYLog->setRange(0.1, 1.0);
        m_displayLuminance.clear();
        m_displayLuminanceRaw.clear();
        m_displayRed.clear();
        m_displayRedRaw.clear();
        m_displayGreen.clear();
        m_displayGreenRaw.clear();
        m_displayBlue.clear();
        m_displayBlueRaw.clear();
        m_displayXValues.clear();
        m_warningLabel->clear();
        updateStrips();
        return;
    }

    // Average the pixel-domain profiles over the last N frames
    const int framesWanted = m_settings.m_opticalSpectrumAverageFrames;
    const int frames = qMin(framesWanted, static_cast<int>(m_averageHistory.size()));
    const int length = m_spectrumData.m_luminance.size();
    CameraOpticalSpectrumData averaged = m_spectrumData;
    if (frames > 1)
    {
        QVector<double> sumL(length, 0.0);
        QVector<double> sumR(length, 0.0);
        QVector<double> sumG(length, 0.0);
        QVector<double> sumB(length, 0.0);
        auto it = m_averageHistory.crbegin();
        for (int f = 0; f < frames; ++f, ++it)
        {
            for (int i = 0; i < length; i++)
            {
                sumL[i] += it->m_luminance[i];
                sumR[i] += it->m_red[i];
                sumG[i] += it->m_green[i];
                sumB[i] += it->m_blue[i];
            }
        }
        for (int i = 0; i < length; i++)
        {
            averaged.m_luminance[i] = static_cast<float>(sumL[i] / frames);
            averaged.m_red[i] = static_cast<float>(sumR[i] / frames);
            averaged.m_green[i] = static_cast<float>(sumG[i] / frames);
            averaged.m_blue[i] = static_cast<float>(sumB[i] / frames);
        }
    }

    const bool calibrated = isCalibrated();
    QVector<double> xValues(length);
    for (int i = 0; i < length; i++) {
        xValues[i] = calibrated ? wavelengthAt(i) : (m_spectrumData.m_axisOrigin + i);
    }
    m_displayXValues = xValues;
    m_displayLuminance = displayProfile(averaged.m_luminance);
    m_displayRed = displayProfile(averaged.m_red);
    m_displayGreen = displayProfile(averaged.m_green);
    m_displayBlue = displayProfile(averaged.m_blue);
    m_displayLuminanceRaw = m_displayLuminance;
    m_displayRedRaw = m_displayRed;
    m_displayGreenRaw = m_displayGreen;
    m_displayBlueRaw = m_displayBlue;

    // Instrument response correction: divide the luminance by the captured response.
    // Where the response is very weak (the band edges) or the curve does not reach,
    // the division would only amplify noise, so those samples are blanked instead -
    // and a warning shown, as a large blanked region means the response was captured
    // over a narrower wavelength range than this spectrum spans.
    QStringList displayWarnings = m_dataWarnings;
    if (m_settings.m_opticalSpectrumApplyResponse && calibrated && !m_responsePoints.isEmpty())
    {
        int blanked = 0;
        for (int i = 0; i < m_displayLuminance.size(); i++)
        {
            const double response = CameraOpticalSpectrumLibrary::responseAt(m_responsePoints, xValues[i]);
            if (response > 0.05)
            {
                m_displayLuminance[i] = static_cast<float>(m_displayLuminance[i] / response);
            }
            else
            {
                m_displayLuminance[i] = 0.0f;
                blanked++;
            }
        }
        if (blanked > length / 10)
        {
            displayWarnings.append(tr("response covers %1-%2 nm; %3% of this spectrum is outside it and blanked")
                .arg(m_responsePoints.first().x(), 0, 'f', 0)
                .arg(m_responsePoints.last().x(), 0, 'f', 0)
                .arg(100.0 * blanked / length, 0, 'f', 0));
        }

        // Per-channel correction, when the response file carries channel curves. Each
        // channel divides by its own response; the curves share one normalisation so
        // the relative channel sensitivities (and hence the hue in the image strip)
        // are corrected rather than re-scaled away.
        const auto correctChannel = [this, &xValues](QVector<float>& profile, const QVector<QPointF>& response) {
            if (response.isEmpty()) {
                return;
            }
            double channelPeak = 0.0;
            for (const QPointF& point : response) {
                channelPeak = qMax(channelPeak, point.y());
            }
            if (channelPeak <= 0.0) {
                return;
            }
            const double floor = 0.05 * channelPeak;
            for (int i = 0; i < profile.size(); i++)
            {
                const double r = CameraOpticalSpectrumLibrary::responseAt(response, xValues[i]);
                profile[i] = (r > floor) ? static_cast<float>(profile[i] / r) : 0.0f;
            }
        };
        correctChannel(m_displayRed, m_responseRed);
        correctChannel(m_displayGreen, m_responseGreen);
        correctChannel(m_displayBlue, m_responseBlue);
    }
    m_warningLabel->setText(displayWarnings.isEmpty() ? QString() : QStringLiteral("⚠ ") + displayWarnings.join(QStringLiteral("; ")));

    // The display profiles are already averaged, smoothed and (for luminance)
    // response-corrected above
    const struct { const QVector<float>* profile; const char* name; QColor color; bool visible; } channelDefs[] = {
        {&m_displayLuminance, QT_TR_NOOP("Luminance"), Qt::white, m_luminanceCheck->isChecked()},
        {&m_displayRed, QT_TR_NOOP("Red"), Qt::red, m_redCheck->isChecked()},
        {&m_displayGreen, QT_TR_NOOP("Green"), Qt::green, m_greenCheck->isChecked()},
        {&m_displayBlue, QT_TR_NOOP("Blue"), QColor(80, 130, 255), m_blueCheck->isChecked()},
    };

    // Normalisation factor from the maximum of the visible smoothed profiles
    double normFactor = 1.0;
    if (m_settings.m_opticalSpectrumNormalize)
    {
        double maxValue = 0.0;
        for (const auto& def : channelDefs)
        {
            if (!def.visible) {
                continue;
            }
            for (const float v : *def.profile) {
                maxValue = qMax(maxValue, static_cast<double>(v));
            }
        }
        if (maxValue > 0.0) {
            normFactor = 1.0 / maxValue;
        }
    }
    m_displayNormFactor = normFactor;

    // On a log axis values at or below zero cannot be shown, and picking the floor as
    // a fixed fraction of the peak wastes decades of empty chart when the data spans
    // less than that (a bright continuum sits in a thin band at the top). Instead the
    // floor follows the data: a low percentile of the positive values, so a continuum
    // spectrum gets a snug range while an emission spectrum with a genuinely dark
    // background keeps its decades. Zero/blanked samples clamp to the floor.
    const bool logY = m_settings.m_opticalSpectrumLogY;
    double plotFloor = 0.0;
    if (logY)
    {
        double dataMax = 0.0;
        QVector<double> positives;
        for (const auto& def : channelDefs)
        {
            if (!def.visible) {
                continue;
            }
            for (const float v : *def.profile)
            {
                const double value = static_cast<double>(v) * normFactor;
                dataMax = qMax(dataMax, value);
                if (value > 0.0) {
                    positives.append(value);
                }
            }
        }
        if (positives.isEmpty() || (dataMax <= 0.0))
        {
            plotFloor = 1e-4;
        }
        else
        {
            const int k = static_cast<int>(positives.size() / 20); // 5th percentile
            std::nth_element(positives.begin(), positives.begin() + k, positives.end());
            plotFloor = qBound(dataMax * 1e-6, positives[k] * 0.5, dataMax / 10.0);
        }
    }

    double minY = 0.0;
    double maxY = 0.0;
    for (const auto& def : channelDefs)
    {
        if (!def.visible) {
            continue;
        }
        const QVector<float>& smoothed = *def.profile;
        auto* series = new QLineSeries();
        series->setName(tr(def.name));
        QPen pen(def.color);
        pen.setWidth(1);
        series->setPen(pen);
        QList<QPointF> points;
        points.reserve(length);
        for (int i = 0; i < length; i++)
        {
            double v = smoothed[i] * normFactor;
            if (logY) {
                v = qMax(v, plotFloor);
            }
            points.append(QPointF(xValues[i], v));
            minY = qMin(minY, v);
            maxY = qMax(maxY, v);
        }
        series->replace(points);
        m_chart->addSeries(series);
        series->attachAxis(m_axisX);
        series->attachAxis(activeAxisY());
    }

    // Show what background subtraction removed, so an inflated estimate (trace glow or
    // a bright blob sampled as sky) is immediately visible rather than a mystery slope
    if (!m_spectrumData.m_background.isEmpty()
        && (m_spectrumData.m_background.size() == length)
        && m_luminanceCheck->isChecked())
    {
        auto* series = new QLineSeries();
        series->setName(tr("Background"));
        QPen pen(QColor(150, 150, 150));
        pen.setWidth(1);
        pen.setStyle(Qt::DotLine);
        series->setPen(pen);
        QList<QPointF> points;
        points.reserve(length);
        for (int i = 0; i < length; i++)
        {
            double v = m_spectrumData.m_background[i] * normFactor;
            if (logY) {
                v = qMax(v, plotFloor);
            }
            points.append(QPointF(xValues[i], v));
            maxY = qMax(maxY, v);
        }
        series->replace(points);
        m_chart->addSeries(series);
        series->attachAxis(m_axisX);
        series->attachAxis(activeAxisY());
    }

    const double minX = qMin(xValues.first(), xValues.last());
    const double maxX = qMax(xValues.first(), xValues.last());
    const double redshiftScale = 1.0 + m_settings.m_opticalSpectrumRedshift;

    // Reference template: only meaningful against a wavelength axis. Rest-frame
    // spectra are shifted by the redshift setting (stars are usually observed at
    // z = 0, so this matters for the emission-object templates).
    if (calibrated && !m_referencePoints.isEmpty() && m_luminanceCheck->isChecked())
    {
        addFittedSeries(m_referencePoints,
            tr("%1 (template)").arg(CameraOpticalSpectrumLibrary::templateDisplayName(m_referenceLoadedKey)),
            QColor(255, 215, 90), Qt::DotLine, minX, maxX, normFactor, redshiftScale, plotFloor, maxY);
    }

    // Comparison overlay from a previously exported spectrum
    if (calibrated && !m_overlayPoints.isEmpty() && m_luminanceCheck->isChecked())
    {
        addFittedSeries(m_overlayPoints, tr("%1 (overlay)").arg(m_overlayName),
            QColor(140, 235, 140), Qt::DashDotLine, minX, maxX, normFactor, 1.0, plotFloor, maxY);
    }

    if (maxY <= 0.0) {
        maxY = 1.0;
    }
    maxY *= 1.05;
    m_axisX->setTitleText(calibrated ? tr("Wavelength (nm)") : tr("Pixel"));
    const QString axisYTitle = m_settings.m_opticalSpectrumNormalize ? tr("Relative intensity") : tr("Intensity");
    m_axisYLog->setTitleText(axisYTitle);
    m_axisY->setTitleText(axisYTitle);

    // Full data ranges, overridden by a manual (rubber-band) zoom. The zoom persists
    // across frame updates, which would otherwise reset the axes every frame; it is
    // cleared by Reset, or stepped by Zoom out. Y for the log axis is clamped positive.
    m_fullXMin = minX;
    m_fullXMax = (maxX > minX) ? maxX : minX + 1.0;
    m_fullYMin = logY ? plotFloor : minY;
    m_fullYMax = maxY;
    double xMin = m_fullXMin;
    double xMax = m_fullXMax;
    double yMin = m_fullYMin;
    double yMax = m_fullYMax;
    if (m_zoomed)
    {
        // Clamp the stored zoom to the current data extent (the RoI or calibration may
        // have moved since the zoom); drop the zoom if it no longer overlaps the data.
        xMin = qBound(m_fullXMin, m_zoomXMin, m_fullXMax);
        xMax = qBound(m_fullXMin, m_zoomXMax, m_fullXMax);
        yMin = qBound(m_fullYMin, m_zoomYMin, m_fullYMax);
        yMax = qBound(m_fullYMin, m_zoomYMax, m_fullYMax);
        if ((xMax - xMin < 1e-9) || (yMax - yMin <= 0.0))
        {
            m_zoomed = false;
            xMin = m_fullXMin;
            xMax = m_fullXMax;
            yMin = m_fullYMin;
            yMax = m_fullYMax;
        }
    }
    m_axisX->setRange(xMin, xMax);
    if (logY) {
        m_axisYLog->setRange(qMax(yMin, 1e-12), yMax);
    } else {
        m_axisY->setRange(yMin, yMax);
    }
    if (m_zoomResetButton) {
        m_zoomResetButton->setEnabled(m_zoomed);
        m_zoomOutButton->setEnabled(m_zoomed);
    }

    const double lineFloor = yMin;
    if (calibrated) {
        updateReferenceLines(lineFloor, maxY);
    }
    if (m_settings.m_opticalSpectrumAutoIdentify && calibrated) {
        updateFeatureAnnotations(xValues, plotFloor);
    }
    updateMarker();
    updateStrips();
}

QAbstractAxis* CameraOpticalSpectrumDialog::activeAxisY() const
{
    return m_settings.m_opticalSpectrumLogY
        ? static_cast<QAbstractAxis*>(m_axisYLog)
        : static_cast<QAbstractAxis*>(m_axisY);
}

double CameraOpticalSpectrumDialog::axisYMax() const
{
    return m_settings.m_opticalSpectrumLogY ? m_axisYLog->max() : m_axisY->max();
}

void CameraOpticalSpectrumDialog::ensureAxisY()
{
    QAbstractAxis* wanted = activeAxisY();
    QAbstractAxis* other = (wanted == m_axisY)
        ? static_cast<QAbstractAxis*>(m_axisYLog)
        : static_cast<QAbstractAxis*>(m_axisY);
    const QList<QAbstractAxis*> attached = m_chart->axes(Qt::Vertical);
    if (attached.contains(other)) {
        m_chart->removeAxis(other);
    }
    if (!attached.contains(wanted)) {
        m_chart->addAxis(wanted, Qt::AlignLeft);
    }
}

void CameraOpticalSpectrumDialog::addFittedSeries(const QVector<QPointF>& points, const QString& name, const QColor& colour,
    Qt::PenStyle penStyle, double minX, double maxX, double normFactor, double xScale, double plotFloor, double& maxY)
{
    // The reference/overlay values are in their own units, so fit a single scale
    // factor to the displayed luminance by least squares over the overlap
    const int length = m_displayLuminance.size();
    if (length < 2) {
        return;
    }
    const double firstX = m_displayXValues.first();
    const double lastX = m_displayXValues.last();
    const auto observedAt = [this, firstX, lastX, length, normFactor](double nm, double& observed) {
        const double index = (nm - firstX) / (lastX - firstX) * (length - 1);
        if ((index < 0.0) || (index > length - 1)) {
            return false;
        }
        observed = interpolate(m_displayLuminance, index) * normFactor;
        return true;
    };

    double sumRefObserved = 0.0;
    double sumRefSquared = 0.0;
    for (const QPointF& point : points)
    {
        const double nm = point.x() * xScale;
        double observed = 0.0;
        if ((nm < minX) || (nm > maxX) || !observedAt(nm, observed)) {
            continue;
        }
        sumRefObserved += observed * point.y();
        sumRefSquared += point.y() * point.y();
    }
    if (sumRefSquared <= 0.0) {
        return;
    }

    const double scale = sumRefObserved / sumRefSquared;
    auto* series = new QLineSeries();
    series->setName(name);
    QPen pen(colour);
    pen.setWidth(1);
    pen.setStyle(penStyle);
    series->setPen(pen);
    QList<QPointF> seriesPoints;
    const bool logY = m_settings.m_opticalSpectrumLogY;
    for (const QPointF& point : points)
    {
        const double nm = point.x() * xScale;
        if ((nm < minX) || (nm > maxX)) {
            continue;
        }
        double v = point.y() * scale;
        if (logY) {
            v = qMax(v, plotFloor);
        }
        seriesPoints.append(QPointF(nm, v));
        maxY = qMax(maxY, v);
    }
    series->replace(seriesPoints);
    m_chart->addSeries(series);
    series->attachAxis(m_axisX);
    series->attachAxis(activeAxisY());
}

void CameraOpticalSpectrumDialog::updateFeatureAnnotations(const QVector<double>& xValues, double plotFloor)
{
    const QVector<CameraOpticalSpectrumFeature> features =
        CameraOpticalSpectrumExtractor::detectFeatures(m_displayLuminance);
    if (features.isEmpty()) {
        return;
    }

    // Candidate lines: the full library plus custom lines, with the redshift applied
    // to source lines and terrestrial lines at rest - matching the overlay rendering
    struct Candidate { QString m_label; double m_nm; };
    QVector<Candidate> candidates;
    const double redshift = m_settings.m_opticalSpectrumRedshift;
    const auto appendLines = [&candidates, redshift](const QVector<CameraOpticalSpectrumRefLine>& lines, bool terrestrial) {
        for (const CameraOpticalSpectrumRefLine& line : lines) {
            candidates.append({line.m_label, terrestrial ? line.m_nm : line.m_nm * (1.0 + redshift)});
        }
    };
    for (const CameraOpticalSpectrumRefLineSet& set : CameraOpticalSpectrumExtractor::referenceLineSets()) {
        appendLines(set.m_lines, set.m_terrestrial);
    }
    appendLines(CameraOpticalSpectrumExtractor::parseCustomLines(m_settings.m_opticalSpectrumCustomLines), false);

    constexpr double kMatchToleranceNm = 2.0;
    const QColor featureColour(140, 220, 255);
    m_featureSeries = new QScatterSeries();
    m_featureSeries->setMarkerSize(7.0);
    m_featureSeries->setColor(featureColour);
    m_featureSeries->setBorderColor(QColor(30, 30, 30));

    const bool logY = m_settings.m_opticalSpectrumLogY;
    for (const CameraOpticalSpectrumFeature& feature : features)
    {
        const double nm = xValues[feature.m_index];
        QString text;
        double bestDistance = kMatchToleranceNm;
        for (const Candidate& candidate : candidates)
        {
            const double distance = std::abs(candidate.m_nm - nm);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                text = candidate.m_label;
            }
        }
        if (text.isEmpty()) {
            text = QStringLiteral("%1?").arg(nm, 0, 'f', 1);
        }

        double value = m_displayLuminance[feature.m_index] * m_displayNormFactor;
        if (logY) {
            value = qMax(value, plotFloor);
        }
        m_featureSeries->append(nm, value);
        m_featureAnchors.append(QPointF(nm, value));
        auto* label = new QGraphicsSimpleTextItem(text, m_chart);
        label->setBrush(QBrush(featureColour));
        QFont font = label->font();
        font.setPointSizeF(font.pointSizeF() * 0.85);
        label->setFont(font);
        m_featureLabels.append(label);
    }

    m_chart->addSeries(m_featureSeries);
    m_featureSeries->attachAxis(m_axisX);
    m_featureSeries->attachAxis(activeAxisY());
    const auto markers = m_chart->legend()->markers(m_featureSeries);
    for (QLegendMarker* marker : markers) {
        marker->setVisible(false);
    }
    positionFeatureLabels();
}

void CameraOpticalSpectrumDialog::positionFeatureLabels()
{
    if (!m_featureSeries) {
        return;
    }
    for (int i = 0; i < m_featureLabels.size(); i++)
    {
        const QPointF pos = m_chart->mapToPosition(m_featureAnchors[i], m_featureSeries);
        QGraphicsSimpleTextItem* label = m_featureLabels[i];
        const QRectF textRect = label->boundingRect();
        double labelX = pos.x() - textRect.width() / 2.0;
        const QRectF plotArea = m_chart->plotArea();
        labelX = qBound(plotArea.left(), labelX, plotArea.right() - textRect.width());
        double labelY = pos.y() - 10.0 - textRect.height();
        if (labelY < plotArea.top()) {
            labelY = pos.y() + 10.0;
        }
        label->setPos(labelX, labelY);
        label->setVisible(plotArea.contains(pos));
    }
}

void CameraOpticalSpectrumDialog::loadOverlay()
{
    const QString fileName = QFileDialog::getOpenFileName(this, tr("Load overlay spectrum"), QString(), tr("CSV files (*.csv)"));
    if (fileName.isEmpty()) {
        return;
    }
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::warning(this, tr("Load overlay"), tr("Failed to open %1").arg(fileName));
        return;
    }
    const QVector<QPointF> points = CameraOpticalSpectrumLibrary::parseExportedSpectrumCsv(file.readAll());
    if (points.isEmpty())
    {
        QMessageBox::warning(this, tr("Load overlay"),
            tr("No usable spectrum found in %1.\nThe file must be an exported spectrum CSV with the wavelength scale calibrated.").arg(QFileInfo(fileName).fileName()));
        return;
    }
    m_overlayPoints = points;
    m_overlayName = QFileInfo(fileName).completeBaseName();
    m_overlayLabel->setText(m_overlayName);
    m_overlayClearButton->setVisible(true);
    updateChart();
}

void CameraOpticalSpectrumDialog::clearOverlay()
{
    m_overlayPoints.clear();
    m_overlayName.clear();
    m_overlayLabel->clear();
    m_overlayClearButton->setVisible(false);
    updateChart();
}

void CameraOpticalSpectrumDialog::updateMarker()
{
    // A must render before B so B's delta readout can reference A's position. The
    // axis ranges are already set, so the log floor follows the axis minimum.
    const double logFloor = m_settings.m_opticalSpectrumLogY ? qMax(1e-12, m_axisYLog->min()) : 0.0;
    renderOneMarker(m_markerPixel, false, logFloor);
    renderOneMarker(m_markerPixelB, true, logFloor);
}

void CameraOpticalSpectrumDialog::renderOneMarker(double& markerPixel, bool secondMarker, double plotFloor)
{
    if ((markerPixel < 0.0) || (m_displayXValues.size() < 2) || m_displayLuminance.isEmpty()) {
        return;
    }
    const double index = markerPixel - m_spectrumData.m_axisOrigin;
    if ((index < 0.0) || (index > m_displayLuminance.size() - 1))
    {
        // The RoI moved away from the marked position
        markerPixel = -1.0;
        return;
    }

    const int i0 = static_cast<int>(index);
    const int i1 = qMin(i0 + 1, m_displayXValues.size() - 1);
    const double frac = index - i0;
    const double x = (1.0 - frac) * m_displayXValues[i0] + frac * m_displayXValues[i1];
    const double value = interpolate(m_displayLuminance, index) * m_displayNormFactor;
    const double plottedValue = m_settings.m_opticalSpectrumLogY ? qMax(value, plotFloor) : value;
    const QColor colour = secondMarker ? QColor(255, 170, 90) : QColor(255, 255, 120);

    auto* series = new QScatterSeries();
    series->setMarkerSize(9.0);
    series->setColor(colour);
    series->setBorderColor(QColor(40, 40, 40));
    series->append(x, plottedValue);
    m_chart->addSeries(series);
    series->attachAxis(m_axisX);
    series->attachAxis(activeAxisY());
    const auto markers = m_chart->legend()->markers(series);
    for (QLegendMarker* marker : markers) {
        marker->setVisible(false);
    }

    const bool calibrated = isCalibrated();
    const QString position = calibrated
        ? tr("%1 nm (px %2)").arg(x, 0, 'f', 1).arg(markerPixel, 0, 'f', 0)
        : tr("px %1").arg(markerPixel, 0, 'f', 0);
    QString text = QStringLiteral("%1\n%2").arg(position, QString::number(value, 'g', 5));

    if (secondMarker)
    {
        // Differences from marker A
        if (m_markerPixel >= 0.0)
        {
            const double indexA = m_markerPixel - m_spectrumData.m_axisOrigin;
            if ((indexA >= 0.0) && (indexA <= m_displayLuminance.size() - 1))
            {
                const int a0 = static_cast<int>(indexA);
                const int a1 = qMin(a0 + 1, m_displayXValues.size() - 1);
                const double fracA = indexA - a0;
                const double xA = (1.0 - fracA) * m_displayXValues[a0] + fracA * m_displayXValues[a1];
                const double valueA = interpolate(m_displayLuminance, indexA) * m_displayNormFactor;
                text += calibrated
                    ? tr("\nΔ %1 nm (%2 px)  ΔI %3")
                        .arg(std::abs(x - xA), 0, 'f', 2)
                        .arg(std::abs(markerPixel - m_markerPixel), 0, 'f', 0)
                        .arg(QString::number(value - valueA, 'g', 4))
                    : tr("\nΔ %1 px  ΔI %2")
                        .arg(std::abs(markerPixel - m_markerPixel), 0, 'f', 0)
                        .arg(QString::number(value - valueA, 'g', 4));
            }
        }
    }
    else
    {
        // Line measurement at the marker: FWHM and equivalent width, when a feature
        // is present. Samples convert to nm through the (linear) dispersion.
        const CameraOpticalSpectrumLineMeasurement measurement =
            CameraOpticalSpectrumExtractor::measureLine(m_displayLuminance, static_cast<int>(std::lround(index)));
        if (measurement.m_valid)
        {
            const QString kind = measurement.m_emission ? tr("emission") : tr("absorption");
            if (calibrated)
            {
                const double nmPerSample = m_settings.m_opticalSpectrumDispersion;
                text += tr("\n%1: FWHM %2 nm, EW %3 nm")
                    .arg(kind)
                    .arg(measurement.m_fwhmSamples * nmPerSample, 0, 'f', 2)
                    .arg(measurement.m_equivalentWidthSamples * nmPerSample, 0, 'f', 2);
            }
            else
            {
                text += tr("\n%1: FWHM %2 px")
                    .arg(kind)
                    .arg(measurement.m_fwhmSamples, 0, 'f', 1);
            }
        }
    }

    auto* label = new QGraphicsSimpleTextItem(text, m_chart);
    label->setBrush(QBrush(colour));
    QFont font = label->font();
    font.setPointSizeF(font.pointSizeF() * 0.9);
    label->setFont(font);

    if (secondMarker)
    {
        m_markerSeriesB = series;
        m_markerLabelB = label;
    }
    else
    {
        m_markerSeries = series;
        m_markerLabel = label;
    }
    positionOneMarkerLabel(series, label);
}

void CameraOpticalSpectrumDialog::positionMarkerLabel()
{
    positionOneMarkerLabel(m_markerSeries, m_markerLabel);
    positionOneMarkerLabel(m_markerSeriesB, m_markerLabelB);
    positionFeatureLabels();
}

void CameraOpticalSpectrumDialog::positionOneMarkerLabel(QScatterSeries* series, QGraphicsSimpleTextItem* label)
{
    if (!label || !series || series->points().isEmpty()) {
        return;
    }
    const QPointF point = series->points().first();
    const QPointF pos = m_chart->mapToPosition(point, series);
    const QRectF plotArea = m_chart->plotArea();
    // Keep the label inside the plot area: prefer above-right of the point, flip
    // left/below near the edges
    const QRectF textRect = label->boundingRect();
    double labelX = pos.x() + 8.0;
    if (labelX + textRect.width() > plotArea.right()) {
        labelX = pos.x() - 8.0 - textRect.width();
    }
    double labelY = pos.y() - 8.0 - textRect.height();
    if (labelY < plotArea.top()) {
        labelY = pos.y() + 8.0;
    }
    label->setPos(labelX, labelY);
    label->setVisible(plotArea.contains(pos));
}

float CameraOpticalSpectrumDialog::interpolate(const QVector<float>& profile, double index)
{
    const int i0 = static_cast<int>(index);
    const int i1 = qMin(i0 + 1, profile.size() - 1);
    const double frac = index - i0;
    return static_cast<float>((1.0 - frac) * profile[i0] + frac * profile[i1]);
}

void CameraOpticalSpectrumDialog::updateStrips()
{
    updateImageStrip();
    updateColourStrip();
}

void CameraOpticalSpectrumDialog::renderStrip(QLabel* strip, const std::function<QRgb(double axisValue, double index)>& colourAt)
{
    const QRectF plotArea = m_chart->plotArea();
    const int labelWidth = qMax(1, strip->width());
    const int stripWidth = static_cast<int>(std::lround(plotArea.width()));
    const int stripLeft = static_cast<int>(std::lround(plotArea.left()));
    const int profileLength = m_displayLuminance.size();
    if ((profileLength < 2) || (stripWidth < 2))
    {
        strip->clear();
        return;
    }

    const double firstX = m_displayXValues.first();
    const double lastX = m_displayXValues.last();
    const double axisMin = m_axisX->min();
    const double axisMax = m_axisX->max();

    // One-row image the full label width, coloured only under the plot area so the
    // strip lines up with the chart's X axis; scaled up to the strip height.
    QImage image(labelWidth, 1, QImage::Format_ARGB32);
    image.fill(Qt::transparent);
    QRgb* line = reinterpret_cast<QRgb*>(image.scanLine(0));
    for (int px = 0; px < stripWidth; px++)
    {
        const int x = stripLeft + px;
        if ((x < 0) || (x >= labelWidth)) {
            continue;
        }
        const double axisValue = axisMin + ((px + 0.5) / stripWidth) * (axisMax - axisMin);
        const double index = (axisValue - firstX) / (lastX - firstX) * (profileLength - 1);
        if ((index < 0.0) || (index > profileLength - 1)) {
            continue;
        }
        line[x] = colourAt(axisValue, index);
    }
    strip->setPixmap(QPixmap::fromImage(image.scaled(labelWidth, kColourStripHeight)));
}

void CameraOpticalSpectrumDialog::updateColourStrip()
{
    if (!m_settings.m_opticalSpectrumColourStrip) {
        return;
    }

    float maxLuminance = 0.0f;
    for (const float v : m_displayLuminance) {
        maxLuminance = qMax(maxLuminance, v);
    }
    if (maxLuminance <= 0.0f)
    {
        m_colourStrip->clear();
        return;
    }

    const bool calibrated = isCalibrated();
    renderStrip(m_colourStrip, [this, maxLuminance, calibrated](double axisValue, double index) {
        const double intensity = qBound(0.0, interpolate(m_displayLuminance, index) / maxLuminance, 1.0);
        if (calibrated && (axisValue >= kVisibleMinNm) && (axisValue <= kVisibleMaxNm)) {
            return CameraOpticalSpectrumExtractor::wavelengthToColour(axisValue, intensity);
        }
        // Uncalibrated or outside the visible range: intensity only
        const int grey = static_cast<int>(std::lround(255.0 * std::pow(intensity, kStripGamma)));
        return qRgb(grey, grey, grey);
    });
}

void CameraOpticalSpectrumDialog::updateImageStrip()
{
    if (!m_settings.m_opticalSpectrumImageStrip) {
        return;
    }

    // Always render from the UNCORRECTED channels: the strip's purpose is the observed
    // colours. Response-corrected channels converge onto the same curve by construction
    // (dividing each channel by its own response removes the Bayer filter curves - the
    // very thing that encodes wavelength as hue), which renders as grey.
    const int length = m_displayLuminance.size();
    if ((m_displayRedRaw.size() != length) || (m_displayGreenRaw.size() != length) || (m_displayBlueRaw.size() != length))
    {
        m_imageStrip->clear();
        return;
    }

    // A single scale factor across all three channels preserves the observed hue;
    // scaling each channel independently would white-balance the spectrum away.
    float maxChannel = 0.0f;
    for (int i = 0; i < length; i++) {
        maxChannel = qMax(maxChannel, qMax(m_displayRedRaw[i], qMax(m_displayGreenRaw[i], m_displayBlueRaw[i])));
    }
    if (maxChannel <= 0.0f)
    {
        m_imageStrip->clear();
        return;
    }

    renderStrip(m_imageStrip, [this, maxChannel](double, double index) {
        const double red = qMax(0.0f, interpolate(m_displayRedRaw, index));
        const double green = qMax(0.0f, interpolate(m_displayGreenRaw, index));
        const double blue = qMax(0.0f, interpolate(m_displayBlueRaw, index));
        const double peak = qMax(red, qMax(green, blue));
        if (peak <= 0.0) {
            return qRgb(0, 0, 0);
        }
        // Gamma is applied to the brightness only. Applying it per channel raises the
        // weaker channels relative to the strongest (a power law preserves ratios of
        // equal exponents, so g/r becomes (g/r)^gamma), which desaturates every colour
        // towards white regardless of how bright it is.
        const double brightness = std::pow(qBound(0.0, peak / maxChannel, 1.0), kStripGamma);
        const auto channel = [peak, brightness](double value) {
            return static_cast<int>(std::lround(255.0 * (value / peak) * brightness));
        };
        return qRgb(channel(red), channel(green), channel(blue));
    });
}

void CameraOpticalSpectrumDialog::resizeEvent(QResizeEvent* event)
{
    QDialog::resizeEvent(event);
    updateStrips();
}

bool CameraOpticalSpectrumDialog::eventFilter(QObject* watched, QEvent* event)
{
    if (watched != m_chartView->viewport()) {
        return QDialog::eventFilter(watched, event);
    }

    const auto positionOf = [](QMouseEvent* e) {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        return e->position();
#else
        return e->localPos();
#endif
    };

    // A left-button drag rubber-bands a zoom rectangle; a left-button click (negligible
    // drag) places a readout marker. Calibration mode keeps its click-to-pick behaviour.
    constexpr int kDragThreshold = 6;

    if (event->type() == QEvent::MouseButtonPress)
    {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);
        if ((mouseEvent->button() == Qt::LeftButton) && !m_calibrateButton->isChecked())
        {
            m_dragStartVp = positionOf(mouseEvent).toPoint();
            m_dragging = true;
            if (!m_rubberBand) {
                m_rubberBand = new QRubberBand(QRubberBand::Rectangle, m_chartView->viewport());
            }
            m_rubberBand->setGeometry(QRect(m_dragStartVp, QSize()));
            m_rubberBand->show();
            return true;
        }
    }
    else if (event->type() == QEvent::MouseMove)
    {
        if (m_dragging && m_rubberBand)
        {
            const QPoint pos = positionOf(static_cast<QMouseEvent*>(event)).toPoint();
            m_rubberBand->setGeometry(QRect(m_dragStartVp, pos).normalized());
            return true;
        }
    }
    else if (event->type() == QEvent::MouseButtonRelease)
    {
        auto* mouseEvent = static_cast<QMouseEvent*>(event);
        const QPointF clickPos = positionOf(mouseEvent);

        if ((mouseEvent->button() == Qt::LeftButton) && m_dragging)
        {
            m_dragging = false;
            const QRect bandRect = m_rubberBand ? m_rubberBand->geometry() : QRect();
            if (m_rubberBand) {
                m_rubberBand->hide();
            }

            // A real drag zooms; a negligible one is a click that places a marker
            if ((bandRect.width() >= kDragThreshold) || (bandRect.height() >= kDragThreshold))
            {
                applyZoomRect(bandRect);
                return true;
            }
            double pixel = pixelFromViewportX(clickPos.x());
            if (!std::isnan(pixel))
            {
                // Shift snaps to the nearest spectral feature; Ctrl places marker B
                if (mouseEvent->modifiers() & Qt::ShiftModifier)
                {
                    const int index = static_cast<int>(std::lround(pixel - m_spectrumData.m_axisOrigin));
                    const int snapped = CameraOpticalSpectrumExtractor::snapToFeature(m_displayLuminance, index);
                    pixel = m_spectrumData.m_axisOrigin + snapped;
                }
                if (mouseEvent->modifiers() & Qt::ControlModifier) {
                    m_markerPixelB = pixel;
                } else {
                    m_markerPixel = pixel;
                }
                updateChart();
            }
            return true;
        }
        if ((mouseEvent->button() == Qt::LeftButton) && m_calibrateButton->isChecked())
        {
            handleCalibrationClick(clickPos);
            return true;
        }
        if ((mouseEvent->button() == Qt::RightButton) && !m_calibrateButton->isChecked()
            && ((m_markerPixel >= 0.0) || (m_markerPixelB >= 0.0)))
        {
            m_markerPixel = -1.0;
            m_markerPixelB = -1.0;
            updateChart();
            return true;
        }
    }
    return QDialog::eventFilter(watched, event);
}

QPointF CameraOpticalSpectrumDialog::chartValueAt(const QPointF& viewportPos) const
{
    const QRectF plotArea = m_chart->plotArea();
    const double fx = (plotArea.width() > 0.0) ? (viewportPos.x() - plotArea.left()) / plotArea.width() : 0.0;
    const double x = m_axisX->min() + fx * (m_axisX->max() - m_axisX->min());
    const double fy = (plotArea.height() > 0.0) ? (plotArea.bottom() - viewportPos.y()) / plotArea.height() : 0.0;
    double y;
    if (m_settings.m_opticalSpectrumLogY)
    {
        const double lo = std::log10(qMax(m_axisYLog->min(), 1e-12));
        const double hi = std::log10(qMax(m_axisYLog->max(), 1e-12));
        y = std::pow(10.0, lo + fy * (hi - lo));
    }
    else
    {
        y = m_axisY->min() + fy * (m_axisY->max() - m_axisY->min());
    }
    return QPointF(x, y);
}

void CameraOpticalSpectrumDialog::applyZoomRect(const QRect& viewportRect)
{
    const QRectF plotArea = m_chart->plotArea();
    const QRectF clipped = QRectF(viewportRect).intersected(plotArea);
    if ((clipped.width() < 2.0) || (clipped.height() < 2.0)) {
        return;
    }
    const QPointF v0 = chartValueAt(clipped.topLeft());
    const QPointF v1 = chartValueAt(clipped.bottomRight());
    m_zoomXMin = qMin(v0.x(), v1.x());
    m_zoomXMax = qMax(v0.x(), v1.x());
    m_zoomYMin = qMin(v0.y(), v1.y());
    m_zoomYMax = qMax(v0.y(), v1.y());
    m_zoomed = true;
    updateChart();
}

void CameraOpticalSpectrumDialog::zoomOut()
{
    if (!m_zoomed) {
        return;
    }
    const bool logY = m_settings.m_opticalSpectrumLogY;
    // Expand the current view by 1.6x about its centre, clamped to the data extent, in
    // log space for a log axis so the step looks even
    const auto expand = [](double lo, double hi, double fullLo, double fullHi, bool log) {
        if (log) {
            lo = std::log10(qMax(lo, 1e-12));
            hi = std::log10(qMax(hi, 1e-12));
            fullLo = std::log10(qMax(fullLo, 1e-12));
            fullHi = std::log10(qMax(fullHi, 1e-12));
        }
        const double centre = (lo + hi) / 2.0;
        const double half = (hi - lo) / 2.0 * 1.6;
        lo = qMax(centre - half, fullLo);
        hi = qMin(centre + half, fullHi);
        if (log) {
            lo = std::pow(10.0, lo);
            hi = std::pow(10.0, hi);
        }
        return QPointF(lo, hi);
    };
    const QPointF xr = expand(m_zoomXMin, m_zoomXMax, m_fullXMin, m_fullXMax, false);
    const QPointF yr = expand(m_zoomYMin, m_zoomYMax, m_fullYMin, m_fullYMax, logY);
    m_zoomXMin = xr.x();
    m_zoomXMax = xr.y();
    m_zoomYMin = yr.x();
    m_zoomYMax = yr.y();
    // Once the view covers the whole data extent, drop the zoom entirely
    const double xEps = (m_fullXMax - m_fullXMin) * 1e-3;
    const double yEps = (m_fullYMax - m_fullYMin) * 1e-3;
    if ((m_zoomXMin <= m_fullXMin + xEps) && (m_zoomXMax >= m_fullXMax - xEps)
        && (m_zoomYMin <= m_fullYMin + yEps) && (m_zoomYMax >= m_fullYMax - yEps)) {
        m_zoomed = false;
    }
    updateChart();
}

void CameraOpticalSpectrumDialog::resetZoom()
{
    if (!m_zoomed) {
        return;
    }
    m_zoomed = false;
    updateChart();
}

void CameraOpticalSpectrumDialog::setCalibrationMode(bool active)
{
    m_calibrationClicks.clear();
    m_chartView->viewport()->setCursor(active ? Qt::CrossCursor : Qt::ArrowCursor);
    if (active) {
        m_calibrateButton->setText(tr("Click 1st point"));
    } else {
        m_calibrateButton->setText(tr("Calibrate..."));
        if (m_calibrateButton->isChecked())
        {
            const QSignalBlocker blocker(m_calibrateButton);
            m_calibrateButton->setChecked(false);
        }
    }
}

double CameraOpticalSpectrumDialog::pixelFromViewportX(double viewportX) const
{
    if (m_displayXValues.size() < 2) {
        return std::nan("");
    }
    const QRectF plotArea = m_chart->plotArea();
    if ((plotArea.width() < 2.0) || (viewportX < plotArea.left()) || (viewportX > plotArea.right())) {
        return std::nan("");
    }
    const double axisValue = m_axisX->min()
        + ((viewportX - plotArea.left()) / plotArea.width()) * (m_axisX->max() - m_axisX->min());
    // The axis value is linear in profile index for both the pixel and wavelength axes
    const double firstX = m_displayXValues.first();
    const double lastX = m_displayXValues.last();
    const double index = (axisValue - firstX) / (lastX - firstX) * (m_displayXValues.size() - 1);
    if ((index < -0.5) || (index > m_displayXValues.size() - 0.5)) {
        return std::nan("");
    }
    return m_spectrumData.m_axisOrigin + index;
}

void CameraOpticalSpectrumDialog::handleCalibrationClick(const QPointF& viewportPos)
{
    const double pixel = pixelFromViewportX(viewportPos.x());
    if (std::isnan(pixel)) {
        return;
    }

    // Offer every known reference line; the user can also type a wavelength directly
    QStringList choices;
    const auto appendLines = [&choices](const QVector<CameraOpticalSpectrumRefLine>& lines) {
        for (const CameraOpticalSpectrumRefLine& line : lines) {
            choices.append(QStringLiteral("%1 (%2 nm)").arg(line.m_label).arg(line.m_nm));
        }
    };
    for (const CameraOpticalSpectrumRefLineSet& set : CameraOpticalSpectrumExtractor::referenceLineSets()) {
        appendLines(set.m_lines);
    }
    appendLines(CameraOpticalSpectrumExtractor::parseCustomLines(m_settings.m_opticalSpectrumCustomLines));

    QString prompt = tr("Wavelength of the clicked feature (nm):");
    if (std::abs(m_settings.m_opticalSpectrumRedshift) > 1e-9) {
        prompt += tr("\n\nNote: these are rest wavelengths - the redshift setting is not applied\nto calibration points. Calibrate on rest-frame features (e.g. telluric bands).");
    }
    bool ok = false;
    const QString chosen = QInputDialog::getItem(
        this,
        tr("Reference point %1").arg(m_calibrationClicks.size() + 1),
        prompt,
        choices,
        0,
        true,
        &ok);
    if (!ok) {
        return; // ignore this click, stay in calibration mode
    }

    // Accept either a list entry ("H-alpha (656.28 nm)") or a typed number
    const QRegularExpressionMatch match = QRegularExpression(QStringLiteral("(\\d+\\.?\\d*)\\s*(?:nm)?\\)?\\s*$")).match(chosen);
    const double nm = match.hasMatch() ? match.captured(1).toDouble() : 0.0;
    if (nm <= 0.0)
    {
        QMessageBox::warning(this, tr("Calibrate"), tr("Could not read a wavelength from '%1'").arg(chosen));
        return;
    }

    m_calibrationClicks.append(QPointF(pixel, nm));
    if (m_calibrationClicks.size() >= 2) {
        finishCalibration();
    } else {
        m_calibrateButton->setText(tr("Click 2nd point (or here)"));
    }
}

void CameraOpticalSpectrumDialog::finishCalibration()
{
    CameraOpticalSpectrumCalibration calibration;
    bool setZeroOrder = false;
    if (m_calibrationClicks.size() >= 2)
    {
        calibration = CameraOpticalSpectrumExtractor::calibrateTwoPoint(
            m_calibrationClicks[0].x(), m_calibrationClicks[0].y(),
            m_calibrationClicks[1].x(), m_calibrationClicks[1].y());
        setZeroOrder = true;
        if (!calibration.m_valid) {
            QMessageBox::warning(this, tr("Calibrate"),
                tr("The two reference points must be at different positions with different wavelengths"));
        }
    }
    else if (m_calibrationClicks.size() == 1)
    {
        calibration = CameraOpticalSpectrumExtractor::calibrateOnePoint(
            m_calibrationClicks[0].x(), m_calibrationClicks[0].y(), zeroOrderImagePosition());
        if (!calibration.m_valid) {
            QMessageBox::warning(this, tr("Calibrate"),
                tr("The reference point is too close to the zero-order position.\n"
                   "Check the zero-order setting, or click two reference points instead."));
        }
    }

    if (calibration.m_valid) {
        applyCalibration(calibration, setZeroOrder);
    }
    setCalibrationMode(false);
}

void CameraOpticalSpectrumDialog::applyCalibration(const CameraOpticalSpectrumCalibration& calibration, bool setZeroOrder)
{
    m_settings.m_opticalSpectrumDispersion = calibration.m_dispersion;
    m_settings.m_opticalSpectrumDirection = calibration.m_redPositive
        ? CameraSettings::OpticalSpectrumDirectionNormal
        : CameraSettings::OpticalSpectrumDirectionFlipped;
    QStringList settingsKeys{QStringLiteral("opticalSpectrumDispersion"), QStringLiteral("opticalSpectrumDirection")};
    if (setZeroOrder)
    {
        m_settings.m_opticalSpectrumZeroOrderAuto = false;
        m_settings.m_opticalSpectrumZeroOrderX = calibration.m_zeroOrderPx;
        settingsKeys.append(QStringLiteral("opticalSpectrumZeroOrderAuto"));
        settingsKeys.append(QStringLiteral("opticalSpectrumZeroOrderX"));
    }

    m_updatingControls = true;
    m_dispersionSpin->setValue(m_settings.m_opticalSpectrumDispersion);
    m_directionCombo->setCurrentIndex(static_cast<int>(m_settings.m_opticalSpectrumDirection));
    if (setZeroOrder)
    {
        m_zeroOrderAutoCheck->setChecked(false);
        m_zeroOrderSpin->setValue(m_settings.m_opticalSpectrumZeroOrderX);
        m_zeroOrderSpin->setEnabled(true);
    }
    m_updatingControls = false;

    emit settingsChanged(settingsKeys);
    updateChart();
}

void CameraOpticalSpectrumDialog::updateReferenceLines(double minY, double maxY)
{
    const double minX = qMin(m_axisX->min(), m_axisX->max());
    const double maxX = qMax(m_axisX->min(), m_axisX->max());
    const double redshift = m_settings.m_opticalSpectrumRedshift;
    const bool redshiftActive = std::abs(redshift) > 1e-9;
    const QVector<CameraOpticalSpectrumRefLine> lines = CameraOpticalSpectrumExtractor::selectedReferenceLines(
        m_settings.m_opticalSpectrumRefLines, m_settings.m_opticalSpectrumCustomLines);
    for (const CameraOpticalSpectrumRefLine& line : lines)
    {
        // Source lines are shifted by the entered redshift; terrestrial (telluric,
        // aurora/airglow) lines always stay at their rest wavelength.
        const bool shifted = redshiftActive && !line.m_terrestrial;
        const double nm = line.m_terrestrial ? line.m_nm : line.m_nm * (1.0 + redshift);
        if ((nm < minX) || (nm > maxX)) {
            continue;
        }
        // Tint shifted lines by direction: warm/orange for redshift, cool/blue for blueshift
        QColor lineColour(180, 180, 180);
        if (shifted) {
            lineColour = (redshift > 0.0) ? QColor(230, 150, 110) : QColor(110, 165, 235);
        }
        auto* series = new QLineSeries();
        QPen pen(lineColour);
        pen.setStyle(Qt::DashLine);
        pen.setWidth(1);
        series->setPen(pen);
        series->append(nm, minY);
        series->append(nm, maxY);
        m_chart->addSeries(series);
        series->attachAxis(m_axisX);
        series->attachAxis(m_axisY);
        const auto markers = m_chart->legend()->markers(series);
        for (QLegendMarker* marker : markers) {
            marker->setVisible(false);
        }
        m_referenceLineSeries.append(series);
        m_referenceLineWavelengths.append(nm);

        auto* label = new QGraphicsSimpleTextItem(line.m_label, m_chart);
        label->setBrush(QBrush(lineColour));
        QFont font = label->font();
        font.setPointSizeF(font.pointSizeF() * 0.85);
        label->setFont(font);
        m_referenceLineLabels.append(label);
    }
    positionReferenceLineLabels();
}

void CameraOpticalSpectrumDialog::updateRedshiftVelocityLabel()
{
    const double z = m_settings.m_opticalSpectrumRedshift;
    constexpr double kSpeedOfLightKmS = 299792.458;
    const double zp1Squared = (1.0 + z) * (1.0 + z);
    const double velocityKmS = kSpeedOfLightKmS * (zp1Squared - 1.0) / (zp1Squared + 1.0);
    m_redshiftVelocityLabel->setText(tr("%L1 km/s").arg(velocityKmS, 0, 'f', 0));
}

void CameraOpticalSpectrumDialog::positionReferenceLineLabels()
{
    for (int i = 0; i < m_referenceLineLabels.size(); i++)
    {
        const double nm = m_referenceLineWavelengths[i];
        QGraphicsSimpleTextItem* label = m_referenceLineLabels[i];
        const QPointF top = m_chart->mapToPosition(QPointF(nm, axisYMax()), m_referenceLineSeries[i]);
        label->setPos(top.x() + 2.0, top.y());
        label->setRotation(90.0);
        label->setVisible(m_chart->plotArea().contains(top));
    }
}

void CameraOpticalSpectrumDialog::exportCsv()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Export spectrum"), QString(), tr("CSV files (*.csv)"));
    if (fileName.isEmpty()) {
        return;
    }
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this, tr("Export spectrum"), tr("Failed to open %1 for writing").arg(fileName));
        return;
    }

    // Export the raw (unsmoothed, single-frame) pixel-domain profiles plus the
    // current wavelength mapping, so external tools get unmodified data. When the
    // instrument response is applied, a corrected luminance column is appended so
    // the correction is visible in the export too.
    QTextStream out(&file);
    const bool calibrated = isCalibrated();
    const bool corrected = m_settings.m_opticalSpectrumApplyResponse && calibrated && !m_responsePoints.isEmpty();
    out << "pixel,wavelength_nm,luminance,red,green,blue";
    if (corrected) {
        out << ",luminance_corrected";
    }
    out << '\n';
    for (int i = 0; i < m_spectrumData.m_luminance.size(); i++)
    {
        out << (m_spectrumData.m_axisOrigin + i) << ',';
        if (calibrated) {
            out << QString::number(wavelengthAt(i), 'f', 2);
        }
        out << ','
            << m_spectrumData.m_luminance[i] << ','
            << m_spectrumData.m_red[i] << ','
            << m_spectrumData.m_green[i] << ','
            << m_spectrumData.m_blue[i];
        if (corrected)
        {
            const double response = CameraOpticalSpectrumLibrary::responseAt(m_responsePoints, wavelengthAt(i));
            out << ',';
            if (response > 0.05) {
                out << QString::number(m_spectrumData.m_luminance[i] / response, 'f', 2);
            }
        }
        out << '\n';
    }
}
