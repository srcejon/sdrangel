///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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

#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFontMetrics>
#include <QHeaderView>
#include <QLabel>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>

#include "device/deviceuiset.h"
#include "device/deviceapi.h"
#include "channel/channelwebapiutils.h"
#include "dsp/dspcommands.h"
#include "dsp/spectrumsettings.h"
#include "feature/feature.h"
#include "feature/featureset.h"
#include "gui/basicchannelsettingsdialog.h"
#include "gui/buttonswitch.h"
#include "gui/dialogpositioner.h"
#include "gui/dialpopup.h"
#include "gui/glspectrum.h"
#include "gui/glspectrumgui.h"
#include "gui/rollupcontents.h"
#include "maincore.h"
#include "plugin/pluginapi.h"
#include "util/csv.h"

#include "meteorgui.h"
#include "rmobreport.h"
#include "ui_meteorgui.h"

namespace {
    constexpr int DetectionSampleUtcMSecsRole = Qt::UserRole + 1;
    constexpr int DetectionOverlayIdRole = Qt::UserRole + 2;
    constexpr int DetectionDisplayUtcMSecsRole = Qt::UserRole + 3;
    constexpr int DetectionRadioRole = Qt::UserRole + 4;
    constexpr int RotatorAvailableRole = Qt::UserRole + 1;
    constexpr int MeteorTrailFFTSize = 1024;
    constexpr int MeteorTrailFFTOverlap = 512;
    constexpr int MeteorHeadFFTSize = 128;
    constexpr int MeteorHeadFFTOverlap = 96;
    constexpr qint64 AutomaticRMOBSaveIntervalS = 60;

    class NumericTableWidgetItem : public QTableWidgetItem
    {
    public:
        explicit NumericTableWidgetItem(const QString& text) :
            QTableWidgetItem(text)
        {}

        bool operator<(const QTableWidgetItem& other) const override
        {
            const QVariant left = data(Qt::UserRole);
            const QVariant right = other.data(Qt::UserRole);

            if (left.isValid() && right.isValid()) {
                return left.toDouble() < right.toDouble();
            }

            return QTableWidgetItem::operator<(other);
        }
    };
}

MeteorGUI* MeteorGUI::create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel)
{
    return new MeteorGUI(pluginAPI, deviceUISet, rxChannel);
}

void MeteorGUI::destroy()
{
    delete this;
}

void MeteorGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applyAllSettings();
}

QByteArray MeteorGUI::serialize() const
{
    return m_settings.serialize();
}

bool MeteorGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        displaySettings();
        applyAllSettings();
        return true;
    }
    else
    {
        resetToDefaults();
        return false;
    }
}

MeteorGUI::MeteorGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel, QWidget* parent) :
    ChannelGUI(parent),
    ui(new Ui::MeteorGUI),
    m_pluginAPI(pluginAPI),
    m_deviceUISet(deviceUISet),
    m_channelMarker(this),
    m_deviceCenterFrequency(0),
    m_basebandSampleRate(1),
    m_doApplySettings(true),
    m_tickCount(0),
    m_meteor(reinterpret_cast<Meteor*>(rxChannel)),
    m_spectrumVis(nullptr),
    m_headSpectrumVis(nullptr),
    m_hourlyChart(nullptr),
    m_totalCount(0),
    m_highlightAllDetectionOverlays(true),
    m_nextDetectionOverlayId(1),
    m_detectionOverlayWindowValid(false),
    m_headDetectionOverlayWindowValid(false)
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/channelrx/meteor/readme.md";
    RollupContents *rollupContents = getRollupContents();
    setupUi(rollupContents);
    setSizePolicy(rollupContents->sizePolicy());
    rollupContents->arrangeRollups();

    connect(rollupContents, SIGNAL(widgetRolled(QWidget*,bool)), this, SLOT(onWidgetRolled(QWidget*,bool)));
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));

    m_meteor->setMessageQueueToGUI(getInputMessageQueue());
    m_spectrumVis = m_meteor->getSpectrumVis();
    m_headSpectrumVis = m_meteor->getHeadSpectrumVis();

    setupSpectrum();

    connect(&MainCore::instance()->getMasterTimer(), SIGNAL(timeout()), this, SLOT(tick()));
    connect(MainCore::instance(), &MainCore::featureAdded, this, &MeteorGUI::onFeatureAdded);
    connect(MainCore::instance(), &MainCore::featureRemoved, this, &MeteorGUI::onFeatureRemoved);

    m_channelMarker.blockSignals(true);
    m_channelMarker.setColor(QColor(m_settings.m_rgbColor));
    m_channelMarker.setBandwidth(m_settings.m_channelSampleRate);
    m_channelMarker.setCenterFrequency(m_settings.m_inputFrequencyOffset);
    m_channelMarker.setTitle(m_settings.m_title);
    m_channelMarker.blockSignals(false);
    m_channelMarker.setVisible(true);

    setTitleColor(m_channelMarker.getColor());
    m_settings.setChannelMarker(&m_channelMarker);
    m_settings.setSpectrumGUI(m_spectrumGUI);
    m_settings.setHeadSpectrumGUI(m_headSpectrumGUI);
    m_settings.setRollupState(&m_rollupState);

    m_deviceUISet->addChannelMarker(&m_channelMarker);

    connect(&m_channelMarker, SIGNAL(changedByCursor()), this, SLOT(channelMarkerChangedByCursor()));
    connect(&m_channelMarker, SIGNAL(highlightedByCursor()), this, SLOT(channelMarkerHighlightedByCursor()));
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    loadRMOBReport(automaticRMOBReportFileName(QDate::currentDate()));
    updateHistogram();
    updateColorgramme();
    displaySettings();
    makeUIConnections();
    applyAllSettings();
    DialPopup::addPopupsToChildDials(this);
    m_resizer.enableChildMouseTracking();
}

MeteorGUI::~MeteorGUI()
{
    saveAutomaticRMOBReports();
    disconnect(&MainCore::instance()->getMasterTimer(), SIGNAL(timeout()), this, SLOT(tick()));
    m_glSpectrum->setPaintGLCallback(nullptr);
    m_headGLSpectrum->setPaintGLCallback(nullptr);
    for (QLabel *label : m_detectionOverlayLabels) {
        delete label;
    }
    m_detectionOverlayLabels.clear();
    for (QLabel *label : m_headDetectionOverlayLabels) {
        delete label;
    }
    m_headDetectionOverlayLabels.clear();
    delete ui;
}

void MeteorGUI::setupUi(RollupContents *rollupContents)
{
    ui->setupUi(rollupContents);

    m_frequencyMode = ui->frequencyMode;
    m_deltaFrequency = ui->deltaFrequency;
    m_deltaUnits = ui->deltaUnits;
    m_sampleRate = ui->sampleRate;
    m_powerLPFCutoff = ui->powerLPFCutoff;
    m_detectionThreshold = ui->detectionThreshold;
    m_minDuration = ui->minDuration;
    m_maxDuration = ui->maxDuration;
    m_maxFrequencyDrift = ui->maxFrequencyDrift;
    m_highlightAllDetections = ui->highlightAllDetections;
    m_detectionBoxPadding = ui->detectionBoxPadding;
    m_detectionLabels = ui->detectionLabels;
    m_transmitterLatitude = ui->transmitterLatitude;
    m_transmitterLongitude = ui->transmitterLongitude;
    m_antennaAzimuth = ui->antennaAzimuth;
    m_antennaElevation = ui->antennaElevation;
    m_antennaBeamwidth = ui->antennaBeamwidth;
    m_rotator = ui->rotator;
    m_totalCountText = ui->totalCountText;
    m_hourCountText = ui->hourCountText;
    m_saveDetections = ui->saveDetections;
    m_saveColorgramme = ui->saveColorgramme;
    m_clearDetections = ui->clearDetections;
    m_detectionsTable = ui->detectionsTable;
    m_colorgrammeTable = ui->colorgrammeTable;
    m_hourlyChartView = ui->hourlyChartView;
    m_glSpectrum = ui->glSpectrum;
    m_spectrumGUI = ui->spectrumGUI;
    m_headGLSpectrum = ui->headGLSpectrum;
    m_headSpectrumGUI = ui->headSpectrumGUI;

    m_frequencyMode->addItem("df");
    m_frequencyMode->addItem("f");
    m_frequencyMode->setToolTip("Select frequency entry mode");
    m_deltaFrequency->setColorMapper(ColorMapper(ColorMapper::GrayGold));
    m_deltaFrequency->setValueRange(false, 7, -9999999, 9999999);

    m_sampleRate->addItems(QStringList({"100 Hz", "300 Hz", "1000 Hz", "3000 Hz"}));
    m_sampleRate->setToolTip("Meteor detector channel sample rate");

    m_powerLPFCutoff->setDecimals(1);
    m_powerLPFCutoff->setRange(0.1, 1350.0);
    m_powerLPFCutoff->setSuffix(" Hz");
    m_powerLPFCutoff->setToolTip("Power low pass filter cutoff frequency");

    m_detectionThreshold->setDecimals(1);
    m_detectionThreshold->setRange(1.0, 60.0);
    m_detectionThreshold->setSuffix(" dB");
    m_detectionThreshold->setToolTip("Detection threshold above the tracked noise floor");

    m_minDuration->setRange(1, 10000);
    m_minDuration->setSuffix(" ms");
    m_minDuration->setToolTip("Minimum pulse duration");

    m_maxDuration->setRange(1, 60000);
    m_maxDuration->setSuffix(" ms");
    m_maxDuration->setToolTip("Maximum pulse duration");

    m_maxFrequencyDrift->setDecimals(1);
    m_maxFrequencyDrift->setRange(0.0, 1500.0);
    m_maxFrequencyDrift->setSuffix(" Hz");
    m_maxFrequencyDrift->setToolTip("Maximum frequency span or drift accepted for a meteor pulse");

    m_highlightAllDetections->setChecked(true);
    m_highlightAllDetections->setToolTip("Highlight all detections on the waterfall. Turn off to highlight only selected table rows.");

    m_detectionBoxPadding->setRange(0, 100);
    m_detectionBoxPadding->setSuffix(" px");
    m_detectionBoxPadding->setToolTip("Extra waterfall bounding box width in pixels on each side of the detected frequency range");

    m_detectionLabels->addItems(QStringList({"None", "Top", "Right"}));
    m_detectionLabels->setToolTip("Display peak power and duration labels on waterfall detection boxes");

    m_transmitterLatitude->setDecimals(6);
    m_transmitterLatitude->setRange(-90.0, 90.0);
    m_transmitterLatitude->setSingleStep(0.0001);
    m_transmitterLatitude->setSuffix(QString::fromUtf8("\u00b0"));
    m_transmitterLatitude->setToolTip("Radar transmitter latitude in decimal degrees");

    m_transmitterLongitude->setDecimals(6);
    m_transmitterLongitude->setRange(-180.0, 180.0);
    m_transmitterLongitude->setSingleStep(0.0001);
    m_transmitterLongitude->setSuffix(QString::fromUtf8("\u00b0"));
    m_transmitterLongitude->setToolTip("Radar transmitter longitude in decimal degrees");

    m_antennaAzimuth->setDecimals(1);
    m_antennaAzimuth->setRange(0.0, 360.0);
    m_antennaAzimuth->setSuffix(QString::fromUtf8("\u00b0"));
    m_antennaAzimuth->setToolTip("Receive antenna azimuth clockwise from north");

    m_antennaElevation->setDecimals(1);
    m_antennaElevation->setRange(-90.0, 90.0);
    m_antennaElevation->setSuffix(QString::fromUtf8("\u00b0"));
    m_antennaElevation->setToolTip("Receive antenna elevation above the horizon");

    m_antennaBeamwidth->setDecimals(1);
    m_antennaBeamwidth->setRange(0.0, 360.0);
    m_antennaBeamwidth->setSuffix(QString::fromUtf8("\u00b0"));
    m_antennaBeamwidth->setToolTip("Receive antenna half-power beamwidth; zero means unspecified");

    m_rotator->setToolTip("Optionally follow the azimuth and elevation reported by a GS232Controller feature");
    populateRotatorCombo();

    m_saveDetections->setIcon(QIcon(":/save.png"));
    m_saveDetections->setToolTip("Save detections to CSV");
    m_saveDetections->setMaximumWidth(28);
    m_saveColorgramme->setIcon(QIcon(":/save.png"));
    m_saveColorgramme->setToolTip("Save Colorgramme to RMOB text report");
    m_saveColorgramme->setMaximumWidth(28);

    m_clearDetections->setIcon(QIcon(":/bin.png"));
    m_clearDetections->setToolTip("Clear detections");
    m_clearDetections->setMaximumWidth(28);

    m_detectionsTable->setColumnCount(12);
    const QStringList detectionHeaders = {
        "Time (local)",
        "Peak (dB)",
        "BG (dB)",
        "Total (dB)",
        "Amp",
        "Duration (ms)",
        "Center (Hz)",
        "Span (Hz)",
        "Drift (Hz)",
        "Rate (Hz)",
        "Mag",
        "Flux"
    };
    const QStringList detectionHeaderTooltips = {
        "Local date and time when the meteor pulse started",
        "Peak signal power during the detection in dB",
        "Estimated background power level at detection time in dB",
        "Integrated meteor signal power across the detection in dB",
        "Peak linear signal amplitude during the detection",
        "Detected pulse duration in milliseconds",
        "Estimated center frequency of the pulse relative to channel center in hertz",
        "Estimated frequency span across the pulse in hertz",
        "Estimated start-to-end frequency drift across the pulse in hertz",
        "Meteor channel detector sample rate in hertz",
        "Camera photometry magnitude for camera-object meteor detections",
        "Camera photometry flux for camera-object meteor detections"
    };

    for (int i = 0; i < detectionHeaders.size(); i++)
    {
        QTableWidgetItem *headerItem = new QTableWidgetItem(detectionHeaders[i]);
        headerItem->setToolTip(detectionHeaderTooltips[i]);
        m_detectionsTable->setHorizontalHeaderItem(i, headerItem);
    }

    m_detectionsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_detectionsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_detectionsTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_detectionsTable->setContextMenuPolicy(Qt::CustomContextMenu);
    m_detectionsTable->horizontalHeader()->setStretchLastSection(true);
    m_detectionsTable->horizontalHeader()->setSectionsMovable(true);
    m_detectionsTable->horizontalHeader()->setContextMenuPolicy(Qt::CustomContextMenu);
    m_detectionsTable->verticalHeader()->setVisible(true);
    m_detectionsTable->setSortingEnabled(true);
    m_detectionsTable->setMinimumHeight(120);

    m_colorgrammeTable->setRowCount(24);
    m_colorgrammeTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_colorgrammeTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_colorgrammeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_colorgrammeTable->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_colorgrammeTable->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
    m_colorgrammeTable->verticalHeader()->setDefaultAlignment(Qt::AlignCenter);
    m_colorgrammeTable->setShowGrid(true);
    m_colorgrammeTable->setGridStyle(Qt::SolidLine);
    m_colorgrammeTable->setStyleSheet("QTableWidget { gridline-color: black; }");
    m_colorgrammeTable->setMinimumHeight(180);
    m_colorgrammeTable->setToolTip("Monthly meteor count colorgramme: columns are days, rows are UTC/local display hours, color shows detections per hour");

    for (int hour = 0; hour < 24; hour++)
    {
        QTableWidgetItem *hourItem = new QTableWidgetItem(QString::number(hour));
        hourItem->setTextAlignment(Qt::AlignCenter);
        m_colorgrammeTable->setVerticalHeaderItem(hour, hourItem);
    }

    m_hourlyChartView->setMinimumHeight(180);
    m_glSpectrum->setMinimumHeight(180);
    m_headGLSpectrum->setMinimumHeight(180);
}

void MeteorGUI::populateRotatorCombo()
{
    const QString currentSelection = m_settings.m_rotator.trimmed();
    QSignalBlocker blocker(m_rotator);
    m_rotator->clear();
    m_rotator->addItem(tr("None"), QString());
    m_rotator->setItemData(0, true, RotatorAvailableRole);

    std::vector<FeatureSet *>& featureSets = MainCore::instance()->getFeatureeSets();

    for (int featureSetIndex = 0; featureSetIndex < (int) featureSets.size(); featureSetIndex++)
    {
        FeatureSet *featureSet = featureSets[featureSetIndex];

        if (!featureSet) {
            continue;
        }

        for (int featureIndex = 0; featureIndex < featureSet->getNumberOfFeatures(); featureIndex++)
        {
            Feature *feature = featureSet->getFeatureAt(featureIndex);

            if (!feature || (feature->getURI() != QLatin1String("sdrangel.feature.gs232controller"))) {
                continue;
            }

            QString title;
            feature->getTitle(title);

            if (title.isEmpty()) {
                title = tr("GS232 Controller");
            }

            const QString id = QStringLiteral("%1:%2").arg(featureSetIndex).arg(featureIndex);
            m_rotator->addItem(
                tr("F%1:%2 %3").arg(featureSetIndex).arg(featureIndex).arg(title),
                id);
            m_rotator->setItemData(m_rotator->count() - 1, true, RotatorAvailableRole);
        }
    }

    int index = m_rotator->findData(currentSelection);

    if ((index < 0) && !currentSelection.isEmpty())
    {
        m_rotator->addItem(tr("Unavailable: %1").arg(currentSelection), currentSelection);
        m_rotator->setItemData(m_rotator->count() - 1, false, RotatorAvailableRole);
        index = m_rotator->count() - 1;
    }

    m_rotator->setCurrentIndex(index >= 0 ? index : 0);
    updateAntennaPointingControls();
}

void MeteorGUI::updateAntennaPointingControls()
{
    const bool followingRotator = !m_settings.m_rotator.isEmpty()
        && m_rotator->itemData(m_rotator->currentIndex(), RotatorAvailableRole).toBool();
    m_antennaAzimuth->setReadOnly(followingRotator);
    m_antennaElevation->setReadOnly(followingRotator);
}

QPair<int, int> MeteorGUI::selectedRotatorIndices() const
{
    const QStringList parts = m_settings.m_rotator.trimmed().split(QLatin1Char(':'));

    if (parts.size() != 2) {
        return qMakePair(-1, -1);
    }

    bool featureSetOK = false;
    bool featureOK = false;
    const int featureSetIndex = parts.at(0).toInt(&featureSetOK);
    const int featureIndex = parts.at(1).toInt(&featureOK);

    if (!featureSetOK || !featureOK || (featureSetIndex < 0) || (featureIndex < 0)) {
        return qMakePair(-1, -1);
    }

    return qMakePair(featureSetIndex, featureIndex);
}

void MeteorGUI::syncFromSelectedRotator()
{
    const QPair<int, int> indices = selectedRotatorIndices();

    if ((indices.first < 0) || (indices.second < 0)) {
        return;
    }

    double azimuth = 0.0;
    double elevation = 0.0;

    if ((!ChannelWebAPIUtils::getFeatureReportValue(indices.first, indices.second, "currentAzimuth", azimuth)
            || !ChannelWebAPIUtils::getFeatureReportValue(indices.first, indices.second, "currentElevation", elevation))
        && (!ChannelWebAPIUtils::getFeatureSetting(indices.first, indices.second, "azimuth", azimuth)
            || !ChannelWebAPIUtils::getFeatureSetting(indices.first, indices.second, "elevation", elevation)))
    {
        return;
    }

    if (!std::isfinite(azimuth) || !std::isfinite(elevation)) {
        return;
    }

    azimuth = std::fmod(azimuth, 360.0);

    if (azimuth < 0.0) {
        azimuth += 360.0;
    }

    elevation = qBound(-90.0, elevation, 90.0);

    if ((std::fabs((double) m_settings.m_antennaAzimuth - azimuth) < 0.05)
        && (std::fabs((double) m_settings.m_antennaElevation - elevation) < 0.05))
    {
        return;
    }

    m_settings.m_antennaAzimuth = (float) azimuth;
    m_settings.m_antennaElevation = (float) elevation;

    const QSignalBlocker azimuthBlocker(m_antennaAzimuth);
    const QSignalBlocker elevationBlocker(m_antennaElevation);
    m_antennaAzimuth->setValue(azimuth);
    m_antennaElevation->setValue(elevation);
    applySettings({"antennaAzimuth", "antennaElevation"});
}

void MeteorGUI::setupSpectrum()
{
    auto setupDisplay = [this](
        SpectrumVis *spectrumVis,
        GLSpectrum *glSpectrum,
        GLSpectrumGUI *spectrumGUI,
        int fftSize,
        int fftOverlap,
        int fpsIndex,
        QVector<QLabel *>& overlayLabels,
        bool& overlayWindowValid,
        QDateTime& overlayWindowStartUtc,
        QDateTime& overlayWindowEndUtc)
    {
        spectrumVis->setGLSpectrum(glSpectrum);
        spectrumGUI->setBuddies(spectrumVis, glSpectrum);
        spectrumGUI->setFFTSize((int) std::round(std::log2((double) fftSize)));

        if (QSpinBox *overlapControl = spectrumGUI->findChild<QSpinBox *>("fftOverlap")) {
            overlapControl->setValue(fftOverlap);
        }

        if ((fpsIndex >= 0) && spectrumGUI->findChild<QComboBox *>("fps")) {
            spectrumGUI->findChild<QComboBox *>("fps")->setCurrentIndex(fpsIndex);
        }

        glSpectrum->setCenterFrequency(0);
        glSpectrum->setSampleRate(m_settings.m_channelSampleRate);
        glSpectrum->setLsbDisplay(false);

        SpectrumSettings spectrumSettings = spectrumVis->getSettings();
        spectrumSettings.m_fftSize = fftSize;
        spectrumSettings.m_fftOverlap = fftOverlap;
        spectrumSettings.m_displayWaterfall = true;
        spectrumSettings.m_displayMaxHold = false;
        spectrumSettings.m_averagingMode = SpectrumSettings::AvgModeNone;
        spectrumSettings.m_averagingIndex = 0;
        spectrumSettings.m_scrollBar = true;

        if (fpsIndex == 4) {
            spectrumSettings.m_fpsPeriodMs = 20;
        }

        SpectrumVis::MsgConfigureSpectrumVis *msg = SpectrumVis::MsgConfigureSpectrumVis::create(spectrumSettings, false);
        spectrumVis->getInputMessageQueue()->push(msg);
        QVector<QLabel *> *overlayLabelsPtr = &overlayLabels;
        bool *overlayWindowValidPtr = &overlayWindowValid;
        QDateTime *overlayWindowStartUtcPtr = &overlayWindowStartUtc;
        QDateTime *overlayWindowEndUtcPtr = &overlayWindowEndUtc;
        glSpectrum->setPaintGLCallback([
            this,
            spectrumVis,
            overlayLabelsPtr,
            overlayWindowValidPtr,
            overlayWindowStartUtcPtr,
            overlayWindowEndUtcPtr](GLSpectrumView *spectrumView)
        {
            drawDetectionOverlays(
                spectrumView,
                spectrumVis,
                *overlayLabelsPtr,
                *overlayWindowValidPtr,
                *overlayWindowStartUtcPtr,
                *overlayWindowEndUtcPtr);
        });

        GLSpectrumView *spectrumView = glSpectrum->getSpectrumView();
        QScrollBar *scrollBar = spectrumView->parentWidget()
            ? spectrumView->parentWidget()->findChild<QScrollBar *>(QString(), Qt::FindDirectChildrenOnly)
            : nullptr;

        if (scrollBar)
        {
            connect(scrollBar, &QScrollBar::valueChanged, this, [spectrumView, overlayWindowValidPtr]() {
                *overlayWindowValidPtr = false;
                spectrumView->update();
            });
        }
    };

    setupDisplay(
        m_spectrumVis,
        m_glSpectrum,
        m_spectrumGUI,
        MeteorTrailFFTSize,
        MeteorTrailFFTOverlap,
        -1,
        m_detectionOverlayLabels,
        m_detectionOverlayWindowValid,
        m_detectionOverlayWindowStartUtc,
        m_detectionOverlayWindowEndUtc);
    setupDisplay(
        m_headSpectrumVis,
        m_headGLSpectrum,
        m_headSpectrumGUI,
        MeteorHeadFFTSize,
        MeteorHeadFFTOverlap,
        4,
        m_headDetectionOverlayLabels,
        m_headDetectionOverlayWindowValid,
        m_headDetectionOverlayWindowStartUtc,
        m_headDetectionOverlayWindowEndUtc);
}

bool MeteorGUI::handleMessage(const Message& message)
{
    if (Meteor::MsgConfigureMeteor::match(message))
    {
        const Meteor::MsgConfigureMeteor& cfg = (Meteor::MsgConfigureMeteor&) message;
        m_settings = cfg.getSettings();
        blockApplySettings(true);
        m_channelMarker.updateSettings(static_cast<const ChannelMarker*>(m_settings.m_channelMarker));
        displaySettings();
        blockApplySettings(false);
        return true;
    }
    else if (DSPSignalNotification::match(message))
    {
        DSPSignalNotification& notif = (DSPSignalNotification&) message;
        m_deviceCenterFrequency = notif.getCenterFrequency();
        m_basebandSampleRate = notif.getSampleRate();
        calcOffset();
        updateAbsoluteCenterFrequency();
        return true;
    }
    else if (MeteorDemodSink::MsgMeteorDetected::match(message))
    {
        const MeteorDemodSink::MsgMeteorDetected& detection = (const MeteorDemodSink::MsgMeteorDetected&) message;
        addDetection(detection);
        return true;
    }
    else if (Meteor::MsgCameraMeteorDetected::match(message))
    {
        const Meteor::MsgCameraMeteorDetected& detection = (const Meteor::MsgCameraMeteorDetected&) message;
        addCameraDetection(detection);
        return true;
    }
    else if (MeteorDemodSink::MsgMeteorDataCollected::match(message))
    {
        const MeteorDemodSink::MsgMeteorDataCollected& data = (const MeteorDemodSink::MsgMeteorDataCollected&) message;
        const QDateTime localTime = data.getDateTimeUtc().toLocalTime();

        if (markHourData(localTime.date(), localTime.time().hour()))
        {
            updateHistogram();
            updateColorgramme();
        }

        updateCounters();
        return true;
    }

    return false;
}

void MeteorGUI::handleInputMessages()
{
    Message* message;

    while ((message = getInputMessageQueue()->pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void MeteorGUI::channelMarkerChangedByCursor()
{
    m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
    m_settings.m_frequency = m_deviceCenterFrequency + m_settings.m_inputFrequencyOffset;

    qint64 value = 0;

    if (m_settings.m_frequencyMode == MeteorSettings::Offset) {
        value = m_settings.m_inputFrequencyOffset;
    } else {
        value = m_settings.m_frequency;
    }

    m_deltaFrequency->blockSignals(true);
    m_deltaFrequency->setValue(value);
    m_deltaFrequency->blockSignals(false);

    updateAbsoluteCenterFrequency();
    applySettings({"frequency", "inputFrequencyOffset"});
}

void MeteorGUI::channelMarkerHighlightedByCursor()
{
    setHighlighted(m_channelMarker.getHighlighted());
}

void MeteorGUI::on_deltaFrequency_changed(qint64 value)
{
    qint64 offset = 0;

    if (m_settings.m_frequencyMode == MeteorSettings::Offset)
    {
        offset = value;
        m_settings.m_frequency = m_deviceCenterFrequency + offset;
    }
    else
    {
        m_settings.m_frequency = value;
        offset = m_settings.m_frequency - m_deviceCenterFrequency;
    }

    m_channelMarker.setCenterFrequency(offset);
    m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
    updateAbsoluteCenterFrequency();
    applySettings({"frequency", "inputFrequencyOffset"});
}

void MeteorGUI::on_frequencyMode_currentIndexChanged(int index)
{
    m_settings.m_frequencyMode = (MeteorSettings::FrequencyMode) index;
    m_deltaFrequency->blockSignals(true);

    if (m_settings.m_frequencyMode == MeteorSettings::Offset)
    {
        m_deltaFrequency->setValueRange(false, 7, -9999999, 9999999);
        m_deltaFrequency->setValue(m_settings.m_inputFrequencyOffset);
        m_deltaUnits->setText("Hz");
    }
    else
    {
        m_deltaFrequency->setValueRange(true, 11, 0, 99999999999, 0);
        m_deltaFrequency->setValue(m_settings.m_frequency);
        m_deltaUnits->setText("Hz");
    }

    m_deltaFrequency->blockSignals(false);

    updateAbsoluteCenterFrequency();
    applySetting("frequencyMode");
}

void MeteorGUI::on_sampleRate_currentIndexChanged(int index)
{
    if ((index < 0) || (index >= 4)) {
        return;
    }

    m_settings.m_channelSampleRate = MeteorSettings::m_supportedSampleRates[index];
    const double maxCutoff = std::max(0.1, m_settings.m_channelSampleRate * 0.45);
    m_powerLPFCutoff->setMaximum(maxCutoff);

    QStringList keys({"channelSampleRate"});
    if (m_settings.m_powerLPFCutoff > maxCutoff)
    {
        m_settings.m_powerLPFCutoff = maxCutoff;
        m_powerLPFCutoff->setValue(m_settings.m_powerLPFCutoff);
        keys.append("powerLPFCutoff");
    }

    m_channelMarker.setBandwidth(m_settings.m_channelSampleRate);
    updateVisualSampleRate();
    applySettings(keys);
}

void MeteorGUI::on_powerLPFCutoff_valueChanged(double value)
{
    m_settings.m_powerLPFCutoff = (float) value;
    applySetting("powerLPFCutoff");
}

void MeteorGUI::on_detectionThreshold_valueChanged(double value)
{
    m_settings.m_detectionThresholdDB = (float) value;
    applySetting("detectionThresholdDB");
}

void MeteorGUI::on_minDuration_valueChanged(int value)
{
    m_settings.m_minDurationMS = value;

    if (m_settings.m_maxDurationMS < m_settings.m_minDurationMS)
    {
        m_settings.m_maxDurationMS = m_settings.m_minDurationMS;
        m_maxDuration->setValue(m_settings.m_maxDurationMS);
        applySettings({"minDurationMS", "maxDurationMS"});
    }
    else
    {
        applySetting("minDurationMS");
    }
}

void MeteorGUI::on_maxDuration_valueChanged(int value)
{
    m_settings.m_maxDurationMS = value;

    if (m_settings.m_minDurationMS > m_settings.m_maxDurationMS)
    {
        m_settings.m_minDurationMS = m_settings.m_maxDurationMS;
        m_minDuration->setValue(m_settings.m_minDurationMS);
        applySettings({"minDurationMS", "maxDurationMS"});
    }
    else
    {
        applySetting("maxDurationMS");
    }
}

void MeteorGUI::on_maxFrequencyDrift_valueChanged(double value)
{
    m_settings.m_maxFrequencyDrift = (float) value;
    applySetting("maxFrequencyDrift");
}

void MeteorGUI::on_highlightAllDetections_toggled(bool checked)
{
    m_highlightAllDetectionOverlays = checked;
    invalidateDetectionOverlayWindows();
    updateSpectrumViews();
}

void MeteorGUI::on_detectionBoxPadding_valueChanged(int value)
{
    m_settings.m_detectionBoxPaddingPixels = value;
    applySetting("detectionBoxPaddingPixels");
    updateSpectrumViews();
}

void MeteorGUI::on_detectionLabels_currentIndexChanged(int index)
{
    m_settings.m_detectionLabelMode = (MeteorSettings::DetectionLabelMode) std::clamp(
        index,
        (int) MeteorSettings::DetectionLabelNone,
        (int) MeteorSettings::DetectionLabelRight);
    applySetting("detectionLabelMode");
    updateSpectrumViews();
}

void MeteorGUI::on_transmitterLatitude_valueChanged(double value)
{
    m_settings.m_transmitterLatitude = value;
    applySetting("transmitterLatitude");
}

void MeteorGUI::on_transmitterLongitude_valueChanged(double value)
{
    m_settings.m_transmitterLongitude = value;
    applySetting("transmitterLongitude");
}

void MeteorGUI::on_antennaAzimuth_valueChanged(double value)
{
    m_settings.m_antennaAzimuth = (float) value;
    applySetting("antennaAzimuth");
}

void MeteorGUI::on_antennaElevation_valueChanged(double value)
{
    m_settings.m_antennaElevation = (float) value;
    applySetting("antennaElevation");
}

void MeteorGUI::on_antennaBeamwidth_valueChanged(double value)
{
    m_settings.m_antennaBeamwidth = (float) value;
    applySetting("antennaBeamwidth");
}

void MeteorGUI::on_rotator_currentIndexChanged(int index)
{
    m_settings.m_rotator = m_rotator->itemData(index).toString();
    updateAntennaPointingControls();
    applySetting("rotator");

    if (!m_settings.m_rotator.isEmpty()) {
        syncFromSelectedRotator();
    }
}

void MeteorGUI::on_saveDetections_clicked()
{
    QFileDialog fileDialog(this, "Select file to save detections to", "", "*.csv");
    fileDialog.setDefaultSuffix("csv");
    fileDialog.setAcceptMode(QFileDialog::AcceptSave);

    if (!fileDialog.exec()) {
        return;
    }

    const QStringList fileNames = fileDialog.selectedFiles();

    if (fileNames.isEmpty()) {
        return;
    }

    QFile file(fileNames[0]);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::critical(this, "Meteor", QString("Failed to open file %1").arg(fileNames[0]));
        return;
    }

    QTextStream out(&file);
    QHeaderView *header = m_detectionsTable->horizontalHeader();
    QVector<int> visibleColumns;

    for (int visualIndex = 0; visualIndex < header->count(); visualIndex++)
    {
        const int logicalIndex = header->logicalIndex(visualIndex);

        if (!m_detectionsTable->isColumnHidden(logicalIndex)) {
            visibleColumns.push_back(logicalIndex);
        }
    }

    for (int i = 0; i < visibleColumns.size(); i++)
    {
        if (i > 0) {
            out << ",";
        }

        out << CSV::escape(m_detectionsTable->horizontalHeaderItem(visibleColumns[i])->text());
    }

    out << "\n";

    for (int row = 0; row < m_detectionsTable->rowCount(); row++)
    {
        for (int i = 0; i < visibleColumns.size(); i++)
        {
            if (i > 0) {
                out << ",";
            }

            QTableWidgetItem *item = m_detectionsTable->item(row, visibleColumns[i]);
            out << CSV::escape(item ? item->text() : QString());
        }

        out << "\n";
    }
}

void MeteorGUI::on_saveColorgramme_clicked()
{
    const QDate monthDate = colorgrammeMonthDate();
    const int year = monthDate.year();
    const int month = monthDate.month();
    QFileDialog fileDialog(
        this,
        "Select file to save Colorgramme to",
        QString("meteor_%1_%2.rmob.txt").arg(year).arg(month, 2, 10, QLatin1Char('0')),
        "*.txt"
    );
    fileDialog.setDefaultSuffix("txt");
    fileDialog.setAcceptMode(QFileDialog::AcceptSave);

    if (!fileDialog.exec()) {
        return;
    }

    const QStringList fileNames = fileDialog.selectedFiles();

    if (fileNames.isEmpty()) {
        return;
    }

    QString error;

    if (!saveRMOBReport(fileNames[0], &error))
    {
        QMessageBox::critical(this, "Meteor", error);
        return;
    }
}

bool MeteorGUI::loadRMOBReport(const QString& fileName)
{
    RMOBReport::Data data;

    if (!RMOBReport::load(fileName, QDate::currentDate(), data)) {
        return false;
    }

    m_hourlyCounts = data.m_hourlyCounts;
    m_hourlyData = data.m_hourlyData;
    m_totalCount = data.m_totalCount;
    return true;
}

bool MeteorGUI::saveRMOBReport(const QString& fileName, QString *error) const
{
    return saveRMOBReport(fileName, colorgrammeMonthDate(), error);
}

bool MeteorGUI::saveRMOBReport(const QString& fileName, const QDate& reportMonthDate, QString *error) const
{
    RMOBReport::Data data;
    data.m_hourlyCounts = m_hourlyCounts;
    data.m_hourlyData = m_hourlyData;

    RMOBReport::Metadata metadata;
    metadata.m_observer = MainCore::instance()->getSettings().getStationName();
    metadata.m_latitude = MainCore::instance()->getSettings().getLatitude();
    metadata.m_longitude = MainCore::instance()->getSettings().getLongitude();
    metadata.m_frequency = rmobReportFrequency();
    metadata.m_receiver = rmobReceiverName();
    metadata.m_antennaAzimuth = m_settings.m_antennaAzimuth;
    metadata.m_antennaElevation = m_settings.m_antennaElevation;
    metadata.m_antennaBeamwidth = m_settings.m_antennaBeamwidth;

    return RMOBReport::save(fileName, reportMonthDate, data, metadata, error);
}

void MeteorGUI::on_clearDetections_clicked()
{
    for (const QDate& date : m_hourlyData.keys()) {
        markRMOBDirty(date);
    }

    for (const QDate& date : m_hourlyCounts.keys()) {
        markRMOBDirty(date);
    }

    m_totalCount = 0;
    m_hourlyCounts.clear();
    m_hourlyData.clear();
    m_detectionOverlays.clear();
    m_nextDetectionOverlayId = 1;
    invalidateDetectionOverlayWindows();
    m_detectionsTable->setRowCount(0);
    hideDetectionOverlayLabels();
    updateCounters();
    updateHistogram();
    updateColorgramme();
    updateSpectrumViews();
}

void MeteorGUI::on_detectionsTable_itemSelectionChanged()
{
    const QList<QTableWidgetItem*> selectedItems = m_detectionsTable->selectedItems();

    if (selectedItems.isEmpty())
    {
        updateSpectrumViews();
        return;
    }

    QTableWidgetItem *timeItem = m_detectionsTable->item(selectedItems.first()->row(), 0);

    if (!timeItem) {
        return;
    }

    bool ok = false;
    qint64 utcMSecs = timeItem->data(DetectionDisplayUtcMSecsRole).toLongLong(&ok);

    if (!ok) {
        utcMSecs = timeItem->data(DetectionSampleUtcMSecsRole).toLongLong(&ok);
    }

    if (!ok) {
        return;
    }

    scrollSpectrumViewsToUTC(QDateTime::fromMSecsSinceEpoch(utcMSecs, Qt::UTC));
    updateSpectrumViews();
}

void MeteorGUI::on_detectionsTable_customContextMenuRequested(const QPoint& pos)
{
    QTableWidgetItem *item = m_detectionsTable->itemAt(pos);

    if (item && !item->isSelected())
    {
        m_detectionsTable->clearSelection();
        m_detectionsTable->selectRow(item->row());
    }

    if (!m_detectionsTable->selectionModel()
        || m_detectionsTable->selectionModel()->selectedRows().isEmpty())
    {
        return;
    }

    QMenu menu(this);
    QAction *deleteAction = menu.addAction("Delete selected detections");

    if (menu.exec(m_detectionsTable->viewport()->mapToGlobal(pos)) == deleteAction) {
        deleteSelectedDetections();
    }
}

void MeteorGUI::on_detectionsTableHeader_customContextMenuRequested(const QPoint& pos)
{
    QHeaderView *header = m_detectionsTable->horizontalHeader();
    QMenu menu(this);

    for (int visualIndex = 0; visualIndex < header->count(); visualIndex++)
    {
        const int logicalIndex = header->logicalIndex(visualIndex);
        QAction *action = menu.addAction(m_detectionsTable->horizontalHeaderItem(logicalIndex)->text());
        action->setCheckable(true);
        action->setChecked(!m_detectionsTable->isColumnHidden(logicalIndex));
        action->setData(logicalIndex);
    }

    QAction *selectedAction = menu.exec(header->viewport()->mapToGlobal(pos));

    if (!selectedAction) {
        return;
    }

    const int logicalIndex = selectedAction->data().toInt();
    const bool hideColumn = !selectedAction->isChecked();
    int visibleColumns = 0;

    for (int i = 0; i < header->count(); i++)
    {
        if (!m_detectionsTable->isColumnHidden(i)) {
            visibleColumns++;
        }
    }

    if (hideColumn && (visibleColumns <= 1)) {
        return;
    }

    m_detectionsTable->setColumnHidden(logicalIndex, hideColumn);
    saveDetectionsColumnVisibility();
}

void MeteorGUI::onWidgetRolled(QWidget* widget, bool rollDown)
{
    (void) widget;
    (void) rollDown;

    getRollupContents()->saveState(m_rollupState);
    applySetting("rollupState");
}

void MeteorGUI::onMenuDialogCalled(const QPoint &p)
{
    if (m_contextMenuType == ContextMenuType::ContextMenuChannelSettings)
    {
        BasicChannelSettingsDialog dialog(&m_channelMarker, this);
        dialog.setDefaultTitle(m_displayedName);

        if (m_deviceUISet->m_deviceMIMOEngine)
        {
            dialog.setNumberOfStreams(m_meteor->getNumberOfDeviceStreams());
            dialog.setStreamIndex(m_settings.m_streamIndex);
        }

        dialog.move(p);
        new DialogPositioner(&dialog, true);
        dialog.exec();

        m_settings.m_rgbColor = m_channelMarker.getColor().rgb();
        m_settings.m_title = m_channelMarker.getTitle();
        setWindowTitle(m_settings.m_title);
        setTitle(m_channelMarker.getTitle());
        setTitleColor(m_settings.m_rgbColor);

        QStringList settingsKeys({"rgbColor", "title"});

        if (m_deviceUISet->m_deviceMIMOEngine)
        {
            m_settings.m_streamIndex = dialog.getSelectedStreamIndex();
            m_channelMarker.clearStreamIndexes();
            m_channelMarker.addStreamIndex(m_settings.m_streamIndex);
            updateIndexLabel();
            settingsKeys.append("streamIndex");
        }

        applySettings(settingsKeys);
    }

    resetContextMenuType();
}

void MeteorGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void MeteorGUI::applySetting(const QString& settingsKey)
{
    applySettings({settingsKey});
}

void MeteorGUI::applySettings(const QStringList& settingsKeys, bool force)
{
    if (!m_doApplySettings) {
        return;
    }

    m_settingsKeys.append(settingsKeys);
    Meteor::MsgConfigureMeteor* message = Meteor::MsgConfigureMeteor::create(m_settings, m_settingsKeys, force);
    m_meteor->getInputMessageQueue()->push(message);
    m_settingsKeys.clear();
}

void MeteorGUI::applyAllSettings()
{
    applySettings(QStringList(), true);
}

void MeteorGUI::displaySettings()
{
    m_channelMarker.blockSignals(true);
    m_channelMarker.setBandwidth(m_settings.m_channelSampleRate);
    m_channelMarker.setCenterFrequency(m_settings.m_inputFrequencyOffset);
    m_channelMarker.setTitle(m_settings.m_title);
    m_channelMarker.blockSignals(false);
    m_channelMarker.setColor(m_settings.m_rgbColor);

    setTitleColor(m_settings.m_rgbColor);
    setWindowTitle(m_channelMarker.getTitle());
    setTitle(m_channelMarker.getTitle());

    blockApplySettings(true);
    const QSignalBlocker frequencyModeBlocker(m_frequencyMode);
    const QSignalBlocker sampleRateBlocker(m_sampleRate);
    const QSignalBlocker powerLPFCutoffBlocker(m_powerLPFCutoff);
    const QSignalBlocker detectionThresholdBlocker(m_detectionThreshold);
    const QSignalBlocker minDurationBlocker(m_minDuration);
    const QSignalBlocker maxDurationBlocker(m_maxDuration);
    const QSignalBlocker maxFrequencyDriftBlocker(m_maxFrequencyDrift);
    const QSignalBlocker detectionBoxPaddingBlocker(m_detectionBoxPadding);
    const QSignalBlocker detectionLabelsBlocker(m_detectionLabels);
    const QSignalBlocker transmitterLatitudeBlocker(m_transmitterLatitude);
    const QSignalBlocker transmitterLongitudeBlocker(m_transmitterLongitude);
    const QSignalBlocker antennaAzimuthBlocker(m_antennaAzimuth);
    const QSignalBlocker antennaElevationBlocker(m_antennaElevation);
    const QSignalBlocker antennaBeamwidthBlocker(m_antennaBeamwidth);

    m_frequencyMode->setCurrentIndex((int) m_settings.m_frequencyMode);
    on_frequencyMode_currentIndexChanged((int) m_settings.m_frequencyMode);

    const int rateIndex = sampleRateIndex(m_settings.m_channelSampleRate);
    m_sampleRate->setCurrentIndex(rateIndex);
    m_powerLPFCutoff->setMaximum(std::max(0.1, m_settings.m_channelSampleRate * 0.45));
    m_powerLPFCutoff->setValue(m_settings.m_powerLPFCutoff);
    m_detectionThreshold->setValue(m_settings.m_detectionThresholdDB);
    m_minDuration->setValue(m_settings.m_minDurationMS);
    m_maxDuration->setValue(m_settings.m_maxDurationMS);
    m_maxFrequencyDrift->setValue(m_settings.m_maxFrequencyDrift);
    m_detectionBoxPadding->setValue(m_settings.m_detectionBoxPaddingPixels);
    m_detectionLabels->setCurrentIndex(std::clamp(
        (int) m_settings.m_detectionLabelMode,
        (int) MeteorSettings::DetectionLabelNone,
        (int) MeteorSettings::DetectionLabelRight));
    m_transmitterLatitude->setValue(m_settings.m_transmitterLatitude);
    m_transmitterLongitude->setValue(m_settings.m_transmitterLongitude);
    m_antennaAzimuth->setValue(m_settings.m_antennaAzimuth);
    m_antennaElevation->setValue(m_settings.m_antennaElevation);
    m_antennaBeamwidth->setValue(m_settings.m_antennaBeamwidth);
    populateRotatorCombo();
    applyDetectionsColumnVisibility();

    updateVisualSampleRate();
    updateIndexLabel();
    getRollupContents()->restoreState(m_rollupState);
    updateAbsoluteCenterFrequency();

    blockApplySettings(false);
}

void MeteorGUI::makeUIConnections()
{
    QObject::connect(m_frequencyMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MeteorGUI::on_frequencyMode_currentIndexChanged);
    QObject::connect(m_deltaFrequency, &ValueDialZ::changed, this, &MeteorGUI::on_deltaFrequency_changed);
    QObject::connect(m_sampleRate, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MeteorGUI::on_sampleRate_currentIndexChanged);
    QObject::connect(m_powerLPFCutoff, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_powerLPFCutoff_valueChanged);
    QObject::connect(m_detectionThreshold, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_detectionThreshold_valueChanged);
    QObject::connect(m_minDuration, QOverload<int>::of(&QSpinBox::valueChanged), this, &MeteorGUI::on_minDuration_valueChanged);
    QObject::connect(m_maxDuration, QOverload<int>::of(&QSpinBox::valueChanged), this, &MeteorGUI::on_maxDuration_valueChanged);
    QObject::connect(m_maxFrequencyDrift, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_maxFrequencyDrift_valueChanged);
    QObject::connect(m_highlightAllDetections, &ButtonSwitch::toggled, this, &MeteorGUI::on_highlightAllDetections_toggled);
    QObject::connect(m_detectionBoxPadding, QOverload<int>::of(&QSpinBox::valueChanged), this, &MeteorGUI::on_detectionBoxPadding_valueChanged);
    QObject::connect(m_detectionLabels, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MeteorGUI::on_detectionLabels_currentIndexChanged);
    QObject::connect(m_transmitterLatitude, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_transmitterLatitude_valueChanged);
    QObject::connect(m_transmitterLongitude, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_transmitterLongitude_valueChanged);
    QObject::connect(m_antennaAzimuth, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_antennaAzimuth_valueChanged);
    QObject::connect(m_antennaElevation, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_antennaElevation_valueChanged);
    QObject::connect(m_antennaBeamwidth, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MeteorGUI::on_antennaBeamwidth_valueChanged);
    QObject::connect(m_rotator, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MeteorGUI::on_rotator_currentIndexChanged);
    QObject::connect(m_saveDetections, &QPushButton::clicked, this, &MeteorGUI::on_saveDetections_clicked);
    QObject::connect(m_saveColorgramme, &QPushButton::clicked, this, &MeteorGUI::on_saveColorgramme_clicked);
    QObject::connect(m_clearDetections, &QPushButton::clicked, this, &MeteorGUI::on_clearDetections_clicked);
    QObject::connect(m_detectionsTable, &QTableWidget::itemSelectionChanged, this, &MeteorGUI::on_detectionsTable_itemSelectionChanged);
    QObject::connect(m_detectionsTable, &QTableWidget::customContextMenuRequested, this, &MeteorGUI::on_detectionsTable_customContextMenuRequested);
    QObject::connect(m_detectionsTable->horizontalHeader(), &QHeaderView::customContextMenuRequested, this, &MeteorGUI::on_detectionsTableHeader_customContextMenuRequested);
    QObject::connect(m_detectionsTable->horizontalHeader(), &QHeaderView::sortIndicatorChanged, this, [this](int, Qt::SortOrder) {
        updateSpectrumViews();
    });
}

void MeteorGUI::calcOffset()
{
    if (m_settings.m_frequencyMode == MeteorSettings::Offset)
    {
        m_deltaFrequency->setValueRange(false, 7, -m_basebandSampleRate/2, m_basebandSampleRate/2);
    }
    else
    {
        qint64 offset = m_settings.m_frequency - m_deviceCenterFrequency;
        m_channelMarker.setCenterFrequency(offset);
        m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
        updateAbsoluteCenterFrequency();
        applySetting("inputFrequencyOffset");
    }
}

void MeteorGUI::updateAbsoluteCenterFrequency()
{
    setStatusFrequency(m_settings.m_frequency);

    if (   (m_basebandSampleRate > 1)
        && (   (m_settings.m_inputFrequencyOffset >= m_basebandSampleRate / 2)
            || (m_settings.m_inputFrequencyOffset < -m_basebandSampleRate / 2))) {
        setStatusText("Frequency out of band");
    } else {
        setStatusText("");
    }
}

void MeteorGUI::updateVisualSampleRate()
{
    m_glSpectrum->setSampleRate(m_settings.m_channelSampleRate);
    m_headGLSpectrum->setSampleRate(m_settings.m_channelSampleRate);

    DSPSignalNotification *msg = new DSPSignalNotification(m_settings.m_channelSampleRate, 0);
    m_spectrumVis->getInputMessageQueue()->push(msg);
    msg = new DSPSignalNotification(m_settings.m_channelSampleRate, 0);
    m_headSpectrumVis->getInputMessageQueue()->push(msg);
}

void MeteorGUI::addDetection(const MeteorDemodSink::MsgMeteorDetected& detection)
{
    const QDateTime sampleTimeUtc = detection.getDateTimeUtc();
    const QDateTime displayTimeUtc = detection.getDisplayDateTimeUtc().isValid()
        ? detection.getDisplayDateTimeUtc()
        : sampleTimeUtc;
    const QDateTime displayLocalTime = displayTimeUtc.toLocalTime();
    const QDate date = displayLocalTime.date();
    const int hour = displayLocalTime.time().hour();

    if (!m_hourlyCounts.contains(date)) {
        m_hourlyCounts[date] = QVector<int>(24, 0);
    }

    markHourData(date, hour);
    m_hourlyCounts[date][hour]++;
    m_totalCount++;
    markRMOBDirty(date);

    const quint64 displayStartSample = detection.getDisplayStartSample();
    const quint64 displayEndSample = detection.getDisplayEndSample();
    const quint64 displaySampleCount = displayEndSample >= displayStartSample
        ? displayEndSample - displayStartSample + 1
        : 1;
    const double displaySampleDurationS = (double) displaySampleCount
        / (double) std::max(1, detection.getSampleRate());
    const double displayTimeScale = std::clamp(
        detection.getDisplayDurationS() / std::max(1e-9, displaySampleDurationS),
        1e-3,
        1e3);
    const quint64 overlayId = m_nextDetectionOverlayId++;
    const DetectionOverlay overlay = {
        overlayId,
        displayTimeUtc,
        detection.getDisplayDurationS(),
        detection.getCenterFrequency(),
        detection.getFrequencySpan(),
        detection.getFrequencyDrift(),
        detection.getPeakPowerDB(),
        displayTimeScale,
        nullptr
    };
    const qint64 overlayStartMSecs = overlay.m_startTimeUtc.toMSecsSinceEpoch();
    const auto insertIt = std::upper_bound(
        m_detectionOverlays.cbegin(),
        m_detectionOverlays.cend(),
        overlayStartMSecs,
        [](qint64 targetMSecs, const DetectionOverlay& detection) {
            return targetMSecs < detection.m_startTimeUtc.toMSecsSinceEpoch();
        });
    m_detectionOverlays.insert((int) std::distance(m_detectionOverlays.cbegin(), insertIt), overlay);

    invalidateDetectionOverlayWindows();

    const bool sortingEnabled = m_detectionsTable->isSortingEnabled();
    m_detectionsTable->setSortingEnabled(false);
    const int row = m_detectionsTable->rowCount();
    m_detectionsTable->insertRow(row);
    QTableWidgetItem *timeItem = makeTableItem(displayLocalTime.toString("yyyy-MM-dd HH:mm:ss.zzz"), displayLocalTime.toMSecsSinceEpoch());
    timeItem->setData(DetectionSampleUtcMSecsRole, sampleTimeUtc.toMSecsSinceEpoch());
    timeItem->setData(DetectionOverlayIdRole, QVariant::fromValue<qulonglong>(overlayId));
    timeItem->setData(DetectionDisplayUtcMSecsRole, displayTimeUtc.toMSecsSinceEpoch());
    timeItem->setData(DetectionRadioRole, true);
    if (detection.getTruncated()) {
        timeItem->setToolTip(tr("Detection duration was clipped to the configured maximum."));
    }
    m_detectionsTable->setItem(row, 0, timeItem);
    for (DetectionOverlay& detectionOverlay : m_detectionOverlays)
    {
        if (detectionOverlay.m_id == overlayId)
        {
            detectionOverlay.m_tableItem = timeItem;
            break;
        }
    }
    m_detectionsTable->setItem(row, 1, makeTableItem(QString::number(detection.getPeakPowerDB(), 'f', 1), detection.getPeakPowerDB()));
    m_detectionsTable->setItem(row, 2, makeTableItem(QString::number(detection.getBackgroundPowerDB(), 'f', 1), detection.getBackgroundPowerDB()));
    m_detectionsTable->setItem(row, 3, makeTableItem(QString::number(detection.getTotalPowerDB(), 'f', 1), detection.getTotalPowerDB()));
    m_detectionsTable->setItem(row, 4, makeTableItem(QString::number(detection.getPeakAmplitude(), 'f', 4), detection.getPeakAmplitude()));
    QTableWidgetItem *durationItem = makeTableItem(QString::number(detection.getDurationS() * 1000.0, 'f', 1), detection.getDurationS());
    if (detection.getTruncated()) {
        durationItem->setToolTip(tr("Detection duration was clipped to the configured maximum."));
    }
    m_detectionsTable->setItem(row, 5, durationItem);
    m_detectionsTable->setItem(row, 6, makeTableItem(QString::number(detection.getCenterFrequency(), 'f', 1), detection.getCenterFrequency()));
    m_detectionsTable->setItem(row, 7, makeTableItem(QString::number(detection.getFrequencySpan(), 'f', 1), detection.getFrequencySpan()));
    m_detectionsTable->setItem(row, 8, makeTableItem(QString::number(detection.getFrequencyDrift(), 'f', 1), detection.getFrequencyDrift()));
    m_detectionsTable->setItem(row, 9, makeTableItem(QString::number(detection.getSampleRate()), detection.getSampleRate()));
    m_detectionsTable->setItem(row, 10, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 11, makeTableItem(QString()));
    m_detectionsTable->setSortingEnabled(sortingEnabled);

    updateCounters();
    updateHistogram();
    updateColorgramme();
    updateSpectrumViews();
}

void MeteorGUI::addCameraDetection(const Meteor::MsgCameraMeteorDetected& detection)
{
    const QDateTime utcTime = detection.getDateTimeUtc().toUTC();
    const QDateTime localTime = utcTime.toLocalTime();
    m_totalCount++;

    const bool sortingEnabled = m_detectionsTable->isSortingEnabled();
    m_detectionsTable->setSortingEnabled(false);
    const int row = m_detectionsTable->rowCount();
    m_detectionsTable->insertRow(row);

    QTableWidgetItem *timeItem = makeTableItem(localTime.toString("yyyy-MM-dd HH:mm:ss.zzz"), localTime.toMSecsSinceEpoch());
    timeItem->setData(DetectionSampleUtcMSecsRole, utcTime.toMSecsSinceEpoch());
    timeItem->setData(DetectionDisplayUtcMSecsRole, utcTime.toMSecsSinceEpoch());
    timeItem->setData(DetectionRadioRole, false);
    m_detectionsTable->setItem(row, 0, timeItem);
    m_detectionsTable->setItem(row, 1, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 2, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 3, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 4, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 5, makeTableItem(QString::number(detection.getDurationS() * 1000.0, 'f', 1), detection.getDurationS()));
    m_detectionsTable->setItem(row, 6, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 7, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 8, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 9, makeTableItem(QString()));
    m_detectionsTable->setItem(row, 10, detection.hasMagnitude()
        ? makeTableItem(QString::number(detection.getMagnitude(), 'f', 2), detection.getMagnitude())
        : makeTableItem(QString()));
    m_detectionsTable->setItem(row, 11, detection.hasFlux()
        ? makeTableItem(QString::number(detection.getFlux(), 'g', 6), detection.getFlux())
        : makeTableItem(QString()));
    m_detectionsTable->setSortingEnabled(sortingEnabled);

    updateCounters();
    updateHistogram();
    updateColorgramme();
    updateSpectrumViews();
}

void MeteorGUI::updateCounters()
{
    m_totalCountText->setText(QString::number(m_totalCount));

    const QDateTime now = QDateTime::currentDateTime();
    int hourCount = 0;

    if (m_hourlyCounts.contains(now.date())) {
        hourCount = m_hourlyCounts[now.date()][now.time().hour()];
    }

    m_hourCountText->setText(QString::number(hourCount));
}

void MeteorGUI::updateHistogram()
{
    QChart *oldChart = m_hourlyChart;
    m_hourlyChart = new QChart();
    m_hourlyChart->layout()->setContentsMargins(0, 0, 0, 0);
    m_hourlyChart->setMargins(QMargins(1, 1, 1, 1));
    m_hourlyChart->setTheme(QChart::ChartThemeDark);
    m_hourlyChart->legend()->setAlignment(Qt::AlignBottom);
    m_hourlyChart->legend()->setVisible(true);

    QBarSeries *series = new QBarSeries();
    int maxCount = 1;

    constexpr int histogramDayCount = 7;
    QList<QDate> dataDates;

    for (auto it = m_hourlyCounts.cbegin(); it != m_hourlyCounts.cend(); ++it)
    {
        bool dateHasData = false;

        for (int hour = 0; (hour < 24) && !dateHasData; hour++) {
            dateHasData = hasHourData(it.key(), hour);
        }

        if (dateHasData) {
            dataDates.append(it.key());
        }
    }

    const qsizetype firstDay = std::max<qsizetype>(0, dataDates.size() - histogramDayCount);

    for (qsizetype day = firstDay; day < dataDates.size(); day++)
    {
        const QDate date = dataDates[day];
        QBarSet *set = new QBarSet(date.toString(Qt::ISODate));

        for (int hour = 0; hour < 24; hour++)
        {
            const int count = m_hourlyCounts.value(date).value(hour);
            *set << count;
            maxCount = std::max(maxCount, count);
        }

        series->append(set);
    }

    if (series->count() == 0)
    {
        QBarSet *set = new QBarSet(QDate::currentDate().toString(Qt::ISODate));
        for (int hour = 0; hour < 24; hour++) {
            *set << 0;
        }
        series->append(set);
    }

    QStringList categories;
    for (int hour = 0; hour < 24; hour++) {
        categories << QString::number(hour);
    }

    QBarCategoryAxis *axisX = new QBarCategoryAxis();
    axisX->append(categories);
    axisX->setTitleText("Hour");

    QValueAxis *axisY = new QValueAxis();
    axisY->setRange(0, maxCount);
    axisY->setLabelFormat("%d");
    axisY->setTitleText("Meteors");

    m_hourlyChart->addSeries(series);
    m_hourlyChart->addAxis(axisX, Qt::AlignBottom);
    m_hourlyChart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisX);
    series->attachAxis(axisY);
    m_hourlyChartView->setChart(m_hourlyChart);

    delete oldChart;
}

void MeteorGUI::updateColorgramme()
{
    if (!m_colorgrammeTable) {
        return;
    }

    QDate monthDate = colorgrammeMonthDate();

    const int year = monthDate.year();
    const int month = monthDate.month();
    const int daysInMonth = monthDate.daysInMonth();
    int maxCount = 0;

    m_colorgrammeTable->setColumnCount(daysInMonth);

    for (int day = 1; day <= daysInMonth; day++)
    {
        QTableWidgetItem *dayItem = new QTableWidgetItem(QString::number(day));
        dayItem->setTextAlignment(Qt::AlignCenter);
        m_colorgrammeTable->setHorizontalHeaderItem(day - 1, dayItem);

        const QDate date(year, month, day);

        if (!m_hourlyData.contains(date)) {
            continue;
        }

        for (int hour = 0; hour < 24; hour++) {
            if (hasHourData(date, hour)) {
                maxCount = std::max(maxCount, m_hourlyCounts[date].value(hour));
            }
        }
    }

    for (int hour = 0; hour < 24; hour++)
    {
        for (int day = 1; day <= daysInMonth; day++)
        {
            const QDate date(year, month, day);
            const bool hasData = hasHourData(date, hour);
            const int count = hasData ? m_hourlyCounts[date].value(hour) : 0;
            QTableWidgetItem *item = m_colorgrammeTable->item(hour, day - 1);

            if (!item)
            {
                item = new QTableWidgetItem();
                item->setTextAlignment(Qt::AlignCenter);
                m_colorgrammeTable->setItem(hour, day - 1, item);
            }

            item->setText(hasData && (count > 0) ? QString::number(count) : QString());
            item->setToolTip(hasData
                ? QString("%1 %2:00-%2:59: %3 meteor%4")
                    .arg(date.toString(Qt::ISODate))
                    .arg(hour, 2, 10, QLatin1Char('0'))
                    .arg(count)
                    .arg(count == 1 ? "" : "s")
                : QString("%1 %2:00-%2:59: no data")
                    .arg(date.toString(Qt::ISODate))
                    .arg(hour, 2, 10, QLatin1Char('0')));

            const QColor color = hasData ? colorgrammeColor(count, maxCount) : QColor(Qt::black);
            item->setBackground(color);
            item->setForeground(color.lightness() < 110 ? Qt::white : Qt::black);
        }
    }
}

QColor MeteorGUI::colorgrammeColor(int count, int maxCount) const
{
    if ((count <= 0) || (maxCount <= 0)) {
        return QColor(0, 0, 170);
    }

    const double position = std::clamp((double) count / (double) maxCount, 0.0, 1.0);
    const QVector<QColor> stops = {
        QColor(0, 0, 170),
        QColor(0, 220, 255),
        QColor(0, 190, 0),
        QColor(255, 230, 0),
        QColor(220, 0, 0)
    };
    const double scaled = position * (double) (stops.size() - 1);
    const int index = std::min((int) std::floor(scaled), (int) stops.size() - 2);
    const double t = scaled - (double) index;
    const QColor a = stops[index];
    const QColor b = stops[index + 1];

    return QColor(
        (int) std::round((1.0 - t) * a.red() + t * b.red()),
        (int) std::round((1.0 - t) * a.green() + t * b.green()),
        (int) std::round((1.0 - t) * a.blue() + t * b.blue())
    );
}

bool MeteorGUI::markHourData(const QDate& date, int hour)
{
    if (!date.isValid() || (hour < 0) || (hour >= 24)) {
        return false;
    }

    if (!m_hourlyCounts.contains(date)) {
        m_hourlyCounts[date] = QVector<int>(24, 0);
    }

    if (!m_hourlyData.contains(date)) {
        m_hourlyData[date] = QVector<bool>(24, false);
    }

    const bool hadData = m_hourlyData[date][hour];
    m_hourlyData[date][hour] = true;

    if (!hadData) {
        markRMOBDirty(date);
    }

    return !hadData;
}

void MeteorGUI::markRMOBDirty(const QDate& date)
{
    if (date.isValid()) {
        m_dirtyRMOBMonths.insert(QDate(date.year(), date.month(), 1));
    }
}

bool MeteorGUI::hasHourData(const QDate& date, int hour) const
{
    return (hour >= 0)
        && (hour < 24)
        && m_hourlyData.contains(date)
        && m_hourlyData[date].value(hour);
}

QString MeteorGUI::automaticRMOBReportFileName(const QDate& date) const
{
    QString appDataPath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

    if (appDataPath.isEmpty()) {
        appDataPath = QDir::homePath();
    }

    QDir dir(appDataPath);
    dir.mkpath("meteor");
    dir.cd("meteor");

    return dir.filePath(QString("meteor_%1_%2.rmob.txt")
        .arg(date.year())
        .arg(date.month(), 2, 10, QLatin1Char('0')));
}

bool MeteorGUI::saveAutomaticRMOBReports() const
{
    QSet<QDate> months = m_dirtyRMOBMonths;

    for (const QDate& date : colorgrammeMonthDates()) {
        months.insert(QDate(date.year(), date.month(), 1));
    }

    bool success = true;

    for (const QDate& monthDate : months) {
        success = saveRMOBReport(automaticRMOBReportFileName(monthDate), monthDate) && success;
    }

    return success;
}

QDate MeteorGUI::colorgrammeMonthDate() const
{
    const QList<QDate> months = colorgrammeMonthDates();

    if (!months.isEmpty()) {
        return months.last();
    }

    return QDate::currentDate();
}

QList<QDate> MeteorGUI::colorgrammeMonthDates() const
{
    QList<QDate> months;

    auto addMonth = [&months](const QDate& date)
    {
        if (!date.isValid()) {
            return;
        }

        const QDate monthDate(date.year(), date.month(), 1);

        if (!months.contains(monthDate)) {
            months.append(monthDate);
        }
    };

    for (auto it = m_hourlyData.cbegin(); it != m_hourlyData.cend(); ++it) {
        addMonth(it.key());
    }

    for (auto it = m_hourlyCounts.cbegin(); it != m_hourlyCounts.cend(); ++it) {
        addMonth(it.key());
    }

    if (months.isEmpty()) {
        addMonth(QDate::currentDate());
    }

    std::sort(months.begin(), months.end());
    return months;
}

qint64 MeteorGUI::rmobReportFrequency() const
{
    double centerFrequency = 0.0;

    if (m_deviceUISet
        && m_deviceUISet->m_deviceAPI
        && ChannelWebAPIUtils::getCenterFrequency((unsigned int) m_deviceUISet->m_deviceAPI->getDeviceSetIndex(), centerFrequency))
    {
        return (qint64) std::llround(centerFrequency) + (qint64) m_settings.m_inputFrequencyOffset;
    }

    if (m_deviceCenterFrequency > 0) {
        return m_deviceCenterFrequency + (qint64) m_settings.m_inputFrequencyOffset;
    }

    return m_settings.m_frequency;
}

QString MeteorGUI::rmobReceiverName() const
{
    if (!m_deviceUISet || !m_deviceUISet->m_deviceAPI) {
        return QString();
    }

    const DeviceAPI *deviceAPI = m_deviceUISet->m_deviceAPI;

    if (!deviceAPI->getSamplingDeviceDisplayName().isEmpty()) {
        return deviceAPI->getSamplingDeviceDisplayName();
    }

    if (!deviceAPI->getSamplingDeviceId().isEmpty()) {
        return deviceAPI->getSamplingDeviceId();
    }

    return deviceAPI->getHardwareId();
}

void MeteorGUI::applyDetectionsColumnVisibility()
{
    if (!m_detectionsTable) {
        return;
    }

    int visibleColumns = 0;

    for (int i = 0; i < m_detectionsTable->columnCount(); i++)
    {
        const bool hidden = (m_settings.m_detectionsTableColumnHidden & (1u << i)) != 0;
        m_detectionsTable->setColumnHidden(i, hidden);

        if (!hidden) {
            visibleColumns++;
        }
    }

    if ((visibleColumns == 0) && (m_detectionsTable->columnCount() > 0))
    {
        m_detectionsTable->setColumnHidden(0, false);
        saveDetectionsColumnVisibility();
    }
}

void MeteorGUI::saveDetectionsColumnVisibility()
{
    quint32 hiddenColumns = 0;

    for (int i = 0; i < m_detectionsTable->columnCount(); i++)
    {
        if (m_detectionsTable->isColumnHidden(i)) {
            hiddenColumns |= (1u << i);
        }
    }

    if (hiddenColumns != m_settings.m_detectionsTableColumnHidden)
    {
        m_settings.m_detectionsTableColumnHidden = hiddenColumns;
        applySetting("detectionsTableColumnHidden");
    }
}

QSet<quint64> MeteorGUI::selectedDetectionOverlayIds() const
{
    QSet<quint64> ids;

    if (!m_detectionsTable->selectionModel()) {
        return ids;
    }

    const QModelIndexList rows = m_detectionsTable->selectionModel()->selectedRows();

    for (const QModelIndex& rowIndex : rows)
    {
        QTableWidgetItem *timeItem = m_detectionsTable->item(rowIndex.row(), 0);

        if (timeItem)
        {
            bool ok = false;
            const quint64 id = timeItem->data(DetectionOverlayIdRole).toULongLong(&ok);

            if (ok) {
                ids.insert(id);
            }
        }
    }

    return ids;
}

void MeteorGUI::deleteSelectedDetections()
{
    if (!m_detectionsTable->selectionModel()) {
        return;
    }

    QSet<quint64> idsToDelete;
    QVector<int> rowsToDelete;
    const QModelIndexList rows = m_detectionsTable->selectionModel()->selectedRows();

    for (const QModelIndex& rowIndex : rows)
    {
        QTableWidgetItem *timeItem = m_detectionsTable->item(rowIndex.row(), 0);

        if (!timeItem) {
            continue;
        }

        bool idOK = false;
        const quint64 id = timeItem->data(DetectionOverlayIdRole).toULongLong(&idOK);

        if (idOK) {
            idsToDelete.insert(id);
        }

        bool timeOK = false;
        const bool radioDetection = timeItem->data(DetectionRadioRole).toBool();
        const qint64 utcMSecs = timeItem->data(DetectionDisplayUtcMSecsRole).toLongLong(&timeOK);

        if (timeOK && radioDetection)
        {
            const QDateTime localTime = QDateTime::fromMSecsSinceEpoch(utcMSecs, Qt::UTC).toLocalTime();
            const QDate date = localTime.date();
            const int hour = localTime.time().hour();

            if (m_hourlyCounts.contains(date) && (hour >= 0) && (hour < m_hourlyCounts[date].size()) && (m_hourlyCounts[date][hour] > 0)) {
                m_hourlyCounts[date][hour]--;
            }

            markRMOBDirty(date);

        }

        if (m_totalCount > 0) {
            m_totalCount--;
        }

        rowsToDelete.push_back(rowIndex.row());
    }

    if (!idsToDelete.isEmpty())
    {
        for (int i = m_detectionOverlays.size() - 1; i >= 0; i--)
        {
            if (idsToDelete.contains(m_detectionOverlays[i].m_id)) {
                m_detectionOverlays.removeAt(i);
            }
        }
        invalidateDetectionOverlayWindows();
    }

    std::sort(rowsToDelete.begin(), rowsToDelete.end(), std::greater<int>());

    for (int row : rowsToDelete) {
        m_detectionsTable->removeRow(row);
    }

    updateCounters();
    updateHistogram();
    updateColorgramme();
    updateSpectrumViews();
}

void MeteorGUI::drawDetectionOverlays(
    GLSpectrumView *spectrumView,
    SpectrumVis *spectrumVis,
    QVector<QLabel *>& overlayLabels,
    bool& overlayWindowValid,
    QDateTime& overlayWindowStartUtc,
    QDateTime& overlayWindowEndUtc)
{
    const QColor color(255, 190, 0);
    const QColor selectedColor(255, 225, 96);
    const QSet<quint64> selectedIds = selectedDetectionOverlayIds();
    const SpectrumSettings spectrumSettings = spectrumVis->getSettings();
    const double timePerPixel = spectrumView->waterfallTimePerPixel();
    const int fftSize = std::max(1, spectrumSettings.m_fftSize);
    const int fftOverlap = std::clamp(spectrumSettings.m_fftOverlap, 0, fftSize - 1);
    const int fftHopSize = fftSize - fftOverlap;
    int timingRate = 1;

    if ((spectrumSettings.m_averagingMode == SpectrumSettings::AvgModeFixed)
        || (spectrumSettings.m_averagingMode == SpectrumSettings::AvgModeMax))
    {
        timingRate = std::max(
            1,
            (int) SpectrumSettings::getAveragingValue(
                spectrumSettings.m_averagingIndex,
                spectrumSettings.m_averagingMode));
    }

    const double sampleWaterfallRowDurationS = m_settings.m_channelSampleRate > 0
        ? (double) (fftHopSize * timingRate) / (double) m_settings.m_channelSampleRate
        : 0.0;
    const double sampleSpectrumWindowCenterDelayS = m_settings.m_channelSampleRate > 0
        ? (double) (fftSize + (timingRate - 1) * fftHopSize)
            / (2.0 * (double) m_settings.m_channelSampleRate)
        : 0.0;
    struct LabelOverlay
    {
        const DetectionOverlay *m_detection;
        float m_xMin;
        float m_yStart;
        float m_xMax;
        float m_yEnd;
        bool m_selected;
    };
    QVector<LabelOverlay> labelOverlays;

    if (!m_highlightAllDetectionOverlays && selectedIds.isEmpty())
    {
        hideDetectionOverlayLabels(overlayLabels);
        overlayWindowValid = false;
        return;
    }

    QDateTime visibleStartUtc;
    QDateTime visibleEndUtc;

    if (!spectrumView->waterfallVisibleTimeRange(visibleStartUtc, visibleEndUtc))
    {
        hideDetectionOverlayLabels(overlayLabels);
        overlayWindowValid = false;
        return;
    }

    overlayWindowStartUtc = visibleStartUtc;
    overlayWindowEndUtc = visibleEndUtc;
    overlayWindowValid = true;

    auto scanOverlays = [&](int beginIndex, int endIndex)
    {
        for (int i = beginIndex; i < endIndex; i++)
        {
            const DetectionOverlay& detection = m_detectionOverlays[i];

            if (!m_highlightAllDetectionOverlays && !selectedIds.contains(detection.m_id)) {
                continue;
            }

            const bool selected = selectedIds.contains(detection.m_id);

            const qint64 durationMSecs = (qint64) std::ceil(detection.m_durationS * 1000.0);
            const QDateTime endTimeUtc = detection.m_startTimeUtc.addMSecs(durationMSecs);
            float yStart = 0.0f;
            float yEnd = 0.0f;

            if (fftOverlap > 0)
            {
                const double waterfallRowDurationS = sampleWaterfallRowDurationS
                    * detection.m_displayTimeScale;
                const double spectrumWindowCenterDelayS = sampleSpectrumWindowCenterDelayS
                    * detection.m_displayTimeScale;
                const qint64 centerDelayMSecs = (qint64) std::llround(spectrumWindowCenterDelayS * 1000.0);
                const QDateTime centerTimeUtc = detection.m_startTimeUtc.addMSecs(durationMSecs / 2 + centerDelayMSecs);
                float yCenter = 0.0f;

                if (!spectrumView->waterfallTimeToY(centerTimeUtc, yCenter)) {
                    continue;
                }

                if ((waterfallRowDurationS > 0.0) && (timePerPixel > 0.0))
                {
                    const double durationRows = detection.m_durationS / waterfallRowDurationS;
                    const float halfHeight = (float) (durationRows * timePerPixel * 0.5);
                    yStart = yCenter - halfHeight;
                    yEnd = yCenter + halfHeight;
                }
            }
            else if (!spectrumView->waterfallTimeToY(detection.m_startTimeUtc, yStart)
                || !spectrumView->waterfallTimeToY(endTimeUtc, yEnd))
            {
                continue;
            }

            const int paddingPixels = std::max(0, m_settings.m_detectionBoxPaddingPixels);
            const double minHalfWidthHz = std::max(2.0, (double) m_settings.m_channelSampleRate * 0.0025);
            const double paddingHz = paddingPixels * spectrumView->waterfallFrequencyPerPixel();
            const float paddingY = (float) (paddingPixels * spectrumView->waterfallTimePerPixel());
            const double halfBandwidth = std::max({
                std::fabs(detection.m_frequencySpan) * 0.5,
                std::fabs(detection.m_frequencyDrift) * 0.5,
                minHalfWidthHz
            }) + paddingHz;
            float xMin = 0.0f;
            float xMax = 0.0f;

            if (!spectrumView->waterfallFrequencyToX(detection.m_centerFrequency - halfBandwidth, xMin)
                || !spectrumView->waterfallFrequencyToX(detection.m_centerFrequency + halfBandwidth, xMax))
            {
                continue;
            }

            yStart = std::clamp(yStart - paddingY, 0.0f, 1.0f);
            yEnd = std::clamp(yEnd + paddingY, 0.0f, 1.0f);
            spectrumView->drawWaterfallOverlayBox(
                xMin,
                yStart,
                xMax,
                yEnd,
                selected ? selectedColor : color,
                1.0f);

            if (m_settings.m_detectionLabelMode != MeteorSettings::DetectionLabelNone) {
                labelOverlays.push_back({&detection, xMin, yStart, xMax, yEnd, selected});
            }

        }
    };

    if (!m_detectionOverlays.isEmpty())
    {
        const qint64 durationPaddingMSecs = std::max(1000, m_settings.m_maxDurationMS + 1000);
        const qint64 windowStartMSecs = visibleStartUtc.addMSecs(-durationPaddingMSecs).toMSecsSinceEpoch();
        const qint64 windowEndMSecs = visibleEndUtc.addMSecs(1000).toMSecsSinceEpoch();
        const auto beginIt = std::lower_bound(
            m_detectionOverlays.cbegin(),
            m_detectionOverlays.cend(),
            windowStartMSecs,
            [](const DetectionOverlay& detection, qint64 targetMSecs) {
                return detection.m_startTimeUtc.toMSecsSinceEpoch() < targetMSecs;
            });
        const auto endIt = std::upper_bound(
            m_detectionOverlays.cbegin(),
            m_detectionOverlays.cend(),
            windowEndMSecs,
            [](qint64 targetMSecs, const DetectionOverlay& detection) {
                return targetMSecs < detection.m_startTimeUtc.toMSecsSinceEpoch();
            });
        scanOverlays(
            (int) std::distance(m_detectionOverlays.cbegin(), beginIt),
            (int) std::distance(m_detectionOverlays.cbegin(), endIt));
    }

    if (labelOverlays.isEmpty()) {
        hideDetectionOverlayLabels(overlayLabels);
        return;
    }

    const double frequencyPerPixel = spectrumView->waterfallFrequencyPerPixel();
    if ((frequencyPerPixel <= 0.0) || (timePerPixel <= 0.0))
    {
        hideDetectionOverlayLabels(overlayLabels);
        return;
    }

    float onePixelFraction = 0.0f;

    for (const LabelOverlay& overlay : labelOverlays)
    {
        const double frequency = overlay.m_detection->m_centerFrequency;
        float x = 0.0f;
        float xPlus = 0.0f;
        float xMinus = 0.0f;

        if (spectrumView->waterfallFrequencyToX(frequency, x)
            && spectrumView->waterfallFrequencyToX(frequency + frequencyPerPixel, xPlus)
            && (xPlus > x))
        {
            onePixelFraction = xPlus - x;
            break;
        }

        if (spectrumView->waterfallFrequencyToX(frequency - frequencyPerPixel, xMinus)
            && spectrumView->waterfallFrequencyToX(frequency, x)
            && (x > xMinus))
        {
            onePixelFraction = x - xMinus;
            break;
        }
    }

    if (onePixelFraction <= 0.0f)
    {
        hideDetectionOverlayLabels(overlayLabels);
        return;
    }

    const QFontMetrics fontMetrics(spectrumView->font());
    const int rightMargin = fontMetrics.horizontalAdvance("000");
    const int waterfallWidth = std::max(1, (int) std::round(1.0 / onePixelFraction));
    const int waterfallHeight = std::max(1, (int) std::round(1.0 / timePerPixel));
    const int leftMargin = std::clamp(spectrumView->width() - rightMargin - waterfallWidth, 0, spectrumView->width());
    const int topMargin = fontMetrics.ascent() * 2;
    const int bottomMargin = fontMetrics.ascent();
    const int frequencyScaleHeight = fontMetrics.height() * 3;
    int waterfallTop = topMargin;

    if ((spectrumSettings.m_displayWaterfall || spectrumSettings.m_display3DSpectrogram)
        && (spectrumSettings.m_displayHistogram || spectrumSettings.m_displayMaxHold || spectrumSettings.m_displayCurrent))
    {
        if (spectrumSettings.m_invertedWaterfall)
        {
            const int histogramHeight = spectrumView->height() - topMargin - waterfallHeight - frequencyScaleHeight - bottomMargin;
            waterfallTop = topMargin + std::max(0, histogramHeight) + frequencyScaleHeight + 1;
        }
    }
    else if (!(spectrumSettings.m_displayWaterfall || spectrumSettings.m_display3DSpectrogram))
    {
        hideDetectionOverlayLabels(overlayLabels);
        return;
    }

    waterfallTop = std::clamp(waterfallTop, 0, std::max(0, spectrumView->height() - waterfallHeight));
    int labelIndex = 0;

    for (const LabelOverlay& overlay : labelOverlays)
    {
        const QString label = detectionOverlayLabel(*overlay.m_detection);

        if (label.isEmpty()) {
            continue;
        }

        const int textWidth = fontMetrics.horizontalAdvance(label);
        const int textHeight = fontMetrics.height();
        const int labelWidth = textWidth + 6;
        const int labelHeight = textHeight + 4;
        const int boxLeft = leftMargin + (int) std::round(std::min(overlay.m_xMin, overlay.m_xMax) * waterfallWidth);
        const int boxRight = leftMargin + (int) std::round(std::max(overlay.m_xMin, overlay.m_xMax) * waterfallWidth);
        const int boxTop = waterfallTop + (int) std::round(std::min(overlay.m_yStart, overlay.m_yEnd) * waterfallHeight);
        const int boxBottom = waterfallTop + (int) std::round(std::max(overlay.m_yStart, overlay.m_yEnd) * waterfallHeight);
        int labelX = boxLeft + (boxRight - boxLeft - labelWidth) / 2;
        int labelY = boxTop - labelHeight - 2;

        if (m_settings.m_detectionLabelMode == MeteorSettings::DetectionLabelRight)
        {
            labelX = boxRight + 3;
            labelY = boxTop + (boxBottom - boxTop - labelHeight) / 2;

            if (labelX + labelWidth > spectrumView->width()) {
                labelX = boxLeft - labelWidth - 3;
            }
        }
        else if (labelY < 0)
        {
            labelY = boxBottom + 2;
        }

        labelX = std::clamp(labelX, 0, std::max(0, spectrumView->width() - labelWidth));
        labelY = std::clamp(labelY, 0, std::max(0, spectrumView->height() - labelHeight));
        const QRect labelRect(labelX, labelY, labelWidth, labelHeight);

        if (labelIndex >= overlayLabels.size())
        {
            QLabel *labelWidget = new QLabel(spectrumView);
            labelWidget->setAttribute(Qt::WA_TransparentForMouseEvents, true);
            labelWidget->setAlignment(Qt::AlignCenter);
            overlayLabels.push_back(labelWidget);
        }

        QLabel *labelWidget = overlayLabels[labelIndex++];
        const QVariant selectedProperty = labelWidget->property("meteorSelected");

        if (!selectedProperty.isValid() || (selectedProperty.toBool() != overlay.m_selected))
        {
            labelWidget->setProperty("meteorSelected", overlay.m_selected);
            labelWidget->setStyleSheet(overlay.m_selected
                ? "QLabel { color: rgb(255, 225, 96); background-color: rgba(0, 0, 0, 190); padding: 1px 3px; }"
                : "QLabel { color: rgb(255, 190, 0); background-color: rgba(0, 0, 0, 190); padding: 1px 3px; }");
        }

        labelWidget->setText(label);
        labelWidget->setGeometry(labelRect);
        labelWidget->show();
        labelWidget->raise();
    }

    for (int i = labelIndex; i < overlayLabels.size(); i++) {
        overlayLabels[i]->hide();
    }
}

void MeteorGUI::hideDetectionOverlayLabels()
{
    hideDetectionOverlayLabels(m_detectionOverlayLabels);
    hideDetectionOverlayLabels(m_headDetectionOverlayLabels);
}

void MeteorGUI::hideDetectionOverlayLabels(QVector<QLabel *>& overlayLabels)
{
    for (QLabel *label : overlayLabels) {
        label->hide();
    }
}

void MeteorGUI::invalidateDetectionOverlayWindows()
{
    m_detectionOverlayWindowValid = false;
    m_headDetectionOverlayWindowValid = false;
}

void MeteorGUI::updateSpectrumViews()
{
    m_glSpectrum->getSpectrumView()->update();
    m_headGLSpectrum->getSpectrumView()->update();
}

void MeteorGUI::scrollSpectrumViewsToUTC(const QDateTime& dateTimeUtc)
{
    m_glSpectrum->scrollWaterfallToUTC(dateTimeUtc);
    m_headGLSpectrum->scrollWaterfallToUTC(dateTimeUtc);
    invalidateDetectionOverlayWindows();
}

QString MeteorGUI::detectionOverlayLabel(const DetectionOverlay& detection) const
{
    const QString duration = QString("%1 ms").arg(detection.m_durationS * 1000.0, 0, 'f', 0);
    const int tableRow = detection.m_tableItem ? m_detectionsTable->row(detection.m_tableItem) : -1;
    const QString prefix = tableRow >= 0 ? QString("#%1 ").arg(tableRow + 1) : QString();

    if (std::isfinite(detection.m_peakPowerDB)) {
        return prefix + QString("%1 dB  %2").arg(detection.m_peakPowerDB, 0, 'f', 1).arg(duration);
    }

    return prefix + duration;
}

int MeteorGUI::sampleRateIndex(int sampleRate) const
{
    for (int i = 0; i < (int) MeteorSettings::m_supportedSampleRates.size(); i++)
    {
        if (MeteorSettings::m_supportedSampleRates[i] == sampleRate) {
            return i;
        }
    }

    return 2;
}

QTableWidgetItem *MeteorGUI::makeTableItem(const QString& text, const QVariant& sortValue) const
{
    QTableWidgetItem *item = new NumericTableWidgetItem(text);
    item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);

    if (sortValue.isValid()) {
        item->setData(Qt::UserRole, sortValue);
    }

    return item;
}

void MeteorGUI::leaveEvent(QEvent* event)
{
    m_channelMarker.setHighlighted(false);
    ChannelGUI::leaveEvent(event);
}

void MeteorGUI::enterEvent(EnterEventType* event)
{
    m_channelMarker.setHighlighted(true);
    ChannelGUI::enterEvent(event);
}

void MeteorGUI::tick()
{
    if ((m_tickCount++ % 20) == 0)
    {
        if (!m_settings.m_rotator.isEmpty()) {
            syncFromSelectedRotator();
        }

        updateCounters();

        const QDateTime nowUtc = QDateTime::currentDateTimeUtc();

        if (!m_dirtyRMOBMonths.isEmpty()
            && (!m_lastAutomaticRMOBSaveUtc.isValid()
                || (m_lastAutomaticRMOBSaveUtc.secsTo(nowUtc) >= AutomaticRMOBSaveIntervalS)))
        {
            if (saveAutomaticRMOBReports()) {
                m_dirtyRMOBMonths.clear();
            }

            m_lastAutomaticRMOBSaveUtc = nowUtc;
        }
    }
}

void MeteorGUI::onFeatureAdded(int featureSetIndex, Feature *feature)
{
    (void) featureSetIndex;

    if (feature && (feature->getURI() == QLatin1String("sdrangel.feature.gs232controller"))) {
        populateRotatorCombo();
    }
}

void MeteorGUI::onFeatureRemoved(int featureSetIndex, Feature *feature)
{
    (void) featureSetIndex;

    if (feature && (feature->getURI() == QLatin1String("sdrangel.feature.gs232controller"))) {
        populateRotatorCombo();
    }
}
