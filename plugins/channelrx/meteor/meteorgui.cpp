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

#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>

#include "device/deviceuiset.h"
#include "dsp/dspcommands.h"
#include "dsp/glscopesettings.h"
#include "dsp/spectrumsettings.h"
#include "gui/basicchannelsettingsdialog.h"
#include "gui/dialogpositioner.h"
#include "gui/dialpopup.h"
#include "gui/glscope.h"
#include "gui/glscopegui.h"
#include "gui/glspectrum.h"
#include "gui/glspectrumgui.h"
#include "gui/rollupcontents.h"
#include "maincore.h"
#include "plugin/pluginapi.h"

#include "meteorgui.h"
#include "ui_meteorgui.h"

const int MeteorGUI::m_sampleRates[4] = {100, 300, 1000, 3000};

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
    m_scopeVis(nullptr),
    m_hourlyChart(nullptr),
    m_totalCount(0)
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
    m_scopeVis = m_meteor->getScopeVis();

    setupSpectrum();
    setupScope();

    connect(&MainCore::instance()->getMasterTimer(), SIGNAL(timeout()), this, SLOT(tick()));

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
    m_settings.setScopeGUI(m_scopeGUI);
    m_settings.setRollupState(&m_rollupState);

    m_deviceUISet->addChannelMarker(&m_channelMarker);

    connect(&m_channelMarker, SIGNAL(changedByCursor()), this, SLOT(channelMarkerChangedByCursor()));
    connect(&m_channelMarker, SIGNAL(highlightedByCursor()), this, SLOT(channelMarkerHighlightedByCursor()));
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    updateHistogram();
    displaySettings();
    makeUIConnections();
    applyAllSettings();
    DialPopup::addPopupsToChildDials(this);
    m_resizer.enableChildMouseTracking();
}

MeteorGUI::~MeteorGUI()
{
    disconnect(&MainCore::instance()->getMasterTimer(), SIGNAL(timeout()), this, SLOT(tick()));
    m_glScope->disconnectTimer();
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
    m_totalCountText = ui->totalCountText;
    m_hourCountText = ui->hourCountText;
    m_clearDetections = ui->clearDetections;
    m_detectionsTable = ui->detectionsTable;
    m_hourlyChartView = ui->hourlyChartView;
    m_glSpectrum = ui->glSpectrum;
    m_spectrumGUI = ui->spectrumGUI;
    m_glScope = ui->glScope;
    m_scopeGUI = ui->scopeGUI;

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

    m_clearDetections->setIcon(QIcon(":/bin.png"));
    m_clearDetections->setToolTip("Clear detections");
    m_clearDetections->setMaximumWidth(28);

    m_detectionsTable->setColumnCount(7);
    const QStringList detectionHeaders = {
        "Time (local)",
        "Peak (dB)",
        "Amp",
        "Duration (ms)",
        "Span (Hz)",
        "Drift (Hz)",
        "Rate (Hz)"
    };
    const QStringList detectionHeaderTooltips = {
        "Local date and time when the meteor pulse started",
        "Peak signal power during the detection in dB",
        "Peak linear signal amplitude during the detection",
        "Detected pulse duration in milliseconds",
        "Estimated frequency span across the pulse in hertz",
        "Estimated start-to-end frequency drift across the pulse in hertz",
        "Meteor channel detector sample rate in hertz"
    };

    for (int i = 0; i < detectionHeaders.size(); i++)
    {
        QTableWidgetItem *headerItem = new QTableWidgetItem(detectionHeaders[i]);
        headerItem->setToolTip(detectionHeaderTooltips[i]);
        m_detectionsTable->setHorizontalHeaderItem(i, headerItem);
    }

    m_detectionsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_detectionsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_detectionsTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_detectionsTable->horizontalHeader()->setStretchLastSection(true);
    m_detectionsTable->horizontalHeader()->setSectionsMovable(true);
    m_detectionsTable->verticalHeader()->setVisible(false);
    m_detectionsTable->setSortingEnabled(true);
    m_detectionsTable->setMinimumHeight(120);

    m_hourlyChartView->setMinimumHeight(180);
    m_glSpectrum->setMinimumHeight(180);
    m_glScope->setMinimumHeight(180);
}

void MeteorGUI::setupSpectrum()
{
    m_spectrumVis->setGLSpectrum(m_glSpectrum);
    m_spectrumGUI->setBuddies(m_spectrumVis, m_glSpectrum);
    m_glSpectrum->setCenterFrequency(0);
    m_glSpectrum->setSampleRate(m_settings.m_channelSampleRate);
    m_glSpectrum->setLsbDisplay(false);

    SpectrumSettings spectrumSettings = m_spectrumVis->getSettings();
    spectrumSettings.m_displayWaterfall = true;
    spectrumSettings.m_displayMaxHold = false;
    spectrumSettings.m_scrollBar = true;
    SpectrumVis::MsgConfigureSpectrumVis *msg = SpectrumVis::MsgConfigureSpectrumVis::create(spectrumSettings, false);
    m_spectrumVis->getInputMessageQueue()->push(msg);
}

void MeteorGUI::setupScope()
{
    m_scopeVis->setGLScope(m_glScope);
    m_scopeVis->setSpectrumVis(m_spectrumVis);
    m_glScope->connectTimer(MainCore::instance()->getMasterTimer());
    m_scopeGUI->setBuddies(m_scopeVis->getInputMessageQueue(), m_scopeVis, m_glScope);
    m_scopeGUI->setStreams(QStringList({"IQ", "Power dB", "LPF power dB", "Meteor", "Noise floor dB"}));
    m_scopeGUI->setPreTrigger(1);
    m_scopeGUI->setDisplayMode(GLScopeSettings::DisplayX);

    GLScopeSettings::TraceData traceDataI;
    traceDataI.m_streamIndex = 0;
    traceDataI.m_projectionType = Projector::ProjectionReal;
    traceDataI.m_amp = 1.0f;
    traceDataI.setColor(Qt::yellow);
    m_scopeGUI->changeTrace(0, traceDataI);

    GLScopeSettings::TraceData traceDataQ;
    traceDataQ.m_streamIndex = 0;
    traceDataQ.m_projectionType = Projector::ProjectionImag;
    traceDataQ.m_amp = 1.0f;
    traceDataQ.setColor(Qt::cyan);
    m_scopeGUI->addTrace(traceDataQ);

    GLScopeSettings::TraceData traceDataPower;
    traceDataPower.m_streamIndex = 1;
    traceDataPower.m_projectionType = Projector::ProjectionReal;
    traceDataPower.m_amp = 0.02f;
    traceDataPower.m_ofs = -50.0f;
    traceDataPower.setColor(Qt::green);
    m_scopeGUI->addTrace(traceDataPower);

    GLScopeSettings::TraceData traceDataFiltered;
    traceDataFiltered.m_streamIndex = 2;
    traceDataFiltered.m_projectionType = Projector::ProjectionReal;
    traceDataFiltered.m_amp = 0.02f;
    traceDataFiltered.m_ofs = -50.0f;
    traceDataFiltered.setColor(QColor(255, 128, 0));
    m_scopeGUI->addTrace(traceDataFiltered);

    GLScopeSettings::TraceData traceDataMeteor;
    traceDataMeteor.m_streamIndex = 3;
    traceDataMeteor.m_projectionType = Projector::ProjectionReal;
    traceDataMeteor.m_amp = 1.0f;
    traceDataMeteor.m_ofs = -0.5f;
    traceDataMeteor.setColor(Qt::red);
    m_scopeGUI->addTrace(traceDataMeteor);

    GLScopeSettings::TraceData traceDataNoiseFloor;
    traceDataNoiseFloor.m_streamIndex = 4;
    traceDataNoiseFloor.m_projectionType = Projector::ProjectionReal;
    traceDataNoiseFloor.m_amp = 0.02f;
    traceDataNoiseFloor.m_ofs = -50.0f;
    traceDataNoiseFloor.setColor(QColor(180, 120, 255));
    m_scopeGUI->addTrace(traceDataNoiseFloor);

    GLScopeSettings::TriggerData triggerData;
    triggerData.m_streamIndex = 3;
    triggerData.m_triggerLevel = 0.5f;
    triggerData.m_triggerLevelCoarse = 50;
    triggerData.m_triggerPositiveEdge = true;
    m_scopeGUI->changeTrigger(0, triggerData);
    m_scopeGUI->focusOnTrace(0);
    m_scopeGUI->focusOnTrigger(0);
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

    m_settings.m_channelSampleRate = m_sampleRates[index];
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

void MeteorGUI::on_clearDetections_clicked()
{
    m_totalCount = 0;
    m_hourlyCounts.clear();
    m_detectionsTable->setRowCount(0);
    updateCounters();
    updateHistogram();
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
    m_settingsKeys.append(settingsKeys);

    if (m_doApplySettings)
    {
        Meteor::MsgConfigureMeteor* message = Meteor::MsgConfigureMeteor::create(m_settings, m_settingsKeys, force);
        m_meteor->getInputMessageQueue()->push(message);
        m_settingsKeys.clear();
    }
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
    QObject::connect(m_clearDetections, &QPushButton::clicked, this, &MeteorGUI::on_clearDetections_clicked);
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
    m_scopeVis->setLiveRate(m_settings.m_channelSampleRate);
    m_scopeGUI->setSampleRate(m_settings.m_channelSampleRate);

    DSPSignalNotification *msg = new DSPSignalNotification(m_settings.m_channelSampleRate, 0);
    m_spectrumVis->getInputMessageQueue()->push(msg);
}

void MeteorGUI::addDetection(const MeteorDemodSink::MsgMeteorDetected& detection)
{
    const QDateTime localTime = detection.getDateTimeUtc().toLocalTime();
    const QDate date = localTime.date();
    const int hour = localTime.time().hour();

    if (!m_hourlyCounts.contains(date)) {
        m_hourlyCounts[date] = QVector<int>(24, 0);
    }

    m_hourlyCounts[date][hour]++;
    m_totalCount++;

    m_detectionsTable->setSortingEnabled(false);
    m_detectionsTable->insertRow(0);
    m_detectionsTable->setItem(0, 0, makeTableItem(localTime.toString("yyyy-MM-dd HH:mm:ss.zzz"), localTime.toMSecsSinceEpoch()));
    m_detectionsTable->setItem(0, 1, makeTableItem(QString::number(detection.getPeakPowerDB(), 'f', 1), detection.getPeakPowerDB()));
    m_detectionsTable->setItem(0, 2, makeTableItem(QString::number(detection.getPeakAmplitude(), 'f', 4), detection.getPeakAmplitude()));
    m_detectionsTable->setItem(0, 3, makeTableItem(QString::number(detection.getDurationS() * 1000.0, 'f', 1), detection.getDurationS()));
    m_detectionsTable->setItem(0, 4, makeTableItem(QString::number(detection.getFrequencySpan(), 'f', 1), detection.getFrequencySpan()));
    m_detectionsTable->setItem(0, 5, makeTableItem(QString::number(detection.getFrequencyDrift(), 'f', 1), detection.getFrequencyDrift()));
    m_detectionsTable->setItem(0, 6, makeTableItem(QString::number(detection.getSampleRate()), detection.getSampleRate()));
    m_detectionsTable->setSortingEnabled(true);

    updateCounters();
    updateHistogram();
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

    for (auto it = m_hourlyCounts.cbegin(); it != m_hourlyCounts.cend(); ++it)
    {
        QBarSet *set = new QBarSet(it.key().toString(Qt::ISODate));

        for (int hour = 0; hour < 24; hour++)
        {
            const int count = it.value().value(hour);
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

int MeteorGUI::sampleRateIndex(int sampleRate) const
{
    for (int i = 0; i < 4; i++)
    {
        if (m_sampleRates[i] == sampleRate) {
            return i;
        }
    }

    return 2;
}

QTableWidgetItem *MeteorGUI::makeTableItem(const QString& text, const QVariant& sortValue) const
{
    QTableWidgetItem *item = new QTableWidgetItem(text);
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
    if ((m_tickCount++ % 20) == 0) {
        updateCounters();
    }
}
