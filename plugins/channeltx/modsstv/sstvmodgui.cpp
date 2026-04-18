///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by Copilot / Claude Sonnet                                          //
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

#include <QDockWidget>
#include <QMainWindow>
#include <QFileDialog>
#include <QTimer>
#include <QDebug>

#include "device/deviceuiset.h"
#include "plugin/pluginapi.h"
#include "util/db.h"
#include "dsp/dspengine.h"
#include "dsp/dspcommands.h"
#include "dsp/spectrumvis.h"
#include "dsp/scopevis.h"
#include "dsp/glscopesettings.h"
#include "gui/glspectrum.h"
#include "gui/buttonswitch.h"
#include "gui/crightclickenabler.h"
#include "gui/basicchannelsettingsdialog.h"
#include "gui/dialpopup.h"
#include "gui/dialogpositioner.h"
#include "maincore.h"

#include "ui_sstvmodgui.h"
#include "sstvmodsource.h"
#include "sstvmodgui.h"


SSTVModGUI* SSTVModGUI::create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx)
{
    SSTVModGUI* gui = new SSTVModGUI(pluginAPI, deviceUISet, channelTx);
    return gui;
}

void SSTVModGUI::destroy()
{
    delete this;
}

void SSTVModGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applySettings(QStringList(), true);
}

QByteArray SSTVModGUI::serialize() const
{
    return m_settings.serialize();
}

bool SSTVModGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        displaySettings();
        applySettings(QStringList(), true);
        return true;
    }
    else
    {
        resetToDefaults();
        return false;
    }
}

bool SSTVModGUI::handleMessage(const Message& message)
{
    if (SSTVMod::MsgConfigureSSTVMod::match(message))
    {
        const auto& cfg = static_cast<const SSTVMod::MsgConfigureSSTVMod&>(message);
        m_settings.applySettings(cfg.getSettingsKeys(), cfg.getSettings());
        displaySettings();
        return true;
    }
    else if (SSTVMod::MsgReportTransmitComplete::match(message))
    {
        qDebug("SSTVModGUI: transmission complete");
        ui->startStop->blockSignals(true);
        ui->startStop->setChecked(false);
        ui->startStop->blockSignals(false);
        ui->startStop->setStyleSheet("QToolButton { background:rgb(79,79,79); }");
        return true;
    }
    else if (DSPSignalNotification::match(message))
    {
        const auto& notif = static_cast<const DSPSignalNotification&>(message);
        m_deviceCenterFrequency = notif.getCenterFrequency();
        updateAbsoluteCenterFrequency();
        return true;
    }
    return false;
}

void SSTVModGUI::channelMarkerChangedByCursor()
{
    ui->deltaFrequency->setValue(m_channelMarker.getCenterFrequency());
    m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
    applySettings(QStringList("inputFrequencyOffset"));
}

void SSTVModGUI::on_deltaFrequency_changed(qint64 value)
{
    m_channelMarker.setCenterFrequency(value);
    m_settings.m_inputFrequencyOffset = value;
    updateAbsoluteCenterFrequency();
    applySettings(QStringList("inputFrequencyOffset"));
}

void SSTVModGUI::on_rfBW_valueChanged(int value)
{
    float bw = value * 100.0f;
    ui->rfBWText->setText(QString("%1k").arg(bw / 1000.0f, 0, 'f', 1));
    m_channelMarker.setBandwidth((int) bw);
    m_settings.m_rfBandwidth = bw;
    applySettings(QStringList("rfBandwidth"));
}

void SSTVModGUI::on_modulation_currentIndexChanged(int index)
{
    m_settings.m_modulation = (SSTVModSettings::Modulation) index;
    // Show/hide FM deviation controls
    bool isFM = (index == 0);
    ui->fmDevLabel->setVisible(isFM);
    ui->fmDeviation->setVisible(isFM);
    ui->fmDevText->setVisible(isFM);
    applySettings(QStringList("modulation"));
}

void SSTVModGUI::on_sstvMode_currentIndexChanged(int index)
{
    m_settings.m_sstvMode = static_cast<SSTVModSettings::SSTVMode>(index);
    applySettings(QStringList("sstvMode"));
    // Reload the image so it is rescaled to the new mode dimensions
    if (!m_settings.m_imagePath.isEmpty()) {
        loadImage();
    }
}

void SSTVModGUI::on_fmDeviation_valueChanged(int value)
{
    float dev = value * 100.0f;
    ui->fmDevText->setText(QString("%1k").arg(dev / 1000.0f, 0, 'f', 1));
    m_settings.m_fmDeviation = dev;
    applySettings(QStringList("fmDeviation"));
}

void SSTVModGUI::on_loadImage_clicked(bool /*checked*/)
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open Image"),
        m_settings.m_imagePath.isEmpty() ? QDir::homePath() : QFileInfo(m_settings.m_imagePath).dir().absolutePath(),
        tr("Images (*.png *.jpg *.jpeg *.bmp)")
    );

    if (fileName.isEmpty()) {
        return;
    }

    m_settings.m_imagePath = fileName;
    ui->imagePath->setText(QFileInfo(fileName).fileName());

    applySettings(QStringList("imagePath"));

    loadImage();
}

void SSTVModGUI::loadImage()
{
    QImage image(m_settings.m_imagePath);
    if (image.isNull())
    {
        qWarning() << "SSTVModGUI::on_loadImage_clicked: failed to load" << m_settings.m_imagePath;
        return;
    }

    // Scale image to active mode dimensions
    const SSTVModSettings::SSTVModeParams modeParams = SSTVModSettings::getModeParams(m_settings.m_sstvMode);
    image = image.scaled(modeParams.width, modeParams.height);

    // Display preview
    QPixmap pix = QPixmap::fromImage(image);
    //ui->imagePreview->setPixmap(pix.scaled(ui->imagePreview->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->imagePreview->setPixmap(pix);
}

void SSTVModGUI::on_startStop_toggled(bool checked)
{
    if (checked)
    {
        // Start transmission
        ui->startStop->setStyleSheet("QToolButton { background:rgb(200,100,0); }");
        SSTVMod::MsgStartStop *msg = SSTVMod::MsgStartStop::create(true);
        m_sstvMod->getInputMessageQueue()->push(msg);
    }
    else
    {
        // Stop transmission
        ui->startStop->setStyleSheet("QToolButton { background:rgb(79,79,79); }");
        SSTVMod::MsgStartStop *msg = SSTVMod::MsgStartStop::create(false);
        m_sstvMod->getInputMessageQueue()->push(msg);
    }
}

void SSTVModGUI::on_repeat_toggled(bool checked)
{
    m_settings.m_repeat = checked;
    applySettings(QStringList("repeat"));
}

void SSTVModGUI::onWidgetRolled(QWidget* /*widget*/, bool /*rollDown*/)
{
    getRollupContents()->saveState(m_rollupState);
    applySettings(QStringList("rollupState"));
}

void SSTVModGUI::onMenuDialogCalled(const QPoint& p)
{
    if (m_contextMenuType == ContextMenuType::ContextMenuChannelSettings)
    {
        BasicChannelSettingsDialog dialog(&m_channelMarker, this);
        dialog.setUseReverseAPI(m_settings.m_useReverseAPI);
        dialog.setReverseAPIAddress(m_settings.m_reverseAPIAddress);
        dialog.setReverseAPIPort(m_settings.m_reverseAPIPort);
        dialog.setReverseAPIDeviceIndex(m_settings.m_reverseAPIDeviceIndex);
        dialog.setReverseAPIChannelIndex(m_settings.m_reverseAPIChannelIndex);
        dialog.setDefaultTitle(m_displayedName);

        if (m_deviceUISet->m_deviceMIMOEngine) {
            dialog.setNumberOfStreams(m_sstvMod->getNumberOfDeviceStreams());
            dialog.setStreamIndex(m_settings.m_streamIndex);
        }

        dialog.move(p);
        new DialogPositioner(&dialog, false);
        dialog.exec();

        m_settings.m_rgbColor = m_channelMarker.getColor().rgb();
        m_settings.m_title = m_channelMarker.getTitle();
        m_settings.m_useReverseAPI = dialog.useReverseAPI();
        m_settings.m_reverseAPIAddress = dialog.getReverseAPIAddress();
        m_settings.m_reverseAPIPort = dialog.getReverseAPIPort();
        m_settings.m_reverseAPIDeviceIndex = dialog.getReverseAPIDeviceIndex();
        m_settings.m_reverseAPIChannelIndex = dialog.getReverseAPIChannelIndex();

        if (m_deviceUISet->m_deviceMIMOEngine)
        {
            m_settings.m_streamIndex = dialog.getSelectedStreamIndex();
            m_channelMarker.clearStreamIndexes();
            m_channelMarker.addStreamIndex(m_settings.m_streamIndex);
        }

        setTitle(m_channelMarker.getTitle());
        setTitleColor(m_settings.m_rgbColor);
        applySettings(QStringList({ "rgbColor", "title", "streamIndex",
            "useReverseAPI", "reverseAPIAddress", "reverseAPIPort",
            "reverseAPIDeviceIndex", "reverseAPIChannelIndex" }));
    }

    resetContextMenuType();
}

void SSTVModGUI::tick()
{
    double powDb = CalcDb::dbPower(m_sstvMod->getMagSq());
    m_channelPowerDbAvg(powDb);
    ui->channelPower->setText(QString::number(m_channelPowerDbAvg.asDouble(), 'f', 1));
}

void SSTVModGUI::handleSourceMessages()
{
    Message* message;
    while ((message = getInputMessageQueue()->pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

SSTVModGUI::SSTVModGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx, QWidget* parent) :
    ChannelGUI(parent),
    ui(new Ui::SSTVModGUI),
    m_pluginAPI(pluginAPI),
    m_deviceUISet(deviceUISet),
    m_channelMarker(this),
    m_doApplySettings(true),
    m_sstvMod(nullptr)
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/channeltx/modsstv/readme.md";
    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    setSizePolicy(rollupContents->sizePolicy());
    rollupContents->arrangeRollups();
    connect(rollupContents, SIGNAL(widgetRolled(QWidget*, bool)), this, SLOT(onWidgetRolled(QWidget*, bool)));
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));

    m_sstvMod = (SSTVMod*) channelTx;
    m_sstvMod->setMessageQueueToGUI(getInputMessageQueue());

    connect(&MainCore::instance()->getMasterTimer(), SIGNAL(timeout()), this, SLOT(tick()));

    // Scope setup
    m_scopeVis = m_sstvMod->getScopeSink();
    m_scopeVis->setGLScope(ui->glScope);
    ui->glScope->connectTimer(MainCore::instance()->getMasterTimer());
    ui->scopeGUI->setBuddies(m_scopeVis->getInputMessageQueue(), m_scopeVis, ui->glScope);

    ui->scopeGUI->setPreTrigger(1);
    GLScopeSettings::TraceData traceDataI, traceDataQ;
    traceDataI.m_projectionType = Projector::ProjectionReal;
    traceDataI.m_amp = 1.0;
    traceDataI.m_ofs = 0.0;
    traceDataQ.m_projectionType = Projector::ProjectionImag;
    traceDataQ.m_amp = 1.0;
    traceDataQ.m_ofs = 0.0;
    ui->scopeGUI->changeTrace(0, traceDataI);
    ui->scopeGUI->addTrace(traceDataQ);
    ui->scopeGUI->setDisplayMode(GLScopeSettings::DisplayX);
    ui->scopeGUI->focusOnTrace(0);

    GLScopeSettings::TriggerData triggerData;
    triggerData.m_triggerLevel = 0.1;
    triggerData.m_triggerLevelCoarse = 10;
    triggerData.m_triggerPositiveEdge = true;
    ui->scopeGUI->changeTrigger(0, triggerData);
    ui->scopeGUI->focusOnTrigger(0);

    m_scopeVis->setLiveRate(SSTV_SAMPLE_RATE);

    // Spectrum setup
    m_spectrumVis = m_sstvMod->getSpectrumVis();
    m_spectrumVis->setGLSpectrum(ui->glSpectrum);
    ui->spectrumGUI->setBuddies(m_spectrumVis, ui->glSpectrum);

    ui->glSpectrum->setCenterFrequency(0);
    ui->glSpectrum->setSampleRate(SSTV_SAMPLE_RATE);
    ui->glSpectrum->setLsbDisplay(false);

    SpectrumSettings spectrumSettings = m_spectrumVis->getSettings();
    spectrumSettings.m_ssb = false;
    spectrumSettings.m_displayCurrent = true;
    spectrumSettings.m_displayWaterfall = false;
    spectrumSettings.m_displayMaxHold = false;
    spectrumSettings.m_displayHistogram = false;
    SpectrumVis::MsgConfigureSpectrumVis *specMsg = SpectrumVis::MsgConfigureSpectrumVis::create(spectrumSettings, false);
    m_spectrumVis->getInputMessageQueue()->push(specMsg);

    ui->scopeContainer->setVisible(false);
    ui->spectrumContainer->setVisible(false);

    ui->deltaFrequency->setColorMapper(ColorMapper(ColorMapper::GrayGold));
    ui->deltaFrequency->setValueRange(false, 7, -9999999, 9999999);

    m_channelMarker.blockSignals(true);
    m_channelMarker.setColor(Qt::red);
    m_channelMarker.setBandwidth(16000);
    m_channelMarker.setCenterFrequency(0);
    m_channelMarker.setTitle("SSTV Modulator");
    m_channelMarker.setSourceOrSinkStream(false);
    m_channelMarker.blockSignals(false);
    m_channelMarker.setVisible(true);

    setTitleColor(m_channelMarker.getColor());

    m_deviceUISet->addChannelMarker(&m_channelMarker);

    connect(&m_channelMarker, SIGNAL(changedByCursor()), this, SLOT(channelMarkerChangedByCursor()));
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleSourceMessages()));

    m_settings.setChannelMarker(&m_channelMarker);
    m_settings.setRollupState(&m_rollupState);
    m_settings.setSpectrumGUI(ui->spectrumGUI);
    m_settings.setScopeGUI(ui->scopeGUI);

    displaySettings();
    makeUIConnections();
    applySettings(QStringList(), true);
    updateAbsoluteCenterFrequency();
    DialPopup::addPopupsToChildDials(this);
    m_resizer.enableChildMouseTracking();
}

SSTVModGUI::~SSTVModGUI()
{
    delete ui;
}

void SSTVModGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void SSTVModGUI::applySettings(const QStringList& settingsKeys, bool force)
{
    if (!m_doApplySettings) {
        return;
    }
    SSTVMod::MsgConfigureSSTVMod *msg = SSTVMod::MsgConfigureSSTVMod::create(settingsKeys, m_settings, force);
    m_sstvMod->getInputMessageQueue()->push(msg);
}

void SSTVModGUI::displaySettings()
{
    m_channelMarker.blockSignals(true);
    m_channelMarker.setCenterFrequency(m_settings.m_inputFrequencyOffset);
    m_channelMarker.setBandwidth((int) m_settings.m_rfBandwidth);
    m_channelMarker.setTitle(m_settings.m_title);
    m_channelMarker.blockSignals(false);
    m_channelMarker.setColor(m_settings.m_rgbColor);

    setTitleColor(m_settings.m_rgbColor);
    setWindowTitle(m_channelMarker.getTitle());
    setTitle(m_channelMarker.getTitle());

    blockApplySettings(true);

    ui->deltaFrequency->setValue(m_settings.m_inputFrequencyOffset);

    int rfBWkHz = (int)(m_settings.m_rfBandwidth / 100.0f);
    ui->rfBW->setValue(rfBWkHz);
    ui->rfBWText->setText(QString("%1k").arg(m_settings.m_rfBandwidth / 1000.0f, 0, 'f', 1));

    ui->modulation->setCurrentIndex((int) m_settings.m_modulation);
    bool isFM = (m_settings.m_modulation == SSTVModSettings::ModulationFM);
    ui->fmDevLabel->setVisible(isFM);
    ui->fmDeviation->setVisible(isFM);
    ui->fmDevText->setVisible(isFM);

    ui->sstvMode->setCurrentIndex((int) m_settings.m_sstvMode);

    int fmDevVal = (int)(m_settings.m_fmDeviation / 100.0f);
    ui->fmDeviation->setValue(fmDevVal);
    ui->fmDevText->setText(QString("%1k").arg(m_settings.m_fmDeviation / 1000.0f, 0, 'f', 1));

    if (!m_settings.m_imagePath.isEmpty())
    {
        ui->imagePath->setText(QFileInfo(m_settings.m_imagePath).fileName());
        loadImage();
    }

    ui->repeat->setChecked(m_settings.m_repeat);

    getRollupContents()->restoreState(m_rollupState);
    updateAbsoluteCenterFrequency();

    blockApplySettings(false);
}

void SSTVModGUI::leaveEvent(QEvent* event)
{
    m_channelMarker.setHighlighted(false);
    ChannelGUI::leaveEvent(event);
}

void SSTVModGUI::enterEvent(EnterEventType* event)
{
    m_channelMarker.setHighlighted(true);
    ChannelGUI::enterEvent(event);
}

void SSTVModGUI::makeUIConnections()
{
    QObject::connect(ui->deltaFrequency, &ValueDialZ::changed, this, &SSTVModGUI::on_deltaFrequency_changed);
    QObject::connect(ui->rfBW, &QSlider::valueChanged, this, &SSTVModGUI::on_rfBW_valueChanged);
    QObject::connect(ui->modulation, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SSTVModGUI::on_modulation_currentIndexChanged);
    QObject::connect(ui->sstvMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SSTVModGUI::on_sstvMode_currentIndexChanged);
    QObject::connect(ui->fmDeviation, &QSlider::valueChanged, this, &SSTVModGUI::on_fmDeviation_valueChanged);
    QObject::connect(ui->loadImage, &QToolButton::clicked, this, &SSTVModGUI::on_loadImage_clicked);
    QObject::connect(ui->startStop, &QToolButton::toggled, this, &SSTVModGUI::on_startStop_toggled);
    QObject::connect(ui->repeat, &ButtonSwitch::toggled, this, &SSTVModGUI::on_repeat_toggled);
}

void SSTVModGUI::updateAbsoluteCenterFrequency()
{
    setStatusFrequency(m_deviceCenterFrequency + m_settings.m_inputFrequencyOffset);
}
