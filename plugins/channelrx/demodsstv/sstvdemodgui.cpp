///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016 Edouard Griffiths, F4EXB                                   //
// Copyright (C) 2021 Jon Beniston, M7RCE                                        //
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
#include <QDebug>
#include <QFileDialog>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QPushButton>
#include <QComboBox>

#include "sstvdemodgui.h"

#include "device/deviceuiset.h"
#include "dsp/dspengine.h"
#include "dsp/dspcommands.h"
#include "dsp/glscopesettings.h"
#include "ui_sstvdemodgui.h"
#include "plugin/pluginapi.h"
#include "util/db.h"
#include "gui/basicchannelsettingsdialog.h"
#include "gui/dialpopup.h"
#include "gui/dialogpositioner.h"
#include "maincore.h"

#include "sstvdemod.h"

SSTVDemodGUI* SSTVDemodGUI::create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel)
{
    SSTVDemodGUI *gui = new SSTVDemodGUI(pluginAPI, deviceUISet, rxChannel);
    return gui;
}

void SSTVDemodGUI::destroy()
{
    delete this;
}

void SSTVDemodGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applySettings(QStringList(), true);
}

QByteArray SSTVDemodGUI::serialize() const
{
    return m_settings.serialize();
}

bool SSTVDemodGUI::deserialize(const QByteArray& data)
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

bool SSTVDemodGUI::handleMessage(const Message& message)
{
    if (SSTVDemod::MsgConfigureSSTVDemod::match(message))
    {
        qDebug("SSTVDemodGUI::handleMessage: SSTVDemod::MsgConfigureSSTVDemod");
        const SSTVDemod::MsgConfigureSSTVDemod& cfg = (SSTVDemod::MsgConfigureSSTVDemod&) message;
        m_settings = cfg.getSettings();
        blockApplySettings(true);
        m_channelMarker.updateSettings(static_cast<const ChannelMarker*>(m_settings.m_channelMarker));
        displaySettings();
        blockApplySettings(false);
        return true;
    }
    else if (SSTVDemod::MsgImage::match(message))
    {
        const SSTVDemod::MsgImage& imgMsg = (SSTVDemod::MsgImage&) message;
        const QImage& lineImage = imgMsg.getImage();
        int lineIndex = imgMsg.getLineIndex();

        // Initialise or grow the display image as lines arrive
        if (m_image.isNull() || (m_image.width() == 0))
        {
            const SSTVDemodSettings::PDModeParams modeParams = SSTVDemodSettings::getPDModeParams(m_settings.m_pdMode);
            m_image = QImage(modeParams.width, modeParams.height, QImage::Format_RGB32);
            m_image.fill(Qt::black);
        }

        // Copy the two decoded scan lines into the correct position in the image
        int destY = lineIndex * 2;
        if ((destY + 1) < m_image.height())
        {
            // Copy odd line (row 0 of the block)
            const uchar *srcLine0 = lineImage.scanLine(0);
            uchar *dstLine0 = m_image.scanLine(destY);
            memcpy(dstLine0, srcLine0, (size_t)(lineImage.bytesPerLine()));

            // Copy even line (row 1 of the block)
            const uchar *srcLine1 = lineImage.scanLine(1);
            uchar *dstLine1 = m_image.scanLine(destY + 1);
            memcpy(dstLine1, srcLine1, (size_t)(lineImage.bytesPerLine()));
        }

        m_pixmap.convertFromImage(m_image);
        if (m_pixmapItem != nullptr)
        {
            m_pixmapItem->setPixmap(m_pixmap);
            if (ui->zoomAll->isChecked()) {
                ui->image->fitInView(m_pixmapItem, Qt::KeepAspectRatio);
            }
        }
        else
        {
            m_pixmapItem = m_scene->addPixmap(m_pixmap);
            m_pixmapItem->setPos(0, 0);
            ui->image->fitInView(m_pixmapItem, Qt::KeepAspectRatio);
        }

        return true;
    }
    else if (DSPSignalNotification::match(message))
    {
        DSPSignalNotification& notif = (DSPSignalNotification&) message;
        m_deviceCenterFrequency = notif.getCenterFrequency();
        m_basebandSampleRate = notif.getSampleRate();
        ui->deltaFrequency->setValueRange(false, 7, -m_basebandSampleRate / 2, m_basebandSampleRate / 2);
        ui->deltaFrequencyLabel->setToolTip(
            tr("Range %1 %L2 Hz").arg(QChar(0xB1)).arg(m_basebandSampleRate / 2));
        updateAbsoluteCenterFrequency();
        return true;
    }

    return false;
}

void SSTVDemodGUI::handleInputMessages()
{
    Message *message;

    while ((message = getInputMessageQueue()->pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void SSTVDemodGUI::channelMarkerChangedByCursor()
{
    ui->deltaFrequency->setValue(m_channelMarker.getCenterFrequency());
    m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
    applySettings(QStringList("inputFrequencyOffset"), false);
}

void SSTVDemodGUI::channelMarkerHighlightedByCursor()
{
    setHighlighted(m_channelMarker.getHighlighted());
}

void SSTVDemodGUI::on_deltaFrequency_changed(qint64 value)
{
    m_channelMarker.setCenterFrequency(value);
    m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
    updateAbsoluteCenterFrequency();
    applySettings(QStringList("inputFrequencyOffset"), false);
}

void SSTVDemodGUI::on_rfBW_valueChanged(int value)
{
    float bw = value * 100.0f;
    ui->rfBWText->setText(QString("%1k").arg(value / 10.0, 0, 'f', 1));
    m_channelMarker.setBandwidth(bw);
    m_settings.m_rfBandwidth = bw;
    applySettings(QStringList("rfBandwidth"), false);
}

void SSTVDemodGUI::on_fmDev_valueChanged(int value)
{
    ui->fmDevText->setText(QString("%1k").arg(value / 10.0, 0, 'f', 1));
    m_settings.m_fmDeviation = value * 100.0f;
    applySettings(QStringList("fmDeviation"), false);
}

void SSTVDemodGUI::on_modulation_currentIndexChanged(int index)
{
    m_settings.m_modulation = static_cast<SSTVDemodSettings::Modulation>(index);
    // Show FM deviation controls only for FM mode; they are not used for SSB
    const bool isFM = (m_settings.m_modulation == SSTVDemodSettings::ModulationFM);
    ui->fmDevLabel->setVisible(isFM);
    ui->fmDev->setVisible(isFM);
    ui->fmDevText->setVisible(isFM);
    applySettings(QStringList("modulation"), false);
}

void SSTVDemodGUI::on_pdMode_currentIndexChanged(int index)
{
    m_settings.m_pdMode = static_cast<SSTVDemodSettings::PDMode>(index);
    applySettings(QStringList("pdMode"), false);
    // Clear the display when the mode changes: the image geometry changes
    resetImage();
    m_sstvDemod->getInputMessageQueue()->push(SSTVDemod::MsgResetDecoder::create());
}

void SSTVDemodGUI::on_startStop_clicked(bool checked)
{
    m_settings.m_decodeEnabled = checked;
    applySettings(QStringList("decodeEnabled"), false);
}

void SSTVDemodGUI::on_resetDecoder_clicked()
{
    resetImage();
    m_sstvDemod->getInputMessageQueue()->push(SSTVDemod::MsgResetDecoder::create());
}

void SSTVDemodGUI::on_saveImage_clicked()
{
    QFileDialog fileDialog(nullptr, "Select file to save image to", "",
                           "PNG (*.png);;JPEG (*.jpg *.jpeg);;BMP (*.bmp);;All files (*.*)");
    fileDialog.setAcceptMode(QFileDialog::AcceptSave);
    if (fileDialog.exec())
    {
        QStringList fileNames = fileDialog.selectedFiles();
        if (!fileNames.isEmpty())
        {
            if (!m_image.save(fileNames[0])) {
                qWarning() << "SSTVDemodGUI::on_saveImage_clicked: failed to save image to" << fileNames[0];
            }
        }
    }
}

void SSTVDemodGUI::on_zoomIn_clicked()
{
    ui->image->scale(1.25, 1.25);
    ui->zoomAll->setChecked(false);
}

void SSTVDemodGUI::on_zoomOut_clicked()
{
    ui->image->scale(0.75, 0.75);
    ui->zoomAll->setChecked(false);
}

void SSTVDemodGUI::on_zoomAll_clicked(bool checked)
{
    if (checked && (m_pixmapItem != nullptr)) {
        ui->image->fitInView(m_pixmapItem, Qt::KeepAspectRatio);
    }
}

void SSTVDemodGUI::onWidgetRolled(QWidget* widget, bool rollDown)
{
    (void) widget;
    (void) rollDown;

    getRollupContents()->saveState(m_rollupState);
    applySettings(QStringList(), false);
}

void SSTVDemodGUI::onMenuDialogCalled(const QPoint &p)
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

        if (m_deviceUISet->m_deviceMIMOEngine)
        {
            dialog.setNumberOfStreams(m_sstvDemod->getNumberOfDeviceStreams());
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

        setWindowTitle(m_settings.m_title);
        setTitle(m_channelMarker.getTitle());
        setTitleColor(m_settings.m_rgbColor);

        if (m_deviceUISet->m_deviceMIMOEngine)
        {
            m_settings.m_streamIndex = dialog.getSelectedStreamIndex();
            m_channelMarker.clearStreamIndexes();
            m_channelMarker.addStreamIndex(m_settings.m_streamIndex);
            updateIndexLabel();
        }

        applySettings(QStringList({
            "rgbColor",
            "title",
            "useReverseAPI",
            "reverseAPIAddress",
            "reverseAPIPort",
            "reverseAPIDeviceIndex",
            "reverseAPIChannelIndex",
            "streamIndex"
        }), false);
    }

    resetContextMenuType();
}

SSTVDemodGUI::SSTVDemodGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel, QWidget *parent) :
    ChannelGUI(parent),
    ui(new Ui::SSTVDemodGUI),
    m_pluginAPI(pluginAPI),
    m_deviceUISet(deviceUISet),
    m_channelMarker(this),
    m_deviceCenterFrequency(0),
    m_doApplySettings(true),
    m_basebandSampleRate(0),
    m_tickCount(0),
    m_scopeVis(nullptr),
    m_spectrumVis(nullptr),
    m_scene(nullptr),
    m_pixmapItem(nullptr)
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/channelrx/demodsstv/readme.md";
    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    setSizePolicy(rollupContents->sizePolicy());
    rollupContents->arrangeRollups();
    connect(rollupContents, SIGNAL(widgetRolled(QWidget*, bool)), this, SLOT(onWidgetRolled(QWidget*, bool)));
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));

    m_sstvDemod = reinterpret_cast<SSTVDemod*>(rxChannel);
    m_sstvDemod->setMessageQueueToGUI(getInputMessageQueue());

    connect(&MainCore::instance()->getMasterTimer(), SIGNAL(timeout()), this, SLOT(tick())); // 50 ms

    m_scopeVis = m_sstvDemod->getScopeSink();
    m_scopeVis->setGLScope(ui->glScope);
    ui->glScope->connectTimer(MainCore::instance()->getMasterTimer());
    ui->scopeGUI->setBuddies(m_scopeVis->getInputMessageQueue(), m_scopeVis, ui->glScope);
    ui->scopeGUI->setStreams(QStringList({"FM demod", "Freq (Hz)", "Sync tone", "PLL locked", "State"}));

    // Set up a default trace for the FM demod signal
    ui->scopeGUI->setPreTrigger(1);
    GLScopeSettings::TraceData traceData;
    traceData.m_projectionType = Projector::ProjectionReal;
    traceData.m_amp = 1.0;
    traceData.m_ofs = 0.0;
    ui->scopeGUI->changeTrace(0, traceData);
    ui->scopeGUI->focusOnTrace(0);

    GLScopeSettings::TriggerData triggerData;
    triggerData.m_triggerLevel = 0.1;
    triggerData.m_triggerLevelCoarse = 10;
    triggerData.m_triggerPositiveEdge = true;
    ui->scopeGUI->changeTrigger(0, triggerData);
    ui->scopeGUI->focusOnTrigger(0);

    m_scopeVis->setLiveRate(SSTVDEMOD_CHANNEL_SAMPLE_RATE);

    m_spectrumVis = m_sstvDemod->getSpectrumVis();
    m_spectrumVis->setGLSpectrum(ui->glSpectrum);
    ui->glSpectrum->setCenterFrequency(0);
    ui->glSpectrum->setSampleRate(SSTVDEMOD_CHANNEL_SAMPLE_RATE);
    ui->glSpectrum->setDisplayWaterfall(true);
    ui->glSpectrum->setDisplayMaxHold(false);
    ui->glSpectrum->setDisplayHistogram(false);
    ui->spectrumGUI->setBuddies(m_spectrumVis, ui->glSpectrum);

    ui->deltaFrequencyLabel->setText(QString("%1f").arg(QChar(0x94, 0x03)));
    ui->deltaFrequency->setColorMapper(ColorMapper(ColorMapper::GrayGold));
    ui->deltaFrequency->setValueRange(false, 7, -9999999, 9999999);
    ui->channelPowerMeter->setColorTheme(LevelMeterSignalDB::ColorGreenAndBlue);

    m_channelMarker.blockSignals(true);
    m_channelMarker.setColor(Qt::red);
    m_channelMarker.setBandwidth(m_settings.m_rfBandwidth);
    m_channelMarker.setCenterFrequency(m_settings.m_inputFrequencyOffset);
    m_channelMarker.setTitle("SSTV Demodulator");
    m_channelMarker.blockSignals(false);
    m_channelMarker.setVisible(true);

    setTitleColor(m_channelMarker.getColor());
    m_settings.setChannelMarker(&m_channelMarker);
    m_settings.setScopeGUI(ui->scopeGUI);
    m_settings.setSpectrumGUI(ui->spectrumGUI);
    m_settings.setRollupState(&m_rollupState);

    m_deviceUISet->addChannelMarker(&m_channelMarker);

    connect(&m_channelMarker, SIGNAL(changedByCursor()), this, SLOT(channelMarkerChangedByCursor()));
    connect(&m_channelMarker, SIGNAL(highlightedByCursor()), this, SLOT(channelMarkerHighlightedByCursor()));
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    m_scene = new QGraphicsScene(ui->image);
    ui->image->setScene(m_scene);
    ui->image->show();
    ui->image->setDragMode(QGraphicsView::ScrollHandDrag);

    displaySettings();
    makeUIConnections();
    applySettings(QStringList(), true);
    DialPopup::addPopupsToChildDials(this);
    m_resizer.enableChildMouseTracking();
}

SSTVDemodGUI::~SSTVDemodGUI()
{
    delete ui;
}

void SSTVDemodGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void SSTVDemodGUI::applySettings(const QStringList& settingsKeys, bool force)
{
    if (m_doApplySettings)
    {
        SSTVDemod::MsgConfigureSSTVDemod *message =
            SSTVDemod::MsgConfigureSSTVDemod::create(settingsKeys, m_settings, force);
        m_sstvDemod->getInputMessageQueue()->push(message);
    }
}

void SSTVDemodGUI::displaySettings()
{
    m_channelMarker.blockSignals(true);
    m_channelMarker.setBandwidth(m_settings.m_rfBandwidth);
    m_channelMarker.setCenterFrequency(m_settings.m_inputFrequencyOffset);
    m_channelMarker.setTitle(m_settings.m_title);
    m_channelMarker.blockSignals(false);
    m_channelMarker.setColor(m_settings.m_rgbColor);

    setTitleColor(m_settings.m_rgbColor);
    setWindowTitle(m_channelMarker.getTitle());
    setTitle(m_channelMarker.getTitle());

    blockApplySettings(true);

    ui->deltaFrequency->setValue(m_channelMarker.getCenterFrequency());

    ui->rfBWText->setText(QString("%1k").arg(m_settings.m_rfBandwidth / 1000.0, 0, 'f', 1));
    ui->rfBW->setValue((int)(m_settings.m_rfBandwidth / 100.0f));

    ui->fmDevText->setText(QString("%1k").arg(m_settings.m_fmDeviation / 1000.0, 0, 'f', 1));
    ui->fmDev->setValue((int)(m_settings.m_fmDeviation / 100.0f));

    ui->modulation->setCurrentIndex((int) m_settings.m_modulation);
    const bool isFM = (m_settings.m_modulation == SSTVDemodSettings::ModulationFM);
    ui->fmDevLabel->setVisible(isFM);
    ui->fmDev->setVisible(isFM);
    ui->fmDevText->setVisible(isFM);

    ui->pdMode->setCurrentIndex((int) m_settings.m_pdMode);

    ui->startStop->setChecked(m_settings.m_decodeEnabled);

    updateIndexLabel();
    getRollupContents()->restoreState(m_rollupState);
    updateAbsoluteCenterFrequency();
    blockApplySettings(false);
}

void SSTVDemodGUI::resetImage()
{
    if (m_pixmapItem != nullptr)
    {
        m_image = QImage();
        m_pixmapItem->setPixmap(QPixmap());
    }
    ui->imageContainer->setWindowTitle("Received image");
}

void SSTVDemodGUI::makeUIConnections()
{
    QObject::connect(ui->deltaFrequency, &ValueDialZ::changed, this, &SSTVDemodGUI::on_deltaFrequency_changed);
    QObject::connect(ui->rfBW, &QSlider::valueChanged, this, &SSTVDemodGUI::on_rfBW_valueChanged);
    QObject::connect(ui->fmDev, &QSlider::valueChanged, this, &SSTVDemodGUI::on_fmDev_valueChanged);
    QObject::connect(ui->modulation, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SSTVDemodGUI::on_modulation_currentIndexChanged);
    QObject::connect(ui->pdMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SSTVDemodGUI::on_pdMode_currentIndexChanged);
    QObject::connect(ui->startStop, &ButtonSwitch::clicked, this, &SSTVDemodGUI::on_startStop_clicked);
    QObject::connect(ui->resetDecoder, &QPushButton::clicked, this, &SSTVDemodGUI::on_resetDecoder_clicked);
    QObject::connect(ui->saveImage, &QPushButton::clicked, this, &SSTVDemodGUI::on_saveImage_clicked);
    QObject::connect(ui->zoomIn, &QPushButton::clicked, this, &SSTVDemodGUI::on_zoomIn_clicked);
    QObject::connect(ui->zoomOut, &QPushButton::clicked, this, &SSTVDemodGUI::on_zoomOut_clicked);
    QObject::connect(ui->zoomAll, &ButtonSwitch::clicked, this, &SSTVDemodGUI::on_zoomAll_clicked);
}

void SSTVDemodGUI::updateAbsoluteCenterFrequency()
{
    setStatusFrequency(m_deviceCenterFrequency + m_settings.m_inputFrequencyOffset);
}

void SSTVDemodGUI::leaveEvent(QEvent* event)
{
    m_channelMarker.setHighlighted(false);
    ChannelGUI::leaveEvent(event);
}

void SSTVDemodGUI::enterEvent(EnterEventType* event)
{
    m_channelMarker.setHighlighted(true);
    ChannelGUI::enterEvent(event);
}

void SSTVDemodGUI::tick()
{
    double magsqAvg, magsqPeak;
    int nbMagsqSamples;
    m_sstvDemod->getMagSqLevels(magsqAvg, magsqPeak, nbMagsqSamples);
    double powDbAvg = CalcDb::dbPower(magsqAvg);
    double powDbPeak = CalcDb::dbPower(magsqPeak);

    ui->channelPowerMeter->levelChanged(
        (100.0f + powDbAvg) / 100.0f,
        (100.0f + powDbPeak) / 100.0f,
        nbMagsqSamples);

    if (m_tickCount % 4 == 0) {
        ui->channelPower->setText(QString::number(powDbAvg, 'f', 1));
    }

    m_tickCount++;
}
