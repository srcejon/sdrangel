///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Alejandro Aleman                                           //
// Copyright (C) 2020-2026 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2021-2023 Jon Beniston, M7RCE <jon@beniston.com>                //
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

#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QDebug>
#include <QDockWidget>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QTime>
#include <QVBoxLayout>
#include <cmath>

#include "meshcore_identity.h"

#include "device/deviceuiset.h"
#include "device/deviceapi.h"
#include "dsp/devicesamplesink.h"
#include "dsp/devicesamplemimo.h"
#include "plugin/pluginapi.h"
#include "util/db.h"
#include "dsp/dspengine.h"
#include "dsp/dspcommands.h"
#include "gui/basicchannelsettingsdialog.h"
#include "gui/dialpopup.h"
#include "gui/dialogpositioner.h"
#include "maincore.h"

#include "ui_meshcoremodgui.h"
#include "meshcoremodgui.h"
#include "meshcorepacket.h"


MeshcoreModGUI* MeshcoreModGUI::create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx)
{
    MeshcoreModGUI* gui = new MeshcoreModGUI(pluginAPI, deviceUISet, channelTx);
	return gui;
}

void MeshcoreModGUI::destroy()
{
    delete this;
}

void MeshcoreModGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applySettings(true);
}

QByteArray MeshcoreModGUI::serialize() const
{
    return m_settings.serialize();
}

bool MeshcoreModGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        displaySettings();
        applySettings(true);
        return true;
    }
    else
    {
        resetToDefaults();
        return false;
    }
}

bool MeshcoreModGUI::handleMessage(const Message& message)
{
    if (MeshcoreMod::MsgConfigureMeshcoreMod::match(message))
    {
        const MeshcoreMod::MsgConfigureMeshcoreMod& cfg = (MeshcoreMod::MsgConfigureMeshcoreMod&) message;
        m_settings = cfg.getSettings();
        blockApplySettings(true);
        m_channelMarker.updateSettings(static_cast<const ChannelMarker*>(m_settings.m_channelMarker));
        displaySettings();
        blockApplySettings(false);
        return true;
    }
    else if (MeshcoreMod::MsgReportPayloadTime::match(message))
    {
        const MeshcoreMod::MsgReportPayloadTime& rpt = (MeshcoreMod::MsgReportPayloadTime&) message;
        float fourthsMs = ((1<<m_settings.m_spreadFactor) * 250.0) / MeshcoreModSettings::bandwidths[m_settings.m_bandwidthIndex];
        int fourthsChirps = 4*m_settings.m_preambleChirps;
        fourthsChirps += m_settings.hasSyncWord() ? 8 : 0;
        fourthsChirps += m_settings.getNbSFDFourths();
        float controlMs = fourthsChirps * fourthsMs; // preamble + sync word + SFD
        ui->timeMessageLengthText->setText(tr("%1").arg(rpt.getNbSymbols()));
        ui->timePayloadText->setText(tr("%1 ms").arg(QString::number(rpt.getPayloadTimeMs(), 'f', 0)));
        ui->timeTotalText->setText(tr("%1 ms").arg(QString::number(rpt.getPayloadTimeMs() + controlMs, 'f', 0)));
        ui->timeSymbolText->setText(tr("%1 ms").arg(QString::number(4.0*fourthsMs, 'f', 1)));
        return true;
    }
    else if (DSPSignalNotification::match(message))
    {
        DSPSignalNotification& notif = (DSPSignalNotification&) message;
        m_deviceCenterFrequency = notif.getCenterFrequency();
        int basebandSampleRate = notif.getSampleRate();
        qDebug() << "MeshcoreModGUI::handleMessage: DSPSignalNotification: m_basebandSampleRate: " << basebandSampleRate;

        if (basebandSampleRate != m_basebandSampleRate)
        {
            m_basebandSampleRate = basebandSampleRate;
            setBandwidths();
        }

        ui->deltaFrequency->setValueRange(false, 7, -m_basebandSampleRate/2, m_basebandSampleRate/2);
        ui->deltaFrequencyLabel->setToolTip(tr("Range %1 %L2 Hz").arg(QChar(0xB1)).arg(m_basebandSampleRate/2));
        updateAbsoluteCenterFrequency();

        return true;
    }
    else
    {
        return false;
    }
}

void MeshcoreModGUI::channelMarkerChangedByCursor()
{
    ui->deltaFrequency->setValue(m_channelMarker.getCenterFrequency());
    m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
	applySettings();
}

void MeshcoreModGUI::handleSourceMessages()
{
    Message* message;

    while ((message = getInputMessageQueue()->pop()) != 0)
    {
        if (handleMessage(*message))
        {
            delete message;
        }
    }
}

QString MeshcoreModGUI::getActivePayloadText() const
{
    switch (m_settings.m_messageType)
    {
    case MeshcoreModSettings::MessageText:
        return m_settings.m_textMessage;
    case MeshcoreModSettings::MessageGrpTxt:
    case MeshcoreModSettings::MessageTxtMsg:
    case MeshcoreModSettings::MessageAnonReq:
        // For encrypted text payloads the same edit field carries the plaintext.
        return m_settings.m_textMessage;
    default:
        return QString();
    }
}

int MeshcoreModGUI::findBandwidthIndex(int bandwidthHz) const
{
    int bestIndex = -1;
    int bestDelta = 1 << 30;

    for (int i = 0; i < MeshcoreModSettings::nbBandwidths; ++i)
    {
        const int delta = std::abs(MeshcoreModSettings::bandwidths[i] - bandwidthHz);
        if (delta < bestDelta)
        {
            bestDelta = delta;
            bestIndex = i;
        }
    }

    return bestIndex;
}

bool MeshcoreModGUI::retuneDeviceToFrequency(qint64 centerFrequencyHz)
{
    if (!m_deviceUISet || !m_deviceUISet->m_deviceAPI) {
        return false;
    }

    DeviceAPI* deviceAPI = m_deviceUISet->m_deviceAPI;

    if (deviceAPI->getDeviceSinkEngine() && deviceAPI->getSampleSink())
    {
        deviceAPI->getSampleSink()->setCenterFrequency(centerFrequencyHz);
        return true;
    }

    if (deviceAPI->getDeviceMIMOEngine() && deviceAPI->getSampleMIMO())
    {
        deviceAPI->getSampleMIMO()->setSinkCenterFrequency(centerFrequencyHz, m_settings.m_streamIndex);
        return true;
    }

    return false;
}

void MeshcoreModGUI::applyMeshcoreProfileFromSelection()
{
    const QString region = ui->meshRegion->currentText();
    const QString preset = ui->meshPreset->currentText();
    const int meshChannel = ui->meshChannel->currentData().toInt();
    const int channelNum = meshChannel + 1; // planner expects 1-based channel_num

    if (region.isEmpty() || preset.isEmpty()) {
        return;
    }

    const QString command = QString("MESH:preset=%1;region=%2;channel_num=%3").arg(preset, region).arg(channelNum);
    modemmeshcore::TxRadioSettings meshRadio;
    QString error;

    if (!modemmeshcore::Packet::deriveTxRadioSettings(command, meshRadio, error))
    {
        qWarning() << "MeshcoreModGUI::applyMeshcoreProfileFromSelection:" << error;
        return;
    }

    bool changed = false;
    bool selectionStateChanged = false;

    if (m_settings.m_meshcoreRegionCode != region)
    {
        m_settings.m_meshcoreRegionCode = region;
        selectionStateChanged = true;
    }
    if (m_settings.m_meshcorePresetName != preset)
    {
        m_settings.m_meshcorePresetName = preset;
        selectionStateChanged = true;
    }
    if (m_settings.m_meshcoreChannelIndex != meshChannel)
    {
        m_settings.m_meshcoreChannelIndex = meshChannel;
        selectionStateChanged = true;
    }

    const int bwIndex = findBandwidthIndex(meshRadio.bandwidthHz);
    if (bwIndex >= 0 && bwIndex != m_settings.m_bandwidthIndex)
    {
        m_settings.m_bandwidthIndex = bwIndex;
        changed = true;
    }

    if (meshRadio.spreadFactor > 0 && meshRadio.spreadFactor != m_settings.m_spreadFactor)
    {
        m_settings.m_spreadFactor = meshRadio.spreadFactor;
        changed = true;
    }

    if (meshRadio.deBits != m_settings.m_deBits)
    {
        m_settings.m_deBits = meshRadio.deBits;
        changed = true;
    }

    if (meshRadio.parityBits > 0 && meshRadio.parityBits != m_settings.m_nbParityBits)
    {
        m_settings.m_nbParityBits = meshRadio.parityBits;
        changed = true;
    }

    const int meshPreambleChirps = meshRadio.preambleChirps;
    if (m_settings.m_preambleChirps != static_cast<unsigned int>(meshPreambleChirps))
    {
        m_settings.m_preambleChirps = static_cast<unsigned int>(meshPreambleChirps);
        changed = true;
    }

    if (meshRadio.syncWord != m_settings.m_syncWord)
    {
        m_settings.m_syncWord = meshRadio.syncWord;
        changed = true;
    }

    if (meshRadio.hasCenterFrequency)
    {
        if (retuneDeviceToFrequency(meshRadio.centerFrequencyHz))
        {
            if (m_settings.m_inputFrequencyOffset != 0)
            {
                m_settings.m_inputFrequencyOffset = 0;
                changed = true;
            }
        }
        else if (m_deviceCenterFrequency != 0)
        {
            const qint64 wantedOffset = meshRadio.centerFrequencyHz - m_deviceCenterFrequency;
            const qint64 maxOffset = m_basebandSampleRate / 2;

            if (std::abs(wantedOffset) <= maxOffset)
            {
                if (wantedOffset != m_settings.m_inputFrequencyOffset)
                {
                    m_settings.m_inputFrequencyOffset = static_cast<int>(wantedOffset);
                    changed = true;
                }
            }
            else
            {
                qWarning() << "MeshcoreModGUI::applyMeshcoreProfileFromSelection: requested frequency"
                    << meshRadio.centerFrequencyHz
                    << "is out of channel offset range with current baseband sample rate";
            }
        }
        else
        {
            qWarning() << "MeshcoreModGUI::applyMeshcoreProfileFromSelection: device center frequency unknown, cannot auto-center";
        }
    }

    if (!changed && !selectionStateChanged) {
        return;
    }

    qInfo() << "MeshcoreModGUI::applyMeshcoreProfileFromSelection:" << meshRadio.summary;

    if (!changed)
    {
        applySettings();
        return;
    }

    const int thisBW = MeshcoreModSettings::bandwidths[m_settings.m_bandwidthIndex];
    m_channelMarker.blockSignals(true);
    m_channelMarker.setCenterFrequency(m_settings.m_inputFrequencyOffset);
    m_channelMarker.setBandwidth(thisBW);
    m_channelMarker.blockSignals(false);

    blockApplySettings(true);
    ui->deltaFrequency->setValue(m_settings.m_inputFrequencyOffset);
    ui->bw->setValue(m_settings.m_bandwidthIndex);
    ui->bwText->setText(QString("%1 Hz").arg(thisBW));
    ui->spread->setValue(m_settings.m_spreadFactor);
    ui->spreadText->setText(tr("%1").arg(m_settings.m_spreadFactor));
    ui->deBits->setValue(m_settings.m_deBits);
    ui->deBitsText->setText(tr("%1").arg(m_settings.m_deBits));
    ui->preambleChirps->setValue(m_settings.m_preambleChirps);
    ui->preambleChirpsText->setText(tr("%1").arg(m_settings.m_preambleChirps));
    ui->fecParity->setValue(m_settings.m_nbParityBits);
    ui->fecParityText->setText(tr("%1").arg(m_settings.m_nbParityBits));
    ui->syncWord->setText(tr("%1").arg(m_settings.m_syncWord, 2, 16));
    blockApplySettings(false);

    updateAbsoluteCenterFrequency();
    applySettings();
}

void MeshcoreModGUI::rebuildMeshcoreChannelOptions()
{
    const QString region = ui->meshRegion->currentText();
    const QString preset = ui->meshPreset->currentText();
    const int previousChannel = ui->meshChannel->currentData().toInt();

    m_meshControlsUpdating = true;
    ui->meshChannel->clear();

    int added = 0;
    for (int meshChannel = 0; meshChannel <= 200; ++meshChannel)
    {
        modemmeshcore::TxRadioSettings meshRadio;
        QString error;
        const int channelNum = meshChannel + 1; // planner expects 1-based channel_num
        const QString command = QString("MESH:preset=%1;region=%2;channel_num=%3").arg(preset, region).arg(channelNum);

        if (!modemmeshcore::Packet::deriveTxRadioSettings(command, meshRadio, error))
        {
            if (added > 0) {
                break;
            } else {
                continue;
            }
        }

        const QString label = meshRadio.hasCenterFrequency
            ? QString("%1 (%2 MHz)").arg(meshChannel).arg(meshRadio.centerFrequencyHz / 1000000.0, 0, 'f', 3)
            : QString::number(meshChannel);

        ui->meshChannel->addItem(label, meshChannel);
        added++;
    }

    if (added == 0) {
        ui->meshChannel->addItem("0", 0);
    }

    ui->meshChannel->setToolTip(tr("Meshcore channel number (%1 available for %2/%3)")
        .arg(added)
        .arg(region)
        .arg(preset));
    int restoreIndex = ui->meshChannel->findData(previousChannel);
    if (restoreIndex < 0) {
        restoreIndex = 0;
    }
    ui->meshChannel->setCurrentIndex(restoreIndex);
    m_meshControlsUpdating = false;

    qInfo() << "MeshcoreModGUI::rebuildMeshcoreChannelOptions:"
            << "region=" << region
            << "preset=" << preset
            << "channels=" << added;

    QMetaObject::invokeMethod(this, [this]() {
        if (!m_meshControlsUpdating) {
            applyMeshcoreProfileFromSelection();
        }
    }, Qt::QueuedConnection);
}

void MeshcoreModGUI::setupMeshcoreAutoProfileControls()
{
    // Hide Region and Channel controls — region is implicit in each
    // preset's frequency, and MeshCore has no numbered Channel concept.
    // Preset combo + Apply button stay visible (see
    // modemmeshcore::command::applyMeshcorePreset).
    if (ui->meshRegionLabel)  ui->meshRegionLabel->hide();
    if (ui->meshRegion)       ui->meshRegion->hide();
    if (ui->meshChannelLabel) ui->meshChannelLabel->hide();
    if (ui->meshChannel)      ui->meshChannel->hide();
}

void MeshcoreModGUI::on_deltaFrequency_changed(qint64 value)
{
    m_channelMarker.setCenterFrequency(value);
    m_settings.m_inputFrequencyOffset = m_channelMarker.getCenterFrequency();
    updateAbsoluteCenterFrequency();
    applySettings();
}

void MeshcoreModGUI::on_bw_valueChanged(int value)
{
    if (value < 0) {
        m_settings.m_bandwidthIndex = 0;
    } else if (value < MeshcoreModSettings::nbBandwidths) {
        m_settings.m_bandwidthIndex = value;
    } else {
        m_settings.m_bandwidthIndex = MeshcoreModSettings::nbBandwidths - 1;
    }

	int thisBW = MeshcoreModSettings::bandwidths[value];
	ui->bwText->setText(QString("%1 Hz").arg(thisBW));
	m_channelMarker.setBandwidth(thisBW);

	applySettings();
}

void MeshcoreModGUI::on_channelMute_toggled(bool checked)
{
    m_settings.m_channelMute = checked;
	applySettings();
}

void MeshcoreModGUI::on_spread_valueChanged(int value)
{
    m_settings.m_spreadFactor = value;
    ui->spreadText->setText(tr("%1").arg(value));

    applySettings();
}

void MeshcoreModGUI::on_deBits_valueChanged(int value)
{
    m_settings.m_deBits = value;
    ui->deBitsText->setText(tr("%1").arg(m_settings.m_deBits));
    applySettings();
}

void MeshcoreModGUI::on_preambleChirps_valueChanged(int value)
{
    m_settings.m_preambleChirps = value;
    ui->preambleChirpsText->setText(tr("%1").arg(m_settings.m_preambleChirps));
    applySettings();
}

void MeshcoreModGUI::on_idleTime_valueChanged(int value)
{
    m_settings.m_quietMillis = value * 100;
    ui->idleTimeText->setText(tr("%1").arg(m_settings.m_quietMillis / 1000.0, 0, 'f', 1));
    applySettings();
}

void MeshcoreModGUI::on_syncWord_editingFinished()
{
    bool ok;
    unsigned int syncWord = ui->syncWord->text().toUInt(&ok, 16);

    if (ok)
    {
        m_settings.m_syncWord = syncWord > 255 ? 0 : syncWord;
        applySettings();
    }
}

void MeshcoreModGUI::on_fecParity_valueChanged(int value)
{
    m_settings.m_nbParityBits = value;
    ui->fecParityText->setText(tr("%1").arg(m_settings.m_nbParityBits));
    applySettings();
}

void MeshcoreModGUI::on_playMessage_clicked(bool checked)
{
    (void) checked;
    applySettings();
    m_meshcoreMod->sendMessage();
}

void MeshcoreModGUI::on_repeatMessage_valueChanged(int value)
{
    m_settings.m_messageRepeat = value;
    ui->repeatText->setText(tr("%1").arg(m_settings.m_messageRepeat));
    applySettings();
}

void MeshcoreModGUI::on_messageText_editingFinished()
{
    switch (m_settings.m_messageType)
    {
    case MeshcoreModSettings::MessageText:
    case MeshcoreModSettings::MessageGrpTxt:
    case MeshcoreModSettings::MessageTxtMsg:
    case MeshcoreModSettings::MessageAnonReq:
        m_settings.m_textMessage = ui->messageText->toPlainText();
        break;
    default:
        break;
    }
    applySettings();
}

void MeshcoreModGUI::on_hexText_editingFinished()
{
    m_settings.m_bytesMessage = QByteArray::fromHex(ui->hexText->text().toLatin1());
    applySettings();
}

void MeshcoreModGUI::on_udpEnabled_clicked(bool checked)
{
    m_settings.m_udpEnabled = checked;
    applySettings();
}

void MeshcoreModGUI::on_udpAddress_editingFinished()
{
    m_settings.m_udpAddress = ui->udpAddress->text();
    applySettings();
}

void MeshcoreModGUI::on_udpPort_editingFinished()
{
    m_settings.m_udpPort = ui->udpPort->text().toInt();
    applySettings();
}

void MeshcoreModGUI::on_invertRamps_stateChanged(int state)
{
    m_settings.m_invertRamps = (state == Qt::Checked);
    applySettings();
}

void MeshcoreModGUI::on_meshRegion_currentIndexChanged(int index)
{
    (void) index;
    if (m_meshControlsUpdating) {
        return;
    }

    rebuildMeshcoreChannelOptions();
    applyMeshcoreProfileFromSelection();
}

void MeshcoreModGUI::on_meshPreset_currentIndexChanged(int index)
{
    (void) index;
    if (m_meshControlsUpdating) {
        return;
    }

    rebuildMeshcoreChannelOptions();
    applyMeshcoreProfileFromSelection();
}

void MeshcoreModGUI::on_meshChannel_currentIndexChanged(int index)
{
    (void) index;
    if (m_meshControlsUpdating) {
        return;
    }

    applyMeshcoreProfileFromSelection();
}

void MeshcoreModGUI::on_meshApply_clicked(bool checked)
{
    (void) checked;
    if (m_meshControlsUpdating) {
        return;
    }

    rebuildMeshcoreChannelOptions();
    applyMeshcoreProfileFromSelection();
}

void MeshcoreModGUI::onWidgetRolled(QWidget* widget, bool rollDown)
{
    (void) widget;
    (void) rollDown;

    getRollupContents()->saveState(m_rollupState);
    applySettings();
}

void MeshcoreModGUI::onMenuDialogCalled(const QPoint &p)
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
            dialog.setNumberOfStreams(m_meshcoreMod->getNumberOfDeviceStreams());
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

        applySettings();
    }

    resetContextMenuType();
}

MeshcoreModGUI::MeshcoreModGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx, QWidget* parent) :
	ChannelGUI(parent),
	ui(new Ui::MeshcoreModGUI),
	m_pluginAPI(pluginAPI),
	m_deviceUISet(deviceUISet),
	m_channelMarker(this),
    m_deviceCenterFrequency(0),
    m_basebandSampleRate(125000),
	m_doApplySettings(true),
    m_meshControlsUpdating(false),
    m_meshIdPanel(nullptr),
    m_meshIdPubLabel(nullptr),
    m_meshIdNodeNameEdit(nullptr),
    m_meshIdGenerateButton(nullptr),
    m_meshIdCopyPubkeyButton(nullptr),
    m_meshIdMessageTypeCombo(nullptr),
    m_meshIdDestPubEdit(nullptr),
    m_meshIdChannelEdit(nullptr),
    m_meshIdSendNowButton(nullptr),
    m_tickCount(0)
{
	setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/channeltx/modmeshcore/readme.md";
    RollupContents *rollupContents = getRollupContents();
	ui->setupUi(rollupContents);
    setSizePolicy(rollupContents->sizePolicy());
    rollupContents->arrangeRollups();
	connect(rollupContents, SIGNAL(widgetRolled(QWidget*,bool)), this, SLOT(onWidgetRolled(QWidget*,bool)));
	connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));

	m_meshcoreMod = (MeshcoreMod*) channelTx;
	m_meshcoreMod->setMessageQueueToGUI(getInputMessageQueue());

	connect(&MainCore::instance()->getMasterTimer(), SIGNAL(timeout()), this, SLOT(tick()));

    ui->fecParity->setEnabled(true);

    ui->deltaFrequencyLabel->setText(QString("%1f").arg(QChar(0x94, 0x03)));
    ui->deltaFrequency->setColorMapper(ColorMapper(ColorMapper::GrayGold));
    ui->deltaFrequency->setValueRange(false, 7, -9999999, 9999999);
    ui->deltaFrequency->setToolTip(tr("Offset from device center frequency (Hz)."));
    ui->deltaFrequencyLabel->setToolTip(tr("Frequency offset control for the modulator channel."));
    ui->deltaUnits->setToolTip(tr("Frequency unit for the offset control."));
    ui->bw->setToolTip(tr("LoRa transmit bandwidth."));
    ui->bwLabel->setToolTip(tr("LoRa transmit bandwidth selector."));
    ui->bwText->setToolTip(tr("Current LoRa transmit bandwidth in Hz."));
    ui->spread->setToolTip(tr("LoRa spreading factor (SF)."));
    ui->spreadLabel->setToolTip(tr("LoRa spreading factor selector."));
    ui->spreadText->setToolTip(tr("Current spreading factor value."));
    ui->deBits->setToolTip(tr("Low data-rate optimization bits (DE)."));
    ui->deBitsLabel->setToolTip(tr("Low data-rate optimization setting."));
    ui->deBitsText->setToolTip(tr("Current low data-rate optimization value."));
    ui->preambleChirps->setToolTip(tr("LoRa preamble chirp count."));
    ui->preambleChirpsLabel->setToolTip(tr("LoRa preamble chirp count selector."));
    ui->preambleChirpsText->setToolTip(tr("Current preamble chirp value."));
    ui->idleTime->setToolTip(tr("Silence interval between repeated messages (x0.1s)."));
    ui->idleTimeLabel->setToolTip(tr("Idle interval between repeated transmissions."));
    ui->idleTimeText->setToolTip(tr("Current idle interval in seconds."));
    ui->syncWord->setToolTip(tr("LoRa sync word in hexadecimal (00-ff)."));
    ui->syncLabel->setToolTip(tr("LoRa sync word."));
    ui->fecParity->setToolTip(tr("LoRa coding rate parity denominator (CR)."));
    ui->fecParityLabel->setToolTip(tr("LoRa coding rate parity setting."));
    ui->fecParityText->setToolTip(tr("Current coding rate parity value."));
    ui->channelMute->setToolTip(tr("Mute this channel output."));
    ui->playMessage->setToolTip(tr("Queue one transmission of current message type."));
    ui->repeatMessage->setToolTip(tr("Number of repetitions for each triggered transmission."));
    ui->repeatLabel->setToolTip(tr("Transmission repetition count."));
    ui->messageText->setToolTip(tr(
        "Text payload editor. Lines starting with 'MESHCORE:' are parsed as "
        "wire-packet commands and can override radio settings.\n"
        "Examples:\n"
        "  MESHCORE: type=advert; seed=<hex64>; name=Foo\n"
        "  MESHCORE: type=txt_msg; seed=<hex64>; dest=<hex64>; text=Hello\n"
        "  MESHCORE: type=grp_txt; channel=public; text=Hello group\n"
        "  MESHCORE: type=ack; dest=<hex64>; msg_hash=<hex8>\n"
        "Optional radio overrides on any command: sf=, bw=, cr=, sync=, "
        "freq=, preamble="));
    ui->msgLabel->setToolTip(tr("Message payload editor."));
    ui->hexText->setToolTip(tr("Raw hexadecimal payload bytes."));
    ui->hexLabel->setToolTip(tr("Hexadecimal payload editor."));
    ui->udpEnabled->setToolTip(tr("Receive message payloads from UDP input."));
    ui->udpAddress->setToolTip(tr("UDP listen address for incoming payloads."));
    ui->udpPort->setToolTip(tr("UDP listen port for incoming payloads."));
    ui->udpSeparator->setToolTip(tr("UDP input controls."));
    ui->invertRamps->setToolTip(tr("Invert chirp ramp direction. Disabled"));
    ui->invertRamps->setEnabled(false);
    ui->channelPower->setToolTip(tr("Estimated channel output power."));
    ui->timesLabel->setToolTip(tr("Estimated timing values for current LoRa frame."));
    ui->timeSymbolText->setToolTip(tr("Estimated LoRa symbol time."));
    ui->timeSymbolLabel->setToolTip(tr("LoRa symbol time estimate."));
    ui->timeMessageLengthText->setToolTip(tr("Estimated payload symbol count."));
    ui->timeMessageLengthLabel->setToolTip(tr("Payload symbol count estimate."));
    ui->timePayloadText->setToolTip(tr("Estimated payload airtime."));
    ui->timePayloadLabel->setToolTip(tr("Payload airtime estimate."));
    ui->timeTotalText->setToolTip(tr("Estimated total airtime including preamble/control."));
    ui->timeTotalLabel->setToolTip(tr("Total frame airtime estimate."));
    ui->repeatText->setToolTip(tr("Current repetition count."));

    m_channelMarker.blockSignals(true);
    m_channelMarker.setColor(Qt::red);
    m_channelMarker.setBandwidth(12500);
    m_channelMarker.setCenterFrequency(0);
    m_channelMarker.setTitle("MeshCore Modulator");
    m_channelMarker.setSourceOrSinkStream(false);
    m_channelMarker.blockSignals(false);
	m_channelMarker.setVisible(true); // activate signal on the last setting only

	m_deviceUISet->addChannelMarker(&m_channelMarker);

	connect(&m_channelMarker, SIGNAL(changedByCursor()), this, SLOT(channelMarkerChangedByCursor()));

	connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleSourceMessages()));

    m_settings.setChannelMarker(&m_channelMarker);
    m_settings.setRollupState(&m_rollupState);

    setBandwidths();
    setupMeshcoreAutoProfileControls();
    setupMeshcoreIdentityControls();
    displaySettings();
    makeUIConnections();
    applySettings();
    DialPopup::addPopupsToChildDials(this);
    m_resizer.enableChildMouseTracking();
}

MeshcoreModGUI::~MeshcoreModGUI()
{
	delete ui;
}

void MeshcoreModGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void MeshcoreModGUI::applySettings(bool force)
{
	if (m_doApplySettings)
	{
		MeshcoreMod::MsgConfigureMeshcoreMod *msg = MeshcoreMod::MsgConfigureMeshcoreMod::create(m_settings, force);
		m_meshcoreMod->getInputMessageQueue()->push(msg);
	}
}

void MeshcoreModGUI::displaySettings()
{
    int thisBW = MeshcoreModSettings::bandwidths[m_settings.m_bandwidthIndex];

    m_channelMarker.blockSignals(true);
    m_channelMarker.setTitle(m_settings.m_title);
    m_channelMarker.setCenterFrequency(m_settings.m_inputFrequencyOffset);
    m_channelMarker.setBandwidth(thisBW);
    m_channelMarker.blockSignals(false);
    m_channelMarker.setColor(m_settings.m_rgbColor);
    setTitleColor(m_settings.m_rgbColor);

    setWindowTitle(m_channelMarker.getTitle());
    setTitle(m_channelMarker.getTitle());
    updateIndexLabel();
    displayCurrentPayloadMessage();
    displayBinaryMessage();

    ui->fecParity->setEnabled(MeshcoreModSettings::m_codingScheme == MeshcoreModSettings::CodingLoRa);

    blockApplySettings(true);
    ui->deltaFrequency->setValue(m_channelMarker.getCenterFrequency());
    ui->bwText->setText(QString("%1 Hz").arg(thisBW));
    ui->bw->setValue(m_settings.m_bandwidthIndex);
    ui->spread->setValue(m_settings.m_spreadFactor);
    ui->spreadText->setText(tr("%1").arg(m_settings.m_spreadFactor));
    ui->deBits->setValue(m_settings.m_deBits);
    ui->deBitsText->setText(tr("%1").arg(m_settings.m_deBits));
    ui->preambleChirps->setValue(m_settings.m_preambleChirps);
    ui->preambleChirpsText->setText(tr("%1").arg(m_settings.m_preambleChirps));
    ui->idleTime->setValue(m_settings.m_quietMillis / 100);
    ui->idleTimeText->setText(tr("%1").arg(m_settings.m_quietMillis / 1000.0, 0, 'f', 1));
    ui->syncWord->setText((tr("%1").arg(m_settings.m_syncWord, 2, 16)));
    ui->channelMute->setChecked(m_settings.m_channelMute);
    ui->fecParity->setValue(m_settings.m_nbParityBits);
    ui->fecParityText->setText(tr("%1").arg(m_settings.m_nbParityBits));
    ui->repeatMessage->setValue(m_settings.m_messageRepeat);
    ui->repeatText->setText(tr("%1").arg(m_settings.m_messageRepeat));
    ui->udpEnabled->setChecked(m_settings.m_udpEnabled);
    ui->udpAddress->setText(m_settings.m_udpAddress);
    ui->udpPort->setText(QString::number(m_settings.m_udpPort));
    ui->invertRamps->setChecked(m_settings.m_invertRamps);

    m_meshControlsUpdating = true;

    int regionIndex = ui->meshRegion->findText(m_settings.m_meshcoreRegionCode);
    if (regionIndex < 0) {
        regionIndex = ui->meshRegion->findText("EU_868");
    }
    if (regionIndex < 0) {
        regionIndex = 0;
    }
    ui->meshRegion->setCurrentIndex(regionIndex);

    int presetIndex = ui->meshPreset->findText(m_settings.m_meshcorePresetName);
    if (presetIndex < 0) {
        presetIndex = ui->meshPreset->findText("EU_NARROW");
    }
    if (presetIndex < 0) {
        presetIndex = 0;
    }
    ui->meshPreset->setCurrentIndex(presetIndex);
    m_meshControlsUpdating = false;

    rebuildMeshcoreChannelOptions();

    m_meshControlsUpdating = true;
    int channelIndex = ui->meshChannel->findData(m_settings.m_meshcoreChannelIndex);
    if (channelIndex < 0) {
        channelIndex = 0;
    }
    ui->meshChannel->setCurrentIndex(channelIndex);
    m_meshControlsUpdating = false;

    getRollupContents()->restoreState(m_rollupState);
    updateAbsoluteCenterFrequency();

    displayMeshcoreIdentity();
    updateMeshcoreMessageTypeFields();

    blockApplySettings(false);
}

void MeshcoreModGUI::displayCurrentPayloadMessage()
{
    ui->messageText->blockSignals(true);

    switch (m_settings.m_messageType)
    {
    case MeshcoreModSettings::MessageText:
    case MeshcoreModSettings::MessageGrpTxt:
    case MeshcoreModSettings::MessageTxtMsg:
    case MeshcoreModSettings::MessageAnonReq:
        ui->messageText->setText(m_settings.m_textMessage);
        break;
    case MeshcoreModSettings::MessageAdvert:
        ui->messageText->setText(QString());  // ADVERT body driven by identity, not text
        break;
    case MeshcoreModSettings::MessageAck:
        ui->messageText->setText(QString());  // ACK body driven by msg-hash field
        break;
    }

    ui->messageText->blockSignals(false);
}

void MeshcoreModGUI::displayBinaryMessage()
{
    ui->hexText->setText(m_settings.m_bytesMessage.toHex());
}

void MeshcoreModGUI::setBandwidths()
{
    int maxBandwidth = m_basebandSampleRate / MeshcoreModSettings::oversampling;
    int maxIndex = 0;

    for (; (maxIndex < MeshcoreModSettings::nbBandwidths) && (MeshcoreModSettings::bandwidths[maxIndex] <= maxBandwidth); maxIndex++)
    {}

    if (maxIndex != 0)
    {
        qDebug("MeshcoreModGUI::setBandwidths: avl: %d max: %d", maxBandwidth, MeshcoreModSettings::bandwidths[maxIndex-1]);
        ui->bw->setMaximum(maxIndex - 1);
        int index = ui->bw->value();
        ui->bwText->setText(QString("%1 Hz").arg(MeshcoreModSettings::bandwidths[index]));
    }
}

void MeshcoreModGUI::leaveEvent(QEvent* event)
{
	m_channelMarker.setHighlighted(false);
    ChannelGUI::leaveEvent(event);
}

void MeshcoreModGUI::enterEvent(EnterEventType* event)
{
	m_channelMarker.setHighlighted(true);
    ChannelGUI::enterEvent(event);
}

void MeshcoreModGUI::tick()
{
    if (m_tickCount < 10)
    {
        m_tickCount++;
    }
    else
    {
        m_tickCount = 0;
        double powDb = CalcDb::dbPower(m_meshcoreMod->getMagSq());
        m_channelPowerDbAvg(powDb);
        ui->channelPower->setText(tr("%1 dB").arg(m_channelPowerDbAvg.asDouble(), 0, 'f', 1));

        if (m_meshcoreMod->getModulatorActive()) {
            ui->playMessage->setStyleSheet("QPushButton { background-color : green; }");
        } else {
            ui->playMessage->setStyleSheet("QPushButton { background:rgb(79,79,79); }");
        }
    }
}

void MeshcoreModGUI::makeUIConnections()
{
    QObject::connect(ui->deltaFrequency, &ValueDialZ::changed, this, &MeshcoreModGUI::on_deltaFrequency_changed);
    QObject::connect(ui->bw, &QSlider::valueChanged, this, &MeshcoreModGUI::on_bw_valueChanged);
    QObject::connect(ui->spread, &QSlider::valueChanged, this, &MeshcoreModGUI::on_spread_valueChanged);
    QObject::connect(ui->deBits, &QSlider::valueChanged, this, &MeshcoreModGUI::on_deBits_valueChanged);
    QObject::connect(ui->preambleChirps, &QSlider::valueChanged, this, &MeshcoreModGUI::on_preambleChirps_valueChanged);
    QObject::connect(ui->idleTime, &QSlider::valueChanged, this, &MeshcoreModGUI::on_idleTime_valueChanged);
    QObject::connect(ui->syncWord, &QLineEdit::editingFinished, this, &MeshcoreModGUI::on_syncWord_editingFinished);
    QObject::connect(ui->channelMute, &QToolButton::toggled, this, &MeshcoreModGUI::on_channelMute_toggled);
    QObject::connect(ui->fecParity, &QDial::valueChanged, this, &MeshcoreModGUI::on_fecParity_valueChanged);
    QObject::connect(ui->playMessage, &QPushButton::clicked, this, &MeshcoreModGUI::on_playMessage_clicked);
    QObject::connect(ui->repeatMessage, &QDial::valueChanged, this, &MeshcoreModGUI::on_repeatMessage_valueChanged);
    QObject::connect(ui->messageText, &CustomTextEdit::editingFinished, this, &MeshcoreModGUI::on_messageText_editingFinished);
    QObject::connect(ui->hexText, &QLineEdit::editingFinished, this, &MeshcoreModGUI::on_hexText_editingFinished);
    QObject::connect(ui->udpEnabled, &QCheckBox::clicked, this, &MeshcoreModGUI::on_udpEnabled_clicked);
    QObject::connect(ui->udpAddress, &QLineEdit::editingFinished, this, &MeshcoreModGUI::on_udpAddress_editingFinished);
    QObject::connect(ui->udpPort, &QLineEdit::editingFinished, this, &MeshcoreModGUI::on_udpPort_editingFinished);
    QObject::connect(ui->meshRegion, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MeshcoreModGUI::on_meshRegion_currentIndexChanged);
    QObject::connect(ui->meshPreset, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MeshcoreModGUI::on_meshPreset_currentIndexChanged);
    QObject::connect(ui->meshChannel, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MeshcoreModGUI::on_meshChannel_currentIndexChanged);
    QObject::connect(ui->meshApply, &QPushButton::clicked, this, &MeshcoreModGUI::on_meshApply_clicked);
}

void MeshcoreModGUI::updateAbsoluteCenterFrequency()
{
    setStatusFrequency(m_deviceCenterFrequency + m_settings.m_inputFrequencyOffset);
}

// ---- MeshCore Identity / Message-Type panel -----------------------------------
//
// Built programmatically and inserted into the existing rollup contents.
// Layout: a single QGroupBox "MeshCore Identity & Message" with:
//   - Pubkey label (read-only, autogenerated on first launch)
//   - Node name line edit (default "SDRangel-<short-pubkey>")
//   - [Generate] / [Copy pubkey] buttons
//   - Message-Type combo (Text/Advert/TxtMsg/GrpTxt/AnonReq/Ack)
//   - Dest pubkey line edit (relevant for TxtMsg/AnonReq/Ack)
//   - Channel name line edit (relevant for GrpTxt; default "public")
//   - [Send Now] button — equivalent to playMessage but bypasses checked state
//
// Keeps the heavy 41-KB UI XML untouched. Region/preset/channel selectors
// already exist above this panel and govern the carrier PHY for ADVERT TX.

void MeshcoreModGUI::setupMeshcoreIdentityControls()
{
    RollupContents *rollupContents = getRollupContents();
    if (!rollupContents) {
        return;
    }

    m_meshIdPanel = new QGroupBox(tr("MeshCore Identity & Message"), rollupContents);
    m_meshIdPanel->setObjectName(QStringLiteral("meshIdPanel"));

    QFormLayout *form = new QFormLayout(m_meshIdPanel);
    form->setContentsMargins(6, 6, 6, 6);
    form->setHorizontalSpacing(6);
    form->setVerticalSpacing(4);

    m_meshIdPubLabel = new QLabel(QStringLiteral("(loading…)"), m_meshIdPanel);
    m_meshIdPubLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_meshIdPubLabel->setToolTip(
        tr("Local Ed25519 pubkey (32 bytes, hex). Auto-generated on first use; "
           "stored at AppDataLocation/meshcore/identity.bin."));
    form->addRow(tr("Pubkey"), m_meshIdPubLabel);

    m_meshIdNodeNameEdit = new QLineEdit(m_meshIdPanel);
    m_meshIdNodeNameEdit->setPlaceholderText(QStringLiteral("SDRangel-<short-pubkey>"));
    m_meshIdNodeNameEdit->setToolTip(
        tr("Node name advertised in our ADVERT packets. Defaults to "
           "'SDRangel-<first-8-hex-of-pubkey>' when empty."));
    form->addRow(tr("Name"), m_meshIdNodeNameEdit);

    QHBoxLayout *btnRow = new QHBoxLayout();
    btnRow->setSpacing(4);
    m_meshIdGenerateButton = new QPushButton(tr("Generate"), m_meshIdPanel);
    m_meshIdGenerateButton->setToolTip(
        tr("Replace the on-disk identity with a fresh keypair. Existing "
           "contacts that learned our previous pubkey will not recognise us."));
    m_meshIdCopyPubkeyButton = new QPushButton(tr("Copy pubkey"), m_meshIdPanel);
    m_meshIdCopyPubkeyButton->setToolTip(tr("Copy our pubkey hex to the clipboard."));
    btnRow->addWidget(m_meshIdGenerateButton);
    btnRow->addWidget(m_meshIdCopyPubkeyButton);
    btnRow->addStretch(1);
    form->addRow(QString(), btnRow);

    m_meshIdMessageTypeCombo = new QComboBox(m_meshIdPanel);
    m_meshIdMessageTypeCombo->addItem(tr("Text (raw / MESHCORE: command)"),
                                       MeshcoreModSettings::MessageText);
    m_meshIdMessageTypeCombo->addItem(tr("ADVERT (broadcast our identity)"),
                                       MeshcoreModSettings::MessageAdvert);
    m_meshIdMessageTypeCombo->addItem(tr("TXT_MSG (ECDH-encrypted DM)"),
                                       MeshcoreModSettings::MessageTxtMsg);
    m_meshIdMessageTypeCombo->addItem(tr("GRP_TXT (channel PSK)"),
                                       MeshcoreModSettings::MessageGrpTxt);
    m_meshIdMessageTypeCombo->addItem(tr("ANON_REQ (encrypted, sender pub embedded)"),
                                       MeshcoreModSettings::MessageAnonReq);
    m_meshIdMessageTypeCombo->addItem(tr("ACK (plaintext)"),
                                       MeshcoreModSettings::MessageAck);
    m_meshIdMessageTypeCombo->setToolTip(
        tr("MeshCore wire packet type to encode + transmit. ADVERT broadcasts "
           "our identity; the other types build packets from the dest pubkey "
           "/ channel / text fields below."));
    form->addRow(tr("Type"), m_meshIdMessageTypeCombo);

    m_meshIdDestPubEdit = new QLineEdit(m_meshIdPanel);
    m_meshIdDestPubEdit->setPlaceholderText(QStringLiteral("64 hex chars"));
    m_meshIdDestPubEdit->setToolTip(
        tr("Destination pubkey (64 hex). Used by TXT_MSG, ANON_REQ, ACK. "
           "Paste from a companion's contact share."));
    form->addRow(tr("Dest pubkey"), m_meshIdDestPubEdit);

    m_meshIdChannelEdit = new QLineEdit(m_meshIdPanel);
    m_meshIdChannelEdit->setPlaceholderText(QStringLiteral("public"));
    m_meshIdChannelEdit->setToolTip(
        tr("Group channel name for GRP_TXT messages (default 'public', "
           "PSK 8b3387e9c5cdea6ac9e5edbaa115cd72)."));
    form->addRow(tr("Channel"), m_meshIdChannelEdit);

    m_meshIdSendNowButton = new QPushButton(tr("Send now"), m_meshIdPanel);
    m_meshIdSendNowButton->setToolTip(
        tr("Re-encode the selected message type and queue one transmission. "
           "Equivalent to clicking play, but reasserts repeat-count and "
           "re-runs the encoder so the timestamp/identity are fresh."));
    form->addRow(QString(), m_meshIdSendNowButton);

    // Append the panel to the existing rollup vertical layout.
    QLayout *outer = rollupContents->layout();
    if (outer) {
        outer->addWidget(m_meshIdPanel);
    }

    // Wire signals.
    QObject::connect(m_meshIdMessageTypeCombo,
                     QOverload<int>::of(&QComboBox::currentIndexChanged),
                     this, &MeshcoreModGUI::onMeshIdMessageTypeChanged);
    QObject::connect(m_meshIdGenerateButton, &QPushButton::clicked,
                     this, &MeshcoreModGUI::onMeshIdGenerateClicked);
    QObject::connect(m_meshIdCopyPubkeyButton, &QPushButton::clicked,
                     this, &MeshcoreModGUI::onMeshIdCopyPubkeyClicked);
    QObject::connect(m_meshIdNodeNameEdit, &QLineEdit::editingFinished,
                     this, &MeshcoreModGUI::onMeshIdNodeNameEdited);
    QObject::connect(m_meshIdDestPubEdit, &QLineEdit::editingFinished,
                     this, &MeshcoreModGUI::onMeshIdDestPubEdited);
    QObject::connect(m_meshIdChannelEdit, &QLineEdit::editingFinished,
                     this, &MeshcoreModGUI::onMeshIdChannelEdited);
    QObject::connect(m_meshIdSendNowButton, &QPushButton::clicked,
                     this, &MeshcoreModGUI::onMeshIdSendNowClicked);

    displayMeshcoreIdentity();
    updateMeshcoreMessageTypeFields();

    rollupContents->arrangeRollups();
}

void MeshcoreModGUI::displayMeshcoreIdentity()
{
    if (!m_meshIdPanel) return;

    const QString path = m_settings.m_meshIdentityPath.isEmpty()
        ? modemmeshcore::identity::defaultIdentityPath()
        : m_settings.m_meshIdentityPath;
    modemmeshcore::identity::Identity id =
        modemmeshcore::identity::loadOrCreateIdentity(path);

    if (id.isValid()) {
        m_meshIdPubLabel->setText(id.pubHex());
    } else {
        m_meshIdPubLabel->setText(tr("(identity load failed)"));
    }

    {
        QSignalBlocker block(m_meshIdNodeNameEdit);
        if (m_settings.m_meshNodeName.isEmpty()) {
            m_meshIdNodeNameEdit->setText(modemmeshcore::identity::defaultNodeNameFor(id));
        } else {
            m_meshIdNodeNameEdit->setText(m_settings.m_meshNodeName);
        }
    }

    {
        QSignalBlocker block(m_meshIdMessageTypeCombo);
        const int found = m_meshIdMessageTypeCombo->findData(
            static_cast<int>(m_settings.m_messageType));
        if (found >= 0) {
            m_meshIdMessageTypeCombo->setCurrentIndex(found);
        }
    }

    {
        QSignalBlocker block(m_meshIdDestPubEdit);
        m_meshIdDestPubEdit->setText(m_settings.m_meshDestPubKeyHex);
    }
    {
        QSignalBlocker block(m_meshIdChannelEdit);
        m_meshIdChannelEdit->setText(m_settings.m_meshGroupChannelName);
    }
}

void MeshcoreModGUI::updateMeshcoreMessageTypeFields()
{
    if (!m_meshIdDestPubEdit || !m_meshIdChannelEdit) return;
    const auto t = m_settings.m_messageType;
    const bool needsDest =
        (t == MeshcoreModSettings::MessageTxtMsg)
        || (t == MeshcoreModSettings::MessageAnonReq)
        || (t == MeshcoreModSettings::MessageAck);
    const bool needsChannel = (t == MeshcoreModSettings::MessageGrpTxt);
    m_meshIdDestPubEdit->setEnabled(needsDest);
    m_meshIdChannelEdit->setEnabled(needsChannel);
}

void MeshcoreModGUI::onMeshIdMessageTypeChanged(int index)
{
    if (!m_meshIdMessageTypeCombo) return;
    const int raw = m_meshIdMessageTypeCombo->itemData(index).toInt();
    if (raw < MeshcoreModSettings::MessageText
        || raw > MeshcoreModSettings::MessageAck) return;
    m_settings.m_messageType = static_cast<MeshcoreModSettings::MessageType>(raw);
    updateMeshcoreMessageTypeFields();
    applySettings();
}

void MeshcoreModGUI::onMeshIdGenerateClicked()
{
    const QString path = m_settings.m_meshIdentityPath.isEmpty()
        ? modemmeshcore::identity::defaultIdentityPath()
        : m_settings.m_meshIdentityPath;
    modemmeshcore::identity::Identity fresh =
        modemmeshcore::identity::generateIdentity();
    if (!modemmeshcore::identity::saveIdentity(fresh, path)) {
        qWarning() << "MeshcoreModGUI: saveIdentity failed for path:" << path;
        return;
    }
    qInfo() << "MeshcoreModGUI: regenerated identity, new pubkey="
            << fresh.pubHex();
    displayMeshcoreIdentity();
    applySettings(true);  // re-encode with new identity
}

void MeshcoreModGUI::onMeshIdCopyPubkeyClicked()
{
    if (!m_meshIdPubLabel) return;
    QClipboard *cb = QGuiApplication::clipboard();
    if (cb) {
        cb->setText(m_meshIdPubLabel->text());
    }
}

void MeshcoreModGUI::onMeshIdNodeNameEdited()
{
    if (!m_meshIdNodeNameEdit) return;
    m_settings.m_meshNodeName = m_meshIdNodeNameEdit->text();
    applySettings();
}

void MeshcoreModGUI::onMeshIdDestPubEdited()
{
    if (!m_meshIdDestPubEdit) return;
    m_settings.m_meshDestPubKeyHex = m_meshIdDestPubEdit->text().trimmed();
    applySettings();
}

void MeshcoreModGUI::onMeshIdChannelEdited()
{
    if (!m_meshIdChannelEdit) return;
    QString name = m_meshIdChannelEdit->text().trimmed();
    if (name.isEmpty()) name = QStringLiteral("public");
    m_settings.m_meshGroupChannelName = name;
    applySettings();
}

void MeshcoreModGUI::onMeshIdSendNowClicked()
{
    // Re-encode + queue one transmission. Mirrors play-button semantics but
    // forces a re-encode so the freshly-rebuilt MESHCORE: command (with a
    // current timestamp) is what hits the air.
    applySettings(true);
    if (!ui->playMessage->isChecked()) {
        ui->playMessage->setChecked(true);
        on_playMessage_clicked(true);
    } else {
        // Already in play state — toggle off then on so the encoder runs again.
        ui->playMessage->setChecked(false);
        on_playMessage_clicked(false);
        ui->playMessage->setChecked(true);
        on_playMessage_clicked(true);
    }
}
