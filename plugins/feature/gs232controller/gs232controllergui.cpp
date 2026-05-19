///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020-2023 Jon Beniston, M7RCE <jon@beniston.com>                //
// Copyright (C) 2020-2022 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2023 Lamar Owen <lamar.owen@gmail.com>                          //
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

#include <cmath>
#include <QMessageBox>
#include <QPushButton>
#include <QSerialPortInfo>

#include "SWGTargetAzimuthElevation.h"

#include "feature/featureuiset.h"
#include "gui/basicfeaturesettingsdialog.h"
#include "gui/dialogpositioner.h"
#include "gui/crightclickenabler.h"
#include "util/astronomy.h"

#include "ui_gs232controllergui.h"
#include "gs232controller.h"
#include "gs232controllergui.h"
#include "gs232controllerreport.h"
#include "alpacaprotocol.h"
#include "dfmprotocol.h"
#include "maincore.h"

GS232ControllerGUI* GS232ControllerGUI::create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature)
{
    GS232ControllerGUI* gui = new GS232ControllerGUI(pluginAPI, featureUISet, feature);
    return gui;
}

void GS232ControllerGUI::destroy()
{
    delete this;
}

void GS232ControllerGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applyAllSettings();
}

QByteArray GS232ControllerGUI::serialize() const
{
    return m_settings.serialize();
}

bool GS232ControllerGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        m_feature->setWorkspaceIndex(m_settings.m_workspaceIndex);
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

void GS232ControllerGUI::azElToDisplay(float az, float el, float& coord1, float& coord2) const
{
    AzAlt aa;
    double c1, c2;
    if (m_settings.m_coordinates == GS232ControllerSettings::X_Y_85)
    {
        aa.az = az;
        aa.alt = el;
        Astronomy::azAltToXY85(aa, c1, c2);
        coord1 = (float)c1;
        coord2 = (float)c2;
    }
    else if (m_settings.m_coordinates == GS232ControllerSettings::X_Y_30)
    {
        aa.az = az;
        aa.alt = el;
        Astronomy::azAltToXY30(aa, c1, c2);
        coord1 = (float)c1;
        coord2 = (float)c2;
    }
    else
    {
        coord1 = az;
        coord2 = el;
    }
}

void GS232ControllerGUI::displayToAzEl(float coord1, float coord2)
{
    if (m_settings.m_coordinates == GS232ControllerSettings::X_Y_85)
    {
        AzAlt aa = Astronomy::xy85ToAzAlt(coord1, coord2);
        m_settings.m_azimuth = aa.az;
        m_settings.m_elevation = aa.alt;
    }
    else if (m_settings.m_coordinates == GS232ControllerSettings::X_Y_30)
    {
        AzAlt aa = Astronomy::xy30ToAzAlt(coord1, coord2);
        m_settings.m_azimuth = aa.az;
        m_settings.m_elevation = aa.alt;
    }
    else
    {
        m_settings.m_azimuth = coord1;
        m_settings.m_elevation = coord2;
    }
    applySettings({"azimuth", "elevation"});
}

bool GS232ControllerGUI::handleMessage(const Message& message)
{
    if (GS232Controller::MsgConfigureGS232Controller::match(message))
    {
        qDebug("GS232ControllerGUI::handleMessage: GS232Controller::MsgConfigureGS232Controller");
        const GS232Controller::MsgConfigureGS232Controller& cfg = (GS232Controller::MsgConfigureGS232Controller&) message;

        if (cfg.getForce()) {
            m_settings = cfg.getSettings();
        } else {
            m_settings.applySettings(cfg.getSettingsKeys(), cfg.getSettings());
        }

        blockApplySettings(true);
        displaySettings();
        blockApplySettings(false);

        return true;
    }
    else if (GS232Controller::MsgReportAvailableChannelOrFeatures::match(message))
    {
        GS232Controller::MsgReportAvailableChannelOrFeatures& report =
            (GS232Controller::MsgReportAvailableChannelOrFeatures&) message;
        updatePipeList(report.getItems(), report.getRenameFrom(), report.getRenameTo());
        return true;
    }
    else if (GS232ControllerReport::MsgReportAzAl::match(message))
    {
        GS232ControllerReport::MsgReportAzAl& azAl = (GS232ControllerReport::MsgReportAzAl&) message;
        float coord1, coord2;
        azElToDisplay(azAl.getAzimuth(), azAl.getElevation(), coord1, coord2);
        ui->coord1CurrentText->setText(QString::number(coord1, 'f', m_settings.m_precision));
        ui->coord2CurrentText->setText(QString::number(coord2, 'f', m_settings.m_precision));
        return true;
    }
    else if (MainCore::MsgTargetAzimuthElevation::match(message))
    {
        MainCore::MsgTargetAzimuthElevation& msg = (MainCore::MsgTargetAzimuthElevation&) message;
        SWGSDRangel::SWGTargetAzimuthElevation *swgTarget = msg.getSWGTargetAzimuthElevation();
        float coord1, coord2;
        azElToDisplay(swgTarget->getAzimuth(), swgTarget->getElevation(), coord1, coord2);
        ui->coord1->setValue(coord1);
        ui->coord2->setValue(coord2);
        ui->targetName->setText(*swgTarget->getName());
        return true;
    }
    else if (GS232Controller::MsgReportSerialPorts::match(message))
    {
        GS232Controller::MsgReportSerialPorts& msg = (GS232Controller::MsgReportSerialPorts&) message;
        updateSerialPortList(msg.getSerialPorts());
        return true;
    }
    else if (DFMProtocol::MsgReportDFMStatus::match(message))
    {
        DFMProtocol::MsgReportDFMStatus& report = (DFMProtocol::MsgReportDFMStatus&) message;
        m_dfmStatusDialog.displayStatus(report.getDFMStatus());
        return true;
    }
    else if (ControllerProtocol::MsgReportParkState::match(message))
    {
        ControllerProtocol::MsgReportParkState& report = (ControllerProtocol::MsgReportParkState&) message;
        m_canPark = report.canPark();
        m_atPark = report.atPark();
        m_parkStateValid = report.parkValid();
        m_canFindHome = report.canFindHome();
        m_atHome = report.atHome();
        m_homeStateValid = report.homeValid();
        m_slewing = report.slewing();
        m_slewingStateValid = report.slewingValid();
        updateParkAndHomeControls();
        return true;
    }
    else if (ControllerProtocol::MsgReportPositionMismatch::match(message))
    {
        ControllerProtocol::MsgReportPositionMismatch& report = (ControllerProtocol::MsgReportPositionMismatch&) message;
        handlePositionMismatch(report);
        return true;
    }

    return false;
}

void GS232ControllerGUI::handleInputMessages()
{
    Message* message;

    while ((message = getInputMessageQueue()->pop()))
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void GS232ControllerGUI::onWidgetRolled(QWidget* widget, bool rollDown)
{
    (void) widget;
    (void) rollDown;

    getRollupContents()->saveState(m_rollupState);
    applySetting("rollupState");
}

GS232ControllerGUI::GS232ControllerGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent) :
    FeatureGUI(parent),
    ui(new Ui::GS232ControllerGUI),
    m_pluginAPI(pluginAPI),
    m_featureUISet(featureUISet),
    m_doApplySettings(true),
    m_lastFeatureState(0),
    m_lastOnTarget(false),
    m_canPark(false),
    m_atPark(false),
    m_parkStateValid(false),
    m_canFindHome(false),
    m_atHome(false),
    m_homeStateValid(false),
    m_slewing(false),
    m_slewingStateValid(false),
    m_dfmStatusDialog(),
    m_inputController(nullptr),
    m_inputCoord1(0.0),
    m_inputCoord2(0.0),
    m_inputAzOffset(0.0),
    m_inputElOffset(0.0),
    m_inputUpdate(false)
{
    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/gs232controller/readme.md";
    RollupContents *rollupContents = getRollupContents();
	ui->setupUi(rollupContents);
    rollupContents->arrangeRollups();
	connect(rollupContents, SIGNAL(widgetRolled(QWidget*,bool)), this, SLOT(onWidgetRolled(QWidget*,bool)));

    m_gs232Controller = reinterpret_cast<GS232Controller*>(feature);
    m_gs232Controller->setMessageQueueToGUI(&m_inputMessageQueue);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    connect(&m_statusTimer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    m_statusTimer.start(250);

    ui->coord1CurrentText->setText("-");
    ui->coord2CurrentText->setText("-");
    setProtocol(m_settings.m_protocol); // Hide DFM buttons

    updateSerialPortList();
    if (ui->serialPort->currentIndex() >= 0) {
        on_serialPort_currentIndexChanged(ui->serialPort->currentIndex());
    }

    m_settings.setRollupState(&m_rollupState);

    //ui->inputConfigure->setVisible(false);
    ui->inputConfigure->setEnabled(false);
    updateInputControllerList();
    connect(InputControllerManager::instance(), &InputControllerManager::controllersChanged, this, &GS232ControllerGUI::updateInputControllerList);
    connect(&m_inputTimer, &QTimer::timeout, this, &GS232ControllerGUI::checkInputController);

    CRightClickEnabler *useMyPositionClickEnabler = new CRightClickEnabler(ui->useMyPosition);
    connect(useMyPositionClickEnabler, &CRightClickEnabler::rightClick, this, &GS232ControllerGUI::useMyPosition_rightClicked);


    displaySettings();
    applyAllSettings();
    makeUIConnections();

    // Get pre-existing pipes
    m_gs232Controller->getInputMessageQueue()->push(GS232Controller::MsgScanAvailableChannelOrFeatures::create());

    new DialogPositioner(&m_dfmStatusDialog, true);
    m_resizer.enableChildMouseTracking();
}

void GS232ControllerGUI::updateInputControllerList()
{
    ui->inputController->blockSignals(true);
    ui->inputController->clear();
    ui->inputController->addItem("None");

    QStringList controllers = InputControllerManager::getAllControllers();
    for (const auto& controller : controllers) {
        ui->inputController->addItem(controller);
    }
    ui->inputController->blockSignals(false);
    int index = ui->inputController->findText(m_settings.m_inputController);
    ui->inputController->setCurrentIndex(index);
}

void GS232ControllerGUI::updateInputController()
{
    delete m_inputController;
    m_inputController = nullptr;

    bool enabled = false;
    if (m_settings.m_inputController != "None")
    {
        m_inputController = InputControllerManager::open(m_settings.m_inputController);
        if (m_inputController)
        {
            connect(m_inputController, &InputController::buttonChanged, this, &GS232ControllerGUI::buttonChanged);
            connect(m_inputController, &InputController::configurationComplete, this, &GS232ControllerGUI::inputConfigurationComplete);
            m_inputTimer.start(20);
            enabled = true;
        }
    }
    else
    {
        m_inputTimer.stop();
    }
    ui->inputConfigure->setEnabled(enabled);
}

void GS232ControllerGUI::buttonChanged(int button, bool released)
{
    if (!released)
    {
        switch (button)
        {
        case INPUTCONTROLLER_BUTTON_RIGHT_TOP:
            ui->startStop->doToggle(!ui->startStop->isChecked());
            break;
        case INPUTCONTROLLER_BUTTON_RIGHT_BOTTOM:
            ui->track->click();
            break;
        case INPUTCONTROLLER_BUTTON_RIGHT_RIGHT:
            ui->enableTargetControl->click();
            break;
        case INPUTCONTROLLER_BUTTON_RIGHT_LEFT:
            ui->enableOffsetControl->click();
            break;
        case INPUTCONTROLLER_BUTTON_R1:
            ui->highSensitivity->click();
            break;
        }
    }
}

void GS232ControllerGUI::checkInputController()
{
    if (m_inputController)
    {
        // If our input device has two sticks (four axes), we use one for target and one for offset
        // If only one stick (two axes), it's used both for target when not tracking and offset, when tracking
        // Use separate variables rather than values in UI, to allow for higher precision

        if (!m_settings.m_track)
        {
            if (m_settings.m_targetControlEnabled)
            {
                m_inputCoord1 += m_extraSensitivity * m_inputController->getAxisCalibratedValue(0, &m_settings.m_inputControllerSettings, m_settings.m_highSensitivity);
                m_inputCoord2 += m_extraSensitivity * -m_inputController->getAxisCalibratedValue(1, &m_settings.m_inputControllerSettings, m_settings.m_highSensitivity);
            }

            if (m_settings.m_coordinates == GS232ControllerSettings::AZ_EL)
            {
                m_inputCoord1 = std::max(m_inputCoord1, (double) m_settings.m_azimuthMin);
                m_inputCoord1 = std::min(m_inputCoord1, (double) m_settings.m_azimuthMax);
                m_inputCoord2 = std::max(m_inputCoord2, (double) m_settings.m_elevationMin);
                m_inputCoord2 = std::min(m_inputCoord2, (double) m_settings.m_elevationMax);
            }
            else
            {
                m_inputCoord1 = std::max(m_inputCoord1, -90.0);
                m_inputCoord1 = std::min(m_inputCoord1, 90.0);
                m_inputCoord2 = std::max(m_inputCoord2, -90.0);
                m_inputCoord2 = std::min(m_inputCoord2, 90.0);
            }
        }

        if ((m_inputController->getNumberOfAxes() < 4) && m_settings.m_track)
        {
            if (m_settings.m_offsetControlEnabled)
            {
                m_inputAzOffset += m_extraSensitivity * m_inputController->getAxisCalibratedValue(0, &m_settings.m_inputControllerSettings, m_settings.m_highSensitivity);
                m_inputElOffset += m_extraSensitivity * -m_inputController->getAxisCalibratedValue(1, &m_settings.m_inputControllerSettings, m_settings.m_highSensitivity);
            }
        }
        else if (m_inputController->getNumberOfAxes() >= 4)
        {
            if (m_settings.m_offsetControlEnabled)
            {
                m_inputAzOffset += m_extraSensitivity * m_inputController->getAxisCalibratedValue(2, &m_settings.m_inputControllerSettings, m_settings.m_highSensitivity);
                m_inputElOffset += m_extraSensitivity * -m_inputController->getAxisCalibratedValue(3, &m_settings.m_inputControllerSettings, m_settings.m_highSensitivity);
            }
        }
        m_inputAzOffset = std::max(m_inputAzOffset, -360.0);
        m_inputAzOffset = std::min(m_inputAzOffset, 360.0);
        m_inputElOffset = std::max(m_inputElOffset, -180.0);
        m_inputElOffset = std::min(m_inputElOffset, 180.0);

        m_inputUpdate = true;
        if (!m_settings.m_track)
        {
            ui->coord1->setValue(m_inputCoord1);
            ui->coord2->setValue(m_inputCoord2);
        }
        if (((m_inputController->getNumberOfAxes() < 4) && m_settings.m_track) || (m_inputController->getNumberOfAxes() >= 4))
        {
            ui->azimuthOffset->setValue(m_inputAzOffset);
            ui->elevationOffset->setValue(m_inputElOffset);
        }
        m_inputUpdate = false;
    }
}

void GS232ControllerGUI::on_inputController_currentIndexChanged(int index)
{
    // Don't update settings if set to -1
    if (index >= 0)
    {
        m_settings.m_inputController = ui->inputController->currentText();
        applySetting("inputController");
        updateInputController();
    }
}

void GS232ControllerGUI::on_inputConfigure_clicked()
{
    if (m_inputController) {
        m_inputController->configure(&m_settings.m_inputControllerSettings);
    }
}

void GS232ControllerGUI::on_highSensitivity_clicked(bool checked)
{
    m_settings.m_highSensitivity = checked;
    ui->highSensitivity->setText(checked ? "H" : "L");
    applySetting("highSensitivity");
}

void GS232ControllerGUI::on_enableTargetControl_clicked(bool checked)
{
    m_settings.m_targetControlEnabled = checked;
    applySetting("targetControlEnabled");
}

void GS232ControllerGUI::on_enableOffsetControl_clicked(bool checked)
{
    m_settings.m_offsetControlEnabled  = checked;
    applySetting("offsetControlEnabled");
}

void GS232ControllerGUI::inputConfigurationComplete()
{
    applySetting("inputControllerSettings");
}

GS232ControllerGUI::~GS232ControllerGUI()
{
    m_dfmStatusDialog.close();
    delete ui;
}

void GS232ControllerGUI::setWorkspaceIndex(int index)
{
    m_settings.m_workspaceIndex = index;
    m_feature->setWorkspaceIndex(index);
}

void GS232ControllerGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void GS232ControllerGUI::displaySettings()
{
    setTitleColor(m_settings.m_rgbColor);
    setWindowTitle(m_settings.m_title);
    setTitle(m_settings.m_title);
    blockApplySettings(true);
    ui->precision->setValue(m_settings.m_precision); // Must set before protocol and az/el
    ui->protocol->setCurrentIndex((int)m_settings.m_protocol);
    ui->coordinates->setCurrentIndex((int)m_settings.m_coordinates);
    float coord1, coord2;
    azElToDisplay(m_settings.m_azimuth, m_settings.m_elevation, coord1, coord2);
    ui->coord1->setValue(coord1);
    ui->coord2->setValue(coord2);
    ui->connection->setCurrentIndex((int)m_settings.m_connection);
    if (m_settings.m_serialPort.length() > 0) {
        ui->serialPort->lineEdit()->setText(m_settings.m_serialPort);
    }
    ui->baudRate->setCurrentText(QString("%1").arg(m_settings.m_baudRate));
    ui->host->setText(m_settings.m_host);
    ui->port->setValue(m_settings.m_port);
    ui->track->setChecked(m_settings.m_track);
    ui->sources->setCurrentIndex(ui->sources->findText(m_settings.m_source));
    ui->azimuthOffset->setValue(m_settings.m_azimuthOffset);
    ui->elevationOffset->setValue(m_settings.m_elevationOffset);
    ui->azimuthMin->setValue(m_settings.m_azimuthMin);
    ui->azimuthMax->setValue(m_settings.m_azimuthMax);
    ui->elevationMin->setValue(m_settings.m_elevationMin);
    ui->elevationMax->setValue(m_settings.m_elevationMax);
    ui->tolerance->setValue(m_settings.m_tolerance);
    ui->inputController->setCurrentText(m_settings.m_inputController);
    ui->highSensitivity->setChecked(m_settings.m_highSensitivity);
    ui->enableTargetControl->setChecked(m_settings.m_targetControlEnabled);
    ui->enableOffsetControl->setChecked(m_settings.m_offsetControlEnabled);
    ui->lineEnding->setCurrentIndex((int) m_settings.m_lineEnding);
    ui->latitude->setValue(m_settings.m_latitude);
    ui->longitude->setValue(m_settings.m_longitude);
    ui->altitude->setValue(m_settings.m_altitude);
    ui->useMyPosition->setChecked(m_settings.m_positionSync);
    ui->dfmTrack->setChecked(m_settings.m_dfmTrackOn);
    ui->dfmLubePumps->setChecked(m_settings.m_dfmLubePumpsOn);
    ui->dfmBrakes->setChecked(m_settings.m_dfmBrakesOn);
    ui->dfmDrives->setChecked(m_settings.m_dfmDrivesOn);
    getRollupContents()->restoreState(m_rollupState);
    updateConnectionWidgets();
    blockApplySettings(false);
    applyPositionSync();
}

void GS232ControllerGUI::updateConnectionWidgets()
{
    bool serial = (m_settings.m_connection == GS232ControllerSettings::SERIAL)
        && (m_settings.m_protocol != GS232ControllerSettings::ALPACA);
    ui->serialPortLabel->setVisible(serial);
    ui->serialPort->setVisible(serial);
    ui->baudRateLabel->setVisible(serial);
    ui->baudRate->setVisible(serial);
    ui->hostLabel->setVisible(!serial);
    ui->host->setVisible(!serial);
    ui->portLabel->setVisible(!serial);
    ui->port->setVisible(!serial);
    ui->connection->setEnabled(m_settings.m_protocol != GS232ControllerSettings::ALPACA);
}

void GS232ControllerGUI::updateSerialPortList()
{
    ui->serialPort->clear();
    QList<QSerialPortInfo> serialPorts = QSerialPortInfo::availablePorts();
    QListIterator<QSerialPortInfo> i(serialPorts);
    while (i.hasNext())
    {
        QSerialPortInfo info = i.next();
        ui->serialPort->addItem(info.portName());
    }
}

void GS232ControllerGUI::updateSerialPortList(const QStringList& serialPorts)
{
    ui->serialPort->blockSignals(true);
    ui->serialPort->clear();
    for (const auto& serialPort : serialPorts) {
        ui->serialPort->addItem(serialPort);
    }
    if (!m_settings.m_serialPort.isEmpty()) {
        ui->serialPort->setCurrentText(m_settings.m_serialPort);
    }
    ui->serialPort->blockSignals(false);
}

void GS232ControllerGUI::updatePipeList(const AvailableChannelOrFeatureList& sources, const QStringList& renameFrom, const QStringList& renameTo)
{
    // Update source setting if it has been renamed
    if (renameFrom.contains(m_settings.m_source))
    {
        m_settings.m_source = renameTo[renameFrom.indexOf(m_settings.m_source)];
        applySetting("source");
    }

    int prevIdx = ui->sources->currentIndex();
    ui->sources->blockSignals(true);
    ui->sources->clear();

    for (const auto& source : sources) {
        ui->sources->addItem(source.getLongId());
    }

    // Select current setting, if it exists
    // If not, and no prior setting, make sure nothing selected, as channel/feature may be created later on
    // If not found and something was previously selected, clear the setting, as probably deleted
    int idx = ui->sources->findText(m_settings.m_source);
    if (idx >= 0)
    {
        ui->sources->setCurrentIndex(idx);
    }
    else if (prevIdx == -1)
    {
        ui->sources->setCurrentIndex(-1);
    }
    else
    {
        m_settings.m_source = "";
        applySetting("source");
        ui->targetName->setText("");
    }

    ui->sources->blockSignals(false);

    // If no current setting, select first available
    if (m_settings.m_source.isEmpty() && (ui->sources->count() > 0))
    {
        ui->sources->setCurrentIndex(0);
        on_sources_currentTextChanged(ui->sources->currentText());
    }
}

void GS232ControllerGUI::onMenuDialogCalled(const QPoint &p)
{
    if (m_contextMenuType == ContextMenuType::ContextMenuChannelSettings)
    {
        BasicFeatureSettingsDialog dialog(this);
        dialog.setTitle(m_settings.m_title);
        dialog.setUseReverseAPI(m_settings.m_useReverseAPI);
        dialog.setReverseAPIAddress(m_settings.m_reverseAPIAddress);
        dialog.setReverseAPIPort(m_settings.m_reverseAPIPort);
        dialog.setReverseAPIFeatureSetIndex(m_settings.m_reverseAPIFeatureSetIndex);
        dialog.setReverseAPIFeatureIndex(m_settings.m_reverseAPIFeatureIndex);
        dialog.setDefaultTitle(m_displayedName);

        dialog.move(p);
        new DialogPositioner(&dialog, false);
        dialog.exec();

        m_settings.m_title = dialog.getTitle();
        m_settings.m_useReverseAPI = dialog.useReverseAPI();
        m_settings.m_reverseAPIAddress = dialog.getReverseAPIAddress();
        m_settings.m_reverseAPIPort = dialog.getReverseAPIPort();
        m_settings.m_reverseAPIFeatureSetIndex = dialog.getReverseAPIFeatureSetIndex();
        m_settings.m_reverseAPIFeatureIndex = dialog.getReverseAPIFeatureIndex();

        setTitle(m_settings.m_title);
        setTitleColor(m_settings.m_rgbColor);

        QList<QString> settingsKeys({
            "rgbColor",
            "title",
            "useReverseAPI",
            "reverseAPIAddress",
            "reverseAPIPort",
            "reverseAPIDeviceIndex",
            "reverseAPIChannelIndex"
        });

        applySettings(settingsKeys);
    }

    resetContextMenuType();
}

void GS232ControllerGUI::on_startStop_toggled(bool checked)
{
    if (m_doApplySettings)
    {
        GS232Controller::MsgStartStop *message = GS232Controller::MsgStartStop::create(checked);
        m_gs232Controller->getInputMessageQueue()->push(message);
    }
}

void GS232ControllerGUI::setProtocol(GS232ControllerSettings::Protocol protocol)
{
    if (protocol == GS232ControllerSettings::GS232)
    {
        ui->precision->setValue(0);
        ui->precision->setEnabled(false);
        ui->precisionLabel->setEnabled(false);
        ui->lineEnding->setEnabled(true);
    }
    else if (protocol == GS232ControllerSettings::SPID)
    {
        ui->precision->setValue(1);
        ui->precision->setEnabled(false);
        ui->precisionLabel->setEnabled(false);
        ui->lineEnding->setEnabled(false);
    }
    else
    {
        ui->precision->setEnabled(true);
        ui->precisionLabel->setEnabled(true);
        ui->lineEnding->setEnabled(false);
    }
    bool dfm = protocol == GS232ControllerSettings::DFM;
    ui->dfmLine->setVisible(dfm);
    ui->dfmTrack->setVisible(dfm);
    ui->dfmLubePumps->setVisible(dfm);
    ui->dfmBrakes->setVisible(dfm);
    ui->dfmDrives->setVisible(dfm);
    ui->dfmShowStatus->setVisible(dfm);

    bool alpaca = protocol == GS232ControllerSettings::ALPACA;
    ui->park->setVisible(alpaca);
    ui->home->setVisible(alpaca);
    updateParkAndHomeControls();

    updateConnectionWidgets();

    // See RemoteControlGUI::createGUI() for additional weirdness in trying
    // to resize a window after widgets are changed
    getRollupContents()->arrangeRollups();
    layout()->activate(); // Recalculate sizeHint
    setMinimumSize(sizeHint());
    setMaximumSize(sizeHint());
    resize(sizeHint());
}

void GS232ControllerGUI::setPrecision()
{
    ui->coord1->setDecimals(m_settings.m_precision);
    ui->coord2->setDecimals(m_settings.m_precision);
    ui->tolerance->setDecimals(m_settings.m_precision);
    ui->azimuthOffset->setDecimals(m_settings.m_precision);
    ui->elevationOffset->setDecimals(m_settings.m_precision);
    double step = pow(10.0, -m_settings.m_precision);
    ui->coord1->setSingleStep(step);
    ui->coord2->setSingleStep(step);
    ui->tolerance->setSingleStep(step);
    ui->azimuthOffset->setSingleStep(step);
    ui->elevationOffset->setSingleStep(step);
}

void GS232ControllerGUI::updateParkAndHomeControls()
{
    const bool alpaca = m_settings.m_protocol == GS232ControllerSettings::ALPACA;
    const bool moving = m_slewingStateValid && m_slewing;

    bool oldParkState = ui->park->blockSignals(true);
    ui->park->setChecked(alpaca && m_parkStateValid && m_atPark);
    ui->park->blockSignals(oldParkState);
    ui->park->setEnabled(alpaca && m_parkStateValid && !moving && (m_atPark || m_canPark));

    bool oldHomeState = ui->home->blockSignals(true);
    ui->home->setChecked(alpaca && m_homeStateValid && m_atHome);
    ui->home->blockSignals(oldHomeState);
    ui->home->setEnabled(alpaca && m_homeStateValid && !moving && m_canFindHome);
}

void GS232ControllerGUI::handlePositionMismatch(const ControllerProtocol::MsgReportPositionMismatch& report)
{
    QMessageBox msgBox(this);
    msgBox.setIcon(QMessageBox::Question);
    msgBox.setWindowTitle(tr("Position mismatch"));
    msgBox.setText(tr("The rotator position differs from the current settings."));
    msgBox.setInformativeText(QString(
        "Rotator:\n"
        "  Latitude: %1\n"
        "  Longitude: %2\n"
        "  Elevation: %3 m\n"
        "Settings:\n"
        "  Latitude: %4\n"
        "  Longitude: %5\n"
        "  Elevation: %6 m\n")
        .arg(report.rotatorLatitude(), 0, 'f', 6)
        .arg(report.rotatorLongitude(), 0, 'f', 6)
        .arg(report.rotatorElevation(), 0, 'f', 1)
        .arg(report.localLatitude(), 0, 'f', 6)
        .arg(report.localLongitude(), 0, 'f', 6)
        .arg(report.localElevation(), 0, 'f', 1));

    QPushButton *updaterotator = msgBox.addButton(tr("Update rotator"), QMessageBox::ActionRole);
    QPushButton *updateSettings = msgBox.addButton(tr("Update settings"), QMessageBox::ActionRole);
    msgBox.addButton(tr("No change"), QMessageBox::RejectRole);
    msgBox.setDefaultButton(updaterotator);
    msgBox.exec();

    if (msgBox.clickedButton() == updaterotator)
    {
        m_gs232Controller->getInputMessageQueue()->push(GS232Controller::MsgSetPosition::create(
            report.localLatitude(),
            report.localLongitude(),
            report.localElevation()));
    }
    else if (msgBox.clickedButton() == updateSettings)
    {
        ui->latitude->setValue(report.rotatorLatitude());
        ui->longitude->setValue(report.rotatorLongitude());
        ui->altitude->setValue(report.rotatorElevation());
    }
}

void GS232ControllerGUI::handleDateTimeMismatch(const ControllerProtocol::MsgReportDateTimeMismatch& report)
{
    const QString rotatorUtc = report.rotatorUtcDate().toUTC().toString(Qt::ISODateWithMs);
    const QString localUtc = report.localUtcDate().toUTC().toString(Qt::ISODateWithMs);

    QMessageBox msgBox(this);
    msgBox.setIcon(QMessageBox::Question);
    msgBox.setWindowTitle(tr("Date & time mismatch"));
    msgBox.setText(tr("The rotator date & time differs from the local system clock."));
    msgBox.setInformativeText(QString(
        "Rotator:\n"
        "  UTC date: %1\n\n"
        "Local:\n"
        "  UTC date: %2\n\n")
        .arg(rotatorUtc)
        .arg(localUtc));

    QPushButton *updaterotator = msgBox.addButton(tr("Update rotator"), QMessageBox::ActionRole);
    msgBox.addButton(tr("No change"), QMessageBox::RejectRole);
    msgBox.setDefaultButton(updaterotator);
    msgBox.exec();

    if (msgBox.clickedButton() == updaterotator)
    {
        m_gs232Controller->getInputMessageQueue()->push(GS232Controller::MsgSetDateTime::create(
            QDateTime::currentDateTimeUtc()));
    }
}

void GS232ControllerGUI::on_protocol_currentIndexChanged(int index)
{
    m_settings.m_protocol = (GS232ControllerSettings::Protocol)index;
    setProtocol(m_settings.m_protocol);
    applySetting("protocol");
}

void GS232ControllerGUI::on_connection_currentIndexChanged(int index)
{
    m_settings.m_connection = (GS232ControllerSettings::Connection)index;
    applySetting("connection");
    updateConnectionWidgets();
}

void GS232ControllerGUI::on_serialPort_currentIndexChanged(int index)
{
    (void) index;
    m_settings.m_serialPort = ui->serialPort->currentText();
    applySetting("serialPort");
}

void GS232ControllerGUI::on_baudRate_currentIndexChanged(int index)
{
    (void) index;
    m_settings.m_baudRate = ui->baudRate->currentText().toInt();
    applySetting("baudRate");
}

void GS232ControllerGUI::on_host_editingFinished()
{
    m_settings.m_host = ui->host->text();
    applySetting("host");
}

void GS232ControllerGUI::on_port_valueChanged(int value)
{
    m_settings.m_port = value;
    applySetting("port");
}

void GS232ControllerGUI::on_coord1_valueChanged(double value)
{
    if (!m_inputUpdate) {
        m_inputCoord1 = value;
    }
    displayToAzEl(value, ui->coord2->value());
    ui->targetName->setText("");
}

void GS232ControllerGUI::on_coord2_valueChanged(double value)
{
    if (!m_inputUpdate) {
        m_inputCoord2 = value;
    }
    displayToAzEl(ui->coord1->value(), value);
    ui->targetName->setText("");
}

void GS232ControllerGUI::on_azimuthOffset_valueChanged(double value)
{
    if (!m_inputUpdate) {
        m_inputAzOffset = value;
    }
    m_settings.m_azimuthOffset = (float) value;
    applySetting("azimuthOffset");
}

void GS232ControllerGUI::on_elevationOffset_valueChanged(double value)
{
    if (!m_inputUpdate) {
        m_inputElOffset = value;
    }
    m_settings.m_elevationOffset = (float) value;
    applySetting("elevationOffset");
}

void GS232ControllerGUI::on_azimuthMin_valueChanged(int value)
{
    m_settings.m_azimuthMin = value;
    applySetting("azimuthMin");
}

void GS232ControllerGUI::on_azimuthMax_valueChanged(int value)
{
    m_settings.m_azimuthMax = value;
    applySetting("azimuthMax");
}

void GS232ControllerGUI::on_elevationMin_valueChanged(int value)
{
    m_settings.m_elevationMin = value;
    applySetting("elevationMin");
}

void GS232ControllerGUI::on_elevationMax_valueChanged(int value)
{
    m_settings.m_elevationMax = value;
    applySetting("elevationMax");
}

void GS232ControllerGUI::on_tolerance_valueChanged(double value)
{
    m_settings.m_tolerance = value;
    applySetting("tolerance");
}

void GS232ControllerGUI::on_precision_valueChanged(int value)
{
    m_settings.m_precision = value;
    setPrecision();
    applySetting("precision");
}

void GS232ControllerGUI::on_coordinates_currentIndexChanged(int index)
{
    m_settings.m_coordinates = (GS232ControllerSettings::Coordinates)index;
    applySetting("coordinates");

    float coord1, coord2;
    azElToDisplay(m_settings.m_azimuth, m_settings.m_elevation, coord1, coord2);

    ui->coord1->blockSignals(true);
    if (m_settings.m_coordinates == GS232ControllerSettings::AZ_EL)
    {
        ui->coord1->setMinimum(0.0);
        ui->coord1->setMaximum(450.0);
        ui->coord1->setToolTip("Target azimuth in degrees");
        ui->coord1Label->setText("Azimuth");
        ui->coord1CurrentText->setToolTip("Current azimuth in degrees");
    }
    else
    {
        ui->coord1->setMinimum(-90.0);
        ui->coord1->setMaximum(90.0);
        ui->coord1->setToolTip("Target X in degrees");
        ui->coord1Label->setText("X");
        ui->coord1CurrentText->setToolTip("Current X coordinate in degrees");
    }
    ui->coord1->setValue(coord1);
    ui->coord1->blockSignals(false);
    ui->coord2->blockSignals(true);
    if (m_settings.m_coordinates == GS232ControllerSettings::AZ_EL)
    {
        ui->coord2->setMinimum(0.0);
        ui->coord2->setMaximum(180.0);
        ui->coord2->setToolTip("Target elevation in degrees");
        ui->coord2Label->setText("Elevation");
        ui->coord2CurrentText->setToolTip("Current elevation in degrees");
    }
    else
    {
        ui->coord2->setMinimum(-90.0);
        ui->coord2->setMaximum(90.0);
        ui->coord2->setToolTip("Target Y in degrees");
        ui->coord2Label->setText("Y");
        ui->coord2CurrentText->setToolTip("Current Y coordinate in degrees");
    }
    ui->coord2->setValue(coord2);
    ui->coord2->blockSignals(false);
}

void GS232ControllerGUI::on_track_stateChanged(int state)
{
    m_settings.m_track = state == Qt::Checked;
    ui->targetName->setEnabled(m_settings.m_track);
    ui->sources->setEnabled(m_settings.m_track);

    if (!m_settings.m_track) {
        ui->targetName->setText("");
    }

    applySetting("track");
}

void GS232ControllerGUI::on_sources_currentTextChanged(const QString& text)
{
    qDebug("GS232ControllerGUI::on_sources_currentTextChanged: %s", qPrintable(text));
    m_settings.m_source = text;
    ui->targetName->setText("");
    applySetting("source");
}

void GS232ControllerGUI::on_lineEnding_currentIndexChanged(int index)
{
    m_settings.m_lineEnding = (GS232ControllerSettings::LineEnding)index;
    applySetting("lineEnding");
}

void GS232ControllerGUI::on_latitude_valueChanged(double value)
{
    m_settings.m_latitude = (float) value;
    applySetting("latitude");
}

void GS232ControllerGUI::on_longitude_valueChanged(double value)
{
    m_settings.m_longitude = (float) value;
    applySetting("longitude");
}

void GS232ControllerGUI::on_altitude_valueChanged(double value)
{
    m_settings.m_altitude = (float) value;
    applySetting("altitude");
}

void GS232ControllerGUI::on_useMyPosition_clicked(bool checked)
{
    (void) checked;

    ui->latitude->setValue(MainCore::instance()->getSettings().getLatitude());
    ui->longitude->setValue(MainCore::instance()->getSettings().getLongitude());
    ui->altitude->setValue(MainCore::instance()->getSettings().getAltitude());
}

void GS232ControllerGUI::useMyPosition_rightClicked(const QPoint& p)
{
    (void) p;

    m_settings.m_positionSync = !m_settings.m_positionSync;
    applySetting("positionSync");
    applyPositionSync();
}

void GS232ControllerGUI::applyPositionSync()
{
    if (m_settings.m_positionSync) {
        ui->useMyPosition->setStyleSheet(QString("QToolButton{ background-color: %1; }").arg(palette().highlight().color().darker(150).name()));
    } else {
        ui->useMyPosition->setStyleSheet(QString("QToolButton{ background-color: %1; }").arg(palette().button().color().name()));
    }

    ui->latitude->setReadOnly(m_settings.m_positionSync);
    ui->longitude->setReadOnly(m_settings.m_positionSync);
    ui->altitude->setReadOnly(m_settings.m_positionSync);
    if (m_settings.m_positionSync)
    {
        on_useMyPosition_clicked(true);
        connect(&MainCore::instance()->getSettings(), &MainSettings::preferenceChanged, this, &GS232ControllerGUI::preferenceChanged);
    }
    else
    {
        disconnect(&MainCore::instance()->getSettings(), &MainSettings::preferenceChanged, this, &GS232ControllerGUI::preferenceChanged);
    }
}

void GS232ControllerGUI::preferenceChanged(int elementType)
{
    Preferences::ElementType pref = (Preferences::ElementType)elementType;
    if (pref == Preferences::Latitude) {
        ui->latitude->setValue(MainCore::instance()->getSettings().getLatitude());
    }
    if (pref == Preferences::Longitude) {
        ui->longitude->setValue(MainCore::instance()->getSettings().getLongitude());
    }
    if (pref == Preferences::Altitude) {
        ui->altitude->setValue(MainCore::instance()->getSettings().getAltitude());
    }
}

void GS232ControllerGUI::on_dfmTrack_clicked(bool checked)
{
    m_settings.m_dfmTrackOn = checked;
    applySetting("dfmTrackOn");
}

void GS232ControllerGUI::on_dfmLubePumps_clicked(bool checked)
{
    m_settings.m_dfmLubePumpsOn = checked;
    applySetting("dfmLubePumpsOn");
}

void GS232ControllerGUI::on_dfmBrakes_clicked(bool checked)
{
    m_settings.m_dfmBrakesOn = checked;
    applySetting("dfmBrakesOn");
}

void GS232ControllerGUI::on_dfmDrives_clicked(bool checked)
{
    m_settings.m_dfmDrivesOn = checked;
    applySetting("dfmDrivesOn");
}

void GS232ControllerGUI::on_dfmShowStatus_clicked()
{
    m_dfmStatusDialog.show();
    m_dfmStatusDialog.raise();
    m_dfmStatusDialog.activateWindow();
}

void GS232ControllerGUI::on_park_toggled(bool checked)
{
    if (checked) {
        m_gs232Controller->getInputMessageQueue()->push(GS232Controller::MsgPark::create());
    } else {
        m_gs232Controller->getInputMessageQueue()->push(GS232Controller::MsgUnpark::create());
    }
}

void GS232ControllerGUI::on_home_clicked(bool checked)
{
    (void) checked;

    if (m_canFindHome) {
        m_gs232Controller->getInputMessageQueue()->push(GS232Controller::MsgHome::create());
    }

    updateParkAndHomeControls();
}

void GS232ControllerGUI::updateStatus()
{
    int state = m_gs232Controller->getState();
    bool onTarget = m_gs232Controller->getOnTarget();

    if (m_lastFeatureState != state)
    {
        // We set checked state of start/stop button, in case it was changed via API
        bool oldState;
        switch (state)
        {
            case Feature::StNotStarted:
                ui->startStop->setStyleSheet("QToolButton { background:rgb(79,79,79); }");
                break;
            case Feature::StIdle:
                oldState = ui->startStop->blockSignals(true);
                ui->startStop->setChecked(false);
                ui->startStop->blockSignals(oldState);
                ui->startStop->setStyleSheet("QToolButton { background-color : blue; }");
                break;
            case Feature::StRunning:
                oldState = ui->startStop->blockSignals(true);
                ui->startStop->setChecked(true);
                ui->startStop->blockSignals(oldState);
                if (onTarget) {
                    ui->startStop->setStyleSheet("QToolButton { background-color : green; }");
                } else {
                    ui->startStop->setStyleSheet("QToolButton { background-color : yellow; }");
                }
                m_lastOnTarget = onTarget;
                break;
            case Feature::StError:
                ui->startStop->setStyleSheet("QToolButton { background-color : red; }");
                QMessageBox::critical(this, m_settings.m_title, m_gs232Controller->getErrorMessage());
                break;
            default:
                break;
        }

        m_lastFeatureState = state;
    }
    else if (state == Feature::StRunning)
    {
        if (onTarget != m_lastOnTarget)
        {
            if (onTarget) {
                ui->startStop->setStyleSheet("QToolButton { background-color : green; }");
            } else {
                ui->startStop->setStyleSheet("QToolButton { background-color : yellow; }");
            }
        }
        m_lastOnTarget = onTarget;
    }
}

void GS232ControllerGUI::applySetting(const QString& settingsKey)
{
    applySettings({settingsKey});
}

void GS232ControllerGUI::applySettings(const QStringList& settingsKeys, bool force)
{
    m_settingsKeys.append(settingsKeys);
    if (m_doApplySettings)
    {
        GS232Controller::MsgConfigureGS232Controller* message = GS232Controller::MsgConfigureGS232Controller::create(m_settings, m_settingsKeys, force);
        m_gs232Controller->getInputMessageQueue()->push(message);
        m_settingsKeys.clear();
    }
}

void GS232ControllerGUI::applyAllSettings()
{
    applySettings(QStringList(), true);
}

void GS232ControllerGUI::makeUIConnections()
{
    QObject::connect(ui->startStop, &ButtonSwitch::toggled, this, &GS232ControllerGUI::on_startStop_toggled);
    QObject::connect(ui->protocol, qOverload<int>(&QComboBox::currentIndexChanged), this, &GS232ControllerGUI::on_protocol_currentIndexChanged);
    QObject::connect(ui->connection, qOverload<int>(&QComboBox::currentIndexChanged), this, &GS232ControllerGUI::on_connection_currentIndexChanged);
    QObject::connect(ui->serialPort, qOverload<int>(&QComboBox::currentIndexChanged), this, &GS232ControllerGUI::on_serialPort_currentIndexChanged);
    QObject::connect(ui->host, &QLineEdit::editingFinished, this, &GS232ControllerGUI::on_host_editingFinished);
    QObject::connect(ui->port, qOverload<int>(&QSpinBox::valueChanged), this, &GS232ControllerGUI::on_port_valueChanged);
    QObject::connect(ui->baudRate, qOverload<int>(&QComboBox::currentIndexChanged), this, &GS232ControllerGUI::on_baudRate_currentIndexChanged);
    QObject::connect(ui->track, &QCheckBox::stateChanged, this, &GS232ControllerGUI::on_track_stateChanged);
    QObject::connect(ui->coord1, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_coord1_valueChanged);
    QObject::connect(ui->coord2, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_coord2_valueChanged);
    QObject::connect(ui->sources, &QComboBox::currentTextChanged, this, &GS232ControllerGUI::on_sources_currentTextChanged);
    QObject::connect(ui->azimuthOffset, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_azimuthOffset_valueChanged);
    QObject::connect(ui->elevationOffset, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_elevationOffset_valueChanged);
    QObject::connect(ui->azimuthMin, qOverload<int>(&QSpinBox::valueChanged), this, &GS232ControllerGUI::on_azimuthMin_valueChanged);
    QObject::connect(ui->azimuthMax, qOverload<int>(&QSpinBox::valueChanged), this, &GS232ControllerGUI::on_azimuthMax_valueChanged);
    QObject::connect(ui->elevationMin, qOverload<int>(&QSpinBox::valueChanged), this, &GS232ControllerGUI::on_elevationMin_valueChanged);
    QObject::connect(ui->elevationMax, qOverload<int>(&QSpinBox::valueChanged), this, &GS232ControllerGUI::on_elevationMax_valueChanged);
    QObject::connect(ui->tolerance, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_tolerance_valueChanged);
    QObject::connect(ui->precision, qOverload<int>(&QSpinBox::valueChanged), this, &GS232ControllerGUI::on_precision_valueChanged);
    QObject::connect(ui->coordinates, qOverload<int>(&QComboBox::currentIndexChanged), this, &GS232ControllerGUI::on_coordinates_currentIndexChanged);
    QObject::connect(ui->inputController, qOverload<int>(&QComboBox::currentIndexChanged), this, &GS232ControllerGUI::on_inputController_currentIndexChanged);
    QObject::connect(ui->inputConfigure, &QToolButton::clicked, this, &GS232ControllerGUI::on_inputConfigure_clicked);
    QObject::connect(ui->highSensitivity, &QToolButton::clicked, this, &GS232ControllerGUI::on_highSensitivity_clicked);
    QObject::connect(ui->enableTargetControl, &QToolButton::clicked, this, &GS232ControllerGUI::on_enableTargetControl_clicked);
    QObject::connect(ui->enableOffsetControl, &QToolButton::clicked, this, &GS232ControllerGUI::on_enableOffsetControl_clicked);
    QObject::connect(ui->lineEnding, qOverload<int>(&QComboBox::currentIndexChanged), this, &GS232ControllerGUI::on_lineEnding_currentIndexChanged);
    QObject::connect(ui->latitude, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_latitude_valueChanged);
    QObject::connect(ui->longitude, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_longitude_valueChanged);
    QObject::connect(ui->altitude, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &GS232ControllerGUI::on_altitude_valueChanged);
    QObject::connect(ui->useMyPosition, &ButtonSwitch::clicked, this, &GS232ControllerGUI::on_useMyPosition_clicked);
    QObject::connect(ui->dfmTrack, &QToolButton::toggled, this, &GS232ControllerGUI::on_dfmTrack_clicked);
    QObject::connect(ui->dfmLubePumps, &QToolButton::toggled, this, &GS232ControllerGUI::on_dfmLubePumps_clicked);
    QObject::connect(ui->dfmBrakes, &QToolButton::toggled, this, &GS232ControllerGUI::on_dfmBrakes_clicked);
    QObject::connect(ui->dfmDrives, &QToolButton::toggled, this, &GS232ControllerGUI::on_dfmDrives_clicked);
    QObject::connect(ui->dfmShowStatus, &QToolButton::clicked, this, &GS232ControllerGUI::on_dfmShowStatus_clicked);
    QObject::connect(ui->park, &ButtonSwitch::toggled, this, &GS232ControllerGUI::on_park_toggled);
    QObject::connect(ui->home, &ButtonSwitch::clicked, this, &GS232ControllerGUI::on_home_clicked);
}
