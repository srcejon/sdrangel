///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Edouard Griffiths, F4EXB <f4exb06@gmail.com>               //
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

#include <QFileDialog>
#include <QPixmap>

#include "feature/featureuiset.h"

#include "ui_cameragui.h"
#include "camera.h"
#include "cameraworker.h"
#include "cameragui.h"

CameraGUI* CameraGUI::create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature)
{
    return new CameraGUI(pluginAPI, featureUISet, feature);
}

void CameraGUI::destroy()
{
    delete this;
}

void CameraGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applySettings(true);
}

QByteArray CameraGUI::serialize() const
{
    return m_settings.serialize();
}

bool CameraGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        m_feature->setWorkspaceIndex(m_settings.m_workspaceIndex);
        displaySettings();
        applySettings(true);
        return true;
    }

    resetToDefaults();
    return false;
}

bool CameraGUI::handleMessage(const Message& message)
{
    if (Camera::MsgConfigureCamera::match(message))
    {
        const Camera::MsgConfigureCamera& cfg = (Camera::MsgConfigureCamera&) message;

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
    else if (CameraWorker::MsgReportCameraList::match(message))
    {
        const CameraWorker::MsgReportCameraList& report = (CameraWorker::MsgReportCameraList&) message;
        const QString current = ui->cameraCombo->currentText();

        ui->cameraCombo->blockSignals(true);
        ui->cameraCombo->clear();
        ui->cameraCombo->addItems(report.getCameraIds());

        const int index = ui->cameraCombo->findText(current);

        if (index >= 0) {
            ui->cameraCombo->setCurrentIndex(index);
        } else if (!m_settings.m_cameraId.isEmpty()) {
            ui->cameraCombo->setCurrentText(m_settings.m_cameraId);
        }

        ui->cameraCombo->blockSignals(false);

        return true;
    }
    else if (CameraWorker::MsgReportFrame::match(message))
    {
        const CameraWorker::MsgReportFrame& report = (CameraWorker::MsgReportFrame&) message;
        m_lastImage = report.getImage();
        updateImageWidget();
        return true;
    }

    return false;
}

void CameraGUI::handleInputMessages()
{
    Message* message;

    while ((message = getInputMessageQueue()->pop()))
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

CameraGUI::CameraGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent) :
    FeatureGUI(parent),
    ui(new Ui::CameraGUI),
    m_pluginAPI(pluginAPI),
    m_featureUISet(featureUISet),
    m_doApplySettings(true)
{
    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/camera/readme.md";

    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    rollupContents->arrangeRollups();

    m_camera = reinterpret_cast<Camera*>(feature);
    m_camera->setMessageQueueToGUI(&m_inputMessageQueue);

    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    m_settings.setRollupState(&m_rollupState);

    displaySettings();
    applySettings(true);
    m_resizer.enableChildMouseTracking();
}

CameraGUI::~CameraGUI()
{
    delete ui;
}

void CameraGUI::setWorkspaceIndex(int index)
{
    m_settings.m_workspaceIndex = index;
    m_feature->setWorkspaceIndex(index);
}

void CameraGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void CameraGUI::displaySettings()
{
    setWindowTitle(m_settings.m_title);
    setTitle(m_settings.m_title);

    ui->startStop->setChecked(m_settings.m_captureActive);
    ui->apiCombo->setCurrentIndex(static_cast<int>(m_settings.m_cameraAPI));
    ui->cameraCombo->setCurrentText(m_settings.m_cameraId);
    ui->resolutionWidth->setValue(m_settings.m_resolutionWidth);
    ui->resolutionHeight->setValue(m_settings.m_resolutionHeight);
    ui->fpsSpin->setValue(m_settings.m_framesPerSecond);
    ui->exposureSpin->setValue(m_settings.m_exposureTimeMs);
    ui->isoSpin->setValue(m_settings.m_isoSensitivity);
    ui->alpacaHostEdit->setText(m_settings.m_alpacaHost);
    ui->alpacaPortSpin->setValue(m_settings.m_alpacaPort);
    ui->alpacaCameraIdSpin->setValue(m_settings.m_alpacaCameraId);
    ui->saveImageCheck->setChecked(m_settings.m_saveImage);
    ui->imagePathEdit->setText(m_settings.m_imageFileName);
    ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
    ui->videoPathEdit->setText(m_settings.m_videoFileName);
}

void CameraGUI::applySettings(bool force)
{
    if (!m_doApplySettings) {
        return;
    }

    Camera::MsgConfigureCamera *msg = Camera::MsgConfigureCamera::create(m_settings, m_settingsKeys, force);
    m_camera->getInputMessageQueue()->push(msg);

    m_settingsKeys.clear();
}

void CameraGUI::updateImageWidget()
{
    if (m_lastImage.isNull()) {
        return;
    }

    ui->imageLabel->setPixmap(QPixmap::fromImage(m_lastImage).scaled(
        ui->imageLabel->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation
    ));
}

void CameraGUI::on_startStop_clicked(bool checked)
{
    m_settings.m_captureActive = checked;
    m_settingsKeys.append("captureActive");
    applySettings();
    m_camera->getInputMessageQueue()->push(Camera::MsgStartStop::create(checked));
}

void CameraGUI::on_refreshCamerasButton_clicked()
{
    m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create());
}

void CameraGUI::on_apiCombo_currentIndexChanged(int index)
{
    m_settings.m_cameraAPI = static_cast<CameraSettings::CameraAPI>(index);
    m_settingsKeys.append("cameraAPI");
    applySettings();
}

void CameraGUI::on_cameraCombo_currentTextChanged(const QString& text)
{
    m_settings.m_cameraId = text;
    m_settingsKeys.append("cameraId");
    applySettings();
}

void CameraGUI::on_resolutionWidth_valueChanged(int value)
{
    m_settings.m_resolutionWidth = value;
    m_settingsKeys.append("resolutionWidth");
    applySettings();
}

void CameraGUI::on_resolutionHeight_valueChanged(int value)
{
    m_settings.m_resolutionHeight = value;
    m_settingsKeys.append("resolutionHeight");
    applySettings();
}

void CameraGUI::on_fpsSpin_valueChanged(int value)
{
    m_settings.m_framesPerSecond = value;
    m_settingsKeys.append("framesPerSecond");
    applySettings();
}

void CameraGUI::on_exposureSpin_valueChanged(int value)
{
    m_settings.m_exposureTimeMs = value;
    m_settingsKeys.append("exposureTimeMs");
    applySettings();
}

void CameraGUI::on_isoSpin_valueChanged(int value)
{
    m_settings.m_isoSensitivity = value;
    m_settingsKeys.append("isoSensitivity");
    applySettings();
}

void CameraGUI::on_alpacaHostEdit_editingFinished()
{
    m_settings.m_alpacaHost = ui->alpacaHostEdit->text();
    m_settingsKeys.append("alpacaHost");
    applySettings();
}

void CameraGUI::on_alpacaPortSpin_valueChanged(int value)
{
    m_settings.m_alpacaPort = static_cast<uint16_t>(value);
    m_settingsKeys.append("alpacaPort");
    applySettings();
}

void CameraGUI::on_alpacaCameraIdSpin_valueChanged(int value)
{
    m_settings.m_alpacaCameraId = value;
    m_settingsKeys.append("alpacaCameraId");
    applySettings();
}

void CameraGUI::on_saveImageCheck_toggled(bool checked)
{
    m_settings.m_saveImage = checked;
    m_settingsKeys.append("saveImage");
    applySettings();
}

void CameraGUI::on_imagePathEdit_editingFinished()
{
    m_settings.m_imageFileName = ui->imagePathEdit->text();
    m_settingsKeys.append("imageFileName");
    applySettings();
}

void CameraGUI::on_imagePathButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Save JPEG"), m_settings.m_imageFileName, tr("JPEG image (*.jpg *.jpeg)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_imageFileName = fileName;
        ui->imagePathEdit->setText(fileName);
        m_settingsKeys.append("imageFileName");
        applySettings();
    }
}

void CameraGUI::on_saveVideoCheck_toggled(bool checked)
{
    m_settings.m_saveVideo = checked;
    m_settingsKeys.append("saveVideo");
    applySettings();
}

void CameraGUI::on_videoPathEdit_editingFinished()
{
    m_settings.m_videoFileName = ui->videoPathEdit->text();
    m_settingsKeys.append("videoFileName");
    applySettings();
}

void CameraGUI::on_videoPathButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Save MP4"), m_settings.m_videoFileName, tr("MP4 video (*.mp4)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_videoFileName = fileName;
        ui->videoPathEdit->setText(fileName);
        m_settingsKeys.append("videoFileName");
        applySettings();
    }
}
