///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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
    else if (CameraWorker::MsgReportResolutions::match(message))
    {
        const CameraWorker::MsgReportResolutions& report = (CameraWorker::MsgReportResolutions&) message;
        const QString current = QString("%1x%2").arg(m_settings.m_resolutionWidth).arg(m_settings.m_resolutionHeight);

        ui->resolutionCombo->blockSignals(true);
        ui->resolutionCombo->clear();

        for (const QSize& size : report.getResolutions()) {
            ui->resolutionCombo->addItem(QString("%1x%2").arg(size.width()).arg(size.height()));
        }

        const int idx = ui->resolutionCombo->findText(current);
        if (idx >= 0) {
            ui->resolutionCombo->setCurrentIndex(idx);
        } else if (ui->resolutionCombo->count() > 0) {
            ui->resolutionCombo->setCurrentIndex(0);
        }

        ui->resolutionCombo->blockSignals(false);
        return true;
    }
    else if (CameraWorker::MsgReportFrame::match(message))
    {
        const CameraWorker::MsgReportFrame& report = (CameraWorker::MsgReportFrame&) message;
        m_lastImage = report.getImage();
        updateImageWidget();
        return true;
    }
    else if (CameraWorker::MsgReportAlpacaCameraInfo::match(message))
    {
        const CameraWorker::MsgReportAlpacaCameraInfo& info = (CameraWorker::MsgReportAlpacaCameraInfo&) message;
        updateAlpacaCapabilities(info);
        return true;
    }
    else if (CameraWorker::MsgReportAlpacaStatus::match(message))
    {
        const CameraWorker::MsgReportAlpacaStatus& status = (CameraWorker::MsgReportAlpacaStatus&) message;

        static const QStringList cameraStateNames = {
            "Idle", "Waiting", "Exposing", "Reading", "Download", "Error"
        };
        const int cs = status.getCameraState();
        ui->cameraStateLabel->setText((cs >= 0 && cs < cameraStateNames.size()) ? cameraStateNames[cs] : (cs >= 0 ? QString::number(cs) : "-"));

        if (status.isCcdTemperatureValid()) {
            ui->ccdTempLabel->setText(QString::number(status.getCcdTemperature(), 'f', 1));
        }

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
    m_doApplySettings(true),
    m_alpacaHasNamedGains(false)
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
    makeUIConnections();
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

    const QString resText = QString("%1x%2").arg(m_settings.m_resolutionWidth).arg(m_settings.m_resolutionHeight);
    const int resIdx = ui->resolutionCombo->findText(resText);
    if (resIdx >= 0) {
        ui->resolutionCombo->setCurrentIndex(resIdx);
    }

    ui->fpsSpin->setValue(m_settings.m_framesPerSecond);
    ui->exposureSpin->setValue(m_settings.m_exposureTimeMs);
    ui->isoSpin->setValue(m_settings.m_isoSensitivity);
    ui->alpacaHostEdit->setText(m_settings.m_alpacaHost);
    ui->alpacaPortSpin->setValue(m_settings.m_alpacaPort);
    ui->alpacaCameraIdSpin->setValue(m_settings.m_alpacaCameraId);
    ui->alpacaBinXSpin->setValue(m_settings.m_alpacaBinX);
    ui->alpacaBinYSpin->setValue(m_settings.m_alpacaBinY);

    if (m_alpacaHasNamedGains) {
        ui->alpacaGainCombo->setCurrentIndex(m_settings.m_alpacaGain >= 0 ? m_settings.m_alpacaGain : 0);
    } else {
        ui->alpacaGainSpin->setValue(m_settings.m_alpacaGain >= 0 ? m_settings.m_alpacaGain : 0);
    }

    ui->alpacaReadoutModeCombo->setCurrentIndex(m_settings.m_alpacaReadoutMode);
    ui->saveImageCheck->setChecked(m_settings.m_saveImage);
    ui->imagePathEdit->setText(m_settings.m_imageFileName);
    ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
    ui->videoPathEdit->setText(m_settings.m_videoFileName);
    updateAlpacaVisibility();
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

void CameraGUI::makeUIConnections()
{
    QObject::connect(ui->startStop, &QPushButton::clicked, this, &CameraGUI::on_startStop_clicked);
    QObject::connect(ui->refreshCamerasButton, &QPushButton::clicked, this, &CameraGUI::on_refreshCamerasButton_clicked);
    QObject::connect(ui->apiCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_apiCombo_currentIndexChanged);
    QObject::connect(ui->cameraCombo, &QComboBox::currentTextChanged, this, &CameraGUI::on_cameraCombo_currentTextChanged);
    QObject::connect(ui->resolutionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_resolutionCombo_currentIndexChanged);
    QObject::connect(ui->fpsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_fpsSpin_valueChanged);
    QObject::connect(ui->exposureSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_exposureSpin_valueChanged);
    QObject::connect(ui->isoSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_isoSpin_valueChanged);
    QObject::connect(ui->alpacaHostEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_alpacaHostEdit_editingFinished);
    QObject::connect(ui->alpacaPortSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaPortSpin_valueChanged);
    QObject::connect(ui->alpacaCameraIdSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaCameraIdSpin_valueChanged);
    QObject::connect(ui->alpacaBinXSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaBinXSpin_valueChanged);
    QObject::connect(ui->alpacaBinYSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaBinYSpin_valueChanged);
    QObject::connect(ui->alpacaGainCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaGainCombo_currentIndexChanged);
    QObject::connect(ui->alpacaGainSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaGainSpin_valueChanged);
    QObject::connect(ui->alpacaReadoutModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaReadoutModeCombo_currentIndexChanged);
    QObject::connect(ui->saveImageCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveImageCheck_toggled);
    QObject::connect(ui->imagePathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_imagePathEdit_editingFinished);
    QObject::connect(ui->imagePathButton, &QPushButton::clicked, this, &CameraGUI::on_imagePathButton_clicked);
    QObject::connect(ui->saveVideoCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveVideoCheck_toggled);
    QObject::connect(ui->videoPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_videoPathEdit_editingFinished);
    QObject::connect(ui->videoPathButton, &QPushButton::clicked, this, &CameraGUI::on_videoPathButton_clicked);
}

void CameraGUI::updateAlpacaVisibility()
{
    const bool alpaca = (m_settings.m_cameraAPI == CameraSettings::CameraAPIAlpaca);

    ui->resolutionLabel->setVisible(!alpaca);
    ui->resolutionCombo->setVisible(!alpaca);
    ui->alpacaHostLabel->setVisible(alpaca);
    ui->alpacaHostEdit->setVisible(alpaca);
    ui->alpacaPortLabel->setVisible(alpaca);
    ui->alpacaPortSpin->setVisible(alpaca);
    ui->alpacaCameraIdLabel->setVisible(alpaca);
    ui->alpacaCameraIdSpin->setVisible(alpaca);
    ui->alpacaBinXLabel->setVisible(alpaca);
    ui->alpacaBinXSpin->setVisible(alpaca);
    ui->alpacaBinYLabel->setVisible(alpaca);
    ui->alpacaBinYSpin->setVisible(alpaca);
    ui->alpacaGainLabel->setVisible(alpaca);
    ui->alpacaGainCombo->setVisible(alpaca && m_alpacaHasNamedGains);
    ui->alpacaGainSpin->setVisible(alpaca && !m_alpacaHasNamedGains);
    ui->alpacaReadoutModeLabel->setVisible(alpaca);
    ui->alpacaReadoutModeCombo->setVisible(alpaca);
    ui->alpacaStatusGroup->setVisible(alpaca);

    // Video saving only makes sense for Qt camera
    ui->saveVideoCheck->setVisible(!alpaca);
    ui->videoPathEdit->setVisible(!alpaca);
    ui->videoPathButton->setVisible(!alpaca);
}


void CameraGUI::updateAlpacaCapabilities(const CameraWorker::MsgReportAlpacaCameraInfo& info)
{
    blockApplySettings(true);

    // Bin X
    ui->alpacaBinXSpin->setMaximum(std::max(1, info.getMaxBinX()));
    ui->alpacaBinXSpin->setValue(qBound(1, m_settings.m_alpacaBinX, info.getMaxBinX()));

    // Bin Y
    ui->alpacaBinYSpin->setMaximum(std::max(1, info.getMaxBinY()));
    ui->alpacaBinYSpin->setValue(qBound(1, m_settings.m_alpacaBinY, info.getMaxBinY()));

    // Gain
    m_alpacaHasNamedGains = !info.getGains().isEmpty();
    if (m_alpacaHasNamedGains)
    {
        ui->alpacaGainCombo->blockSignals(true);
        ui->alpacaGainCombo->clear();
        ui->alpacaGainCombo->addItems(info.getGains());
        const int gainIdx = (m_settings.m_alpacaGain >= 0 && m_settings.m_alpacaGain < info.getGains().size())
            ? m_settings.m_alpacaGain : 0;
        ui->alpacaGainCombo->setCurrentIndex(gainIdx);
        ui->alpacaGainCombo->blockSignals(false);
    }
    else
    {
        ui->alpacaGainSpin->setMinimum(info.getGainMin());
        ui->alpacaGainSpin->setMaximum(std::max(info.getGainMin(), info.getGainMax()));
        const int gainVal = (m_settings.m_alpacaGain >= 0) ? m_settings.m_alpacaGain : info.getGainMin();
        ui->alpacaGainSpin->setValue(qBound(info.getGainMin(), gainVal, info.getGainMax()));
    }

    // Readout mode
    ui->alpacaReadoutModeCombo->blockSignals(true);
    ui->alpacaReadoutModeCombo->clear();
    ui->alpacaReadoutModeCombo->addItems(info.getReadoutModes());
    if (m_settings.m_alpacaReadoutMode < info.getReadoutModes().size()) {
        ui->alpacaReadoutModeCombo->setCurrentIndex(m_settings.m_alpacaReadoutMode);
    }
    ui->alpacaReadoutModeCombo->blockSignals(false);

    // Status labels
    ui->sensorNameLabel->setText(info.getSensorName().isEmpty() ? "-" : info.getSensorName());

    static const QStringList sensorTypeNames = {
        "Monochrome", "Colour", "RGGB", "CMYG", "CMYG2", "LRGB"
    };
    const int st = info.getSensorType();
    ui->sensorTypeLabel->setText((st >= 0 && st < sensorTypeNames.size()) ? sensorTypeNames[st] : QString::number(st));

    if (info.getPixelSizeX() > 0 || info.getPixelSizeY() > 0) {
        ui->pixelSizeLabel->setText(QString("%1 × %2")
            .arg(info.getPixelSizeX(), 0, 'f', 2)
            .arg(info.getPixelSizeY(), 0, 'f', 2));
    } else {
        ui->pixelSizeLabel->setText("-");
    }

    if (info.getCameraSizeX() > 0 || info.getCameraSizeY() > 0) {
        ui->cameraSizeLabel->setText(QString("%1 × %2").arg(info.getCameraSizeX()).arg(info.getCameraSizeY()));
    } else {
        ui->cameraSizeLabel->setText("-");
    }

    if (info.isCcdTemperatureValid()) {
        ui->ccdTempLabel->setText(QString::number(info.getCcdTemperature(), 'f', 1));
    } else {
        ui->ccdTempLabel->setText("-");
    }

    updateAlpacaVisibility();
    blockApplySettings(false);
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
    updateAlpacaVisibility();
    applySettings();
}

void CameraGUI::on_cameraCombo_currentTextChanged(const QString& text)
{
    m_settings.m_cameraId = text;
    m_settingsKeys.append("cameraId");
    applySettings();
}

void CameraGUI::on_resolutionCombo_currentIndexChanged(int index)
{
    (void) index;
    const QStringList parts = ui->resolutionCombo->currentText().split('x');

    if (parts.size() == 2)
    {
        bool ok1 = false, ok2 = false;
        const int width = parts[0].trimmed().toInt(&ok1);
        const int height = parts[1].trimmed().toInt(&ok2);

        if (ok1 && ok2 && width > 0 && height > 0)
        {
            m_settings.m_resolutionWidth = width;
            m_settings.m_resolutionHeight = height;
            m_settingsKeys.append("resolutionWidth");
            m_settingsKeys.append("resolutionHeight");
            applySettings();
        }
    }
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

void CameraGUI::on_alpacaBinXSpin_valueChanged(int value)
{
    m_settings.m_alpacaBinX = value;
    m_settingsKeys.append("alpacaBinX");
    applySettings();
}

void CameraGUI::on_alpacaBinYSpin_valueChanged(int value)
{
    m_settings.m_alpacaBinY = value;
    m_settingsKeys.append("alpacaBinY");
    applySettings();
}

void CameraGUI::on_alpacaGainCombo_currentIndexChanged(int index)
{
    m_settings.m_alpacaGain = index;
    m_settingsKeys.append("alpacaGain");
    applySettings();
}

void CameraGUI::on_alpacaGainSpin_valueChanged(int value)
{
    m_settings.m_alpacaGain = value;
    m_settingsKeys.append("alpacaGain");
    applySettings();
}

void CameraGUI::on_alpacaReadoutModeCombo_currentIndexChanged(int index)
{
    m_settings.m_alpacaReadoutMode = index;
    m_settingsKeys.append("alpacaReadoutMode");
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
