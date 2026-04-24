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

#include <QColorDialog>
#include <QFileDialog>
#include <QFontDatabase>
#include <QGraphicsView>
#include <QPainter>
#include <QPixmap>
#include <QStandardItemModel>
#include <QWheelEvent>

#include "feature/featureuiset.h"
#include "gui/crightclickenabler.h"
#include "gui/audioselectdialog.h"
#include "gui/dialogpositioner.h"
#include "dsp/dspengine.h"

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCamera>
#else
#include <QCamera>
#include <QCameraFocus>
#include <QCameraImageProcessing>
#endif

#include "ui_cameragui.h"
#include "camera.h"
#include "camerahistogramdialog.h"
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
    else if (CameraWorker::MsgReportAvailableDevices::match(message))
    {
        const CameraWorker::MsgReportAvailableDevices& report = (CameraWorker::MsgReportAvailableDevices&) message;
        const QString currentDevice = m_settings.m_spectrumDevice;

        ui->spectrumDeviceCombo->blockSignals(true);
        ui->spectrumDeviceCombo->clear();
        for (const QString& id : report.getDeviceLongIds()) {
            ui->spectrumDeviceCombo->addItem(id);
        }
        const int idx = ui->spectrumDeviceCombo->findText(currentDevice);
        ui->spectrumDeviceCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        ui->spectrumDeviceCombo->blockSignals(false);

        return true;
    }
    else if (CameraWorker::MsgReportQtCameraCapabilities::match(message))
    {
        const CameraWorker::MsgReportQtCameraCapabilities& caps =
            (CameraWorker::MsgReportQtCameraCapabilities&) message;
        const double minZoom = caps.getMinZoomFactor();
        const double maxZoom = caps.getMaxZoomFactor();
        m_qtZoomSupported = (maxZoom > minZoom + 0.01);

        blockApplySettings(true);
        ui->zoomSpin->setMinimum(minZoom);
        ui->zoomSpin->setMaximum(maxZoom > minZoom ? maxZoom : minZoom);
        ui->zoomSpin->setEnabled(m_qtZoomSupported);
        ui->zoomLabel->setEnabled(m_qtZoomSupported);
        blockApplySettings(false);

        return true;
    }
    else if (CameraWorker::MsgReportQtFocusModes::match(message))
    {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        const CameraWorker::MsgReportQtFocusModes& modesMsg =
            (CameraWorker::MsgReportQtFocusModes&) message;
        const QList<int>& supported = modesMsg.getFocusModes();

        blockApplySettings(true);
        for (int i = 0; i < ui->focusModeCombo->count(); ++i)
        {
            const int modeVal = ui->focusModeCombo->itemData(i).toInt();
            // Mark unsupported modes as disabled via the user role flag
            const QStandardItemModel *model = qobject_cast<QStandardItemModel*>(ui->focusModeCombo->model());
            if (model) {
                QStandardItem *item = model->item(i);
                if (item) {
                    item->setEnabled(supported.contains(modeVal));
                }
            }
        }
        // If the currently-saved focus mode is not supported, fall back to first supported mode
        if (!supported.isEmpty() && !supported.contains(m_settings.m_focusMode))
        {
            m_settings.m_focusMode = supported.first();
            const int idx = ui->focusModeCombo->findData(m_settings.m_focusMode);
            if (idx >= 0) {
                ui->focusModeCombo->setCurrentIndex(idx);
            }
        }
        blockApplySettings(false);
#endif
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
    m_alpacaHasNamedGains(false),
    m_alpacaHasNamedOffsets(false),
    m_qtZoomSupported(false),
    m_imageScene(nullptr),
    m_imagePixmapItem(nullptr)
{
    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/camera/readme.md";

    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    rollupContents->arrangeRollups();

    // Set up the QGraphicsView for camera preview
    m_imageScene = new QGraphicsScene(this);
    m_imagePixmapItem = m_imageScene->addPixmap(QPixmap());
    ui->imageView->setScene(m_imageScene);
    ui->imageView->setDragMode(QGraphicsView::ScrollHandDrag);
    ui->imageView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    ui->imageView->setRenderHint(QPainter::SmoothPixmapTransform, true);
    ui->imageView->setBackgroundBrush(QBrush(Qt::black));
    ui->imageView->viewport()->installEventFilter(this);

    m_camera = reinterpret_cast<Camera*>(feature);
    m_camera->setMessageQueueToGUI(&m_inputMessageQueue);

    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    m_settings.setRollupState(&m_rollupState);

    CRightClickEnabler *audioMuteRightClickEnabler = new CRightClickEnabler(ui->audioMute);
    connect(audioMuteRightClickEnabler, SIGNAL(rightClick(const QPoint &)), this, SLOT(audioSelect(const QPoint &)));

    // Populate white-balance combo (indices match QCamera::WhiteBalanceMode / QCameraImageProcessing::WhiteBalanceMode)
    ui->whiteBalanceCombo->addItem(tr("Auto"),        0);
    ui->whiteBalanceCombo->addItem(tr("Manual"),      1);
    ui->whiteBalanceCombo->addItem(tr("Sunlight"),    2);
    ui->whiteBalanceCombo->addItem(tr("Cloudy"),      3);
    ui->whiteBalanceCombo->addItem(tr("Shade"),       4);
    ui->whiteBalanceCombo->addItem(tr("Tungsten"),    5);
    ui->whiteBalanceCombo->addItem(tr("Fluorescent"), 6);
    ui->whiteBalanceCombo->addItem(tr("Flash"),       7);
    ui->whiteBalanceCombo->addItem(tr("Sunset"),      8);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    // Populate focus-mode combo with all Qt 6 modes; enabled items are updated once
    // MsgReportQtFocusModes arrives (after the camera starts).
    ui->focusModeCombo->addItem(tr("Auto"),       static_cast<int>(QCamera::FocusModeAuto));
    ui->focusModeCombo->addItem(tr("Auto near"),  static_cast<int>(QCamera::FocusModeAutoNear));
    ui->focusModeCombo->addItem(tr("Auto far"),   static_cast<int>(QCamera::FocusModeAutoFar));
    ui->focusModeCombo->addItem(tr("Hyperfocal"), static_cast<int>(QCamera::FocusModeHyperfocal));
    ui->focusModeCombo->addItem(tr("Infinity"),   static_cast<int>(QCamera::FocusModeInfinity));
    ui->focusModeCombo->addItem(tr("Manual"),     static_cast<int>(QCamera::FocusModeManual));
#endif

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
    ui->alpacaBinXSpin->setValue(m_settings.m_alpacaBinX);
    ui->alpacaBinYSpin->setValue(m_settings.m_alpacaBinY);

    if (m_alpacaHasNamedGains) {
        ui->alpacaGainCombo->setCurrentIndex(m_settings.m_alpacaGain >= 0 ? m_settings.m_alpacaGain : 0);
    } else {
        ui->alpacaGainSpin->setValue(m_settings.m_alpacaGain >= 0 ? m_settings.m_alpacaGain : 0);
    }

    if (m_alpacaHasNamedOffsets) {
        ui->alpacaOffsetCombo->setCurrentIndex(m_settings.m_alpacaOffset >= 0 ? m_settings.m_alpacaOffset : 0);
    } else {
        ui->alpacaOffsetSpin->setValue(m_settings.m_alpacaOffset >= 0 ? m_settings.m_alpacaOffset : 0);
    }

    ui->alpacaReadoutModeCombo->setCurrentIndex(m_settings.m_alpacaReadoutMode);
    ui->saveImageCheck->setChecked(m_settings.m_saveImage);
    ui->imagePathEdit->setText(m_settings.m_imageFileName);
    ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
    ui->videoPathEdit->setText(m_settings.m_videoFileName);
    ui->videoPostProcessButton->setChecked(m_settings.m_videoPostProcess);
    ui->brightnessSlider->setValue(static_cast<int>(m_settings.m_brightness));
    ui->brightnessValue->setText(QString::number(m_settings.m_brightness, 'f', 0));
    ui->contrastSlider->setValue(static_cast<int>(m_settings.m_contrast * 100.0));
    ui->contrastValue->setText(QString::number(m_settings.m_contrast, 'f', 2));
    ui->invertColorsButton->setChecked(m_settings.m_invertColors);
    ui->overlayDateTimeButton->setChecked(m_settings.m_overlayDateTime);
    ui->dateTimeFormatEdit->setText(m_settings.m_dateTimeFormat);
    ui->dateTimePosXSlider->setValue(m_settings.m_dateTimePosX);
    ui->dateTimePosXValue->setText(QString::number(m_settings.m_dateTimePosX));
    ui->dateTimePosYSlider->setValue(m_settings.m_dateTimePosY);
    ui->dateTimePosYValue->setText(QString::number(m_settings.m_dateTimePosY));
    ui->diffMaskButton->setChecked(m_settings.m_diffMask);
    ui->dilationSpin->setValue(m_settings.m_dilationSize);
    ui->overlayFontCombo->setCurrentText(m_settings.m_overlayFontFamily);
    ui->overlayFontScaleSpin->setValue(m_settings.m_overlayFontScale);
    ui->motionDetectButton->setChecked(m_settings.m_motionDetect);
    ui->minContourAreaSpin->setValue(m_settings.m_minContourArea);
    updateColorButton(ui->dateTimeColorButton, m_settings.m_dateTimeColor);
    updateColorButton(ui->motionBoxColorButton, m_settings.m_motionBoxColor);
    ui->spectrumOverlayButton->setChecked(m_settings.m_overlaySpectrum);
    {
        // Select the saved device in the spectrum combo
        const int idx = ui->spectrumDeviceCombo->findText(m_settings.m_spectrumDevice);
        ui->spectrumDeviceCombo->blockSignals(true);
        ui->spectrumDeviceCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        ui->spectrumDeviceCombo->blockSignals(false);
    }
    ui->spectrumOffsetXSlider->setValue(m_settings.m_spectrumOffsetX);
    ui->spectrumOffsetXValue->setText(QString::number(m_settings.m_spectrumOffsetX));
    ui->spectrumOffsetYSlider->setValue(m_settings.m_spectrumOffsetY);
    ui->spectrumOffsetYValue->setText(QString::number(m_settings.m_spectrumOffsetY));
    ui->spectrumScaleSpin->setValue(m_settings.m_spectrumScale);
    ui->yoloButton->setChecked(m_settings.m_yoloEnabled);
    ui->yoloModelPathEdit->setText(m_settings.m_yoloModelPath);
    ui->yoloLabelsPathEdit->setText(m_settings.m_yoloLabelsPath);
    ui->yoloConfSpin->setValue(m_settings.m_yoloConfThreshold);
    ui->yoloNmsSpin->setValue(m_settings.m_yoloNmsThreshold);
    updateColorButton(ui->yoloBoxColorButton, m_settings.m_yoloBoxColor);
    ui->audioMute->setChecked(m_settings.m_audioMute);

    // White balance (select by stored mode integer)
    {
        const int idx = ui->whiteBalanceCombo->findData(m_settings.m_whiteBalanceMode);
        ui->whiteBalanceCombo->blockSignals(true);
        ui->whiteBalanceCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        ui->whiteBalanceCombo->blockSignals(false);
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ui->exposureCompSpin->setValue(m_settings.m_exposureCompensation);
    {
        const int idx = ui->focusModeCombo->findData(m_settings.m_focusMode);
        ui->focusModeCombo->blockSignals(true);
        ui->focusModeCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        ui->focusModeCombo->blockSignals(false);
    }
    ui->focusDistSpin->setValue(m_settings.m_focusDistance);
#endif

    ui->zoomSpin->setValue(m_settings.m_zoomFactor);
    updateAlpacaVisibility();
    updateEnabledControls();
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
    if (m_lastImage.isNull() || !m_imagePixmapItem) {
        return;
    }

    const QPixmap pixmap = QPixmap::fromImage(m_lastImage);
    m_imagePixmapItem->setPixmap(pixmap);
    m_imageScene->setSceneRect(pixmap.rect());

    // Fit the image in the view (preserving aspect ratio) only when no zoom has been applied
    if (ui->imageView->transform().isIdentity()) {
        ui->imageView->fitInView(m_imagePixmapItem, Qt::KeepAspectRatio);
    }
}

void CameraGUI::makeUIConnections()
{
    QObject::connect(ui->startStop, &QPushButton::clicked, this, &CameraGUI::on_startStop_clicked);
    QObject::connect(ui->refreshCamerasButton, &QPushButton::clicked, this, &CameraGUI::on_refreshCamerasButton_clicked);
    QObject::connect(ui->cameraCombo, &QComboBox::currentTextChanged, this, &CameraGUI::on_cameraCombo_currentTextChanged);
    QObject::connect(ui->resolutionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_resolutionCombo_currentIndexChanged);
    QObject::connect(ui->fpsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_fpsSpin_valueChanged);
    QObject::connect(ui->exposureSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_exposureSpin_valueChanged);
    QObject::connect(ui->isoSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_isoSpin_valueChanged);
    QObject::connect(ui->alpacaHostEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_alpacaHostEdit_editingFinished);
    QObject::connect(ui->alpacaPortSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaPortSpin_valueChanged);
    QObject::connect(ui->alpacaBinXSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaBinXSpin_valueChanged);
    QObject::connect(ui->alpacaBinYSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaBinYSpin_valueChanged);
    QObject::connect(ui->alpacaGainCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaGainCombo_currentIndexChanged);
    QObject::connect(ui->alpacaGainSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaGainSpin_valueChanged);
    QObject::connect(ui->alpacaOffsetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaOffsetCombo_currentIndexChanged);
    QObject::connect(ui->alpacaOffsetSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaOffsetSpin_valueChanged);
    QObject::connect(ui->alpacaReadoutModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaReadoutModeCombo_currentIndexChanged);
    QObject::connect(ui->saveImageCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveImageCheck_toggled);
    QObject::connect(ui->imagePathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_imagePathEdit_editingFinished);
    QObject::connect(ui->imagePathButton, &QPushButton::clicked, this, &CameraGUI::on_imagePathButton_clicked);
    QObject::connect(ui->saveVideoCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveVideoCheck_toggled);
    QObject::connect(ui->videoPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_videoPathEdit_editingFinished);
    QObject::connect(ui->videoPathButton, &QPushButton::clicked, this, &CameraGUI::on_videoPathButton_clicked);
    QObject::connect(ui->videoPostProcessButton, &QToolButton::toggled, this, &CameraGUI::on_videoPostProcessButton_toggled);
    QObject::connect(ui->brightnessSlider, &QSlider::valueChanged, this, &CameraGUI::on_brightnessSlider_valueChanged);
    QObject::connect(ui->contrastSlider, &QSlider::valueChanged, this, &CameraGUI::on_contrastSlider_valueChanged);
    QObject::connect(ui->invertColorsButton, &QToolButton::toggled, this, &CameraGUI::on_invertColorsButton_toggled);
    QObject::connect(ui->overlayDateTimeButton, &QToolButton::toggled, this, &CameraGUI::on_overlayDateTimeButton_toggled);
    QObject::connect(ui->dateTimeColorButton, &QToolButton::clicked, this, &CameraGUI::on_dateTimeColorButton_clicked);
    QObject::connect(ui->dateTimeFormatEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_dateTimeFormatEdit_editingFinished);
    QObject::connect(ui->dateTimePosXSlider, &QSlider::valueChanged, this, &CameraGUI::on_dateTimePosXSlider_valueChanged);
    QObject::connect(ui->dateTimePosYSlider, &QSlider::valueChanged, this, &CameraGUI::on_dateTimePosYSlider_valueChanged);
    QObject::connect(ui->diffMaskButton, &QToolButton::toggled, this, &CameraGUI::on_diffMaskButton_toggled);
    QObject::connect(ui->dilationSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_dilationSpin_valueChanged);
    QObject::connect(ui->histogramButton, &QToolButton::clicked, this, &CameraGUI::on_histogramButton_clicked);
    QObject::connect(ui->overlayFontCombo, &QFontComboBox::currentFontChanged, this, &CameraGUI::on_overlayFontCombo_currentFontChanged);
    QObject::connect(ui->overlayFontScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_overlayFontScaleSpin_valueChanged);
    QObject::connect(ui->motionDetectButton, &QToolButton::toggled, this, &CameraGUI::on_motionDetectButton_toggled);
    QObject::connect(ui->minContourAreaSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_minContourAreaSpin_valueChanged);
    QObject::connect(ui->motionBoxColorButton, &QToolButton::clicked, this, &CameraGUI::on_motionBoxColorButton_clicked);
    QObject::connect(ui->spectrumOverlayButton, &QToolButton::toggled, this, &CameraGUI::on_spectrumOverlayButton_toggled);
    QObject::connect(ui->spectrumDeviceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_spectrumDeviceCombo_currentIndexChanged);
    QObject::connect(ui->spectrumOffsetXSlider, &QSlider::valueChanged, this, &CameraGUI::on_spectrumOffsetXSlider_valueChanged);
    QObject::connect(ui->spectrumOffsetYSlider, &QSlider::valueChanged, this, &CameraGUI::on_spectrumOffsetYSlider_valueChanged);
    QObject::connect(ui->spectrumScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_spectrumScaleSpin_valueChanged);
    QObject::connect(ui->yoloButton, &QToolButton::toggled, this, &CameraGUI::on_yoloButton_toggled);
    QObject::connect(ui->yoloModelPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_yoloModelPathEdit_editingFinished);
    QObject::connect(ui->yoloModelPathButton, &QPushButton::clicked, this, &CameraGUI::on_yoloModelPathButton_clicked);
    QObject::connect(ui->yoloLabelsPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_yoloLabelsPathEdit_editingFinished);
    QObject::connect(ui->yoloLabelsPathButton, &QPushButton::clicked, this, &CameraGUI::on_yoloLabelsPathButton_clicked);
    QObject::connect(ui->yoloConfSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_yoloConfSpin_valueChanged);
    QObject::connect(ui->yoloNmsSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_yoloNmsSpin_valueChanged);
    QObject::connect(ui->yoloBoxColorButton, &QToolButton::clicked, this, &CameraGUI::on_yoloBoxColorButton_clicked);
    QObject::connect(ui->zoomInButton, &QToolButton::clicked, this, &CameraGUI::on_zoomInButton_clicked);
    QObject::connect(ui->zoomOutButton, &QToolButton::clicked, this, &CameraGUI::on_zoomOutButton_clicked);
    QObject::connect(ui->fitInViewButton, &QToolButton::clicked, this, &CameraGUI::on_fitInViewButton_clicked);
    QObject::connect(ui->audioMute, &QToolButton::toggled, this, &CameraGUI::on_audioMute_toggled);
    QObject::connect(ui->whiteBalanceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_whiteBalanceCombo_currentIndexChanged);
    QObject::connect(ui->exposureCompSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_exposureCompSpin_valueChanged);
    QObject::connect(ui->focusModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_focusModeCombo_currentIndexChanged);
    QObject::connect(ui->focusDistSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_focusDistSpin_valueChanged);
    QObject::connect(ui->zoomSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_zoomSpin_valueChanged);
}

void CameraGUI::updateAlpacaVisibility()
{
    const bool alpaca = m_settings.isAlpacaCamera();

    ui->resolutionLabel->setVisible(!alpaca);
    ui->resolutionCombo->setVisible(!alpaca);
    ui->isoLabel->setVisible(!alpaca);
    ui->isoSpin->setVisible(!alpaca);
    /*ui->alpacaHostLabel->setVisible(alpaca);
    ui->alpacaHostEdit->setVisible(alpaca);
    ui->alpacaPortLabel->setVisible(alpaca);
    ui->alpacaPortSpin->setVisible(alpaca);*/
    ui->alpacaBinXLabel->setVisible(alpaca);
    ui->alpacaBinXSpin->setVisible(alpaca);
    ui->alpacaBinYLabel->setVisible(alpaca);
    ui->alpacaBinYSpin->setVisible(alpaca);
    ui->alpacaGainLabel->setVisible(alpaca);
    ui->alpacaGainCombo->setVisible(alpaca && m_alpacaHasNamedGains);
    ui->alpacaGainSpin->setVisible(alpaca && !m_alpacaHasNamedGains);
    ui->alpacaOffsetLabel->setVisible(alpaca);
    ui->alpacaOffsetCombo->setVisible(alpaca && m_alpacaHasNamedOffsets);
    ui->alpacaOffsetSpin->setVisible(alpaca && !m_alpacaHasNamedOffsets);
    ui->alpacaReadoutModeLabel->setVisible(alpaca);
    ui->alpacaReadoutModeCombo->setVisible(alpaca);
    ui->alpacaStatusGroup->setVisible(alpaca);
    ui->audioMute->setVisible(!alpaca);

    // Qt-camera-only controls
    ui->whiteBalanceLabel->setVisible(!alpaca);
    ui->whiteBalanceCombo->setVisible(!alpaca);
    ui->zoomLabel->setVisible(!alpaca);
    ui->zoomSpin->setVisible(!alpaca);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ui->exposureCompLabel->setVisible(!alpaca);
    ui->exposureCompSpin->setVisible(!alpaca);
    ui->focusModeLabel->setVisible(!alpaca);
    ui->focusModeCombo->setVisible(!alpaca);
    ui->focusDistLabel->setVisible(!alpaca);
    ui->focusDistSpin->setVisible(!alpaca);
#else
    ui->exposureCompLabel->setVisible(false);
    ui->exposureCompSpin->setVisible(false);
    ui->focusModeLabel->setVisible(false);
    ui->focusModeCombo->setVisible(false);
    ui->focusDistLabel->setVisible(false);
    ui->focusDistSpin->setVisible(false);
#endif
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

    // Offset
    m_alpacaHasNamedOffsets = !info.getOffsets().isEmpty();
    if (m_alpacaHasNamedOffsets)
    {
        ui->alpacaOffsetCombo->blockSignals(true);
        ui->alpacaOffsetCombo->clear();
        ui->alpacaOffsetCombo->addItems(info.getOffsets());
        const int offsetIdx = (m_settings.m_alpacaOffset >= 0 && m_settings.m_alpacaOffset < info.getOffsets().size())
            ? m_settings.m_alpacaOffset : 0;
        ui->alpacaOffsetCombo->setCurrentIndex(offsetIdx);
        ui->alpacaOffsetCombo->blockSignals(false);
    }
    else
    {
        ui->alpacaOffsetSpin->setMinimum(info.getOffsetMin());
        ui->alpacaOffsetSpin->setMaximum(std::max(info.getOffsetMin(), info.getOffsetMax()));
        const int offsetVal = (m_settings.m_alpacaOffset >= 0) ? m_settings.m_alpacaOffset : info.getOffsetMin();
        ui->alpacaOffsetSpin->setValue(qBound(info.getOffsetMin(), offsetVal, info.getOffsetMax()));
    }

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

void CameraGUI::on_alpacaOffsetCombo_currentIndexChanged(int index)
{
    m_settings.m_alpacaOffset = index;
    m_settingsKeys.append("alpacaOffset");
    applySettings();
}

void CameraGUI::on_alpacaOffsetSpin_valueChanged(int value)
{
    m_settings.m_alpacaOffset = value;
    m_settingsKeys.append("alpacaOffset");
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

void CameraGUI::on_videoPostProcessButton_toggled(bool checked)
{
    m_settings.m_videoPostProcess = checked;
    m_settingsKeys.append("videoPostProcess");
    applySettings();
}

void CameraGUI::on_brightnessSlider_valueChanged(int value)
{
    m_settings.m_brightness = static_cast<double>(value);
    ui->brightnessValue->setText(QString::number(value));
    m_settingsKeys.append("brightness");
    applySettings();
}

void CameraGUI::on_contrastSlider_valueChanged(int value)
{
    m_settings.m_contrast = value / 100.0;
    ui->contrastValue->setText(QString::number(m_settings.m_contrast, 'f', 2));
    m_settingsKeys.append("contrast");
    applySettings();
}

void CameraGUI::on_invertColorsButton_toggled(bool checked)
{
    m_settings.m_invertColors = checked;
    m_settingsKeys.append("invertColors");
    applySettings();
}

void CameraGUI::on_overlayDateTimeButton_toggled(bool checked)
{
    m_settings.m_overlayDateTime = checked;
    m_settingsKeys.append("overlayDateTime");
    updateEnabledControls();
    applySettings();
}

void CameraGUI::on_dateTimeColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_dateTimeColor, this, tr("Select date/time text colour"));

    if (color.isValid())
    {
        m_settings.m_dateTimeColor = color;
        updateColorButton(ui->dateTimeColorButton, m_settings.m_dateTimeColor);
        m_settingsKeys.append("dateTimeColor");
        applySettings();
    }
}

void CameraGUI::on_dateTimeFormatEdit_editingFinished()
{
    m_settings.m_dateTimeFormat = ui->dateTimeFormatEdit->text();
    m_settingsKeys.append("dateTimeFormat");
    applySettings();
}

void CameraGUI::on_dateTimePosXSlider_valueChanged(int value)
{
    m_settings.m_dateTimePosX = value;
    ui->dateTimePosXValue->setText(QString::number(value));
    m_settingsKeys.append("dateTimePosX");
    applySettings();
}

void CameraGUI::on_dateTimePosYSlider_valueChanged(int value)
{
    m_settings.m_dateTimePosY = value;
    ui->dateTimePosYValue->setText(QString::number(value));
    m_settingsKeys.append("dateTimePosY");
    applySettings();
}

void CameraGUI::on_diffMaskButton_toggled(bool checked)
{
    m_settings.m_diffMask = checked;
    m_settingsKeys.append("diffMask");
    updateEnabledControls();
    applySettings();
}

void CameraGUI::on_dilationSpin_valueChanged(int value)
{
    m_settings.m_dilationSize = value;
    m_settingsKeys.append("dilationSize");
    applySettings();
}

void CameraGUI::on_histogramButton_clicked()
{
    if (!m_lastImage.isNull())
    {
        CameraHistogramDialog dialog(m_lastImage, this);
        dialog.exec();
    }
}

/*static*/ void CameraGUI::updateColorButton(QToolButton* btn, const QColor& color)
{
    QPixmap px(16, 16);
    px.fill(color);
    btn->setIcon(QIcon(px));
    btn->setStyleSheet(QString());
}

void CameraGUI::updateEnabledControls()
{
    const bool dtActive = m_settings.m_overlayDateTime;
    ui->dateTimeColorButton->setEnabled(dtActive);
    ui->overlayFontLabel->setEnabled(dtActive);
    ui->overlayFontCombo->setEnabled(dtActive);
    ui->overlayFontSizeLabel->setEnabled(dtActive);
    ui->overlayFontScaleSpin->setEnabled(dtActive);
    ui->dateTimeFormatLabel->setEnabled(dtActive);
    ui->dateTimeFormatEdit->setEnabled(dtActive);
    ui->dateTimePosXLabel->setEnabled(dtActive);
    ui->dateTimePosXSlider->setEnabled(dtActive);
    ui->dateTimePosXValue->setEnabled(dtActive);
    ui->dateTimePosYLabel->setEnabled(dtActive);
    ui->dateTimePosYSlider->setEnabled(dtActive);
    ui->dateTimePosYValue->setEnabled(dtActive);

    const bool diffActive = m_settings.m_diffMask;
    ui->dilationLabel->setEnabled(diffActive);
    ui->dilationSpin->setEnabled(diffActive);

    const bool motionActive = m_settings.m_motionDetect;
    ui->minContourAreaLabel->setEnabled(motionActive);
    ui->minContourAreaSpin->setEnabled(motionActive);
    ui->motionBoxColorButton->setEnabled(motionActive);

    const bool specActive = m_settings.m_overlaySpectrum;
    ui->spectrumDeviceCombo->setEnabled(specActive);
    ui->spectrumOffsetXLabel->setEnabled(specActive);
    ui->spectrumOffsetXSlider->setEnabled(specActive);
    ui->spectrumOffsetXValue->setEnabled(specActive);
    ui->spectrumOffsetYLabel->setEnabled(specActive);
    ui->spectrumOffsetYSlider->setEnabled(specActive);
    ui->spectrumOffsetYValue->setEnabled(specActive);
    ui->spectrumScaleLabel->setEnabled(specActive);
    ui->spectrumScaleSpin->setEnabled(specActive);

    const bool yoloActive = m_settings.m_yoloEnabled;
    ui->yoloModelPathEdit->setEnabled(yoloActive);
    ui->yoloModelPathButton->setEnabled(yoloActive);
    ui->yoloLabelsLabel->setEnabled(yoloActive);
    ui->yoloLabelsPathEdit->setEnabled(yoloActive);
    ui->yoloLabelsPathButton->setEnabled(yoloActive);
    ui->yoloConfLabel->setEnabled(yoloActive);
    ui->yoloConfSpin->setEnabled(yoloActive);
    ui->yoloNmsLabel->setEnabled(yoloActive);
    ui->yoloNmsSpin->setEnabled(yoloActive);
    ui->yoloBoxColorButton->setEnabled(yoloActive);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const bool manualFocus = (m_settings.m_focusMode == static_cast<int>(QCamera::FocusModeManual));
    ui->focusDistLabel->setEnabled(manualFocus);
    ui->focusDistSpin->setEnabled(manualFocus);
#endif

    // Zoom control enabled state is set when MsgReportQtCameraCapabilities arrives
    if (!m_qtZoomSupported)
    {
        ui->zoomLabel->setEnabled(false);
        ui->zoomSpin->setEnabled(false);
    }
}

void CameraGUI::on_overlayFontCombo_currentFontChanged(const QFont& font)
{
    m_settings.m_overlayFontFamily = font.family();
    m_settingsKeys.append("overlayFontFamily");
    applySettings();
}

void CameraGUI::on_overlayFontScaleSpin_valueChanged(double value)
{
    m_settings.m_overlayFontScale = value;
    m_settingsKeys.append("overlayFontScale");
    applySettings();
}

void CameraGUI::on_motionDetectButton_toggled(bool checked)
{
    m_settings.m_motionDetect = checked;
    m_settingsKeys.append("motionDetect");
    updateEnabledControls();
    applySettings();
}

void CameraGUI::on_minContourAreaSpin_valueChanged(int value)
{
    m_settings.m_minContourArea = value;
    m_settingsKeys.append("minContourArea");
    applySettings();
}

void CameraGUI::on_motionBoxColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_motionBoxColor, this, tr("Select bounding box colour"));

    if (color.isValid())
    {
        m_settings.m_motionBoxColor = color;
        updateColorButton(ui->motionBoxColorButton, color);
        m_settingsKeys.append("motionBoxColor");
        applySettings();
    }
}

void CameraGUI::on_spectrumOverlayButton_toggled(bool checked)
{
    m_settings.m_overlaySpectrum = checked;
    m_settingsKeys.append("overlaySpectrum");
    updateEnabledControls();
    applySettings();
}

void CameraGUI::on_spectrumDeviceCombo_currentIndexChanged(int index)
{
    m_settings.m_spectrumDevice = ui->spectrumDeviceCombo->itemText(index);
    m_settingsKeys.append("spectrumDevice");
    applySettings();
}

void CameraGUI::on_spectrumOffsetXSlider_valueChanged(int value)
{
    m_settings.m_spectrumOffsetX = value;
    ui->spectrumOffsetXValue->setText(QString::number(value));
    m_settingsKeys.append("spectrumOffsetX");
    applySettings();
}

void CameraGUI::on_spectrumOffsetYSlider_valueChanged(int value)
{
    m_settings.m_spectrumOffsetY = value;
    ui->spectrumOffsetYValue->setText(QString::number(value));
    m_settingsKeys.append("spectrumOffsetY");
    applySettings();
}

void CameraGUI::on_spectrumScaleSpin_valueChanged(double value)
{
    m_settings.m_spectrumScale = value;
    m_settingsKeys.append("spectrumScale");
    applySettings();
}

void CameraGUI::on_yoloButton_toggled(bool checked)
{
    m_settings.m_yoloEnabled = checked;
    m_settingsKeys.append("yoloEnabled");
    updateEnabledControls();
    applySettings();
}

void CameraGUI::on_yoloModelPathEdit_editingFinished()
{
    m_settings.m_yoloModelPath = ui->yoloModelPathEdit->text();
    m_settingsKeys.append("yoloModelPath");
    applySettings();
}

void CameraGUI::on_yoloModelPathButton_clicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Select YOLO ONNX model"), m_settings.m_yoloModelPath,
        tr("ONNX model (*.onnx);;All files (*)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_yoloModelPath = fileName;
        ui->yoloModelPathEdit->setText(fileName);
        m_settingsKeys.append("yoloModelPath");
        applySettings();
    }
}

void CameraGUI::on_yoloLabelsPathEdit_editingFinished()
{
    m_settings.m_yoloLabelsPath = ui->yoloLabelsPathEdit->text();
    m_settingsKeys.append("yoloLabelsPath");
    applySettings();
}

void CameraGUI::on_yoloLabelsPathButton_clicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Select class labels file"), m_settings.m_yoloLabelsPath,
        tr("Names file (*.names *.txt);;All files (*)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_yoloLabelsPath = fileName;
        ui->yoloLabelsPathEdit->setText(fileName);
        m_settingsKeys.append("yoloLabelsPath");
        applySettings();
    }
}

void CameraGUI::on_yoloConfSpin_valueChanged(double value)
{
    m_settings.m_yoloConfThreshold = value;
    m_settingsKeys.append("yoloConfThreshold");
    applySettings();
}

void CameraGUI::on_yoloNmsSpin_valueChanged(double value)
{
    m_settings.m_yoloNmsThreshold = value;
    m_settingsKeys.append("yoloNmsThreshold");
    applySettings();
}

void CameraGUI::on_yoloBoxColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_yoloBoxColor, this, tr("Select bounding box colour"));

    if (color.isValid())
    {
        m_settings.m_yoloBoxColor = color;
        updateColorButton(ui->yoloBoxColorButton, color);
        m_settingsKeys.append("yoloBoxColor");
        applySettings();
    }
}

bool CameraGUI::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->imageView->viewport() && event->type() == QEvent::Wheel)
    {
        const QWheelEvent *wheelEvent = static_cast<QWheelEvent*>(event);
        const double factor = (wheelEvent->angleDelta().y() > 0) ? 1.25 : 1.0 / 1.25;
        ui->imageView->scale(factor, factor);
        return true;
    }

    return FeatureGUI::eventFilter(watched, event);
}

void CameraGUI::on_zoomInButton_clicked()
{
    ui->imageView->scale(1.25, 1.25);
}

void CameraGUI::on_zoomOutButton_clicked()
{
    ui->imageView->scale(1.0 / 1.25, 1.0 / 1.25);
}

void CameraGUI::on_fitInViewButton_clicked()
{
    ui->imageView->resetTransform();
    if (m_imagePixmapItem && !m_imagePixmapItem->pixmap().isNull()) {
        ui->imageView->fitInView(m_imagePixmapItem, Qt::KeepAspectRatio);
    }
}

void CameraGUI::on_audioMute_toggled(bool checked)
{
    m_settings.m_audioMute = checked;
    m_settingsKeys.append("audioMute");
    applySettings();
}

void CameraGUI::audioSelect(const QPoint& p)
{
    qDebug("CameraGUI::audioSelect");
    AudioSelectDialog audioSelect(DSPEngine::instance()->getAudioDeviceManager(), m_settings.m_audioDeviceName);
    audioSelect.move(p);
    new DialogPositioner(&audioSelect, false);
    audioSelect.exec();

    if (audioSelect.m_selected)
    {
        m_settings.m_audioDeviceName = audioSelect.m_audioDeviceName;
        m_settingsKeys.append("audioDeviceName");
        applySettings();
    }
}

void CameraGUI::on_whiteBalanceCombo_currentIndexChanged(int index)
{
    m_settings.m_whiteBalanceMode = ui->whiteBalanceCombo->itemData(index).toInt();
    m_settingsKeys.append("whiteBalanceMode");
    applySettings();
}

void CameraGUI::on_exposureCompSpin_valueChanged(double value)
{
    m_settings.m_exposureCompensation = value;
    m_settingsKeys.append("exposureCompensation");
    applySettings();
}

void CameraGUI::on_focusModeCombo_currentIndexChanged(int index)
{
    m_settings.m_focusMode = ui->focusModeCombo->itemData(index).toInt();
    m_settingsKeys.append("focusMode");
    updateEnabledControls();
    applySettings();
}

void CameraGUI::on_focusDistSpin_valueChanged(double value)
{
    m_settings.m_focusDistance = value;
    m_settingsKeys.append("focusDistance");
    applySettings();
}

void CameraGUI::on_zoomSpin_valueChanged(double value)
{
    m_settings.m_zoomFactor = value;
    m_settingsKeys.append("zoomFactor");
    applySettings();
}
