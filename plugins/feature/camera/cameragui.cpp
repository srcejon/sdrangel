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
#include <QColorDialog>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFontDatabase>
#include <QGraphicsView>
#include <QPainter>
#include <QPixmap>
#include <QSet>
#include <QSignalBlocker>
#include <QStandardItemModel>
#include <QTextStream>
#include <QWheelEvent>
#include <QMessageBox>

#include "feature/featureuiset.h"
#include "gui/crightclickenabler.h"
#include "gui/audioselectdialog.h"
#include "gui/dialogpositioner.h"
#include "dsp/dspengine.h"

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCamera>
#include <QCameraDevice>
#include <QCameraFormat>
#include <QImageCapture>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QVideoFrame>
#include <QVideoSink>
#else
#include <QAbstractVideoBuffer>
#include <QAbstractVideoSurface>
#include <QCamera>
#include <QCameraExposure>
#include <QCameraFocus>
#include <QCameraImageCapture>
#include <QCameraImageProcessing>
#include <QCameraInfo>
#include <QCameraViewfinderSettings>
#include <QVideoFrame>
#endif

#include "ui_cameragui.h"
#include "camera.h"
#include "camerahistogramdialog.h"
#include "camerasettingsdialog.h"
#include "cameraworker.h"
#include "cameragui.h"

namespace {

QString resolutionKey(const QSize& size)
{
    return QStringLiteral("%1x%2").arg(size.width()).arg(size.height());
}

QString resolutionKey(int width, int height)
{
    return QStringLiteral("%1x%2").arg(width).arg(height);
}

int decimalsForStepSize(double step)
{
    const double normalizedStep = std::max(0.001, step);

    for (int decimals = 0; decimals <= 6; ++decimals)
    {
        const double scaled = normalizedStep * std::pow(10.0, decimals);
        if (std::abs(scaled - std::round(scaled)) < 1e-6) {
            return decimals;
        }
    }

    return 6;
}

int doubleSpinBoxSliderMaximum(const QDoubleSpinBox *spinBox)
{
    const double step = std::max(0.000001, spinBox->singleStep());
    return std::max(0, static_cast<int>(std::llround((spinBox->maximum() - spinBox->minimum()) / step)));
}

int doubleSpinBoxValueToSlider(const QDoubleSpinBox *spinBox, double value)
{
    const double step = std::max(0.000001, spinBox->singleStep());
    const int sliderValue = static_cast<int>(std::llround((value - spinBox->minimum()) / step));
    return qBound(0, sliderValue, doubleSpinBoxSliderMaximum(spinBox));
}

double sliderValueToDoubleSpinBox(const QDoubleSpinBox *spinBox, int sliderValue)
{
    const double step = std::max(0.000001, spinBox->singleStep());
    return qBound(spinBox->minimum(), spinBox->minimum() + (sliderValue * step), spinBox->maximum());
}

double currentExposureUnitScaleMs(const Ui::CameraSettingsDialog *ui)
{
    const QVariant data = ui->exposureUnitsCombo->currentData();
    return data.isValid() ? data.toDouble() : 1.0;
}

void appendFpsRange(QSet<int>& fpsValues, qreal minFps, qreal maxFps)
{
    const int minRounded = qMax(1, static_cast<int>(std::ceil(minFps)));
    const int maxRounded = qMax(minRounded, static_cast<int>(std::floor(maxFps)));

    for (int fps = minRounded; fps <= maxRounded; ++fps) {
        fpsValues.insert(fps);
    }
}

CameraGUI::FrameRateOptions makeFrameRateOptions(const QSet<int>& fpsValues)
{
    CameraGUI::FrameRateOptions options{true, 1, 1, {}};
    options.values = fpsValues.values();
    std::sort(options.values.begin(), options.values.end());

    if (options.values.isEmpty()) {
        options.values.append(1);
    }

    options.minFps = options.values.first();
    options.maxFps = options.values.last();
    options.contiguous = true;

    for (int i = 1; i < options.values.size(); ++i)
    {
        if (options.values.at(i) != options.values.at(i - 1) + 1)
        {
            options.contiguous = false;
            break;
        }
    }

    return options;
}

}

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
    const bool wasAlpaca = m_settings.isAlpacaCamera();
    const QString previousCameraId = m_settings.m_cameraId;

    if (Camera::MsgConfigureCamera::match(message))
    {
        const Camera::MsgConfigureCamera& cfg = (Camera::MsgConfigureCamera&) message;

        if (cfg.getForce()) {
            m_settings = cfg.getSettings();
        } else {
            m_settings.applySettings(cfg.getSettingsKeys(), cfg.getSettings());
        }

        if ((previousCameraId != m_settings.m_cameraId) || (wasAlpaca != m_settings.isAlpacaCamera())) {
            m_settingsDialog->clearAlpacaStatus();
        }

        blockApplySettings(true);
        displaySettings();
        blockApplySettings(false);

        // (Re)start or update the Qt camera in response to settings changes or feature start
        applyQtCameraSettings(cfg.getSettingsKeys(), cfg.getForce());

        return true;
    }
    else if (Camera::MsgStartStop::match(message))
    {
        const Camera::MsgStartStop& cfg = (Camera::MsgStartStop&) message;

        if ((previousCameraId != m_settings.m_cameraId) || (wasAlpaca != m_settings.isAlpacaCamera())) {
            m_settingsDialog->clearAlpacaStatus();
        }

        if (cfg.getStartStop())
        {
            if (m_settings.isQtCamera()) {
                setupQtCapture();
            }
        }
        else
        {
            cleanupQtCapture();
        }

        return true;
    }
    else if (CameraWorker::MsgReportCameraList::match(message))
    {
        const CameraWorker::MsgReportCameraList& report = (CameraWorker::MsgReportCameraList&) message;
        const QString current = ui->cameraCombo->currentText();
        QString selectedCameraId;

        ui->cameraCombo->blockSignals(true);
        ui->cameraCombo->clear();
        ui->cameraCombo->addItems(report.getCameraIds());

        const int index = ui->cameraCombo->findText(current);

        if (index >= 0) {
            ui->cameraCombo->setCurrentIndex(index);
            selectedCameraId = ui->cameraCombo->itemText(index);
        } else if (!m_settings.m_cameraId.isEmpty()) {
            ui->cameraCombo->setCurrentText(m_settings.m_cameraId);
            selectedCameraId = ui->cameraCombo->currentText();
        } else if (ui->cameraCombo->count() > 0) {
            ui->cameraCombo->setCurrentIndex(0);
            selectedCameraId = ui->cameraCombo->itemText(0);
        }

        ui->cameraCombo->blockSignals(false);

        if (m_settings.m_cameraId.isEmpty() && !selectedCameraId.isEmpty())
        {
            m_settings.m_cameraId = selectedCameraId;
            m_settingsKeys.append("cameraId");
            updateAlpacaVisibility();
            updateEnabledControls();
            applySettings();
        }

        return true;
    }
    else if (CameraPostProcessor::MsgReportFrame::match(message))
    {
        const CameraPostProcessor::MsgReportFrame& report = (CameraPostProcessor::MsgReportFrame&) message;
        QSize oldSize = m_lastImage.size();
        m_lastImage = report.getImage();
        updateImageWidget();
        if (m_histogramDialog) {
            m_histogramDialog->updateImage(m_lastImage);
        }
        // When the image size changes, refit to view
        if (oldSize != m_lastImage.size()) {
            ui->imageView->fitInView(m_imagePixmapItem, Qt::KeepAspectRatio);
        }
        return true;
    }
    else if (CameraPostProcessor::MsgDownloadProgress::match(message))
    {
        const CameraPostProcessor::MsgDownloadProgress& report = (CameraPostProcessor::MsgDownloadProgress&) message;

        if (m_progressDialog)
        {
            m_progressDialog->setValue(100 * report.getBytesRead() / report.getTotalBytes());
        }
        else
        {
            QString text = QString("Downloading: %1\nTo: %2.")
                .arg(report.getURL())
                .arg(report.getFilename());

            m_progressDialog = new QProgressDialog(this);
            m_progressDialog->setCancelButton(nullptr);
            m_progressDialog->setMinimumDuration(500);
            m_progressDialog->setLabelText(text);
        }

        return true;
    }
    else if (CameraPostProcessor::MsgDownloadComplete::match(message))
    {
        const CameraPostProcessor::MsgDownloadComplete& report = (CameraPostProcessor::MsgDownloadComplete&) message;

        // Close progress dialog
        if (m_progressDialog)
        {
            m_progressDialog->close();
            m_progressDialog->deleteLater();
            m_progressDialog = nullptr;
        }

        // Report any errors
        if (!report.getSuccess())
        {
            QString error = report.getError();
            if (!error.isEmpty())
            {
                error = QString("An unknown error occurred during download from %1 to %2.")
                    .arg(report.getURL())
                    .arg(report.getFilename());
            }
            QMessageBox::warning(this, "Download failed", error);
        }

        // Update settings to use downloaded file rather than URL
        if (report.getURL() == m_settings.m_yoloModelPath) {
            settingsUI()->yoloModelPathCombo->setCurrentText(report.getFilename());
        }
        if (report.getURL() == m_settings.m_yoloLabelsPath) {
            settingsUI()->yoloLabelsPathCombo->setCurrentText(report.getFilename());
        }
        return true;
    }
    else if (CameraPostProcessor::MsgReportSaveVideoState::match(message))
    {
        const CameraPostProcessor::MsgReportSaveVideoState& report = (CameraPostProcessor::MsgReportSaveVideoState&) message;
        m_settings.m_saveVideo = report.getSaveVideo();
        ui->saveVideoCheck->blockSignals(true);
        ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
        ui->saveVideoCheck->blockSignals(false);
        m_settingsKeys.append("saveVideo");
        applySettings();
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
        settingsUI()->cameraStateLabel->setText(
            (cs >= 0 && cs < cameraStateNames.size()) ? cameraStateNames[cs] : (cs >= 0 ? QString::number(cs) : "-"));
        settingsUI()->captureTimeLabel->setText(
            status.getCaptureTimeMs() >= 0 ? QString::number(status.getCaptureTimeMs()) : "-");

        if (status.isCcdTemperatureValid()) {
            settingsUI()->ccdTempLabel->setText(QString::number(status.getCcdTemperature(), 'f', 1));
            m_settingsDialog->appendTemperatureSample(QDateTime::currentDateTime(), status.getCcdTemperature());
        }

        return true;
    }
    else if (CameraWorker::MsgReportAvailableDevices::match(message))
    {
        const CameraWorker::MsgReportAvailableDevices& report = (CameraWorker::MsgReportAvailableDevices&) message;
        const QString currentDevice = m_settings.m_spectrumDevice;

        settingsUI()->spectrumDeviceCombo->blockSignals(true);
        settingsUI()->spectrumDeviceCombo->clear();
        for (const QString& id : report.getDeviceLongIds()) {
            settingsUI()->spectrumDeviceCombo->addItem(id);
        }
        const int idx = settingsUI()->spectrumDeviceCombo->findText(currentDevice);
        settingsUI()->spectrumDeviceCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        settingsUI()->spectrumDeviceCombo->blockSignals(false);

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
    m_lastFeatureState(0),
    m_progressDialog(nullptr),
    m_settingsDialog(nullptr),
    m_histogramDialog(nullptr),
    m_alpacaHasNamedGains(false),
    m_alpacaHasNamedOffsets(false),
    m_qtZoomSupported(false),
    m_qtManualExposureSupported(true),
    m_qtIsoSensitivitySupported(true),
    m_qtWhiteBalanceModeSupported(true),
    m_qtExposureCompensationSupported(true),
    m_exposureMinimumMs(1.0),
    m_exposureMaximumMs(60000.0),
    m_exposureStepMs(1.0),
    m_imageScene(nullptr),
    m_imagePixmapItem(nullptr),
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    m_qtCamera(nullptr),
    m_imageCapture(nullptr),
    m_videoSink(nullptr),
    m_captureSession(nullptr)
#else
    m_qtCamera(nullptr),
    m_imageCapture(nullptr),
    m_videoSurface(nullptr)
#endif
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
    ui->imageContainer->setGeometry(10, 90, 402, 412);

    m_camera = reinterpret_cast<Camera*>(feature);
    m_camera->setMessageQueueToGUI(&m_inputMessageQueue);

    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    m_settings.setRollupState(&m_rollupState);

    m_settingsDialog = new CameraSettingsDialog(this);
    new DialogPositioner(m_settingsDialog, false);

    CRightClickEnabler *audioMuteRightClickEnabler = new CRightClickEnabler(ui->audioMute);
    connect(audioMuteRightClickEnabler, SIGNAL(rightClick(const QPoint &)), this, SLOT(audioSelect(const QPoint &)));

    // Populate white-balance combo (indices match QCamera::WhiteBalanceMode / QCameraImageProcessing::WhiteBalanceMode)
    settingsUI()->whiteBalanceCombo->addItem(tr("Auto"),        0);
    settingsUI()->whiteBalanceCombo->addItem(tr("Manual"),      1);
    settingsUI()->whiteBalanceCombo->addItem(tr("Sunlight"),    2);
    settingsUI()->whiteBalanceCombo->addItem(tr("Cloudy"),      3);
    settingsUI()->whiteBalanceCombo->addItem(tr("Shade"),       4);
    settingsUI()->whiteBalanceCombo->addItem(tr("Tungsten"),    5);
    settingsUI()->whiteBalanceCombo->addItem(tr("Fluorescent"), 6);
    settingsUI()->whiteBalanceCombo->addItem(tr("Flash"),       7);
    settingsUI()->whiteBalanceCombo->addItem(tr("Sunset"),      8);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    // Populate focus-mode combo with all Qt 6 modes; enabled items are updated once
    // the camera starts (inside setupQtCapture).
    settingsUI()->focusModeCombo->addItem(tr("Auto"),       static_cast<int>(QCamera::FocusModeAuto));
    settingsUI()->focusModeCombo->addItem(tr("Auto near"),  static_cast<int>(QCamera::FocusModeAutoNear));
    settingsUI()->focusModeCombo->addItem(tr("Auto far"),   static_cast<int>(QCamera::FocusModeAutoFar));
    settingsUI()->focusModeCombo->addItem(tr("Hyperfocal"), static_cast<int>(QCamera::FocusModeHyperfocal));
    settingsUI()->focusModeCombo->addItem(tr("Infinity"),   static_cast<int>(QCamera::FocusModeInfinity));
    settingsUI()->focusModeCombo->addItem(tr("Manual"),     static_cast<int>(QCamera::FocusModeManual));
#endif

    connect(&m_statusTimer, &QTimer::timeout, this, &CameraGUI::updateStatus);
    connect(m_settingsDialog, &QDialog::finished, this, &CameraGUI::onSettingsDialogFinished);
    connect(&m_qtStillCaptureTimer, &QTimer::timeout, this, &CameraGUI::triggerQtStillCapture);
    m_qtStillCaptureTimer.setSingleShot(false);

    settingsUI()->fpsLabel->addItem(tr("Frame Rate"), CameraSettings::CaptureModeFrameRate);
    settingsUI()->fpsLabel->addItem(tr("Interval"), CameraSettings::CaptureModeInterval);
    settingsUI()->intervalUnitsCombo->addItem(tr("Seconds"), CameraSettings::CaptureIntervalSeconds);
    settingsUI()->intervalUnitsCombo->addItem(tr("Minutes"), CameraSettings::CaptureIntervalMinutes);
    settingsUI()->exposureUnitsCombo->addItem(tr("us"), 0.001);
    settingsUI()->exposureUnitsCombo->addItem(tr("ms"), 1.0);
    settingsUI()->exposureUnitsCombo->addItem(tr("s"), 1000.0);
    settingsUI()->exposureUnitsCombo->addItem(tr("min"), 60000.0);
    settingsUI()->exposureUnitsCombo->setCurrentIndex(1);
    m_statusTimer.start(250);

    displaySettings();
    applySettings(true);
    makeUIConnections();
    m_resizer.enableChildMouseTracking();

    m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create());
}

CameraGUI::~CameraGUI()
{
    cleanupQtCapture();
    delete m_histogramDialog;
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

Ui::CameraSettingsDialog *CameraGUI::settingsUI() const
{
    return m_settingsDialog->getUI();
}

void CameraGUI::displaySettings()
{
    setWindowTitle(m_settings.m_title);
    setTitle(m_settings.m_title);

    ui->cameraCombo->setCurrentText(m_settings.m_cameraId);

    const QString resText = QString("%1x%2").arg(m_settings.m_resolutionWidth).arg(m_settings.m_resolutionHeight);
    const int resIdx = settingsUI()->resolutionCombo->findText(resText);
    if (resIdx >= 0) {
        settingsUI()->resolutionCombo->setCurrentIndex(resIdx);
    }

    {
        const int captureModeIndex = settingsUI()->fpsLabel->findData(m_settings.m_captureMode);
        settingsUI()->fpsLabel->setCurrentIndex(captureModeIndex >= 0 ? captureModeIndex : 0);
    }

    updateFrameRateControlForResolution(resText);
    settingsUI()->intervalSpin->setValue(m_settings.m_captureInterval);
    {
        const int intervalUnitsIndex = settingsUI()->intervalUnitsCombo->findData(m_settings.m_captureIntervalUnits);
        settingsUI()->intervalUnitsCombo->setCurrentIndex(intervalUnitsIndex >= 0 ? intervalUnitsIndex : 0);
    }
    updateCaptureModeControls();
    populateActionClasses();
    {
        QSignalBlocker blocker(settingsUI()->actionsDisappearDebounceSpin);
        settingsUI()->actionsDisappearDebounceSpin->setValue(m_settings.m_yoloDisappearDebounce);
    }
    rebuildActionTabsForCurrentClass();
    updateExposureControls();
    settingsUI()->isoSpin->setValue(m_settings.m_isoSensitivity);
    settingsUI()->alpacaHostEdit->setText(m_settings.m_alpacaHost);
    settingsUI()->alpacaPortSpin->setValue(m_settings.m_alpacaPort);
    settingsUI()->alpacaBinXSpin->setValue(m_settings.m_alpacaBinX);
    settingsUI()->alpacaBinYSpin->setValue(m_settings.m_alpacaBinY);

    if (m_alpacaHasNamedGains) {
        settingsUI()->alpacaGainCombo->setCurrentIndex(m_settings.m_alpacaGain >= 0 ? m_settings.m_alpacaGain : 0);
    } else {
        settingsUI()->alpacaGainSpin->setValue(m_settings.m_alpacaGain >= 0 ? m_settings.m_alpacaGain : 0);
        settingsUI()->alpacaGainSlider->setValue(m_settings.m_alpacaGain >= 0 ? m_settings.m_alpacaGain : 0);
    }

    if (m_alpacaHasNamedOffsets) {
        settingsUI()->alpacaOffsetCombo->setCurrentIndex(m_settings.m_alpacaOffset >= 0 ? m_settings.m_alpacaOffset : 0);
    } else {
        settingsUI()->alpacaOffsetSpin->setValue(m_settings.m_alpacaOffset >= 0 ? m_settings.m_alpacaOffset : 0);
        settingsUI()->alpacaOffsetSlider->setValue(m_settings.m_alpacaOffset >= 0 ? m_settings.m_alpacaOffset : 0);
    }

    settingsUI()->alpacaReadoutModeCombo->setCurrentIndex(m_settings.m_alpacaReadoutMode);
    ui->saveImageCheck->setChecked(m_settings.m_saveImage);
    settingsUI()->imagePathEdit->setText(m_settings.m_imageFileName);
    ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
    settingsUI()->videoPathEdit->setText(m_settings.m_videoFileName);
    settingsUI()->videoPostProcessCombo->setCurrentIndex(static_cast<int>(m_settings.m_videoPostProcess));
    settingsUI()->postProcessWhiteBalanceModeCombo->setCurrentIndex(m_settings.m_postProcessWhiteBalanceMode);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->setValue(m_settings.m_postProcessWhiteBalanceRedGain);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->setValue(m_settings.m_postProcessWhiteBalanceGreenGain);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->setValue(m_settings.m_postProcessWhiteBalanceBlueGain);
    settingsUI()->postProcessWhiteBalanceRedGainSlider->setMaximum(doubleSpinBoxSliderMaximum(settingsUI()->postProcessWhiteBalanceRedGainSpin));
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->setMaximum(doubleSpinBoxSliderMaximum(settingsUI()->postProcessWhiteBalanceGreenGainSpin));
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->setMaximum(doubleSpinBoxSliderMaximum(settingsUI()->postProcessWhiteBalanceBlueGainSpin));
    settingsUI()->postProcessWhiteBalanceRedGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceRedGainSpin, m_settings.m_postProcessWhiteBalanceRedGain));
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceGreenGainSpin, m_settings.m_postProcessWhiteBalanceGreenGain));
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceBlueGainSpin, m_settings.m_postProcessWhiteBalanceBlueGain));
    settingsUI()->saturationSlider->setValue(static_cast<int>(m_settings.m_saturation * 100.0));
    settingsUI()->saturationSpin->setValue(m_settings.m_saturation);
    settingsUI()->gammaSlider->setValue(static_cast<int>(m_settings.m_gamma * 100.0));
    settingsUI()->gammaSpin->setValue(m_settings.m_gamma);
    settingsUI()->gaussianBlurSlider->setValue(m_settings.m_gaussianBlur);
    settingsUI()->gaussianBlurSpin->setValue(m_settings.m_gaussianBlur);
    settingsUI()->medianBlurSlider->setValue(m_settings.m_medianBlur);
    settingsUI()->medianBlurSpin->setValue(m_settings.m_medianBlur);
    settingsUI()->sharpenSlider->setValue(static_cast<int>(m_settings.m_sharpen * 100.0));
    settingsUI()->sharpenSpin->setValue(m_settings.m_sharpen);
    settingsUI()->sobelEdgeSlider->setValue(static_cast<int>(m_settings.m_sobelEdge * 100.0));
    settingsUI()->sobelEdgeSpin->setValue(m_settings.m_sobelEdge);
    settingsUI()->flipXButton->setChecked(m_settings.m_flipX);
    settingsUI()->flipYButton->setChecked(m_settings.m_flipY);
    settingsUI()->brightnessSlider->setValue(static_cast<int>(m_settings.m_brightness));
    settingsUI()->brightnessSpin->setValue(static_cast<int>(m_settings.m_brightness));
    settingsUI()->contrastSlider->setValue(static_cast<int>(m_settings.m_contrast * 100.0));
    settingsUI()->contrastSpin->setValue(m_settings.m_contrast);
    updatePostProcessWhiteBalanceControls();
    ui->invertColorsButton->setChecked(m_settings.m_invertColors);
    ui->overlayDateTimeButton->setChecked(m_settings.m_overlayDateTime);
    settingsUI()->dateTimeFormatEdit->setText(m_settings.m_dateTimeFormat);
    settingsUI()->dateTimePosXSlider->setValue(m_settings.m_dateTimePosX);
    settingsUI()->dateTimePosXValue->setText(QString::number(m_settings.m_dateTimePosX));
    settingsUI()->dateTimePosYSlider->setValue(m_settings.m_dateTimePosY);
    settingsUI()->dateTimePosYValue->setText(QString::number(m_settings.m_dateTimePosY));
    ui->overlayTextButton->setChecked(m_settings.m_overlayText);
    settingsUI()->overlayTextEdit->blockSignals(true);
    settingsUI()->overlayTextEdit->setPlainText(m_settings.m_overlayTextString);
    settingsUI()->overlayTextEdit->blockSignals(false);
    settingsUI()->overlayTextPosXSlider->setValue(m_settings.m_overlayTextPosX);
    settingsUI()->overlayTextPosXValue->setText(QString::number(m_settings.m_overlayTextPosX));
    settingsUI()->overlayTextPosYSlider->setValue(m_settings.m_overlayTextPosY);
    settingsUI()->overlayTextPosYValue->setText(QString::number(m_settings.m_overlayTextPosY));
    ui->diffMaskButton->setChecked(m_settings.m_diffMask);
    settingsUI()->diffThresholdSpin->setValue(m_settings.m_diffThreshold);
    settingsUI()->dilationSpin->setValue(m_settings.m_dilationSize);
    settingsUI()->diffMaskHistoryFramesSpin->setValue(m_settings.m_diffMaskHistoryFrames);
    settingsUI()->overlayFontCombo->setCurrentText(m_settings.m_overlayFontFamily);
    settingsUI()->overlayFontScaleSpin->setValue(m_settings.m_overlayFontScale);
    settingsUI()->overlayTextFontCombo->setCurrentText(m_settings.m_overlayTextFontFamily);
    settingsUI()->overlayTextFontScaleSpin->setValue(m_settings.m_overlayTextFontScale);
    ui->motionDetectButton->setChecked(m_settings.m_motionDetect);
    settingsUI()->minContourAreaSpin->setValue(m_settings.m_minContourArea);
    updateColorButton(settingsUI()->dateTimeColorButton, m_settings.m_dateTimeColor);
    updateColorButton(settingsUI()->overlayTextColorButton, m_settings.m_overlayTextColor);
    updateColorButton(settingsUI()->motionBoxColorButton, m_settings.m_motionBoxColor);
    ui->spectrumOverlayButton->setChecked(m_settings.m_overlaySpectrum);
    {
        // Select the saved device in the spectrum combo
        const int idx = settingsUI()->spectrumDeviceCombo->findText(m_settings.m_spectrumDevice);
        settingsUI()->spectrumDeviceCombo->blockSignals(true);
        settingsUI()->spectrumDeviceCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        settingsUI()->spectrumDeviceCombo->blockSignals(false);
    }
    settingsUI()->spectrumOffsetXSlider->setValue(m_settings.m_spectrumOffsetX);
    settingsUI()->spectrumOffsetXValue->setText(QString::number(m_settings.m_spectrumOffsetX));
    settingsUI()->spectrumOffsetYSlider->setValue(m_settings.m_spectrumOffsetY);
    settingsUI()->spectrumOffsetYValue->setText(QString::number(m_settings.m_spectrumOffsetY));
    settingsUI()->spectrumScaleSpin->setValue(m_settings.m_spectrumScale);
    ui->yoloButton->setChecked(m_settings.m_yoloEnabled);
    settingsUI()->yoloModelPathCombo->setCurrentText(m_settings.m_yoloModelPath);
    settingsUI()->yoloLabelsPathCombo->setCurrentText(m_settings.m_yoloLabelsPath);
    settingsUI()->yoloConfSpin->setValue(m_settings.m_yoloConfThreshold);
    settingsUI()->yoloNmsSpin->setValue(m_settings.m_yoloNmsThreshold);
    settingsUI()->yoloTargetCombo->setCurrentIndex((int) m_settings.m_yoloDnnTarget);
    updateColorButton(settingsUI()->yoloBoxColorButton, m_settings.m_yoloBoxColor);
    ui->audioMute->setChecked(m_settings.m_audioMute);

    // White balance (select by stored mode integer)
    {
        const int idx = settingsUI()->whiteBalanceCombo->findData(m_settings.m_whiteBalanceMode);
        settingsUI()->whiteBalanceCombo->blockSignals(true);
        settingsUI()->whiteBalanceCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        settingsUI()->whiteBalanceCombo->blockSignals(false);
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    settingsUI()->exposureCompSpin->setValue(m_settings.m_exposureCompensation);
    {
        const int idx = settingsUI()->focusModeCombo->findData(m_settings.m_focusMode);
        settingsUI()->focusModeCombo->blockSignals(true);
        settingsUI()->focusModeCombo->setCurrentIndex(idx >= 0 ? idx : 0);
        settingsUI()->focusModeCombo->blockSignals(false);
    }
    settingsUI()->focusDistSpin->setValue(m_settings.m_focusDistance);
#endif

    settingsUI()->zoomSpin->setValue(m_settings.m_zoomFactor);
    updateAlpacaVisibility();
    updateEnabledControls();
    applyVideoPath();
    applyImagePath();
}

void CameraGUI::applySettings(bool force)
{
    if (!m_doApplySettings) {
        return;
    }

    Camera::MsgConfigureCamera *msg = Camera::MsgConfigureCamera::create(m_settings, m_settingsKeys, force);
    m_camera->getInputMessageQueue()->push(msg);

    applyQtCameraSettings(m_settingsKeys, force);

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

    // Update max overlay positions according to size of image
    const int maxX = m_lastImage.width();
    const int maxY = m_lastImage.height();
    settingsUI()->overlayTextPosXSlider->setMaximum(maxX);
    settingsUI()->overlayTextPosYSlider->setMaximum(maxY);
    settingsUI()->dateTimePosXSlider->setMaximum(maxX);
    settingsUI()->dateTimePosYSlider->setMaximum(maxY);
    settingsUI()->spectrumOffsetXSlider->setMaximum(maxX);
    settingsUI()->spectrumOffsetYSlider->setMaximum(maxY);
}

void CameraGUI::makeUIConnections()
{
    QObject::connect(ui->cameraSettingsButton, &QToolButton::clicked, this, &CameraGUI::on_cameraSettingsButton_clicked);
    QObject::connect(ui->startStop, &QPushButton::clicked, this, &CameraGUI::on_startStop_clicked);
    QObject::connect(ui->refreshCamerasButton, &QPushButton::clicked, this, &CameraGUI::on_refreshCamerasButton_clicked);
    QObject::connect(ui->cameraCombo, &QComboBox::currentTextChanged, this, &CameraGUI::on_cameraCombo_currentTextChanged);
    QObject::connect(settingsUI()->resolutionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_resolutionCombo_currentIndexChanged);
    QObject::connect(settingsUI()->fpsLabel, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_fpsLabel_currentIndexChanged);
    QObject::connect(settingsUI()->fpsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_fpsSpin_valueChanged);
    QObject::connect(settingsUI()->fpsCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_fpsCombo_currentIndexChanged);
    QObject::connect(settingsUI()->intervalSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_intervalSpin_valueChanged);
    QObject::connect(settingsUI()->intervalUnitsCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_intervalUnitsCombo_currentIndexChanged);
    QObject::connect(settingsUI()->exposureSlider, &QSlider::valueChanged, this, &CameraGUI::on_exposureSlider_valueChanged);
    QObject::connect(settingsUI()->exposureSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_exposureSpin_valueChanged);
    QObject::connect(settingsUI()->exposureUnitsCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_exposureUnitsCombo_currentIndexChanged);
    QObject::connect(settingsUI()->isoSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_isoSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaHostEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_alpacaHostEdit_editingFinished);
    QObject::connect(settingsUI()->alpacaPortSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaPortSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaBinXSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaBinXSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaBinYSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaBinYSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaGainCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaGainCombo_currentIndexChanged);
    QObject::connect(settingsUI()->alpacaGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_alpacaGainSlider_valueChanged);
    QObject::connect(settingsUI()->alpacaGainSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaGainSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaOffsetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaOffsetCombo_currentIndexChanged);
    QObject::connect(settingsUI()->alpacaOffsetSlider, &QSlider::valueChanged, this, &CameraGUI::on_alpacaOffsetSlider_valueChanged);
    QObject::connect(settingsUI()->alpacaOffsetSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaOffsetSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaReadoutModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaReadoutModeCombo_currentIndexChanged);
    QObject::connect(ui->saveImageCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveImageCheck_toggled);
    QObject::connect(settingsUI()->imagePathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_imagePathEdit_editingFinished);
    QObject::connect(settingsUI()->imagePathButton, &QToolButton::clicked, this, &CameraGUI::on_imagePathButton_clicked);
    QObject::connect(ui->saveVideoCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveVideoCheck_toggled);
    QObject::connect(settingsUI()->videoPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_videoPathEdit_editingFinished);
    QObject::connect(settingsUI()->videoPathButton, &QToolButton::clicked, this, &CameraGUI::on_videoPathButton_clicked);
    QObject::connect(settingsUI()->videoPostProcessCombo, &QComboBox::currentIndexChanged, this, &CameraGUI::on_videoPostProcessCombo_currentIndexChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_postProcessWhiteBalanceModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceRedGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceRedGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceRedGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceRedGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceGreenGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceGreenGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceGreenGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceGreenGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceBlueGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceBlueGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceBlueGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceBlueGainSpin_valueChanged);
    QObject::connect(settingsUI()->saturationSlider, &QSlider::valueChanged, this, &CameraGUI::on_saturationSlider_valueChanged);
    QObject::connect(settingsUI()->saturationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_saturationSpin_valueChanged);
    QObject::connect(settingsUI()->gammaSlider, &QSlider::valueChanged, this, &CameraGUI::on_gammaSlider_valueChanged);
    QObject::connect(settingsUI()->gammaSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_gammaSpin_valueChanged);
    QObject::connect(settingsUI()->gaussianBlurSlider, &QSlider::valueChanged, this, &CameraGUI::on_gaussianBlurSlider_valueChanged);
    QObject::connect(settingsUI()->gaussianBlurSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_gaussianBlurSpin_valueChanged);
    QObject::connect(settingsUI()->medianBlurSlider, &QSlider::valueChanged, this, &CameraGUI::on_medianBlurSlider_valueChanged);
    QObject::connect(settingsUI()->medianBlurSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_medianBlurSpin_valueChanged);
    QObject::connect(settingsUI()->sharpenSlider, &QSlider::valueChanged, this, &CameraGUI::on_sharpenSlider_valueChanged);
    QObject::connect(settingsUI()->sharpenSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_sharpenSpin_valueChanged);
    QObject::connect(settingsUI()->sobelEdgeSlider, &QSlider::valueChanged, this, &CameraGUI::on_sobelEdgeSlider_valueChanged);
    QObject::connect(settingsUI()->sobelEdgeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_sobelEdgeSpin_valueChanged);
    QObject::connect(settingsUI()->flipXButton, &QCheckBox::toggled, this, &CameraGUI::on_flipXButton_toggled);
    QObject::connect(settingsUI()->flipYButton, &QCheckBox::toggled, this, &CameraGUI::on_flipYButton_toggled);
    QObject::connect(settingsUI()->brightnessSlider, &QSlider::valueChanged, this, &CameraGUI::on_brightnessSlider_valueChanged);
    QObject::connect(settingsUI()->brightnessSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_brightnessSpin_valueChanged);
    QObject::connect(settingsUI()->contrastSlider, &QSlider::valueChanged, this, &CameraGUI::on_contrastSlider_valueChanged);
    QObject::connect(settingsUI()->contrastSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_contrastSpin_valueChanged);
    QObject::connect(ui->invertColorsButton, &QToolButton::toggled, this, &CameraGUI::on_invertColorsButton_toggled);
    QObject::connect(ui->overlayDateTimeButton, &QToolButton::toggled, this, &CameraGUI::on_overlayDateTimeButton_toggled);
    QObject::connect(settingsUI()->dateTimeColorButton, &QToolButton::clicked, this, &CameraGUI::on_dateTimeColorButton_clicked);
    QObject::connect(settingsUI()->dateTimeFormatEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_dateTimeFormatEdit_editingFinished);
    QObject::connect(settingsUI()->dateTimePosXSlider, &QSlider::valueChanged, this, &CameraGUI::on_dateTimePosXSlider_valueChanged);
    QObject::connect(settingsUI()->dateTimePosYSlider, &QSlider::valueChanged, this, &CameraGUI::on_dateTimePosYSlider_valueChanged);
    QObject::connect(ui->overlayTextButton, &QToolButton::toggled, this, &CameraGUI::on_overlayTextButton_toggled);
    QObject::connect(settingsUI()->overlayTextColorButton, &QToolButton::clicked, this, &CameraGUI::on_overlayTextColorButton_clicked);
    QObject::connect(settingsUI()->overlayTextEdit, &QTextEdit::textChanged, this, &CameraGUI::on_overlayTextEdit_textChanged);
    QObject::connect(settingsUI()->overlayTextPosXSlider, &QSlider::valueChanged, this, &CameraGUI::on_overlayTextPosXSlider_valueChanged);
    QObject::connect(settingsUI()->overlayTextPosYSlider, &QSlider::valueChanged, this, &CameraGUI::on_overlayTextPosYSlider_valueChanged);
    QObject::connect(ui->diffMaskButton, &QToolButton::toggled, this, &CameraGUI::on_diffMaskButton_toggled);
    QObject::connect(settingsUI()->diffThresholdSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_diffThresholdSpin_valueChanged);
    QObject::connect(settingsUI()->dilationSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_dilationSpin_valueChanged);
    QObject::connect(settingsUI()->diffMaskHistoryFramesSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_diffMaskHistoryFramesSpin_valueChanged);
    QObject::connect(ui->histogramButton, &QToolButton::clicked, this, &CameraGUI::on_histogramButton_clicked);
    QObject::connect(settingsUI()->defaultColorSettingsButton, &QToolButton::clicked, this, &CameraGUI::on_defaultColorSettingsButton_clicked);
    QObject::connect(settingsUI()->overlayFontCombo, &QFontComboBox::currentFontChanged, this, &CameraGUI::on_overlayFontCombo_currentFontChanged);
    QObject::connect(settingsUI()->overlayFontScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_overlayFontScaleSpin_valueChanged);
    QObject::connect(settingsUI()->overlayTextFontCombo, &QFontComboBox::currentFontChanged, this, &CameraGUI::on_overlayTextFontCombo_currentFontChanged);
    QObject::connect(settingsUI()->overlayTextFontScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_overlayTextFontScaleSpin_valueChanged);
    QObject::connect(ui->motionDetectButton, &QToolButton::toggled, this, &CameraGUI::on_motionDetectButton_toggled);
    QObject::connect(settingsUI()->minContourAreaSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_minContourAreaSpin_valueChanged);
    QObject::connect(settingsUI()->motionBoxColorButton, &QToolButton::clicked, this, &CameraGUI::on_motionBoxColorButton_clicked);
    QObject::connect(ui->spectrumOverlayButton, &QToolButton::toggled, this, &CameraGUI::on_spectrumOverlayButton_toggled);
    QObject::connect(settingsUI()->spectrumDeviceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_spectrumDeviceCombo_currentIndexChanged);
    QObject::connect(settingsUI()->spectrumOffsetXSlider, &QSlider::valueChanged, this, &CameraGUI::on_spectrumOffsetXSlider_valueChanged);
    QObject::connect(settingsUI()->spectrumOffsetYSlider, &QSlider::valueChanged, this, &CameraGUI::on_spectrumOffsetYSlider_valueChanged);
    QObject::connect(settingsUI()->spectrumScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_spectrumScaleSpin_valueChanged);
    QObject::connect(ui->yoloButton, &QToolButton::toggled, this, &CameraGUI::on_yoloButton_toggled);
    QObject::connect(settingsUI()->yoloModelPathCombo, &QComboBox::currentTextChanged, this, &CameraGUI::on_yoloModelPathCombo_currentTextChanged);
    QObject::connect(settingsUI()->yoloModelPathButton, &QPushButton::clicked, this, &CameraGUI::on_yoloModelPathButton_clicked);
    QObject::connect(settingsUI()->yoloLabelsPathCombo, &QComboBox::currentTextChanged, this, &CameraGUI::on_yoloLabelsPathCombo_currentTextChanged);
    QObject::connect(settingsUI()->yoloLabelsPathButton, &QPushButton::clicked, this, &CameraGUI::on_yoloLabelsPathButton_clicked);
    QObject::connect(settingsUI()->yoloTargetCombo, &QComboBox::currentIndexChanged, this, &CameraGUI::on_yoloTargetCombo_currentIndexChanged);
    QObject::connect(settingsUI()->actionsClassCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_actionsClassCombo_currentIndexChanged);
    QObject::connect(settingsUI()->actionsDisappearDebounceSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_actionsDisappearDebounceSpin_valueChanged);
    QObject::connect(settingsUI()->actionsAddButton, &QPushButton::clicked, this, &CameraGUI::on_actionsAddButton_clicked);
    QObject::connect(settingsUI()->actionsTabWidget, &QTabWidget::tabCloseRequested, this, &CameraGUI::on_actionsTabWidget_tabCloseRequested);
    QObject::connect(settingsUI()->yoloConfSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_yoloConfSpin_valueChanged);
    QObject::connect(settingsUI()->yoloNmsSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_yoloNmsSpin_valueChanged);
    QObject::connect(settingsUI()->yoloBoxColorButton, &QToolButton::clicked, this, &CameraGUI::on_yoloBoxColorButton_clicked);
    QObject::connect(ui->zoomInButton, &QToolButton::clicked, this, &CameraGUI::on_zoomInButton_clicked);
    QObject::connect(ui->zoomOutButton, &QToolButton::clicked, this, &CameraGUI::on_zoomOutButton_clicked);
    QObject::connect(ui->fitInViewButton, &QToolButton::clicked, this, &CameraGUI::on_fitInViewButton_clicked);
    QObject::connect(ui->audioMute, &QToolButton::toggled, this, &CameraGUI::on_audioMute_toggled);
    QObject::connect(settingsUI()->whiteBalanceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_whiteBalanceCombo_currentIndexChanged);
    QObject::connect(settingsUI()->exposureCompSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_exposureCompSpin_valueChanged);
    QObject::connect(settingsUI()->focusModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_focusModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->focusDistSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_focusDistSpin_valueChanged);
    QObject::connect(settingsUI()->zoomSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_zoomSpin_valueChanged);
}

void CameraGUI::reportResolutions()
{
    QList<QSize> resolutions;
    QHash<QString, FrameRateOptions> frameRateOptionsByResolution;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_settings.isQtCamera())
    {
        const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
        const QString targetId = m_settings.cameraIdString();
        const QString targetDescription = m_settings.cameraDescription();

        for (const QCameraDevice& device : cameras)
        {
            const QString id = QString::fromUtf8(device.id());
            if ((id == targetId) || (device.description() == targetDescription))
            {
                QHash<QString, QSet<int>> fpsByResolution;
                QSet<QString> seen;

                for (const QCameraFormat& format : device.videoFormats())
                {
                    const QSize size = format.resolution();
                    const QString key = resolutionKey(size);
                    if (!seen.contains(key))
                    {
                        seen.insert(key);
                        resolutions.append(size);
                    }

                    appendFpsRange(fpsByResolution[key], format.minFrameRate(), format.maxFrameRate());
                }

                for (auto it = fpsByResolution.cbegin(); it != fpsByResolution.cend(); ++it) {
                    frameRateOptionsByResolution.insert(it.key(), makeFrameRateOptions(it.value()));
                }

                break;
            }
        }
    }
#else
    if (m_settings.isQtCamera())
    {
        const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
        const QString targetId = m_settings.cameraIdString();
        const QString targetDescription = m_settings.cameraDescription();

        for (const QCameraInfo& info : cameras)
        {
            const QString id = info.deviceName();
            if ((id == targetId) || (info.description() == targetDescription))
            {
                QCamera tempCam(info);
                QHash<QString, QSet<int>> fpsByResolution;
                QSet<QString> seen;

                const QList<QCameraViewfinderSettings> viewfinderSettings = tempCam.supportedViewfinderSettings();

                for (const QCameraViewfinderSettings& vfSettings : viewfinderSettings)
                {
                    const QSize size = vfSettings.resolution();
                    if (!size.isValid()) {
                        continue;
                    }

                    const QString key = resolutionKey(size);
                    if (!seen.contains(key))
                    {
                        seen.insert(key);
                        resolutions.append(size);
                    }

                    appendFpsRange(fpsByResolution[key], vfSettings.minimumFrameRate(), vfSettings.maximumFrameRate());
                }

                if (resolutions.isEmpty())
                {
                    for (const QSize& res : tempCam.supportedViewfinderResolutions())
                    {
                        const QString key = resolutionKey(res);
                        if (!seen.contains(key))
                        {
                            seen.insert(key);
                            resolutions.append(res);
                        }
                        fpsByResolution[key].insert(qMax(1, m_settings.m_framesPerSecond));
                    }
                }

                for (auto it = fpsByResolution.cbegin(); it != fpsByResolution.cend(); ++it) {
                    frameRateOptionsByResolution.insert(it.key(), makeFrameRateOptions(it.value()));
                }

                break;
            }
        }
    }
#endif

    populateQtFormatControls(resolutions, frameRateOptionsByResolution);
}

void CameraGUI::populateQtFormatControls(const QList<QSize>& resolutions, const QHash<QString, FrameRateOptions>& frameRateOptionsByResolution)
{
    m_qtFrameRateOptionsByResolution = frameRateOptionsByResolution;

    const QString current = resolutionKey(m_settings.m_resolutionWidth, m_settings.m_resolutionHeight);
    QSignalBlocker blocker(settingsUI()->resolutionCombo);
    settingsUI()->resolutionCombo->clear();

    for (const QSize& size : resolutions) {
        settingsUI()->resolutionCombo->addItem(resolutionKey(size));
    }

    const int idx = settingsUI()->resolutionCombo->findText(current);
    if (idx >= 0) {
        settingsUI()->resolutionCombo->setCurrentIndex(idx);
    } else if (settingsUI()->resolutionCombo->count() > 0) {
        settingsUI()->resolutionCombo->setCurrentIndex(0);
    }

    updateFrameRateControlForResolution(settingsUI()->resolutionCombo->currentText());
}

void CameraGUI::updateFrameRateControlForResolution(const QString& resolutionText)
{
    const FrameRateOptions options = m_qtFrameRateOptionsByResolution.value(
        resolutionText,
        FrameRateOptions{true, 1, 240, QList<int>{m_settings.m_framesPerSecond}});

    const int desiredFps = m_settings.m_framesPerSecond;

    if (options.contiguous)
    {
        const int clampedFps = qBound(options.minFps, desiredFps, options.maxFps);
        m_settings.m_framesPerSecond = clampedFps;

        QSignalBlocker blockSpin(settingsUI()->fpsSpin);
        settingsUI()->fpsSpin->setMinimum(options.minFps);
        settingsUI()->fpsSpin->setMaximum(options.maxFps);
        settingsUI()->fpsSpin->setValue(clampedFps);
        settingsUI()->fpsStack->setCurrentWidget(settingsUI()->fpsSpinPage);
    }
    else
    {
        QSignalBlocker blockCombo(settingsUI()->fpsCombo);
        settingsUI()->fpsCombo->clear();
        for (int fps : options.values) {
            settingsUI()->fpsCombo->addItem(QString::number(fps), fps);
        }

        int selectedIndex = settingsUI()->fpsCombo->findData(desiredFps);
        if (selectedIndex < 0 && settingsUI()->fpsCombo->count() > 0) {
            selectedIndex = 0;
        }
        if (selectedIndex >= 0) {
            settingsUI()->fpsCombo->setCurrentIndex(selectedIndex);
            m_settings.m_framesPerSecond = settingsUI()->fpsCombo->currentData().toInt();
        }
        settingsUI()->fpsStack->setCurrentWidget(settingsUI()->fpsComboPage);
    }
}

void CameraGUI::updateCaptureModeControls()
{
    const bool intervalMode = m_settings.isAlpacaCamera() || m_settings.isIntervalCaptureMode();
    settingsUI()->captureValueStack->setCurrentWidget(intervalMode ? settingsUI()->intervalPage : settingsUI()->frameRatePage);
}

void CameraGUI::updateExposureControls()
{
    const double unitScaleMs = currentExposureUnitScaleMs(settingsUI());
    const double minimum = m_exposureMinimumMs / unitScaleMs;
    const double maximum = m_exposureMaximumMs / unitScaleMs;
    const double singleStep = std::max(0.000001, m_exposureStepMs / unitScaleMs);
    const double value = qBound(minimum, m_settings.m_exposureTimeMs / unitScaleMs, maximum);
    const double sliderMaximumValue = std::min(maximum, 10000.0);

    {
        QSignalBlocker blocker(settingsUI()->exposureSpin);
        settingsUI()->exposureSpin->setDecimals(decimalsForStepSize(singleStep));
        settingsUI()->exposureSpin->setSingleStep(singleStep);
        settingsUI()->exposureSpin->setMinimum(minimum);
        settingsUI()->exposureSpin->setMaximum(maximum);
        settingsUI()->exposureSpin->setValue(value);
    }

    {
        QSignalBlocker blocker(settingsUI()->exposureSlider);
        settingsUI()->exposureSlider->setMinimum(0);
        settingsUI()->exposureSlider->setMaximum(std::max(
            0,
            static_cast<int>(std::llround((sliderMaximumValue - minimum) / singleStep))));
        settingsUI()->exposureSlider->setValue(std::min(
            settingsUI()->exposureSlider->maximum(),
            doubleSpinBoxValueToSlider(settingsUI()->exposureSpin, value)));
    }
}

QStringList CameraGUI::loadActionObjectClasses() const
{
    QStringList classes;

    if (!m_settings.m_yoloLabelsPath.isEmpty() && !(m_settings.m_yoloLabelsPath.startsWith("http://") || m_settings.m_yoloLabelsPath.startsWith("https://")))
    {
        QFile labelsFile(m_settings.m_yoloLabelsPath);
        if (labelsFile.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream textStream(&labelsFile);
            while (!textStream.atEnd())
            {
                const QString line = textStream.readLine().trimmed();
                if (!line.isEmpty() && !classes.contains(line)) {
                    classes.append(line);
                }
            }
        }
    }

    for (auto it = m_settings.m_objectDeviceSettings.cbegin(); it != m_settings.m_objectDeviceSettings.cend(); ++it) {
        if (!classes.contains(it.key())) {
            classes.append(it.key());
        }
    }

    return classes;
}

void CameraGUI::saveCurrentActionClassSettings()
{
    for (CameraObjectDeviceSettingsGUI *gui : m_actionDeviceSettingsGUIs) {
        gui->accept();
    }
}

void CameraGUI::populateActionClasses()
{
    const QString currentClass = settingsUI()->actionsClassCombo->currentText();
    const QStringList classes = loadActionObjectClasses();

    QSignalBlocker blocker(settingsUI()->actionsClassCombo);
    settingsUI()->actionsClassCombo->clear();
    settingsUI()->actionsClassCombo->addItems(classes);

    const int index = settingsUI()->actionsClassCombo->findText(currentClass);
    if (index >= 0) {
        settingsUI()->actionsClassCombo->setCurrentIndex(index);
    } else if (settingsUI()->actionsClassCombo->count() > 0) {
        settingsUI()->actionsClassCombo->setCurrentIndex(0);
    }
}

void CameraGUI::rebuildActionTabsForCurrentClass()
{
    settingsUI()->actionsTabWidget->clear();
    qDeleteAll(m_actionDeviceSettingsGUIs);
    m_actionDeviceSettingsGUIs.clear();

    const QString className = settingsUI()->actionsClassCombo->currentText();
    if (className.isEmpty())
    {
        updateActionControls();
        return;
    }

    if (!m_settings.m_objectDeviceSettings.contains(className)) {
        m_settings.m_objectDeviceSettings.insert(className, new QList<CameraSettings::ObjectDeviceSettings *>());
    }

    QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
    for (CameraSettings::ObjectDeviceSettings *deviceSettings : *deviceSettingsList)
    {
        CameraObjectDeviceSettingsGUI *deviceSettingsGUI =
            new CameraObjectDeviceSettingsGUI(deviceSettings, settingsUI()->actionsTabWidget, settingsUI()->actionsTabWidget);
        const int index = settingsUI()->actionsTabWidget->addTab(deviceSettingsGUI, QString("R%1").arg(deviceSettings->m_deviceSetIndex));
        settingsUI()->actionsTabWidget->setCurrentIndex(index);
        m_actionDeviceSettingsGUIs.append(deviceSettingsGUI);
    }

    updateActionControls();
}

void CameraGUI::updateActionControls()
{
    const bool hasClasses = settingsUI()->actionsClassCombo->count() > 0;
    const bool hasClassSelection = !settingsUI()->actionsClassCombo->currentText().isEmpty();

    settingsUI()->actionsClassCombo->setEnabled(hasClasses);
    settingsUI()->actionsAddButton->setEnabled(hasClassSelection);
    settingsUI()->actionsTabWidget->setEnabled(hasClassSelection);

    if (!hasClasses)
    {
        if (m_settings.m_yoloLabelsPath.isEmpty()) {
            settingsUI()->actionsStatusLabel->setText(tr("Select a YOLO labels file first to configure per-class actions."));
        } else {
            settingsUI()->actionsStatusLabel->setText(tr("No class names could be loaded from the labels file."));
        }
    }
    else
    {
        settingsUI()->actionsStatusLabel->setText(
            tr("Configure what each device set should do when the selected class is detected or disappears."));
    }
}

void CameraGUI::applyActionSettings()
{
    saveCurrentActionClassSettings();
    m_settingsKeys.append("yoloDisappearDebounce");
    m_settingsKeys.append("objectDeviceSettings");
    applySettings();
}

// ---------------------------------------------------------------------------
// Qt camera capture — runs entirely on the GUI / main thread
// ---------------------------------------------------------------------------

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
CameraVideoSurface::CameraVideoSurface(QObject *parent)
    : QAbstractVideoSurface(parent)
{
}

QList<QVideoFrame::PixelFormat> CameraVideoSurface::supportedPixelFormats(
    QAbstractVideoBuffer::HandleType handleType) const
{
    Q_UNUSED(handleType);
    return {
        QVideoFrame::Format_ARGB32,
        QVideoFrame::Format_ARGB32_Premultiplied,
        QVideoFrame::Format_RGB32,
        QVideoFrame::Format_RGB24,
        QVideoFrame::Format_RGB565,
        QVideoFrame::Format_RGB555,
        QVideoFrame::Format_BGRA32,
        QVideoFrame::Format_BGR32,
        QVideoFrame::Format_BGR24,
        QVideoFrame::Format_YUV444,
        QVideoFrame::Format_YUV420P,
        QVideoFrame::Format_YV12,
        QVideoFrame::Format_UYVY,
        QVideoFrame::Format_YUYV,
        QVideoFrame::Format_NV12,
        QVideoFrame::Format_NV21,
    };
}

bool CameraVideoSurface::present(const QVideoFrame& frame)
{
    if (!frame.isValid()) {
        return false;
    }

    QVideoFrame mutableFrame(frame);
    mutableFrame.map(QAbstractVideoBuffer::ReadOnly);

    const QImage::Format imageFormat = QVideoFrame::imageFormatFromPixelFormat(mutableFrame.pixelFormat());
    QImage image;

    if (imageFormat != QImage::Format_Invalid)
    {
        image = QImage(
            mutableFrame.bits(),
            mutableFrame.width(),
            mutableFrame.height(),
            mutableFrame.bytesPerLine(),
            imageFormat
        ).copy();
    }

    mutableFrame.unmap();

    if (!image.isNull()) {
        emit frameAvailable(image);
    }

    return true;
}
#endif // Qt 5

void CameraGUI::setupQtCapture()
{
    cleanupQtCapture();
    m_qtStillCaptureTimer.stop();

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)

    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    if (cameras.isEmpty()) {
        return;
    }

    QCameraDevice selectedDevice = cameras.front();
    for (const QCameraDevice& device : cameras)
    {
        const QString id = QString::fromUtf8(device.id());
        if ((id == m_settings.m_cameraId) || (device.description() == m_settings.m_cameraId)) {
            selectedDevice = device;
            break;
        }
    }

    reportResolutions();

    m_captureSession = new QMediaCaptureSession(this);
    m_qtCamera       = new QCamera(selectedDevice, this);
    m_imageCapture   = nullptr;
    m_videoSink      = nullptr;

    // Select a matching camera format if one exists
    QCameraFormat chosenFormat;
    for (const QCameraFormat& fmt : selectedDevice.videoFormats())
    {
        if ((fmt.resolution().width()  == m_settings.m_resolutionWidth)
         && (fmt.resolution().height() == m_settings.m_resolutionHeight)
            && (fmt.maxFrameRate()     >= m_settings.m_framesPerSecond)
            && (fmt.minFrameRate()     <= m_settings.m_framesPerSecond)
            )
        {
            chosenFormat = fmt;
            break;
        }
    }
    if (!chosenFormat.isNull()) {
        m_qtCamera->setCameraFormat(chosenFormat);
    } else {
        qWarning() << "CameraGUI::setupQtCapture: No matching camera format"
            << m_settings.m_resolutionWidth
            << m_settings.m_resolutionHeight
            << m_settings.m_framesPerSecond;
    }

    m_qtCamera->setExposureMode(QCamera::ExposureManual);
    m_qtCamera->setManualExposureTime(static_cast<float>(m_settings.m_exposureTimeMs) / 1000.0f);
    m_qtCamera->setManualIsoSensitivity(m_settings.m_isoSensitivity);
    m_qtCamera->setWhiteBalanceMode(static_cast<QCamera::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
    m_qtCamera->setExposureCompensation(static_cast<float>(m_settings.m_exposureCompensation));
    m_qtCamera->setFocusMode(static_cast<QCamera::FocusMode>(m_settings.m_focusMode));
    m_qtCamera->setFocusDistance(static_cast<float>(m_settings.m_focusDistance));

    m_captureSession->setCamera(m_qtCamera);

    if (m_settings.isIntervalCaptureMode())
    {
        m_imageCapture = new QImageCapture(this);
        m_captureSession->setImageCapture(m_imageCapture);
        connect(m_imageCapture, &QImageCapture::imageCaptured, this, &CameraGUI::onQtImageCaptured);
        connect(m_imageCapture, &QImageCapture::errorOccurred, this,
                [this](int id, QImageCapture::Error error, const QString& errorString)
                {
                    Q_UNUSED(id)
                    Q_UNUSED(error)
                    qWarning() << "CameraGUI::setupQtCapture: image capture error:" << errorString;
                });
    }
    else
    {
        m_videoSink = new QVideoSink(this);
        m_captureSession->setVideoOutput(m_videoSink);
        connect(m_videoSink, &QVideoSink::videoFrameChanged, this, &CameraGUI::onQtVideoFrame);
    }

    m_qtCamera->start();
    if (m_settings.isIntervalCaptureMode()) {
        m_qtStillCaptureTimer.start(m_settings.getCaptureIntervalMs());
    }

    // Probe capabilities after start
    const QCamera::Features cameraFeatures = m_qtCamera->supportedFeatures();
    m_qtManualExposureSupported = cameraFeatures.testFlag(QCamera::Feature::ManualExposureTime);
    m_qtIsoSensitivitySupported = cameraFeatures.testFlag(QCamera::Feature::IsoSensitivity);
    m_qtWhiteBalanceModeSupported = cameraFeatures.testFlag(QCamera::Feature::ColorTemperature);
    m_qtExposureCompensationSupported = cameraFeatures.testFlag(QCamera::Feature::ExposureCompensation);

    const float minZoom = m_qtCamera->minimumZoomFactor();
    const float maxZoom = m_qtCamera->maximumZoomFactor();
    m_qtZoomSupported = (maxZoom > minZoom + 0.01f);

    blockApplySettings(true);
    settingsUI()->zoomSpin->setMinimum(minZoom);
    settingsUI()->zoomSpin->setMaximum(maxZoom > minZoom ? maxZoom : minZoom);
    settingsUI()->zoomSpin->setEnabled(m_qtZoomSupported);
    settingsUI()->zoomLabel->setEnabled(m_qtZoomSupported);
    settingsUI()->exposureLabel->setEnabled(m_qtManualExposureSupported);
    settingsUI()->exposureSlider->setEnabled(m_qtManualExposureSupported);
    settingsUI()->exposureSpin->setEnabled(m_qtManualExposureSupported);
    settingsUI()->exposureUnitsCombo->setEnabled(m_qtManualExposureSupported);
    settingsUI()->isoLabel->setEnabled(m_qtIsoSensitivitySupported);
    settingsUI()->isoSpin->setEnabled(m_qtIsoSensitivitySupported);
    settingsUI()->whiteBalanceLabel->setEnabled(m_qtWhiteBalanceModeSupported);
    settingsUI()->whiteBalanceCombo->setEnabled(m_qtWhiteBalanceModeSupported);
    settingsUI()->exposureCompLabel->setEnabled(m_qtExposureCompensationSupported);
    settingsUI()->exposureCompSpin->setEnabled(m_qtExposureCompensationSupported);
    blockApplySettings(false);

    // Clamp and apply zoom
    const float clampedZoom = qBound(minZoom, static_cast<float>(m_settings.m_zoomFactor), maxZoom);
    m_qtCamera->setZoomFactor(clampedZoom);

    // Populate focus mode combo with supported modes
    {
        static const QList<int> kFocusModes = {
            static_cast<int>(QCamera::FocusModeAuto),
            static_cast<int>(QCamera::FocusModeAutoNear),
            static_cast<int>(QCamera::FocusModeAutoFar),
            static_cast<int>(QCamera::FocusModeHyperfocal),
            static_cast<int>(QCamera::FocusModeInfinity),
            static_cast<int>(QCamera::FocusModeManual),
        };
        QList<int> supportedModes;
        for (int mode : kFocusModes) {
            if ((mode == static_cast<int>(QCamera::FocusModeManual))
             && !cameraFeatures.testFlag(QCamera::Feature::FocusDistance))
            {
                continue;
            }

            if (m_qtCamera->isFocusModeSupported(static_cast<QCamera::FocusMode>(mode))) {
                supportedModes.append(mode);
            }
        }
        blockApplySettings(true);
        for (int i = 0; i < settingsUI()->focusModeCombo->count(); ++i)
        {
            const int modeVal = settingsUI()->focusModeCombo->itemData(i).toInt();
            const QStandardItemModel *model = qobject_cast<QStandardItemModel*>(settingsUI()->focusModeCombo->model());
            if (model) {
                QStandardItem *item = model->item(i);
                if (item) {
                    item->setEnabled(supportedModes.contains(modeVal));
                }
            }
        }
        if (!supportedModes.isEmpty() && !supportedModes.contains(m_settings.m_focusMode))
        {
            m_settings.m_focusMode = supportedModes.first();
            const int idx = settingsUI()->focusModeCombo->findData(m_settings.m_focusMode);
            if (idx >= 0) {
                settingsUI()->focusModeCombo->setCurrentIndex(idx);
            }
        }
        blockApplySettings(false);
    }

#else // Qt 5

    const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    if (cameras.isEmpty()) {
        return;
    }

    QCameraInfo selectedInfo = cameras.front();
    for (const QCameraInfo& info : cameras)
    {
        const QString id = info.deviceName();
        if ((id == m_settings.m_cameraId) || (info.description() == m_settings.m_cameraId))
        {
            selectedInfo = info;
            break;
        }
    }

    m_qtCamera     = new QCamera(selectedInfo, this);
    m_imageCapture = nullptr;
    m_videoSurface = nullptr;

    if (m_settings.isIntervalCaptureMode())
    {
        m_imageCapture = new QCameraImageCapture(m_qtCamera, this);
        m_imageCapture->setCaptureDestination(QCameraImageCapture::CaptureToBuffer);
        connect(m_imageCapture, QOverload<int, const QImage&>::of(&QCameraImageCapture::imageCaptured),
                this, &CameraGUI::onQtImageCaptured);
        connect(m_imageCapture,
                QOverload<int, QCameraImageCapture::Error, const QString&>::of(&QCameraImageCapture::error),
                this,
                [this](int id, QCameraImageCapture::Error error, const QString& errorString)
                {
                    Q_UNUSED(id)
                    Q_UNUSED(error)
                    qWarning() << "CameraGUI::setupQtCapture: image capture error:" << errorString;
                });
    }
    else
    {
        m_videoSurface = new CameraVideoSurface(this);
        m_qtCamera->setViewfinder(m_videoSurface);
    }

    if (m_settings.m_resolutionWidth > 0 && m_settings.m_resolutionHeight > 0)
    {
        QCameraViewfinderSettings vfSettings;
        vfSettings.setResolution(m_settings.m_resolutionWidth, m_settings.m_resolutionHeight);
        vfSettings.setMaximumFrameRate(m_settings.m_framesPerSecond);
        m_qtCamera->setViewfinderSettings(vfSettings);
    }

    QCameraExposure *exposure = m_qtCamera->exposure();
    if (exposure)
    {
        exposure->setExposureMode(QCameraExposure::ExposureManual);
        exposure->setManualShutterSpeed(static_cast<qreal>(m_settings.m_exposureTimeMs) / 1000.0);
        exposure->setManualIsoSensitivity(m_settings.m_isoSensitivity);
    }

    QCameraImageProcessing *imageProcessing = m_qtCamera->imageProcessing();
    if (imageProcessing) {
        imageProcessing->setWhiteBalanceMode(
            static_cast<QCameraImageProcessing::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
    }

    if (m_videoSurface)
    {
        // Queued connection: present() may be called from the camera's internal thread
        connect(m_videoSurface, &CameraVideoSurface::frameAvailable,
                this, &CameraGUI::onQt5VideoFrame, Qt::QueuedConnection);
    }

    m_qtCamera->start();
    if (m_settings.isIntervalCaptureMode()) {
        m_qtStillCaptureTimer.start(m_settings.getCaptureIntervalMs());
    }

    // Probe capabilities
    {
        QCameraFocus *cameraFocus = m_qtCamera->focus();
        const qreal minZoom = 1.0;
        const qreal maxZoom = cameraFocus ? cameraFocus->maximumOpticalZoom() : 1.0;
        m_qtZoomSupported = (maxZoom > minZoom + 0.01);

        QCameraExposure *exp = m_qtCamera->exposure();
        m_qtManualExposureSupported = (exp != nullptr);
        m_qtIsoSensitivitySupported = exp && !exp->supportedIsoSensitivities().isEmpty();

        QCameraImageProcessing *ip = m_qtCamera->imageProcessing();
        m_qtWhiteBalanceModeSupported =
            ip && ip->isWhiteBalanceModeSupported(QCameraImageProcessing::WhiteBalanceAuto);

        blockApplySettings(true);
        settingsUI()->zoomSpin->setMinimum(minZoom);
        settingsUI()->zoomSpin->setMaximum(maxZoom > minZoom ? maxZoom : minZoom);
        settingsUI()->zoomSpin->setEnabled(m_qtZoomSupported);
        settingsUI()->zoomLabel->setEnabled(m_qtZoomSupported);
        settingsUI()->exposureLabel->setEnabled(m_qtManualExposureSupported);
        settingsUI()->exposureSlider->setEnabled(m_qtManualExposureSupported);
        settingsUI()->exposureSpin->setEnabled(m_qtManualExposureSupported);
        settingsUI()->exposureUnitsCombo->setEnabled(m_qtManualExposureSupported);
        settingsUI()->isoLabel->setEnabled(m_qtIsoSensitivitySupported);
        settingsUI()->isoSpin->setEnabled(m_qtIsoSensitivitySupported);
        settingsUI()->whiteBalanceLabel->setEnabled(m_qtWhiteBalanceModeSupported);
        settingsUI()->whiteBalanceCombo->setEnabled(m_qtWhiteBalanceModeSupported);
        blockApplySettings(false);

        if (cameraFocus && maxZoom > minZoom) {
            const qreal clampedZoom = qBound(minZoom, static_cast<qreal>(m_settings.m_zoomFactor), maxZoom);
            cameraFocus->zoomTo(clampedZoom, 1.0);
        }
    }

#endif // Qt version
}

void CameraGUI::cleanupQtCapture()
{
    m_qtStillCaptureTimer.stop();
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_imageCapture) {
        delete m_imageCapture;
        m_imageCapture = nullptr;
    }
    if (m_videoSink) {
        delete m_videoSink;
        m_videoSink = nullptr;
    }
    if (m_captureSession) {
        delete m_captureSession;
        m_captureSession = nullptr;
    }
    if (m_qtCamera) {
        m_qtCamera->stop();
        delete m_qtCamera;
        m_qtCamera = nullptr;
    }
#else
    if (m_imageCapture) {
        delete m_imageCapture;
        m_imageCapture = nullptr;
    }
    if (m_videoSurface) {
        delete m_videoSurface;
        m_videoSurface = nullptr;
    }
    if (m_qtCamera) {
        m_qtCamera->stop();
        delete m_qtCamera;
        m_qtCamera = nullptr;
    }
#endif
}

void CameraGUI::applyQtCameraSettings(const QList<QString>& settingsKeys, bool force)
{
    if (!m_settings.isQtCamera())
    {
        // Camera type switched away from Qt — stop any running Qt camera
        if (m_qtCamera) {
            cleanupQtCapture();
        }
        m_qtFrameRateOptionsByResolution.clear();
        settingsUI()->fpsStack->setCurrentWidget(settingsUI()->fpsSpinPage);
        updateCaptureModeControls();
        settingsUI()->fpsSpin->setMinimum(1);
        settingsUI()->fpsSpin->setMaximum(240);
        settingsUI()->fpsSpin->setValue(m_settings.m_framesPerSecond);
        return;
    }

    if (force || settingsKeys.contains("cameraId")) {
        reportResolutions();
    }

    if (force || settingsKeys.contains("resolutionWidth") || settingsKeys.contains("resolutionHeight")) {
        updateFrameRateControlForResolution(resolutionKey(m_settings.m_resolutionWidth, m_settings.m_resolutionHeight));
    }

    // Decide whether a full restart is needed
    updateCaptureModeControls();

    const bool recapture = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("captureMode")
        || settingsKeys.contains("captureInterval")
        || settingsKeys.contains("captureIntervalUnits")
        || settingsKeys.contains("framesPerSecond")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("isoSensitivity");

    if (!m_qtCamera && (m_camera->getState() == Feature::StRunning))
    {
        // Start the camera (we've probably just switched to Qt camera type)
        setupQtCapture();
    }
    else if (recapture && m_qtCamera)
    {
        // Restart the camera so the new format / exposure parameters take effect
        setupQtCapture();
    }
    else if (m_qtCamera)
    {
        // Apply inline settings that don't require a camera restart
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        if (force || settingsKeys.contains("whiteBalanceMode")) {
            m_qtCamera->setWhiteBalanceMode(static_cast<QCamera::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
        }
        if (force || settingsKeys.contains("exposureCompensation")) {
            m_qtCamera->setExposureCompensation(static_cast<float>(m_settings.m_exposureCompensation));
        }
        if (force || settingsKeys.contains("focusMode")) {
            m_qtCamera->setFocusMode(static_cast<QCamera::FocusMode>(m_settings.m_focusMode));
        }
        if (force || settingsKeys.contains("focusDistance")) {
            m_qtCamera->setFocusDistance(static_cast<float>(m_settings.m_focusDistance));
        }
        if (force || settingsKeys.contains("zoomFactor")) {
            const float clampedZoom = static_cast<float>(
                qBound(m_qtCamera->minimumZoomFactor(),
                       static_cast<float>(m_settings.m_zoomFactor),
                       m_qtCamera->maximumZoomFactor()));
            m_qtCamera->setZoomFactor(clampedZoom);
        }
#else
        if (force || settingsKeys.contains("whiteBalanceMode"))
        {
            QCameraImageProcessing *ip = m_qtCamera->imageProcessing();
            if (ip) {
                ip->setWhiteBalanceMode(
                    static_cast<QCameraImageProcessing::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
            }
        }
        if (force || settingsKeys.contains("zoomFactor"))
        {
            QCameraFocus *cameraFocus = m_qtCamera->focus();
            if (cameraFocus) {
                cameraFocus->zoomTo(m_settings.m_zoomFactor, 1.0);
            }
        }
#endif
    }
}

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
void CameraGUI::onQtVideoFrame(const QVideoFrame& frame)
{
    const QImage image = frame.toImage();
    onQtImageCaptured(-1, image);
}
#else
void CameraGUI::onQt5VideoFrame(const QImage& image)
{
    onQtImageCaptured(-1, image);
}
#endif

void CameraGUI::onQtImageCaptured(int id, const QImage& image)
{
    Q_UNUSED(id)

    if (image.isNull()) {
        return;
    }

    MessageQueue *postProcessorMQ = m_camera->getPostProcessorInputMessageQueue();
    if (postProcessorMQ) {
        postProcessorMQ->push(CameraPostProcessor::MsgProcessFrame::create(image));
    }
}

void CameraGUI::triggerQtStillCapture()
{
    if (!m_settings.isQtCamera() || !m_settings.isIntervalCaptureMode() || !m_qtCamera || !m_imageCapture) {
        return;
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_imageCapture->isReadyForCapture()) {
        m_imageCapture->capture();
    }
#else
    if (m_imageCapture->isReadyForCapture()) {
        m_imageCapture->capture();
    }
#endif
}

void CameraGUI::updateAlpacaVisibility()
{
    const bool alpaca = m_settings.isAlpacaCamera();

    settingsUI()->resolutionLabel->setVisible(!alpaca);
    settingsUI()->resolutionCombo->setVisible(!alpaca);
    settingsUI()->fpsLabel->setEnabled(!alpaca);
    updateCaptureModeControls();
    if (alpaca || !m_settings.isIntervalCaptureMode()) {
        settingsUI()->fpsStack->setCurrentWidget(settingsUI()->fpsSpinPage);
    }
    settingsUI()->isoLabel->setVisible(!alpaca);
    settingsUI()->isoSpin->setVisible(!alpaca);
    settingsUI()->alpacaBinXLabel->setVisible(alpaca);
    settingsUI()->alpacaBinXSpin->setVisible(alpaca);
    settingsUI()->alpacaBinYLabel->setVisible(alpaca);
    settingsUI()->alpacaBinYSpin->setVisible(alpaca);
    settingsUI()->alpacaGainLabel->setVisible(alpaca);
    settingsUI()->alpacaGainCombo->setVisible(alpaca && m_alpacaHasNamedGains);
    settingsUI()->alpacaGainSlider->setVisible(alpaca && !m_alpacaHasNamedGains);
    settingsUI()->alpacaGainSpin->setVisible(alpaca && !m_alpacaHasNamedGains);
    settingsUI()->alpacaOffsetLabel->setVisible(alpaca);
    settingsUI()->alpacaOffsetCombo->setVisible(alpaca && m_alpacaHasNamedOffsets);
    settingsUI()->alpacaOffsetSlider->setVisible(alpaca && !m_alpacaHasNamedOffsets);
    settingsUI()->alpacaOffsetSpin->setVisible(alpaca && !m_alpacaHasNamedOffsets);
    settingsUI()->alpacaReadoutModeLabel->setVisible(alpaca);
    settingsUI()->alpacaReadoutModeCombo->setVisible(alpaca);
    settingsUI()->alpacaStatusGroup->setVisible(alpaca);
    ui->audioMute->setVisible(!alpaca);

    // Qt-camera-only controls
    settingsUI()->whiteBalanceLabel->setVisible(!alpaca);
    settingsUI()->whiteBalanceCombo->setVisible(!alpaca);
    settingsUI()->zoomLabel->setVisible(!alpaca);
    settingsUI()->zoomSpin->setVisible(!alpaca);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    settingsUI()->exposureCompLabel->setVisible(!alpaca);
    settingsUI()->exposureCompSpin->setVisible(!alpaca);
    settingsUI()->focusModeLabel->setVisible(!alpaca);
    settingsUI()->focusModeCombo->setVisible(!alpaca);
    settingsUI()->focusDistLabel->setVisible(!alpaca);
    settingsUI()->focusDistSpin->setVisible(!alpaca);
#else
    settingsUI()->exposureCompLabel->setVisible(false);
    settingsUI()->exposureCompSpin->setVisible(false);
    settingsUI()->focusModeLabel->setVisible(false);
    settingsUI()->focusModeCombo->setVisible(false);
    settingsUI()->focusDistLabel->setVisible(false);
    settingsUI()->focusDistSpin->setVisible(false);
#endif
}


void CameraGUI::updateAlpacaCapabilities(const CameraWorker::MsgReportAlpacaCameraInfo& info)
{
    blockApplySettings(true);

    const double exposureMinMs = std::max(0.001, info.getExposureMinMs());
    const double exposureMaxMs = std::max(exposureMinMs, info.getExposureMaxMs());
    const double exposureResolutionMs = std::max(0.000001, info.getExposureResolutionMs());

    // Bin X
    settingsUI()->alpacaBinXSpin->setMaximum(std::max(1, info.getMaxBinX()));
    settingsUI()->alpacaBinXSpin->setValue(qBound(1, m_settings.m_alpacaBinX, info.getMaxBinX()));

    // Bin Y
    settingsUI()->alpacaBinYSpin->setMaximum(std::max(1, info.getMaxBinY()));
    settingsUI()->alpacaBinYSpin->setValue(qBound(1, m_settings.m_alpacaBinY, info.getMaxBinY()));

    // Gain
    m_alpacaHasNamedGains = !info.getGains().isEmpty();
    if (m_alpacaHasNamedGains)
    {
        settingsUI()->alpacaGainCombo->blockSignals(true);
        settingsUI()->alpacaGainCombo->clear();
        settingsUI()->alpacaGainCombo->addItems(info.getGains());
        const int gainIdx = (m_settings.m_alpacaGain >= 0 && m_settings.m_alpacaGain < info.getGains().size())
            ? m_settings.m_alpacaGain : 0;
        settingsUI()->alpacaGainCombo->setCurrentIndex(gainIdx);
        settingsUI()->alpacaGainCombo->blockSignals(false);
    }
    else
    {
        settingsUI()->alpacaGainSpin->setMinimum(info.getGainMin());
        settingsUI()->alpacaGainSlider->setMinimum(info.getGainMin());
        settingsUI()->alpacaGainSpin->setMaximum(std::max(info.getGainMin(), info.getGainMax()));
        settingsUI()->alpacaGainSlider->setMaximum(std::max(info.getGainMin(), info.getGainMax()));
        const int gainVal = (m_settings.m_alpacaGain >= 0) ? m_settings.m_alpacaGain : info.getGainMin();
        settingsUI()->alpacaGainSpin->setValue(qBound(info.getGainMin(), gainVal, info.getGainMax()));
        settingsUI()->alpacaGainSlider->setValue(qBound(info.getGainMin(), gainVal, info.getGainMax()));
    }

    // Readout mode
    settingsUI()->alpacaReadoutModeCombo->blockSignals(true);
    settingsUI()->alpacaReadoutModeCombo->clear();
    settingsUI()->alpacaReadoutModeCombo->addItems(info.getReadoutModes());
    if (m_settings.m_alpacaReadoutMode < info.getReadoutModes().size()) {
        settingsUI()->alpacaReadoutModeCombo->setCurrentIndex(m_settings.m_alpacaReadoutMode);
    }
    settingsUI()->alpacaReadoutModeCombo->blockSignals(false);

    // Offset
    m_alpacaHasNamedOffsets = !info.getOffsets().isEmpty();
    if (m_alpacaHasNamedOffsets)
    {
        settingsUI()->alpacaOffsetCombo->blockSignals(true);
        settingsUI()->alpacaOffsetCombo->clear();
        settingsUI()->alpacaOffsetCombo->addItems(info.getOffsets());
        const int offsetIdx = (m_settings.m_alpacaOffset >= 0 && m_settings.m_alpacaOffset < info.getOffsets().size())
            ? m_settings.m_alpacaOffset : 0;
        settingsUI()->alpacaOffsetCombo->setCurrentIndex(offsetIdx);
        settingsUI()->alpacaOffsetCombo->blockSignals(false);
    }
    else
    {
        settingsUI()->alpacaOffsetSpin->setMinimum(info.getOffsetMin());
        settingsUI()->alpacaOffsetSlider->setMinimum(info.getOffsetMin());
        settingsUI()->alpacaOffsetSpin->setMaximum(std::max(info.getOffsetMin(), info.getOffsetMax()));
        settingsUI()->alpacaOffsetSlider->setMaximum(std::max(info.getOffsetMin(), info.getOffsetMax()));
        const int offsetVal = (m_settings.m_alpacaOffset >= 0) ? m_settings.m_alpacaOffset : info.getOffsetMin();
        settingsUI()->alpacaOffsetSpin->setValue(qBound(info.getOffsetMin(), offsetVal, info.getOffsetMax()));
        settingsUI()->alpacaOffsetSlider->setValue(qBound(info.getOffsetMin(), offsetVal, info.getOffsetMax()));
    }

    m_exposureMinimumMs = exposureMinMs;
    m_exposureMaximumMs = exposureMaxMs;
    m_exposureStepMs = exposureResolutionMs;
    m_settings.m_exposureTimeMs = qBound(exposureMinMs, m_settings.m_exposureTimeMs, exposureMaxMs);
    updateExposureControls();

    // Status labels
    settingsUI()->sensorNameLabel->setText(info.getSensorName().isEmpty() ? "-" : info.getSensorName());

    static const QStringList sensorTypeNames = {
        "Monochrome", "Colour", "RGGB", "CMYG", "CMYG2", "LRGB"
    };
    const int st = info.getSensorType();
    settingsUI()->sensorTypeLabel->setText(
        (st >= 0 && st < sensorTypeNames.size()) ? sensorTypeNames[st] : QString::number(st));

    if (info.getPixelSizeX() > 0 || info.getPixelSizeY() > 0) {
        settingsUI()->pixelSizeLabel->setText(QString("%1 × %2")
            .arg(info.getPixelSizeX(), 0, 'f', 2)
            .arg(info.getPixelSizeY(), 0, 'f', 2));
    } else {
        settingsUI()->pixelSizeLabel->setText("-");
    }

    if (info.getCameraSizeX() > 0 || info.getCameraSizeY() > 0) {
        settingsUI()->cameraSizeLabel->setText(QString("%1 × %2").arg(info.getCameraSizeX()).arg(info.getCameraSizeY()));
    } else {
        settingsUI()->cameraSizeLabel->setText("-");
    }

    if (info.isCcdTemperatureValid()) {
        settingsUI()->ccdTempLabel->setText(QString::number(info.getCcdTemperature(), 'f', 1));
        m_settingsDialog->appendTemperatureSample(QDateTime::currentDateTime(), info.getCcdTemperature());
    } else {
        settingsUI()->ccdTempLabel->setText("-");
    }

    updateAlpacaVisibility();
    blockApplySettings(false);
}

void CameraGUI::on_startStop_clicked(bool checked)
{
    m_camera->getInputMessageQueue()->push(Camera::MsgStartStop::create(checked));
}

void CameraGUI::on_refreshCamerasButton_clicked()
{
    m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create());
}

void CameraGUI::on_cameraCombo_currentTextChanged(const QString& text)
{
    const bool wasAlpaca = m_settings.isAlpacaCamera();
    m_settings.m_cameraId = text;
    if (wasAlpaca != m_settings.isAlpacaCamera()) {
        m_settingsDialog->clearAlpacaStatus();
    }
    if (m_settings.isAlpacaCamera() && (m_settings.m_captureMode != CameraSettings::CaptureModeInterval))
    {
        m_settings.m_captureMode = CameraSettings::CaptureModeInterval;
        m_settingsKeys.append("captureMode");
        const int intervalIndex = settingsUI()->fpsLabel->findData(CameraSettings::CaptureModeInterval);
        if (intervalIndex >= 0) {
            QSignalBlocker blocker(settingsUI()->fpsLabel);
            settingsUI()->fpsLabel->setCurrentIndex(intervalIndex);
        }
    }
    m_settingsKeys.append("cameraId");
    updateAlpacaVisibility();
    updateEnabledControls();
    applySettings();
}

void CameraGUI::on_resolutionCombo_currentIndexChanged(int index)
{
    (void) index;
    const QStringList parts = settingsUI()->resolutionCombo->currentText().split('x');

    if (parts.size() == 2)
    {
        bool ok1 = false, ok2 = false;
        const int width = parts[0].trimmed().toInt(&ok1);
        const int height = parts[1].trimmed().toInt(&ok2);

        if (ok1 && ok2 && width > 0 && height > 0)
        {
            m_settings.m_resolutionWidth = width;
            m_settings.m_resolutionHeight = height;
            updateFrameRateControlForResolution(settingsUI()->resolutionCombo->currentText());
            m_settingsKeys.append("resolutionWidth");
            m_settingsKeys.append("resolutionHeight");
            m_settingsKeys.append("framesPerSecond");
            applySettings();
        }
    }
}

void CameraGUI::on_fpsLabel_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    if (m_settings.isAlpacaCamera()) {
        return;
    }

    m_settings.m_captureMode = static_cast<CameraSettings::CaptureMode>(settingsUI()->fpsLabel->itemData(index).toInt());
    updateCaptureModeControls();
    m_settingsKeys.append("captureMode");
    applySettings();
}

void CameraGUI::on_fpsSpin_valueChanged(int value)
{
    m_settings.m_framesPerSecond = value;
    m_settingsKeys.append("framesPerSecond");
    applySettings();
}

void CameraGUI::on_fpsCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_framesPerSecond = settingsUI()->fpsCombo->itemData(index).toInt();
    m_settingsKeys.append("framesPerSecond");
    applySettings();
}

void CameraGUI::on_intervalSpin_valueChanged(double value)
{
    m_settings.m_captureInterval = value;
    m_settingsKeys.append("captureInterval");
    applySettings();
}

void CameraGUI::on_intervalUnitsCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_captureIntervalUnits = static_cast<CameraSettings::CaptureIntervalUnits>(settingsUI()->intervalUnitsCombo->itemData(index).toInt());
    m_settingsKeys.append("captureIntervalUnits");
    applySettings();
}

void CameraGUI::on_exposureSlider_valueChanged(int value)
{
    const double exposureValue = sliderValueToDoubleSpinBox(settingsUI()->exposureSpin, value);
    const double exposureMs = exposureValue * currentExposureUnitScaleMs(settingsUI());
    settingsUI()->exposureSpin->blockSignals(true);
    settingsUI()->exposureSpin->setValue(exposureValue);
    settingsUI()->exposureSpin->blockSignals(false);
    m_settings.m_exposureTimeMs = exposureMs;
    m_settingsKeys.append("exposureTimeMs");
    applySettings();
}

void CameraGUI::on_exposureSpin_valueChanged(double value)
{
    settingsUI()->exposureSlider->blockSignals(true);
    settingsUI()->exposureSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->exposureSpin, value));
    settingsUI()->exposureSlider->blockSignals(false);
    m_settings.m_exposureTimeMs = value * currentExposureUnitScaleMs(settingsUI());
    m_settingsKeys.append("exposureTimeMs");
    applySettings();
}

void CameraGUI::on_exposureUnitsCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    updateExposureControls();
}

void CameraGUI::on_isoSpin_valueChanged(int value)
{
    m_settings.m_isoSensitivity = value;
    m_settingsKeys.append("isoSensitivity");
    applySettings();
}

void CameraGUI::on_alpacaHostEdit_editingFinished()
{
    m_settings.m_alpacaHost = settingsUI()->alpacaHostEdit->text();
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

void CameraGUI::on_alpacaGainSlider_valueChanged(int value)
{
    settingsUI()->alpacaGainSpin->blockSignals(true);
    settingsUI()->alpacaGainSpin->setValue(value);
    settingsUI()->alpacaGainSpin->blockSignals(false);
    m_settings.m_alpacaGain = value;
    m_settingsKeys.append("alpacaGain");
    applySettings();
}

void CameraGUI::on_alpacaGainSpin_valueChanged(int value)
{
    settingsUI()->alpacaGainSlider->blockSignals(true);
    settingsUI()->alpacaGainSlider->setValue(value);
    settingsUI()->alpacaGainSlider->blockSignals(false);
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

void CameraGUI::on_alpacaOffsetSlider_valueChanged(int value)
{
    settingsUI()->alpacaOffsetSpin->blockSignals(true);
    settingsUI()->alpacaOffsetSpin->setValue(value);
    settingsUI()->alpacaOffsetSpin->blockSignals(false);
    m_settings.m_alpacaOffset = value;
    m_settingsKeys.append("alpacaOffset");
    applySettings();
}

void CameraGUI::on_alpacaOffsetSpin_valueChanged(int value)
{
    settingsUI()->alpacaOffsetSlider->blockSignals(true);
    settingsUI()->alpacaOffsetSlider->setValue(value);
    settingsUI()->alpacaOffsetSlider->blockSignals(false);
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
    m_settings.m_imageFileName = settingsUI()->imagePathEdit->text();
    m_settingsKeys.append("imageFileName");
    applySettings();
    applyImagePath();
}

void CameraGUI::on_imagePathButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Save JPEG"), m_settings.m_imageFileName, tr("JPEG image (*.jpg *.jpeg)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_imageFileName = fileName;
        settingsUI()->imagePathEdit->setText(fileName);
        m_settingsKeys.append("imageFileName");
        applySettings();
        applyImagePath();
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
    m_settings.m_videoFileName = settingsUI()->videoPathEdit->text();
    m_settingsKeys.append("videoFileName");
    applySettings();
    applyVideoPath();
}

void CameraGUI::on_videoPathButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Save MP4"), m_settings.m_videoFileName, tr("MP4 video (*.mp4)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_videoFileName = fileName;
        settingsUI()->videoPathEdit->setText(fileName);
        m_settingsKeys.append("videoFileName");
        applySettings();
        applyVideoPath();
    }
}

void CameraGUI::on_videoPostProcessCombo_currentIndexChanged(int index)
{
    m_settings.m_videoPostProcess = static_cast<bool>(index);
    m_settingsKeys.append("videoPostProcess");
    applySettings();
}

void CameraGUI::updatePostProcessWhiteBalanceControls()
{
    const bool manual = m_settings.m_postProcessWhiteBalanceMode == 2;
    settingsUI()->postProcessWhiteBalanceRedGainSlider->setEnabled(manual);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->setEnabled(manual);
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->setEnabled(manual);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->setEnabled(manual);
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->setEnabled(manual);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->setEnabled(manual);
}

void CameraGUI::on_postProcessWhiteBalanceModeCombo_currentIndexChanged(int index)
{
    m_settings.m_postProcessWhiteBalanceMode = index;
    updatePostProcessWhiteBalanceControls();
    m_settingsKeys.append("postProcessWhiteBalanceMode");
    applySettings();
}

void CameraGUI::on_postProcessWhiteBalanceRedGainSlider_valueChanged(int value)
{
    const double gain = sliderValueToDoubleSpinBox(settingsUI()->postProcessWhiteBalanceRedGainSpin, value);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->setValue(gain);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceRedGain = gain;
    m_settingsKeys.append("postProcessWhiteBalanceRedGain");
    applySettings();
}

void CameraGUI::on_postProcessWhiteBalanceRedGainSpin_valueChanged(double value)
{
    settingsUI()->postProcessWhiteBalanceRedGainSlider->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceRedGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceRedGainSpin, value));
    settingsUI()->postProcessWhiteBalanceRedGainSlider->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceRedGain = value;
    m_settingsKeys.append("postProcessWhiteBalanceRedGain");
    applySettings();
}

void CameraGUI::on_postProcessWhiteBalanceGreenGainSlider_valueChanged(int value)
{
    const double gain = sliderValueToDoubleSpinBox(settingsUI()->postProcessWhiteBalanceGreenGainSpin, value);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->setValue(gain);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceGreenGain = gain;
    m_settingsKeys.append("postProcessWhiteBalanceGreenGain");
    applySettings();
}

void CameraGUI::on_postProcessWhiteBalanceGreenGainSpin_valueChanged(double value)
{
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceGreenGainSpin, value));
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceGreenGain = value;
    m_settingsKeys.append("postProcessWhiteBalanceGreenGain");
    applySettings();
}

void CameraGUI::on_postProcessWhiteBalanceBlueGainSlider_valueChanged(int value)
{
    const double gain = sliderValueToDoubleSpinBox(settingsUI()->postProcessWhiteBalanceBlueGainSpin, value);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->setValue(gain);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceBlueGain = gain;
    m_settingsKeys.append("postProcessWhiteBalanceBlueGain");
    applySettings();
}

void CameraGUI::on_postProcessWhiteBalanceBlueGainSpin_valueChanged(double value)
{
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceBlueGainSpin, value));
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceBlueGain = value;
    m_settingsKeys.append("postProcessWhiteBalanceBlueGain");
    applySettings();
}

void CameraGUI::on_saturationSlider_valueChanged(int value)
{
    m_settings.m_saturation = value / 100.0;
    settingsUI()->saturationSpin->blockSignals(true);
    settingsUI()->saturationSpin->setValue(m_settings.m_saturation);
    settingsUI()->saturationSpin->blockSignals(false);
    m_settingsKeys.append("saturation");
    applySettings();
}

void CameraGUI::on_saturationSpin_valueChanged(double value)
{
    settingsUI()->saturationSlider->blockSignals(true);
    settingsUI()->saturationSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->saturationSlider->blockSignals(false);
    m_settings.m_saturation = value;
    m_settingsKeys.append("saturation");
    applySettings();
}

void CameraGUI::on_gammaSlider_valueChanged(int value)
{
    m_settings.m_gamma = value / 100.0;
    settingsUI()->gammaSpin->blockSignals(true);
    settingsUI()->gammaSpin->setValue(m_settings.m_gamma);
    settingsUI()->gammaSpin->blockSignals(false);
    m_settingsKeys.append("gamma");
    applySettings();
}

void CameraGUI::on_gammaSpin_valueChanged(double value)
{
    settingsUI()->gammaSlider->blockSignals(true);
    settingsUI()->gammaSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->gammaSlider->blockSignals(false);
    m_settings.m_gamma = value;
    m_settingsKeys.append("gamma");
    applySettings();
}

void CameraGUI::on_gaussianBlurSlider_valueChanged(int value)
{
    settingsUI()->gaussianBlurSpin->blockSignals(true);
    settingsUI()->gaussianBlurSpin->setValue(value);
    settingsUI()->gaussianBlurSpin->blockSignals(false);
    m_settings.m_gaussianBlur = value;
    m_settingsKeys.append("gaussianBlur");
    applySettings();
}

void CameraGUI::on_gaussianBlurSpin_valueChanged(int value)
{
    settingsUI()->gaussianBlurSlider->blockSignals(true);
    settingsUI()->gaussianBlurSlider->setValue(value);
    settingsUI()->gaussianBlurSlider->blockSignals(false);
    m_settings.m_gaussianBlur = value;
    m_settingsKeys.append("gaussianBlur");
    applySettings();
}

void CameraGUI::on_medianBlurSlider_valueChanged(int value)
{
    settingsUI()->medianBlurSpin->blockSignals(true);
    settingsUI()->medianBlurSpin->setValue(value);
    settingsUI()->medianBlurSpin->blockSignals(false);
    m_settings.m_medianBlur = value;
    m_settingsKeys.append("medianBlur");
    applySettings();
}

void CameraGUI::on_medianBlurSpin_valueChanged(int value)
{
    settingsUI()->medianBlurSlider->blockSignals(true);
    settingsUI()->medianBlurSlider->setValue(value);
    settingsUI()->medianBlurSlider->blockSignals(false);
    m_settings.m_medianBlur = value;
    m_settingsKeys.append("medianBlur");
    applySettings();
}

void CameraGUI::on_sharpenSlider_valueChanged(int value)
{
    m_settings.m_sharpen = value / 100.0;
    settingsUI()->sharpenSpin->blockSignals(true);
    settingsUI()->sharpenSpin->setValue(m_settings.m_sharpen);
    settingsUI()->sharpenSpin->blockSignals(false);
    m_settingsKeys.append("sharpen");
    applySettings();
}

void CameraGUI::on_sharpenSpin_valueChanged(double value)
{
    settingsUI()->sharpenSlider->blockSignals(true);
    settingsUI()->sharpenSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->sharpenSlider->blockSignals(false);
    m_settings.m_sharpen = value;
    m_settingsKeys.append("sharpen");
    applySettings();
}

void CameraGUI::on_sobelEdgeSlider_valueChanged(int value)
{
    m_settings.m_sobelEdge = value / 100.0;
    settingsUI()->sobelEdgeSpin->blockSignals(true);
    settingsUI()->sobelEdgeSpin->setValue(m_settings.m_sobelEdge);
    settingsUI()->sobelEdgeSpin->blockSignals(false);
    m_settingsKeys.append("sobelEdge");
    applySettings();
}

void CameraGUI::on_sobelEdgeSpin_valueChanged(double value)
{
    settingsUI()->sobelEdgeSlider->blockSignals(true);
    settingsUI()->sobelEdgeSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->sobelEdgeSlider->blockSignals(false);
    m_settings.m_sobelEdge = value;
    m_settingsKeys.append("sobelEdge");
    applySettings();
}

void CameraGUI::on_flipXButton_toggled(bool checked)
{
    m_settings.m_flipX = checked;
    m_settingsKeys.append("flipX");
    applySettings();
}

void CameraGUI::on_flipYButton_toggled(bool checked)
{
    m_settings.m_flipY = checked;
    m_settingsKeys.append("flipY");
    applySettings();
}

void CameraGUI::on_brightnessSlider_valueChanged(int value)
{
    m_settings.m_brightness = static_cast<double>(value);
    settingsUI()->brightnessSpin->blockSignals(true);
    settingsUI()->brightnessSpin->setValue(value);
    settingsUI()->brightnessSpin->blockSignals(false);
    m_settingsKeys.append("brightness");
    applySettings();
}

void CameraGUI::on_brightnessSpin_valueChanged(int value)
{
    settingsUI()->brightnessSlider->blockSignals(true);
    settingsUI()->brightnessSlider->setValue(value);
    settingsUI()->brightnessSlider->blockSignals(false);
    m_settings.m_brightness = static_cast<double>(value);
    m_settingsKeys.append("brightness");
    applySettings();
}

void CameraGUI::on_contrastSlider_valueChanged(int value)
{
    m_settings.m_contrast = value / 100.0;
    settingsUI()->contrastSpin->blockSignals(true);
    settingsUI()->contrastSpin->setValue(m_settings.m_contrast);
    settingsUI()->contrastSpin->blockSignals(false);
    m_settingsKeys.append("contrast");
    applySettings();
}

void CameraGUI::on_contrastSpin_valueChanged(double value)
{
    settingsUI()->contrastSlider->blockSignals(true);
    settingsUI()->contrastSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->contrastSlider->blockSignals(false);
    m_settings.m_contrast = value;
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
    applySettings();
}

void CameraGUI::on_dateTimeColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_dateTimeColor, this, tr("Select date/time text colour"));

    if (color.isValid())
    {
        m_settings.m_dateTimeColor = color;
        updateColorButton(settingsUI()->dateTimeColorButton, m_settings.m_dateTimeColor);
        m_settingsKeys.append("dateTimeColor");
        applySettings();
    }
}

void CameraGUI::on_dateTimeFormatEdit_editingFinished()
{
    m_settings.m_dateTimeFormat = settingsUI()->dateTimeFormatEdit->text();
    m_settingsKeys.append("dateTimeFormat");
    applySettings();
}

void CameraGUI::on_dateTimePosXSlider_valueChanged(int value)
{
    m_settings.m_dateTimePosX = value;
    settingsUI()->dateTimePosXValue->setText(QString::number(value));
    m_settingsKeys.append("dateTimePosX");
    applySettings();
}

void CameraGUI::on_dateTimePosYSlider_valueChanged(int value)
{
    m_settings.m_dateTimePosY = value;
    settingsUI()->dateTimePosYValue->setText(QString::number(value));
    m_settingsKeys.append("dateTimePosY");
    applySettings();
}

void CameraGUI::on_overlayTextButton_toggled(bool checked)
{
    m_settings.m_overlayText = checked;
    m_settingsKeys.append("overlayText");
    applySettings();
}

void CameraGUI::on_overlayTextColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_overlayTextColor, this, tr("Select overlay text colour"));

    if (color.isValid())
    {
        m_settings.m_overlayTextColor = color;
        updateColorButton(settingsUI()->overlayTextColorButton, m_settings.m_overlayTextColor);
        m_settingsKeys.append("overlayTextColor");
        applySettings();
    }
}

void CameraGUI::on_overlayTextEdit_textChanged()
{
    m_settings.m_overlayTextString = settingsUI()->overlayTextEdit->toPlainText();
    m_settingsKeys.append("overlayTextString");
    applySettings();
}

void CameraGUI::on_overlayTextPosXSlider_valueChanged(int value)
{
    m_settings.m_overlayTextPosX = value;
    settingsUI()->overlayTextPosXValue->setText(QString::number(value));
    m_settingsKeys.append("overlayTextPosX");
    applySettings();
}

void CameraGUI::on_overlayTextPosYSlider_valueChanged(int value)
{
    m_settings.m_overlayTextPosY = value;
    settingsUI()->overlayTextPosYValue->setText(QString::number(value));
    m_settingsKeys.append("overlayTextPosY");
    applySettings();
}

void CameraGUI::on_diffMaskButton_toggled(bool checked)
{
    m_settings.m_diffMask = checked;
    m_settingsKeys.append("diffMask");
    applySettings();
}

void CameraGUI::on_diffThresholdSpin_valueChanged(int value)
{
    m_settings.m_diffThreshold = value;
    m_settingsKeys.append("diffThreshold");
    applySettings();
}

void CameraGUI::on_dilationSpin_valueChanged(int value)
{
    m_settings.m_dilationSize = value;
    m_settingsKeys.append("dilationSize");
    applySettings();
}

void CameraGUI::on_diffMaskHistoryFramesSpin_valueChanged(int value)
{
    m_settings.m_diffMaskHistoryFrames = value;
    m_settingsKeys.append("diffMaskHistoryFrames");
    applySettings();
}

void CameraGUI::on_histogramButton_clicked()
{
    if (!m_lastImage.isNull())
    {
        if (!m_histogramDialog)
        {
            m_histogramDialog = new CameraHistogramDialog(m_lastImage, this);
            m_histogramDialog->setAttribute(Qt::WA_DeleteOnClose); // Delete when closed, so we don't waste CPU calculating the histogram when not visible
            connect(m_histogramDialog, &QObject::destroyed, this, [this]() { m_histogramDialog = nullptr; });
        }
        else
        {
            m_histogramDialog->updateImage(m_lastImage);
        }

        m_histogramDialog->show();
        m_histogramDialog->raise();
        m_histogramDialog->activateWindow();
    }
}

void CameraGUI::on_defaultColorSettingsButton_clicked()
{
    settingsUI()->postProcessWhiteBalanceModeCombo->setCurrentIndex(0);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->setValue(1);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->setValue(1);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->setValue(1);
    settingsUI()->brightnessSpin->setValue(0);
    settingsUI()->contrastSpin->setValue(1.0);
    settingsUI()->saturationSpin->setValue(1.0);
    settingsUI()->gammaSpin->setValue(1.0);
    settingsUI()->gaussianBlurSpin->setValue(0);
    settingsUI()->medianBlurSpin->setValue(0);
    settingsUI()->sharpenSpin->setValue(0.0);
    settingsUI()->sobelEdgeSpin->setValue(0.0);
    settingsUI()->flipXButton->setChecked(false);
    settingsUI()->flipYButton->setChecked(false);
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
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const bool manualFocus = (m_settings.m_focusMode == static_cast<int>(QCamera::FocusModeManual));
    settingsUI()->focusDistLabel->setEnabled(manualFocus);
    settingsUI()->focusDistSpin->setEnabled(manualFocus);
#endif

    settingsUI()->alpacaStatusGroup->setVisible(m_settings.isAlpacaCamera());

    if (m_settings.isAlpacaCamera())
    {
        settingsUI()->exposureLabel->setEnabled(true);
        settingsUI()->exposureSlider->setEnabled(true);
        settingsUI()->exposureSpin->setEnabled(true);
        settingsUI()->exposureUnitsCombo->setEnabled(true);
    }
    else
    {
        // Zoom and exposure control enabled states are set inside setupQtCapture;
        // re-apply them here so other updateEnabledControls callers don't accidentally re-enable them.
        if (!m_qtZoomSupported)
        {
            settingsUI()->zoomLabel->setEnabled(false);
            settingsUI()->zoomSpin->setEnabled(false);
        }
        if (!m_qtManualExposureSupported)
        {
            settingsUI()->exposureLabel->setEnabled(false);
            settingsUI()->exposureSlider->setEnabled(false);
            settingsUI()->exposureSpin->setEnabled(false);
            settingsUI()->exposureUnitsCombo->setEnabled(false);
        }
        if (!m_qtIsoSensitivitySupported)
        {
            settingsUI()->isoLabel->setEnabled(false);
            settingsUI()->isoSpin->setEnabled(false);
        }
        if (!m_qtExposureCompensationSupported)
        {
            settingsUI()->exposureCompLabel->setEnabled(false);
            settingsUI()->exposureCompSpin->setEnabled(false);
        }
        if (!m_qtWhiteBalanceModeSupported)
        {
            settingsUI()->whiteBalanceLabel->setEnabled(false);
            settingsUI()->whiteBalanceCombo->setEnabled(false);
        }
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

void CameraGUI::on_overlayTextFontCombo_currentFontChanged(const QFont& font)
{
    m_settings.m_overlayTextFontFamily = font.family();
    m_settingsKeys.append("overlayTextFontFamily");
    applySettings();
}

void CameraGUI::on_overlayTextFontScaleSpin_valueChanged(double value)
{
    m_settings.m_overlayTextFontScale = value;
    m_settingsKeys.append("overlayTextFontScale");
    applySettings();
}

void CameraGUI::on_motionDetectButton_toggled(bool checked)
{
    m_settings.m_motionDetect = checked;
    m_settingsKeys.append("motionDetect");
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
        updateColorButton(settingsUI()->motionBoxColorButton, color);
        m_settingsKeys.append("motionBoxColor");
        applySettings();
    }
}

void CameraGUI::on_spectrumOverlayButton_toggled(bool checked)
{
    m_settings.m_overlaySpectrum = checked;
    m_settingsKeys.append("overlaySpectrum");
    applySettings();
}

void CameraGUI::on_spectrumDeviceCombo_currentIndexChanged(int index)
{
    m_settings.m_spectrumDevice = settingsUI()->spectrumDeviceCombo->itemText(index);
    m_settingsKeys.append("spectrumDevice");
    applySettings();
}

void CameraGUI::on_spectrumOffsetXSlider_valueChanged(int value)
{
    m_settings.m_spectrumOffsetX = value;
    settingsUI()->spectrumOffsetXValue->setText(QString::number(value));
    m_settingsKeys.append("spectrumOffsetX");
    applySettings();
}

void CameraGUI::on_spectrumOffsetYSlider_valueChanged(int value)
{
    m_settings.m_spectrumOffsetY = value;
    settingsUI()->spectrumOffsetYValue->setText(QString::number(value));
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
    applySettings();
}

void CameraGUI::on_yoloModelPathCombo_currentTextChanged(const QString &text)
{
    m_settings.m_yoloModelPath = text;
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
        settingsUI()->yoloModelPathCombo->setCurrentText(fileName);
        m_settingsKeys.append("yoloModelPath");
        applySettings();
    }
}

void CameraGUI::on_yoloLabelsPathCombo_currentTextChanged(const QString& text)
{
    m_settings.m_yoloLabelsPath = text;
    populateActionClasses();
    rebuildActionTabsForCurrentClass();
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
        settingsUI()->yoloLabelsPathCombo->setCurrentText(fileName);
        populateActionClasses();
        rebuildActionTabsForCurrentClass();
        m_settingsKeys.append("yoloLabelsPath");
        applySettings();
    }
}

void CameraGUI::on_yoloTargetCombo_currentIndexChanged(int index)
{
    m_settings.m_yoloDnnTarget = static_cast<CameraSettings::DNNTarget>(index);
    m_settingsKeys.append("yoloDnnTarget");
    applySettings();
}

void CameraGUI::on_actionsClassCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    saveCurrentActionClassSettings();
    rebuildActionTabsForCurrentClass();
    m_settingsKeys.append("objectDeviceSettings");
    applySettings();
}

void CameraGUI::on_actionsDisappearDebounceSpin_valueChanged(double value)
{
    m_settings.m_yoloDisappearDebounce = value;
    applyActionSettings();
}

void CameraGUI::on_actionsAddButton_clicked()
{
    const QString className = settingsUI()->actionsClassCombo->currentText();
    if (className.isEmpty()) {
        return;
    }

    if (!m_settings.m_objectDeviceSettings.contains(className)) {
        m_settings.m_objectDeviceSettings.insert(className, new QList<CameraSettings::ObjectDeviceSettings *>());
    }

    CameraSettings::ObjectDeviceSettings *deviceSettings = new CameraSettings::ObjectDeviceSettings();
    CameraObjectDeviceSettingsGUI *deviceSettingsGUI =
        new CameraObjectDeviceSettingsGUI(deviceSettings, settingsUI()->actionsTabWidget, settingsUI()->actionsTabWidget);

    const int index = settingsUI()->actionsTabWidget->addTab(deviceSettingsGUI, QStringLiteral("R0"));
    settingsUI()->actionsTabWidget->setCurrentIndex(index);
    m_actionDeviceSettingsGUIs.append(deviceSettingsGUI);
    m_settings.m_objectDeviceSettings.value(className)->append(deviceSettings);
    applyActionSettings();
}

void CameraGUI::on_actionsTabWidget_tabCloseRequested(int index)
{
    const QString className = settingsUI()->actionsClassCombo->currentText();
    if (className.isEmpty() || !m_settings.m_objectDeviceSettings.contains(className)) {
        return;
    }

    settingsUI()->actionsTabWidget->removeTab(index);
    delete m_actionDeviceSettingsGUIs.takeAt(index);

    QList<CameraSettings::ObjectDeviceSettings *> *deviceSettingsList = m_settings.m_objectDeviceSettings.value(className);
    delete deviceSettingsList->takeAt(index);
    applyActionSettings();
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
        updateColorButton(settingsUI()->yoloBoxColorButton, color);
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
    m_settings.m_whiteBalanceMode = settingsUI()->whiteBalanceCombo->itemData(index).toInt();
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
    m_settings.m_focusMode = settingsUI()->focusModeCombo->itemData(index).toInt();
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

void CameraGUI::on_cameraSettingsButton_clicked()
{
    m_settingsDialog->show();
    m_settingsDialog->raise();
    m_settingsDialog->activateWindow();
}

void CameraGUI::applyImagePath()
{
    ui->saveImageCheck->setToolTip(QString("Save images to %1").arg(m_settings.m_imageFileName));
}

void CameraGUI::applyVideoPath()
{
    ui->saveVideoCheck->setToolTip(QString("Record video to %1").arg(m_settings.m_videoFileName));
}

void CameraGUI::onSettingsDialogFinished(int result)
{
    Q_UNUSED(result)
    applyActionSettings();
}

void CameraGUI::updateStatus()
{
    int state = m_camera->getState();

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
            ui->startStop->setStyleSheet("QToolButton { background-color : green; }");
            break;
        case Feature::StError:
            ui->startStop->setStyleSheet("QToolButton { background-color : red; }");
            QMessageBox::critical(this, m_settings.m_title, m_camera->getErrorMessage());
            break;
        default:
            break;
        }

        m_lastFeatureState = state;
    }
}
