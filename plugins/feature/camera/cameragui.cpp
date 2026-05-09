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
#include <limits>

#include <QCheckBox>
#include <QColorDialog>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QMouseEvent>
#include <QLineEdit>
#include <QNetworkReply>
#include <QPainter>
#include <QPixmap>
#include <QSet>
#include <QSignalBlocker>
#include <QStandardItemModel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QWheelEvent>
#include <QMessageBox>
#include <QUrl>
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCamera>
#include <QCameraDevice>
#include <QCameraFormat>
#include <QImageCapture>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QMediaPlayer>
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

#include "feature/featureuiset.h"
#include "gui/crightclickenabler.h"
#include "gui/audioselectdialog.h"
#include "gui/dialogpositioner.h"
#include "dsp/dspengine.h"
#include "maincore.h"
#include "feature/featureset.h"
#include "channel/channelwebapiutils.h"

#include "ui_cameragui.h"
#include "camera.h"
#include "cameraframestacker.h"
#include "camerahistogramdialog.h"
#include "camerasettingsdialog.h"
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
    applyAllSettings();
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
        applyAllSettings();
        updateHardware();
        updateScheduledCapture();
        m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create());
        return true;
    }

    resetToDefaults();
    return false;
}

bool CameraGUI::handleMessage(const Message& message)
{
    const CameraInfo previousCamera = selectedCameraFromSettings();

    if (Camera::MsgConfigureCamera::match(message))
    {
        const Camera::MsgConfigureCamera& cfg = (Camera::MsgConfigureCamera&) message;

        if (cfg.getForce()) {
            m_settings = cfg.getSettings();
        } else {
            m_settings.applySettings(cfg.getSettingsKeys(), cfg.getSettings());
        }

        if (!sameCameraIdentity(previousCamera, selectedCameraFromSettings())) {
            resetCameraStatus();
        }

        // Web API may supply cameraDescription without cameraProtocol/cameraId.
        // Resolve the full selection from the combo box and push a follow-up
        // configure so the engine also receives the correct protocol and id.
        if (!cfg.getForce()
            && cfg.getSettingsKeys().contains("cameraDescription")
            && !cfg.getSettingsKeys().contains("cameraProtocol")
            && !cfg.getSettingsKeys().contains("cameraId"))
        {
            int matchedIndex = -1;
            for (int i = 0; i < ui->cameraCombo->count(); ++i)
            {
                if (ui->cameraCombo->itemData(i, CameraDescriptionRole).toString()
                        == m_settings.m_cameraDescription)
                {
                    if (matchedIndex >= 0) {
                        matchedIndex = -1;
                        break;
                    }

                    matchedIndex = i;
                }
            }

            if (matchedIndex >= 0)
            {
                const CameraInfo resolvedCamera = comboCameraInfo(matchedIndex);
                if (!sameCameraIdentity(resolvedCamera, selectedCameraFromSettings()))
                {
                    setSelectedCamera(resolvedCamera.m_protocol, resolvedCamera.m_id,
                        resolvedCamera.m_description, resolvedCamera.m_host, resolvedCamera.m_port);
                    applySettings(cameraSelectionSettingsKeys(resolvedCamera));
                }
            }
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

        if (!sameCameraIdentity(previousCamera, selectedCameraFromSettings())) {
            resetCameraStatus();
        }

        if (cfg.getStartStop())
        {
            if (m_settings.isQtCamera() || m_settings.isFileCamera()) {
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
        CameraInfo selectedCamera;
        QList<CameraInfo> entries;
        QHash<QString, int> displayCounts;

        for (const CameraInfo& camera : report.getCameras())
        {
            entries.append(camera);

            const QString displayKey = camera.m_protocol.isEmpty()
                ? camera.m_id
                : (camera.m_protocol == QLatin1String("file")
                    ? (camera.m_description.isEmpty() ? QStringLiteral("file:") : QStringLiteral("file:%1").arg(camera.m_description))
                    : QString("%1:%2").arg(camera.m_protocol, camera.m_description));
            displayCounts[displayKey] = displayCounts.value(displayKey) + 1;
        }

        ui->cameraCombo->blockSignals(true);
        ui->cameraCombo->clear();
        for (const CameraInfo& entry : entries)
        {
            QString displayText = entry.m_protocol.isEmpty()
                ? entry.m_id
                : (entry.m_protocol == QLatin1String("file")
                    ? (entry.m_description.isEmpty() ? QStringLiteral("file:") : QStringLiteral("file:%1").arg(entry.m_description))
                    : QString("%1:%2").arg(entry.m_protocol, entry.m_description));

            if (!entry.m_host.isEmpty() && displayCounts.value(displayText) > 1) {
                displayText = QString("%1 (%2:%3)").arg(displayText, entry.m_host).arg(entry.m_port);
            }

            ui->cameraCombo->addItem(displayText);
            const int itemIndex = ui->cameraCombo->count() - 1;
            ui->cameraCombo->setItemData(itemIndex, entry.m_protocol, CameraProtocolRole);
            ui->cameraCombo->setItemData(itemIndex, entry.m_id, CameraIdRole);
            ui->cameraCombo->setItemData(itemIndex, entry.m_description, CameraDescriptionRole);
            ui->cameraCombo->setItemData(itemIndex, entry.m_host, CameraAlpacaHostRole);
            ui->cameraCombo->setItemData(itemIndex, static_cast<int>(entry.m_port), CameraAlpacaPortRole);
        }

        int index = findCameraComboIndex(
            m_settings.m_cameraProtocol,
            m_settings.m_cameraId,
            m_settings.m_alpacaHost,
            m_settings.m_alpacaPort);
        if (index < 0 && ui->cameraCombo->count() > 0) {
            index = 0;
        }

        if (index >= 0)
        {
            ui->cameraCombo->setCurrentIndex(index);
            selectedCamera = comboCameraInfo(index);
        }

        ui->cameraCombo->blockSignals(false);

        const bool selectedCameraDiffers =
            !selectedCamera.m_id.isEmpty() && !sameCameraIdentity(selectedCamera, selectedCameraFromSettings());

        if (selectedCameraDiffers)
        {
            setSelectedCamera(selectedCamera.m_protocol, selectedCamera.m_id, selectedCamera.m_description,
                selectedCamera.m_host, selectedCamera.m_port);
            QStringList settingsKeys = cameraSelectionSettingsKeys(selectedCamera);
            updateCameraSettingsVisibility();
            applySettings(settingsKeys);
        }
        else if ((selectedCamera.m_protocol == QLatin1String("qt")) && !ui->startStop->isChecked())
        {
            setSelectedCamera(selectedCamera.m_protocol, selectedCamera.m_id, selectedCamera.m_description,
                selectedCamera.m_host, selectedCamera.m_port);
            probeQtCameraCapabilities();
            updateCameraSettingsVisibility();
        }

        return true;
    }
    else if (CameraWorker::MsgReportAlpacaDeviceList::match(message))
    {
        QList<QString> settingsKeys;
        const QString previousFocuserHost = m_settings.m_alpacaFocuserHost;
        const quint16 previousFocuserPort = m_settings.m_alpacaFocuserPort;
        const int previousFocuserDeviceNumber = m_settings.m_alpacaFocuserDeviceNumber;
        const QString previousFilterWheelHost = m_settings.m_alpacaFilterWheelHost;
        const quint16 previousFilterWheelPort = m_settings.m_alpacaFilterWheelPort;
        const int previousFilterWheelDeviceNumber = m_settings.m_alpacaFilterWheelDeviceNumber;
        const CameraWorker::MsgReportAlpacaDeviceList& report = (CameraWorker::MsgReportAlpacaDeviceList&) message;
        m_discoveredAlpacaFocusers = report.getFocusers();
        m_discoveredAlpacaFilterWheels = report.getFilterWheels();
        populateAlpacaAccessoryCombos();

        {
            QSignalBlocker blocker1(settingsUI()->alpacaFocuserHostEdit);
            QSignalBlocker blocker2(settingsUI()->alpacaFocuserPortSpin);
            QSignalBlocker blocker4(settingsUI()->alpacaFilterWheelHostEdit);
            QSignalBlocker blocker5(settingsUI()->alpacaFilterWheelPortSpin);
            settingsUI()->alpacaFocuserHostEdit->setText(m_settings.m_alpacaFocuserHost);
            settingsUI()->alpacaFocuserPortSpin->setValue(m_settings.m_alpacaFocuserPort);
            settingsUI()->alpacaFilterWheelHostEdit->setText(m_settings.m_alpacaFilterWheelHost);
            settingsUI()->alpacaFilterWheelPortSpin->setValue(m_settings.m_alpacaFilterWheelPort);
        }

        if ((previousFocuserHost != m_settings.m_alpacaFocuserHost)
            || (previousFocuserPort != m_settings.m_alpacaFocuserPort)
            || (previousFocuserDeviceNumber != m_settings.m_alpacaFocuserDeviceNumber))
        {
            settingsKeys.append("alpacaFocuserHost");
            settingsKeys.append("alpacaFocuserPort");
            settingsKeys.append("alpacaFocuserDeviceNumber");
        }

        if ((previousFilterWheelHost != m_settings.m_alpacaFilterWheelHost)
            || (previousFilterWheelPort != m_settings.m_alpacaFilterWheelPort)
            || (previousFilterWheelDeviceNumber != m_settings.m_alpacaFilterWheelDeviceNumber))
        {
            settingsKeys.append("alpacaFilterWheelHost");
            settingsKeys.append("alpacaFilterWheelPort");
            settingsKeys.append("alpacaFilterWheelDeviceNumber");
        }

        if (!settingsKeys.isEmpty()) {
            applySettings(settingsKeys);
        }

        return true;
    }
    else if (CameraPostProcessor::MsgReportFrame::match(message))
    {
        const CameraPostProcessor::MsgReportFrame& report = (CameraPostProcessor::MsgReportFrame&) message;
        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
        QSize oldSize = m_lastImage.size();
        m_lastImage = report.getImage();
        m_lastHistogramData = report.getHistogramData();
        m_lastStackCount = report.getStackCount();
        settingsUI()->stackCurrentCountValue->setText(QString::number(m_lastStackCount));
        m_pipelineFrameTimes.append(nowMs);
        while ((m_pipelineFrameTimes.size() > 1) && (m_pipelineFrameTimes.first() < nowMs - 5000)) {
            m_pipelineFrameTimes.removeFirst();
        }
        if (m_pipelineFrameTimes.size() >= 2)
        {
            const qint64 spanMs = m_pipelineFrameTimes.last() - m_pipelineFrameTimes.first();
            m_lastPipelineFps = spanMs > 0
                ? (1000.0 * static_cast<double>(m_pipelineFrameTimes.size() - 1) / static_cast<double>(spanMs))
                : 0.0;
        }
        else
        {
            m_lastPipelineFps = 0.0;
        }
        settingsUI()->pipelineFpsLabel->setText(
            m_lastPipelineFps > 0.0 ? QString::number(m_lastPipelineFps, 'f', 1) : "-");
        updateImageWidget();
        if (m_histogramDialog) {
            m_histogramDialog->updateHistogram(m_lastHistogramData);
        }
        // When the image size changes, refit to view
        if (oldSize != m_lastImage.size()) {
            ui->imageView->fitInView(m_imagePixmapItem, Qt::KeepAspectRatio);
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
        applySetting("saveVideo");
        return true;
    }
    else if (CameraWorker::MsgReportAlpacaCameraInfo::match(message))
    {
        const CameraWorker::MsgReportAlpacaCameraInfo& info = (CameraWorker::MsgReportAlpacaCameraInfo&) message;
        updateAlpacaCapabilities(info);
        return true;
    }
    else if (CameraWorker::MsgReportAsiCameraInfo::match(message))
    {
        const CameraWorker::MsgReportAsiCameraInfo& info = (CameraWorker::MsgReportAsiCameraInfo&) message;
        updateAsiCapabilities(info);
        return true;
    }
    else if (CameraWorker::MsgReportAlpacaFilterWheelInfo::match(message))
    {
        const CameraWorker::MsgReportAlpacaFilterWheelInfo& info = (CameraWorker::MsgReportAlpacaFilterWheelInfo&) message;
        if (!info.getNames().isEmpty()) {
            m_alpacaFilterWheelNames = info.getNames();
        }

        QSignalBlocker blocker(settingsUI()->alpacaFilterWheelPositionCombo);
        if (!info.getNames().isEmpty())
        {
            settingsUI()->alpacaFilterWheelPositionCombo->clear();

            for (int i = 0; i < m_alpacaFilterWheelNames.size(); ++i) {
                settingsUI()->alpacaFilterWheelPositionCombo->addItem(m_alpacaFilterWheelNames.at(i), i);
            }
        }

        if ((info.getPosition() >= 0) && (info.getPosition() < settingsUI()->alpacaFilterWheelPositionCombo->count()))
        {
            settingsUI()->alpacaFilterWheelPositionCombo->setCurrentIndex(info.getPosition());
            m_settings.m_alpacaFilterWheelPosition = info.getPosition();
        }
        else if (!info.getNames().isEmpty() && (settingsUI()->alpacaFilterWheelPositionCombo->count() > 0))
        {
            settingsUI()->alpacaFilterWheelPositionCombo->setCurrentIndex(
                qBound(0, m_settings.m_alpacaFilterWheelPosition, settingsUI()->alpacaFilterWheelPositionCombo->count() - 1));
        }

        updateCameraSettingsVisibility();
        return true;
    }
    else if (CameraWorker::MsgReportAlpacaStatus::match(message))
    {
        const CameraWorker::MsgReportAlpacaStatus& status = (CameraWorker::MsgReportAlpacaStatus&) message;
        m_lastAlpacaCameraState = status.getCameraState();
        m_lastAlpacaCaptureTimeMs = status.getCaptureTimeMs();
        m_lastAlpacaReceiveImageFormat = status.getReceiveImageFormat();
        m_lastAlpacaCcdTemperature = status.getCcdTemperature();
        m_lastAlpacaCcdTemperatureValid = status.isCcdTemperatureValid();
        m_lastAlpacaErrorNumber = status.getLastErrorNumber();
        m_lastAlpacaErrorMessage = status.getLastErrorMessage();
        updateCameraStatusDisplay();

        if (status.isCcdTemperatureValid()) {
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
    m_forceSettings(false),
    m_updateTimer(this),
    m_scheduleTimer(this),
    m_lastFeatureState(0),
    m_dlm(this),
    m_settingsDialog(nullptr),
    m_histogramDialog(nullptr),
    m_alpacaHasNamedGains(false),
    m_alpacaHasNamedOffsets(false),
    m_lastAlpacaCameraState(-1),
    m_lastAlpacaCaptureTimeMs(-1),
    m_lastAlpacaReceiveImageFormat(),
    m_lastAlpacaCcdTemperature(0.0),
    m_lastAlpacaCcdTemperatureValid(false),
    m_lastAlpacaErrorNumber(0),
    m_lastAlpacaErrorMessage(),
    m_alpacaCameraSizeX(0),
    m_alpacaCameraSizeY(0),
    m_qtZoomSupported(false),
    m_qtManualExposureSupported(true),
    m_qtIsoSensitivitySupported(true),
    m_qtWhiteBalanceModeSupported(true),
    m_qtExposureCompensationSupported(true),
    m_exposureMinimumMs(1.0),
    m_exposureMaximumMs(60000.0),
    m_exposureStepMs(1.0),
    m_asiCoolerSupported(false),
    m_asiTargetTempSupported(false),
    m_asiUsbBandwidthSupported(false),
    m_asiHighSpeedModeSupported(false),
    m_asiColorCameraActive(false),
    m_asiRgb24Supported(false),
    m_asiRaw16Supported(false),
    m_imageScene(nullptr),
    m_imagePixmapItem(nullptr),
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    m_qtCamera(nullptr),
    m_imageCapture(nullptr),
    m_mediaPlayer(nullptr),
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
    CRightClickEnabler *useMyPositionRightClickEnabler = new CRightClickEnabler(settingsUI()->useMyPositionButton);
    connect(useMyPositionRightClickEnabler, SIGNAL(rightClick(const QPoint &)), this, SLOT(useMyPositionButton_rightClicked(const QPoint &)));

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
    connect(&m_scheduleTimer, &QTimer::timeout, this, &CameraGUI::updateScheduledCapture);
    connect(m_settingsDialog, &QDialog::finished, this, &CameraGUI::onSettingsDialogFinished);
    connect(&m_qtStillCaptureTimer, &QTimer::timeout, this, &CameraGUI::triggerQtStillCapture);
    connect(&m_dlm, &HttpDownloadManagerGUI::downloadComplete, this, &CameraGUI::handleYoloDownloadComplete);
    connect(MainCore::instance(), &MainCore::featureAdded, this, &CameraGUI::onFeatureAdded);
    connect(MainCore::instance(), &MainCore::featureRemoved, this, &CameraGUI::onFeatureRemoved);
    m_qtStillCaptureTimer.setSingleShot(false);

    settingsUI()->fpsLabel->addItem(tr("Frame Rate"), CameraSettings::CaptureModeFrameRate);
    settingsUI()->fpsLabel->addItem(tr("Interval"), CameraSettings::CaptureModeInterval);
    settingsUI()->intervalUnitsCombo->addItem(tr("s"), CameraSettings::CaptureIntervalSeconds);
    settingsUI()->intervalUnitsCombo->addItem(tr("min"), CameraSettings::CaptureIntervalMinutes);
    settingsUI()->exposureUnitsCombo->addItem(tr("us"), 0.001);
    settingsUI()->exposureUnitsCombo->addItem(tr("ms"), 1.0);
    settingsUI()->exposureUnitsCombo->addItem(tr("s"), 1000.0);
    settingsUI()->exposureUnitsCombo->addItem(tr("min"), 60000.0);
    settingsUI()->exposureUnitsCombo->setCurrentIndex(1);
    m_statusTimer.start(250);
    m_scheduleTimer.start(1000);

    connect(&m_updateTimer, &QTimer::timeout, this, &CameraGUI::updateHardware);

    displaySettings();
    applyAllSettings();
    updateScheduledCapture();
    makeUIConnections();
    m_resizer.enableChildMouseTracking();

    m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create());
}

CameraGUI::~CameraGUI()
{
    if (m_camera) {
        m_camera->setMessageQueueToGUI(nullptr);
    }
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

void CameraGUI::setSelectedCamera(const QString& protocol, const QString& cameraId, const QString& description,
    const QString& alpacaHost, quint16 alpacaPort)
{
    m_settings.m_cameraProtocol = protocol;
    m_settings.m_cameraId = cameraId;
    m_settings.m_cameraDescription = description;

    if (protocol == QLatin1String("file")) {
        m_settings.m_videoFileCameraPath = cameraId;
    }

    if (protocol == QLatin1String("alpaca")) {
        m_settings.m_alpacaHost = alpacaHost;
        m_settings.m_alpacaPort = alpacaPort;
    }
}

CameraInfo CameraGUI::comboCameraInfo(int index) const
{
    CameraInfo cameraInfo;

    if ((index < 0) || (index >= ui->cameraCombo->count())) {
        return cameraInfo;
    }

    cameraInfo.m_protocol = ui->cameraCombo->itemData(index, CameraProtocolRole).toString();
    cameraInfo.m_id = ui->cameraCombo->itemData(index, CameraIdRole).toString();
    cameraInfo.m_description = ui->cameraCombo->itemData(index, CameraDescriptionRole).toString();
    cameraInfo.m_host = ui->cameraCombo->itemData(index, CameraAlpacaHostRole).toString();
    cameraInfo.m_port = static_cast<quint16>(ui->cameraCombo->itemData(index, CameraAlpacaPortRole).toUInt());
    return cameraInfo;
}

CameraInfo CameraGUI::selectedCameraFromSettings() const
{
    CameraInfo cameraInfo;
    cameraInfo.m_protocol = m_settings.m_cameraProtocol;
    cameraInfo.m_id = m_settings.m_cameraId;
    cameraInfo.m_description = m_settings.m_cameraDescription;

    if (m_settings.isAlpacaCamera())
    {
        cameraInfo.m_host = m_settings.m_alpacaHost;
        cameraInfo.m_port = m_settings.m_alpacaPort;
    }

    return cameraInfo;
}

bool CameraGUI::sameCameraIdentity(const CameraInfo& lhs, const CameraInfo& rhs)
{
    if ((lhs.m_protocol != rhs.m_protocol) || (lhs.m_id != rhs.m_id)) {
        return false;
    }

    if (lhs.m_protocol == QLatin1String("alpaca")) {
        return (lhs.m_host == rhs.m_host) && (lhs.m_port == rhs.m_port);
    }

    return true;
}

bool CameraGUI::isSameHardwareCameraBackend(const CameraInfo& lhs, const CameraInfo& rhs)
{
    const auto isSharedHardwareCamera = [](const QString& protocol) {
        return (protocol == QLatin1String("alpaca")) || (protocol == QLatin1String("asi"));
    };

    return isSharedHardwareCamera(lhs.m_protocol) && (lhs.m_protocol == rhs.m_protocol);
}

QStringList CameraGUI::cameraSelectionSettingsKeys(const CameraInfo& cameraInfo) const
{
    QStringList settingsKeys {"cameraProtocol", "cameraId", "cameraDescription"};

    if (cameraInfo.m_protocol == QLatin1String("file")) {
        settingsKeys.append("videoFileCameraPath");
    }

    if (cameraInfo.m_protocol == QLatin1String("alpaca")) {
        settingsKeys.append("alpacaHost");
        settingsKeys.append("alpacaPort");
    }

    return settingsKeys;
}

void CameraGUI::resetCameraStatus()
{
    m_lastAlpacaCameraState = -1;
    m_lastAlpacaCaptureTimeMs = -1;
    m_lastAlpacaReceiveImageFormat.clear();
    m_lastAlpacaCcdTemperatureValid = false;
    m_lastAlpacaErrorNumber = 0;
    m_lastAlpacaErrorMessage.clear();
    m_pipelineFrameTimes.clear();
    m_lastPipelineFps = 0.0;
    settingsUI()->pipelineFpsLabel->setText("-");
    m_settingsDialog->clearCameraStatus();
}

int CameraGUI::findCameraComboIndex(const QString& protocol, const QString& cameraId,
    const QString& alpacaHost, quint16 alpacaPort) const
{
    for (int i = 0; i < ui->cameraCombo->count(); ++i)
    {
        if (ui->cameraCombo->itemData(i, CameraProtocolRole).toString() != protocol
            || ui->cameraCombo->itemData(i, CameraIdRole).toString() != cameraId)
        {
            continue;
        }

        if (protocol == QLatin1String("alpaca"))
        {
            if (ui->cameraCombo->itemData(i, CameraAlpacaHostRole).toString() == alpacaHost
                && static_cast<quint16>(ui->cameraCombo->itemData(i, CameraAlpacaPortRole).toUInt()) == alpacaPort)
            {
                return i;
            }
        }
        else
        {
            return i;
        }
    }

    return -1;
}

bool CameraGUI::chooseVideoFileCameraFile(int comboIndex, const QString& previousCameraProtocol,
    const QString& previousCameraId,
    const QString& previousAlpacaHost, quint16 previousAlpacaPort)
{
    const QString filePath = QFileDialog::getOpenFileName(
        this,
        tr("Select Video File"),
        m_settings.m_videoFileCameraPath,
        tr("Video Files (*.mp4 *.mkv *.mov *.avi *.m4v *.wmv *.webm);;All Files (*.*)"));

    if (filePath.isEmpty())
    {
        const int previousIndex = findCameraComboIndex(
            previousCameraProtocol,
            previousCameraId,
            previousAlpacaHost,
            previousAlpacaPort);
        if (previousIndex >= 0)
        {
            QSignalBlocker blocker(ui->cameraCombo);
            ui->cameraCombo->setCurrentIndex(previousIndex);
        }
        return false;
    }

    const QString description = QFileInfo(filePath).fileName();
    ui->cameraCombo->setItemData(comboIndex, filePath, CameraIdRole);
    ui->cameraCombo->setItemData(comboIndex, description, CameraDescriptionRole);
    ui->cameraCombo->setItemText(comboIndex, QStringLiteral("file:%1").arg(description));
    return true;
}

CameraGUI::FrameRateOptions CameraGUI::makeFrameRateOptions(const QSet<int>& fpsValues)
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

void CameraGUI::displaySettings()
{
    setWindowTitle(m_settings.m_title);
    setTitle(m_settings.m_title);

    const int cameraIndex = findCameraComboIndex(
        m_settings.m_cameraProtocol,
        m_settings.m_cameraId,
        m_settings.m_alpacaHost,
        m_settings.m_alpacaPort);
    if (cameraIndex >= 0) {
        ui->cameraCombo->setCurrentIndex(cameraIndex);
    } else if (!m_settings.cameraDisplayName().isEmpty()) {
        ui->cameraCombo->setCurrentText(m_settings.cameraDisplayName());
    }
    updateVideoFileControls();

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
    settingsUI()->alpacaDiscoveryCheck->setChecked(m_settings.m_alpacaDiscoveryEnabled);
    settingsUI()->alpacaApiLogCheck->setChecked(m_settings.m_alpacaApiLogEnabled);
    settingsUI()->alpacaHostEdit->setText(m_settings.m_alpacaHost);
    settingsUI()->alpacaPortSpin->setValue(m_settings.m_alpacaPort);
    settingsUI()->alpacaFocuserEnabledCheck->setChecked(m_settings.m_alpacaFocuserEnabled);
    populateAlpacaAccessoryCombos();
    settingsUI()->alpacaFocuserHostEdit->setText(m_settings.m_alpacaFocuserHost);
    settingsUI()->alpacaFocuserPortSpin->setValue(m_settings.m_alpacaFocuserPort);
    settingsUI()->alpacaFocusPositionSpin->setValue(m_settings.m_alpacaFocusPosition);
    settingsUI()->alpacaFocusStepSizeSpin->setValue(m_settings.m_alpacaFocusStepSize);
    settingsUI()->alpacaFilterWheelEnabledCheck->setChecked(m_settings.m_alpacaFilterWheelEnabled);
    settingsUI()->alpacaFilterWheelHostEdit->setText(m_settings.m_alpacaFilterWheelHost);
    settingsUI()->alpacaFilterWheelPortSpin->setValue(m_settings.m_alpacaFilterWheelPort);
    if (!m_alpacaFilterWheelNames.isEmpty())
    {
        QSignalBlocker blocker(settingsUI()->alpacaFilterWheelPositionCombo);
        settingsUI()->alpacaFilterWheelPositionCombo->clear();
        for (int i = 0; i < m_alpacaFilterWheelNames.size(); ++i) {
            settingsUI()->alpacaFilterWheelPositionCombo->addItem(m_alpacaFilterWheelNames.at(i), i);
        }
        settingsUI()->alpacaFilterWheelPositionCombo->setCurrentIndex(
            qBound(0, m_settings.m_alpacaFilterWheelPosition, settingsUI()->alpacaFilterWheelPositionCombo->count() - 1));
    }
    settingsUI()->cameraBinXSpin->setValue(m_settings.m_cameraBinX);
    settingsUI()->cameraBinYSpin->setValue(m_settings.m_cameraBinY);
    settingsUI()->cameraNumXSpin->setValue(m_settings.m_cameraNumX);
    settingsUI()->cameraNumYSpin->setValue(m_settings.m_cameraNumY);
    settingsUI()->cameraStartXSpin->setValue(m_settings.m_cameraStartX);
    settingsUI()->cameraStartYSpin->setValue(m_settings.m_cameraStartY);

    if (m_alpacaHasNamedGains) {
        settingsUI()->cameraGainCombo->setCurrentIndex(m_settings.m_cameraGain >= 0 ? m_settings.m_cameraGain : 0);
    } else {
        settingsUI()->cameraGainSpin->setValue(m_settings.m_cameraGain >= 0 ? m_settings.m_cameraGain : 0);
        settingsUI()->cameraGainSlider->setValue(m_settings.m_cameraGain >= 0 ? m_settings.m_cameraGain : 0);
    }

    if (m_alpacaHasNamedOffsets) {
        settingsUI()->cameraOffsetCombo->setCurrentIndex(m_settings.m_cameraOffset >= 0 ? m_settings.m_cameraOffset : 0);
    } else {
        settingsUI()->cameraOffsetSpin->setValue(m_settings.m_cameraOffset >= 0 ? m_settings.m_cameraOffset : 0);
        settingsUI()->cameraOffsetSlider->setValue(m_settings.m_cameraOffset >= 0 ? m_settings.m_cameraOffset : 0);
    }

    settingsUI()->alpacaReadoutModeCombo->setCurrentIndex(m_settings.m_cameraReadoutMode);
    settingsUI()->asiCoolerOnCheck->setChecked(m_settings.m_asiCoolerOn > 0);
    settingsUI()->asiTargetTempSpin->setValue(
        m_settings.m_asiTargetTemp == std::numeric_limits<int>::min() ? 0 : m_settings.m_asiTargetTemp);
    settingsUI()->asiUsbBandwidthSpin->setValue(std::max(0, m_settings.m_asiUsbBandwidth));
    settingsUI()->asiHighSpeedModeCheck->setChecked(m_settings.m_asiHighSpeedMode > 0);
    settingsUI()->asiAutoExposureGainCheck->setChecked(m_settings.m_asiAutoExposureGain);
    settingsUI()->asiColorImageTypeCombo->setCurrentIndex(static_cast<int>(m_settings.m_asiColorImageType));
    ui->saveImageCheck->setChecked(m_settings.m_saveImage);
    settingsUI()->imagePathEdit->setText(m_settings.m_imageFileName);
    ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
    settingsUI()->videoPathEdit->setText(m_settings.m_videoFileName);
    settingsUI()->videoHwAccelerationCheck->setChecked(m_settings.m_videoHwAcceleration);
    settingsUI()->recordModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_recordMode));
    ui->stackEnabledButton->setChecked(m_settings.m_stackEnabled);
    settingsUI()->stackFrameCountSpin->setValue(m_settings.m_stackFrameCount);
    settingsUI()->stackMethodCombo->setCurrentIndex(static_cast<int>(m_settings.m_stackMethod));
    settingsUI()->stackAlignmentCombo->setCurrentIndex(static_cast<int>(m_settings.m_stackAlignmentMethod));
    settingsUI()->stackDarkFileEdit->setText(m_settings.m_stackDarkFileName);
    settingsUI()->stackFlatFileEdit->setText(m_settings.m_stackFlatFileName);
    settingsUI()->stackBiasFileEdit->setText(m_settings.m_stackBiasFileName);
    settingsUI()->latitudeSpin->setValue(m_settings.m_latitude);
    settingsUI()->longitudeSpin->setValue(m_settings.m_longitude);
    settingsUI()->altitudeSpin->setValue(m_settings.m_altitude);
    settingsUI()->owmApiKeyEdit->setText(m_settings.m_owmAPIKey);
    settingsUI()->azimuthSpin->setValue(m_settings.m_azimuth);
    settingsUI()->elevationSpin->setValue(m_settings.m_elevation);
    settingsUI()->rollSpin->setValue(m_settings.m_roll);
    settingsUI()->fovSpin->setValue(m_settings.m_fov);
    settingsUI()->lensProjectionCombo->setCurrentIndex(static_cast<int>(m_settings.m_lensProjection));
    settingsUI()->scheduleEnabledCheck->setChecked(m_settings.m_scheduleEnabled);
    settingsUI()->scheduleStartTimeEdit->setTime(parseScheduleTime(m_settings.m_scheduleStartTime, QTime(20, 0, 0)));
    settingsUI()->scheduleEndTimeEdit->setTime(parseScheduleTime(m_settings.m_scheduleEndTime, QTime(6, 0, 0)));
    settingsUI()->scheduleMondayCheck->setChecked((m_settings.m_scheduleWeekdays & (1 << 0)) != 0);
    settingsUI()->scheduleTuesdayCheck->setChecked((m_settings.m_scheduleWeekdays & (1 << 1)) != 0);
    settingsUI()->scheduleWednesdayCheck->setChecked((m_settings.m_scheduleWeekdays & (1 << 2)) != 0);
    settingsUI()->scheduleThursdayCheck->setChecked((m_settings.m_scheduleWeekdays & (1 << 3)) != 0);
    settingsUI()->scheduleFridayCheck->setChecked((m_settings.m_scheduleWeekdays & (1 << 4)) != 0);
    settingsUI()->scheduleSaturdayCheck->setChecked((m_settings.m_scheduleWeekdays & (1 << 5)) != 0);
    settingsUI()->scheduleSundayCheck->setChecked((m_settings.m_scheduleWeekdays & (1 << 6)) != 0);
    populateGs232ControllerCombo();
    applyPositionSync();
    updatePositionControls();
    updateScheduleControls();
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
    settingsUI()->stackCurrentCountValue->setText(QString::number(m_lastStackCount));
    settingsUI()->postProcessUnwarpCheck->setChecked(m_settings.m_postProcessUnwarp);
    settingsUI()->histogramStretchModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_histogramStretch));
    settingsUI()->histogramStretchBlackPointSlider->setValue(static_cast<int>(std::lround(m_settings.m_histogramStretchBlackPoint * 1000.0)));
    settingsUI()->histogramStretchBlackPointSpin->setValue(m_settings.m_histogramStretchBlackPoint);
    settingsUI()->histogramStretchWhitePointSlider->setValue(static_cast<int>(std::lround(m_settings.m_histogramStretchWhitePoint * 1000.0)));
    settingsUI()->histogramStretchWhitePointSpin->setValue(m_settings.m_histogramStretchWhitePoint);
    settingsUI()->histogramStretchGammaSlider->setValue(static_cast<int>(std::lround(m_settings.m_histogramStretchGamma * 100.0)));
    settingsUI()->histogramStretchGammaSpin->setValue(m_settings.m_histogramStretchGamma);
    settingsUI()->histogramStretchAsinhSlider->setValue(static_cast<int>(std::lround(m_settings.m_histogramStretchAsinhStrength * 10.0)));
    settingsUI()->histogramStretchAsinhSpin->setValue(m_settings.m_histogramStretchAsinhStrength);
    settingsUI()->histogramStretchLogSlider->setValue(static_cast<int>(std::lround(m_settings.m_histogramStretchLogStrength * 10.0)));
    settingsUI()->histogramStretchLogSpin->setValue(m_settings.m_histogramStretchLogStrength);
    settingsUI()->postProcessGreyscaleCheck->setChecked(m_settings.m_postProcessGreyscale);
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
    updateHistogramStretchControls();
    ui->invertColorsButton->setChecked(m_settings.m_invertColors);
    ui->overlayDateTimeButton->setChecked(m_settings.m_overlayDateTime);
    settingsUI()->dateTimeFormatEdit->setText(m_settings.m_dateTimeFormat);
    settingsUI()->dateTimePosXSlider->setValue(m_settings.m_dateTimePosX);
    settingsUI()->dateTimePosXValue->setText(QString::number(m_settings.m_dateTimePosX));
    settingsUI()->dateTimePosYSlider->setValue(m_settings.m_dateTimePosY);
    settingsUI()->dateTimePosYValue->setText(QString::number(m_settings.m_dateTimePosY));
    ui->equatorialGridButton->setChecked(m_settings.m_equatorialGrid);
    ui->altAzGridButton->setChecked(m_settings.m_altAzGrid);
    ui->trackObjectsButton->setChecked(m_settings.m_trackObjects);
    settingsUI()->trackObjectMinElevationSpin->setValue(m_settings.m_trackObjectMinElevation);
    settingsUI()->trackObjectFontScaleSpin->setValue(m_settings.m_trackObjectFontScale);
    settingsUI()->gridLabelFontCombo->setCurrentText(m_settings.m_gridLabelFontFamily);
    settingsUI()->gridLabelFontScaleSpin->setValue(m_settings.m_gridLabelFontScale);
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
    settingsUI()->diffMaskOpenSizeSpin->setValue(m_settings.m_diffMaskOpenSize);
    settingsUI()->dilationSpin->setValue(m_settings.m_dilationSize);
    settingsUI()->diffMaskHistoryFramesSpin->setValue(m_settings.m_diffMaskHistoryFrames);
    settingsUI()->diffMaskCloseSizeSpin->setValue(m_settings.m_diffMaskCloseSize);
    settingsUI()->overlayFontCombo->setCurrentText(m_settings.m_overlayFontFamily);
    settingsUI()->overlayFontScaleSpin->setValue(m_settings.m_overlayFontScale);
    settingsUI()->overlayTextFontCombo->setCurrentText(m_settings.m_overlayTextFontFamily);
    settingsUI()->overlayTextFontScaleSpin->setValue(m_settings.m_overlayTextFontScale);
    settingsUI()->detectionRoiXSpin->setValue(m_settings.m_detectionRoiX);
    settingsUI()->detectionRoiYSpin->setValue(m_settings.m_detectionRoiY);
    settingsUI()->detectionRoiWidthSpin->setValue(m_settings.m_detectionRoiWidth);
    settingsUI()->detectionRoiHeightSpin->setValue(m_settings.m_detectionRoiHeight);
    ui->motionDetectButton->setChecked(m_settings.m_motionDetect);
    settingsUI()->motionBackgroundSubtractorCombo->setCurrentIndex(static_cast<int>(m_settings.m_motionBackgroundSubtractor));
    settingsUI()->motionHistorySpin->setValue(m_settings.m_motionHistory);
    settingsUI()->motionVarThresholdSpin->setValue(m_settings.m_motionVarThreshold);
    settingsUI()->motionLearningRateSpin->setValue(m_settings.m_motionLearningRate);
    settingsUI()->motionConfirmFramesSpin->setValue(m_settings.m_motionConfirmFrames);
    settingsUI()->motionDownscaleCombo->setCurrentIndex(
        qFuzzyCompare(m_settings.m_motionDownscale, 0.5) ? 1 :
        qFuzzyCompare(m_settings.m_motionDownscale, 0.25) ? 2 : 0);
    settingsUI()->motionDetectShadowsCheck->setChecked(m_settings.m_motionDetectShadows);
    settingsUI()->motionOpenSizeSpin->setValue(m_settings.m_motionOpenSize);
    settingsUI()->motionCloseSizeSpin->setValue(m_settings.m_motionCloseSize);
    settingsUI()->motionPersistenceFramesSpin->setValue(m_settings.m_motionPersistenceFrames);
    settingsUI()->minContourAreaSpin->setValue(m_settings.m_minContourArea);
    settingsUI()->motionExclusionShowButton->setChecked(m_showMotionExclusionRects);
    updateMotionExclusionRectsTable();
    updateColorButton(settingsUI()->dateTimeColorButton, m_settings.m_dateTimeColor);
    updateColorButton(settingsUI()->equatorialGridColorButton, m_settings.m_equatorialGridColor);
    updateColorButton(settingsUI()->altAzGridColorButton, m_settings.m_altAzGridColor);
    updateColorButton(settingsUI()->trackObjectColorButton, m_settings.m_trackObjectColor);
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
    updateCameraSettingsVisibility();
    updateCameraStatusDisplay();
    applyVideoToolTip();
    applyImageToolTip();
}

void CameraGUI::applySetting(const QString& settingsKey)
{
    applySettings({settingsKey});
}

void CameraGUI::applySettings(const QStringList& settingsKeys, bool force)
{
    m_settingsKeys.append(settingsKeys);

    if (!m_doApplySettings) {
        return;
    }

    if (force) {
        m_forceSettings = true;
    }

    // Combine updates to avoid applying settings in h/w multiple times when multiple values are changed at once, as that can be slow
    if (!m_updateTimer.isActive()) {
        m_updateTimer.start(100);
    }
}

void CameraGUI::applyAllSettings()
{
    applySettings(QStringList(), true);
}

void CameraGUI::populateAlpacaAccessoryCombos()
{
    auto populateCombo = [](QComboBox *combo,
                            const QList<AlpacaDeviceInfo>& entries,
                            const QString& currentHost,
                            quint16 currentPort,
                            int currentDeviceNumber) -> bool
    {
        QHash<QString, int> displayCounts;

        for (const AlpacaDeviceInfo& entry : entries)
        {
            const QString displayKey = entry.m_description.isEmpty() ? entry.m_id : entry.m_description;
            displayCounts[displayKey] = displayCounts.value(displayKey) + 1;
        }

        QSignalBlocker blocker(combo);
        combo->clear();

        int selectedIndex = -1;
        bool foundSelection = false;

        for (const AlpacaDeviceInfo& entry : entries)
        {
            bool ok = false;
            const int deviceNumber = entry.m_id.toInt(&ok);
            const int safeDeviceNumber = ok ? deviceNumber : 0;
            QString displayText = entry.m_description.isEmpty() ? entry.m_id : entry.m_description;

            if (displayCounts.value(displayText) > 1 || displayText.isEmpty()) {
                displayText = QStringLiteral("%1 (%2:%3 #%4)")
                    .arg(displayText.isEmpty() ? QStringLiteral("Device") : displayText)
                    .arg(entry.m_host)
                    .arg(entry.m_port)
                    .arg(safeDeviceNumber);
            }

            combo->addItem(displayText);
            const int itemIndex = combo->count() - 1;
            combo->setItemData(itemIndex, safeDeviceNumber, AccessoryDeviceNumberRole);
            combo->setItemData(itemIndex, entry.m_description, AccessoryDescriptionRole);
            combo->setItemData(itemIndex, entry.m_host, AccessoryAlpacaHostRole);
            combo->setItemData(itemIndex, static_cast<int>(entry.m_port), AccessoryAlpacaPortRole);

            if ((entry.m_host == currentHost) && (entry.m_port == currentPort) && (safeDeviceNumber == currentDeviceNumber)) {
                selectedIndex = itemIndex;
                foundSelection = true;
            }
        }

        if (selectedIndex >= 0) {
            combo->setCurrentIndex(selectedIndex);
        } else if (combo->count() > 0) {
            combo->setCurrentIndex(0);
        }

        return foundSelection;
    };

    const bool focuserSelectionFound = populateCombo(settingsUI()->alpacaFocuserCombo,
        m_discoveredAlpacaFocusers,
        m_settings.m_alpacaFocuserHost,
        m_settings.m_alpacaFocuserPort,
        m_settings.m_alpacaFocuserDeviceNumber);

    if (!focuserSelectionFound && (settingsUI()->alpacaFocuserCombo->count() > 0))
    {
        const int index = settingsUI()->alpacaFocuserCombo->currentIndex();
        m_settings.m_alpacaFocuserHost = settingsUI()->alpacaFocuserCombo->itemData(index, AccessoryAlpacaHostRole).toString();
        m_settings.m_alpacaFocuserPort = static_cast<uint16_t>(settingsUI()->alpacaFocuserCombo->itemData(index, AccessoryAlpacaPortRole).toUInt());
        m_settings.m_alpacaFocuserDeviceNumber = settingsUI()->alpacaFocuserCombo->itemData(index, AccessoryDeviceNumberRole).toInt();
    }

    const bool filterWheelSelectionFound = populateCombo(settingsUI()->alpacaFilterWheelCombo,
        m_discoveredAlpacaFilterWheels,
        m_settings.m_alpacaFilterWheelHost,
        m_settings.m_alpacaFilterWheelPort,
        m_settings.m_alpacaFilterWheelDeviceNumber);

    if (!filterWheelSelectionFound && (settingsUI()->alpacaFilterWheelCombo->count() > 0))
    {
        const int index = settingsUI()->alpacaFilterWheelCombo->currentIndex();
        m_settings.m_alpacaFilterWheelHost = settingsUI()->alpacaFilterWheelCombo->itemData(index, AccessoryAlpacaHostRole).toString();
        m_settings.m_alpacaFilterWheelPort = static_cast<uint16_t>(settingsUI()->alpacaFilterWheelCombo->itemData(index, AccessoryAlpacaPortRole).toUInt());
        m_settings.m_alpacaFilterWheelDeviceNumber = settingsUI()->alpacaFilterWheelCombo->itemData(index, AccessoryDeviceNumberRole).toInt();
    }
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
    updateMotionExclusionPreview();
}

void CameraGUI::makeUIConnections()
{
    QObject::connect(ui->cameraSettingsButton, &QToolButton::clicked, this, &CameraGUI::on_cameraSettingsButton_clicked);
    QObject::connect(ui->startStop, &QPushButton::clicked, this, &CameraGUI::on_startStop_clicked);
    QObject::connect(ui->refreshCamerasButton, &QPushButton::clicked, this, &CameraGUI::on_refreshCamerasButton_clicked);
    QObject::connect(ui->cameraCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_cameraCombo_currentIndexChanged);
    QObject::connect(ui->browseVideoFileButton, &QToolButton::clicked, this, &CameraGUI::on_browseVideoFileButton_clicked);
    QObject::connect(ui->restartVideo, &QToolButton::clicked, this, &CameraGUI::on_restartVideo_clicked);
    QObject::connect(ui->playPauseVideo, &ButtonSwitch::clicked, this, &CameraGUI::on_playPauseVideo_clicked);
    QObject::connect(ui->loopVideo, &ButtonSwitch::clicked, this, &CameraGUI::on_loopVideo_clicked);
    QObject::connect(ui->playbackRateSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_playbackRateSpin_valueChanged);
    QObject::connect(ui->playbackPositionSlider, &QSlider::sliderMoved, this, &CameraGUI::on_playbackPositionSlider_sliderMoved);
    QObject::connect(ui->playbackPositionSlider, &QSlider::sliderReleased, this, &CameraGUI::on_playbackPositionSlider_sliderReleased);
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
    QObject::connect(settingsUI()->alpacaDiscoveryCheck, &QCheckBox::toggled, this, &CameraGUI::on_alpacaDiscoveryCheck_toggled);
    QObject::connect(settingsUI()->alpacaApiLogCheck, &QCheckBox::toggled, this, &CameraGUI::on_alpacaApiLogCheck_toggled);
    QObject::connect(settingsUI()->alpacaHostEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_alpacaHostEdit_editingFinished);
    QObject::connect(settingsUI()->alpacaPortSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaPortSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaFocuserEnabledCheck, &QCheckBox::toggled, this, &CameraGUI::on_alpacaFocuserEnabledCheck_toggled);
    QObject::connect(settingsUI()->alpacaFocuserCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaFocuserCombo_currentIndexChanged);
    QObject::connect(settingsUI()->alpacaFocuserHostEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_alpacaFocuserHostEdit_editingFinished);
    QObject::connect(settingsUI()->alpacaFocuserPortSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaFocuserPortSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaFocusPositionSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaFocusPositionSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaFocusStepSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaFocusStepSizeSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaFilterWheelEnabledCheck, &QCheckBox::toggled, this, &CameraGUI::on_alpacaFilterWheelEnabledCheck_toggled);
    QObject::connect(settingsUI()->alpacaFilterWheelCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaFilterWheelCombo_currentIndexChanged);
    QObject::connect(settingsUI()->alpacaFilterWheelHostEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_alpacaFilterWheelHostEdit_editingFinished);
    QObject::connect(settingsUI()->alpacaFilterWheelPortSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_alpacaFilterWheelPortSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaFilterWheelPositionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaFilterWheelPositionCombo_currentIndexChanged);
    QObject::connect(settingsUI()->cameraBinXSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraBinXSpin_valueChanged);
    QObject::connect(settingsUI()->cameraBinYSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraBinYSpin_valueChanged);
    QObject::connect(settingsUI()->cameraNumXSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraNumXSpin_valueChanged);
    QObject::connect(settingsUI()->cameraNumYSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraNumYSpin_valueChanged);
    QObject::connect(settingsUI()->cameraStartXSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraStartXSpin_valueChanged);
    QObject::connect(settingsUI()->cameraStartYSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraStartYSpin_valueChanged);
    QObject::connect(settingsUI()->cameraGainCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_cameraGainCombo_currentIndexChanged);
    QObject::connect(settingsUI()->cameraGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_cameraGainSlider_valueChanged);
    QObject::connect(settingsUI()->cameraGainSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraGainSpin_valueChanged);
    QObject::connect(settingsUI()->cameraOffsetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_cameraOffsetCombo_currentIndexChanged);
    QObject::connect(settingsUI()->cameraOffsetSlider, &QSlider::valueChanged, this, &CameraGUI::on_cameraOffsetSlider_valueChanged);
    QObject::connect(settingsUI()->cameraOffsetSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_cameraOffsetSpin_valueChanged);
    QObject::connect(settingsUI()->alpacaReadoutModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_alpacaReadoutModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->asiCoolerOnCheck, &QCheckBox::toggled, this, &CameraGUI::on_asiCoolerOnCheck_toggled);
    QObject::connect(settingsUI()->asiTargetTempSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_asiTargetTempSpin_valueChanged);
    QObject::connect(settingsUI()->asiUsbBandwidthSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_asiUsbBandwidthSpin_valueChanged);
    QObject::connect(settingsUI()->asiHighSpeedModeCheck, &QCheckBox::toggled, this, &CameraGUI::on_asiHighSpeedModeCheck_toggled);
    QObject::connect(settingsUI()->asiAutoExposureGainCheck, &QCheckBox::toggled, this, &CameraGUI::on_asiAutoExposureGainCheck_toggled);
    QObject::connect(settingsUI()->asiColorImageTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_asiColorImageTypeCombo_currentIndexChanged);
    QObject::connect(ui->saveImageCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveImageCheck_toggled);
    QObject::connect(settingsUI()->imagePathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_imagePathEdit_editingFinished);
    QObject::connect(settingsUI()->imagePathButton, &QToolButton::clicked, this, &CameraGUI::on_imagePathButton_clicked);
    QObject::connect(ui->saveVideoCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveVideoCheck_toggled);
    QObject::connect(settingsUI()->videoPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_videoPathEdit_editingFinished);
    QObject::connect(settingsUI()->videoPathButton, &QToolButton::clicked, this, &CameraGUI::on_videoPathButton_clicked);
    QObject::connect(settingsUI()->videoHwAccelerationCheck, &QCheckBox::toggled, this, &CameraGUI::on_videoHwAccelerationCheck_toggled);
    QObject::connect(settingsUI()->recordModeCombo, &QComboBox::currentIndexChanged, this, &CameraGUI::on_recordModeCombo_currentIndexChanged);
    QObject::connect(ui->stackEnabledButton, &QToolButton::toggled, this, &CameraGUI::on_stackEnabledCheck_toggled);
    QObject::connect(settingsUI()->stackFrameCountSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_stackFrameCountSpin_valueChanged);
    QObject::connect(settingsUI()->stackMethodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_stackMethodCombo_currentIndexChanged);
    QObject::connect(settingsUI()->stackAlignmentCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_stackAlignmentCombo_currentIndexChanged);
    QObject::connect(settingsUI()->stackDarkFileEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_stackDarkFileEdit_editingFinished);
    QObject::connect(settingsUI()->stackDarkFileButton, &QToolButton::clicked, this, &CameraGUI::on_stackDarkFileButton_clicked);
    QObject::connect(settingsUI()->stackFlatFileEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_stackFlatFileEdit_editingFinished);
    QObject::connect(settingsUI()->stackFlatFileButton, &QToolButton::clicked, this, &CameraGUI::on_stackFlatFileButton_clicked);
    QObject::connect(settingsUI()->stackBiasFileEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_stackBiasFileEdit_editingFinished);
    QObject::connect(settingsUI()->stackBiasFileButton, &QToolButton::clicked, this, &CameraGUI::on_stackBiasFileButton_clicked);
    QObject::connect(settingsUI()->latitudeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_latitudeSpin_valueChanged);
    QObject::connect(settingsUI()->longitudeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_longitudeSpin_valueChanged);
    QObject::connect(settingsUI()->altitudeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_altitudeSpin_valueChanged);
    QObject::connect(settingsUI()->owmApiKeyEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_owmApiKeyEdit_editingFinished);
    QObject::connect(settingsUI()->useMyPositionButton, &QToolButton::clicked, this, &CameraGUI::on_useMyPositionButton_clicked);
    QObject::connect(settingsUI()->azimuthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_azimuthSpin_valueChanged);
    QObject::connect(settingsUI()->elevationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_elevationSpin_valueChanged);
    QObject::connect(settingsUI()->rollSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_rollSpin_valueChanged);
    QObject::connect(settingsUI()->rotatorControllerCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_rotatorControllerCombo_currentIndexChanged);
    QObject::connect(settingsUI()->fovSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_fovSpin_valueChanged);
    QObject::connect(settingsUI()->lensProjectionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_lensProjectionCombo_currentIndexChanged);
    QObject::connect(settingsUI()->scheduleEnabledCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleEnabledCheck_toggled);
    QObject::connect(settingsUI()->scheduleStartTimeEdit, &QTimeEdit::timeChanged, this, &CameraGUI::on_scheduleStartTimeEdit_timeChanged);
    QObject::connect(settingsUI()->scheduleEndTimeEdit, &QTimeEdit::timeChanged, this, &CameraGUI::on_scheduleEndTimeEdit_timeChanged);
    QObject::connect(settingsUI()->scheduleMondayCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleWeekdayCheck_toggled);
    QObject::connect(settingsUI()->scheduleTuesdayCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleWeekdayCheck_toggled);
    QObject::connect(settingsUI()->scheduleWednesdayCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleWeekdayCheck_toggled);
    QObject::connect(settingsUI()->scheduleThursdayCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleWeekdayCheck_toggled);
    QObject::connect(settingsUI()->scheduleFridayCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleWeekdayCheck_toggled);
    QObject::connect(settingsUI()->scheduleSaturdayCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleWeekdayCheck_toggled);
    QObject::connect(settingsUI()->scheduleSundayCheck, &QCheckBox::toggled, this, &CameraGUI::on_scheduleWeekdayCheck_toggled);
    QObject::connect(settingsUI()->postProcessWhiteBalanceModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_postProcessWhiteBalanceModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceRedGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceRedGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceRedGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceRedGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceGreenGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceGreenGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceGreenGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceGreenGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceBlueGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceBlueGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceBlueGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceBlueGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessUnwarpCheck, &QCheckBox::toggled, this, &CameraGUI::on_postProcessUnwarpCheck_toggled);
    QObject::connect(settingsUI()->histogramStretchModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_histogramStretchModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->histogramStretchBlackPointSlider, &QSlider::valueChanged, this, &CameraGUI::on_histogramStretchBlackPointSlider_valueChanged);
    QObject::connect(settingsUI()->histogramStretchBlackPointSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_histogramStretchBlackPointSpin_valueChanged);
    QObject::connect(settingsUI()->histogramStretchWhitePointSlider, &QSlider::valueChanged, this, &CameraGUI::on_histogramStretchWhitePointSlider_valueChanged);
    QObject::connect(settingsUI()->histogramStretchWhitePointSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_histogramStretchWhitePointSpin_valueChanged);
    QObject::connect(settingsUI()->histogramStretchGammaSlider, &QSlider::valueChanged, this, &CameraGUI::on_histogramStretchGammaSlider_valueChanged);
    QObject::connect(settingsUI()->histogramStretchGammaSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_histogramStretchGammaSpin_valueChanged);
    QObject::connect(settingsUI()->histogramStretchAsinhSlider, &QSlider::valueChanged, this, &CameraGUI::on_histogramStretchAsinhSlider_valueChanged);
    QObject::connect(settingsUI()->histogramStretchAsinhSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_histogramStretchAsinhSpin_valueChanged);
    QObject::connect(settingsUI()->histogramStretchLogSlider, &QSlider::valueChanged, this, &CameraGUI::on_histogramStretchLogSlider_valueChanged);
    QObject::connect(settingsUI()->histogramStretchLogSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_histogramStretchLogSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessGreyscaleCheck, &QCheckBox::toggled, this, &CameraGUI::on_postProcessGreyscaleCheck_toggled);
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
    QObject::connect(ui->equatorialGridButton, &QToolButton::toggled, this, &CameraGUI::on_equatorialGridCheck_toggled);
    QObject::connect(settingsUI()->equatorialGridColorButton, &QToolButton::clicked, this, &CameraGUI::on_equatorialGridColorButton_clicked);
    QObject::connect(ui->altAzGridButton, &QToolButton::toggled, this, &CameraGUI::on_altAzGridCheck_toggled);
    QObject::connect(settingsUI()->altAzGridColorButton, &QToolButton::clicked, this, &CameraGUI::on_altAzGridColorButton_clicked);
    QObject::connect(ui->trackObjectsButton, &QToolButton::toggled, this, &CameraGUI::on_trackObjectsCheck_toggled);
    QObject::connect(settingsUI()->trackObjectMinElevationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_trackObjectMinElevationSpin_valueChanged);
    QObject::connect(settingsUI()->trackObjectColorButton, &QToolButton::clicked, this, &CameraGUI::on_trackObjectColorButton_clicked);
    QObject::connect(settingsUI()->trackObjectFontScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_trackObjectFontScaleSpin_valueChanged);
    QObject::connect(settingsUI()->gridLabelFontCombo, &QFontComboBox::currentFontChanged, this, &CameraGUI::on_gridLabelFontCombo_currentFontChanged);
    QObject::connect(settingsUI()->gridLabelFontScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_gridLabelFontScaleSpin_valueChanged);
    QObject::connect(ui->overlayTextButton, &QToolButton::toggled, this, &CameraGUI::on_overlayTextButton_toggled);
    QObject::connect(settingsUI()->overlayTextColorButton, &QToolButton::clicked, this, &CameraGUI::on_overlayTextColorButton_clicked);
    QObject::connect(settingsUI()->overlayTextEdit, &QTextEdit::textChanged, this, &CameraGUI::on_overlayTextEdit_textChanged);
    QObject::connect(settingsUI()->overlayTextPosXSlider, &QSlider::valueChanged, this, &CameraGUI::on_overlayTextPosXSlider_valueChanged);
    QObject::connect(settingsUI()->overlayTextPosYSlider, &QSlider::valueChanged, this, &CameraGUI::on_overlayTextPosYSlider_valueChanged);
    QObject::connect(ui->diffMaskButton, &QToolButton::toggled, this, &CameraGUI::on_diffMaskButton_toggled);
    QObject::connect(settingsUI()->diffThresholdSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_diffThresholdSpin_valueChanged);
    QObject::connect(settingsUI()->diffMaskOpenSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_diffMaskOpenSizeSpin_valueChanged);
    QObject::connect(settingsUI()->dilationSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_dilationSpin_valueChanged);
    QObject::connect(settingsUI()->diffMaskHistoryFramesSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_diffMaskHistoryFramesSpin_valueChanged);
    QObject::connect(settingsUI()->diffMaskCloseSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_diffMaskCloseSizeSpin_valueChanged);
    QObject::connect(ui->histogramButton, &QToolButton::clicked, this, &CameraGUI::on_histogramButton_clicked);
    QObject::connect(settingsUI()->defaultColorSettingsButton, &QToolButton::clicked, this, &CameraGUI::on_defaultColorSettingsButton_clicked);
    QObject::connect(settingsUI()->overlayFontCombo, &QFontComboBox::currentFontChanged, this, &CameraGUI::on_overlayFontCombo_currentFontChanged);
    QObject::connect(settingsUI()->overlayFontScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_overlayFontScaleSpin_valueChanged);
    QObject::connect(settingsUI()->overlayTextFontCombo, &QFontComboBox::currentFontChanged, this, &CameraGUI::on_overlayTextFontCombo_currentFontChanged);
    QObject::connect(settingsUI()->overlayTextFontScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_overlayTextFontScaleSpin_valueChanged);
    QObject::connect(settingsUI()->detectionRoiXSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_detectionRoiXSpin_valueChanged);
    QObject::connect(settingsUI()->detectionRoiYSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_detectionRoiYSpin_valueChanged);
    QObject::connect(settingsUI()->detectionRoiWidthSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_detectionRoiWidthSpin_valueChanged);
    QObject::connect(settingsUI()->detectionRoiHeightSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_detectionRoiHeightSpin_valueChanged);
    QObject::connect(ui->motionDetectButton, &QToolButton::toggled, this, &CameraGUI::on_motionDetectButton_toggled);
    QObject::connect(settingsUI()->motionBackgroundSubtractorCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_motionBackgroundSubtractorCombo_currentIndexChanged);
    QObject::connect(settingsUI()->motionHistorySpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_motionHistorySpin_valueChanged);
    QObject::connect(settingsUI()->motionVarThresholdSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_motionVarThresholdSpin_valueChanged);
    QObject::connect(settingsUI()->motionLearningRateSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_motionLearningRateSpin_valueChanged);
    QObject::connect(settingsUI()->motionConfirmFramesSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_motionConfirmFramesSpin_valueChanged);
    QObject::connect(settingsUI()->motionDownscaleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_motionDownscaleCombo_currentIndexChanged);
    QObject::connect(settingsUI()->motionDetectShadowsCheck, &QCheckBox::toggled, this, &CameraGUI::on_motionDetectShadowsCheck_toggled);
    QObject::connect(settingsUI()->motionOpenSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_motionOpenSizeSpin_valueChanged);
    QObject::connect(settingsUI()->motionCloseSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_motionCloseSizeSpin_valueChanged);
    QObject::connect(settingsUI()->motionPersistenceFramesSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_motionPersistenceFramesSpin_valueChanged);
    QObject::connect(settingsUI()->minContourAreaSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_minContourAreaSpin_valueChanged);
    QObject::connect(settingsUI()->motionBoxColorButton, &QToolButton::clicked, this, &CameraGUI::on_motionBoxColorButton_clicked);
    QObject::connect(settingsUI()->motionExclusionAddButton, &QToolButton::clicked, this, &CameraGUI::on_motionExclusionAddButton_clicked);
    QObject::connect(settingsUI()->motionExclusionRemoveButton, &QToolButton::clicked, this, &CameraGUI::on_motionExclusionRemoveButton_clicked);
    QObject::connect(settingsUI()->motionExclusionShowButton, &QToolButton::toggled, this, &CameraGUI::on_motionExclusionShowButton_toggled);
    QObject::connect(settingsUI()->motionExclusionTable, &QTableWidget::itemChanged, this, &CameraGUI::on_motionExclusionTable_itemChanged);
    QObject::connect(ui->spectrumOverlayButton, &QToolButton::toggled, this, &CameraGUI::on_spectrumOverlayButton_toggled);
    QObject::connect(settingsUI()->spectrumDeviceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_spectrumDeviceCombo_currentIndexChanged);
    QObject::connect(settingsUI()->spectrumOffsetXSlider, &QSlider::valueChanged, this, &CameraGUI::on_spectrumOffsetXSlider_valueChanged);
    QObject::connect(settingsUI()->spectrumOffsetYSlider, &QSlider::valueChanged, this, &CameraGUI::on_spectrumOffsetYSlider_valueChanged);
    QObject::connect(settingsUI()->spectrumScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_spectrumScaleSpin_valueChanged);
    QObject::connect(ui->yoloButton, &QToolButton::toggled, this, &CameraGUI::on_yoloButton_toggled);
    QObject::connect(settingsUI()->yoloModelPathCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_yoloModelPathCombo_currentIndexChanged);
    if (settingsUI()->yoloModelPathCombo->lineEdit()) {
        QObject::connect(settingsUI()->yoloModelPathCombo->lineEdit(), &QLineEdit::editingFinished, this, &CameraGUI::on_yoloModelPathEdit_editingFinished);
    }
    QObject::connect(settingsUI()->yoloModelPathButton, &QPushButton::clicked, this, &CameraGUI::on_yoloModelPathButton_clicked);
    QObject::connect(settingsUI()->yoloLabelsPathCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_yoloLabelsPathCombo_currentIndexChanged);
    if (settingsUI()->yoloLabelsPathCombo->lineEdit()) {
        QObject::connect(settingsUI()->yoloLabelsPathCombo->lineEdit(), &QLineEdit::editingFinished, this, &CameraGUI::on_yoloLabelsPathEdit_editingFinished);
    }
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
    settingsUI()->intervalUnitsCombo->setVisible(intervalMode);
    settingsUI()->captureValueStack->setCurrentWidget(intervalMode ? settingsUI()->intervalPage : settingsUI()->frameRatePage);
}

void CameraGUI::updateExposureControls()
{
    const double unitScaleMs = currentExposureUnitScaleMs(settingsUI());
    const double minimum = m_exposureMinimumMs / unitScaleMs;
    const double maximum = m_exposureMaximumMs / unitScaleMs;
    const double singleStep = std::max(0.000001, m_exposureStepMs / unitScaleMs);
    const double value = qBound(minimum, m_settings.m_exposureTimeMs / unitScaleMs, maximum);

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
        settingsUI()->exposureSlider->setMaximum(1000);
        settingsUI()->exposureSlider->setValue(exposureValueToSlider(settingsUI()->exposureSpin, value));
    }
}

void CameraGUI::updateVideoFileControls()
{
    const bool fileCameraSelected = m_settings.isFileCamera();
    const bool hasVideoFile = fileCameraSelected && !m_settings.m_videoFileCameraPath.isEmpty();
    const bool hasPlaybackPosition = hasVideoFile && (m_mediaPlayerDurationMs > 0);

    ui->browseVideoFileButton->setVisible(fileCameraSelected);
    ui->browseVideoFileButton->setEnabled(fileCameraSelected);
    ui->restartVideo->setVisible(fileCameraSelected);
    ui->restartVideo->setEnabled(hasVideoFile);
    ui->playPauseVideo->setVisible(fileCameraSelected);
    ui->playPauseVideo->setEnabled(hasVideoFile);
    ui->loopVideo->setVisible(fileCameraSelected);
    ui->loopVideo->setEnabled(hasVideoFile);
    ui->playbackRateSpin->setVisible(fileCameraSelected);
    ui->playbackRateSpin->setEnabled(hasVideoFile);
    ui->playbackPositionSlider->setVisible(fileCameraSelected);
    ui->playbackPositionSlider->setEnabled(hasPlaybackPosition);
    ui->videoLine->setVisible(fileCameraSelected);
}

void CameraGUI::populateGs232ControllerCombo()
{
    const QString currentSelection = m_settings.m_rotator;
    QSignalBlocker blocker(settingsUI()->rotatorControllerCombo);
    settingsUI()->rotatorControllerCombo->clear();
    settingsUI()->rotatorControllerCombo->addItem(tr("None"), QString());

    std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();

    for (int featureSetIndex = 0; featureSetIndex < static_cast<int>(featureSets.size()); ++featureSetIndex)
    {
        FeatureSet *featureSet = featureSets[featureSetIndex];

        if (!featureSet) {
            continue;
        }

        for (int featureIndex = 0; featureIndex < featureSet->getNumberOfFeatures(); ++featureIndex)
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

            const QString selectionId = QStringLiteral("%1:%2").arg(featureSetIndex).arg(featureIndex);
            settingsUI()->rotatorControllerCombo->addItem(
                QStringLiteral("F%1:%2 %3").arg(featureSetIndex).arg(featureIndex).arg(title),
                selectionId);
        }
    }

    const int index = settingsUI()->rotatorControllerCombo->findData(currentSelection);
    settingsUI()->rotatorControllerCombo->setCurrentIndex(index >= 0 ? index : 0);
}

void CameraGUI::applyPositionSync()
{
    if (m_settings.m_positionSync) {
        settingsUI()->useMyPositionButton->setStyleSheet(
            QStringLiteral("QToolButton{ background-color: %1; }")
                .arg(palette().highlight().color().darker(150).name()));
        syncFromMainSettings();
        connect(&MainCore::instance()->getSettings(), &MainSettings::preferenceChanged,
            this, &CameraGUI::preferenceChanged, Qt::UniqueConnection);
    } else {
        settingsUI()->useMyPositionButton->setStyleSheet(
            QStringLiteral("QToolButton{ background-color: %1; }")
                .arg(palette().button().color().name()));
        disconnect(&MainCore::instance()->getSettings(), &MainSettings::preferenceChanged,
            this, &CameraGUI::preferenceChanged);
    }
}

void CameraGUI::updatePositionControls()
{
    const bool azElSynced = !m_settings.m_rotator.isEmpty();
    settingsUI()->latitudeSpin->setReadOnly(m_settings.m_positionSync);
    settingsUI()->longitudeSpin->setReadOnly(m_settings.m_positionSync);
    settingsUI()->altitudeSpin->setReadOnly(m_settings.m_positionSync);
    settingsUI()->azimuthSpin->setReadOnly(azElSynced);
    settingsUI()->elevationSpin->setReadOnly(azElSynced);
}

void CameraGUI::updateScheduleControls()
{
    const bool enabled = m_settings.m_scheduleEnabled;
    settingsUI()->scheduleStartTimeEdit->setEnabled(enabled);
    settingsUI()->scheduleEndTimeEdit->setEnabled(enabled);
    settingsUI()->scheduleMondayCheck->setEnabled(enabled);
    settingsUI()->scheduleTuesdayCheck->setEnabled(enabled);
    settingsUI()->scheduleWednesdayCheck->setEnabled(enabled);
    settingsUI()->scheduleThursdayCheck->setEnabled(enabled);
    settingsUI()->scheduleFridayCheck->setEnabled(enabled);
    settingsUI()->scheduleSaturdayCheck->setEnabled(enabled);
    settingsUI()->scheduleSundayCheck->setEnabled(enabled);

    QString statusText;
    if (!enabled) {
        statusText = tr("Disabled");
    } else if (m_scheduleManualStartLatch) {
        statusText = tr("Manual run until next window");
    } else if (m_scheduleManualStopLatch) {
        statusText = tr("Paused until next window");
    } else if (m_settings.m_scheduleWeekdays == 0) {
        statusText = tr("Inactive: no weekdays selected");
    } else if (m_settings.m_scheduleStartTime == m_settings.m_scheduleEndTime) {
        statusText = tr("Inactive: start and end are the same");
    } else if (isWithinScheduleWindow()) {
        statusText = tr("Inside schedule window");
    } else {
        statusText = tr("Outside schedule window");
    }

    settingsUI()->scheduleStatusValue->setText(statusText);
}

void CameraGUI::updateScheduledCapture()
{
    const bool withinWindow = isWithinScheduleWindow();

    if (!m_scheduleLastWithinWindow && withinWindow) {
        m_scheduleManualStartLatch = false;
    }

    if (m_scheduleLastWithinWindow && !withinWindow) {
        m_scheduleManualStopLatch = false;
    }
    m_scheduleLastWithinWindow = withinWindow;

    updateScheduleControls();

    if (!m_settings.m_scheduleEnabled || (m_settings.m_scheduleStartTime == m_settings.m_scheduleEndTime) || (m_settings.m_scheduleWeekdays == 0)) {
        return;
    }

    const bool shouldRun = (withinWindow && !m_scheduleManualStopLatch) || (!withinWindow && m_scheduleManualStartLatch);
    const int state = m_camera->getState();

    if (shouldRun && (state == Feature::StIdle)) {
        m_camera->getInputMessageQueue()->push(Camera::MsgStartStop::create(true));
    } else if (!shouldRun && (state == Feature::StRunning)) {
        m_camera->getInputMessageQueue()->push(Camera::MsgStartStop::create(false));
    }
}

bool CameraGUI::isWithinScheduleWindow() const
{
    const QTime startTime = parseScheduleTime(m_settings.m_scheduleStartTime, QTime(20, 0, 0));
    const QTime endTime = parseScheduleTime(m_settings.m_scheduleEndTime, QTime(6, 0, 0));

    if (!startTime.isValid() || !endTime.isValid() || (startTime == endTime) || (m_settings.m_scheduleWeekdays == 0)) {
        return false;
    }

    const QTime now = QTime::currentTime();
    const int today = QDate::currentDate().dayOfWeek();
    const int previousDay = (today == 1) ? 7 : (today - 1);

    if (startTime < endTime) {
        return isScheduleWeekdaySelected(today) && (now >= startTime) && (now < endTime);
    }

    if (now >= startTime) {
        return isScheduleWeekdaySelected(today);
    }

    return (now < endTime) && isScheduleWeekdaySelected(previousDay);
}

bool CameraGUI::isScheduleWeekdaySelected(int dayOfWeek) const
{
    if ((dayOfWeek < 1) || (dayOfWeek > 7)) {
        return false;
    }

    return (m_settings.m_scheduleWeekdays & (1 << (dayOfWeek - 1))) != 0;
}

int CameraGUI::scheduleWeekdayMaskFromUi() const
{
    int mask = 0;
    if (settingsUI()->scheduleMondayCheck->isChecked()) { mask |= (1 << 0); }
    if (settingsUI()->scheduleTuesdayCheck->isChecked()) { mask |= (1 << 1); }
    if (settingsUI()->scheduleWednesdayCheck->isChecked()) { mask |= (1 << 2); }
    if (settingsUI()->scheduleThursdayCheck->isChecked()) { mask |= (1 << 3); }
    if (settingsUI()->scheduleFridayCheck->isChecked()) { mask |= (1 << 4); }
    if (settingsUI()->scheduleSaturdayCheck->isChecked()) { mask |= (1 << 5); }
    if (settingsUI()->scheduleSundayCheck->isChecked()) { mask |= (1 << 6); }
    return mask;
}

QTime CameraGUI::parseScheduleTime(const QString& timeText, const QTime& fallbackTime)
{
    const QTime parsedTime = QTime::fromString(timeText, QStringLiteral("HH:mm:ss"));
    return parsedTime.isValid() ? parsedTime : fallbackTime;
}

void CameraGUI::syncFromMainSettings()
{
    settingsUI()->latitudeSpin->setValue(MainCore::instance()->getSettings().getLatitude());
    settingsUI()->longitudeSpin->setValue(MainCore::instance()->getSettings().getLongitude());
    settingsUI()->altitudeSpin->setValue(MainCore::instance()->getSettings().getAltitude());
}

QPair<int, int> CameraGUI::selectedGs232ControllerIndices() const
{
    const QString selection = m_settings.m_rotator.trimmed();
    const QStringList parts = selection.split(QLatin1Char(':'));

    if (parts.size() != 2) {
        return qMakePair(-1, -1);
    }

    bool okFeatureSet = false;
    bool okFeature = false;
    const int featureSetIndex = parts.at(0).toInt(&okFeatureSet);
    const int featureIndex = parts.at(1).toInt(&okFeature);

    if (!okFeatureSet || !okFeature) {
        return qMakePair(-1, -1);
    }

    return qMakePair(featureSetIndex, featureIndex);
}

void CameraGUI::syncFromSelectedGs232Controller()
{
    const QPair<int, int> indices = selectedGs232ControllerIndices();

    if ((indices.first < 0) || (indices.second < 0)) {
        return;
    }

    double azimuth = 0.0;
    double elevation = 0.0;
    if (!ChannelWebAPIUtils::getFeatureSetting(indices.first, indices.second, "azimuth", azimuth)
        || !ChannelWebAPIUtils::getFeatureSetting(indices.first, indices.second, "elevation", elevation))
    {
        return;
    }

    settingsUI()->azimuthSpin->setValue(azimuth);
    settingsUI()->elevationSpin->setValue(elevation);
}

void CameraGUI::probeQtCameraCapabilities()
{
    if (!m_settings.isQtCamera()) {
        return;
    }

    reportResolutions();

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    if (cameras.isEmpty()) {
        return;
    }

    QCameraDevice selectedDevice = cameras.front();
    const QString targetId = m_settings.cameraIdString();
    const QString targetDescription = m_settings.cameraDescription();

    for (const QCameraDevice& device : cameras)
    {
        const QString id = QString::fromUtf8(device.id());
        if ((id == targetId) || (device.description() == targetDescription)) {
            selectedDevice = device;
            break;
        }
    }

    QCamera probeCamera(selectedDevice);
    const QCamera::Features cameraFeatures = probeCamera.supportedFeatures();
    m_qtManualExposureSupported = cameraFeatures.testFlag(QCamera::Feature::ManualExposureTime);
    m_qtIsoSensitivitySupported = cameraFeatures.testFlag(QCamera::Feature::IsoSensitivity);
    m_qtWhiteBalanceModeSupported = cameraFeatures.testFlag(QCamera::Feature::ColorTemperature);
    m_qtExposureCompensationSupported = cameraFeatures.testFlag(QCamera::Feature::ExposureCompensation);

    const float minZoom = probeCamera.minimumZoomFactor();
    const float maxZoom = probeCamera.maximumZoomFactor();
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

        if (probeCamera.isFocusModeSupported(static_cast<QCamera::FocusMode>(mode))) {
            supportedModes.append(mode);
        }
    }

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
#else
    const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    if (cameras.isEmpty()) {
        return;
    }

    QCameraInfo selectedInfo = cameras.front();
    const QString targetId = m_settings.cameraIdString();
    const QString targetDescription = m_settings.cameraDescription();
    for (const QCameraInfo& info : cameras)
    {
        const QString id = info.deviceName();
        if ((id == targetId) || (info.description() == targetDescription))
        {
            selectedInfo = info;
            break;
        }
    }

    QCamera probeCamera(selectedInfo);
    QCameraFocus *cameraFocus = probeCamera.focus();
    const qreal minZoom = 1.0;
    const qreal maxZoom = cameraFocus ? cameraFocus->maximumOpticalZoom() : 1.0;
    m_qtZoomSupported = (maxZoom > minZoom + 0.01);

    QCameraExposure *exp = probeCamera.exposure();
    m_qtManualExposureSupported = (exp != nullptr);
    m_qtIsoSensitivitySupported = exp && !exp->supportedIsoSensitivities().isEmpty();

    QCameraImageProcessing *ip = probeCamera.imageProcessing();
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
#endif
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
        connect(deviceSettingsGUI, &CameraObjectDeviceSettingsGUI::settingsChanged, this, &CameraGUI::applyActionSettings);
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
    applySettings({"yoloDisappearDebounce", "objectDeviceSettings"});
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
    if (m_settings.isFileCamera())
    {
        if (m_settings.m_videoFileCameraPath.isEmpty()) {
            return;
        }

        m_pendingQtVideoFrame = QVideoFrame();
        m_processingQtVideoFrame = false;
        m_mediaPlayerDurationMs = 0;
        {
            QSignalBlocker blocker(ui->playbackPositionSlider);
            ui->playbackPositionSlider->setValue(0);
        }
        updateVideoFileControls();

        m_mediaPlayer = new QMediaPlayer(this);
        m_videoSink = new QVideoSink(this);
        m_mediaPlayer->setVideoOutput(m_videoSink);
        connect(m_videoSink, &QVideoSink::videoFrameChanged, this, &CameraGUI::onQtVideoFrame);
        connect(m_mediaPlayer, &QMediaPlayer::positionChanged, this, &CameraGUI::handleMediaPlayerPositionChanged);
        connect(m_mediaPlayer, &QMediaPlayer::durationChanged, this, &CameraGUI::handleMediaPlayerDurationChanged);
        connect(m_mediaPlayer, &QMediaPlayer::playbackStateChanged, this, &CameraGUI::handleMediaPlayerPlaybackStateChanged);
        m_mediaPlayer->setSource(QUrl::fromLocalFile(m_settings.m_videoFileCameraPath));
        m_mediaPlayer->setPlaybackRate(ui->playbackRateSpin->value());
        m_mediaPlayer->play();

        m_qtZoomSupported = false;
        m_qtManualExposureSupported = false;
        m_qtIsoSensitivitySupported = false;
        m_qtWhiteBalanceModeSupported = false;
        m_qtExposureCompensationSupported = false;
        updateCameraSettingsVisibility();
        return;
    }

    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    if (cameras.isEmpty()) {
        return;
    }

    QCameraDevice selectedDevice = cameras.front();
    const QString targetId = m_settings.cameraIdString();
    const QString targetDescription = m_settings.cameraDescription();
    for (const QCameraDevice& device : cameras)
    {
        const QString id = QString::fromUtf8(device.id());
        if ((id == targetId) || (device.description() == targetDescription)) {
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
        m_pendingQtVideoFrame = QVideoFrame();
        m_processingQtVideoFrame = false;
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
    const QString targetId = m_settings.cameraIdString();
    const QString targetDescription = m_settings.cameraDescription();
    for (const QCameraInfo& info : cameras)
    {
        const QString id = info.deviceName();
        if ((id == targetId) || (info.description() == targetDescription))
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

        if (cameraFocus && maxZoom > minZoom)
        {
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
    m_pendingQtVideoFrame = QVideoFrame();
    m_processingQtVideoFrame = false;
    if (m_imageCapture)
    {
        if (m_captureSession) {
            m_captureSession->setImageCapture(nullptr);
        }
        delete m_imageCapture;
        m_imageCapture = nullptr;
    }
    if (m_mediaPlayer)
    {
        m_mediaPlayer->setVideoOutput(static_cast<QVideoSink *>(nullptr));
        m_mediaPlayer->stop();
        m_mediaPlayer->setSource(QUrl());
        delete m_mediaPlayer;
        m_mediaPlayer = nullptr;
    }
    if (m_videoSink)
    {
        disconnect(m_videoSink, nullptr, this, nullptr);
        delete m_videoSink;
        m_videoSink = nullptr;
    }
    if (m_captureSession)
    {
        m_captureSession->setCamera(nullptr);
        m_captureSession->setVideoOutput(static_cast<QVideoSink *>(nullptr));
        delete m_captureSession;
        m_captureSession = nullptr;
    }
    if (m_qtCamera)
    {
        m_qtCamera->stop();
        delete m_qtCamera;
        m_qtCamera = nullptr;
    }
    m_mediaPlayerDurationMs = 0;
    {
        QSignalBlocker blocker(ui->playbackPositionSlider);
        ui->playbackPositionSlider->setValue(0);
    }
    {
        QSignalBlocker blocker(ui->playPauseVideo);
        ui->playPauseVideo->setChecked(false);
    }
    updateVideoFileControls();
#else
    if (m_imageCapture)
    {
        delete m_imageCapture;
        m_imageCapture = nullptr;
    }
    if (m_videoSurface)
    {
        delete m_videoSurface;
        m_videoSurface = nullptr;
    }
    if (m_qtCamera)
    {
        m_qtCamera->stop();
        delete m_qtCamera;
        m_qtCamera = nullptr;
    }
#endif
}

void CameraGUI::applyQtCameraSettings(const QList<QString>& settingsKeys, bool force)
{
    if (!m_settings.isQtCamera() && !m_settings.isFileCamera())
    {
        // Camera type switched away from Qt — stop any running Qt camera
        if (m_qtCamera
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            || m_mediaPlayer
#endif
        ) {
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

    if (m_settings.isQtCamera() && (force || settingsKeys.contains("cameraId"))) {
        reportResolutions();
    }

    if (m_settings.isQtCamera() && (force || settingsKeys.contains("resolutionWidth") || settingsKeys.contains("resolutionHeight"))) {
        updateFrameRateControlForResolution(resolutionKey(m_settings.m_resolutionWidth, m_settings.m_resolutionHeight));
    }

    // Decide whether a full restart is needed
    updateCaptureModeControls();

    const bool recapture = force
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("videoFileCameraPath")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("captureMode")
        || settingsKeys.contains("captureInterval")
        || settingsKeys.contains("captureIntervalUnits")
        || settingsKeys.contains("framesPerSecond")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("isoSensitivity");

    const bool hasActiveVisualSource =
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        (m_qtCamera != nullptr) || (m_mediaPlayer != nullptr);
#else
        (m_qtCamera != nullptr);
#endif

    if (!hasActiveVisualSource && (m_camera->getState() == Feature::StRunning))
    {
        // Start the visual source (we've probably just switched to Qt or file camera type)
        setupQtCapture();
    }
    else if (recapture && hasActiveVisualSource)
    {
        // Restart the visual source so the new source or capture parameters take effect
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
        if (force || settingsKeys.contains("zoomFactor"))
        {
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
    if (!frame.isValid()) {
        return;
    }

    m_pendingQtVideoFrame = frame;

    if (!m_processingQtVideoFrame)
    {
        m_processingQtVideoFrame = true;
        QMetaObject::invokeMethod(this, &CameraGUI::processPendingQtVideoFrame, Qt::QueuedConnection);
    }
}

void CameraGUI::processPendingQtVideoFrame()
{
    QVideoFrame frame = m_pendingQtVideoFrame;
    m_pendingQtVideoFrame = QVideoFrame();

    if (!frame.isValid())
    {
        m_processingQtVideoFrame = false;
        return;
    }

    const QImage image = frame.toImage();
    onQtImageCaptured(-1, image);

    if (m_pendingQtVideoFrame.isValid()) {
        QMetaObject::invokeMethod(this, &CameraGUI::processPendingQtVideoFrame, Qt::QueuedConnection);
    } else {
        m_processingQtVideoFrame = false;
    }
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

    CameraFrameAligner *frameAligner = m_camera->getFrameAligner();
    if (frameAligner) {
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = image;
        frame->m_captureDateTime = QDateTime::currentDateTime();
        frameAligner->submitFrame(frame);
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

void CameraGUI::updateCameraSettingsVisibility()
{
    const bool alpaca = m_settings.isAlpacaCamera();
    const bool asi = m_settings.isAsiCamera();
    const bool fileCamera = m_settings.isFileCamera();
    const bool qtCamera = m_settings.isQtCamera();
    const bool sharedHardwareCamera = alpaca || asi;

    settingsUI()->resolutionLabel->setVisible(qtCamera);
    settingsUI()->resolutionCombo->setVisible(qtCamera);
    settingsUI()->fpsLabel->setVisible(!fileCamera);
    settingsUI()->fpsLabel->setEnabled(!alpaca && !fileCamera);
    updateCaptureModeControls();
    settingsUI()->captureValueStack->setVisible(!fileCamera);
    if (alpaca || fileCamera || !m_settings.isIntervalCaptureMode()) {
        settingsUI()->fpsStack->setCurrentWidget(settingsUI()->fpsSpinPage);
    }
    settingsUI()->isoLabel->setVisible(qtCamera);
    settingsUI()->isoSpin->setVisible(qtCamera);
    settingsUI()->cameraBinXLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraBinXSpin->setVisible(sharedHardwareCamera);
    settingsUI()->cameraBinYLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraBinYSpin->setVisible(sharedHardwareCamera);
    settingsUI()->cameraNumXLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraNumXSpin->setVisible(sharedHardwareCamera);
    settingsUI()->cameraNumYLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraNumYSpin->setVisible(sharedHardwareCamera);
    settingsUI()->cameraStartXLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraStartXSpin->setVisible(sharedHardwareCamera);
    settingsUI()->cameraStartYLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraStartYSpin->setVisible(sharedHardwareCamera);
    settingsUI()->cameraGainLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraGainCombo->setVisible(alpaca && m_alpacaHasNamedGains);
    settingsUI()->cameraGainSlider->setVisible(sharedHardwareCamera && (!alpaca || !m_alpacaHasNamedGains));
    settingsUI()->cameraGainSpin->setVisible(sharedHardwareCamera && (!alpaca || !m_alpacaHasNamedGains));
    settingsUI()->cameraOffsetLabel->setVisible(sharedHardwareCamera);
    settingsUI()->cameraOffsetCombo->setVisible(alpaca && m_alpacaHasNamedOffsets);
    settingsUI()->cameraOffsetSlider->setVisible(sharedHardwareCamera && (!alpaca || !m_alpacaHasNamedOffsets));
    settingsUI()->cameraOffsetSpin->setVisible(sharedHardwareCamera && (!alpaca || !m_alpacaHasNamedOffsets));
    settingsUI()->alpacaReadoutModeLabel->setVisible(alpaca);
    settingsUI()->alpacaReadoutModeCombo->setVisible(alpaca);
    settingsUI()->asiCoolerOnLabel->setVisible(asi && m_asiCoolerSupported);
    settingsUI()->asiCoolerOnCheck->setVisible(asi && m_asiCoolerSupported);
    settingsUI()->asiTargetTempLabel->setVisible(asi && m_asiTargetTempSupported);
    settingsUI()->asiTargetTempSpin->setVisible(asi && m_asiTargetTempSupported);
    settingsUI()->asiUsbBandwidthLabel->setVisible(asi && m_asiUsbBandwidthSupported);
    settingsUI()->asiUsbBandwidthSpin->setVisible(asi && m_asiUsbBandwidthSupported);
    settingsUI()->asiHighSpeedModeLabel->setVisible(asi && m_asiHighSpeedModeSupported);
    settingsUI()->asiHighSpeedModeCheck->setVisible(asi && m_asiHighSpeedModeSupported);
    settingsUI()->asiAutoExposureGainLabel->setVisible(asi);
    settingsUI()->asiAutoExposureGainCheck->setVisible(asi);
    settingsUI()->asiColorImageTypeLabel->setVisible(asi && m_asiColorCameraActive && (m_asiRgb24Supported || m_asiRaw16Supported));
    settingsUI()->asiColorImageTypeCombo->setVisible(asi && m_asiColorCameraActive && (m_asiRgb24Supported || m_asiRaw16Supported));
    settingsUI()->alpacaFocusPositionLabel->setVisible(alpaca);
    settingsUI()->alpacaFocusPositionSpin->setVisible(alpaca);
    settingsUI()->alpacaFocusStepSizeLabel->setVisible(alpaca);
    settingsUI()->alpacaFocusStepSizeSpin->setVisible(alpaca);
    settingsUI()->alpacaFilterWheelPositionLabel->setVisible(alpaca);
    settingsUI()->alpacaFilterWheelPositionCombo->setVisible(alpaca);

    bool focuserAvailable = alpaca && (settingsUI()->alpacaFocuserCombo->count() > 0);
    settingsUI()->alpacaFocuserEnabledCheck->setEnabled(focuserAvailable);
    settingsUI()->alpacaFocuserCombo->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    const bool asiAutoExposureGainEnabled = asi && (m_settings.m_captureMode == CameraSettings::CaptureModeFrameRate);
    const bool asiManualExposureGainEnabled = !(asi && m_settings.m_asiAutoExposureGain && asiAutoExposureGainEnabled);
    settingsUI()->asiAutoExposureGainLabel->setEnabled(asiAutoExposureGainEnabled);
    settingsUI()->asiAutoExposureGainCheck->setEnabled(asiAutoExposureGainEnabled);
    settingsUI()->alpacaFocusPositionLabel->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaFocusPositionSpin->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaFocusStepSizeLabel->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaFocusStepSizeSpin->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);

    bool filterWheelAvailable = alpaca && (settingsUI()->alpacaFilterWheelCombo->count() > 0);
    settingsUI()->alpacaFilterWheelEnabledCheck->setEnabled(filterWheelAvailable);
    settingsUI()->alpacaFilterWheelCombo->setEnabled(m_settings.m_alpacaFilterWheelEnabled && filterWheelAvailable);
    settingsUI()->alpacaFilterWheelPositionLabel->setEnabled(m_settings.m_alpacaFilterWheelEnabled && filterWheelAvailable);
    settingsUI()->alpacaFilterWheelPositionCombo->setEnabled(m_settings.m_alpacaFilterWheelEnabled && filterWheelAvailable);

    settingsUI()->tabWidget->setTabEnabled(0, !fileCamera);
    settingsUI()->tabWidget->setTabEnabled(1, sharedHardwareCamera);
    ui->audioMute->setVisible(qtCamera);

    // Qt-camera-only controls
    settingsUI()->exposureLabel->setVisible(!fileCamera);
    settingsUI()->exposureSlider->setVisible(!fileCamera);
    settingsUI()->exposureSpin->setVisible(!fileCamera);
    settingsUI()->exposureUnitsCombo->setVisible(!fileCamera);
    settingsUI()->whiteBalanceLabel->setVisible(qtCamera);
    settingsUI()->whiteBalanceCombo->setVisible(qtCamera);
    settingsUI()->zoomLabel->setVisible(qtCamera);
    settingsUI()->zoomSpin->setVisible(qtCamera);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    settingsUI()->exposureCompLabel->setVisible(qtCamera);
    settingsUI()->exposureCompSpin->setVisible(qtCamera);
    settingsUI()->focusModeLabel->setVisible(qtCamera);
    settingsUI()->focusModeCombo->setVisible(qtCamera);
    settingsUI()->focusDistLabel->setVisible(qtCamera);
    settingsUI()->focusDistSpin->setVisible(qtCamera);
#else
    settingsUI()->exposureCompLabel->setVisible(false);
    settingsUI()->exposureCompSpin->setVisible(false);
    settingsUI()->focusModeLabel->setVisible(false);
    settingsUI()->focusModeCombo->setVisible(false);
    settingsUI()->focusDistLabel->setVisible(false);
    settingsUI()->focusDistSpin->setVisible(false);
#endif

    if (alpaca || asi)
    {
        settingsUI()->exposureLabel->setEnabled(asiManualExposureGainEnabled);
        settingsUI()->exposureSlider->setEnabled(asiManualExposureGainEnabled);
        settingsUI()->exposureSpin->setEnabled(asiManualExposureGainEnabled);
        settingsUI()->exposureUnitsCombo->setEnabled(asiManualExposureGainEnabled);
        settingsUI()->cameraGainLabel->setEnabled(asiManualExposureGainEnabled);
        settingsUI()->cameraGainCombo->setEnabled((alpaca && m_alpacaHasNamedGains) ? true : asiManualExposureGainEnabled);
        settingsUI()->cameraGainSlider->setEnabled(asiManualExposureGainEnabled);
        settingsUI()->cameraGainSpin->setEnabled(asiManualExposureGainEnabled);
    }
    else if (fileCamera)
    {
        // No extra enabled-state updates needed here: the Qt-only controls above are hidden.
    }
    else
    {
        settingsUI()->zoomLabel->setEnabled(m_qtZoomSupported);
        settingsUI()->zoomSpin->setEnabled(m_qtZoomSupported);
        settingsUI()->exposureLabel->setEnabled(m_qtManualExposureSupported);
        settingsUI()->exposureSlider->setEnabled(m_qtManualExposureSupported);
        settingsUI()->exposureSpin->setEnabled(m_qtManualExposureSupported);
        settingsUI()->exposureUnitsCombo->setEnabled(m_qtManualExposureSupported);
        settingsUI()->isoLabel->setEnabled(m_qtIsoSensitivitySupported);
        settingsUI()->isoSpin->setEnabled(m_qtIsoSensitivitySupported);
        settingsUI()->whiteBalanceLabel->setEnabled(m_qtWhiteBalanceModeSupported);
        settingsUI()->whiteBalanceCombo->setEnabled(m_qtWhiteBalanceModeSupported);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        settingsUI()->exposureCompLabel->setEnabled(m_qtExposureCompensationSupported);
        settingsUI()->exposureCompSpin->setEnabled(m_qtExposureCompensationSupported);
        const bool manualFocus = (m_settings.m_focusMode == static_cast<int>(QCamera::FocusModeManual));
        settingsUI()->focusDistLabel->setEnabled(manualFocus);
        settingsUI()->focusDistSpin->setEnabled(manualFocus);
#endif
    }

    updateVideoFileControls();
    updateCameraStatusDisplay();
}

void CameraGUI::updateCameraStatusDisplay()
{
    if (!m_settingsDialog) {
        return;
    }

    settingsUI()->pipelineFpsLabel->setText(
        m_lastPipelineFps > 0.0 ? QString::number(m_lastPipelineFps, 'f', 1) : "-");

    if (!m_settings.isAlpacaCamera() && !m_settings.isAsiCamera()) {
        return;
    }

    QString cameraStateText;
    if (m_settings.isAsiCamera())
    {
        static const QStringList cameraStateNames = {
            "Idle", "Capturing"
        };

        cameraStateText = (m_lastAlpacaCameraState >= 0 && m_lastAlpacaCameraState < cameraStateNames.size())
            ? cameraStateNames[m_lastAlpacaCameraState]
            : (m_lastAlpacaCameraState >= 0 ? QString::number(m_lastAlpacaCameraState) : "-");
    }
    else
    {
        static const QStringList cameraStateNames = {
            "Idle", "Waiting", "Exposing", "Reading", "Download", "Error"
        };

        cameraStateText = (m_lastAlpacaCameraState >= 0 && m_lastAlpacaCameraState < cameraStateNames.size())
            ? cameraStateNames[m_lastAlpacaCameraState]
            : (m_lastAlpacaCameraState >= 0 ? QString::number(m_lastAlpacaCameraState) : "-");
    }

    settingsUI()->cameraStateLabel->setText(cameraStateText);
    settingsUI()->captureTimeLabel->setText(
        m_lastAlpacaCaptureTimeMs >= 0 ? QString::number(m_lastAlpacaCaptureTimeMs) : "-");
    settingsUI()->receiveImageFormatLabel->setText(
        m_lastAlpacaReceiveImageFormat.isEmpty() ? "-" : m_lastAlpacaReceiveImageFormat);
    settingsUI()->ccdTempLabel->setText(
        m_lastAlpacaCcdTemperatureValid ? QString::number(m_lastAlpacaCcdTemperature, 'f', 1) : "-");
    settingsUI()->alpacaErrorCodeLabel->setText(QString::number(m_lastAlpacaErrorNumber));
    settingsUI()->alpacaErrorMessageLabel->setText(
        m_lastAlpacaErrorMessage.isEmpty() ? "-" : m_lastAlpacaErrorMessage);
}

void CameraGUI::handleMediaPlayerPositionChanged(qint64 position)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_mediaPlayerDurationMs <= 0 || ui->playbackPositionSlider->isSliderDown()) {
        return;
    }

    const int sliderValue = static_cast<int>(
        qBound<qint64>(0,
            (position * PlaybackPositionSliderMaximum) / m_mediaPlayerDurationMs,
            static_cast<qint64>(PlaybackPositionSliderMaximum)));
    QSignalBlocker blocker(ui->playbackPositionSlider);
    ui->playbackPositionSlider->setValue(sliderValue);
#else
    Q_UNUSED(position)
#endif
}

void CameraGUI::handleMediaPlayerDurationChanged(qint64 duration)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    m_mediaPlayerDurationMs = qMax<qint64>(0, duration);
    if (m_mediaPlayerDurationMs <= 0)
    {
        QSignalBlocker blocker(ui->playbackPositionSlider);
        ui->playbackPositionSlider->setValue(0);
    }
    updateVideoFileControls();
#else
    Q_UNUSED(duration)
#endif
}

void CameraGUI::handleMediaPlayerPlaybackStateChanged(QMediaPlayer::PlaybackState state)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QSignalBlocker blocker(ui->playPauseVideo);
    ui->playPauseVideo->setChecked(state == QMediaPlayer::PlayingState);
#else
    Q_UNUSED(state)
#endif
}


void CameraGUI::updateAlpacaCapabilities(const CameraWorker::MsgReportAlpacaCameraInfo& info)
{
    blockApplySettings(true);

    const double exposureMinMs = std::max(0.001, info.getExposureMinMs());
    const double exposureMaxMs = std::max(exposureMinMs, info.getExposureMaxMs());
    const double exposureResolutionMs = std::max(0.000001, info.getExposureResolutionMs());

    // Bin X
    settingsUI()->cameraBinXSpin->setMaximum(std::max(1, info.getMaxBinX()));
    settingsUI()->cameraBinXSpin->setValue(qBound(1, m_settings.m_cameraBinX, info.getMaxBinX()));

    // Bin Y
    settingsUI()->cameraBinYSpin->setMaximum(std::max(1, info.getMaxBinY()));
    settingsUI()->cameraBinYSpin->setValue(qBound(1, m_settings.m_cameraBinY, info.getMaxBinY()));
    m_alpacaCameraSizeX = std::max(0, info.getCameraSizeX());
    m_alpacaCameraSizeY = std::max(0, info.getCameraSizeY());
    updateCameraSubframeControls();

    // Gain
    m_alpacaHasNamedGains = !info.getGains().isEmpty();
    if (m_alpacaHasNamedGains)
    {
        settingsUI()->cameraGainCombo->blockSignals(true);
        settingsUI()->cameraGainCombo->clear();
        settingsUI()->cameraGainCombo->addItems(info.getGains());
        const int gainIdx = (m_settings.m_cameraGain >= 0 && m_settings.m_cameraGain < info.getGains().size())
            ? m_settings.m_cameraGain : 0;
        settingsUI()->cameraGainCombo->setCurrentIndex(gainIdx);
        settingsUI()->cameraGainCombo->blockSignals(false);
    }
    else
    {
        settingsUI()->cameraGainSpin->setMinimum(info.getGainMin());
        settingsUI()->cameraGainSlider->setMinimum(info.getGainMin());
        settingsUI()->cameraGainSpin->setMaximum(std::max(info.getGainMin(), info.getGainMax()));
        settingsUI()->cameraGainSlider->setMaximum(std::max(info.getGainMin(), info.getGainMax()));
        const int gainVal = (m_settings.m_cameraGain >= 0) ? m_settings.m_cameraGain : info.getGainMin();
        settingsUI()->cameraGainSpin->setValue(qBound(info.getGainMin(), gainVal, info.getGainMax()));
        settingsUI()->cameraGainSlider->setValue(qBound(info.getGainMin(), gainVal, info.getGainMax()));
    }

    // Readout mode
    settingsUI()->alpacaReadoutModeCombo->blockSignals(true);
    settingsUI()->alpacaReadoutModeCombo->clear();
    settingsUI()->alpacaReadoutModeCombo->addItems(info.getReadoutModes());
    if (m_settings.m_cameraReadoutMode < info.getReadoutModes().size()) {
        settingsUI()->alpacaReadoutModeCombo->setCurrentIndex(m_settings.m_cameraReadoutMode);
    }
    settingsUI()->alpacaReadoutModeCombo->blockSignals(false);

    // Offset
    m_alpacaHasNamedOffsets = !info.getOffsets().isEmpty();
    if (m_alpacaHasNamedOffsets)
    {
        settingsUI()->cameraOffsetCombo->blockSignals(true);
        settingsUI()->cameraOffsetCombo->clear();
        settingsUI()->cameraOffsetCombo->addItems(info.getOffsets());
        const int offsetIdx = (m_settings.m_cameraOffset >= 0 && m_settings.m_cameraOffset < info.getOffsets().size())
            ? m_settings.m_cameraOffset : 0;
        settingsUI()->cameraOffsetCombo->setCurrentIndex(offsetIdx);
        settingsUI()->cameraOffsetCombo->blockSignals(false);
    }
    else
    {
        settingsUI()->cameraOffsetSpin->setMinimum(info.getOffsetMin());
        settingsUI()->cameraOffsetSlider->setMinimum(info.getOffsetMin());
        settingsUI()->cameraOffsetSpin->setMaximum(std::max(info.getOffsetMin(), info.getOffsetMax()));
        settingsUI()->cameraOffsetSlider->setMaximum(std::max(info.getOffsetMin(), info.getOffsetMax()));
        const int offsetVal = (m_settings.m_cameraOffset >= 0) ? m_settings.m_cameraOffset : info.getOffsetMin();
        settingsUI()->cameraOffsetSpin->setValue(qBound(info.getOffsetMin(), offsetVal, info.getOffsetMax()));
        settingsUI()->cameraOffsetSlider->setValue(qBound(info.getOffsetMin(), offsetVal, info.getOffsetMax()));
    }

    m_exposureMinimumMs = exposureMinMs;
    m_exposureMaximumMs = exposureMaxMs;
    m_exposureStepMs = exposureResolutionMs;
    m_settings.m_exposureTimeMs = qBound(exposureMinMs, m_settings.m_exposureTimeMs, exposureMaxMs);
    updateExposureControls();

    // Status labels
    settingsUI()->cameraNameLabel->setText(info.getName().isEmpty() ? "-" : info.getName());
    settingsUI()->cameraDescriptionLabel->setText(info.getDescription().isEmpty() ? "-" : info.getDescription());
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

    updateCameraSettingsVisibility();
    blockApplySettings(false);
}

void CameraGUI::updateAsiCapabilities(const CameraWorker::MsgReportAsiCameraInfo& info)
{
    blockApplySettings(true);

    m_alpacaHasNamedGains = false;
    m_alpacaHasNamedOffsets = false;
    m_asiCoolerSupported = info.isCoolerSupported();
    m_asiTargetTempSupported = info.isTargetTempSupported();
    m_asiUsbBandwidthSupported = info.isUsbBandwidthSupported();
    m_asiHighSpeedModeSupported = info.isHighSpeedModeSupported();
    m_asiColorCameraActive = info.isColor();
    m_asiRgb24Supported = info.isRgb24Supported();
    m_asiRaw16Supported = info.isRaw16Supported();
    m_alpacaCameraSizeX = std::max(0, info.getCameraSizeX());
    m_alpacaCameraSizeY = std::max(0, info.getCameraSizeY());

    settingsUI()->cameraBinXSpin->setMaximum(std::max(1, info.getMaxBinX()));
    settingsUI()->cameraBinXSpin->setValue(qBound(1, m_settings.m_cameraBinX, info.getMaxBinX()));
    settingsUI()->cameraBinYSpin->setMaximum(std::max(1, info.getMaxBinY()));
    settingsUI()->cameraBinYSpin->setValue(qBound(1, m_settings.m_cameraBinY, info.getMaxBinY()));
    updateCameraSubframeControls();

    settingsUI()->cameraGainSpin->setMinimum(info.getGainMin());
    settingsUI()->cameraGainSlider->setMinimum(info.getGainMin());
    settingsUI()->cameraGainSpin->setMaximum(std::max(info.getGainMin(), info.getGainMax()));
    settingsUI()->cameraGainSlider->setMaximum(std::max(info.getGainMin(), info.getGainMax()));
    settingsUI()->cameraGainSpin->setValue(qBound(info.getGainMin(), std::max(info.getGainMin(), m_settings.m_cameraGain), info.getGainMax()));
    settingsUI()->cameraGainSlider->setValue(settingsUI()->cameraGainSpin->value());

    settingsUI()->cameraOffsetSpin->setMinimum(info.getOffsetMin());
    settingsUI()->cameraOffsetSlider->setMinimum(info.getOffsetMin());
    settingsUI()->cameraOffsetSpin->setMaximum(std::max(info.getOffsetMin(), info.getOffsetMax()));
    settingsUI()->cameraOffsetSlider->setMaximum(std::max(info.getOffsetMin(), info.getOffsetMax()));
    settingsUI()->cameraOffsetSpin->setValue(qBound(info.getOffsetMin(), std::max(info.getOffsetMin(), m_settings.m_cameraOffset), info.getOffsetMax()));
    settingsUI()->cameraOffsetSlider->setValue(settingsUI()->cameraOffsetSpin->value());

    m_exposureMinimumMs = std::max(0.001, info.getExposureMinMs());
    m_exposureMaximumMs = std::max(m_exposureMinimumMs, info.getExposureMaxMs());
    m_exposureStepMs = m_exposureMinimumMs;
    m_settings.m_exposureTimeMs = qBound(m_exposureMinimumMs, m_settings.m_exposureTimeMs, m_exposureMaximumMs);
    updateExposureControls();

    if (info.isCoolerSupported() && (m_settings.m_asiCoolerOn < 0)) {
        m_settings.m_asiCoolerOn = info.isCoolerOn() ? 1 : 0;
    }
    settingsUI()->asiCoolerOnCheck->setChecked(m_settings.m_asiCoolerOn > 0);

    settingsUI()->asiTargetTempSpin->setMinimum(info.getTargetTempMin());
    settingsUI()->asiTargetTempSpin->setMaximum(std::max(info.getTargetTempMin(), info.getTargetTempMax()));
    if (info.isTargetTempSupported() && (m_settings.m_asiTargetTemp == std::numeric_limits<int>::min())) {
        m_settings.m_asiTargetTemp = info.getTargetTemp();
    }
    settingsUI()->asiTargetTempSpin->setValue(qBound(
        info.getTargetTempMin(),
        m_settings.m_asiTargetTemp == std::numeric_limits<int>::min() ? info.getTargetTemp() : m_settings.m_asiTargetTemp,
        std::max(info.getTargetTempMin(), info.getTargetTempMax())));

    settingsUI()->asiUsbBandwidthSpin->setMinimum(info.getUsbBandwidthMin());
    settingsUI()->asiUsbBandwidthSpin->setMaximum(std::max(info.getUsbBandwidthMin(), info.getUsbBandwidthMax()));
    if (info.isUsbBandwidthSupported() && (m_settings.m_asiUsbBandwidth < 0)) {
        m_settings.m_asiUsbBandwidth = info.getUsbBandwidth();
    }
    settingsUI()->asiUsbBandwidthSpin->setValue(qBound(
        info.getUsbBandwidthMin(),
        m_settings.m_asiUsbBandwidth < 0 ? info.getUsbBandwidth() : m_settings.m_asiUsbBandwidth,
        std::max(info.getUsbBandwidthMin(), info.getUsbBandwidthMax())));

    if (info.isHighSpeedModeSupported() && (m_settings.m_asiHighSpeedMode < 0)) {
        m_settings.m_asiHighSpeedMode = info.isHighSpeedMode() ? 1 : 0;
    }
    settingsUI()->asiHighSpeedModeCheck->setChecked(m_settings.m_asiHighSpeedMode > 0);

    {
        QSignalBlocker blocker(settingsUI()->asiColorImageTypeCombo);
        settingsUI()->asiColorImageTypeCombo->clear();
        if (info.isRgb24Supported()) {
            settingsUI()->asiColorImageTypeCombo->addItem(QStringLiteral("RGB24"), CameraSettings::AsiColorImageTypeRgb24);
        }
        if (info.isRaw16Supported()) {
            settingsUI()->asiColorImageTypeCombo->addItem(QStringLiteral("RAW16"), CameraSettings::AsiColorImageTypeRaw16);
        }

        int comboIndex = settingsUI()->asiColorImageTypeCombo->findData(m_settings.m_asiColorImageType);
        if ((comboIndex < 0) && (settingsUI()->asiColorImageTypeCombo->count() > 0))
        {
            comboIndex = 0;
            m_settings.m_asiColorImageType = static_cast<CameraSettings::AsiColorImageType>(
                settingsUI()->asiColorImageTypeCombo->itemData(0).toInt());
        }
        settingsUI()->asiColorImageTypeCombo->setCurrentIndex(comboIndex);
    }

    settingsUI()->cameraNameLabel->setText(info.getName().isEmpty() ? "-" : info.getName());
    settingsUI()->cameraDescriptionLabel->setText(QStringLiteral("ASI Camera"));
    settingsUI()->sensorNameLabel->setText(info.getName().isEmpty() ? "-" : info.getName());
    settingsUI()->sensorTypeLabel->setText(info.isColor() ? QStringLiteral("Colour") : QStringLiteral("Monochrome"));

    if (info.getPixelSizeUm() > 0.0) {
        settingsUI()->pixelSizeLabel->setText(QString("%1 × %1").arg(info.getPixelSizeUm(), 0, 'f', 2));
    } else {
        settingsUI()->pixelSizeLabel->setText("-");
    }

    if (info.getCameraSizeX() > 0 || info.getCameraSizeY() > 0) {
        settingsUI()->cameraSizeLabel->setText(QString("%1 × %2").arg(info.getCameraSizeX()).arg(info.getCameraSizeY()));
    } else {
        settingsUI()->cameraSizeLabel->setText("-");
    }

    updateCameraSettingsVisibility();
    blockApplySettings(false);
}

void CameraGUI::updateCameraSubframeControls()
{
    const int maxSubframeX = std::max(1, m_alpacaCameraSizeX / std::max(1, m_settings.m_cameraBinX));
    const int maxSubframeY = std::max(1, m_alpacaCameraSizeY / std::max(1, m_settings.m_cameraBinY));
    const int startX = qBound(0, m_settings.m_cameraStartX, maxSubframeX - 1);
    const int startY = qBound(0, m_settings.m_cameraStartY, maxSubframeY - 1);
    const int maxNumX = std::max(1, maxSubframeX - startX);
    const int maxNumY = std::max(1, maxSubframeY - startY);
    const int numX = (m_settings.m_cameraNumX == 0) ? 0 : qBound(1, m_settings.m_cameraNumX, maxNumX);
    const int numY = (m_settings.m_cameraNumY == 0) ? 0 : qBound(1, m_settings.m_cameraNumY, maxNumY);

    m_settings.m_cameraStartX = startX;
    m_settings.m_cameraStartY = startY;
    m_settings.m_cameraNumX = numX;
    m_settings.m_cameraNumY = numY;

    settingsUI()->cameraNumXSpin->setMinimum(0);
    settingsUI()->cameraNumYSpin->setMinimum(0);
    settingsUI()->cameraStartXSpin->setMaximum(maxSubframeX - 1);
    settingsUI()->cameraStartYSpin->setMaximum(maxSubframeY - 1);
    settingsUI()->cameraStartXSpin->setValue(startX);
    settingsUI()->cameraStartYSpin->setValue(startY);
    settingsUI()->cameraNumXSpin->setMaximum(maxNumX);
    settingsUI()->cameraNumYSpin->setMaximum(maxNumY);
    settingsUI()->cameraNumXSpin->setValue(numX);
    settingsUI()->cameraNumYSpin->setValue(numY);
}

void CameraGUI::on_startStop_clicked(bool checked)
{
    if (m_settings.m_scheduleEnabled)
    {
        if (checked)
        {
            if (!isWithinScheduleWindow()) {
                m_scheduleManualStartLatch = true;
            }
            m_scheduleManualStopLatch = false;
        }
        else
        {
            m_scheduleManualStartLatch = false;

            if (isWithinScheduleWindow()) {
                m_scheduleManualStopLatch = true;
            }
        }
    }

    m_camera->getInputMessageQueue()->push(Camera::MsgStartStop::create(checked));
}

void CameraGUI::on_refreshCamerasButton_clicked()
{
    m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create());
}

void CameraGUI::on_browseVideoFileButton_clicked()
{
    if (!m_settings.isFileCamera()) {
        return;
    }

    const int index = ui->cameraCombo->currentIndex();
    if (index < 0) {
        return;
    }

    if (!chooseVideoFileCameraFile(
            index,
            m_settings.m_cameraProtocol,
            m_settings.m_cameraId,
            m_settings.m_alpacaHost,
            m_settings.m_alpacaPort))
    {
        return;
    }

    m_settings.m_cameraId = ui->cameraCombo->itemData(index, CameraIdRole).toString();
    m_settings.m_cameraDescription = ui->cameraCombo->itemData(index, CameraDescriptionRole).toString();
    m_settings.m_videoFileCameraPath = m_settings.m_cameraId;
    updateVideoFileControls();
    applySettings({"cameraId", "cameraDescription", "videoFileCameraPath"});
}

void CameraGUI::on_restartVideo_clicked()
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (!m_settings.isFileCamera() || m_settings.m_videoFileCameraPath.isEmpty()) {
        return;
    }

    if (!m_mediaPlayer && (m_camera->getState() == Feature::StRunning)) {
        setupQtCapture();
    }

    if (m_mediaPlayer)
    {
        m_mediaPlayer->setPosition(0);
        m_mediaPlayer->play();
    }
#endif
}

void CameraGUI::on_playPauseVideo_clicked(bool checked)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (!m_settings.isFileCamera() || m_settings.m_videoFileCameraPath.isEmpty()) {
        return;
    }

    if (!m_mediaPlayer && checked && (m_camera->getState() == Feature::StRunning)) {
        setupQtCapture();
    }

    if (!m_mediaPlayer) {
        QSignalBlocker blocker(ui->playPauseVideo);
        ui->playPauseVideo->setChecked(false);
        return;
    }

    if (checked) {
        m_mediaPlayer->play();
    } else {
        m_mediaPlayer->pause();
    }
#else
    Q_UNUSED(checked)
#endif
}

void CameraGUI::on_loopVideo_clicked(bool checked)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_mediaPlayer)
    {
        // On Windows, setLoops doesn't appear to apply if already playing
        bool wasPlaying = m_mediaPlayer->isPlaying();
        qint64 position;

        if (wasPlaying)
        {
            position = m_mediaPlayer->position();
            m_mediaPlayer->stop();
        }
        m_mediaPlayer->setLoops(checked ? QMediaPlayer::Infinite : 1);
        if (wasPlaying)
        {
            m_mediaPlayer->setPosition(position);
            m_mediaPlayer->play();
        }
    }
#endif
}

void CameraGUI::on_playbackRateSpin_valueChanged(double value)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_mediaPlayer) {
        m_mediaPlayer->setPlaybackRate(value);
    }
#else
    Q_UNUSED(value)
#endif
}

void CameraGUI::on_playbackPositionSlider_sliderMoved(int value)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (!m_mediaPlayer || (m_mediaPlayerDurationMs <= 0)) {
        return;
    }

    const qint64 position = (static_cast<qint64>(value) * m_mediaPlayerDurationMs) / PlaybackPositionSliderMaximum;
    m_mediaPlayer->setPosition(position);
#else
    Q_UNUSED(value)
#endif
}

void CameraGUI::on_playbackPositionSlider_sliderReleased()
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    on_playbackPositionSlider_sliderMoved(ui->playbackPositionSlider->value());
#endif
}

void CameraGUI::on_cameraCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    const CameraInfo previousCamera = selectedCameraFromSettings();
    CameraInfo selectedCamera = comboCameraInfo(index);
    const bool wasAlpaca = previousCamera.m_protocol == QLatin1String("alpaca");
    const bool wasAsi = previousCamera.m_protocol == QLatin1String("asi");

    if ((selectedCamera.m_protocol == QLatin1String("file")) && selectedCamera.m_id.isEmpty())
    {
        if (!chooseVideoFileCameraFile(index,
                previousCamera.m_protocol,
                previousCamera.m_id,
                previousCamera.m_host,
                previousCamera.m_port))
        {
            return;
        }

        selectedCamera = comboCameraInfo(index);
    }

    setSelectedCamera(selectedCamera.m_protocol, selectedCamera.m_id, selectedCamera.m_description,
        selectedCamera.m_host, selectedCamera.m_port);

    const bool switchedBetweenAsiAndAlpaca =
        (wasAsi && m_settings.isAlpacaCamera()) || (wasAlpaca && m_settings.isAsiCamera());

    if (switchedBetweenAsiAndAlpaca)
    {
        m_settings.m_cameraStartX = 0;
        m_settings.m_cameraStartY = 0;
        m_settings.m_cameraNumX = 0;
        m_settings.m_cameraNumY = 0;
        m_alpacaCameraSizeX = 0;
        m_alpacaCameraSizeY = 0;
    }

    if (!isSameHardwareCameraBackend(previousCamera, selectedCameraFromSettings())) {
        resetCameraStatus();
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
    else if (m_settings.isQtCamera() && !ui->startStop->isChecked())
    {
        probeQtCameraCapabilities();
    }
    QStringList settingsKeys = cameraSelectionSettingsKeys(selectedCamera);
    if (switchedBetweenAsiAndAlpaca) {
        settingsKeys.append("cameraStartX");
        settingsKeys.append("cameraStartY");
        settingsKeys.append("cameraNumX");
        settingsKeys.append("cameraNumY");
    }
    updateCameraSettingsVisibility();
    if (switchedBetweenAsiAndAlpaca) {
        updateCameraSubframeControls();
    }
    applySettings(settingsKeys);
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
            applySettings({"resolutionWidth", "resolutionHeight", "framesPerSecond"});
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
    updateCameraSettingsVisibility();
    applySetting("captureMode");
}

void CameraGUI::on_fpsSpin_valueChanged(int value)
{
    m_settings.m_framesPerSecond = value;
    applySetting("framesPerSecond");
}

void CameraGUI::on_fpsCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_framesPerSecond = settingsUI()->fpsCombo->itemData(index).toInt();
    applySetting("framesPerSecond");
}

void CameraGUI::on_intervalSpin_valueChanged(double value)
{
    m_settings.m_captureInterval = value;
    applySetting("captureInterval");
}

void CameraGUI::on_intervalUnitsCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_captureIntervalUnits = static_cast<CameraSettings::CaptureIntervalUnits>(settingsUI()->intervalUnitsCombo->itemData(index).toInt());
    applySetting("captureIntervalUnits");
}

void CameraGUI::on_exposureSlider_valueChanged(int value)
{
    const double exposureValue = sliderToExposureValue(settingsUI()->exposureSpin, value);
    const double exposureMs = exposureValue * currentExposureUnitScaleMs(settingsUI());
    settingsUI()->exposureSpin->blockSignals(true);
    settingsUI()->exposureSpin->setValue(exposureValue);
    settingsUI()->exposureSpin->blockSignals(false);
    m_settings.m_exposureTimeMs = exposureMs;
    applySetting("exposureTimeMs");
}

void CameraGUI::on_exposureSpin_valueChanged(double value)
{
    settingsUI()->exposureSlider->blockSignals(true);
    settingsUI()->exposureSlider->setValue(exposureValueToSlider(settingsUI()->exposureSpin, value));
    settingsUI()->exposureSlider->blockSignals(false);
    m_settings.m_exposureTimeMs = value * currentExposureUnitScaleMs(settingsUI());
    applySetting("exposureTimeMs");
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
    applySetting("isoSensitivity");
}

void CameraGUI::on_alpacaDiscoveryCheck_toggled(bool checked)
{
    m_settings.m_alpacaDiscoveryEnabled = checked;
    applySetting("alpacaDiscoveryEnabled");
}

void CameraGUI::on_alpacaApiLogCheck_toggled(bool checked)
{
    m_settings.m_alpacaApiLogEnabled = checked;
    applySetting("alpacaApiLogEnabled");
}

void CameraGUI::on_alpacaHostEdit_editingFinished()
{
    m_settings.m_alpacaHost = settingsUI()->alpacaHostEdit->text();
    applySetting("alpacaHost");
}

void CameraGUI::on_alpacaPortSpin_valueChanged(int value)
{
    m_settings.m_alpacaPort = static_cast<uint16_t>(value);
    applySetting("alpacaPort");
}

void CameraGUI::on_alpacaFocuserEnabledCheck_toggled(bool checked)
{
    m_settings.m_alpacaFocuserEnabled = checked;
    updateCameraSettingsVisibility();
    applySetting("alpacaFocuserEnabled");
}

void CameraGUI::on_alpacaFocuserCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_alpacaFocuserHost = settingsUI()->alpacaFocuserCombo->itemData(index, AccessoryAlpacaHostRole).toString();
    m_settings.m_alpacaFocuserPort = static_cast<uint16_t>(settingsUI()->alpacaFocuserCombo->itemData(index, AccessoryAlpacaPortRole).toUInt());
    m_settings.m_alpacaFocuserDeviceNumber = settingsUI()->alpacaFocuserCombo->itemData(index, AccessoryDeviceNumberRole).toInt();
    settingsUI()->alpacaFocuserHostEdit->setText(m_settings.m_alpacaFocuserHost);
    settingsUI()->alpacaFocuserPortSpin->setValue(m_settings.m_alpacaFocuserPort);
    applySettings({"alpacaFocuserHost", "alpacaFocuserPort", "alpacaFocuserDeviceNumber"});
}

void CameraGUI::on_alpacaFocuserHostEdit_editingFinished()
{
    m_settings.m_alpacaFocuserHost = settingsUI()->alpacaFocuserHostEdit->text();
    applySetting("alpacaFocuserHost");
}

void CameraGUI::on_alpacaFocuserPortSpin_valueChanged(int value)
{
    m_settings.m_alpacaFocuserPort = static_cast<uint16_t>(value);
    applySetting("alpacaFocuserPort");
}

void CameraGUI::on_alpacaFocusPositionSpin_valueChanged(int value)
{
    m_settings.m_alpacaFocusPosition = value;
    applySetting("alpacaFocusPosition");
}

void CameraGUI::on_alpacaFocusStepSizeSpin_valueChanged(int value)
{
    m_settings.m_alpacaFocusStepSize = value;
    applySetting("alpacaFocusStepSize");
}

void CameraGUI::on_alpacaFilterWheelEnabledCheck_toggled(bool checked)
{
    m_settings.m_alpacaFilterWheelEnabled = checked;
    updateCameraSettingsVisibility();
    applySetting("alpacaFilterWheelEnabled");
}

void CameraGUI::on_alpacaFilterWheelCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_alpacaFilterWheelHost = settingsUI()->alpacaFilterWheelCombo->itemData(index, AccessoryAlpacaHostRole).toString();
    m_settings.m_alpacaFilterWheelPort = static_cast<uint16_t>(settingsUI()->alpacaFilterWheelCombo->itemData(index, AccessoryAlpacaPortRole).toUInt());
    m_settings.m_alpacaFilterWheelDeviceNumber = settingsUI()->alpacaFilterWheelCombo->itemData(index, AccessoryDeviceNumberRole).toInt();
    settingsUI()->alpacaFilterWheelHostEdit->setText(m_settings.m_alpacaFilterWheelHost);
    settingsUI()->alpacaFilterWheelPortSpin->setValue(m_settings.m_alpacaFilterWheelPort);
    applySettings({"alpacaFilterWheelHost", "alpacaFilterWheelPort", "alpacaFilterWheelDeviceNumber"});
}

void CameraGUI::on_alpacaFilterWheelHostEdit_editingFinished()
{
    m_settings.m_alpacaFilterWheelHost = settingsUI()->alpacaFilterWheelHostEdit->text();
    applySetting("alpacaFilterWheelHost");
}

void CameraGUI::on_alpacaFilterWheelPortSpin_valueChanged(int value)
{
    m_settings.m_alpacaFilterWheelPort = static_cast<uint16_t>(value);
    applySetting("alpacaFilterWheelPort");
}

void CameraGUI::on_alpacaFilterWheelPositionCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_alpacaFilterWheelPosition = settingsUI()->alpacaFilterWheelPositionCombo->itemData(index).toInt();
    applySetting("alpacaFilterWheelPosition");
}

void CameraGUI::on_cameraBinXSpin_valueChanged(int value)
{
    m_settings.m_cameraBinX = value;
    updateCameraSubframeControls();
    applySettings({"cameraBinX", "cameraNumX", "cameraStartX"});
}

void CameraGUI::on_cameraBinYSpin_valueChanged(int value)
{
    m_settings.m_cameraBinY = value;
    updateCameraSubframeControls();
    applySettings({"cameraBinY", "cameraNumY", "cameraStartY"});
}

void CameraGUI::on_cameraNumXSpin_valueChanged(int value)
{
    m_settings.m_cameraNumX = value;
    updateCameraSubframeControls();
    applySettings({"cameraNumX", "cameraStartX"});
}

void CameraGUI::on_cameraNumYSpin_valueChanged(int value)
{
    m_settings.m_cameraNumY = value;
    updateCameraSubframeControls();
    applySettings({"cameraNumY", "cameraStartY"});
}

void CameraGUI::on_cameraStartXSpin_valueChanged(int value)
{
    m_settings.m_cameraStartX = value;
    updateCameraSubframeControls();
    applySettings({"cameraStartX", "cameraNumX"});
}

void CameraGUI::on_cameraStartYSpin_valueChanged(int value)
{
    m_settings.m_cameraStartY = value;
    updateCameraSubframeControls();
    applySettings({"cameraStartY", "cameraNumY"});
}

void CameraGUI::on_cameraGainCombo_currentIndexChanged(int index)
{
    m_settings.m_cameraGain = index;
    applySetting("cameraGain");
}

void CameraGUI::on_cameraGainSlider_valueChanged(int value)
{
    settingsUI()->cameraGainSpin->blockSignals(true);
    settingsUI()->cameraGainSpin->setValue(value);
    settingsUI()->cameraGainSpin->blockSignals(false);
    m_settings.m_cameraGain = value;
    applySetting("cameraGain");
}

void CameraGUI::on_cameraGainSpin_valueChanged(int value)
{
    settingsUI()->cameraGainSlider->blockSignals(true);
    settingsUI()->cameraGainSlider->setValue(value);
    settingsUI()->cameraGainSlider->blockSignals(false);
    m_settings.m_cameraGain = value;
    applySetting("cameraGain");
}

void CameraGUI::on_cameraOffsetCombo_currentIndexChanged(int index)
{
    m_settings.m_cameraOffset = index;
    applySetting("cameraOffset");
}

void CameraGUI::on_cameraOffsetSlider_valueChanged(int value)
{
    settingsUI()->cameraOffsetSpin->blockSignals(true);
    settingsUI()->cameraOffsetSpin->setValue(value);
    settingsUI()->cameraOffsetSpin->blockSignals(false);
    m_settings.m_cameraOffset = value;
    applySetting("cameraOffset");
}

void CameraGUI::on_cameraOffsetSpin_valueChanged(int value)
{
    settingsUI()->cameraOffsetSlider->blockSignals(true);
    settingsUI()->cameraOffsetSlider->setValue(value);
    settingsUI()->cameraOffsetSlider->blockSignals(false);
    m_settings.m_cameraOffset = value;
    applySetting("cameraOffset");
}

void CameraGUI::on_alpacaReadoutModeCombo_currentIndexChanged(int index)
{
    m_settings.m_cameraReadoutMode = index;
    applySetting("cameraReadoutMode");
}

void CameraGUI::on_asiCoolerOnCheck_toggled(bool checked)
{
    m_settings.m_asiCoolerOn = checked ? 1 : 0;
    applySetting("asiCoolerOn");
}

void CameraGUI::on_asiTargetTempSpin_valueChanged(int value)
{
    m_settings.m_asiTargetTemp = value;
    applySetting("asiTargetTemp");
}

void CameraGUI::on_asiUsbBandwidthSpin_valueChanged(int value)
{
    m_settings.m_asiUsbBandwidth = value;
    applySetting("asiUsbBandwidth");
}

void CameraGUI::on_asiHighSpeedModeCheck_toggled(bool checked)
{
    m_settings.m_asiHighSpeedMode = checked ? 1 : 0;
    applySetting("asiHighSpeedMode");
}

void CameraGUI::on_asiAutoExposureGainCheck_toggled(bool checked)
{
    m_settings.m_asiAutoExposureGain = checked;
    updateCameraSettingsVisibility();
    applySetting("asiAutoExposureGain");
}

void CameraGUI::on_asiColorImageTypeCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_asiColorImageType = static_cast<CameraSettings::AsiColorImageType>(
        settingsUI()->asiColorImageTypeCombo->itemData(index).toInt());
    applySetting("asiColorImageType");
}

void CameraGUI::on_saveImageCheck_toggled(bool checked)
{
    m_settings.m_saveImage = checked;
    applySetting("saveImage");
}

void CameraGUI::on_imagePathEdit_editingFinished()
{
    m_settings.m_imageFileName = settingsUI()->imagePathEdit->text();
    applySetting("imageFileName");
    applyImageToolTip();
}

void CameraGUI::on_imagePathButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Save JPEG"), m_settings.m_imageFileName, tr("JPEG image (*.jpg *.jpeg)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_imageFileName = fileName;
        settingsUI()->imagePathEdit->setText(fileName);
        applySetting("imageFileName");
        applyImageToolTip();
    }
}

void CameraGUI::on_saveVideoCheck_toggled(bool checked)
{
    m_settings.m_saveVideo = checked;
    applySetting("saveVideo");
}

void CameraGUI::on_videoPathEdit_editingFinished()
{
    m_settings.m_videoFileName = settingsUI()->videoPathEdit->text();
    applySetting("videoFileName");
    applyVideoToolTip();
}

void CameraGUI::on_videoPathButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Save video"), m_settings.m_videoFileName, tr("MPEG video (*.mp4 *.mov)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_videoFileName = fileName;
        settingsUI()->videoPathEdit->setText(fileName);
        applySetting("videoFileName");
        applyVideoToolTip();
    }
}

void CameraGUI::on_videoHwAccelerationCheck_toggled(bool checked)
{
    m_settings.m_videoHwAcceleration = checked;
    applySetting("videoHwAcceleration");
}

void CameraGUI::on_recordModeCombo_currentIndexChanged(int index)
{
    m_settings.m_recordMode = qBound(CameraSettings::SavedMediaRaw,
        static_cast<CameraSettings::SavedMediaMode>(index),
        CameraSettings::SavedMediaBoth);
    applySetting("videoPostProcess");
    applyImageToolTip();
    applyVideoToolTip();
}

void CameraGUI::on_stackEnabledCheck_toggled(bool checked)
{
    m_settings.m_stackEnabled = checked;
    applySetting("stackEnabled");
}

void CameraGUI::on_stackFrameCountSpin_valueChanged(int value)
{
    m_settings.m_stackFrameCount = value;
    applySetting("stackFrameCount");
}

void CameraGUI::on_stackMethodCombo_currentIndexChanged(int index)
{
    m_settings.m_stackMethod = static_cast<CameraSettings::StackMethod>(index);
    applySetting("stackMethod");
}

void CameraGUI::on_stackAlignmentCombo_currentIndexChanged(int index)
{
    m_settings.m_stackAlignmentMethod = static_cast<CameraSettings::StackAlignmentMethod>(index);
    applySetting("stackAlignmentMethod");
}

void CameraGUI::on_stackDarkFileEdit_editingFinished()
{
    m_settings.m_stackDarkFileName = settingsUI()->stackDarkFileEdit->text();
    applySetting("stackDarkFileName");
}

void CameraGUI::on_stackDarkFileButton_clicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Select dark FITS"),
        m_settings.m_stackDarkFileName,
        tr("FITS files (*.fits *.fit *.fts);;All files (*)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_stackDarkFileName = fileName;
        settingsUI()->stackDarkFileEdit->setText(fileName);
        applySetting("stackDarkFileName");
    }
}

void CameraGUI::on_stackFlatFileEdit_editingFinished()
{
    m_settings.m_stackFlatFileName = settingsUI()->stackFlatFileEdit->text();
    applySetting("stackFlatFileName");
}

void CameraGUI::on_stackFlatFileButton_clicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Select flat FITS"),
        m_settings.m_stackFlatFileName,
        tr("FITS files (*.fits *.fit *.fts);;All files (*)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_stackFlatFileName = fileName;
        settingsUI()->stackFlatFileEdit->setText(fileName);
        applySetting("stackFlatFileName");
    }
}

void CameraGUI::on_stackBiasFileEdit_editingFinished()
{
    m_settings.m_stackBiasFileName = settingsUI()->stackBiasFileEdit->text();
    applySetting("stackBiasFileName");
}

void CameraGUI::on_stackBiasFileButton_clicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Select bias FITS"),
        m_settings.m_stackBiasFileName,
        tr("FITS files (*.fits *.fit *.fts);;All files (*)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_stackBiasFileName = fileName;
        settingsUI()->stackBiasFileEdit->setText(fileName);
        applySetting("stackBiasFileName");
    }
}

void CameraGUI::on_latitudeSpin_valueChanged(double value)
{
    m_settings.m_latitude = static_cast<float>(value);
    applySetting("latitude");
}

void CameraGUI::on_longitudeSpin_valueChanged(double value)
{
    m_settings.m_longitude = static_cast<float>(value);
    applySetting("longitude");
}

void CameraGUI::on_altitudeSpin_valueChanged(double value)
{
    m_settings.m_altitude = static_cast<float>(value);
    applySetting("altitude");
}

void CameraGUI::on_owmApiKeyEdit_editingFinished()
{
    m_settings.m_owmAPIKey = settingsUI()->owmApiKeyEdit->text().trimmed();
    applySetting("owmAPIKey");
}

void CameraGUI::on_useMyPositionButton_clicked()
{
    syncFromMainSettings();
}

void CameraGUI::useMyPositionButton_rightClicked(const QPoint& p)
{
    (void) p;
    m_settings.m_positionSync = !m_settings.m_positionSync;
    applyPositionSync();
    updatePositionControls();
    applySetting("positionSync");
}

void CameraGUI::on_azimuthSpin_valueChanged(double value)
{
    m_settings.m_azimuth = static_cast<float>(value);
    applySetting("azimuth");
}

void CameraGUI::on_elevationSpin_valueChanged(double value)
{
    m_settings.m_elevation = static_cast<float>(value);
    applySetting("elevation");
}

void CameraGUI::on_rollSpin_valueChanged(double value)
{
    m_settings.m_roll = static_cast<float>(value);
    applySetting("roll");
}

void CameraGUI::on_rotatorControllerCombo_currentIndexChanged(int index)
{
    m_settings.m_rotator = settingsUI()->rotatorControllerCombo->itemData(index).toString();
    updatePositionControls();
    if (!m_settings.m_rotator.isEmpty()) {
        syncFromSelectedGs232Controller();
    }
    applySetting("rotator");
}

void CameraGUI::on_fovSpin_valueChanged(double value)
{
    m_settings.m_fov = static_cast<float>(value);
    applySetting("fov");
}

void CameraGUI::on_lensProjectionCombo_currentIndexChanged(int index)
{
    m_settings.m_lensProjection = static_cast<CameraSettings::LensProjection>(index);
    applySetting("lensProjection");
}

void CameraGUI::on_scheduleEnabledCheck_toggled(bool checked)
{
    m_settings.m_scheduleEnabled = checked;
    if (!checked)
    {
        m_scheduleManualStartLatch = false;
        m_scheduleManualStopLatch = false;
    }
    updateScheduleControls();
    applySetting("scheduleEnabled");
    updateScheduledCapture();
}

void CameraGUI::on_scheduleStartTimeEdit_timeChanged(const QTime& time)
{
    m_settings.m_scheduleStartTime = time.toString(QStringLiteral("HH:mm:ss"));
    updateScheduleControls();
    applySetting("scheduleStartTime");
    updateScheduledCapture();
}

void CameraGUI::on_scheduleEndTimeEdit_timeChanged(const QTime& time)
{
    m_settings.m_scheduleEndTime = time.toString(QStringLiteral("HH:mm:ss"));
    updateScheduleControls();
    applySetting("scheduleEndTime");
    updateScheduledCapture();
}

void CameraGUI::on_scheduleWeekdayCheck_toggled(bool checked)
{
    Q_UNUSED(checked)

    m_settings.m_scheduleWeekdays = scheduleWeekdayMaskFromUi();
    updateScheduleControls();
    applySetting("scheduleWeekdays");
    updateScheduledCapture();
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

void CameraGUI::updateHistogramStretchControls()
{
    const CameraSettings::HistogramStretch stretchMode = m_settings.m_histogramStretch;
    const bool stretchEnabled = stretchMode != CameraSettings::HistogramStretchOff;
    const bool gammaMode = stretchMode == CameraSettings::HistogramStretchGamma;
    const bool asinhMode = stretchMode == CameraSettings::HistogramStretchAsinh;
    const bool logMode = stretchMode == CameraSettings::HistogramStretchLog;

    settingsUI()->histogramStretchBlackPointLabel->setEnabled(stretchEnabled);
    settingsUI()->histogramStretchBlackPointSlider->setEnabled(stretchEnabled);
    settingsUI()->histogramStretchBlackPointSpin->setEnabled(stretchEnabled);
    settingsUI()->histogramStretchWhitePointLabel->setEnabled(stretchEnabled);
    settingsUI()->histogramStretchWhitePointSlider->setEnabled(stretchEnabled);
    settingsUI()->histogramStretchWhitePointSpin->setEnabled(stretchEnabled);

    settingsUI()->histogramStretchGammaLabel->setEnabled(gammaMode);
    settingsUI()->histogramStretchGammaSlider->setEnabled(gammaMode);
    settingsUI()->histogramStretchGammaSpin->setEnabled(gammaMode);
    settingsUI()->histogramStretchAsinhLabel->setEnabled(asinhMode);
    settingsUI()->histogramStretchAsinhSlider->setEnabled(asinhMode);
    settingsUI()->histogramStretchAsinhSpin->setEnabled(asinhMode);
    settingsUI()->histogramStretchLogLabel->setEnabled(logMode);
    settingsUI()->histogramStretchLogSlider->setEnabled(logMode);
    settingsUI()->histogramStretchLogSpin->setEnabled(logMode);
}

void CameraGUI::updateMotionExclusionRectsTable()
{
    m_updatingMotionExclusionRectsTable = true;
    settingsUI()->motionExclusionTable->setRowCount(m_settings.m_motionExclusionRects.size());

    for (int i = 0; i < m_settings.m_motionExclusionRects.size(); ++i)
    {
        const QRect& rect = m_settings.m_motionExclusionRects.at(i);
        settingsUI()->motionExclusionTable->setItem(i, 0, new QTableWidgetItem(QString::number(rect.x())));
        settingsUI()->motionExclusionTable->setItem(i, 1, new QTableWidgetItem(QString::number(rect.y())));
        settingsUI()->motionExclusionTable->setItem(i, 2, new QTableWidgetItem(QString::number(rect.width())));
        settingsUI()->motionExclusionTable->setItem(i, 3, new QTableWidgetItem(QString::number(rect.height())));
    }

    m_updatingMotionExclusionRectsTable = false;
    updateMotionExclusionPreview();
}

void CameraGUI::applyMotionExclusionRectsFromTable()
{
    QList<QRect> rects;

    for (int row = 0; row < settingsUI()->motionExclusionTable->rowCount(); ++row)
    {
        auto *xItem = settingsUI()->motionExclusionTable->item(row, 0);
        auto *yItem = settingsUI()->motionExclusionTable->item(row, 1);
        auto *wItem = settingsUI()->motionExclusionTable->item(row, 2);
        auto *hItem = settingsUI()->motionExclusionTable->item(row, 3);

        const int x = qMax(0, xItem ? xItem->text().toInt() : 0);
        const int y = qMax(0, yItem ? yItem->text().toInt() : 0);
        const int w = qMax(1, wItem ? wItem->text().toInt() : 1);
        const int h = qMax(1, hItem ? hItem->text().toInt() : 1);
        rects.append(QRect(x, y, w, h));
    }

    m_settings.m_motionExclusionRects = rects;
    updateMotionExclusionPreview();
}

void CameraGUI::updateMotionExclusionPreview()
{
    if (!m_imageScene || !m_imagePixmapItem) {
        return;
    }

    for (QGraphicsRectItem *item : std::as_const(m_motionExclusionRectItems))
    {
        if (item) {
            m_imageScene->removeItem(item);
            delete item;
        }
    }
    m_motionExclusionRectItems.clear();

    if (m_lastImage.isNull() || !m_showMotionExclusionRects) {
        return;
    }

    QPen pen(QColor(255, 215, 0));
    pen.setWidth(2);
    pen.setStyle(Qt::DashLine);

    const QRect imageBounds(0, 0, m_lastImage.width(), m_lastImage.height());

    for (const QRect& rect : m_settings.m_motionExclusionRects)
    {
        const QRect clipped = rect.intersected(imageBounds);
        if (!clipped.isValid() || clipped.isEmpty()) {
            continue;
        }

        QGraphicsRectItem *item = m_imageScene->addRect(QRectF(clipped), pen);
        item->setZValue(1.0);
        m_motionExclusionRectItems.append(item);
    }
}

void CameraGUI::setMotionExclusionDrawMode(bool enabled)
{
    m_motionExclusionDrawMode = enabled;
    m_motionExclusionDragging = false;

    if (!enabled && m_motionExclusionDragItem)
    {
        m_imageScene->removeItem(m_motionExclusionDragItem);
        delete m_motionExclusionDragItem;
        m_motionExclusionDragItem = nullptr;
    }

    ui->imageView->setDragMode(enabled ? QGraphicsView::NoDrag : QGraphicsView::ScrollHandDrag);
    ui->imageView->viewport()->setCursor(enabled ? Qt::CrossCursor : Qt::ArrowCursor);
}

QPoint CameraGUI::mapViewportPointToImage(const QPoint& viewportPos) const
{
    if (m_lastImage.isNull() || !m_imagePixmapItem) {
        return QPoint(-1, -1);
    }

    const QPointF scenePos = ui->imageView->mapToScene(viewportPos);
    const int x = qBound(0, static_cast<int>(std::floor(scenePos.x())), m_lastImage.width() - 1);
    const int y = qBound(0, static_cast<int>(std::floor(scenePos.y())), m_lastImage.height() - 1);
    return QPoint(x, y);
}

void CameraGUI::on_postProcessWhiteBalanceModeCombo_currentIndexChanged(int index)
{
    m_settings.m_postProcessWhiteBalanceMode = index;
    updatePostProcessWhiteBalanceControls();
    applySetting("postProcessWhiteBalanceMode");
}

void CameraGUI::on_postProcessWhiteBalanceRedGainSlider_valueChanged(int value)
{
    const double gain = sliderValueToDoubleSpinBox(settingsUI()->postProcessWhiteBalanceRedGainSpin, value);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->setValue(gain);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceRedGain = gain;
    applySetting("postProcessWhiteBalanceRedGain");
}

void CameraGUI::on_postProcessWhiteBalanceRedGainSpin_valueChanged(double value)
{
    settingsUI()->postProcessWhiteBalanceRedGainSlider->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceRedGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceRedGainSpin, value));
    settingsUI()->postProcessWhiteBalanceRedGainSlider->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceRedGain = value;
    applySetting("postProcessWhiteBalanceRedGain");
}

void CameraGUI::on_postProcessWhiteBalanceGreenGainSlider_valueChanged(int value)
{
    const double gain = sliderValueToDoubleSpinBox(settingsUI()->postProcessWhiteBalanceGreenGainSpin, value);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->setValue(gain);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceGreenGain = gain;
    applySetting("postProcessWhiteBalanceGreenGain");
}

void CameraGUI::on_postProcessWhiteBalanceGreenGainSpin_valueChanged(double value)
{
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceGreenGainSpin, value));
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceGreenGain = value;
    applySetting("postProcessWhiteBalanceGreenGain");
}

void CameraGUI::on_postProcessWhiteBalanceBlueGainSlider_valueChanged(int value)
{
    const double gain = sliderValueToDoubleSpinBox(settingsUI()->postProcessWhiteBalanceBlueGainSpin, value);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->setValue(gain);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceBlueGain = gain;
    applySetting("postProcessWhiteBalanceBlueGain");
}

void CameraGUI::on_postProcessWhiteBalanceBlueGainSpin_valueChanged(double value)
{
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceBlueGainSpin, value));
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceBlueGain = value;
    applySetting("postProcessWhiteBalanceBlueGain");
}

void CameraGUI::on_postProcessGreyscaleCheck_toggled(bool checked)
{
    m_settings.m_postProcessGreyscale = checked;
    applySetting("postProcessGreyscale");
}

void CameraGUI::on_postProcessUnwarpCheck_toggled(bool checked)
{
    m_settings.m_postProcessUnwarp = checked;
    applySetting("postProcessUnwarp");
}

void CameraGUI::on_histogramStretchModeCombo_currentIndexChanged(int index)
{
    m_settings.m_histogramStretch = static_cast<CameraSettings::HistogramStretch>(
        qBound(static_cast<int>(CameraSettings::HistogramStretchOff), index, static_cast<int>(CameraSettings::HistogramStretchLog)));
    updateHistogramStretchControls();
    applySetting("histogramStretch");
}

void CameraGUI::on_histogramStretchBlackPointSlider_valueChanged(int value)
{
    const double blackPoint = value / 1000.0;
    settingsUI()->histogramStretchBlackPointSpin->blockSignals(true);
    settingsUI()->histogramStretchBlackPointSpin->setValue(blackPoint);
    settingsUI()->histogramStretchBlackPointSpin->blockSignals(false);
    m_settings.m_histogramStretchBlackPoint = blackPoint;

    if (m_settings.m_histogramStretchWhitePoint <= blackPoint) {
        settingsUI()->histogramStretchWhitePointSpin->setValue(std::min(1.0, blackPoint + 0.001));
    }

    applySetting("histogramStretchBlackPoint");
}

void CameraGUI::on_histogramStretchBlackPointSpin_valueChanged(double value)
{
    settingsUI()->histogramStretchBlackPointSlider->blockSignals(true);
    settingsUI()->histogramStretchBlackPointSlider->setValue(static_cast<int>(std::lround(value * 1000.0)));
    settingsUI()->histogramStretchBlackPointSlider->blockSignals(false);
    m_settings.m_histogramStretchBlackPoint = value;

    if (m_settings.m_histogramStretchWhitePoint <= value) {
        settingsUI()->histogramStretchWhitePointSpin->setValue(std::min(1.0, value + 0.001));
    }

    applySetting("histogramStretchBlackPoint");
}

void CameraGUI::on_histogramStretchWhitePointSlider_valueChanged(int value)
{
    const double whitePoint = value / 1000.0;
    settingsUI()->histogramStretchWhitePointSpin->blockSignals(true);
    settingsUI()->histogramStretchWhitePointSpin->setValue(whitePoint);
    settingsUI()->histogramStretchWhitePointSpin->blockSignals(false);
    m_settings.m_histogramStretchWhitePoint = whitePoint;

    if (whitePoint <= m_settings.m_histogramStretchBlackPoint) {
        settingsUI()->histogramStretchBlackPointSpin->setValue(std::max(0.0, whitePoint - 0.001));
    }

    applySetting("histogramStretchWhitePoint");
}

void CameraGUI::on_histogramStretchWhitePointSpin_valueChanged(double value)
{
    settingsUI()->histogramStretchWhitePointSlider->blockSignals(true);
    settingsUI()->histogramStretchWhitePointSlider->setValue(static_cast<int>(std::lround(value * 1000.0)));
    settingsUI()->histogramStretchWhitePointSlider->blockSignals(false);
    m_settings.m_histogramStretchWhitePoint = value;

    if (value <= m_settings.m_histogramStretchBlackPoint) {
        settingsUI()->histogramStretchBlackPointSpin->setValue(std::max(0.0, value - 0.001));
    }

    applySetting("histogramStretchWhitePoint");
}

void CameraGUI::on_histogramStretchGammaSlider_valueChanged(int value)
{
    const double gammaValue = value / 100.0;
    settingsUI()->histogramStretchGammaSpin->blockSignals(true);
    settingsUI()->histogramStretchGammaSpin->setValue(gammaValue);
    settingsUI()->histogramStretchGammaSpin->blockSignals(false);
    m_settings.m_histogramStretchGamma = gammaValue;
    applySetting("histogramStretchGamma");
}

void CameraGUI::on_histogramStretchGammaSpin_valueChanged(double value)
{
    settingsUI()->histogramStretchGammaSlider->blockSignals(true);
    settingsUI()->histogramStretchGammaSlider->setValue(static_cast<int>(std::lround(value * 100.0)));
    settingsUI()->histogramStretchGammaSlider->blockSignals(false);
    m_settings.m_histogramStretchGamma = value;
    applySetting("histogramStretchGamma");
}

void CameraGUI::on_histogramStretchAsinhSlider_valueChanged(int value)
{
    const double strength = value / 10.0;
    settingsUI()->histogramStretchAsinhSpin->blockSignals(true);
    settingsUI()->histogramStretchAsinhSpin->setValue(strength);
    settingsUI()->histogramStretchAsinhSpin->blockSignals(false);
    m_settings.m_histogramStretchAsinhStrength = strength;
    applySetting("histogramStretchAsinhStrength");
}

void CameraGUI::on_histogramStretchAsinhSpin_valueChanged(double value)
{
    settingsUI()->histogramStretchAsinhSlider->blockSignals(true);
    settingsUI()->histogramStretchAsinhSlider->setValue(static_cast<int>(std::lround(value * 10.0)));
    settingsUI()->histogramStretchAsinhSlider->blockSignals(false);
    m_settings.m_histogramStretchAsinhStrength = value;
    applySetting("histogramStretchAsinhStrength");
}

void CameraGUI::on_histogramStretchLogSlider_valueChanged(int value)
{
    const double strength = value / 10.0;
    settingsUI()->histogramStretchLogSpin->blockSignals(true);
    settingsUI()->histogramStretchLogSpin->setValue(strength);
    settingsUI()->histogramStretchLogSpin->blockSignals(false);
    m_settings.m_histogramStretchLogStrength = strength;
    applySetting("histogramStretchLogStrength");
}

void CameraGUI::on_histogramStretchLogSpin_valueChanged(double value)
{
    settingsUI()->histogramStretchLogSlider->blockSignals(true);
    settingsUI()->histogramStretchLogSlider->setValue(static_cast<int>(std::lround(value * 10.0)));
    settingsUI()->histogramStretchLogSlider->blockSignals(false);
    m_settings.m_histogramStretchLogStrength = value;
    applySetting("histogramStretchLogStrength");
}

void CameraGUI::on_saturationSlider_valueChanged(int value)
{
    m_settings.m_saturation = value / 100.0;
    settingsUI()->saturationSpin->blockSignals(true);
    settingsUI()->saturationSpin->setValue(m_settings.m_saturation);
    settingsUI()->saturationSpin->blockSignals(false);
    applySetting("saturation");
}

void CameraGUI::on_saturationSpin_valueChanged(double value)
{
    settingsUI()->saturationSlider->blockSignals(true);
    settingsUI()->saturationSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->saturationSlider->blockSignals(false);
    m_settings.m_saturation = value;
    applySetting("saturation");
}

void CameraGUI::on_gammaSlider_valueChanged(int value)
{
    m_settings.m_gamma = value / 100.0;
    settingsUI()->gammaSpin->blockSignals(true);
    settingsUI()->gammaSpin->setValue(m_settings.m_gamma);
    settingsUI()->gammaSpin->blockSignals(false);
    applySetting("gamma");
}

void CameraGUI::on_gammaSpin_valueChanged(double value)
{
    settingsUI()->gammaSlider->blockSignals(true);
    settingsUI()->gammaSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->gammaSlider->blockSignals(false);
    m_settings.m_gamma = value;
    applySetting("gamma");
}

void CameraGUI::on_gaussianBlurSlider_valueChanged(int value)
{
    settingsUI()->gaussianBlurSpin->blockSignals(true);
    settingsUI()->gaussianBlurSpin->setValue(value);
    settingsUI()->gaussianBlurSpin->blockSignals(false);
    m_settings.m_gaussianBlur = value;
    applySetting("gaussianBlur");
}

void CameraGUI::on_gaussianBlurSpin_valueChanged(int value)
{
    settingsUI()->gaussianBlurSlider->blockSignals(true);
    settingsUI()->gaussianBlurSlider->setValue(value);
    settingsUI()->gaussianBlurSlider->blockSignals(false);
    m_settings.m_gaussianBlur = value;
    applySetting("gaussianBlur");
}

void CameraGUI::on_medianBlurSlider_valueChanged(int value)
{
    settingsUI()->medianBlurSpin->blockSignals(true);
    settingsUI()->medianBlurSpin->setValue(value);
    settingsUI()->medianBlurSpin->blockSignals(false);
    m_settings.m_medianBlur = value;
    applySetting("medianBlur");
}

void CameraGUI::on_medianBlurSpin_valueChanged(int value)
{
    settingsUI()->medianBlurSlider->blockSignals(true);
    settingsUI()->medianBlurSlider->setValue(value);
    settingsUI()->medianBlurSlider->blockSignals(false);
    m_settings.m_medianBlur = value;
    applySetting("medianBlur");
}

void CameraGUI::on_sharpenSlider_valueChanged(int value)
{
    m_settings.m_sharpen = value / 100.0;
    settingsUI()->sharpenSpin->blockSignals(true);
    settingsUI()->sharpenSpin->setValue(m_settings.m_sharpen);
    settingsUI()->sharpenSpin->blockSignals(false);
    applySetting("sharpen");
}

void CameraGUI::on_sharpenSpin_valueChanged(double value)
{
    settingsUI()->sharpenSlider->blockSignals(true);
    settingsUI()->sharpenSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->sharpenSlider->blockSignals(false);
    m_settings.m_sharpen = value;
    applySetting("sharpen");
}

void CameraGUI::on_sobelEdgeSlider_valueChanged(int value)
{
    m_settings.m_sobelEdge = value / 100.0;
    settingsUI()->sobelEdgeSpin->blockSignals(true);
    settingsUI()->sobelEdgeSpin->setValue(m_settings.m_sobelEdge);
    settingsUI()->sobelEdgeSpin->blockSignals(false);
    applySetting("sobelEdge");
}

void CameraGUI::on_sobelEdgeSpin_valueChanged(double value)
{
    settingsUI()->sobelEdgeSlider->blockSignals(true);
    settingsUI()->sobelEdgeSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->sobelEdgeSlider->blockSignals(false);
    m_settings.m_sobelEdge = value;
    applySetting("sobelEdge");
}

void CameraGUI::on_flipXButton_toggled(bool checked)
{
    m_settings.m_flipX = checked;
    applySetting("flipX");
}

void CameraGUI::on_flipYButton_toggled(bool checked)
{
    m_settings.m_flipY = checked;
    applySetting("flipY");
}

void CameraGUI::on_brightnessSlider_valueChanged(int value)
{
    m_settings.m_brightness = static_cast<double>(value);
    settingsUI()->brightnessSpin->blockSignals(true);
    settingsUI()->brightnessSpin->setValue(value);
    settingsUI()->brightnessSpin->blockSignals(false);
    applySetting("brightness");
}

void CameraGUI::on_brightnessSpin_valueChanged(int value)
{
    settingsUI()->brightnessSlider->blockSignals(true);
    settingsUI()->brightnessSlider->setValue(value);
    settingsUI()->brightnessSlider->blockSignals(false);
    m_settings.m_brightness = static_cast<double>(value);
    applySetting("brightness");
}

void CameraGUI::on_contrastSlider_valueChanged(int value)
{
    m_settings.m_contrast = value / 100.0;
    settingsUI()->contrastSpin->blockSignals(true);
    settingsUI()->contrastSpin->setValue(m_settings.m_contrast);
    settingsUI()->contrastSpin->blockSignals(false);
    applySetting("contrast");
}

void CameraGUI::on_contrastSpin_valueChanged(double value)
{
    settingsUI()->contrastSlider->blockSignals(true);
    settingsUI()->contrastSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->contrastSlider->blockSignals(false);
    m_settings.m_contrast = value;
    applySetting("contrast");
}

void CameraGUI::on_invertColorsButton_toggled(bool checked)
{
    m_settings.m_invertColors = checked;
    applySetting("invertColors");
}

void CameraGUI::on_overlayDateTimeButton_toggled(bool checked)
{
    m_settings.m_overlayDateTime = checked;
    applySetting("overlayDateTime");
}

void CameraGUI::on_dateTimeColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_dateTimeColor, this, tr("Select date/time text colour"));

    if (color.isValid())
    {
        m_settings.m_dateTimeColor = color;
        updateColorButton(settingsUI()->dateTimeColorButton, m_settings.m_dateTimeColor);
        applySetting("dateTimeColor");
    }
}

void CameraGUI::on_dateTimeFormatEdit_editingFinished()
{
    m_settings.m_dateTimeFormat = settingsUI()->dateTimeFormatEdit->text();
    applySetting("dateTimeFormat");
}

void CameraGUI::on_dateTimePosXSlider_valueChanged(int value)
{
    m_settings.m_dateTimePosX = value;
    settingsUI()->dateTimePosXValue->setText(QString::number(value));
    applySetting("dateTimePosX");
}

void CameraGUI::on_dateTimePosYSlider_valueChanged(int value)
{
    m_settings.m_dateTimePosY = value;
    settingsUI()->dateTimePosYValue->setText(QString::number(value));
    applySetting("dateTimePosY");
}

void CameraGUI::on_equatorialGridCheck_toggled(bool checked)
{
    m_settings.m_equatorialGrid = checked;
    applySetting("equatorialGrid");
}

void CameraGUI::on_equatorialGridColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_equatorialGridColor, this, tr("Select equatorial grid colour"));

    if (color.isValid())
    {
        m_settings.m_equatorialGridColor = color;
        updateColorButton(settingsUI()->equatorialGridColorButton, m_settings.m_equatorialGridColor);
        applySetting("equatorialGridColor");
    }
}

void CameraGUI::on_altAzGridCheck_toggled(bool checked)
{
    m_settings.m_altAzGrid = checked;
    applySetting("altAzGrid");
}

void CameraGUI::on_altAzGridColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_altAzGridColor, this, tr("Select alt-az grid colour"));

    if (color.isValid())
    {
        m_settings.m_altAzGridColor = color;
        updateColorButton(settingsUI()->altAzGridColorButton, m_settings.m_altAzGridColor);
        applySetting("altAzGridColor");
    }
}

void CameraGUI::on_trackObjectsCheck_toggled(bool checked)
{
    m_settings.m_trackObjects = checked;
    applySetting("trackObjects");
}

void CameraGUI::on_trackObjectMinElevationSpin_valueChanged(double value)
{
    m_settings.m_trackObjectMinElevation = value;
    applySetting("trackObjectMinElevation");
}

void CameraGUI::on_trackObjectColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_trackObjectColor, this, tr("Select tracked object colour"));

    if (color.isValid())
    {
        m_settings.m_trackObjectColor = color;
        updateColorButton(settingsUI()->trackObjectColorButton, m_settings.m_trackObjectColor);
        applySetting("trackObjectColor");
    }
}

void CameraGUI::on_trackObjectFontScaleSpin_valueChanged(double value)
{
    m_settings.m_trackObjectFontScale = value;
    applySetting("trackObjectFontScale");
}

void CameraGUI::on_gridLabelFontCombo_currentFontChanged(const QFont& font)
{
    m_settings.m_gridLabelFontFamily = font.family();
    applySetting("gridLabelFontFamily");
}

void CameraGUI::on_gridLabelFontScaleSpin_valueChanged(double value)
{
    m_settings.m_gridLabelFontScale = value;
    applySetting("gridLabelFontScale");
}

void CameraGUI::on_overlayTextButton_toggled(bool checked)
{
    m_settings.m_overlayText = checked;
    applySetting("overlayText");
}

void CameraGUI::on_overlayTextColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_overlayTextColor, this, tr("Select overlay text colour"));

    if (color.isValid())
    {
        m_settings.m_overlayTextColor = color;
        updateColorButton(settingsUI()->overlayTextColorButton, m_settings.m_overlayTextColor);
        applySetting("overlayTextColor");
    }
}

void CameraGUI::on_overlayTextEdit_textChanged()
{
    m_settings.m_overlayTextString = settingsUI()->overlayTextEdit->toPlainText();
    applySetting("overlayTextString");
}

void CameraGUI::on_overlayTextPosXSlider_valueChanged(int value)
{
    m_settings.m_overlayTextPosX = value;
    settingsUI()->overlayTextPosXValue->setText(QString::number(value));
    applySetting("overlayTextPosX");
}

void CameraGUI::on_overlayTextPosYSlider_valueChanged(int value)
{
    m_settings.m_overlayTextPosY = value;
    settingsUI()->overlayTextPosYValue->setText(QString::number(value));
    applySetting("overlayTextPosY");
}

void CameraGUI::on_diffMaskButton_toggled(bool checked)
{
    m_settings.m_diffMask = checked;
    applySetting("diffMask");
}

void CameraGUI::on_diffThresholdSpin_valueChanged(int value)
{
    m_settings.m_diffThreshold = value;
    applySetting("diffThreshold");
}

void CameraGUI::on_diffMaskOpenSizeSpin_valueChanged(int value)
{
    m_settings.m_diffMaskOpenSize = value;
    applySetting("diffMaskOpenSize");
}

void CameraGUI::on_dilationSpin_valueChanged(int value)
{
    m_settings.m_dilationSize = value;
    applySetting("dilationSize");
}

void CameraGUI::on_diffMaskHistoryFramesSpin_valueChanged(int value)
{
    m_settings.m_diffMaskHistoryFrames = value;
    applySetting("diffMaskHistoryFrames");
}

void CameraGUI::on_diffMaskCloseSizeSpin_valueChanged(int value)
{
    m_settings.m_diffMaskCloseSize = value;
    applySetting("diffMaskCloseSize");
}

void CameraGUI::on_histogramButton_clicked()
{
    if (m_lastHistogramData.isValid())
    {
        if (!m_histogramDialog)
        {
            m_histogramDialog = new CameraHistogramDialog(m_lastHistogramData, this);
            m_histogramDialog->setAttribute(Qt::WA_DeleteOnClose); // Delete when closed, so we don't waste CPU calculating the histogram when not visible
            connect(m_histogramDialog, &QObject::destroyed, this, [this]() { m_histogramDialog = nullptr; });
        }
        else
        {
            m_histogramDialog->updateHistogram(m_lastHistogramData);
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
    settingsUI()->postProcessUnwarpCheck->setChecked(false);
    settingsUI()->histogramStretchModeCombo->setCurrentIndex(static_cast<int>(CameraSettings::HistogramStretchOff));
    settingsUI()->histogramStretchBlackPointSpin->setValue(0.0);
    settingsUI()->histogramStretchWhitePointSpin->setValue(1.0);
    settingsUI()->histogramStretchGammaSpin->setValue(1.0);
    settingsUI()->histogramStretchAsinhSpin->setValue(10.0);
    settingsUI()->histogramStretchLogSpin->setValue(10.0);
    settingsUI()->postProcessGreyscaleCheck->setChecked(false);
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

void CameraGUI::updateColorButton(QToolButton* btn, const QColor& color)
{
    QPixmap px(16, 16);
    px.fill(color);
    btn->setIcon(QIcon(px));
    btn->setStyleSheet(QString());
}

void CameraGUI::on_overlayFontCombo_currentFontChanged(const QFont& font)
{
    m_settings.m_overlayFontFamily = font.family();
    applySetting("overlayFontFamily");
}

void CameraGUI::on_overlayFontScaleSpin_valueChanged(double value)
{
    m_settings.m_overlayFontScale = value;
    applySetting("overlayFontScale");
}

void CameraGUI::on_overlayTextFontCombo_currentFontChanged(const QFont& font)
{
    m_settings.m_overlayTextFontFamily = font.family();
    applySetting("overlayTextFontFamily");
}

void CameraGUI::on_overlayTextFontScaleSpin_valueChanged(double value)
{
    m_settings.m_overlayTextFontScale = value;
    applySetting("overlayTextFontScale");
}

void CameraGUI::on_detectionRoiXSpin_valueChanged(int value)
{
    m_settings.m_detectionRoiX = value;
    applySetting("detectionRoiX");
}

void CameraGUI::on_detectionRoiYSpin_valueChanged(int value)
{
    m_settings.m_detectionRoiY = value;
    applySetting("detectionRoiY");
}

void CameraGUI::on_detectionRoiWidthSpin_valueChanged(int value)
{
    m_settings.m_detectionRoiWidth = value;
    applySetting("detectionRoiWidth");
}

void CameraGUI::on_detectionRoiHeightSpin_valueChanged(int value)
{
    m_settings.m_detectionRoiHeight = value;
    applySetting("detectionRoiHeight");
}

void CameraGUI::on_motionDetectButton_toggled(bool checked)
{
    m_settings.m_motionDetect = checked;
    applySetting("motionDetect");
}

void CameraGUI::on_motionBackgroundSubtractorCombo_currentIndexChanged(int index)
{
    m_settings.m_motionBackgroundSubtractor = static_cast<CameraSettings::MotionBackgroundSubtractor>(index);
    applySetting("motionBackgroundSubtractor");
}

void CameraGUI::on_motionHistorySpin_valueChanged(int value)
{
    m_settings.m_motionHistory = value;
    applySetting("motionHistory");
}

void CameraGUI::on_motionVarThresholdSpin_valueChanged(double value)
{
    m_settings.m_motionVarThreshold = value;
    applySetting("motionVarThreshold");
}

void CameraGUI::on_motionLearningRateSpin_valueChanged(double value)
{
    m_settings.m_motionLearningRate = value;
    applySetting("motionLearningRate");
}

void CameraGUI::on_motionConfirmFramesSpin_valueChanged(int value)
{
    m_settings.m_motionConfirmFrames = value;
    applySetting("motionConfirmFrames");
}

void CameraGUI::on_motionDownscaleCombo_currentIndexChanged(int index)
{
    m_settings.m_motionDownscale = index == 1 ? 0.5 : index == 2 ? 0.25 : 1.0;
    applySetting("motionDownscale");
}

void CameraGUI::on_motionDetectShadowsCheck_toggled(bool checked)
{
    m_settings.m_motionDetectShadows = checked;
    applySetting("motionDetectShadows");
}

void CameraGUI::on_motionOpenSizeSpin_valueChanged(int value)
{
    m_settings.m_motionOpenSize = value;
    applySetting("motionOpenSize");
}

void CameraGUI::on_motionCloseSizeSpin_valueChanged(int value)
{
    m_settings.m_motionCloseSize = value;
    applySetting("motionCloseSize");
}

void CameraGUI::on_motionPersistenceFramesSpin_valueChanged(int value)
{
    m_settings.m_motionPersistenceFrames = value;
    applySetting("motionPersistenceFrames");
}

void CameraGUI::on_minContourAreaSpin_valueChanged(int value)
{
    m_settings.m_minContourArea = value;
    applySetting("minContourArea");
}

void CameraGUI::on_motionBoxColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_motionBoxColor, this, tr("Select bounding box colour"));

    if (color.isValid())
    {
        m_settings.m_motionBoxColor = color;
        updateColorButton(settingsUI()->motionBoxColorButton, color);
        applySetting("motionBoxColor");
    }
}

void CameraGUI::on_motionExclusionAddButton_clicked()
{
    if (m_lastImage.isNull()) {
        return;
    }

    setMotionExclusionDrawMode(true);
}

void CameraGUI::on_motionExclusionRemoveButton_clicked()
{
    const int row = settingsUI()->motionExclusionTable->currentRow();

    if ((row >= 0) && (row < m_settings.m_motionExclusionRects.size()))
    {
        m_settings.m_motionExclusionRects.removeAt(row);
        updateMotionExclusionRectsTable();
        applySetting("motionExclusionRects");
    }
}

void CameraGUI::on_motionExclusionShowButton_toggled(bool checked)
{
    m_showMotionExclusionRects = checked;
    updateMotionExclusionPreview();
}

void CameraGUI::on_motionExclusionTable_itemChanged(QTableWidgetItem *item)
{
    if (m_updatingMotionExclusionRectsTable || !item) {
        return;
    }

    applyMotionExclusionRectsFromTable();
    applySetting("motionExclusionRects");
}

void CameraGUI::on_spectrumOverlayButton_toggled(bool checked)
{
    m_settings.m_overlaySpectrum = checked;
    applySetting("overlaySpectrum");
}

void CameraGUI::on_spectrumDeviceCombo_currentIndexChanged(int index)
{
    m_settings.m_spectrumDevice = settingsUI()->spectrumDeviceCombo->itemText(index);
    applySetting("spectrumDevice");
}

void CameraGUI::on_spectrumOffsetXSlider_valueChanged(int value)
{
    m_settings.m_spectrumOffsetX = value;
    settingsUI()->spectrumOffsetXValue->setText(QString::number(value));
    applySetting("spectrumOffsetX");
}

void CameraGUI::on_spectrumOffsetYSlider_valueChanged(int value)
{
    m_settings.m_spectrumOffsetY = value;
    settingsUI()->spectrumOffsetYValue->setText(QString::number(value));
    applySetting("spectrumOffsetY");
}

void CameraGUI::on_spectrumScaleSpin_valueChanged(double value)
{
    m_settings.m_spectrumScale = value;
    applySetting("spectrumScale");
}

void CameraGUI::on_yoloButton_toggled(bool checked)
{
    m_settings.m_yoloEnabled = checked;
    applySetting("yoloEnabled");
}

void CameraGUI::applyYoloPathSetting(const QString& settingKey, const QString& path)
{
    if (settingKey == "yoloModelPath")
    {
        m_settings.m_yoloModelPath = path;
    }
    else if (settingKey == "yoloLabelsPath")
    {
        m_settings.m_yoloLabelsPath = path;
        populateActionClasses();
        rebuildActionTabsForCurrentClass();
    }
    else
    {
        return;
    }

    applySetting(settingKey);
}

void CameraGUI::requestYoloDownload(const QString& settingKey, const QString& path)
{
    if (!(path.startsWith("http://") || path.startsWith("https://")))
    {
        applyYoloPathSetting(settingKey, path);
        return;
    }

    QDir downloadDir(HttpDownloadManager::downloadDir());
    const QString destSubDir = QStringLiteral("onnx");
    if (!downloadDir.exists(destSubDir) && !downloadDir.mkdir(destSubDir))
    {
        QMessageBox::warning(this, tr("Download failed"),
            tr("Failed to create download directory: %1").arg(downloadDir.filePath(destSubDir)));
        return;
    }

    const QString localFilename = CameraSettings::urlToFilename(path, destSubDir);
    if (m_pendingYoloDownloads.contains(localFilename))
    {
        // Already downloading this file - just ignore the new request
        return;
    }
    m_pendingYoloDownloads.insert(localFilename, settingKey);

    if (QFileInfo::exists(localFilename))
    {
        handleYoloDownloadComplete(localFilename, true, path, QString());
        return;
    }

    m_dlm.download(QUrl(path), localFilename, this);
}

void CameraGUI::handleYoloDownloadComplete(const QString& filename, bool success, const QString& url, const QString& errorMessage)
{
    const QString settingKey = m_pendingYoloDownloads.take(filename);
    if (settingKey.isEmpty()) {
        return;
    }

    if (!success)
    {
        QString error = errorMessage;
        if (error.isEmpty())
        {
            error = QString("An unknown error occurred during download from %1 to %2.")
                .arg(url)
                .arg(filename);
        }
        QMessageBox::warning(this, tr("Download failed"), error);
        return;
    }

    QComboBox *combo = (settingKey == "yoloModelPath") ? settingsUI()->yoloModelPathCombo : settingsUI()->yoloLabelsPathCombo;
    const QSignalBlocker blocker(combo);
    combo->setCurrentText(filename);
    applyYoloPathSetting(settingKey, filename);
}

void CameraGUI::on_yoloModelPathCombo_currentIndexChanged(int index)
{
    if (index >= 0) {
        requestYoloDownload("yoloModelPath", settingsUI()->yoloModelPathCombo->itemText(index));
    }
}

void CameraGUI::on_yoloModelPathEdit_editingFinished()
{
    requestYoloDownload("yoloModelPath", settingsUI()->yoloModelPathCombo->currentText().trimmed());
}

void CameraGUI::on_yoloModelPathButton_clicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Select YOLO ONNX model"), m_settings.m_yoloModelPath,
        tr("ONNX model (*.onnx);;All files (*)"));

    if (!fileName.isEmpty())
    {
        settingsUI()->yoloModelPathCombo->setCurrentText(fileName);
        applyYoloPathSetting("yoloModelPath", fileName);
    }
}

void CameraGUI::on_yoloLabelsPathCombo_currentIndexChanged(int index)
{
    if (index >= 0) {
        requestYoloDownload("yoloLabelsPath", settingsUI()->yoloLabelsPathCombo->itemText(index));
    }
}

void CameraGUI::on_yoloLabelsPathEdit_editingFinished()
{
    requestYoloDownload("yoloLabelsPath", settingsUI()->yoloLabelsPathCombo->currentText().trimmed());
}

void CameraGUI::on_yoloLabelsPathButton_clicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Select class labels file"), m_settings.m_yoloLabelsPath,
        tr("Names file (*.names *.txt);;All files (*)"));

    if (!fileName.isEmpty())
    {
        settingsUI()->yoloLabelsPathCombo->setCurrentText(fileName);
        applyYoloPathSetting("yoloLabelsPath", fileName);
    }
}

void CameraGUI::on_yoloTargetCombo_currentIndexChanged(int index)
{
    m_settings.m_yoloDnnTarget = static_cast<CameraSettings::DNNTarget>(index);
    applySetting("yoloDnnTarget");
}

void CameraGUI::on_actionsClassCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    saveCurrentActionClassSettings();
    rebuildActionTabsForCurrentClass();
    applySetting("objectDeviceSettings");
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
    connect(deviceSettingsGUI, &CameraObjectDeviceSettingsGUI::settingsChanged, this, &CameraGUI::applyActionSettings);

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
    applySetting("yoloConfThreshold");
}

void CameraGUI::on_yoloNmsSpin_valueChanged(double value)
{
    m_settings.m_yoloNmsThreshold = value;
    applySetting("yoloNmsThreshold");
}

void CameraGUI::on_yoloBoxColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_yoloBoxColor, this, tr("Select bounding box colour"));

    if (color.isValid())
    {
        m_settings.m_yoloBoxColor = color;
        updateColorButton(settingsUI()->yoloBoxColorButton, color);
        applySetting("yoloBoxColor");
    }
}

bool CameraGUI::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->imageView->viewport())
    {
        if (event->type() == QEvent::Wheel)
        {
            const QWheelEvent *wheelEvent = static_cast<QWheelEvent*>(event);
            const double factor = (wheelEvent->angleDelta().y() > 0) ? 1.25 : 1.0 / 1.25;
            ui->imageView->scale(factor, factor);
            return true;
        }

        if (m_motionExclusionDrawMode)
        {
            if (event->type() == QEvent::MouseButtonPress)
            {
                const QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if (mouseEvent->button() == Qt::LeftButton)
                {
                    m_motionExclusionDragStartImagePos = mapViewportPointToImage(mouseEvent->pos());
                    if (m_motionExclusionDragStartImagePos.x() < 0) {
                        return true;
                    }

                    m_motionExclusionDragging = true;

                    if (!m_motionExclusionDragItem)
                    {
                        QPen pen(QColor(255, 255, 0));
                        pen.setWidth(2);
                        pen.setStyle(Qt::DashLine);
                        m_motionExclusionDragItem = m_imageScene->addRect(QRectF(), pen);
                        m_motionExclusionDragItem->setZValue(2.0);
                    }

                    m_motionExclusionDragItem->setRect(QRectF(
                        QPointF(m_motionExclusionDragStartImagePos),
                        QSizeF(1.0, 1.0)));
                    return true;
                }
                if (mouseEvent->button() == Qt::RightButton)
                {
                    setMotionExclusionDrawMode(false);
                    return true;
                }
            }
            else if (event->type() == QEvent::MouseMove)
            {
                const QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if (m_motionExclusionDragging && m_motionExclusionDragItem)
                {
                    const QPoint current = mapViewportPointToImage(mouseEvent->pos());
                    const QRect rect(
                        QPoint(std::min(m_motionExclusionDragStartImagePos.x(), current.x()),
                               std::min(m_motionExclusionDragStartImagePos.y(), current.y())),
                        QPoint(std::max(m_motionExclusionDragStartImagePos.x(), current.x()),
                               std::max(m_motionExclusionDragStartImagePos.y(), current.y())));
                    m_motionExclusionDragItem->setRect(QRectF(rect));
                    return true;
                }
            }
            else if (event->type() == QEvent::MouseButtonRelease)
            {
                const QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if (m_motionExclusionDragging && mouseEvent->button() == Qt::LeftButton)
                {
                    m_motionExclusionDragging = false;
                    const QPoint end = mapViewportPointToImage(mouseEvent->pos());
                    const int left = std::min(m_motionExclusionDragStartImagePos.x(), end.x());
                    const int top = std::min(m_motionExclusionDragStartImagePos.y(), end.y());
                    const int width = std::abs(end.x() - m_motionExclusionDragStartImagePos.x()) + 1;
                    const int height = std::abs(end.y() - m_motionExclusionDragStartImagePos.y()) + 1;

                    setMotionExclusionDrawMode(false);

                    if ((width > 1) && (height > 1))
                    {
                        m_settings.m_motionExclusionRects.append(QRect(left, top, width, height));
                        updateMotionExclusionRectsTable();
                        settingsUI()->motionExclusionTable->selectRow(settingsUI()->motionExclusionTable->rowCount() - 1);
                        applySetting("motionExclusionRects");
                    }

                    return true;
                }
            }
        }
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
    applySetting("audioMute");
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
        applySetting("audioDeviceName");
    }
}

void CameraGUI::on_whiteBalanceCombo_currentIndexChanged(int index)
{
    m_settings.m_whiteBalanceMode = settingsUI()->whiteBalanceCombo->itemData(index).toInt();
    applySetting("whiteBalanceMode");
}

void CameraGUI::on_exposureCompSpin_valueChanged(double value)
{
    m_settings.m_exposureCompensation = value;
    applySetting("exposureCompensation");
}

void CameraGUI::on_focusModeCombo_currentIndexChanged(int index)
{
    m_settings.m_focusMode = settingsUI()->focusModeCombo->itemData(index).toInt();
    updateCameraSettingsVisibility();
    applySetting("focusMode");
}

void CameraGUI::on_focusDistSpin_valueChanged(double value)
{
    m_settings.m_focusDistance = value;
    applySetting("focusDistance");
}

void CameraGUI::on_zoomSpin_valueChanged(double value)
{
    m_settings.m_zoomFactor = value;
    applySetting("zoomFactor");
}

void CameraGUI::on_cameraSettingsButton_clicked()
{
    m_settingsDialog->show();
    m_settingsDialog->raise();
    m_settingsDialog->activateWindow();
}

void CameraGUI::applyImageToolTip()
{
    switch (m_settings.m_recordMode)
    {
    case CameraSettings::SavedMediaRaw:
        ui->saveImageCheck->setToolTip(QString("Save raw images to %1").arg(m_settings.m_imageFileName));
        break;
    case CameraSettings::SavedMediaProcessed:
        ui->saveImageCheck->setToolTip(QString("Save processed images to %1").arg(m_settings.m_imageFileName));
        break;
    case CameraSettings::SavedMediaBoth:
        ui->saveImageCheck->setToolTip(QString("Save raw and processed images to %1").arg(m_settings.m_imageFileName));
        break;
    }
}

void CameraGUI::applyVideoToolTip()
{
    switch (m_settings.m_recordMode)
    {
    case CameraSettings::SavedMediaRaw:
        ui->saveVideoCheck->setToolTip(QString("Record raw video to %1").arg(m_settings.m_videoFileName));
        break;
    case CameraSettings::SavedMediaProcessed:
        ui->saveVideoCheck->setToolTip(QString("Record processed video to %1").arg(m_settings.m_videoFileName));
        break;
    case CameraSettings::SavedMediaBoth:
        ui->saveVideoCheck->setToolTip(QString("Record raw and processed video to %1").arg(m_settings.m_videoFileName));
        break;
    }
}

void CameraGUI::onSettingsDialogFinished(int result)
{
    Q_UNUSED(result)
    applyActionSettings();
}

void CameraGUI::updateStatus()
{
    if (!m_settings.m_rotator.isEmpty()) {
        syncFromSelectedGs232Controller();
    }

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

void CameraGUI::preferenceChanged(int elementType)
{
    const Preferences::ElementType pref = static_cast<Preferences::ElementType>(elementType);

    if (!m_settings.m_positionSync) {
        return;
    }

    if ((pref == Preferences::Latitude)
        || (pref == Preferences::Longitude)
        || (pref == Preferences::Altitude))
    {
        syncFromMainSettings();
    }
}

void CameraGUI::onFeatureAdded(int featureSetIndex, Feature *feature)
{
    (void) featureSetIndex;
    if (feature && (feature->getURI() == QLatin1String("sdrangel.feature.gs232controller"))) {
        populateGs232ControllerCombo();
        updatePositionControls();
    }
}

void CameraGUI::onFeatureRemoved(int featureSetIndex, Feature *feature)
{
    (void) featureSetIndex;
    if (feature && (feature->getURI() == QLatin1String("sdrangel.feature.gs232controller"))) {
        populateGs232ControllerCombo();
        if (settingsUI()->rotatorControllerCombo->findData(m_settings.m_rotator) < 0) {
            m_settings.m_rotator.clear();
            applySetting("rotator");
        }
        updatePositionControls();
    }
}

void CameraGUI::updateHardware()
{
    if (m_doApplySettings)
    {
        Camera::MsgConfigureCamera *msg = Camera::MsgConfigureCamera::create(m_settings, m_settingsKeys, m_forceSettings);
        m_camera->getInputMessageQueue()->push(msg);

        applyQtCameraSettings(m_settingsKeys, m_forceSettings);

        m_forceSettings = false;
        m_settingsKeys.clear();
        m_updateTimer.stop();
    }
}

QString CameraGUI::resolutionKey(const QSize& size)
{
    return QStringLiteral("%1x%2").arg(size.width()).arg(size.height());
}

QString CameraGUI::resolutionKey(int width, int height)
{
    return QStringLiteral("%1x%2").arg(width).arg(height);
}

int CameraGUI::decimalsForStepSize(double step)
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

int CameraGUI::doubleSpinBoxSliderMaximum(const QDoubleSpinBox *spinBox)
{
    const double step = std::max(0.000001, spinBox->singleStep());
    return std::max(0, static_cast<int>(std::llround((spinBox->maximum() - spinBox->minimum()) / step)));
}

int CameraGUI::doubleSpinBoxValueToSlider(const QDoubleSpinBox *spinBox, double value)
{
    const double step = std::max(0.000001, spinBox->singleStep());
    const int sliderValue = static_cast<int>(std::llround((value - spinBox->minimum()) / step));
    return qBound(0, sliderValue, doubleSpinBoxSliderMaximum(spinBox));
}

double CameraGUI::sliderValueToDoubleSpinBox(const QDoubleSpinBox *spinBox, int sliderValue)
{
    const double step = std::max(0.000001, spinBox->singleStep());
    return qBound(spinBox->minimum(), spinBox->minimum() + (sliderValue * step), spinBox->maximum());
}

int CameraGUI::exposureValueToSlider(const QDoubleSpinBox *spinBox, double value)
{
    const double minimum = std::max(0.000001, spinBox->minimum());
    const double maximum = std::max(minimum, spinBox->maximum());
    const int sliderMaximum = 1000;

    if (maximum <= minimum) {
        return 0;
    }

    const double clampedValue = qBound(minimum, value, maximum);
    const double normalized = (std::log(clampedValue) - std::log(minimum)) / (std::log(maximum) - std::log(minimum));
    return qBound(0, static_cast<int>(std::lround(normalized * sliderMaximum)), sliderMaximum);
}

double CameraGUI::sliderToExposureValue(const QDoubleSpinBox *spinBox, int sliderValue)
{
    const double minimum = std::max(0.000001, spinBox->minimum());
    const double maximum = std::max(minimum, spinBox->maximum());
    const int sliderMaximum = 1000;

    if (maximum <= minimum) {
        return minimum;
    }

    const double normalized = qBound(0.0, static_cast<double>(sliderValue) / sliderMaximum, 1.0);
    const double value = std::exp(std::log(minimum) + normalized * (std::log(maximum) - std::log(minimum)));
    return qBound(minimum, value, maximum);
}

double CameraGUI::currentExposureUnitScaleMs(const Ui::CameraSettingsDialog *ui)
{
    const QVariant data = ui->exposureUnitsCombo->currentData();
    return data.isValid() ? data.toDouble() : 1.0;
}

void CameraGUI::appendFpsRange(QSet<int>& fpsValues, qreal minFps, qreal maxFps)
{
    const int minRounded = qMax(1, static_cast<int>(std::ceil(minFps)));
    const int maxRounded = qMax(minRounded, static_cast<int>(std::floor(maxFps)));

    for (int fps = minRounded; fps <= maxRounded; ++fps) {
        fpsValues.insert(fps);
    }
}
