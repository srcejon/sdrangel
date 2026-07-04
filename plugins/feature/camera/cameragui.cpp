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
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QColorDialog>
#include <QComboBox>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QDesktopServices>
#include <QDial>
#include <QDialog>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QInputDialog>
#include <QLabel>
#include <QMenu>
#include <QMouseEvent>
#include <QLineEdit>
#include <QNetworkReply>
#include <QPainter>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QProgressDialog>
#include <QPushButton>
#include <QRegularExpression>
#include <QSet>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QTimer>
#include <QWheelEvent>
#include <QMessageBox>
#include <QUrl>
#include <QUrlQuery>
#include <QVBoxLayout>
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCamera>
#include <QCameraDevice>
#include <QCameraFormat>
#include <QImageCapture>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QOpenGLWidget>
#include <QVideoFrame>
#include <QVideoFrameFormat>
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
#ifdef QT_SENSORS_FOUND
#include <QCompass>
#include <QCompassReading>
#include <QRotationSensor>
#include <QRotationReading>
#include <QTiltSensor>
#include <QTiltReading>
#endif

#include "feature/featureuiset.h"
#include "gui/crightclickenabler.h"
#include "gui/audioselectdialog.h"
#include "gui/basicfeaturesettingsdialog.h"
#include "gui/dialogpositioner.h"
#include "dsp/dspengine.h"
#include "maincore.h"
#include "feature/featureset.h"
#include "channel/channelwebapiutils.h"
#include "feature/featurewebapiutils.h"
#include "pipes/objectpipe.h"

#include "cameraplatesolver.h"
#include "ui_cameragui.h"
#include "camera.h"
#include "cameradetectionhistory.h"
#include "cameraframestacker.h"
#include "camerahistogramdialog.h"
#include "camerafilesequencedialog.h"
#include "camerarecorder.h"
#include "camerasettingsdialog.h"
#include "cameraworker.h"
#include "cameragui.h"

namespace {

enum AutoExposureGainControl
{
    AutoExposureGainNone = 0,
    AutoExposureGainSoftware,
    AutoExposureGainHardware
};

const QString kDefaultDirectionSensorId = QStringLiteral("__default__");
const QString kDirectionSourceRotatorPrefix = QStringLiteral("rotator:");
const QString kDirectionSourceSensorPrefix = QStringLiteral("sensor:");

QString rotatorDirectionSourceId(const QString& rotatorId)
{
    return kDirectionSourceRotatorPrefix + rotatorId;
}

QString sensorDirectionSourceId(const QString& sensorId)
{
    return kDirectionSourceSensorPrefix + sensorId;
}

bool directionSourceIsRotator(const QString& sourceId)
{
    return sourceId.startsWith(kDirectionSourceRotatorPrefix);
}

bool directionSourceIsSensor(const QString& sourceId)
{
    return sourceId.startsWith(kDirectionSourceSensorPrefix);
}

QString directionSourceValue(const QString& sourceId, const QString& prefix)
{
    return sourceId.mid(prefix.size());
}

double normalizeSignedDegrees(double value)
{
    value = std::fmod(value, 360.0);
    if (value <= -180.0) {
        value += 360.0;
    } else if (value > 180.0) {
        value -= 360.0;
    }
    return value;
}

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
int nearestCameraFormatFps(const QCameraFormat& format, int desiredFps)
{
    int minFps = static_cast<int>(std::ceil(format.minFrameRate()));
    int maxFps = static_cast<int>(std::floor(format.maxFrameRate()));

    if (minFps <= 0) {
        minFps = 1;
    }
    if (maxFps < minFps) {
        maxFps = minFps;
    }

    return qBound(minFps, desiredFps, maxFps);
}

int cameraFormatFpsDistance(const QCameraFormat& format, int desiredFps)
{
    return std::abs(nearestCameraFormatFps(format, desiredFps) - desiredFps);
}
#endif

std::array<QLabel*, 4> hdrExposureLabels(Ui::CameraSettingsDialog *ui)
{
    return {{
        ui->stackHdrExposure1Label,
        ui->stackHdrExposure2Label,
        ui->stackHdrExposure3Label,
        ui->stackHdrExposure4Label
    }};
}

std::array<QSlider*, 4> hdrExposureSliders(Ui::CameraSettingsDialog *ui)
{
    return {{
        ui->stackHdrExposure1Slider,
        ui->stackHdrExposure2Slider,
        ui->stackHdrExposure3Slider,
        ui->stackHdrExposure4Slider
    }};
}

std::array<QDoubleSpinBox*, 4> hdrExposureSpins(Ui::CameraSettingsDialog *ui)
{
    return {{
        ui->stackHdrExposure1Spin,
        ui->stackHdrExposure2Spin,
        ui->stackHdrExposure3Spin,
        ui->stackHdrExposure4Spin
    }};
}

void setVisibleEnabled(QWidget *widget, bool visible, bool enabled)
{
    widget->setVisible(visible);
    widget->setEnabled(enabled);
}

struct BitratePreset
{
    const char *m_label;
    int m_kbps;
};

const std::array<BitratePreset, 8>& bitratePresets()
{
    static const std::array<BitratePreset, 8> presets = {{
        {"720p 30FPS - 5 Mbps", 5000},
        {"720p 60FPS - 7.5 Mbps", 7500},
        {"1080p 30FPS - 8 Mbps", 8000},
        {"1080p 60FPS - 12 Mbps", 12000},
        {"4K 30FPS - 35 Mbps", 35000},
        {"4K 30FPS HDR - 45 Mbps", 45000},
        {"4K 60FPS - 53 Mbps", 53000},
        {"4K 60FPS HDR - 68 Mbps", 68000}
    }};
    return presets;
}

QString bitrateText(int kbps)
{
    if ((kbps % 1000) == 0) {
        return QStringLiteral("%1 Mbps").arg(kbps / 1000);
    }
    return QStringLiteral("%1 Mbps").arg(kbps / 1000.0, 0, 'f', 1);
}

int parseBitrateKbps(const QString& text, int fallbackKbps)
{
    const QString trimmed = text.trimmed();
    const QString bitrateText = trimmed.contains(QLatin1Char('-'))
        ? trimmed.mid(trimmed.lastIndexOf(QLatin1Char('-')) + 1).trimmed()
        : trimmed;
    const QRegularExpressionMatch match = QRegularExpression(QStringLiteral("([0-9]+(?:[\\.,][0-9]+)?)")).match(bitrateText);
    if (!match.hasMatch()) {
        return fallbackKbps;
    }

    bool ok = false;
    const double value = QString(match.captured(1)).replace(QLatin1Char(','), QLatin1Char('.')).toDouble(&ok);
    if (!ok || (value <= 0.0)) {
        return fallbackKbps;
    }

    const QString lower = bitrateText.toLower();
    const bool explicitKbps = lower.contains(QStringLiteral("kbps")) || lower.contains(QStringLiteral("kbit"));
    const bool explicitMbps = lower.contains(QStringLiteral("mbps")) || lower.contains(QStringLiteral("mbit"));
    const bool assumeMbps = !explicitKbps && !explicitMbps && (value < 1000.0);
    const int kbps = static_cast<int>((explicitMbps || assumeMbps ? value * 1000.0 : value) + 0.5);
    return qBound(100, kbps, 240000);
}

void populateBitratePresetCombo(QComboBox *combo, bool clear = true)
{
    if (clear) {
        combo->clear();
    }

    for (const BitratePreset& preset : bitratePresets()) {
        combo->addItem(QString::fromLatin1(preset.m_label), preset.m_kbps);
    }

    combo->setInsertPolicy(QComboBox::NoInsert);
}

QString videoRecordBitrateText(int kbps)
{
    return kbps > 0 ? bitrateText(kbps) : QStringLiteral("Auto");
}

int parseVideoRecordBitrateKbps(const QString& text, int fallbackKbps)
{
    const QString trimmed = text.trimmed();
    if (trimmed.isEmpty() || (trimmed.compare(QStringLiteral("auto"), Qt::CaseInsensitive) == 0)) {
        return 0;
    }

    return parseBitrateKbps(trimmed, fallbackKbps > 0 ? fallbackKbps : 8000);
}

QDateTime captureDateTimeFromFileName(const QString& fileName)
{
    static const QRegularExpression dateTimeRe(
        QStringLiteral("(\\d{4}-\\d{2}-\\d{2})T(\\d{2})[_:](\\d{2})[_:](\\d{2})(?:[_.](\\d{1,3}))?"));
    const QRegularExpressionMatch match = dateTimeRe.match(QFileInfo(fileName).fileName());
    if (!match.hasMatch()) {
        return QDateTime();
    }

    const QDate date = QDate::fromString(match.captured(1), Qt::ISODate);
    QString milliseconds = match.captured(5);
    while (!milliseconds.isEmpty() && milliseconds.size() < 3) {
        milliseconds.append(QLatin1Char('0'));
    }

    const QTime time(
        match.captured(2).toInt(),
        match.captured(3).toInt(),
        match.captured(4).toInt(),
        milliseconds.left(3).toInt());
    const QDateTime dateTime(date, time, Qt::UTC);
    return dateTime.isValid() ? dateTime : QDateTime();
}

bool isSyntheticGaiaCatalogLabel(const QString& label)
{
    const QString trimmed = label.trimmed();
    return trimmed.startsWith(QStringLiteral("Gaia Astro "), Qt::CaseInsensitive)
        || trimmed.startsWith(QStringLiteral("Gaia SPCC "), Qt::CaseInsensitive)
        || trimmed.startsWith(QStringLiteral("Gaia J"), Qt::CaseInsensitive);
}

bool hasCatalogCoordinates(const CameraPipelineStarDetection& star)
{
    return star.m_solved
        && std::isfinite(star.m_catalogRightAscensionDegrees)
        && std::isfinite(star.m_catalogDeclinationDegrees);
}

QUrl simbadUrlForStarDetection(const CameraPipelineStarDetection& star, const QString& target)
{
    if (isSyntheticGaiaCatalogLabel(target) && hasCatalogCoordinates(star))
    {
        QUrl url(QStringLiteral("https://simbad.cds.unistra.fr/simbad/sim-coo"));
        QUrlQuery query;
        query.addQueryItem(
            QStringLiteral("Coord"),
            QStringLiteral("%1 %2")
                .arg(star.m_catalogRightAscensionDegrees, 0, 'f', 8)
                .arg(star.m_catalogDeclinationDegrees, 0, 'f', 8));
        query.addQueryItem(QStringLiteral("Radius"), QStringLiteral("5"));
        query.addQueryItem(QStringLiteral("Radius.unit"), QStringLiteral("arcsec"));
        url.setQuery(query);
        return url;
    }

    if (target.trimmed().isEmpty()) {
        return QUrl();
    }

    QUrl url(QStringLiteral("https://simbad.cds.unistra.fr/simbad/sim-id"));
    QUrlQuery query;
    query.addQueryItem(QStringLiteral("Ident"), target);
    url.setQuery(query);
    return url;
}

int discardQueuedReportFrames(MessageQueue& queue, bool requireStartStop)
{
    QList<Message*> messages;
    Message *message = nullptr;
    bool hasStartStop = false;

    while ((message = queue.pop()) != nullptr)
    {
        hasStartStop = hasStartStop || Camera::MsgStartStop::match(*message);
        messages.append(message);
    }

    int dropped = 0;

    for (Message *queuedMessage : messages)
    {
        if ((!requireStartStop || hasStartStop) && CameraPostProcessor::MsgReportFrame::match(*queuedMessage))
        {
            delete queuedMessage;
            ++dropped;
        }
        else
        {
            queue.push(queuedMessage, false);
        }
    }

    return dropped;
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
        m_captureActive = cfg.getStartStop();
        m_captureEpoch = cfg.getCaptureEpoch();
        m_displayedMotionEventActive = false;
        m_displayedObjectEventClasses.clear();
        m_displayedTrackedObjectsInView.clear();
        discardQueuedReportFrames(*getInputMessageQueue(), false);

        if (!sameCameraIdentity(previousCamera, selectedCameraFromSettings())) {
            resetCameraStatus();
        }

        if (cfg.getStartStop())
        {
            if (m_settings.isQtCamera() || m_settings.isImageFileSequenceCamera()) {
                setupQtCapture();
            }
        }
        else
        {
            cleanupQtCapture();
        }

        return true;
    }
    else if (CameraWorker::MsgReportVideoFilePlayback::match(message))
    {
        const CameraWorker::MsgReportVideoFilePlayback& report = (const CameraWorker::MsgReportVideoFilePlayback&) message;
        m_playbackDurationMs = report.getDurationMs();
        m_videoFileVideoPositionMs = report.getPositionMs();
        updatePlaybackPositionLabel(m_videoFileVideoPositionMs);

        if ((m_playbackDurationMs > 0) && !ui->playbackPositionSlider->isSliderDown())
        {
            const int sliderValue = static_cast<int>(
                qBound<qint64>(0,
                    (m_videoFileVideoPositionMs * PlaybackPositionSliderMaximum) / m_playbackDurationMs,
                    static_cast<qint64>(PlaybackPositionSliderMaximum)));
            QSignalBlocker blocker(ui->playbackPositionSlider);
            ui->playbackPositionSlider->setValue(sliderValue);
        }

        {
            QSignalBlocker blocker(ui->playPauseVideo);
            ui->playPauseVideo->setChecked(report.isPlaying());
        }
        updateVideoFileControls();
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
                : (CameraProtocol::isPlaybackSource(camera.m_protocol)
                    ? CameraProtocol::playbackDisplayText(camera.m_protocol, camera.m_description)
                    : QString("%1:%2").arg(camera.m_protocol, camera.m_description));
            displayCounts[displayKey] = displayCounts.value(displayKey) + 1;
        }

        ui->cameraCombo->blockSignals(true);
        ui->cameraCombo->clear();
        for (const CameraInfo& entry : entries)
        {
            QString displayText = entry.m_protocol.isEmpty()
                ? entry.m_id
                : (CameraProtocol::isPlaybackSource(entry.m_protocol)
                    ? CameraProtocol::playbackDisplayText(entry.m_protocol, entry.m_description)
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
        updateVideoFileControls();

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
        updateCameraSettingsVisibility();

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
        if (!report.isManualPreviewFrame() && (!m_captureActive || (report.getCaptureEpoch() != m_captureEpoch))) {
            return true;
        }

        // Send events first, to make Scheduler response as fast as possible
        sendDisplayedFrameEvents(report.getMotionBoxes(), report.getDetections(), report.getMeteorPhotometry(), report.getTrackedObjects(), report.getImage().size(), report.getCaptureDateTime());

        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
        QSize oldSize = m_lastImage.size();
        m_lastImage = report.getImage();
        m_lastHistogramData = report.getHistogramData();
        m_lastStarDetections = report.getStarDetections();
        m_lastPreviewTextLabels = report.getPreviewTextLabels();
        m_lastPreviewRectItems = report.getPreviewRectItems();
        m_lastStackCount = report.getStackCount();
        m_lastStackQueuedCount = report.getStackQueuedCount();
        m_lastStackDroppedCount = report.getStackDroppedCount();
        m_lastStackRejectedCount = report.getStackRejectedCount();
        settingsUI()->stackCurrentCountValue->setText(tr("%1 / %2 / %3 / %4")
            .arg(m_lastStackCount)
            .arg(m_lastStackQueuedCount)
            .arg(m_lastStackDroppedCount)
            .arg(m_lastStackRejectedCount));
        const QString stackStatusToolTip = report.getStackRejectReason().isEmpty()
            ? tr("Stacked / queued / dropped / rejected")
            : tr("Stacked / queued / dropped / rejected\nLast rejected frame: %1").arg(report.getStackRejectReason());
        settingsUI()->stackCurrentCountValue->setToolTip(stackStatusToolTip);
        settingsUI()->stackDisplayFrameSpin->setMaximum(std::max(1, m_lastStackCount));
        settingsUI()->stackDeleteFrameButton->setEnabled(m_lastStackCount > 0);
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
        updateVideoPreRecordBufferMemoryLabel();
        updateScaleControls();
        m_lastPlateSolved = report.isPlateSolved();
        m_lastPlateSolvedMatches = report.getPlateSolvedMatches();
        m_lastPlateSolveDetectedStarsConsidered = report.getPlateSolveDetectedStarsConsidered();
        m_lastPlateSolveCatalogStarsLoaded = report.getPlateSolveCatalogStarsLoaded();
        m_lastPlateSolveCatalogCandidateStars = report.getPlateSolveCatalogCandidateStars();
        m_lastPlateSolveOutlierStars = report.getPlateSolveOutlierStars();
        m_lastPlateSolveRmsError = report.getPlateSolveRmsError();
        m_lastPlateSolveMaxError = report.getPlateSolveMaxError();
        m_lastPlateSolveAzimuth = report.getPlateSolveAzimuth();
        m_lastPlateSolveElevation = report.getPlateSolveElevation();
        m_lastPlateSolveRoll = report.getPlateSolveRoll();
        m_lastPlateSolveFov = report.getPlateSolveFov();
        m_lastPlateSolveCenterOffsetX = report.getPlateSolveCenterOffsetX();
        m_lastPlateSolveCenterOffsetY = report.getPlateSolveCenterOffsetY();
        m_lastPlateSolveDistortionK1 = report.getPlateSolveDistortionK1();
        m_lastPlateSolveCatalogSource = report.getPlateSolveCatalogSource();
        if (m_lastPlateSolved)
        {
            settingsUI()->plateSolveStatusLabel->setText(
                tr("Az %1  El %2  Roll %3  FoV %4  Cx %5  Cy %6  K1 %7")
                .arg(QString::number(m_lastPlateSolveAzimuth, 'f', 2))
                .arg(QString::number(m_lastPlateSolveElevation, 'f', 2))
                .arg(QString::number(m_lastPlateSolveRoll, 'f', 2))
                .arg(QString::number(m_lastPlateSolveFov, 'f', 2))
                .arg(QString::number(m_lastPlateSolveCenterOffsetX, 'f', 1))
                .arg(QString::number(m_lastPlateSolveCenterOffsetY, 'f', 1))
                .arg(QString::number(m_lastPlateSolveDistortionK1, 'f', 3)));
        }
        else
        {
            settingsUI()->plateSolveStatusLabel->setText(tr("Unsolved"));
        }
        settingsUI()->plateSolveMatchesLabel->setText(
            m_lastPlateSolved ? QString::number(m_lastPlateSolvedMatches) : "-");
        settingsUI()->plateSolveDetectedLabel->setText(
            m_lastPlateSolved ? QString::number(m_lastPlateSolveDetectedStarsConsidered) : "-");
        settingsUI()->plateSolveRmsLabel->setText(
            m_lastPlateSolved ? tr("%1 / %2").arg(QString::number(m_lastPlateSolveRmsError, 'f', 1)).arg(QString::number(m_lastPlateSolveMaxError, 'f', 1)) : "-");

        settingsUI()->plateSolveApplyButton->setEnabled(m_lastPlateSolved);
        updateImageWidget();
        if (m_histogramDialog) {
            m_histogramDialog->updateHistogram(m_lastHistogramData);
        }
        // When the image size changes, refit to view - but only if the user
        // hasn't manually zoomed/panned (matching updateImageWidget()).
        if ((oldSize != m_lastImage.size()) && ui->imageView->transform().isIdentity()) {
            ui->imageView->fitInView(m_imagePixmapItem, Qt::KeepAspectRatio);
        }
        return true;
    }
    else if (CameraObjectDetector::MsgReportObjectDetectionHistory::match(message))
    {
        const CameraObjectDetector::MsgReportObjectDetectionHistory& report = (CameraObjectDetector::MsgReportObjectDetectionHistory&) message;
        m_detectionHistory = report.getHistory();
        if (m_detectionHistoryDialog) {
            m_detectionHistoryDialog->updateHistory(m_detectionHistory);
        }
        return true;
    }
    else if (CameraStarDetector::MsgReportPlateSolveStatus::match(message))
    {
        const CameraStarDetector::MsgReportPlateSolveStatus& report = (const CameraStarDetector::MsgReportPlateSolveStatus&) message;
        if (report.isSolving()) {
            settingsUI()->plateSolveStatusLabel->setText(tr("Solving"));
        }
        return true;
    }
    else if (CameraObjectDetector::MsgReportTensorRtConversion::match(message))
    {
        const CameraObjectDetector::MsgReportTensorRtConversion& report = (CameraObjectDetector::MsgReportTensorRtConversion&) message;
        if (report.isActive())
        {
            if (!m_tensorRtProgressDialog)
            {
                m_tensorRtProgressDialog = new QProgressDialog(this);
                m_tensorRtProgressDialog->setWindowTitle(tr("TensorRT conversion"));
                m_tensorRtProgressDialog->setCancelButton(nullptr);
                m_tensorRtProgressDialog->setWindowModality(Qt::NonModal);
                m_tensorRtProgressDialog->setMinimumDuration(0);
                m_tensorRtProgressDialog->setRange(0, 0);
                m_tensorRtProgressDialog->setAttribute(Qt::WA_DeleteOnClose, false);
            }

            const QFileInfo modelInfo(report.getModelPath());
            const QFileInfo engineInfo(report.getEnginePath());
            m_tensorRtProgressDialog->setLabelText(tr("Converting YOLO model to TensorRT engine...\n%1\n\nEngine:\n%2")
                .arg(modelInfo.fileName().isEmpty() ? report.getModelPath() : modelInfo.fileName())
                .arg(engineInfo.fileName().isEmpty() ? report.getEnginePath() : engineInfo.fileName()));
            m_tensorRtProgressDialog->show();
            m_tensorRtProgressDialog->raise();
        }
        else if (m_tensorRtProgressDialog)
        {
            m_tensorRtProgressDialog->hide();
        }
        return true;
    }
    else if (CameraRecorder::MsgReportSaveVideoState::match(message))
    {
        const CameraRecorder::MsgReportSaveVideoState& report = (CameraRecorder::MsgReportSaveVideoState&) message;
        m_settings.m_saveVideo = report.getSaveVideo();
        ui->saveVideoCheck->blockSignals(true);
        ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
        ui->saveVideoCheck->blockSignals(false);
        applySetting("saveVideo");
        return true;
    }
    else if (CameraRecorder::MsgReportSaveImageState::match(message))
    {
        const CameraRecorder::MsgReportSaveImageState& report = (CameraRecorder::MsgReportSaveImageState&) message;
        m_settings.m_saveImage = report.getSaveImage();
        ui->saveImageCheck->blockSignals(true);
        ui->saveImageCheck->setChecked(m_settings.m_saveImage);
        ui->saveImageCheck->blockSignals(false);
        applySetting("saveImage");
        return true;
    }
    else if (CameraRecorder::MsgReportKeogram::match(message))
    {
        const CameraRecorder::MsgReportKeogram& report = (CameraRecorder::MsgReportKeogram&) message;
        updateKeogramPreview(report.getImage(), report.getFileName(), report.getVisible());
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
    else if (CameraWorker::MsgReportAutoExposureGain::match(message))
    {
        const CameraWorker::MsgReportAutoExposureGain& report = (CameraWorker::MsgReportAutoExposureGain&) message;
        m_settings.m_exposureTimeMs = report.getExposureTimeMs();
        m_settings.m_cameraGain = report.getGain();

        {
            QSignalBlocker blocker(settingsUI()->exposureSpin);
            QSignalBlocker blocker2(settingsUI()->exposureSlider);
            updateExposureControls();
        }

        if (m_alpacaHasNamedGains)
        {
            QSignalBlocker blocker(settingsUI()->cameraGainCombo);
            if (settingsUI()->cameraGainCombo->count() > 0) {
                settingsUI()->cameraGainCombo->setCurrentIndex(qBound(0, report.getGain(), settingsUI()->cameraGainCombo->count() - 1));
            }
        }
        else
        {
            QSignalBlocker blocker1(settingsUI()->cameraGainSpin);
            QSignalBlocker blocker2(settingsUI()->cameraGainSlider);
            settingsUI()->cameraGainSpin->setValue(report.getGain());
            settingsUI()->cameraGainSlider->setValue(report.getGain());
        }

        settingsUI()->autoExposureGainCombo->setToolTip(tr("Measured %1%, saturated %2%")
            .arg(QString::number(report.getMeasuredBrightness() * 100.0, 'f', 1))
            .arg(QString::number(report.getSaturatedFraction() * 100.0, 'f', 2)));
        return true;
    }
    else if (CameraWorker::MsgReportAutoFocus::match(message))
    {
        const CameraWorker::MsgReportAutoFocus& report = (CameraWorker::MsgReportAutoFocus&) message;
        if (report.getPosition() >= 0)
        {
            m_settings.m_alpacaFocusPosition = report.getPosition();
            QSignalBlocker blocker(settingsUI()->alpacaFocusPositionSpin);
            settingsUI()->alpacaFocusPositionSpin->setValue(report.getPosition());
        }

        QString status = report.getStatus();
        if (report.getStepCount() > 0) {
            status = tr("%1 %2/%3 score %4")
                .arg(status)
                .arg(report.getStepIndex())
                .arg(report.getStepCount())
                .arg(QString::number(report.getScore(), 'f', 1));
        } else if (report.getScore() > 0.0) {
            status = tr("%1 score %2").arg(status).arg(QString::number(report.getScore(), 'f', 1));
        }
        settingsUI()->alpacaAutoFocusStatusLabel->setText(status);
        settingsUI()->alpacaAutoFocusButton->setEnabled(!report.isActive());
        if (!report.isActive() && (report.getPosition() >= 0) && (report.getScore() > 0.0)) {
            applySetting("alpacaFocusPosition");
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

    discardQueuedReportFrames(*getInputMessageQueue(), true);

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
    m_lastFeatureState(0),
    m_dlm(this),
    m_lastAlpacaCameraState(-1),
    m_lastAlpacaCaptureTimeMs(-1),
    m_lastAlpacaReceiveImageFormat(),
    m_lastAlpacaCcdTemperature(0.0),
    m_lastAlpacaCcdTemperatureValid(false),
    m_lastAlpacaErrorNumber(0),
    m_lastAlpacaErrorMessage(),
    m_settingsDialog(nullptr),
    m_detectionHistoryDialog(nullptr),
    m_histogramDialog(nullptr),
    m_alpacaHasNamedGains(false),
    m_alpacaHasNamedOffsets(false),
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
    m_asiRaw8Supported(false),
    m_imageScene(nullptr),
    m_imagePixmapItem(nullptr),
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    m_qtCamera(nullptr),
    m_imageCapture(nullptr),
    m_videoSink(nullptr),
    m_captureSession(nullptr),
#else
    m_qtCamera(nullptr),
    m_imageCapture(nullptr),
    m_videoSurface(nullptr),
#endif
    m_imageSequenceTimer(this)
{
    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/camera/readme.md";

    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    rollupContents->arrangeRollups();
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));

    // Set up the QGraphicsView for camera preview
    m_imageScene = new QGraphicsScene(this);
    m_imagePixmapItem = new CameraImageGraphicsItem();
    m_imageScene->addItem(m_imagePixmapItem);
    ui->imageView->setScene(m_imageScene);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    // Render the preview through an OpenGL viewport so scaling the (up to 4K)
    // pixmap down to the widget runs on the GPU. The default raster viewport
    // smooth-scales the whole frame on the CPU every frame, which was ~40% of the
    // per-frame page-fault churn on HD/4K sources — the dominant display cost on
    // slower machines. Must precede the viewport()->installEventFilter() below so
    // the filter (wheel-zoom / click-inspect) lands on the new GL viewport.
    ui->imageView->setViewport(new QOpenGLWidget());
    ui->imageView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
#endif
    ui->imageView->setDragMode(QGraphicsView::ScrollHandDrag);
    ui->imageView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    ui->imageView->setRenderHint(QPainter::Antialiasing, true);
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
    initialiseVideoRecordBitrateCombo();
    initialiseYouTubeBitrateCombo();

#ifndef CAMERA_TENSORRT_YOLO
    if (QStandardItemModel *yoloTargetModel = qobject_cast<QStandardItemModel*>(settingsUI()->yoloTargetCombo->model()))
    {
        for (int target : {static_cast<int>(CameraSettings::TensorRT), static_cast<int>(CameraSettings::TensorRT_FP16)})
        {
            if (QStandardItem *item = yoloTargetModel->item(target))
            {
                item->setEnabled(false);
                item->setToolTip(tr("TensorRT support is not available in this build"));
            }
        }
    }
#endif

    settingsUI()->azimuthSpin->setRange(
        static_cast<double>(CameraSettings::m_minAzimuth),
        static_cast<double>(CameraSettings::m_maxAzimuth));
    settingsUI()->elevationSpin->setRange(
        static_cast<double>(CameraSettings::m_minElevation),
        static_cast<double>(CameraSettings::m_maxElevation));
    settingsUI()->rollSpin->setRange(
        static_cast<double>(CameraSettings::m_minRoll),
        static_cast<double>(CameraSettings::m_maxRoll));
    settingsUI()->fovSpin->setSuffix(QString::fromUtf8(" \xC2\xB0"));
    settingsUI()->fovSpin->setRange(
        static_cast<double>(CameraSettings::m_minFov),
        static_cast<double>(CameraSettings::m_maxFov));
    settingsUI()->lensCenterOffsetXSpin->setRange(
        CameraSettings::m_minLensCenterOffset,
        CameraSettings::m_maxLensCenterOffset);
    settingsUI()->lensCenterOffsetYSpin->setRange(
        CameraSettings::m_minLensCenterOffset,
        CameraSettings::m_maxLensCenterOffset);
    settingsUI()->lensDistortionK1Spin->setRange(
        CameraSettings::m_minLensDistortionK1,
        CameraSettings::m_maxLensDistortionK1);

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
    connect(&m_imageSequenceTimer, &QTimer::timeout, this, &CameraGUI::advanceImageSequenceFrame);
    connect(m_settingsDialog, &QDialog::finished, this, &CameraGUI::onSettingsDialogFinished);
    connect(&m_qtStillCaptureTimer, &QTimer::timeout, this, &CameraGUI::triggerQtStillCapture);
    connect(&m_dlm, &HttpDownloadManagerGUI::downloadComplete, this, &CameraGUI::handleYoloDownloadComplete);
    connect(&m_dlm, &HttpDownloadManagerGUI::downloadComplete, this, &CameraGUI::handlePlateSolveCatalogDownloadComplete);
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
    settingsUI()->autoExposureGainCombo->addItem(tr("None"), AutoExposureGainNone);
    settingsUI()->autoExposureGainCombo->addItem(tr("S/W"), AutoExposureGainSoftware);
    settingsUI()->autoExposureGainCombo->addItem(tr("H/W"), AutoExposureGainHardware);
    settingsUI()->autoExposureGainModeCombo->addItem(tr("Exposure first"), CameraSettings::AutoExposureGainExposureFirst);
    settingsUI()->autoExposureGainModeCombo->addItem(tr("Gain first"), CameraSettings::AutoExposureGainGainFirst);
    settingsUI()->autoExposureGainModeCombo->addItem(tr("Exposure only"), CameraSettings::AutoExposureGainExposureOnly);
    settingsUI()->autoExposureGainModeCombo->addItem(tr("Gain only"), CameraSettings::AutoExposureGainGainOnly);
    m_statusTimer.start(250);

    connect(&m_updateTimer, &QTimer::timeout, this, &CameraGUI::updateHardware);

    displaySettings();
    applyAllSettings();
    makeUIConnections();
    m_resizer.enableChildMouseTracking();

    m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create());
}

CameraGUI::~CameraGUI()
{
    if (m_camera) {
        m_camera->setMessageQueueToGUI(nullptr);
    }
    stopDirectionSensors();
    cleanupQtCapture();
    if (m_histogramDialog)
    {
        disconnect(m_histogramDialog, nullptr, this, nullptr);
        delete m_histogramDialog;
        m_histogramDialog = nullptr;
    }
    delete m_tensorRtProgressDialog;
    m_tensorRtProgressDialog = nullptr;
    delete ui;
}

void CameraGUI::setWorkspaceIndex(int index)
{
    m_settings.m_workspaceIndex = index;
    m_feature->setWorkspaceIndex(index);
}

void CameraGUI::onMenuDialogCalled(const QPoint &p)
{
    if (m_contextMenuType == ContextMenuChannelSettings)
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

        if (dialog.hasChanged())
        {
            m_settings.m_title = dialog.getTitle();
            m_settings.m_useReverseAPI = dialog.useReverseAPI();
            m_settings.m_reverseAPIAddress = dialog.getReverseAPIAddress();
            m_settings.m_reverseAPIPort = dialog.getReverseAPIPort();
            m_settings.m_reverseAPIFeatureSetIndex = dialog.getReverseAPIFeatureSetIndex();
            m_settings.m_reverseAPIFeatureIndex = dialog.getReverseAPIFeatureIndex();

            setTitle(m_settings.m_title);
            setWindowTitle(m_settings.m_title);

            applySettings(QStringList({
                QStringLiteral("title"),
                QStringLiteral("useReverseAPI"),
                QStringLiteral("reverseAPIAddress"),
                QStringLiteral("reverseAPIPort"),
                QStringLiteral("reverseAPIFeatureSetIndex"),
                QStringLiteral("reverseAPIFeatureIndex")
            }));
        }
    }

    resetContextMenuType();
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

    if (protocol == CameraProtocol::video()) {
        m_settings.m_videoFileCameraPath = cameraId;
    }
    else if (protocol == CameraProtocol::stream()) {
        m_settings.m_streamUrl = cameraId;
    }
    else if (protocol == CameraProtocol::images()) {
        m_settings.m_cameraId = CameraProtocol::images();
    }

    if (protocol == CameraProtocol::alpaca()) {
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

    if (lhs.m_protocol == CameraProtocol::alpaca()) {
        return (lhs.m_host == rhs.m_host) && (lhs.m_port == rhs.m_port);
    }

    return true;
}

bool CameraGUI::isSameHardwareCameraBackend(const CameraInfo& lhs, const CameraInfo& rhs)
{
    const auto isSharedHardwareCamera = [](const QString& protocol) {
        return (protocol == CameraProtocol::alpaca()) || (protocol == CameraProtocol::asi());
    };

    return isSharedHardwareCamera(lhs.m_protocol) && (lhs.m_protocol == rhs.m_protocol);
}

QStringList CameraGUI::cameraSelectionSettingsKeys(const CameraInfo& cameraInfo) const
{
    QStringList settingsKeys {"cameraProtocol", "cameraId", "cameraDescription"};

    if (cameraInfo.m_protocol == CameraProtocol::video()) {
        settingsKeys.append("videoFileCameraPath");
    }
    else if (cameraInfo.m_protocol == CameraProtocol::stream()) {
        settingsKeys.append("streamUrl");
        settingsKeys.append("streamUrlHistory");
    }
    else if (cameraInfo.m_protocol == CameraProtocol::images()) {
        settingsKeys.append("imageFileCameraPaths");
    }

    if (cameraInfo.m_protocol == CameraProtocol::alpaca()) {
        settingsKeys.append("alpacaHost");
        settingsKeys.append("alpacaPort");
    }

    return settingsKeys;
}

bool CameraGUI::restorePreviousCameraSelection(const QString& previousCameraProtocol,
    const QString& previousCameraId,
    const QString& previousAlpacaHost,
    quint16 previousAlpacaPort)
{
    const int previousIndex = findCameraComboIndex(
        previousCameraProtocol,
        previousCameraId,
        previousAlpacaHost,
        previousAlpacaPort);
    if (previousIndex < 0) {
        return false;
    }

    QSignalBlocker blocker(ui->cameraCombo);
    ui->cameraCombo->setCurrentIndex(previousIndex);
    return true;
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
    m_lastPlateSolved = false;
    m_lastPlateSolvedMatches = 0;
    m_lastPlateSolveDetectedStarsConsidered = 0;
    m_lastPlateSolveCatalogStarsLoaded = 0;
    m_lastPlateSolveCatalogCandidateStars = 0;
    m_lastPlateSolveOutlierStars = 0;
    m_lastPlateSolveRmsError = 0.0;
    m_lastPlateSolveMaxError = 0.0;
    m_lastPlateSolveAzimuth = 0.0;
    m_lastPlateSolveElevation = 0.0;
    m_lastPlateSolveRoll = 0.0;
    m_lastPlateSolveFov = 0.0;
    m_lastPlateSolveCenterOffsetX = 0.0;
    m_lastPlateSolveCenterOffsetY = 0.0;
    m_lastPlateSolveDistortionK1 = 0.0;
    m_lastPlateSolveCatalogSource.clear();
    m_lastStarDetections.clear();
    m_lastPreviewTextLabels.clear();
    m_lastPreviewRectItems.clear();
    clearPreviewOverlayItems();
    settingsUI()->pipelineFpsLabel->setText("-");
    settingsUI()->plateSolveStatusLabel->setText("-");
    settingsUI()->plateSolveMatchesLabel->setText("-");
    settingsUI()->plateSolveDetectedLabel->setText("-");
    settingsUI()->plateSolveRmsLabel->setText("-");
    settingsUI()->plateSolveApplyButton->setEnabled(false);
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
        restorePreviousCameraSelection(
            previousCameraProtocol,
            previousCameraId,
            previousAlpacaHost,
            previousAlpacaPort);
        return false;
    }

    const QString description = QFileInfo(filePath).fileName();
    ui->cameraCombo->setItemData(comboIndex, filePath, CameraIdRole);
    ui->cameraCombo->setItemData(comboIndex, description, CameraDescriptionRole);
    ui->cameraCombo->setItemText(comboIndex, CameraProtocol::playbackDisplayText(CameraProtocol::video(), description));
    return true;
}

bool CameraGUI::chooseImageFileSequenceFiles(int comboIndex, const QString& previousCameraProtocol,
    const QString& previousCameraId,
    const QString& previousAlpacaHost, quint16 previousAlpacaPort)
{
    CameraFileSequenceDialog dialog(m_settings.m_imageFileCameraPaths, this);
    if (dialog.exec() != QDialog::Accepted)
    {
        restorePreviousCameraSelection(
            previousCameraProtocol,
            previousCameraId,
            previousAlpacaHost,
            previousAlpacaPort);
        return false;
    }

    m_settings.m_imageFileCameraPaths = dialog.fileNames();
    const int fileCount = m_settings.m_imageFileCameraPaths.size();
    const QString description = fileCount == 0
        ? QString()
        : tr("%1 image%2").arg(fileCount).arg(fileCount == 1 ? QString() : QStringLiteral("s"));
    ui->cameraCombo->setItemData(comboIndex, CameraProtocol::images(), CameraIdRole);
    ui->cameraCombo->setItemData(comboIndex, description, CameraDescriptionRole);
    ui->cameraCombo->setItemText(comboIndex, CameraProtocol::playbackDisplayText(CameraProtocol::images(), description));
    return true;
}

bool CameraGUI::chooseStreamUrl(int comboIndex, const QString& previousCameraProtocol,
    const QString& previousCameraId,
    const QString& previousAlpacaHost, quint16 previousAlpacaPort)
{
    bool ok = false;
    QStringList history = m_settings.m_streamUrlHistory;
    const QString currentUrl = m_settings.isStreamCamera() ? m_settings.m_streamUrl : QString();
    if (!currentUrl.isEmpty()) {
        history.removeAll(currentUrl);
        history.prepend(currentUrl);
    }
    if (history.isEmpty()) {
        history.append(QStringLiteral("rtsp://"));
    }

    const QString url = QInputDialog::getItem(
        this,
        tr("Open Stream"),
        tr("Stream URL"),
        history,
        0,
        true,
        &ok).trimmed();

    if (!ok || url.isEmpty())
    {
        restorePreviousCameraSelection(
            previousCameraProtocol,
            previousCameraId,
            previousAlpacaHost,
            previousAlpacaPort);
        return false;
    }

    m_settings.m_streamUrlHistory.removeAll(url);
    m_settings.m_streamUrlHistory.prepend(url);
    while (m_settings.m_streamUrlHistory.size() > 20) {
        m_settings.m_streamUrlHistory.removeLast();
    }

    ui->cameraCombo->setItemData(comboIndex, url, CameraIdRole);
    ui->cameraCombo->setItemData(comboIndex, url, CameraDescriptionRole);
    ui->cameraCombo->setItemText(comboIndex, CameraProtocol::playbackDisplayText(CameraProtocol::stream(), url));
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
    const int autoExposureGainControl = m_settings.m_autoExposureGainEnabled
        ? AutoExposureGainSoftware
        : (m_settings.m_asiAutoExposureGain ? AutoExposureGainHardware : AutoExposureGainNone);
    settingsUI()->autoExposureGainCombo->setCurrentIndex(std::max(0,
        settingsUI()->autoExposureGainCombo->findData(autoExposureGainControl)));
    settingsUI()->autoExposureGainModeCombo->setCurrentIndex(std::max(0,
        settingsUI()->autoExposureGainModeCombo->findData(static_cast<int>(m_settings.m_autoExposureGainMode))));
    settingsUI()->autoExposureTargetSpin->setValue(m_settings.m_autoExposureTargetBrightness);
    settingsUI()->autoExposurePercentileSpin->setValue(m_settings.m_autoExposureTargetPercentile);
    settingsUI()->autoExposureMinMsSpin->setValue(m_settings.m_autoExposureMinMs);
    settingsUI()->autoExposureMaxMsSpin->setValue(m_settings.m_autoExposureMaxMs);
    settingsUI()->autoExposureMinGainSpin->setValue(m_settings.m_autoExposureMinGain);
    settingsUI()->autoExposureMaxGainSpin->setValue(m_settings.m_autoExposureMaxGain);
    settingsUI()->autoExposureMaxChangeSpin->setValue(m_settings.m_autoExposureMaxChangePercent);
    settingsUI()->asiColorImageTypeCombo->setCurrentIndex(static_cast<int>(m_settings.m_asiColorImageType));
    ui->saveImageCheck->setChecked(m_settings.m_saveImage);
    settingsUI()->imagePathEdit->setText(m_settings.m_imageFileName);
    ui->saveVideoCheck->setChecked(m_settings.m_saveVideo);
    settingsUI()->videoPathEdit->setText(m_settings.m_videoFileName);
    ui->keogramButton->setChecked(m_settings.m_keogramEnabled);
    settingsUI()->keogramPathEdit->setText(m_settings.m_keogramFileName);
    settingsUI()->keogramDirectionCombo->setCurrentIndex(static_cast<int>(m_settings.m_keogramDirection));
    settingsUI()->keogramDayModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_keogramDayMode));
    settingsUI()->keogramSamplePeriodSpin->setValue(m_settings.m_keogramSamplePeriodMinutes);
    settingsUI()->keogramPreviewCheck->setChecked(m_settings.m_keogramShowPreview);
    ui->youtubeStreamButton->setChecked(m_settings.m_youtubeStreamEnabled);
    settingsUI()->youtubeStreamUrlEdit->setText(m_settings.m_youtubeStreamUrl);
    settingsUI()->youtubeStreamKeyEdit->setText(m_settings.m_youtubeStreamKey);
    settingsUI()->youtubeStreamSourceCombo->setCurrentIndex(m_settings.m_youtubeStreamPostProcessed ? 1 : 0);
    updateYouTubeStreamButtonEnabled();
    updateYouTubeBitrateCombo();
    settingsUI()->youtubeStreamFpsSpin->setValue(m_settings.m_youtubeStreamFps);
    settingsUI()->youtubeStreamWidthSpin->setValue(m_settings.m_youtubeStreamWidth);
    settingsUI()->youtubeStreamHeightSpin->setValue(m_settings.m_youtubeStreamHeight);
    settingsUI()->videoCodecCombo->setCurrentIndex(static_cast<int>(m_settings.m_videoCodec));
    updateVideoRecordBitrateCombo();
    settingsUI()->videoHwAccelerationCheck->setChecked(m_settings.m_videoHwAcceleration);
    settingsUI()->streamBufferingSecondsSpin->setValue(m_settings.m_streamBufferingSeconds);
    settingsUI()->videoPreRecordBufferSpin->setValue(m_settings.m_videoPreRecordBufferSeconds);
    settingsUI()->imageRecordLimitSpin->setValue(m_settings.m_imageRecordLimit);
    settingsUI()->videoRecordLimitSpin->setValue(m_settings.m_videoRecordLimitSeconds);
    settingsUI()->recordRawFitsCheck->setChecked(m_settings.m_recordRawFits);
    settingsUI()->recordCalibratedMediaCheck->setChecked(m_settings.m_recordCalibratedMedia);
    settingsUI()->recordFilteredMediaCheck->setChecked(m_settings.m_recordFilteredMedia);
    settingsUI()->recordPostProcessedMediaCheck->setChecked(m_settings.m_recordPostProcessedMedia);
    ui->stackEnabledButton->setChecked(m_settings.m_stackEnabled);
    settingsUI()->stackFrameCountSpin->setValue(m_settings.m_stackFrameCount);
    settingsUI()->stackMethodCombo->setCurrentIndex(static_cast<int>(m_settings.m_stackMethod));
    settingsUI()->stackHdrAlgorithmCombo->setCurrentIndex(static_cast<int>(m_settings.m_stackHdrAlgorithm));
    settingsUI()->stackHdrExposureCountSpin->setValue(m_settings.getHdrExposureCount());
    settingsUI()->stackAlignmentCombo->setCurrentIndex(static_cast<int>(m_settings.m_stackAlignmentMethod));
    settingsUI()->stackDisplayModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_stackDisplayMode));
    settingsUI()->stackDisplayFrameSpin->setValue(m_settings.m_stackDisplayFrameIndex + 1);
    settingsUI()->stackDisplayFrameLabel->setEnabled(m_settings.m_stackDisplayMode == CameraSettings::StackDisplayHistoryFrame);
    settingsUI()->stackDisplayFrameSpin->setEnabled(m_settings.m_stackDisplayMode == CameraSettings::StackDisplayHistoryFrame);
    settingsUI()->stackRejectBadFramesCheck->setChecked(m_settings.m_stackRejectBadFrames);
    settingsUI()->scaleEnabledCheck->setChecked(m_settings.m_scaleEnabled);
    settingsUI()->scaleWidthSpin->setValue(m_settings.m_scaleWidth);
    settingsUI()->scaleHeightSpin->setValue(m_settings.m_scaleHeight);
    settingsUI()->scaleKeepAspectRatioCheck->setChecked(m_settings.m_scaleKeepAspectRatio);
    settingsUI()->scaleJustificationCombo->setCurrentIndex(static_cast<int>(m_settings.m_scaleJustification));
    updateScaleControls();
    settingsUI()->stackDarkFileEdit->setText(m_settings.m_stackDarkFileName);
    settingsUI()->stackFlatFileEdit->setText(m_settings.m_stackFlatFileName);
    settingsUI()->stackBiasFileEdit->setText(m_settings.m_stackBiasFileName);
    updateHdrExposureControls();
    updateHdrStackingControls();
    settingsUI()->latitudeSpin->setValue(m_settings.m_latitude);
    settingsUI()->longitudeSpin->setValue(m_settings.m_longitude);
    settingsUI()->altitudeSpin->setValue(m_settings.m_altitude);
    settingsUI()->owmApiKeyEdit->setText(m_settings.m_owmAPIKey);
    settingsUI()->azimuthSpin->setValue(m_settings.m_azimuth);
    settingsUI()->elevationSpin->setValue(m_settings.m_elevation);
    settingsUI()->azimuthOffsetSpin->setValue(m_settings.m_azimuthOffset);
    settingsUI()->elevationOffsetSpin->setValue(m_settings.m_elevationOffset);
    settingsUI()->rollOffsetSpin->setValue(m_settings.m_rollOffset);
    settingsUI()->rollSpin->setValue(m_settings.m_roll);
    settingsUI()->fovModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_fovMode));
    settingsUI()->fovSpin->setValue(m_settings.m_fov);
    settingsUI()->fovSensorWidthSpin->setValue(m_settings.m_fovSensorWidthMm);
    settingsUI()->fovSensorHeightSpin->setValue(m_settings.m_fovSensorHeightMm);
    settingsUI()->fovFocalLengthSpin->setValue(m_settings.m_fovFocalLengthMm);
    updateFovControls();
    settingsUI()->lensProjectionCombo->setCurrentIndex(static_cast<int>(m_settings.m_lensProjection));
    settingsUI()->lensCenterOffsetXSpin->setValue(m_settings.m_lensCenterOffsetX);
    settingsUI()->lensCenterOffsetYSpin->setValue(m_settings.m_lensCenterOffsetY);
    settingsUI()->lensDistortionK1Spin->setValue(m_settings.m_lensDistortionK1);
    populateDirectionSourceCombo();
    applyPositionSync();
    updatePositionControls();
    settingsUI()->postProcessWhiteBalanceModeCombo->setCurrentIndex(m_settings.m_postProcessWhiteBalanceMode);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->setValue(m_settings.m_postProcessWhiteBalanceRedGain);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->setValue(m_settings.m_postProcessWhiteBalanceGreenGain);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->setValue(m_settings.m_postProcessWhiteBalanceBlueGain);
    settingsUI()->postProcessWhiteBalanceRedGainSlider->setMaximum(doubleSpinBoxSliderMaximum(settingsUI()->postProcessWhiteBalanceRedGainSpin));
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->setMaximum(doubleSpinBoxSliderMaximum(settingsUI()->postProcessWhiteBalanceGreenGainSpin));
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->setMaximum(doubleSpinBoxSliderMaximum(settingsUI()->postProcessWhiteBalanceBlueGainSpin));
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSlider->setMaximum(doubleSpinBoxSliderMaximum(settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin));
    settingsUI()->postProcessWhiteBalanceRedGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceRedGainSpin, m_settings.m_postProcessWhiteBalanceRedGain));
    settingsUI()->postProcessWhiteBalanceGreenGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceGreenGainSpin, m_settings.m_postProcessWhiteBalanceGreenGain));
    settingsUI()->postProcessWhiteBalanceBlueGainSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceBlueGainSpin, m_settings.m_postProcessWhiteBalanceBlueGain));
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin->setValue(m_settings.m_postProcessWhiteBalanceHighlightProtection);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin, m_settings.m_postProcessWhiteBalanceHighlightProtection));
    settingsUI()->stackCurrentCountValue->setText(tr("%1 / %2 / %3 / %4")
        .arg(m_lastStackCount)
        .arg(m_lastStackQueuedCount)
        .arg(m_lastStackDroppedCount)
        .arg(m_lastStackRejectedCount));
    settingsUI()->postProcessUseCudaCheck->setChecked(m_settings.m_postProcessUseCuda);
#ifndef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    settingsUI()->postProcessUseCudaCheck->setEnabled(false);
    settingsUI()->postProcessUseCudaCheck->setToolTip(tr("OpenCV CUDA camera processing modules are not available in this build"));
#endif
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
    settingsUI()->edgeDisplayModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_edgeDisplayMode));
    settingsUI()->sobelEdgeSlider->setValue(static_cast<int>(m_settings.m_sobelEdge * 100.0));
    settingsUI()->sobelEdgeSpin->setValue(m_settings.m_sobelEdge);
    settingsUI()->cannyEdgeSlider->setValue(static_cast<int>(m_settings.m_cannyEdge * 100.0));
    settingsUI()->cannyEdgeSpin->setValue(m_settings.m_cannyEdge);
    settingsUI()->flipXButton->setChecked(m_settings.m_flipX);
    settingsUI()->flipYButton->setChecked(m_settings.m_flipY);
    settingsUI()->imageRotationCombo->setCurrentIndex(qBound(0, m_settings.m_imageRotation / 90, 3));
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
    ui->constellationButton->setChecked(m_settings.m_constellation);
    settingsUI()->constellationOverlayCombo->setCurrentIndex(static_cast<int>(m_settings.m_constellationOverlay));
    ui->trackObjectsButton->setChecked(m_settings.m_trackObjects);
    settingsUI()->trackObjectTrailsCheck->setChecked(m_settings.m_trackObjectTrails);
    settingsUI()->trackObjectHeatMapCheck->setChecked(m_settings.m_trackObjectHeatMap);
    settingsUI()->trackObjectMinElevationSpin->setValue(m_settings.m_trackObjectMinElevation);
    settingsUI()->trackObjectFontCombo->setCurrentText(m_settings.m_trackObjectFontFamily);
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
    settingsUI()->detectionRoiShowButton->setChecked(m_settings.m_showDetectionRoi);
    ui->motionDetectButton->setChecked(m_settings.m_motionDetect);
    settingsUI()->motionBackgroundSubtractorCombo->setCurrentIndex(static_cast<int>(m_settings.m_motionBackgroundSubtractor));
    settingsUI()->motionMaskViewCombo->setCurrentIndex(static_cast<int>(m_settings.m_motionMaskView));
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
    ui->starDetectButton->setChecked(m_settings.m_starDetect);
    settingsUI()->starThresholdSpin->setValue(m_settings.m_starThreshold);
    settingsUI()->starBackgroundBlurSpin->setValue(m_settings.m_starBackgroundBlur);
    settingsUI()->starMinAreaSpin->setValue(m_settings.m_starMinArea);
    settingsUI()->starMaxAreaSpin->setValue(m_settings.m_starMaxArea);
    settingsUI()->starMaxAspectRatioSpin->setValue(m_settings.m_starMaxAspectRatio);
    settingsUI()->starDebugViewCombo->setCurrentIndex(static_cast<int>(m_settings.m_starDebugView));
    settingsUI()->plateSolveLabelModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_plateSolveLabelMode));
    settingsUI()->plateSolveMaxMagnitudeSpin->setValue(m_settings.m_plateSolveMaxMagnitude);
    settingsUI()->plateSolveMinMatchesSpin->setValue(m_settings.m_plateSolveMinMatches);
    settingsUI()->plateSolveMatchRadiusSpin->setValue(m_settings.m_plateSolveMatchRadius);
    settingsUI()->plateSolveFinalMatchRadiusSpin->setValue(m_settings.m_plateSolveFinalMatchRadius);
    settingsUI()->plateSolveSearchRadiusSpin->setValue(m_settings.m_plateSolveAzElSearchRadius);
    settingsUI()->plateSolveFovToleranceSpin->setValue(m_settings.m_plateSolveFovTolerance);
    settingsUI()->plateSolveStartModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_plateSolveStartMode));
    updatePlateSolveStartModeUi();
    settingsUI()->plateSolveDateTimeModeCombo->setCurrentIndex(m_settings.m_plateSolveUseCaptureDateTime ? 0 : 1);
    settingsUI()->plateSolveDateTimeUtcButton->setChecked(m_settings.m_plateSolveDateTimeUtc);
    updatePlateSolveDateTimeEdit();
    settingsUI()->plateSolveCatalogSourceCombo->setCurrentIndex(static_cast<int>(m_settings.m_plateSolveCatalogSource));
    settingsUI()->starCatalogDiskCacheSizeSpin->setValue(m_settings.m_starCatalogDiskCacheSizeGb);
    settingsUI()->plateSolveApplyModeCombo->setCurrentIndex(static_cast<int>(m_settings.m_plateSolveApplyMode));
    settingsUI()->plateSolveApplyButton->setEnabled(m_lastPlateSolved);
    ui->loopVideo->setChecked(m_settings.m_videoLoop);
    ui->playbackRateSpin->setValue(m_settings.m_videoPlaybackRate);
    settingsUI()->playbackAudioOffsetSpin->setValue(m_settings.m_videoPlaybackAudioOffsetMs);
    updateMotionExclusionRectsTable();
    updateColorButton(settingsUI()->dateTimeColorButton, m_settings.m_dateTimeColor);
    updateColorButton(settingsUI()->equatorialGridColorButton, m_settings.m_equatorialGridColor);
    updateColorButton(settingsUI()->altAzGridColorButton, m_settings.m_altAzGridColor);
    updateColorButton(settingsUI()->constellationColorButton, m_settings.m_constellationColor);
    updateColorButton(settingsUI()->trackObjectColorButton, m_settings.m_trackObjectColor);
    updateColorButton(settingsUI()->starColorButton, m_settings.m_starColor);
    settingsUI()->showStarDetectionBoxesCheck->setChecked(m_settings.m_showStarDetectionBoxes);
    settingsUI()->hideSyntheticNamesCheck->setChecked(m_settings.m_plateSolveLabelHideSyntheticNames);
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
    {
        const bool yoloEnabled = m_settings.m_yoloEnabled;
        QComboBox *modelPathCombo = settingsUI()->yoloModelPathCombo;
        QComboBox *labelsPathCombo = settingsUI()->yoloLabelsPathCombo;
        const QSignalBlocker yoloButtonBlocker(ui->yoloButton);
        const QSignalBlocker modelComboBlocker(modelPathCombo);
        const QSignalBlocker labelsComboBlocker(labelsPathCombo);
        QLineEdit *modelLineEdit = modelPathCombo->lineEdit();
        QLineEdit *labelsLineEdit = labelsPathCombo->lineEdit();
        const bool modelLineEditWasBlocked = modelLineEdit && modelLineEdit->blockSignals(true);
        const bool labelsLineEditWasBlocked = labelsLineEdit && labelsLineEdit->blockSignals(true);

        modelPathCombo->setCurrentText(m_settings.m_yoloModelPath);
        labelsPathCombo->setCurrentText(m_settings.m_yoloLabelsPath);
        updateYoloButtonEnabled();
        ui->yoloButton->setChecked(yoloEnabled && ui->yoloButton->isEnabled());

        if (modelLineEdit) {
            modelLineEdit->blockSignals(modelLineEditWasBlocked);
        }
        if (labelsLineEdit) {
            labelsLineEdit->blockSignals(labelsLineEditWasBlocked);
        }
    }
    settingsUI()->yoloConfSpin->setValue(m_settings.m_yoloConfThreshold);
    settingsUI()->yoloNmsSpin->setValue(m_settings.m_yoloNmsThreshold);
    settingsUI()->yoloTargetCombo->setCurrentIndex((int) m_settings.m_yoloDnnTarget);
    settingsUI()->yoloTileLargeImagesCheck->setChecked(m_settings.m_yoloTileLargeImages);
    settingsUI()->yoloTileOverlapSpin->setValue(m_settings.m_yoloTileOverlapPercent);
    settingsUI()->yoloTileOverlapSpin->setEnabled(m_settings.m_yoloTileLargeImages);
    {
        const QSignalBlocker ignoredClassesBlocker(settingsUI()->yoloIgnoredClassNamesEdit);
        settingsUI()->yoloIgnoredClassNamesEdit->setPlainText(m_settings.m_yoloIgnoredClassNames.join(QStringLiteral("\n")));
    }
    updateColorButton(settingsUI()->yoloBoxColorButton, m_settings.m_yoloBoxColor);
    ui->audioMute->setChecked(m_settings.m_audioMute);
    {
        const QSignalBlocker audioPreviewVolumeBlocker(ui->audioPreviewVolumeDial);
        ui->audioPreviewVolumeDial->setValue(m_settings.m_audioPreviewVolume);
        ui->audioPreviewVolumeDial->setToolTip(tr("Audio preview volume: %1%").arg(m_settings.m_audioPreviewVolume));
    }

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

void CameraImageGraphicsItem::setImage(const QImage& image)
{
    if (m_image.size() != image.size()) {
        prepareGeometryChange();
    }
    m_image = image;
    update();
}

void CameraImageGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)
    if (!m_image.isNull()) {
        painter->drawImage(0, 0, m_image);
    }
}

void CameraGUI::updateImageWidget()
{
    if (m_lastImage.isNull() || !m_imagePixmapItem)
    {
        clearPreviewOverlayItems();
        return;
    }

    // Paint the (pooled) frame directly — no per-frame QPixmap::fromImage alloc.
    m_imagePixmapItem->setImage(m_lastImage);
    m_imageScene->setSceneRect(QRectF(m_lastImage.rect()));
    updatePreviewOverlayItems();

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

void CameraGUI::sendDisplayedFrameEvents(const QVector<QRect>& motionBoxes, const QVector<CameraPipelineDetection>& detections, const QVector<CameraPipelineMeteorPhotometry>& meteorPhotometry, const QVector<CameraPipelineTrackedObject>& trackedObjects, const QSize& imageSize, const QDateTime& captureDateTime)
{
    if (!m_camera) {
        return;
    }

    struct PendingEvent
    {
        MainCore::MsgEvent::EventType m_eventType;
        QString m_data;
    };

    QVector<PendingEvent> pendingEvents;
    const bool motionDetected = !motionBoxes.isEmpty();
    if (motionDetected != m_displayedMotionEventActive)
    {
        QString eventData = QStringLiteral("boxes=%1").arg(motionBoxes.size());
        if (!motionBoxes.isEmpty())
        {
            const QRect& box = motionBoxes.first();
            eventData += QStringLiteral(",x=%1,y=%2,width=%3,height=%4")
                .arg(box.x())
                .arg(box.y())
                .arg(box.width())
                .arg(box.height());
        }

        pendingEvents.append({
            motionDetected
                ? MainCore::MsgEvent::EventType::CameraMotionDetectedEvent
                : MainCore::MsgEvent::EventType::CameraMotionStoppedEvent,
            eventData
        });
        m_displayedMotionEventActive = motionDetected;
    }

    QSet<QString> displayedObjectClasses;
    QHash<QString, QRect> displayedObjectBoxes;
    QHash<QString, CameraPipelineMeteorPhotometry> displayedMeteorPhotometry;
    for (const CameraPipelineDetection& detection : detections)
    {
        if (!detection.m_label.isEmpty())
        {
            displayedObjectClasses.insert(detection.m_label);
            if (!displayedObjectBoxes.contains(detection.m_label)) {
                displayedObjectBoxes.insert(detection.m_label, detection.m_box);
            }
            if (detection.m_label.trimmed().compare(QStringLiteral("meteor"), Qt::CaseInsensitive) == 0)
            {
                const CameraPipelineMeteorPhotometry *bestMeteor = nullptr;
                int bestArea = 0;
                for (const CameraPipelineMeteorPhotometry& meteor : meteorPhotometry)
                {
                    const QRect intersection = detection.m_box.intersected(meteor.m_box);
                    const int area = intersection.width() * intersection.height();
                    if (area > bestArea)
                    {
                        bestArea = area;
                        bestMeteor = &meteor;
                    }
                }
                if (bestMeteor && !displayedMeteorPhotometry.contains(detection.m_label)) {
                    displayedMeteorPhotometry.insert(detection.m_label, *bestMeteor);
                }
            }
        }
    }

    for (const QString& className : displayedObjectClasses)
    {
        if (!m_displayedObjectEventClasses.contains(className))
        {
            const QRect box = displayedObjectBoxes.value(className);
            QString eventData = QStringLiteral("name=%1,x=%2,y=%3,width=%4,height=%5")
                .arg(className)
                .arg(box.x())
                .arg(box.y())
                .arg(box.width())
                .arg(box.height());
            if (displayedMeteorPhotometry.contains(className))
            {
                const CameraPipelineMeteorPhotometry meteor = displayedMeteorPhotometry.value(className);
                eventData += QStringLiteral(",flux=%1,background=%2,backgroundSigma=%3,referenceStars=%4,zeroPoint=%5,zeroPointRms=%6,saturated=%7")
                    .arg(meteor.m_flux, 0, 'g', 12)
                    .arg(meteor.m_background, 0, 'g', 12)
                    .arg(meteor.m_backgroundSigma, 0, 'g', 12)
                    .arg(meteor.m_referenceStars)
                    .arg(meteor.m_zeroPoint, 0, 'g', 12)
                    .arg(meteor.m_zeroPointRms, 0, 'g', 12)
                    .arg(meteor.m_saturated ? 1 : 0);
                if (meteor.m_validMagnitude)
                {
                    eventData += QStringLiteral(",magnitude=%1,magnitudeError=%2")
                        .arg(meteor.m_magnitude, 0, 'g', 12)
                        .arg(meteor.m_magnitudeError, 0, 'g', 12);
                }
                if (!meteor.m_failureReason.isEmpty()) {
                    eventData += QStringLiteral(",photometryStatus=%1").arg(meteor.m_failureReason);
                }
            }
            pendingEvents.append({
                MainCore::MsgEvent::EventType::CameraObjectDetectedEvent,
                eventData
            });
        }
    }

    for (const QString& className : m_displayedObjectEventClasses)
    {
        if (!displayedObjectClasses.contains(className))
        {
            pendingEvents.append({
                MainCore::MsgEvent::EventType::CameraObjectLostEvent,
                QStringLiteral("name=%1").arg(className)
            });
        }
    }
    m_displayedObjectEventClasses = displayedObjectClasses;

    QRectF detectionRoi;
    if (imageSize.isValid())
    {
        const QRect imageBounds(QPoint(0, 0), imageSize);
        QRect roi = imageBounds;
        if ((m_settings.m_detectionRoiWidth > 0) && (m_settings.m_detectionRoiHeight > 0))
        {
            roi = QRect(
                m_settings.m_detectionRoiX,
                m_settings.m_detectionRoiY,
                m_settings.m_detectionRoiWidth,
                m_settings.m_detectionRoiHeight).intersected(imageBounds);
        }
        detectionRoi = QRectF(roi);
    }

    QSet<QString> trackedObjectsInView;
    if (detectionRoi.isValid())
    {
        for (const CameraPipelineTrackedObject& trackedObject : trackedObjects)
        {
            const QString name = trackedObject.m_name.trimmed();
            if (name.isEmpty() || !detectionRoi.contains(trackedObject.m_position)) {
                continue;
            }
            trackedObjectsInView.insert(name);
            if (!m_displayedTrackedObjectsInView.contains(name))
            {
                pendingEvents.append({
                    MainCore::MsgEvent::EventType::CameraObjectInViewEvent,
                    QStringLiteral("name=%1,label=%2,x=%3,y=%4,azimuth=%5,elevation=%6")
                        .arg(name)
                        .arg(trackedObject.m_label)
                        .arg(trackedObject.m_position.x(), 0, 'f', 1)
                        .arg(trackedObject.m_position.y(), 0, 'f', 1)
                        .arg(trackedObject.m_azimuth, 0, 'f', 3)
                        .arg(trackedObject.m_elevation, 0, 'f', 3)
                });
            }
        }
    }

    for (const QString& name : m_displayedTrackedObjectsInView)
    {
        if (!trackedObjectsInView.contains(name))
        {
            pendingEvents.append({
                MainCore::MsgEvent::EventType::CameraObjectOutOfViewEvent,
                QStringLiteral("name=%1").arg(name)
            });
        }
    }
    m_displayedTrackedObjectsInView = trackedObjectsInView;

    if (pendingEvents.isEmpty()) {
        return;
    }

    QList<ObjectPipe*> eventPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_camera, "event", eventPipes);
    if (eventPipes.isEmpty()) {
        return;
    }

    const QDateTime eventTime = captureDateTime.isValid() ? captureDateTime : QDateTime::currentDateTime();
    for (const PendingEvent& event : pendingEvents)
    {
        for (const ObjectPipe *pipe : eventPipes)
        {
            MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
            if (messageQueue) {
                messageQueue->push(MainCore::MsgEvent::create(m_camera, eventTime, event.m_eventType, event.m_data));
            }
        }
    }
}

void CameraGUI::clearPreviewOverlayItems()
{
    if (!m_imageScene) {
        m_previewOverlayItems.clear();
        return;
    }

    for (QGraphicsItem *item : m_previewOverlayItems)
    {
        m_imageScene->removeItem(item);
        delete item;
    }
    m_previewOverlayItems.clear();
}

void CameraGUI::updatePreviewOverlayItems()
{
    clearPreviewOverlayItems();

    if (!m_imageScene || m_lastImage.isNull() || (m_lastPreviewTextLabels.isEmpty() && m_lastPreviewRectItems.isEmpty())) {
        return;
    }

    const QRect imageRect(QPoint(0, 0), m_lastImage.size());

    for (const CameraPostProcessor::PreviewRectItem& previewRect : m_lastPreviewRectItems)
    {
        const QRectF clipped = previewRect.m_rect.intersected(QRectF(imageRect));
        if (clipped.isEmpty()) {
            continue;
        }

        QPen pen(previewRect.m_color);
        pen.setWidthF(std::max(1.0, previewRect.m_lineWidth));
        QGraphicsRectItem *rectItem = m_imageScene->addRect(clipped, pen, QBrush(Qt::NoBrush));
        rectItem->setZValue(1.4);
        m_previewOverlayItems.append(rectItem);
    }

    for (const CameraPostProcessor::PreviewTextLabel& previewLabel : m_lastPreviewTextLabels)
    {
        if (previewLabel.m_text.isEmpty()) {
            continue;
        }

        QFont font;
        if (!previewLabel.m_fontFamily.isEmpty()) {
            font.setFamily(previewLabel.m_fontFamily);
        }
        font.setPointSizeF(std::max(6.0, previewLabel.m_fontPointSize));
        const QFontMetrics fontMetrics(font);
        const QStringList lines = previewLabel.m_text.split(QChar('\n'));
        int textWidth = 0;
        for (const QString& line : lines) {
            textWidth = std::max(textWidth, fontMetrics.horizontalAdvance(line));
        }
        if (textWidth <= 0 || lines.isEmpty()) {
            continue;
        }

        const int lineSpacing = fontMetrics.lineSpacing();
        QRect targetRect;

        if (previewLabel.m_positionIsTopLeft)
        {
            targetRect = QRect(
                qRound(previewLabel.m_position.x()),
                qRound(previewLabel.m_position.y()),
                textWidth + 6,
                lines.size() * lineSpacing + 4);
        }
        else
        {
            const QPointF labelPoint = previewLabel.m_position + QPointF(4.0, -4.0);
            targetRect = QRect(
                qRound(labelPoint.x()),
                qRound(labelPoint.y()) - lines.size() * lineSpacing,
                textWidth + 4,
                lines.size() * lineSpacing + 2);
        }

        if (!imageRect.adjusted(0, 0, -1, -1).intersects(targetRect)) {
            continue;
        }

        const QPointF textPos = previewLabel.m_positionIsTopLeft
            ? QPointF(targetRect.left() + 3.0, targetRect.top() + 2.0)
            : targetRect.topLeft();

        if (previewLabel.m_background)
        {
            QGraphicsRectItem *backgroundItem = m_imageScene->addRect(
                targetRect,
                QPen(Qt::NoPen),
                QBrush(Qt::black));
            backgroundItem->setZValue(1.45);
            m_previewOverlayItems.append(backgroundItem);
        }
        else
        {
            QGraphicsSimpleTextItem *shadowItem = m_imageScene->addSimpleText(previewLabel.m_text, font);
            shadowItem->setPos(textPos + QPointF(1.0, 1.0));
            shadowItem->setBrush(QBrush(Qt::black));
            shadowItem->setZValue(1.5);
            m_previewOverlayItems.append(shadowItem);
        }

        QGraphicsSimpleTextItem *item = m_imageScene->addSimpleText(previewLabel.m_text, font);
        item->setPos(textPos);
        item->setBrush(QBrush(previewLabel.m_color));
        item->setZValue(1.6);
        m_previewOverlayItems.append(item);
    }
}

void CameraGUI::makeUIConnections()
{
    QObject::connect(ui->cameraSettingsButton, &QToolButton::clicked, this, &CameraGUI::on_cameraSettingsButton_clicked);
    QObject::connect(ui->startStop, &QPushButton::clicked, this, &CameraGUI::on_startStop_clicked);
    QObject::connect(ui->refreshCamerasButton, &QPushButton::clicked, this, &CameraGUI::on_refreshCamerasButton_clicked);
    QObject::connect(ui->cameraCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_cameraCombo_currentIndexChanged);
    QObject::connect(ui->browseVideoFileButton, &QToolButton::clicked, this, &CameraGUI::on_browseVideoFileButton_clicked);
    QObject::connect(ui->restartVideo, &QToolButton::clicked, this, &CameraGUI::on_restartVideo_clicked);
    QObject::connect(ui->stepBackVideo, &QToolButton::clicked, this, &CameraGUI::on_stepBackVideo_clicked);
    QObject::connect(ui->stepForwardVideo, &QToolButton::clicked, this, &CameraGUI::on_stepForwardVideo_clicked);
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
    QObject::connect(settingsUI()->alpacaAutoFocusButton, &QPushButton::clicked, this, &CameraGUI::on_alpacaAutoFocusButton_clicked);
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
    QObject::connect(settingsUI()->autoExposureGainCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_autoExposureGainCombo_currentIndexChanged);
    QObject::connect(settingsUI()->autoExposureGainModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_autoExposureGainModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->autoExposureTargetSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_autoExposureTargetSpin_valueChanged);
    QObject::connect(settingsUI()->autoExposurePercentileSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_autoExposurePercentileSpin_valueChanged);
    QObject::connect(settingsUI()->autoExposureMinMsSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_autoExposureMinMsSpin_valueChanged);
    QObject::connect(settingsUI()->autoExposureMaxMsSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_autoExposureMaxMsSpin_valueChanged);
    QObject::connect(settingsUI()->autoExposureMinGainSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_autoExposureMinGainSpin_valueChanged);
    QObject::connect(settingsUI()->autoExposureMaxGainSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_autoExposureMaxGainSpin_valueChanged);
    QObject::connect(settingsUI()->autoExposureMaxChangeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_autoExposureMaxChangeSpin_valueChanged);
    QObject::connect(settingsUI()->asiColorImageTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_asiColorImageTypeCombo_currentIndexChanged);
    QObject::connect(ui->saveImageButton, &QToolButton::clicked, this, &CameraGUI::on_saveImageButton_clicked);
    QObject::connect(ui->saveImageCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveImageCheck_toggled);
    QObject::connect(settingsUI()->imagePathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_imagePathEdit_editingFinished);
    QObject::connect(settingsUI()->imagePathButton, &QToolButton::clicked, this, &CameraGUI::on_imagePathButton_clicked);
    QObject::connect(ui->saveVideoCheck, &QCheckBox::toggled, this, &CameraGUI::on_saveVideoCheck_toggled);
    QObject::connect(settingsUI()->videoPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_videoPathEdit_editingFinished);
    QObject::connect(settingsUI()->videoPathButton, &QToolButton::clicked, this, &CameraGUI::on_videoPathButton_clicked);
    QObject::connect(ui->keogramButton, &QToolButton::toggled, this, &CameraGUI::on_keogramButton_toggled);
    QObject::connect(settingsUI()->keogramPathEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_keogramPathEdit_editingFinished);
    QObject::connect(settingsUI()->keogramPathButton, &QToolButton::clicked, this, &CameraGUI::on_keogramPathButton_clicked);
    QObject::connect(settingsUI()->keogramDirectionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_keogramDirectionCombo_currentIndexChanged);
    QObject::connect(settingsUI()->keogramDayModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_keogramDayModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->keogramSamplePeriodSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_keogramSamplePeriodSpin_valueChanged);
    QObject::connect(settingsUI()->keogramPreviewCheck, &QCheckBox::toggled, this, &CameraGUI::on_keogramPreviewCheck_toggled);
    QObject::connect(ui->youtubeStreamButton, &QToolButton::toggled, this, &CameraGUI::on_youtubeStreamButton_toggled);
    QObject::connect(settingsUI()->youtubeStreamUrlEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_youtubeStreamUrlEdit_editingFinished);
    QObject::connect(settingsUI()->youtubeStreamKeyEdit, &QLineEdit::editingFinished, this, &CameraGUI::on_youtubeStreamKeyEdit_editingFinished);
    QObject::connect(settingsUI()->youtubeStreamKeyEdit, &QLineEdit::textChanged, this, [this]() {
        updateYouTubeStreamButtonEnabled();

        if (settingsUI()->youtubeStreamKeyEdit->text().trimmed().isEmpty() && m_settings.m_youtubeStreamEnabled)
        {
            m_settings.m_youtubeStreamEnabled = false;
            applySetting("youtubeStreamEnabled");
        }
    });
    QObject::connect(settingsUI()->youtubeStreamSourceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_youtubeStreamSourceCombo_currentIndexChanged);
    QObject::connect(settingsUI()->youtubeStreamBitrateCombo, QOverload<int>::of(&QComboBox::activated), this, &CameraGUI::on_youtubeStreamBitrateCombo_activated);
    if (settingsUI()->youtubeStreamBitrateCombo->lineEdit()) {
        QObject::connect(settingsUI()->youtubeStreamBitrateCombo->lineEdit(), &QLineEdit::editingFinished, this, &CameraGUI::on_youtubeStreamBitrateCombo_editingFinished);
    }
    QObject::connect(settingsUI()->youtubeStreamFpsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_youtubeStreamFpsSpin_valueChanged);
    QObject::connect(settingsUI()->youtubeStreamWidthSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_youtubeStreamWidthSpin_valueChanged);
    QObject::connect(settingsUI()->youtubeStreamHeightSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_youtubeStreamHeightSpin_valueChanged);
    QObject::connect(settingsUI()->videoCodecCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_videoCodecCombo_currentIndexChanged);
    QObject::connect(settingsUI()->videoRecordBitrateCombo, QOverload<int>::of(&QComboBox::activated), this, &CameraGUI::on_videoRecordBitrateCombo_activated);
    if (settingsUI()->videoRecordBitrateCombo->lineEdit()) {
        QObject::connect(settingsUI()->videoRecordBitrateCombo->lineEdit(), &QLineEdit::editingFinished, this, &CameraGUI::on_videoRecordBitrateCombo_editingFinished);
    }
    QObject::connect(settingsUI()->videoHwAccelerationCheck, &QCheckBox::toggled, this, &CameraGUI::on_videoHwAccelerationCheck_toggled);
    QObject::connect(settingsUI()->streamBufferingSecondsSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_streamBufferingSecondsSpin_valueChanged);
    QObject::connect(settingsUI()->playbackAudioOffsetSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_playbackAudioOffsetSpin_valueChanged);
    QObject::connect(settingsUI()->videoPreRecordBufferSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_videoPreRecordBufferSpin_valueChanged);
    QObject::connect(settingsUI()->imageRecordLimitSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_imageRecordLimitSpin_valueChanged);
    QObject::connect(settingsUI()->videoRecordLimitSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_videoRecordLimitSpin_valueChanged);
    QObject::connect(settingsUI()->recordRawFitsCheck, &QCheckBox::toggled, this, &CameraGUI::on_recordRawFitsCheck_toggled);
    QObject::connect(settingsUI()->recordCalibratedMediaCheck, &QCheckBox::toggled, this, &CameraGUI::on_recordCalibratedMediaCheck_toggled);
    QObject::connect(settingsUI()->recordFilteredMediaCheck, &QCheckBox::toggled, this, &CameraGUI::on_recordFilteredMediaCheck_toggled);
    QObject::connect(settingsUI()->recordPostProcessedMediaCheck, &QCheckBox::toggled, this, &CameraGUI::on_recordPostProcessedMediaCheck_toggled);
    QObject::connect(ui->stackEnabledButton, &QToolButton::toggled, this, &CameraGUI::on_stackEnabledCheck_toggled);
    QObject::connect(settingsUI()->stackFrameCountSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_stackFrameCountSpin_valueChanged);
    QObject::connect(settingsUI()->stackMethodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_stackMethodCombo_currentIndexChanged);
    QObject::connect(settingsUI()->stackHdrAlgorithmCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int index)
        {
            m_settings.m_stackHdrAlgorithm = static_cast<CameraSettings::StackHdrAlgorithm>(index);
            applySetting("stackHdrAlgorithm");
        });
    QObject::connect(settingsUI()->stackHdrExposureCountSpin, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int value)
        {
            m_settings.m_stackHdrExposureCount = value;
            updateHdrStackingControls();
            updateCaptureIntervalWarning();
            applySetting("stackHdrExposureCount");
        });
    for (int exposureIndex = 0; exposureIndex < CameraSettings::m_maxHdrExposureCount; ++exposureIndex)
    {
        QObject::connect(hdrExposureSliders(settingsUI())[exposureIndex], &QSlider::valueChanged, this,
            [this, exposureIndex](int sliderValue) { handleHdrExposureSliderChanged(exposureIndex, sliderValue); });
        QObject::connect(hdrExposureSpins(settingsUI())[exposureIndex], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this, exposureIndex](double value) { handleHdrExposureSpinChanged(exposureIndex, value); });
    }
    QObject::connect(settingsUI()->stackAlignmentCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_stackAlignmentCombo_currentIndexChanged);
    QObject::connect(settingsUI()->stackDisplayModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_stackDisplayModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->stackDisplayFrameSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_stackDisplayFrameSpin_valueChanged);
    QObject::connect(settingsUI()->stackDeleteFrameButton, &QToolButton::clicked, this, &CameraGUI::on_stackDeleteFrameButton_clicked);
    QObject::connect(settingsUI()->stackRejectBadFramesCheck, &QCheckBox::toggled, this, &CameraGUI::on_stackRejectBadFramesCheck_toggled);
    QObject::connect(settingsUI()->scaleEnabledCheck, &QCheckBox::toggled, this, &CameraGUI::on_scaleEnabledCheck_toggled);
    QObject::connect(settingsUI()->scaleWidthSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_scaleWidthSpin_valueChanged);
    QObject::connect(settingsUI()->scaleHeightSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_scaleHeightSpin_valueChanged);
    QObject::connect(settingsUI()->scaleKeepAspectRatioCheck, &QCheckBox::toggled, this, &CameraGUI::on_scaleKeepAspectRatioCheck_toggled);
    QObject::connect(settingsUI()->scaleJustificationCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_scaleJustificationCombo_currentIndexChanged);
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
    QObject::connect(settingsUI()->azimuthOffsetSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_azimuthOffsetSpin_valueChanged);
    QObject::connect(settingsUI()->elevationOffsetSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_elevationOffsetSpin_valueChanged);
    QObject::connect(settingsUI()->rollOffsetSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_rollOffsetSpin_valueChanged);
    QObject::connect(settingsUI()->rollSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_rollSpin_valueChanged);
    QObject::connect(settingsUI()->directionSourceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_directionSourceCombo_currentIndexChanged);
    QObject::connect(settingsUI()->fovModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_fovModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->fovSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_fovSpin_valueChanged);
    QObject::connect(settingsUI()->fovSensorWidthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_fovSensorWidthSpin_valueChanged);
    QObject::connect(settingsUI()->fovSensorHeightSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_fovSensorHeightSpin_valueChanged);
    QObject::connect(settingsUI()->fovFocalLengthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_fovFocalLengthSpin_valueChanged);
    QObject::connect(settingsUI()->lensProjectionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_lensProjectionCombo_currentIndexChanged);
    QObject::connect(settingsUI()->lensCenterOffsetXSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_lensCenterOffsetXSpin_valueChanged);
    QObject::connect(settingsUI()->lensCenterOffsetYSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_lensCenterOffsetYSpin_valueChanged);
    QObject::connect(settingsUI()->lensDistortionK1Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_lensDistortionK1Spin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_postProcessWhiteBalanceModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceRedGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceRedGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceRedGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceRedGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceGreenGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceGreenGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceGreenGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceGreenGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceBlueGainSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceBlueGainSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceBlueGainSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceBlueGainSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceHighlightProtectionSlider, &QSlider::valueChanged, this, &CameraGUI::on_postProcessWhiteBalanceHighlightProtectionSlider_valueChanged);
    QObject::connect(settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_postProcessWhiteBalanceHighlightProtectionSpin_valueChanged);
    QObject::connect(settingsUI()->postProcessUseCudaCheck, &QCheckBox::toggled, this, &CameraGUI::on_postProcessUseCudaCheck_toggled);
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
    QObject::connect(settingsUI()->edgeDisplayModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_edgeDisplayModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->sobelEdgeSlider, &QSlider::valueChanged, this, &CameraGUI::on_sobelEdgeSlider_valueChanged);
    QObject::connect(settingsUI()->sobelEdgeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_sobelEdgeSpin_valueChanged);
    QObject::connect(settingsUI()->cannyEdgeSlider, &QSlider::valueChanged, this, &CameraGUI::on_cannyEdgeSlider_valueChanged);
    QObject::connect(settingsUI()->cannyEdgeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_cannyEdgeSpin_valueChanged);
    QObject::connect(settingsUI()->flipXButton, &QCheckBox::toggled, this, &CameraGUI::on_flipXButton_toggled);
    QObject::connect(settingsUI()->flipYButton, &QCheckBox::toggled, this, &CameraGUI::on_flipYButton_toggled);
    QObject::connect(settingsUI()->imageRotationCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_imageRotationCombo_currentIndexChanged);
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
    QObject::connect(ui->constellationButton, &QToolButton::toggled, this, &CameraGUI::on_constellationCheck_toggled);
    QObject::connect(settingsUI()->constellationOverlayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_constellationOverlayCombo_currentIndexChanged);
    QObject::connect(settingsUI()->constellationColorButton, &QToolButton::clicked, this, &CameraGUI::on_constellationColorButton_clicked);
    QObject::connect(ui->trackObjectsButton, &QToolButton::toggled, this, &CameraGUI::on_trackObjectsCheck_toggled);
    QObject::connect(settingsUI()->trackObjectTrailsCheck, &QCheckBox::toggled, this, &CameraGUI::on_trackObjectTrailsCheck_toggled);
    QObject::connect(settingsUI()->trackObjectHeatMapCheck, &QCheckBox::toggled, this, &CameraGUI::on_trackObjectHeatMapCheck_toggled);
    QObject::connect(settingsUI()->trackObjectClearHeatMapButton, &QToolButton::clicked, this, &CameraGUI::on_trackObjectClearHeatMapButton_clicked);
    QObject::connect(settingsUI()->trackObjectMinElevationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_trackObjectMinElevationSpin_valueChanged);
    QObject::connect(settingsUI()->trackObjectColorButton, &QToolButton::clicked, this, &CameraGUI::on_trackObjectColorButton_clicked);
    QObject::connect(settingsUI()->trackObjectFontCombo, &QFontComboBox::currentFontChanged, this, &CameraGUI::on_trackObjectFontCombo_currentFontChanged);
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
    QObject::connect(ui->detectionHistoryButton, &QToolButton::clicked, this, &CameraGUI::on_detectionHistoryButton_clicked);
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
    QObject::connect(settingsUI()->detectionRoiShowButton, &QToolButton::toggled, this, &CameraGUI::on_detectionRoiShowButton_toggled);
    QObject::connect(settingsUI()->detectionRoiDrawButton, &QToolButton::clicked, this, &CameraGUI::on_detectionRoiDrawButton_clicked);
    QObject::connect(settingsUI()->detectionRoiDeleteButton, &QToolButton::clicked, this, &CameraGUI::on_detectionRoiDeleteButton_clicked);
    QObject::connect(settingsUI()->detectionResetDefaultsButton, &QToolButton::clicked, this, &CameraGUI::on_detectionResetDefaultsButton_clicked);
    QObject::connect(ui->motionDetectButton, &QToolButton::toggled, this, &CameraGUI::on_motionDetectButton_toggled);
    QObject::connect(ui->starDetectButton, &QToolButton::toggled, this, &CameraGUI::on_starDetectButton_toggled);
    QObject::connect(settingsUI()->motionBackgroundSubtractorCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_motionBackgroundSubtractorCombo_currentIndexChanged);
    QObject::connect(settingsUI()->motionMaskViewCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_motionMaskViewCombo_currentIndexChanged);
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
    QObject::connect(settingsUI()->starThresholdSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_starThresholdSpin_valueChanged);
    QObject::connect(settingsUI()->starBackgroundBlurSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_starBackgroundBlurSpin_valueChanged);
    QObject::connect(settingsUI()->starMinAreaSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_starMinAreaSpin_valueChanged);
    QObject::connect(settingsUI()->starMaxAreaSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_starMaxAreaSpin_valueChanged);
    QObject::connect(settingsUI()->starMaxAspectRatioSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_starMaxAspectRatioSpin_valueChanged);
    QObject::connect(settingsUI()->starDebugViewCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_starDebugViewCombo_currentIndexChanged);
    QObject::connect(settingsUI()->plateSolveLabelModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_plateSolveLabelModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->starColorButton, &QToolButton::clicked, this, &CameraGUI::on_starColorButton_clicked);
    QObject::connect(settingsUI()->showStarDetectionBoxesCheck, &QCheckBox::toggled, this, &CameraGUI::on_showStarDetectionBoxesCheck_toggled);
    QObject::connect(settingsUI()->hideSyntheticNamesCheck, &QCheckBox::toggled, this, &CameraGUI::on_hideSyntheticNamesCheck_toggled);
    QObject::connect(settingsUI()->plateSolveMaxMagnitudeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_plateSolveMaxMagnitudeSpin_valueChanged);
    QObject::connect(settingsUI()->plateSolveMinMatchesSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_plateSolveMinMatchesSpin_valueChanged);
    QObject::connect(settingsUI()->plateSolveMatchRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_plateSolveMatchRadiusSpin_valueChanged);
    QObject::connect(settingsUI()->plateSolveFinalMatchRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_plateSolveFinalMatchRadiusSpin_valueChanged);
    QObject::connect(settingsUI()->plateSolveSearchRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_plateSolveSearchRadiusSpin_valueChanged);
    QObject::connect(settingsUI()->plateSolveFovToleranceSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_plateSolveFovToleranceSpin_valueChanged);
    QObject::connect(settingsUI()->plateSolveStartModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_plateSolveStartModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->plateSolveDateTimeModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_plateSolveDateTimeModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->plateSolveDateTimeEdit, &QDateTimeEdit::dateTimeChanged, this, &CameraGUI::on_plateSolveDateTimeEdit_dateTimeChanged);
    QObject::connect(settingsUI()->plateSolveDateTimeUtcButton, &QToolButton::toggled, this, &CameraGUI::on_plateSolveDateTimeUtcButton_toggled);
    QObject::connect(settingsUI()->plateSolveDateTimeNowButton, &QToolButton::clicked, this, &CameraGUI::on_plateSolveDateTimeNowButton_clicked);
    QObject::connect(settingsUI()->plateSolveCatalogSourceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_plateSolveCatalogSourceCombo_currentIndexChanged);
    QObject::connect(settingsUI()->starCatalogDiskCacheSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_starCatalogDiskCacheSizeSpin_valueChanged);
    QObject::connect(settingsUI()->plateSolveApplyModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_plateSolveApplyModeCombo_currentIndexChanged);
    QObject::connect(settingsUI()->plateSolveDownloadCatalogButton, &QToolButton::clicked, this, &CameraGUI::on_plateSolveDownloadCatalogButton_clicked);
    QObject::connect(settingsUI()->plateSolveApplyButton, &QToolButton::clicked, this, &CameraGUI::on_plateSolveApplyButton_clicked);
    QObject::connect(settingsUI()->motionExclusionAddButton, &QToolButton::clicked, this, &CameraGUI::on_motionExclusionAddButton_clicked);
    QObject::connect(settingsUI()->motionExclusionRemoveButton, &QToolButton::clicked, this, &CameraGUI::on_motionExclusionRemoveButton_clicked);
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
        QObject::connect(settingsUI()->yoloModelPathCombo->lineEdit(), &QLineEdit::textChanged, this, [this]() { updateYoloButtonEnabled(); });
    }
    QObject::connect(settingsUI()->yoloModelPathButton, &QPushButton::clicked, this, &CameraGUI::on_yoloModelPathButton_clicked);
    QObject::connect(settingsUI()->yoloLabelsPathCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_yoloLabelsPathCombo_currentIndexChanged);
    if (settingsUI()->yoloLabelsPathCombo->lineEdit()) {
        QObject::connect(settingsUI()->yoloLabelsPathCombo->lineEdit(), &QLineEdit::editingFinished, this, &CameraGUI::on_yoloLabelsPathEdit_editingFinished);
        QObject::connect(settingsUI()->yoloLabelsPathCombo->lineEdit(), &QLineEdit::textChanged, this, [this]() { updateYoloButtonEnabled(); });
    }
    QObject::connect(settingsUI()->yoloLabelsPathButton, &QPushButton::clicked, this, &CameraGUI::on_yoloLabelsPathButton_clicked);
    QObject::connect(settingsUI()->yoloTargetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraGUI::on_yoloTargetCombo_currentIndexChanged);
    QObject::connect(settingsUI()->yoloConfSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_yoloConfSpin_valueChanged);
    QObject::connect(settingsUI()->yoloNmsSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraGUI::on_yoloNmsSpin_valueChanged);
    QObject::connect(settingsUI()->yoloTileLargeImagesCheck, &QCheckBox::toggled, this, &CameraGUI::on_yoloTileLargeImagesCheck_toggled);
    QObject::connect(settingsUI()->yoloTileOverlapSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &CameraGUI::on_yoloTileOverlapSpin_valueChanged);
    QObject::connect(settingsUI()->yoloIgnoredClassNamesEdit, &QPlainTextEdit::textChanged, this, &CameraGUI::on_yoloIgnoredClassNamesEdit_textChanged);
    QObject::connect(settingsUI()->yoloBoxColorButton, &QToolButton::clicked, this, &CameraGUI::on_yoloBoxColorButton_clicked);
    QObject::connect(ui->zoomInButton, &QToolButton::clicked, this, &CameraGUI::on_zoomInButton_clicked);
    QObject::connect(ui->zoomOutButton, &QToolButton::clicked, this, &CameraGUI::on_zoomOutButton_clicked);
    QObject::connect(ui->fitInViewButton, &QToolButton::clicked, this, &CameraGUI::on_fitInViewButton_clicked);
    QObject::connect(ui->audioMute, &QToolButton::toggled, this, &CameraGUI::on_audioMute_toggled);
    QObject::connect(ui->audioPreviewVolumeDial, &QDial::valueChanged, this, &CameraGUI::on_audioPreviewVolumeDial_valueChanged);
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

    const QString selectedResolution = settingsUI()->resolutionCombo->currentText();
    const QStringList parts = selectedResolution.split('x');
    if (parts.size() == 2)
    {
        bool okWidth = false;
        bool okHeight = false;
        const int width = parts.at(0).trimmed().toInt(&okWidth);
        const int height = parts.at(1).trimmed().toInt(&okHeight);
        if (okWidth && okHeight && (width > 0) && (height > 0))
        {
            m_settings.m_resolutionWidth = width;
            m_settings.m_resolutionHeight = height;
        }
    }

    updateFrameRateControlForResolution(selectedResolution);
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
    updateCaptureIntervalWarning();
}

void CameraGUI::updateCaptureIntervalWarning()
{
    const bool intervalMode = m_settings.isAlpacaCamera() || m_settings.isIntervalCaptureMode();
    double exposureTimeMs = std::max(CameraSettings::m_minExposureTimeMs, m_settings.m_exposureTimeMs);

    if (m_settings.isHdrStackingEnabled())
    {
        for (int exposureIndex = 0; exposureIndex < m_settings.getHdrExposureCount(); ++exposureIndex)
        {
            exposureTimeMs = std::max(exposureTimeMs, m_settings.getHdrExposureTimeMs(exposureIndex));
        }
    }

    const double intervalMs = m_settings.getCaptureIntervalSeconds() * 1000.0;
    const bool intervalTooShort = intervalMode && (intervalMs < exposureTimeMs);

    settingsUI()->intervalSpin->setStyleSheet(intervalTooShort
        ? QStringLiteral("QDoubleSpinBox { background-color: #ff0000; }")
        : QString());
    settingsUI()->intervalSpin->setToolTip(intervalTooShort
        ? tr("Interval is shorter than the exposure time")
        : tr("Interval between still-image captures"));
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

    updateHdrExposureControls();
    updateCaptureIntervalWarning();
}

void CameraGUI::updateVideoFileControls()
{
    const int comboIndex = ui->cameraCombo->currentIndex();
    const QString comboProtocol = comboIndex >= 0
        ? ui->cameraCombo->itemData(comboIndex, CameraProtocolRole).toString()
        : QString();
    const bool comboFileCameraSelected = CameraProtocol::isPlaybackSource(comboProtocol);
    const bool fileCameraSelected = m_settings.isFileCamera() || comboFileCameraSelected;
    const bool imageSequenceSelected =
        (comboProtocol == CameraProtocol::images()) || m_settings.isImageFileSequenceCamera();
    const bool streamSelected =
        (comboProtocol == CameraProtocol::stream()) || m_settings.isStreamCamera();
    const bool hasVideoFile = fileCameraSelected && m_settings.hasFileCameraSource();
    const qint64 playbackDurationMs = imageSequenceSelected ? imageSequenceDurationMs() : m_playbackDurationMs;
    const bool hasPlaybackPosition = hasVideoFile && !streamSelected && (playbackDurationMs > 0);

    if (fileCameraSelected)
    {
        QSignalBlocker blocker(ui->playbackRateSpin);
        ui->playbackRateSpin->setDecimals(imageSequenceSelected ? 1 : 2);
        ui->playbackRateSpin->setMinimum(0.1);
        ui->playbackRateSpin->setMaximum(CameraSettings::m_maxVideoPlaybackRate);
        ui->playbackRateSpin->setSuffix(imageSequenceSelected ? tr(" fps") : QString());
        ui->playbackRateSpin->setToolTip(imageSequenceSelected
            ? tr("Image sequence playback frames per second")
            : tr("Video playback rate"));
    }
    ui->browseVideoFileButton->setToolTip(imageSequenceSelected
        ? tr("Edit image sequence files")
        : (streamSelected ? tr("Edit stream URL") : tr("Select video file")));

    setVisibleEnabled(ui->browseVideoFileButton, fileCameraSelected, fileCameraSelected);
    setVisibleEnabled(ui->restartVideo, fileCameraSelected && !streamSelected, hasVideoFile);
    setVisibleEnabled(ui->stepBackVideo, fileCameraSelected && !streamSelected, hasVideoFile);
    setVisibleEnabled(ui->stepForwardVideo, fileCameraSelected && !streamSelected, hasVideoFile);
    setVisibleEnabled(ui->playPauseVideo, fileCameraSelected, hasVideoFile);
    setVisibleEnabled(ui->loopVideo, fileCameraSelected && !streamSelected, hasVideoFile);
    setVisibleEnabled(ui->playbackRateSpin, fileCameraSelected && !streamSelected, hasVideoFile);
    setVisibleEnabled(ui->playbackPositionSlider, fileCameraSelected, hasPlaybackPosition);
    setVisibleEnabled(ui->playbackPositionLabel, fileCameraSelected, hasPlaybackPosition);
    ui->videoLine->setVisible(fileCameraSelected);
    if (!fileCameraSelected || !hasVideoFile) {
        ui->playbackPositionLabel->setText(imageSequenceSelected ? QStringLiteral("0/0") : QStringLiteral("00:00:00"));
    }
    updateVideoPreRecordBufferMemoryLabel();
}

void CameraGUI::updateVideoPreRecordBufferMemoryLabel()
{
    if (!m_settingsDialog) {
        return;
    }

    const int width = m_lastImage.isNull() ? std::max(0, m_settings.m_resolutionWidth) : m_lastImage.width();
    const int height = m_lastImage.isNull() ? std::max(0, m_settings.m_resolutionHeight) : m_lastImage.height();
    const int seconds = std::max(0, m_settings.m_videoPreRecordBufferSeconds);
    const int streams = (m_settings.m_recordCalibratedMedia ? 1 : 0)
        + (m_settings.m_recordFilteredMedia ? 1 : 0)
        + (m_settings.m_recordPostProcessedMedia ? 1 : 0);
    const double frameRate = std::max(0.0, m_settings.getCaptureFrameRate());
    const double bytes = static_cast<double>(width) * static_cast<double>(height) * 3.0
        * frameRate * static_cast<double>(seconds) * static_cast<double>(streams);
    const double mib = bytes / (1024.0 * 1024.0);

    settingsUI()->videoPreRecordBufferMemoryLabel->setText(QStringLiteral("%1 MiB").arg(mib, 0, 'f', mib < 10.0 ? 1 : 0));
}

void CameraGUI::updateHdrExposureControls()
{
    if (!m_settingsDialog) {
        return;
    }

    const double minimum = m_exposureMinimumMs;
    const double maximum = std::max(minimum, m_exposureMaximumMs);
    const double singleStep = std::max(0.000001, m_exposureStepMs);
    const int decimals = decimalsForStepSize(singleStep);
    const auto labels = hdrExposureLabels(settingsUI());
    const auto sliders = hdrExposureSliders(settingsUI());
    const auto spins = hdrExposureSpins(settingsUI());

    for (int exposureIndex = 0; exposureIndex < CameraSettings::m_maxHdrExposureCount; ++exposureIndex)
    {
        const double value = std::max(minimum, m_settings.getHdrExposureTimeMs(exposureIndex));
        const double controlMaximum = std::max(maximum, value);

        labels[exposureIndex]->setText(tr("Exposure %1").arg(exposureIndex + 1));

        {
            QSignalBlocker blocker(spins[exposureIndex]);
            spins[exposureIndex]->setDecimals(decimals);
            spins[exposureIndex]->setSingleStep(singleStep);
            spins[exposureIndex]->setMinimum(minimum);
            spins[exposureIndex]->setMaximum(controlMaximum);
            spins[exposureIndex]->setValue(value);
        }

        {
            QSignalBlocker blocker(sliders[exposureIndex]);
            sliders[exposureIndex]->setMinimum(0);
            sliders[exposureIndex]->setMaximum(1000);
            sliders[exposureIndex]->setValue(exposureValueToSlider(spins[exposureIndex], value));
        }
    }
}

bool CameraGUI::isHdrStackingSupported() const
{
    if (m_settings.isFileCamera()) {
        return false;
    }

    if (m_settings.isQtCamera()) {
        return m_qtManualExposureSupported && m_settings.isIntervalCaptureMode();
    }

    if (m_settings.isAsiCamera()) {
        return m_settings.isIntervalCaptureMode();
    }

    if (m_settings.isAlpacaCamera()) {
        return true;
    }

    return false;
}

void CameraGUI::updateHdrStackingControls()
{
    if (!m_settingsDialog) {
        return;
    }

    QStandardItemModel *stackMethodModel = qobject_cast<QStandardItemModel*>(settingsUI()->stackMethodCombo->model());
    if (stackMethodModel)
    {
        QStandardItem *hdrItem = stackMethodModel->item(static_cast<int>(CameraSettings::StackMethodHDR));
        if (hdrItem) {
            hdrItem->setEnabled(isHdrStackingSupported());
        }
    }

    if ((m_settings.m_stackMethod == CameraSettings::StackMethodHDR) && !isHdrStackingSupported())
    {
        {
            QSignalBlocker blocker(settingsUI()->stackMethodCombo);
            settingsUI()->stackMethodCombo->setCurrentIndex(static_cast<int>(CameraSettings::StackMethodAverage));
        }
        m_settings.m_stackMethod = CameraSettings::StackMethodAverage;
        applySetting("stackMethod");
    }

    const bool hdrSelected = (m_settings.m_stackMethod == CameraSettings::StackMethodHDR);
    const bool hdrControlsEnabled = hdrSelected && isHdrStackingSupported();
    const int visibleExposureRows = hdrSelected ? m_settings.getHdrExposureCount() : 0;
    const auto labels = hdrExposureLabels(settingsUI());
    const auto sliders = hdrExposureSliders(settingsUI());
    const auto spins = hdrExposureSpins(settingsUI());

    settingsUI()->stackFrameCountLabel->setVisible(!hdrSelected);
    settingsUI()->stackFrameCountSpin->setVisible(!hdrSelected);
    settingsUI()->stackHdrExposureCountLabel->setVisible(hdrSelected);
    settingsUI()->stackHdrExposureCountSpin->setVisible(hdrSelected);
    settingsUI()->stackHdrExposureCountLabel->setEnabled(hdrControlsEnabled);
    settingsUI()->stackHdrExposureCountSpin->setEnabled(hdrControlsEnabled);
    settingsUI()->stackHdrAlgorithmLabel->setVisible(hdrSelected);
    settingsUI()->stackHdrAlgorithmCombo->setVisible(hdrSelected);
    settingsUI()->stackHdrAlgorithmLabel->setEnabled(hdrControlsEnabled);
    settingsUI()->stackHdrAlgorithmCombo->setEnabled(hdrControlsEnabled);

    for (int exposureIndex = 0; exposureIndex < CameraSettings::m_maxHdrExposureCount; ++exposureIndex)
    {
        const bool visible = hdrSelected && (exposureIndex < visibleExposureRows);
        labels[exposureIndex]->setVisible(visible);
        sliders[exposureIndex]->setVisible(visible);
        spins[exposureIndex]->setVisible(visible);
        labels[exposureIndex]->setEnabled(hdrControlsEnabled);
        sliders[exposureIndex]->setEnabled(hdrControlsEnabled);
        spins[exposureIndex]->setEnabled(hdrControlsEnabled);
    }
}

void CameraGUI::updateScaleControls()
{
    if (!m_settingsDialog) {
        return;
    }

    const bool enabled = m_settings.m_scaleEnabled;
    settingsUI()->scaleWidthLabel->setEnabled(enabled);
    settingsUI()->scaleWidthSpin->setEnabled(enabled);
    settingsUI()->scaleOutputWidthLabel->setEnabled(enabled);
    settingsUI()->scaleHeightLabel->setEnabled(enabled);
    settingsUI()->scaleHeightSpin->setEnabled(enabled);
    settingsUI()->scaleOutputHeightLabel->setEnabled(enabled);
    settingsUI()->scaleKeepAspectRatioLabel->setEnabled(enabled);
    settingsUI()->scaleKeepAspectRatioCheck->setEnabled(enabled);
    settingsUI()->scaleJustificationLabel->setEnabled(enabled && m_settings.m_scaleKeepAspectRatio);
    settingsUI()->scaleJustificationCombo->setEnabled(enabled && m_settings.m_scaleKeepAspectRatio);
    settingsUI()->scaleCenterOffsetTitleLabel->setEnabled(enabled && m_settings.m_scaleKeepAspectRatio);
    settingsUI()->scaleCenterOffsetLabel->setEnabled(enabled && m_settings.m_scaleKeepAspectRatio);

    int inputWidth = m_settings.m_cameraNumX > 0 ? m_settings.m_cameraNumX : m_settings.m_resolutionWidth;
    int inputHeight = m_settings.m_cameraNumY > 0 ? m_settings.m_cameraNumY : m_settings.m_resolutionHeight;
    if ((inputWidth <= 0 || inputHeight <= 0) && !m_settings.m_scaleEnabled && !m_lastImage.isNull())
    {
        inputWidth = m_lastImage.width();
        inputHeight = m_lastImage.height();
    }

    QSize outputSize;
    if ((inputWidth > 0) && (inputHeight > 0))
    {
        outputSize = QSize(inputWidth, inputHeight);
        if (m_settings.m_scaleEnabled && (m_settings.m_scaleWidth > 0) && (m_settings.m_scaleHeight > 0)) {
            outputSize = QSize(m_settings.m_scaleWidth, m_settings.m_scaleHeight);
        }
    }
    else if (m_settings.m_scaleEnabled && (m_settings.m_scaleWidth > 0) && (m_settings.m_scaleHeight > 0))
    {
        outputSize = QSize(m_settings.m_scaleWidth, m_settings.m_scaleHeight);
    }

    if (outputSize.isValid())
    {
        settingsUI()->scaleOutputWidthLabel->setText(tr("Output: %1 px").arg(outputSize.width()));
        settingsUI()->scaleOutputHeightLabel->setText(tr("Output: %1 px").arg(outputSize.height()));

        if (enabled && m_settings.m_scaleKeepAspectRatio && (inputWidth > 0) && (inputHeight > 0))
        {
            const double scale = std::min(
                static_cast<double>(outputSize.width()) / static_cast<double>(inputWidth),
                static_cast<double>(outputSize.height()) / static_cast<double>(inputHeight));
            const QSize contentSize(
                std::max(1, static_cast<int>(std::lround(static_cast<double>(inputWidth) * scale))),
                std::max(1, static_cast<int>(std::lround(static_cast<double>(inputHeight) * scale))));
            const int extraWidth = std::max(0, outputSize.width() - contentSize.width());
            const int extraHeight = std::max(0, outputSize.height() - contentSize.height());
            int x = extraWidth / 2;
            int y = extraHeight / 2;

            switch (m_settings.m_scaleJustification)
            {
            case CameraSettings::ScaleJustifyLeft:
                x = 0;
                break;
            case CameraSettings::ScaleJustifyRight:
                x = extraWidth;
                break;
            case CameraSettings::ScaleJustifyTop:
                y = 0;
                break;
            case CameraSettings::ScaleJustifyBottom:
                y = extraHeight;
                break;
            case CameraSettings::ScaleJustifyCenter:
            default:
                break;
            }

            const double centerOffsetX = static_cast<double>(x) + static_cast<double>(contentSize.width()) * 0.5
                - static_cast<double>(outputSize.width()) * 0.5;
            const double centerOffsetY = static_cast<double>(y) + static_cast<double>(contentSize.height()) * 0.5
                - static_cast<double>(outputSize.height()) * 0.5;
            settingsUI()->scaleCenterOffsetLabel->setText(tr("x: %1 px, y: %2 px")
                .arg(centerOffsetX, 0, 'f', 1)
                .arg(centerOffsetY, 0, 'f', 1));
        }
        else
        {
            settingsUI()->scaleCenterOffsetLabel->setText(tr("x: 0.0 px, y: 0.0 px"));
        }
    }
    else
    {
        settingsUI()->scaleOutputWidthLabel->setText(tr("Output: -"));
        settingsUI()->scaleOutputHeightLabel->setText(tr("Output: -"));
        settingsUI()->scaleCenterOffsetLabel->setText(tr("x: -, y: -"));
    }
}

bool CameraGUI::isHdrStackingActiveForQt() const
{
    return m_settings.isHdrStackingEnabled() && isHdrStackingSupported() && m_settings.isQtCamera();
}

void CameraGUI::resetQtHdrBracketState()
{
    m_qtHdrExposureIndex = 0;
}

double CameraGUI::currentQtCaptureExposureTimeMs() const
{
    return isHdrStackingActiveForQt()
        ? m_settings.getHdrExposureTimeMs(currentQtHdrExposureIndex())
        : std::max(CameraSettings::m_minExposureTimeMs, m_settings.m_exposureTimeMs);
}

int CameraGUI::currentQtHdrExposureIndex() const
{
    return isHdrStackingActiveForQt() ? qBound(0, m_qtHdrExposureIndex, currentQtHdrExposureCount() - 1) : -1;
}

int CameraGUI::currentQtHdrExposureCount() const
{
    return isHdrStackingActiveForQt() ? m_settings.getHdrExposureCount() : 0;
}

void CameraGUI::advanceQtHdrBracketState()
{
    if (!isHdrStackingActiveForQt()) {
        return;
    }

    const int hdrExposureCount = currentQtHdrExposureCount();
    m_qtHdrExposureIndex = (m_qtHdrExposureIndex + 1) % std::max(1, hdrExposureCount);
}

void CameraGUI::applyQtExposureTimeMs(double exposureTimeMs)
{
    const double clampedExposureMs = qBound(m_exposureMinimumMs, exposureTimeMs, m_exposureMaximumMs);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_qtCamera) {
        m_qtCamera->setExposureMode(QCamera::ExposureManual);
        m_qtCamera->setManualExposureTime(static_cast<float>(clampedExposureMs) / 1000.0f);
    }
#else
    if (m_qtCamera)
    {
        QCameraExposure *exposure = m_qtCamera->exposure();
        if (exposure)
        {
            exposure->setExposureMode(QCameraExposure::ExposureManual);
            exposure->setManualShutterSpeed(static_cast<qreal>(clampedExposureMs) / 1000.0);
        }
    }
#endif
}

void CameraGUI::populateFrameExposureMetadata(CameraPipelineFrame& frame, double exposureTimeMs, int hdrExposureIndex, int hdrExposureCount, const QDateTime& captureDateTime) const
{
    frame.m_captureDateTime = captureDateTime.isValid() ? captureDateTime : QDateTime::currentDateTime();
    frame.m_exposureTimeMs = std::max(CameraSettings::m_minExposureTimeMs, exposureTimeMs);
    frame.m_hdrExposureIndex = hdrExposureIndex;
    frame.m_hdrExposureCount = hdrExposureCount;
}

void CameraGUI::handleHdrExposureSliderChanged(int exposureIndex, int sliderValue)
{
    const auto spins = hdrExposureSpins(settingsUI());
    const double exposureTimeMs = sliderToExposureValue(spins[exposureIndex], sliderValue);

    {
        QSignalBlocker blocker(spins[exposureIndex]);
        spins[exposureIndex]->setValue(exposureTimeMs);
    }

    m_settings.m_stackHdrExposureTimesMs[static_cast<size_t>(exposureIndex)] = exposureTimeMs;
    updateCaptureIntervalWarning();
    applySetting(QStringLiteral("stackHdrExposure%1Ms").arg(exposureIndex + 1));
}

void CameraGUI::handleHdrExposureSpinChanged(int exposureIndex, double value)
{
    const auto spins = hdrExposureSpins(settingsUI());
    const auto sliders = hdrExposureSliders(settingsUI());
    const double exposureTimeMs = qBound(m_exposureMinimumMs, value, spins[exposureIndex]->maximum());

    {
        QSignalBlocker blocker(sliders[exposureIndex]);
        sliders[exposureIndex]->setValue(exposureValueToSlider(spins[exposureIndex], exposureTimeMs));
    }

    m_settings.m_stackHdrExposureTimesMs[static_cast<size_t>(exposureIndex)] = exposureTimeMs;
    updateCaptureIntervalWarning();
    applySetting(QStringLiteral("stackHdrExposure%1Ms").arg(exposureIndex + 1));
}

void CameraGUI::populateDirectionSourceCombo()
{
    QComboBox *combo = settingsUI()->directionSourceCombo;
    QString currentSelection;
    if (!m_settings.m_rotator.isEmpty()) {
        currentSelection = rotatorDirectionSourceId(m_settings.m_rotator);
    } else if (!m_settings.m_directionSensor.isEmpty()) {
        currentSelection = sensorDirectionSourceId(m_settings.m_directionSensor);
    }

    QSignalBlocker blocker(combo);
    combo->clear();
    combo->addItem(tr("None"), QString());

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
            combo->addItem(
                tr("Rotator F%1:%2 %3").arg(featureSetIndex).arg(featureIndex).arg(title),
                rotatorDirectionSourceId(selectionId));
        }
    }

    bool compatibleDirectionSensors = false;

#ifdef QT_SENSORS_FOUND
    const QList<QByteArray> compassSensors = QSensor::sensorsForType(QCompass::sensorType);
    const QList<QByteArray> rotationSensors = QSensor::sensorsForType(QRotationSensor::sensorType);
    const QList<QByteArray> tiltSensors = QSensor::sensorsForType(QTiltSensor::sensorType);
    compatibleDirectionSensors = !compassSensors.isEmpty() && !rotationSensors.isEmpty() && !tiltSensors.isEmpty();

    if (compatibleDirectionSensors)
    {
        combo->addItem(tr("Sensor default compass + tilt + rotation"), sensorDirectionSourceId(kDefaultDirectionSensorId));

        for (const QByteArray& identifier : compassSensors)
        {
            const QString id = QString::fromUtf8(identifier);
            combo->addItem(tr("Sensor compass %1 + default tilt/rotation").arg(id), sensorDirectionSourceId(id));
        }
    }
    else
    {
        combo->addItem(tr("No compatible Qt Sensors"), QString());
    }
#else
    combo->addItem(tr("Qt Sensors unavailable"), QString());
#endif

    int index = combo->findData(currentSelection);
    if ((index < 0) && !currentSelection.isEmpty())
    {
        combo->addItem(tr("Unavailable: %1").arg(
                !m_settings.m_rotator.isEmpty() ? m_settings.m_rotator : m_settings.m_directionSensor),
            currentSelection);
        index = combo->count() - 1;
    }
    combo->setCurrentIndex(index >= 0 ? index : 0);
    combo->setToolTip(tr("Select a GS232Controller rotator or Qt Sensors compass/tilt/rotation sensor set to continually synchronize the camera direction."));
    settingsUI()->directionSourceLabel->setEnabled(combo->isEnabled());

    if (compatibleDirectionSensors && !m_settings.m_directionSensor.isEmpty()) {
        startDirectionSensors();
    } else {
        stopDirectionSensors();
    }
}

void CameraGUI::startDirectionSensors()
{
#ifdef QT_SENSORS_FOUND
    stopDirectionSensors();

    if (m_settings.m_directionSensor.isEmpty()) {
        return;
    }

    m_directionCompassReadingValid = false;
    m_directionRotationReadingValid = false;
    m_directionTiltReadingValid = false;
    m_directionCompassSensor = new QCompass(this);
    m_directionRotationSensor = new QRotationSensor(this);
    m_directionTiltSensor = new QTiltSensor(this);

    if (m_settings.m_directionSensor != kDefaultDirectionSensorId) {
        m_directionCompassSensor->setIdentifier(m_settings.m_directionSensor.toUtf8());
    }

    connect(m_directionCompassSensor, &QSensor::readingChanged, this, &CameraGUI::syncFromDirectionSensors);
    connect(m_directionRotationSensor, &QSensor::readingChanged, this, &CameraGUI::syncFromDirectionSensors);
    connect(m_directionTiltSensor, &QSensor::readingChanged, this, &CameraGUI::syncFromDirectionSensors);

    const bool compassStarted = m_directionCompassSensor->start();
    const bool rotationStarted = m_directionRotationSensor->start();
    const bool tiltStarted = m_directionTiltSensor->start();

    if (!compassStarted || !rotationStarted || !tiltStarted)
    {
        qWarning() << "CameraGUI: failed to start Qt direction sensors"
            << "compass" << compassStarted
            << "rotation" << rotationStarted
            << "tilt" << tiltStarted
            << "sensor" << m_settings.m_directionSensor;
        stopDirectionSensors();
        return;
    }

    syncFromDirectionSensors();
#endif
}

void CameraGUI::stopDirectionSensors()
{
#ifdef QT_SENSORS_FOUND
    if (m_directionCompassSensor)
    {
        m_directionCompassSensor->stop();
        delete m_directionCompassSensor;
        m_directionCompassSensor = nullptr;
    }
    if (m_directionRotationSensor)
    {
        m_directionRotationSensor->stop();
        delete m_directionRotationSensor;
        m_directionRotationSensor = nullptr;
    }
    if (m_directionTiltSensor)
    {
        m_directionTiltSensor->stop();
        delete m_directionTiltSensor;
        m_directionTiltSensor = nullptr;
    }
    m_directionCompassReadingValid = false;
    m_directionRotationReadingValid = false;
    m_directionTiltReadingValid = false;
#endif
}

void CameraGUI::syncFromDirectionSensors()
{
#ifdef QT_SENSORS_FOUND
    if (m_settings.m_directionSensor.isEmpty()
        || !m_directionCompassSensor
        || !m_directionRotationSensor
        || !m_directionTiltSensor)
    {
        return;
    }

    double azimuth = m_settings.m_azimuth;
    double elevation = m_settings.m_elevation;
    double roll = m_settings.m_roll;

    if (const QCompassReading *compassReading = m_directionCompassSensor->reading())
    {
        const double readingAzimuth = compassReading->azimuth();
        if (std::isfinite(readingAzimuth))
        {
            azimuth = readingAzimuth;
            m_directionCompassReadingValid = true;
        }
    }

    if (const QRotationReading *rotationReading = m_directionRotationSensor->reading())
    {
        const double readingRoll = rotationReading->z();
        if (std::isfinite(readingRoll))
        {
            roll = readingRoll;
            m_directionRotationReadingValid = true;
        }
    }

    if (const QTiltReading *tiltReading = m_directionTiltSensor->reading())
    {
        const double xRotation = tiltReading->xRotation();
        const double yRotation = tiltReading->yRotation();
        if (std::isfinite(xRotation) && std::isfinite(yRotation))
        {
            // Qt tilt is relative to a face-up device. Treat face-up as zenith,
            // and a 90-degree tilt as horizon. This gives a useful camera
            // elevation estimate for phones/tablets exposing compass + tilt.
            const double tiltDegrees = std::min(180.0, std::hypot(xRotation, yRotation));
            elevation = qBound(
                static_cast<double>(CameraSettings::m_minElevation),
                90.0 - tiltDegrees,
                static_cast<double>(CameraSettings::m_maxElevation));
            m_directionTiltReadingValid = true;
        }
    }

    if (!m_directionCompassReadingValid || !m_directionRotationReadingValid || !m_directionTiltReadingValid) {
        return;
    }

    azimuth += m_settings.m_azimuthOffset;
    elevation += m_settings.m_elevationOffset;
    roll += m_settings.m_rollOffset;

    azimuth = std::fmod(azimuth, 360.0);
    if (azimuth < 0.0) {
        azimuth += 360.0;
    }
    roll = normalizeSignedDegrees(roll);
    elevation = qBound(
        static_cast<double>(CameraSettings::m_minElevation),
        elevation,
        static_cast<double>(CameraSettings::m_maxElevation));

    if ((std::fabs(static_cast<double>(m_settings.m_azimuth) - azimuth) < 0.05)
        && (std::fabs(static_cast<double>(m_settings.m_elevation) - elevation) < 0.05)
        && (std::fabs(static_cast<double>(m_settings.m_roll) - roll) < 0.05))
    {
        return;
    }

    m_settings.m_azimuth = static_cast<float>(azimuth);
    m_settings.m_elevation = static_cast<float>(elevation);
    m_settings.m_roll = static_cast<float>(roll);

    {
        QSignalBlocker azimuthBlocker(settingsUI()->azimuthSpin);
        QSignalBlocker elevationBlocker(settingsUI()->elevationSpin);
        QSignalBlocker rollBlocker(settingsUI()->rollSpin);
        settingsUI()->azimuthSpin->setValue(azimuth);
        settingsUI()->elevationSpin->setValue(elevation);
        settingsUI()->rollSpin->setValue(roll);
    }

    applySettings({"azimuth", "elevation", "roll"});
#endif
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
    const bool rotatorSynced = !m_settings.m_rotator.isEmpty();
    const bool sensorSynced = !m_settings.m_directionSensor.isEmpty();
    const bool azElSynced = rotatorSynced || sensorSynced;
    settingsUI()->latitudeSpin->setReadOnly(m_settings.m_positionSync);
    settingsUI()->longitudeSpin->setReadOnly(m_settings.m_positionSync);
    settingsUI()->altitudeSpin->setReadOnly(m_settings.m_positionSync);
    settingsUI()->azimuthSpin->setReadOnly(azElSynced);
    settingsUI()->elevationSpin->setReadOnly(azElSynced);
    settingsUI()->rollSpin->setReadOnly(sensorSynced);
    settingsUI()->azimuthOffsetSpin->setEnabled(azElSynced);
    settingsUI()->elevationOffsetSpin->setEnabled(azElSynced);
    settingsUI()->rollOffsetSpin->setEnabled(sensorSynced);
    settingsUI()->azimuthOffsetLabel->setEnabled(azElSynced);
    settingsUI()->elevationOffsetLabel->setEnabled(azElSynced);
    settingsUI()->rollOffsetLabel->setEnabled(sensorSynced);
#ifdef QT_SENSORS_FOUND
    settingsUI()->directionSourceCombo->setEnabled(true);
#else
    settingsUI()->directionSourceCombo->setEnabled(true);
#endif
    settingsUI()->directionSourceLabel->setEnabled(settingsUI()->directionSourceCombo->isEnabled());
}

void CameraGUI::updateFovControls()
{
    const bool calculateFov = m_settings.m_fovMode == CameraSettings::FovModeSensorFocalLength;
    settingsUI()->fovSpin->setReadOnly(calculateFov);
    settingsUI()->fovSensorWidthLabel->setEnabled(calculateFov);
    settingsUI()->fovSensorWidthSpin->setEnabled(calculateFov);
    settingsUI()->fovSensorHeightLabel->setEnabled(calculateFov);
    settingsUI()->fovSensorHeightSpin->setEnabled(calculateFov);
    settingsUI()->fovFocalLengthLabel->setEnabled(calculateFov);
    settingsUI()->fovFocalLengthSpin->setEnabled(calculateFov);

    if (calculateFov) {
        updateCalculatedFov();
    }
}

void CameraGUI::updateCalculatedFov()
{
    if (m_settings.m_fovMode != CameraSettings::FovModeSensorFocalLength) {
        return;
    }

    const double sensorLongEdgeMm = std::max(m_settings.m_fovSensorWidthMm, m_settings.m_fovSensorHeightMm);
    if ((sensorLongEdgeMm <= 0.0) || (m_settings.m_fovFocalLengthMm <= 0.0)) {
        return;
    }

    constexpr double radiansToDegrees = 180.0 / 3.14159265358979323846;
    const double fovDegrees = 2.0 * std::atan(sensorLongEdgeMm / (2.0 * m_settings.m_fovFocalLengthMm)) * radiansToDegrees;
    m_settings.m_fov = static_cast<float>(qBound(
        static_cast<double>(CameraSettings::m_minFov),
        fovDegrees,
        static_cast<double>(CameraSettings::m_maxFov)));

    settingsUI()->fovSpin->blockSignals(true);
    settingsUI()->fovSpin->setValue(m_settings.m_fov);
    settingsUI()->fovSpin->blockSignals(false);
    if (m_doApplySettings) {
        applySetting("fov");
    }
}

void CameraGUI::updateKeogramPreview(const QImage& image, const QString& fileName, bool visible)
{
    if (!visible || image.isNull())
    {
        if (m_keogramPreviewDialog) {
            m_keogramPreviewDialog->hide();
        }
        return;
    }

    if (!m_keogramPreviewDialog)
    {
        m_keogramPreviewDialog = new QDialog(this);
        m_keogramPreviewDialog->setWindowTitle(tr("Keogram"));
        m_keogramPreviewDialog->resize(640, 360);
        QVBoxLayout *layout = new QVBoxLayout(m_keogramPreviewDialog);
        m_keogramPreviewLabel = new QLabel(m_keogramPreviewDialog);
        m_keogramPreviewLabel->setAlignment(Qt::AlignCenter);
        m_keogramPreviewLabel->setMinimumSize(320, 180);
        m_keogramPreviewLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        layout->addWidget(m_keogramPreviewLabel);
    }

    if (!fileName.isEmpty()) {
        m_keogramPreviewDialog->setWindowTitle(tr("Keogram - %1").arg(QFileInfo(fileName).fileName()));
    }

    const QSize targetSize = m_keogramPreviewLabel->size().isValid()
        ? m_keogramPreviewLabel->size()
        : QSize(640, 360);
    m_keogramPreviewLabel->setPixmap(QPixmap::fromImage(image).scaled(
        targetSize,
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation));
    m_keogramPreviewDialog->show();
}

void CameraGUI::initialiseYouTubeBitrateCombo()
{
    QComboBox *combo = settingsUI()->youtubeStreamBitrateCombo;
    populateBitratePresetCombo(combo);
    updateYouTubeBitrateCombo();
}

void CameraGUI::initialiseVideoRecordBitrateCombo()
{
    QComboBox *combo = settingsUI()->videoRecordBitrateCombo;
    combo->clear();
    combo->addItem(QStringLiteral("Auto"), 0);
    populateBitratePresetCombo(combo, false);
    updateVideoRecordBitrateCombo();
}

void CameraGUI::updateVideoRecordBitrateCombo()
{
    QComboBox *combo = settingsUI()->videoRecordBitrateCombo;
    QSignalBlocker blocker(combo);
    const int index = combo->findData(m_settings.m_videoRecordBitrateKbps);
    if (index >= 0) {
        combo->setCurrentIndex(index);
    } else {
        combo->setCurrentText(videoRecordBitrateText(m_settings.m_videoRecordBitrateKbps));
    }
}

void CameraGUI::applyVideoRecordBitrateComboText()
{
    const int bitrateKbps = parseVideoRecordBitrateKbps(
        settingsUI()->videoRecordBitrateCombo->currentText(),
        m_settings.m_videoRecordBitrateKbps);
    if (bitrateKbps == m_settings.m_videoRecordBitrateKbps)
    {
        updateVideoRecordBitrateCombo();
        return;
    }

    m_settings.m_videoRecordBitrateKbps = bitrateKbps;
    updateVideoRecordBitrateCombo();
    applySetting("videoRecordBitrateKbps");
}

void CameraGUI::updateYouTubeBitrateCombo()
{
    QComboBox *combo = settingsUI()->youtubeStreamBitrateCombo;
    QSignalBlocker blocker(combo);
    const int index = combo->findData(m_settings.m_youtubeStreamBitrateKbps);
    if (index >= 0) {
        combo->setCurrentIndex(index);
    } else {
        combo->setCurrentText(bitrateText(m_settings.m_youtubeStreamBitrateKbps));
    }
}

void CameraGUI::applyYouTubeBitrateComboText()
{
    const int bitrateKbps = parseBitrateKbps(
        settingsUI()->youtubeStreamBitrateCombo->currentText(),
        m_settings.m_youtubeStreamBitrateKbps);
    if (bitrateKbps == m_settings.m_youtubeStreamBitrateKbps)
    {
        updateYouTubeBitrateCombo();
        return;
    }

    m_settings.m_youtubeStreamBitrateKbps = bitrateKbps;
    updateYouTubeBitrateCombo();
    applySetting("youtubeStreamBitrateKbps");
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
    if ((!ChannelWebAPIUtils::getFeatureReportValue(indices.first, indices.second, "currentAzimuth", azimuth)
            || !ChannelWebAPIUtils::getFeatureReportValue(indices.first, indices.second, "currentElevation", elevation))
        && (!ChannelWebAPIUtils::getFeatureSetting(indices.first, indices.second, "azimuth", azimuth)
            || !ChannelWebAPIUtils::getFeatureSetting(indices.first, indices.second, "elevation", elevation)))
    {
        return;
    }

    azimuth = std::fmod(azimuth + m_settings.m_azimuthOffset, 360.0);
    if (azimuth < 0.0) {
        azimuth += 360.0;
    }
    elevation = qBound(
        static_cast<double>(CameraSettings::m_minElevation),
        elevation + m_settings.m_elevationOffset,
        static_cast<double>(CameraSettings::m_maxElevation));

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
    };
}

bool CameraVideoSurface::present(const QVideoFrame& frame)
{
    if (!frame.isValid()) {
        return false;
    }

    QVideoFrame mutableFrame(frame);
    if (!mutableFrame.map(QAbstractVideoBuffer::ReadOnly)) {
        return false;
    }

    const QImage::Format imageFormat = QVideoFrame::imageFormatFromPixelFormat(mutableFrame.pixelFormat());
    QImage image;

    if (imageFormat != QImage::Format_Invalid)
    {
        // Copy the mapped frame into a pooled buffer instead of .copy() allocating
        // a fresh one each frame.
        const QImage view(
            mutableFrame.bits(),
            mutableFrame.width(),
            mutableFrame.height(),
            mutableFrame.bytesPerLine(),
            imageFormat);
        image = m_imagePool.acquire(view.width(), view.height(), imageFormat);
        if (!image.isNull())
        {
            const int bytesPerLine = qMin(static_cast<int>(image.bytesPerLine()), static_cast<int>(view.bytesPerLine()));
            for (int y = 0; y < view.height(); ++y) {
                std::memcpy(image.scanLine(y), view.scanLine(y), static_cast<size_t>(bytesPerLine));
            }
        }
        else
        {
            image = view.copy();
        }
    }

    mutableFrame.unmap();

    if (!image.isNull()) {
        emit frameAvailable(image);
    }

    return true;
}
#endif // Qt 5

bool CameraGUI::ensureVideoFilePlayer(bool startPlayback)
{
    if (!m_settings.isFfmpegMediaSource() || m_settings.ffmpegMediaSourcePath().isEmpty()) {
        return false;
    }

    if ((m_camera->getState() == Feature::StRunning) && startPlayback) {
        sendVideoFileControl(CameraWorker::MsgVideoFileControl::Play);
    }
    updateVideoFileControls();
    return true;
}

void CameraGUI::sendVideoFileControl(CameraWorker::MsgVideoFileControl::Action action, qint64 positionMs)
{
    MessageQueue *queue = m_camera ? m_camera->getWorkerInputMessageQueue() : nullptr;
    if (!queue) {
        return;
    }

    queue->push(CameraWorker::MsgVideoFileControl::create(action, positionMs));
}

void CameraGUI::setupQtCapture()
{
    cleanupQtCapture();
    resetQtHdrBracketState();
    m_reportedFeatureErrorKeys.clear();
    m_qtStillCaptureTimer.stop();

    if (m_settings.isImageFileSequenceCamera())
    {
        if (m_settings.m_imageFileCameraPaths.isEmpty()) {
            return;
        }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        m_pendingQtVideoFrame = QVideoFrame();
        m_processingQtVideoFrame = false;
#endif
        m_imageSequenceLoaded = true;
        m_playbackDurationMs = imageSequenceDurationMs();
        m_imageSequenceIndex = 0;
        updateImageSequencePositionSlider();
        updateVideoFileControls();

        m_qtZoomSupported = false;
        m_qtManualExposureSupported = false;
        m_qtIsoSensitivitySupported = false;
        m_qtWhiteBalanceModeSupported = false;
        m_qtExposureCompensationSupported = false;
        updateCameraSettingsVisibility();

        showImageSequenceFrame(0);
        m_imageSequenceTimer.start(imageSequenceIntervalMs());
        {
            QSignalBlocker blocker(ui->playPauseVideo);
            ui->playPauseVideo->setChecked(true);
        }
        return;
    }

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

    reportResolutions();

    m_captureSession = new QMediaCaptureSession(this);
    m_qtCamera       = new QCamera(selectedDevice, this);
    m_imageCapture   = nullptr;
    m_videoSink      = nullptr;

    // Select the requested format if possible, otherwise use the closest
    // supported frame rate at the requested resolution.
    QCameraFormat chosenFormat;
    bool exactFormat = false;
    bool sameResolutionFallback = false;
    for (const QCameraFormat& fmt : selectedDevice.videoFormats())
    {
        if ((fmt.resolution().width()  == m_settings.m_resolutionWidth)
         && (fmt.resolution().height() == m_settings.m_resolutionHeight)
            && (fmt.maxFrameRate()     >= m_settings.m_framesPerSecond)
            && (fmt.minFrameRate()     <= m_settings.m_framesPerSecond)
            )
        {
            chosenFormat = fmt;
            exactFormat = true;
            break;
        }
    }

    if (chosenFormat.isNull())
    {
        for (const QCameraFormat& fmt : selectedDevice.videoFormats())
        {
            if ((fmt.resolution().width() != m_settings.m_resolutionWidth)
                || (fmt.resolution().height() != m_settings.m_resolutionHeight))
            {
                continue;
            }

            if (chosenFormat.isNull()
                || (cameraFormatFpsDistance(fmt, m_settings.m_framesPerSecond)
                    < cameraFormatFpsDistance(chosenFormat, m_settings.m_framesPerSecond)))
            {
                chosenFormat = fmt;
                sameResolutionFallback = true;
            }
        }
    }

    if (!chosenFormat.isNull())
    {
        m_qtCamera->setCameraFormat(chosenFormat);
        if (sameResolutionFallback && !exactFormat)
        {
            const int requestedFps = m_settings.m_framesPerSecond;
            const int supportedFps = nearestCameraFormatFps(chosenFormat, requestedFps);
            m_settings.m_framesPerSecond = supportedFps;
            {
                QSignalBlocker spinBlocker(settingsUI()->fpsSpin);
                QSignalBlocker comboBlocker(settingsUI()->fpsCombo);
                if (settingsUI()->fpsStack->currentWidget() == settingsUI()->fpsSpinPage) {
                    settingsUI()->fpsSpin->setValue(supportedFps);
                } else {
                    const int index = settingsUI()->fpsCombo->findData(supportedFps);
                    if (index >= 0) {
                        settingsUI()->fpsCombo->setCurrentIndex(index);
                    }
                }
            }
            qDebug() << "CameraGUI::setupQtCapture: using closest Qt camera format"
                << chosenFormat.resolution()
                << "requestedFps" << requestedFps
                << "selectedFps" << supportedFps
                << "formatFpsRange" << chosenFormat.minFrameRate() << chosenFormat.maxFrameRate();
        }
    }
    else if (!selectedDevice.videoFormats().isEmpty())
    {
        qDebug() << "CameraGUI::setupQtCapture: no explicit Qt camera format match; using device default"
            << "requested" << m_settings.m_resolutionWidth
            << m_settings.m_resolutionHeight
            << m_settings.m_framesPerSecond
            << "availableFormats" << selectedDevice.videoFormats().size();
    }

    m_qtCamera->setExposureMode(QCamera::ExposureManual);
    applyQtExposureTimeMs(currentQtCaptureExposureTimeMs());
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
                    reportFeatureError(
                        QStringLiteral("qtImageCaptureError"),
                        tr("Qt image capture failed"),
                        errorString.isEmpty() ? tr("The Qt camera reported an image capture error.") : errorString);
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

    if (m_settings.isFfmpegMediaSource())
    {
        ensureVideoFilePlayer(true);
        return;
    }

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
                    reportFeatureError(
                        QStringLiteral("qtImageCaptureError"),
                        tr("Qt image capture failed"),
                        errorString.isEmpty() ? tr("The Qt camera reported an image capture error.") : errorString);
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
        applyQtExposureTimeMs(currentQtCaptureExposureTimeMs());
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
    m_reportedFeatureErrorKeys.clear();
    m_qtStillCaptureTimer.stop();
    m_imageSequenceTimer.stop();
    m_imageSequenceLoaded = false;
    m_imageSequenceIndex = 0;
    resetQtHdrBracketState();
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
#endif
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
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
    m_playbackDurationMs = 0;
    {
        QSignalBlocker blocker(ui->playbackPositionSlider);
        ui->playbackPositionSlider->setValue(0);
    }
    {
        QSignalBlocker blocker(ui->playPauseVideo);
        ui->playPauseVideo->setChecked(false);
    }
    updateVideoFileControls();
}

void CameraGUI::reportFeatureError(const QString& errorKey, const QString& title, const QString& errorMessage)
{
    if (!m_camera || m_reportedFeatureErrorKeys.contains(errorKey)) {
        return;
    }

    m_reportedFeatureErrorKeys.insert(errorKey);
    m_camera->getInputMessageQueue()->push(Camera::MsgReportError::create(title, errorMessage));
}

void CameraGUI::applyQtCameraSettings(const QList<QString>& settingsKeys, bool force)
{
    const bool hdrSettingsChanged = force
        || settingsKeys.contains("stackEnabled")
        || settingsKeys.contains("stackMethod")
        || settingsKeys.contains("stackHdrAlgorithm")
        || settingsKeys.contains("stackHdrExposureCount")
        || settingsKeys.contains("stackHdrExposure1Ms")
        || settingsKeys.contains("stackHdrExposure2Ms")
        || settingsKeys.contains("stackHdrExposure3Ms")
        || settingsKeys.contains("stackHdrExposure4Ms");

    if (hdrSettingsChanged) {
        resetQtHdrBracketState();
    }

    if (!m_settings.isQtCamera() && !m_settings.isFileCamera())
    {
        // Camera type switched away from Qt — stop any running Qt camera
        if (m_qtCamera
            || m_imageSequenceLoaded
            || m_imageSequenceTimer.isActive())
        {
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
        || settingsKeys.contains("streamUrl")
        || settingsKeys.contains("imageFileCameraPaths")
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
        (m_qtCamera != nullptr) || m_imageSequenceLoaded;
#else
        (m_qtCamera != nullptr) || m_imageSequenceLoaded;
#endif

    if (!hasActiveVisualSource && (m_camera->getState() == Feature::StRunning)
        && (m_settings.isQtCamera() || m_settings.isImageFileSequenceCamera()))
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
        if (force || hdrSettingsChanged) {
            applyQtExposureTimeMs(currentQtCaptureExposureTimeMs());
        }
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

    // Pool the backing buffer for frames whose pixel format maps directly to a
    // single-plane QImage format (RGB/BGR families): copy the mapped frame into a
    // pooled buffer instead of frame.toImage() allocating one per frame. Formats
    // needing conversion (YUV/planar) fall back to toImage(), which we can't pool
    // without reimplementing Qt's colour conversion.
    QImage image;
    {
        QVideoFrame mapped(frame);
        if (mapped.map(QVideoFrame::ReadOnly))
        {
            const QImage::Format fmt = QVideoFrameFormat::imageFormatFromPixelFormat(mapped.pixelFormat());
            if ((fmt != QImage::Format_Invalid) && (mapped.planeCount() == 1))
            {
                const QImage view(mapped.bits(0), mapped.width(), mapped.height(),
                    mapped.bytesPerLine(0), fmt);
                image = m_qtImagePool.acquire(view.width(), view.height(), fmt);
                if (!image.isNull())
                {
                    const int bytesPerLine = qMin(static_cast<int>(image.bytesPerLine()), static_cast<int>(view.bytesPerLine()));
                    for (int y = 0; y < view.height(); ++y) {
                        std::memcpy(image.scanLine(y), view.scanLine(y), static_cast<size_t>(bytesPerLine));
                    }
                }
            }
            mapped.unmap();
        }
    }
    if (image.isNull()) {
        image = frame.toImage();
    }
    submitQtImageFrame(image);

    if (m_pendingQtVideoFrame.isValid()) {
        QMetaObject::invokeMethod(this, &CameraGUI::processPendingQtVideoFrame, Qt::QueuedConnection);
    } else {
        m_processingQtVideoFrame = false;
    }
}
#else
void CameraGUI::onQt5VideoFrame(const QImage& image)
{
    submitQtImageFrame(image);
}
#endif

void CameraGUI::onQtImageCaptured(int id, const QImage& image)
{
    Q_UNUSED(id)

    submitQtImageFrame(image);
}

void CameraGUI::submitQtImageFrame(const QImage& image, qint64 playbackPositionMs, int playbackFrameNumber)
{
    if (image.isNull()) {
        return;
    }

    const double exposureTimeMs = currentQtCaptureExposureTimeMs();
    const int hdrExposureIndex = currentQtHdrExposureIndex();
    const int hdrExposureCount = currentQtHdrExposureCount();

    CameraFrameAligner *frameAligner = m_camera->getFrameAligner();
    if (frameAligner) {
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = image;
        frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        const QDateTime captureDateTime = m_settings.isImageFileSequenceCamera()
            ? captureDateTimeFromFileName(m_settings.m_imageFileCameraPaths.value(m_imageSequenceIndex))
            : QDateTime();
        frame->m_playbackPositionMs = playbackPositionMs;
        frame->m_playbackFrameNumber = playbackFrameNumber;
        frame->m_playbackActiveFrame = m_captureActive && ((playbackPositionMs >= 0) || (playbackFrameNumber > 0));
        populateFrameExposureMetadata(*frame, exposureTimeMs, hdrExposureIndex, hdrExposureCount, captureDateTime);
        frame->m_captureEpoch = m_captureEpoch;
        frame->m_manualPreviewFrame = !m_captureActive;
        frameAligner->submitFrame(frame);
    }

    if (isHdrStackingActiveForQt()) {
        advanceQtHdrBracketState();
    }
}

void CameraGUI::triggerQtStillCapture()
{
    if (!m_settings.isQtCamera() || !m_settings.isIntervalCaptureMode() || !m_qtCamera || !m_imageCapture) {
        return;
    }

    if (isHdrStackingActiveForQt()) {
        applyQtExposureTimeMs(currentQtCaptureExposureTimeMs());
    }

    if (m_imageCapture->isReadyForCapture()) {
        m_imageCapture->capture();
    }
}

void CameraGUI::updateCameraSettingsVisibility()
{
    const bool alpaca = m_settings.isAlpacaCamera();
    const bool asi = m_settings.isAsiCamera();
    const bool fileCamera = m_settings.isFileCamera();
    const bool qtCamera = m_settings.isQtCamera();
    const bool sharedHardwareCamera = alpaca || asi;
    updateHdrStackingControls();
    const bool hdrExposureOverrideActive = m_settings.isHdrStackingEnabled() && isHdrStackingSupported();

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
    settingsUI()->asiAutoExposureGainLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureGainCombo->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureGainModeLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureGainModeCombo->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureTargetLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureTargetSpin->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposurePercentileLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposurePercentileSpin->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMinMsLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMinMsSpin->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMaxMsLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMaxMsSpin->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMinGainLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMinGainSpin->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMaxGainLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMaxGainSpin->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMaxChangeLabel->setVisible(sharedHardwareCamera);
    settingsUI()->autoExposureMaxChangeSpin->setVisible(sharedHardwareCamera);
    settingsUI()->asiColorImageTypeLabel->setVisible(asi && m_asiColorCameraActive && (m_asiRgb24Supported || m_asiRaw16Supported || m_asiRaw8Supported));
    settingsUI()->asiColorImageTypeCombo->setVisible(asi && m_asiColorCameraActive && (m_asiRgb24Supported || m_asiRaw16Supported || m_asiRaw8Supported));
    settingsUI()->alpacaFocusPositionLabel->setVisible(alpaca);
    settingsUI()->alpacaFocusPositionSpin->setVisible(alpaca);
    settingsUI()->alpacaFocusStepSizeLabel->setVisible(alpaca);
    settingsUI()->alpacaFocusStepSizeSpin->setVisible(alpaca);
    settingsUI()->alpacaAutoFocusLabel->setVisible(alpaca);
    settingsUI()->alpacaAutoFocusButton->setVisible(alpaca);
    settingsUI()->alpacaAutoFocusStatusLabel->setVisible(alpaca);
    settingsUI()->alpacaFilterWheelPositionLabel->setVisible(alpaca);
    settingsUI()->alpacaFilterWheelPositionCombo->setVisible(alpaca);

    bool focuserAvailable = alpaca && (settingsUI()->alpacaFocuserCombo->count() > 0);
    settingsUI()->alpacaFocuserEnabledCheck->setEnabled(focuserAvailable);
    settingsUI()->alpacaFocuserCombo->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    const bool asiAutoExposureGainEnabled = asi && (m_settings.m_captureMode == CameraSettings::CaptureModeFrameRate);
    const bool softwareAutoExposureGainEnabled = sharedHardwareCamera && m_settings.m_autoExposureGainEnabled;
    const bool autoExposureGainSettingsEnabled = sharedHardwareCamera && m_settings.m_autoExposureGainEnabled;
    const bool asiManualExposureGainEnabled = !softwareAutoExposureGainEnabled && !(asi && m_settings.m_asiAutoExposureGain && asiAutoExposureGainEnabled);
    settingsUI()->asiAutoExposureGainLabel->setEnabled(sharedHardwareCamera);
    settingsUI()->autoExposureGainCombo->setEnabled(sharedHardwareCamera);
    QStandardItemModel *autoExposureGainModel = qobject_cast<QStandardItemModel*>(settingsUI()->autoExposureGainCombo->model());
    const int hardwareAutoExposureGainIndex = settingsUI()->autoExposureGainCombo->findData(AutoExposureGainHardware);
    if (autoExposureGainModel && (hardwareAutoExposureGainIndex >= 0))
    {
        QStandardItem *hardwareItem = autoExposureGainModel->item(hardwareAutoExposureGainIndex);
        if (hardwareItem) {
            hardwareItem->setEnabled(asiAutoExposureGainEnabled);
        }
    }
    settingsUI()->autoExposureGainModeLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureGainModeCombo->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureTargetLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureTargetSpin->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposurePercentileLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposurePercentileSpin->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMinMsLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMinMsSpin->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMaxMsLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMaxMsSpin->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMinGainLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMinGainSpin->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMaxGainLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMaxGainSpin->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMaxChangeLabel->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->autoExposureMaxChangeSpin->setEnabled(autoExposureGainSettingsEnabled);
    settingsUI()->alpacaFocusPositionLabel->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaFocusPositionSpin->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaFocusStepSizeLabel->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaFocusStepSizeSpin->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaAutoFocusLabel->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaAutoFocusButton->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);
    settingsUI()->alpacaAutoFocusStatusLabel->setEnabled(m_settings.m_alpacaFocuserEnabled && focuserAvailable);

    bool filterWheelAvailable = alpaca && (settingsUI()->alpacaFilterWheelCombo->count() > 0);
    settingsUI()->alpacaFilterWheelEnabledCheck->setEnabled(filterWheelAvailable);
    settingsUI()->alpacaFilterWheelCombo->setEnabled(m_settings.m_alpacaFilterWheelEnabled && filterWheelAvailable);
    settingsUI()->alpacaFilterWheelPositionLabel->setEnabled(m_settings.m_alpacaFilterWheelEnabled && filterWheelAvailable);
    settingsUI()->alpacaFilterWheelPositionCombo->setEnabled(m_settings.m_alpacaFilterWheelEnabled && filterWheelAvailable);

    settingsUI()->tabWidget->setTabEnabled(0, !fileCamera);
    settingsUI()->tabWidget->setTabEnabled(1, sharedHardwareCamera);
    ui->audioMute->setVisible(qtCamera || m_settings.isFfmpegMediaSource());
    ui->audioPreviewVolumeDial->setVisible(qtCamera || m_settings.isFfmpegMediaSource());

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
        settingsUI()->exposureLabel->setEnabled(asiManualExposureGainEnabled && !hdrExposureOverrideActive);
        settingsUI()->exposureSlider->setEnabled(asiManualExposureGainEnabled && !hdrExposureOverrideActive);
        settingsUI()->exposureSpin->setEnabled(asiManualExposureGainEnabled && !hdrExposureOverrideActive);
        settingsUI()->exposureUnitsCombo->setEnabled(asiManualExposureGainEnabled && !hdrExposureOverrideActive);
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
        settingsUI()->exposureLabel->setEnabled(m_qtManualExposureSupported && !hdrExposureOverrideActive);
        settingsUI()->exposureSlider->setEnabled(m_qtManualExposureSupported && !hdrExposureOverrideActive);
        settingsUI()->exposureSpin->setEnabled(m_qtManualExposureSupported && !hdrExposureOverrideActive);
        settingsUI()->exposureUnitsCombo->setEnabled(m_qtManualExposureSupported && !hdrExposureOverrideActive);
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

int CameraGUI::imageSequenceIntervalMs() const
{
    return qMax(1, static_cast<int>(1000.0 / qMax(0.1, m_settings.m_videoPlaybackRate) + 0.5));
}

qint64 CameraGUI::imageSequenceDurationMs() const
{
    const int frameCount = m_settings.m_imageFileCameraPaths.size();
    if (frameCount <= 0) {
        return 0;
    }

    return static_cast<qint64>(frameCount) * imageSequenceIntervalMs();
}

void CameraGUI::updatePlaybackPositionLabel(qint64 videoPositionMs)
{
    if (m_settings.isImageFileSequenceCamera())
    {
        const int frameCount = m_settings.m_imageFileCameraPaths.size();
        const int frameNumber = frameCount > 0 ? qBound(1, m_imageSequenceIndex + 1, frameCount) : 0;
        ui->playbackPositionLabel->setText(QStringLiteral("%1/%2").arg(frameNumber).arg(frameCount));
        return;
    }

    qint64 positionMs = videoPositionMs;
    positionMs = qMax<qint64>(0, positionMs);
    const qint64 totalSeconds = positionMs / 1000;
    const qint64 hours = totalSeconds / 3600;
    const qint64 minutes = (totalSeconds / 60) % 60;
    const qint64 seconds = totalSeconds % 60;
    ui->playbackPositionLabel->setText(QStringLiteral("%1:%2:%3")
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0')));
}

bool CameraGUI::loadImageSequenceFrame(int index, QImage& image) const
{
    if ((index < 0) || (index >= m_settings.m_imageFileCameraPaths.size())) {
        return false;
    }

    image = CameraFileSequenceDialog::loadPreviewImage(m_settings.m_imageFileCameraPaths.at(index));
    return !image.isNull();
}

void CameraGUI::showImageSequenceFrame(int index)
{
    QImage image;
    if (!loadImageSequenceFrame(index, image))
    {
        reportFeatureError(
            QStringLiteral("imageSequenceReadError"),
            tr("Image sequence frame could not be loaded"),
            tr("Could not load image sequence frame: %1")
                .arg((index >= 0) && (index < m_settings.m_imageFileCameraPaths.size())
                    ? m_settings.m_imageFileCameraPaths.at(index)
                    : QString::number(index)));
        return;
    }

    m_imageSequenceIndex = index;
    updateImageSequencePositionSlider();
    submitQtImageFrame(image, -1, index + 1);
}

void CameraGUI::advanceImageSequenceFrame()
{
    if (m_settings.m_imageFileCameraPaths.isEmpty()) {
        return;
    }

    int nextIndex = m_imageSequenceIndex + 1;
    if (nextIndex >= m_settings.m_imageFileCameraPaths.size())
    {
        if (!m_settings.m_videoLoop)
        {
            m_imageSequenceTimer.stop();
            QSignalBlocker blocker(ui->playPauseVideo);
            ui->playPauseVideo->setChecked(false);
            return;
        }

        nextIndex = 0;
    }

    showImageSequenceFrame(nextIndex);
}

void CameraGUI::updateImageSequencePositionSlider()
{
    if (m_settings.m_imageFileCameraPaths.isEmpty())
    {
        QSignalBlocker blocker(ui->playbackPositionSlider);
        ui->playbackPositionSlider->setValue(0);
        return;
    }

    const int denominator = qMax(1, m_settings.m_imageFileCameraPaths.size() - 1);
    const int sliderValue = (m_imageSequenceIndex * PlaybackPositionSliderMaximum) / denominator;
    QSignalBlocker blocker(ui->playbackPositionSlider);
    ui->playbackPositionSlider->setValue(qBound(0, sliderValue, PlaybackPositionSliderMaximum));
    updatePlaybackPositionLabel();
}

bool CameraGUI::prepareImageSequenceManualStep(bool *wasLoaded)
{
    if (wasLoaded) {
        *wasLoaded = m_imageSequenceLoaded;
    }

    m_imageSequenceTimer.stop();
    {
        QSignalBlocker blocker(ui->playPauseVideo);
        ui->playPauseVideo->setChecked(false);
    }

    if (!m_settings.m_imageFileCameraPaths.isEmpty()) {
        m_playbackDurationMs = imageSequenceDurationMs();
        updateVideoFileControls();
        return true;
    }

    return false;
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
    m_asiRaw8Supported = info.isRaw8Supported();
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
        if (info.isRaw8Supported()) {
            settingsUI()->asiColorImageTypeCombo->addItem(QStringLiteral("RAW8"), CameraSettings::AsiColorImageTypeRaw8);
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
    QSignalBlocker numXBlocker(settingsUI()->cameraNumXSpin);
    QSignalBlocker numYBlocker(settingsUI()->cameraNumYSpin);
    QSignalBlocker startXBlocker(settingsUI()->cameraStartXSpin);
    QSignalBlocker startYBlocker(settingsUI()->cameraStartYSpin);

    if ((m_alpacaCameraSizeX <= 0) || (m_alpacaCameraSizeY <= 0))
    {
        settingsUI()->cameraNumXSpin->setMinimum(0);
        settingsUI()->cameraNumYSpin->setMinimum(0);
        settingsUI()->cameraNumXSpin->setMaximum(65535);
        settingsUI()->cameraNumYSpin->setMaximum(65535);
        settingsUI()->cameraStartXSpin->setMaximum(65535);
        settingsUI()->cameraStartYSpin->setMaximum(65535);
        settingsUI()->cameraNumXSpin->setValue(m_settings.m_cameraNumX);
        settingsUI()->cameraNumYSpin->setValue(m_settings.m_cameraNumY);
        settingsUI()->cameraStartXSpin->setValue(m_settings.m_cameraStartX);
        settingsUI()->cameraStartYSpin->setValue(m_settings.m_cameraStartY);
        return;
    }

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
    m_camera->getInputMessageQueue()->push(Camera::MsgStartStop::create(checked));
}

void CameraGUI::on_refreshCamerasButton_clicked()
{
    m_camera->getInputMessageQueue()->push(Camera::MsgRefreshCameraList::create(true));
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

    bool updated = false;
    if (m_settings.isImageFileSequenceCamera())
    {
        updated = chooseImageFileSequenceFiles(
            index,
            m_settings.m_cameraProtocol,
            m_settings.m_cameraId,
            m_settings.m_alpacaHost,
            m_settings.m_alpacaPort);
    }
    else if (m_settings.isStreamCamera())
    {
        updated = chooseStreamUrl(
            index,
            m_settings.m_cameraProtocol,
            m_settings.m_cameraId,
            m_settings.m_alpacaHost,
            m_settings.m_alpacaPort);
    }
    else
    {
        updated = chooseVideoFileCameraFile(
            index,
            m_settings.m_cameraProtocol,
            m_settings.m_cameraId,
            m_settings.m_alpacaHost,
            m_settings.m_alpacaPort);
    }
    if (!updated)
    {
        return;
    }

    m_settings.m_cameraId = ui->cameraCombo->itemData(index, CameraIdRole).toString();
    m_settings.m_cameraDescription = ui->cameraCombo->itemData(index, CameraDescriptionRole).toString();
    if (m_settings.isVideoFileCamera()) {
        m_settings.m_videoFileCameraPath = m_settings.m_cameraId;
    }
    else if (m_settings.isStreamCamera()) {
        m_settings.m_streamUrl = m_settings.m_cameraId;
    }
    updateVideoFileControls();
    applySettings(m_settings.isImageFileSequenceCamera()
        ? QStringList({"cameraId", "cameraDescription", "imageFileCameraPaths"})
        : (m_settings.isStreamCamera()
            ? QStringList({"cameraId", "cameraDescription", "streamUrl", "streamUrlHistory"})
            : QStringList({"cameraId", "cameraDescription", "videoFileCameraPath"})));
}

void CameraGUI::on_restartVideo_clicked()
{
    if (!m_settings.hasFileCameraSource()) {
        return;
    }

    if (m_settings.isImageFileSequenceCamera())
    {
        if (!m_imageSequenceLoaded && (m_camera->getState() == Feature::StRunning)) {
            setupQtCapture();
        } else {
            showImageSequenceFrame(0);
            m_imageSequenceTimer.start(imageSequenceIntervalMs());
            QSignalBlocker blocker(ui->playPauseVideo);
            ui->playPauseVideo->setChecked(true);
        }
        return;
    }

    if (m_settings.isFfmpegMediaSource())
    {
        sendVideoFileControl(CameraWorker::MsgVideoFileControl::Restart);
        return;
    }
}

void CameraGUI::on_stepBackVideo_clicked()
{
    if (!m_settings.hasFileCameraSource()) {
        return;
    }

    if (m_settings.isImageFileSequenceCamera())
    {
        bool wasLoaded = false;
        if (prepareImageSequenceManualStep(&wasLoaded)) {
            showImageSequenceFrame(wasLoaded ? qMax(0, m_imageSequenceIndex - 1) : 0);
        }
        return;
    }

    if (m_settings.isVideoFileCamera())
    {
        sendVideoFileControl(CameraWorker::MsgVideoFileControl::StepBack);
        return;
    }
}

void CameraGUI::on_stepForwardVideo_clicked()
{
    if (!m_settings.hasFileCameraSource()) {
        return;
    }

    if (m_settings.isImageFileSequenceCamera())
    {
        bool wasLoaded = false;
        if (prepareImageSequenceManualStep(&wasLoaded))
        {
            const int maxIndex = m_settings.m_imageFileCameraPaths.size() - 1;
            showImageSequenceFrame(wasLoaded ? qMin(maxIndex, m_imageSequenceIndex + 1) : 0);
        }
        return;
    }

    if (m_settings.isVideoFileCamera())
    {
        sendVideoFileControl(CameraWorker::MsgVideoFileControl::StepForward);
        return;
    }
}

void CameraGUI::on_playPauseVideo_clicked(bool checked)
{
    if (!m_settings.hasFileCameraSource()) {
        return;
    }

    if (m_settings.isImageFileSequenceCamera())
    {
        if (checked)
        {
            if (!m_imageSequenceLoaded && (m_camera->getState() == Feature::StRunning)) {
                setupQtCapture();
            } else if (m_imageSequenceLoaded) {
                m_imageSequenceTimer.start(imageSequenceIntervalMs());
            }
        }
        else
        {
            m_imageSequenceTimer.stop();
        }
        return;
    }

    if (m_settings.isFfmpegMediaSource())
    {
        sendVideoFileControl(checked ? CameraWorker::MsgVideoFileControl::Play : CameraWorker::MsgVideoFileControl::Pause);
    }
}

void CameraGUI::on_loopVideo_clicked(bool checked)
{
    m_settings.m_videoLoop = checked;
    applySetting("videoLoop");
}

void CameraGUI::on_playbackRateSpin_valueChanged(double value)
{
    m_settings.m_videoPlaybackRate = value;
    applySetting("videoPlaybackRate");
    if (m_settings.isImageFileSequenceCamera() && m_imageSequenceTimer.isActive()) {
        m_playbackDurationMs = imageSequenceDurationMs();
        m_imageSequenceTimer.start(imageSequenceIntervalMs());
        updatePlaybackPositionLabel();
    }
}

void CameraGUI::on_playbackAudioOffsetSpin_valueChanged(int value)
{
    m_settings.m_videoPlaybackAudioOffsetMs = value;
    applySetting("videoPlaybackAudioOffsetMs");
}

void CameraGUI::on_playbackPositionSlider_sliderMoved(int value)
{
    if (m_settings.isImageFileSequenceCamera())
    {
        const int count = m_settings.m_imageFileCameraPaths.size();
        if (count <= 0) {
            return;
        }

        const int index = qBound(0, (value * (count - 1)) / PlaybackPositionSliderMaximum, count - 1);
        showImageSequenceFrame(index);
        return;
    }

    if (m_playbackDurationMs <= 0) {
        return;
    }

    const qint64 position = (static_cast<qint64>(value) * m_playbackDurationMs) / PlaybackPositionSliderMaximum;
    updatePlaybackPositionLabel(position);
}

void CameraGUI::on_playbackPositionSlider_sliderReleased()
{
    if (m_settings.isImageFileSequenceCamera())
    {
        on_playbackPositionSlider_sliderMoved(ui->playbackPositionSlider->value());
        return;
    }

    if (m_playbackDurationMs <= 0) {
        return;
    }

    const qint64 position = (static_cast<qint64>(ui->playbackPositionSlider->value()) * m_playbackDurationMs) / PlaybackPositionSliderMaximum;
    updatePlaybackPositionLabel(position);
    sendVideoFileControl(CameraWorker::MsgVideoFileControl::Seek, position);
}

void CameraGUI::on_cameraCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    const CameraInfo previousCamera = selectedCameraFromSettings();
    const int previousResolutionWidth = m_settings.m_resolutionWidth;
    const int previousResolutionHeight = m_settings.m_resolutionHeight;
    const int previousFramesPerSecond = m_settings.m_framesPerSecond;
    CameraInfo selectedCamera = comboCameraInfo(index);
    const bool wasAlpaca = previousCamera.m_protocol == CameraProtocol::alpaca();
    const bool wasAsi = previousCamera.m_protocol == CameraProtocol::asi();

    if ((selectedCamera.m_protocol == CameraProtocol::video()) && selectedCamera.m_id.isEmpty())
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
    else if ((selectedCamera.m_protocol == CameraProtocol::stream()) && selectedCamera.m_id.isEmpty())
    {
        if (!chooseStreamUrl(index,
                previousCamera.m_protocol,
                previousCamera.m_id,
                previousCamera.m_host,
                previousCamera.m_port))
        {
            return;
        }

        selectedCamera = comboCameraInfo(index);
    }
    else if (selectedCamera.m_protocol == CameraProtocol::images())
    {
        if ((previousCamera.m_protocol != CameraProtocol::images()) && m_settings.m_imageFileCameraPaths.isEmpty()
            && !chooseImageFileSequenceFiles(index,
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
    if (m_settings.m_resolutionWidth != previousResolutionWidth) {
        settingsKeys.append("resolutionWidth");
    }
    if (m_settings.m_resolutionHeight != previousResolutionHeight) {
        settingsKeys.append("resolutionHeight");
    }
    if (m_settings.m_framesPerSecond != previousFramesPerSecond) {
        settingsKeys.append("framesPerSecond");
    }
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
            updateVideoPreRecordBufferMemoryLabel();
            updateScaleControls();
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
    updateCaptureIntervalWarning();
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("captureMode");
}

void CameraGUI::on_fpsSpin_valueChanged(int value)
{
    m_settings.m_framesPerSecond = value;
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("framesPerSecond");
}

void CameraGUI::on_fpsCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_framesPerSecond = settingsUI()->fpsCombo->itemData(index).toInt();
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("framesPerSecond");
}

void CameraGUI::on_intervalSpin_valueChanged(double value)
{
    m_settings.m_captureInterval = value;
    updateCaptureIntervalWarning();
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("captureInterval");
}

void CameraGUI::on_intervalUnitsCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_captureIntervalUnits = static_cast<CameraSettings::CaptureIntervalUnits>(settingsUI()->intervalUnitsCombo->itemData(index).toInt());
    updateCaptureIntervalWarning();
    updateVideoPreRecordBufferMemoryLabel();
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
    updateCaptureIntervalWarning();
    applySetting("exposureTimeMs");
}

void CameraGUI::on_exposureSpin_valueChanged(double value)
{
    settingsUI()->exposureSlider->blockSignals(true);
    settingsUI()->exposureSlider->setValue(exposureValueToSlider(settingsUI()->exposureSpin, value));
    settingsUI()->exposureSlider->blockSignals(false);
    m_settings.m_exposureTimeMs = value * currentExposureUnitScaleMs(settingsUI());
    updateCaptureIntervalWarning();
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

void CameraGUI::on_alpacaAutoFocusButton_clicked()
{
    MessageQueue *workerQueue = m_camera->getWorkerInputMessageQueue();
    if (!workerQueue) {
        return;
    }

    settingsUI()->alpacaAutoFocusStatusLabel->setText(tr("Starting"));
    settingsUI()->alpacaAutoFocusButton->setEnabled(false);
    workerQueue->push(CameraWorker::MsgStartAutoFocus::create());
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
    updateScaleControls();
    applySettings({"cameraBinX", "cameraNumX", "cameraStartX"});
}

void CameraGUI::on_cameraBinYSpin_valueChanged(int value)
{
    m_settings.m_cameraBinY = value;
    updateCameraSubframeControls();
    updateScaleControls();
    applySettings({"cameraBinY", "cameraNumY", "cameraStartY"});
}

void CameraGUI::on_cameraNumXSpin_valueChanged(int value)
{
    m_settings.m_cameraNumX = value;
    updateCameraSubframeControls();
    updateScaleControls();
    applySettings({"cameraNumX", "cameraStartX"});
}

void CameraGUI::on_cameraNumYSpin_valueChanged(int value)
{
    m_settings.m_cameraNumY = value;
    updateCameraSubframeControls();
    updateScaleControls();
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

void CameraGUI::on_autoExposureGainCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    const int mode = settingsUI()->autoExposureGainCombo->itemData(index).toInt();
    const bool softwareAutoExposureGain = mode == AutoExposureGainSoftware;
    const bool hardwareAutoExposureGain = mode == AutoExposureGainHardware;
    if ((m_settings.m_autoExposureGainEnabled == softwareAutoExposureGain)
        && (m_settings.m_asiAutoExposureGain == hardwareAutoExposureGain))
    {
        return;
    }

    m_settings.m_autoExposureGainEnabled = softwareAutoExposureGain;
    m_settings.m_asiAutoExposureGain = hardwareAutoExposureGain;
    updateCameraSettingsVisibility();
    applySetting("autoExposureGainEnabled");
    applySetting("asiAutoExposureGain");
}

void CameraGUI::on_autoExposureGainModeCombo_currentIndexChanged(int index)
{
    if (index < 0) {
        return;
    }

    m_settings.m_autoExposureGainMode = static_cast<CameraSettings::AutoExposureGainMode>(
        settingsUI()->autoExposureGainModeCombo->itemData(index).toInt());
    applySetting("autoExposureGainMode");
}

void CameraGUI::on_autoExposureTargetSpin_valueChanged(double value)
{
    m_settings.m_autoExposureTargetBrightness = value;
    applySetting("autoExposureTargetBrightness");
}

void CameraGUI::on_autoExposurePercentileSpin_valueChanged(double value)
{
    m_settings.m_autoExposureTargetPercentile = value;
    applySetting("autoExposureTargetPercentile");
}

void CameraGUI::on_autoExposureMinMsSpin_valueChanged(double value)
{
    m_settings.m_autoExposureMinMs = value;
    if (m_settings.m_autoExposureMaxMs < value)
    {
        QSignalBlocker blocker(settingsUI()->autoExposureMaxMsSpin);
        m_settings.m_autoExposureMaxMs = value;
        settingsUI()->autoExposureMaxMsSpin->setValue(value);
        applySetting("autoExposureMaxMs");
    }
    applySetting("autoExposureMinMs");
}

void CameraGUI::on_autoExposureMaxMsSpin_valueChanged(double value)
{
    m_settings.m_autoExposureMaxMs = std::max(m_settings.m_autoExposureMinMs, value);
    if (m_settings.m_autoExposureMaxMs != value)
    {
        QSignalBlocker blocker(settingsUI()->autoExposureMaxMsSpin);
        settingsUI()->autoExposureMaxMsSpin->setValue(m_settings.m_autoExposureMaxMs);
    }
    applySetting("autoExposureMaxMs");
}

void CameraGUI::on_autoExposureMinGainSpin_valueChanged(int value)
{
    m_settings.m_autoExposureMinGain = value;
    if (m_settings.m_autoExposureMaxGain < value)
    {
        QSignalBlocker blocker(settingsUI()->autoExposureMaxGainSpin);
        m_settings.m_autoExposureMaxGain = value;
        settingsUI()->autoExposureMaxGainSpin->setValue(value);
        applySetting("autoExposureMaxGain");
    }
    applySetting("autoExposureMinGain");
}

void CameraGUI::on_autoExposureMaxGainSpin_valueChanged(int value)
{
    m_settings.m_autoExposureMaxGain = std::max(m_settings.m_autoExposureMinGain, value);
    if (m_settings.m_autoExposureMaxGain != value)
    {
        QSignalBlocker blocker(settingsUI()->autoExposureMaxGainSpin);
        settingsUI()->autoExposureMaxGainSpin->setValue(m_settings.m_autoExposureMaxGain);
    }
    applySetting("autoExposureMaxGain");
}

void CameraGUI::on_autoExposureMaxChangeSpin_valueChanged(double value)
{
    m_settings.m_autoExposureMaxChangePercent = value;
    applySetting("autoExposureMaxChangePercent");
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

void CameraGUI::on_saveImageButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Save image"),
        m_settings.m_imageFileName,
        tr("Image (*.png *.jpg *.jpeg);;PNG image (*.png);;JPEG image (*.jpg *.jpeg)"));

    if (!fileName.isEmpty())
    {
        QImage image(m_imageScene->sceneRect().size().toSize(), QImage::Format_ARGB32);
        image.fill(Qt::transparent);
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        m_imageScene->render(&painter); // Should render full image, regardless of zoom settings
        if (!image.save(fileName)) {
            QMessageBox::warning(this, tr("Save image"), tr("Failed to save image to %1").arg(fileName));
        }
    }
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
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Save image"),
        m_settings.m_imageFileName,
        tr("Image (*.png *.jpg *.jpeg);;PNG image (*.png);;JPEG image (*.jpg *.jpeg)"));

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

void CameraGUI::on_keogramButton_toggled(bool checked)
{
    m_settings.m_keogramEnabled = checked;
    applySetting("keogramEnabled");
}

void CameraGUI::on_keogramPathEdit_editingFinished()
{
    m_settings.m_keogramFileName = settingsUI()->keogramPathEdit->text();
    applySetting("keogramFileName");
}

void CameraGUI::on_keogramPathButton_clicked()
{
    const QString fileName = QFileDialog::getSaveFileName(this, tr("Save keogram"), m_settings.m_keogramFileName, tr("Image (*.png *.jpg *.jpeg)"));

    if (!fileName.isEmpty())
    {
        m_settings.m_keogramFileName = fileName;
        settingsUI()->keogramPathEdit->setText(fileName);
        applySetting("keogramFileName");
    }
}

void CameraGUI::on_keogramDirectionCombo_currentIndexChanged(int index)
{
    m_settings.m_keogramDirection = static_cast<CameraSettings::KeogramDirection>(qBound(0, index, 1));
    applySetting("keogramDirection");
}

void CameraGUI::on_keogramDayModeCombo_currentIndexChanged(int index)
{
    m_settings.m_keogramDayMode = static_cast<CameraSettings::KeogramDayMode>(qBound(0, index, 1));
    applySetting("keogramDayMode");
}

void CameraGUI::on_keogramSamplePeriodSpin_valueChanged(int value)
{
    m_settings.m_keogramSamplePeriodMinutes = value;
    applySetting("keogramSamplePeriodMinutes");
}

void CameraGUI::on_keogramPreviewCheck_toggled(bool checked)
{
    m_settings.m_keogramShowPreview = checked;
    applySetting("keogramShowPreview");

    if (!checked) {
        updateKeogramPreview(QImage(), QString(), false);
    }
}

void CameraGUI::on_youtubeStreamButton_toggled(bool checked)
{
    const QString key = settingsUI()->youtubeStreamKeyEdit->text();

    if (checked && key.trimmed().isEmpty())
    {
        updateYouTubeStreamButtonEnabled();
        return;
    }

    const bool keyChanged = m_settings.m_youtubeStreamKey != key;
    m_settings.m_youtubeStreamKey = key;
    m_settings.m_youtubeStreamEnabled = checked;

    if (keyChanged)
    {
        applySettings({"youtubeStreamKey", "youtubeStreamEnabled"});
    }
    else
    {
        applySetting("youtubeStreamEnabled");
    }
}

void CameraGUI::on_youtubeStreamUrlEdit_editingFinished()
{
    m_settings.m_youtubeStreamUrl = settingsUI()->youtubeStreamUrlEdit->text();
    applySetting("youtubeStreamUrl");
}

void CameraGUI::on_youtubeStreamKeyEdit_editingFinished()
{
    m_settings.m_youtubeStreamKey = settingsUI()->youtubeStreamKeyEdit->text();
    updateYouTubeStreamButtonEnabled();

    if (m_settings.m_youtubeStreamKey.trimmed().isEmpty() && m_settings.m_youtubeStreamEnabled)
    {
        m_settings.m_youtubeStreamEnabled = false;
        applySettings({"youtubeStreamKey", "youtubeStreamEnabled"});
    }
    else
    {
        applySetting("youtubeStreamKey");
    }
}

void CameraGUI::on_youtubeStreamSourceCombo_currentIndexChanged(int index)
{
    m_settings.m_youtubeStreamPostProcessed = index == 1;
    applySetting("youtubeStreamPostProcessed");
}

void CameraGUI::on_youtubeStreamBitrateCombo_activated(int index)
{
    const QVariant bitrateData = settingsUI()->youtubeStreamBitrateCombo->itemData(index);
    if (bitrateData.isValid()) {
        m_settings.m_youtubeStreamBitrateKbps = qBound(100, bitrateData.toInt(), 240000);
    } else {
        m_settings.m_youtubeStreamBitrateKbps = parseBitrateKbps(
            settingsUI()->youtubeStreamBitrateCombo->currentText(),
            m_settings.m_youtubeStreamBitrateKbps);
    }
    updateYouTubeBitrateCombo();
    applySetting("youtubeStreamBitrateKbps");
}

void CameraGUI::on_youtubeStreamBitrateCombo_editingFinished()
{
    applyYouTubeBitrateComboText();
}

void CameraGUI::on_youtubeStreamFpsSpin_valueChanged(int value)
{
    m_settings.m_youtubeStreamFps = value;
    applySetting("youtubeStreamFps");
}

void CameraGUI::on_youtubeStreamWidthSpin_valueChanged(int value)
{
    m_settings.m_youtubeStreamWidth = value;
    applySetting("youtubeStreamWidth");
}

void CameraGUI::on_youtubeStreamHeightSpin_valueChanged(int value)
{
    m_settings.m_youtubeStreamHeight = value;
    applySetting("youtubeStreamHeight");
}

void CameraGUI::on_videoCodecCombo_currentIndexChanged(int index)
{
    m_settings.m_videoCodec = static_cast<CameraSettings::VideoCodec>(qBound(0, index, 1));
    applySetting("videoCodec");
}

void CameraGUI::on_videoRecordBitrateCombo_activated(int index)
{
    const QVariant bitrateData = settingsUI()->videoRecordBitrateCombo->itemData(index);
    if (bitrateData.isValid()) {
        m_settings.m_videoRecordBitrateKbps = qBound(0, bitrateData.toInt(), 240000);
    } else {
        m_settings.m_videoRecordBitrateKbps = parseVideoRecordBitrateKbps(
            settingsUI()->videoRecordBitrateCombo->currentText(),
            m_settings.m_videoRecordBitrateKbps);
    }
    updateVideoRecordBitrateCombo();
    applySetting("videoRecordBitrateKbps");
}

void CameraGUI::on_videoRecordBitrateCombo_editingFinished()
{
    applyVideoRecordBitrateComboText();
}

void CameraGUI::on_videoHwAccelerationCheck_toggled(bool checked)
{
    m_settings.m_videoHwAcceleration = checked;
    applySetting("videoHwAcceleration");
}

void CameraGUI::on_streamBufferingSecondsSpin_valueChanged(double value)
{
    m_settings.m_streamBufferingSeconds = value;
    applySetting("streamBufferingSeconds");
}

void CameraGUI::on_videoPreRecordBufferSpin_valueChanged(int value)
{
    m_settings.m_videoPreRecordBufferSeconds = value;
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("videoPreRecordBufferSeconds");
}

void CameraGUI::on_imageRecordLimitSpin_valueChanged(int value)
{
    m_settings.m_imageRecordLimit = value;
    applySetting("imageRecordLimit");
}

void CameraGUI::on_videoRecordLimitSpin_valueChanged(int value)
{
    m_settings.m_videoRecordLimitSeconds = value;
    applySetting("videoRecordLimitSeconds");
}

void CameraGUI::on_recordRawFitsCheck_toggled(bool checked)
{
    m_settings.m_recordRawFits = checked;
    applySetting("recordRawFits");
    applyImageToolTip();
}

void CameraGUI::on_recordCalibratedMediaCheck_toggled(bool checked)
{
    m_settings.m_recordCalibratedMedia = checked;
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("recordCalibratedMedia");
    applyImageToolTip();
    applyVideoToolTip();
}

void CameraGUI::on_recordFilteredMediaCheck_toggled(bool checked)
{
    m_settings.m_recordFilteredMedia = checked;
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("recordFilteredMedia");
    applyImageToolTip();
    applyVideoToolTip();
}

void CameraGUI::on_recordPostProcessedMediaCheck_toggled(bool checked)
{
    m_settings.m_recordPostProcessedMedia = checked;
    updateVideoPreRecordBufferMemoryLabel();
    applySetting("recordPostProcessedMedia");
    applyImageToolTip();
    applyVideoToolTip();
}

void CameraGUI::on_stackEnabledCheck_toggled(bool checked)
{
    m_settings.m_stackEnabled = checked;
    updateCameraSettingsVisibility();
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
    updateCameraSettingsVisibility();
    applySetting("stackMethod");
}

void CameraGUI::on_stackAlignmentCombo_currentIndexChanged(int index)
{
    m_settings.m_stackAlignmentMethod = static_cast<CameraSettings::StackAlignmentMethod>(index);
    applySetting("stackAlignmentMethod");
}

void CameraGUI::on_stackDisplayModeCombo_currentIndexChanged(int index)
{
    m_settings.m_stackDisplayMode = static_cast<CameraSettings::StackDisplayMode>(index);
    const bool frameControlsEnabled = m_settings.m_stackDisplayMode == CameraSettings::StackDisplayHistoryFrame;
    settingsUI()->stackDisplayFrameLabel->setEnabled(frameControlsEnabled);
    settingsUI()->stackDisplayFrameSpin->setEnabled(frameControlsEnabled);
    applySetting("stackDisplayMode");
}

void CameraGUI::on_stackDisplayFrameSpin_valueChanged(int value)
{
    m_settings.m_stackDisplayFrameIndex = std::max(0, value - 1);
    applySetting("stackDisplayFrameIndex");
}

void CameraGUI::on_stackDeleteFrameButton_clicked()
{
    if (m_camera) {
        m_camera->getInputMessageQueue()->push(Camera::MsgDeleteStackFrame::create(std::max(0, settingsUI()->stackDisplayFrameSpin->value() - 1)));
    }
}

void CameraGUI::on_stackRejectBadFramesCheck_toggled(bool checked)
{
    m_settings.m_stackRejectBadFrames = checked;
    applySetting("stackRejectBadFrames");
}

void CameraGUI::on_scaleEnabledCheck_toggled(bool checked)
{
    m_settings.m_scaleEnabled = checked;
    updateScaleControls();
    applySetting("scaleEnabled");
}

void CameraGUI::on_scaleWidthSpin_valueChanged(int value)
{
    m_settings.m_scaleWidth = value;
    updateScaleControls();
    applySetting("scaleWidth");
}

void CameraGUI::on_scaleHeightSpin_valueChanged(int value)
{
    m_settings.m_scaleHeight = value;
    updateScaleControls();
    applySetting("scaleHeight");
}

void CameraGUI::on_scaleKeepAspectRatioCheck_toggled(bool checked)
{
    m_settings.m_scaleKeepAspectRatio = checked;
    updateScaleControls();
    applySetting("scaleKeepAspectRatio");
}

void CameraGUI::on_scaleJustificationCombo_currentIndexChanged(int index)
{
    m_settings.m_scaleJustification = static_cast<CameraSettings::ScaleJustification>(
        qBound(static_cast<int>(CameraSettings::ScaleJustifyCenter),
            index,
            static_cast<int>(CameraSettings::ScaleJustifyBottom)));
    updateScaleControls();
    applySetting("scaleJustification");
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

void CameraGUI::on_azimuthOffsetSpin_valueChanged(double value)
{
    m_settings.m_azimuthOffset = static_cast<float>(value);
    if (!m_settings.m_directionSensor.isEmpty()) {
        syncFromDirectionSensors();
    } else if (!m_settings.m_rotator.isEmpty()) {
        syncFromSelectedGs232Controller();
    }
    applySetting("azimuthOffset");
}

void CameraGUI::on_elevationOffsetSpin_valueChanged(double value)
{
    m_settings.m_elevationOffset = static_cast<float>(value);
    if (!m_settings.m_directionSensor.isEmpty()) {
        syncFromDirectionSensors();
    } else if (!m_settings.m_rotator.isEmpty()) {
        syncFromSelectedGs232Controller();
    }
    applySetting("elevationOffset");
}

void CameraGUI::on_rollOffsetSpin_valueChanged(double value)
{
    m_settings.m_rollOffset = static_cast<float>(value);
    if (!m_settings.m_directionSensor.isEmpty()) {
        syncFromDirectionSensors();
    }
    applySetting("rollOffset");
}

void CameraGUI::on_directionSourceCombo_currentIndexChanged(int index)
{
    const QString sourceId = settingsUI()->directionSourceCombo->itemData(index).toString();
    QStringList settingsKeys = {"rotator", "directionSensor"};

    if (directionSourceIsRotator(sourceId))
    {
        m_settings.m_rotator = directionSourceValue(sourceId, kDirectionSourceRotatorPrefix);
        m_settings.m_directionSensor.clear();
        stopDirectionSensors();
        syncFromSelectedGs232Controller();
    }
    else if (directionSourceIsSensor(sourceId))
    {
        m_settings.m_rotator.clear();
        m_settings.m_directionSensor = directionSourceValue(sourceId, kDirectionSourceSensorPrefix);
        startDirectionSensors();
    }
    else
    {
        m_settings.m_rotator.clear();
        m_settings.m_directionSensor.clear();
        stopDirectionSensors();
    }

    updatePositionControls();
    applySettings(settingsKeys);
}

void CameraGUI::on_fovModeCombo_currentIndexChanged(int index)
{
    m_settings.m_fovMode = static_cast<CameraSettings::FovMode>(qBound(
        static_cast<int>(CameraSettings::FovModeDirect),
        index,
        static_cast<int>(CameraSettings::FovModeSensorFocalLength)));
    updateFovControls();
    applySettings({"fovMode", "fov"});
}

void CameraGUI::on_fovSpin_valueChanged(double value)
{
    m_settings.m_fov = static_cast<float>(value);
    applySetting("fov");
}

void CameraGUI::on_fovSensorWidthSpin_valueChanged(double value)
{
    m_settings.m_fovSensorWidthMm = value;
    updateCalculatedFov();
    applySetting("fovSensorWidthMm");
}

void CameraGUI::on_fovSensorHeightSpin_valueChanged(double value)
{
    m_settings.m_fovSensorHeightMm = value;
    updateCalculatedFov();
    applySetting("fovSensorHeightMm");
}

void CameraGUI::on_fovFocalLengthSpin_valueChanged(double value)
{
    m_settings.m_fovFocalLengthMm = value;
    updateCalculatedFov();
    applySetting("fovFocalLengthMm");
}

void CameraGUI::on_lensProjectionCombo_currentIndexChanged(int index)
{
    m_settings.m_lensProjection = static_cast<CameraSettings::LensProjection>(index);
    applySetting("lensProjection");
}

void CameraGUI::on_lensCenterOffsetXSpin_valueChanged(double value)
{
    m_settings.m_lensCenterOffsetX = value;
    applySetting("lensCenterOffsetX");
}

void CameraGUI::on_lensCenterOffsetYSpin_valueChanged(double value)
{
    m_settings.m_lensCenterOffsetY = value;
    applySetting("lensCenterOffsetY");
}

void CameraGUI::on_lensDistortionK1Spin_valueChanged(double value)
{
    m_settings.m_lensDistortionK1 = value;
    applySetting("lensDistortionK1");
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
    settingsUI()->postProcessWhiteBalanceHighlightProtectionLabel->setEnabled(manual);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSlider->setEnabled(manual);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin->setEnabled(manual);
}

void CameraGUI::updateHistogramStretchControls()
{
    const CameraSettings::HistogramStretch stretchMode = m_settings.m_histogramStretch;
    const bool stretchEnabled = stretchMode != CameraSettings::HistogramStretchOff;
    const bool gammaMode = stretchMode == CameraSettings::HistogramStretchGamma;
    const bool asinhMode = stretchMode == CameraSettings::HistogramStretchAsinh;
    const bool logMode = stretchMode == CameraSettings::HistogramStretchLog;
    const bool pointControls = stretchMode != CameraSettings::HistogramStretchCLAHE;

    settingsUI()->histogramStretchBlackPointLabel->setEnabled(stretchEnabled && pointControls);
    settingsUI()->histogramStretchBlackPointSlider->setEnabled(stretchEnabled && pointControls);
    settingsUI()->histogramStretchBlackPointSpin->setEnabled(stretchEnabled && pointControls);
    settingsUI()->histogramStretchWhitePointLabel->setEnabled(stretchEnabled && pointControls);
    settingsUI()->histogramStretchWhitePointSlider->setEnabled(stretchEnabled && pointControls);
    settingsUI()->histogramStretchWhitePointSpin->setEnabled(stretchEnabled && pointControls);

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

    if (m_detectionRoiRectItem)
    {
        m_imageScene->removeItem(m_detectionRoiRectItem);
        delete m_detectionRoiRectItem;
        m_detectionRoiRectItem = nullptr;
    }

    if (m_lastImage.isNull()) {
        return;
    }

    const QRect imageBounds(0, 0, m_lastImage.width(), m_lastImage.height());

    if (m_settings.m_showDetectionRoi)
    {
        QPen pen(QColor(255, 215, 0));
        pen.setWidth(2);
        pen.setStyle(Qt::DashLine);

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

    if (m_settings.m_showDetectionRoi
        && (m_settings.m_detectionRoiWidth > 0)
        && (m_settings.m_detectionRoiHeight > 0))
    {
        const QRect clipped = QRect(
            m_settings.m_detectionRoiX,
            m_settings.m_detectionRoiY,
            m_settings.m_detectionRoiWidth,
            m_settings.m_detectionRoiHeight).intersected(imageBounds);
        if (clipped.isValid() && !clipped.isEmpty())
        {
            QPen roiPen(QColor(0, 220, 255));
            roiPen.setWidth(2);
            roiPen.setStyle(Qt::DashLine);
            m_detectionRoiRectItem = m_imageScene->addRect(QRectF(clipped), roiPen);
            m_detectionRoiRectItem->setZValue(1.1);
        }
    }
}

void CameraGUI::updatePlateSolveStartModeUi()
{
    QString searchRadiusLabelText = tr("Az/El search radius");
    const bool usesSearchRadius =
        m_settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovElevation
        || m_settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzEl
        || m_settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRoll
        || m_settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens;
    if (m_settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovElevation) {
        searchRadiusLabelText = tr("Elevation search radius");
    }
    settingsUI()->plateSolveSearchRadiusLabel->setText(searchRadiusLabelText);
    settingsUI()->plateSolveSearchRadiusLabel->setEnabled(usesSearchRadius);
    settingsUI()->plateSolveSearchRadiusSpin->setEnabled(usesSearchRadius);
}

void CameraGUI::updatePlateSolveDateTimeEdit()
{
    const QDateTime fallback = m_settings.m_plateSolveDateTimeUtc
        ? QDateTime::currentDateTimeUtc()
        : QDateTime::currentDateTime();
    const QDateTime dateTime = m_settings.m_plateSolveDateTime.isValid()
        ? (m_settings.m_plateSolveDateTimeUtc
            ? m_settings.m_plateSolveDateTime.toUTC()
            : m_settings.m_plateSolveDateTime.toLocalTime())
        : fallback;

    QSignalBlocker dateTimeBlocker(settingsUI()->plateSolveDateTimeEdit);
    QSignalBlocker utcBlocker(settingsUI()->plateSolveDateTimeUtcButton);
    // Display the edit in the same time spec as the UTC button, so a UTC-stored value shows its UTC
    // wall clock (not the local-shifted one). Without this the edit defaults to Qt::LocalTime and a
    // UTC instant is shown shifted by the local offset -- e.g. 21:53:53Z displayed as 22:53:53 (BST),
    // which made "21:53:53 + UTC checked" actually store 20:53:53Z.
    settingsUI()->plateSolveDateTimeEdit->setTimeSpec(
        m_settings.m_plateSolveDateTimeUtc ? Qt::UTC : Qt::LocalTime);
    settingsUI()->plateSolveDateTimeEdit->setDateTime(dateTime);
    const bool customDateTime = !m_settings.m_plateSolveUseCaptureDateTime;
    settingsUI()->plateSolveDateTimeEdit->setEnabled(customDateTime);
    settingsUI()->plateSolveDateTimeUtcButton->setChecked(m_settings.m_plateSolveDateTimeUtc);
    settingsUI()->plateSolveDateTimeUtcButton->setEnabled(customDateTime);
    settingsUI()->plateSolveDateTimeNowButton->setEnabled(customDateTime);
}

void CameraGUI::setPreviewDrawMode(PreviewDrawMode mode)
{
    m_previewDrawMode = mode;
    m_previewDragging = false;

    if ((mode == PreviewDrawModeNone) && m_previewDrawRectItem)
    {
        m_imageScene->removeItem(m_previewDrawRectItem);
        delete m_previewDrawRectItem;
        m_previewDrawRectItem = nullptr;
    }

    const bool drawing = mode != PreviewDrawModeNone;
    ui->imageView->setDragMode(drawing ? QGraphicsView::NoDrag : QGraphicsView::ScrollHandDrag);
    ui->imageView->viewport()->setCursor(drawing ? Qt::CrossCursor : Qt::ArrowCursor);
}

void CameraGUI::setMotionExclusionDrawMode(bool enabled)
{
    setPreviewDrawMode(enabled ? PreviewDrawModeMotionExclusion : PreviewDrawModeNone);
}

void CameraGUI::setDetectionRoiDrawMode(bool enabled)
{
    setPreviewDrawMode(enabled ? PreviewDrawModeDetectionRoi : PreviewDrawModeNone);
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

int CameraGUI::findStarDetectionAtImagePos(const QPointF& imagePos) const
{
    int bestIndex = -1;
    double bestDistanceSquared = std::numeric_limits<double>::max();

    for (int i = 0; i < m_lastStarDetections.size(); ++i)
    {
        const CameraPipelineStarDetection& star = m_lastStarDetections[i];
        const QPointF delta = star.m_center - imagePos;
        const double distanceSquared = delta.x() * delta.x() + delta.y() * delta.y();
        const double hitRadius = std::max(8.0, static_cast<double>(star.m_radius) + 6.0);

        if ((distanceSquared <= hitRadius * hitRadius) && (distanceSquared < bestDistanceSquared))
        {
            bestDistanceSquared = distanceSquared;
            bestIndex = i;
        }
    }

    return bestIndex;
}

QString CameraGUI::starDetectionDisplayName(const CameraPipelineStarDetection& star) const
{
    const QString label = star.m_label.trimmed();
    if (!label.isEmpty()) {
        return label;
    }
    return star.m_solved ? tr("Matched star") : tr("Detected star");
}

QString CameraGUI::starDetectionSearchTarget(const CameraPipelineStarDetection& star) const
{
    return star.m_label.trimmed();
}

QString CameraGUI::starDetectionDetails(const CameraPipelineStarDetection& star) const
{
    QStringList details;
    details << tr("Name: %1").arg(starDetectionDisplayName(star));
    details << tr("Image position: x=%1 px, y=%2 px")
        .arg(QString::number(star.m_center.x(), 'f', 1))
        .arg(QString::number(star.m_center.y(), 'f', 1));

    if (star.m_solved)
    {
        details << tr("Projected position: x=%1 px, y=%2 px")
            .arg(QString::number(star.m_projectedCenter.x(), 'f', 1))
            .arg(QString::number(star.m_projectedCenter.y(), 'f', 1));
        details << tr("Match error: %1 px").arg(QString::number(star.m_matchDistancePixels, 'f', 2));
        details << tr("Catalog magnitude: %1").arg(QString::number(star.m_catalogMagnitude, 'f', 2));
        if (hasCatalogCoordinates(star))
        {
            details << tr("Catalog coordinates: RA=%1 deg, Dec=%2 deg")
                .arg(QString::number(star.m_catalogRightAscensionDegrees, 'f', 8))
                .arg(QString::number(star.m_catalogDeclinationDegrees, 'f', 8));
        }

        if (!star.m_catalogSpectralType.trimmed().isEmpty()) {
            details << tr("Spectral type: %1").arg(star.m_catalogSpectralType.trimmed());
        }
    }

    details << tr("Peak: %1").arg(QString::number(star.m_peakValue, 'f', 1));
    details << tr("Radius: %1 px").arg(QString::number(star.m_radius, 'f', 1));
    details << tr("Flux: %1").arg(QString::number(star.m_flux, 'f', 1));
    details << tr("SNR: %1").arg(QString::number(star.m_snr, 'f', 1));
    details << tr("FWHM: %1 px").arg(QString::number(star.m_fwhm, 'f', 2));
    details << tr("Quality: %1").arg(QString::number(star.m_qualityScore, 'f', 2));
    details << tr("Roundness: %1").arg(QString::number(star.m_roundness, 'f', 2));
    details << tr("Aspect ratio: %1").arg(QString::number(star.m_aspectRatio, 'f', 2));
    details << tr("Saturated: %1").arg(star.m_saturated ? tr("yes") : tr("no"));
    details << tr("Hot pixel suspect: %1").arg(star.m_hotPixelSuspect ? tr("yes") : tr("no"));
    return details.join('\n');
}

void CameraGUI::showStarDetectionInfoDialog(const CameraPipelineStarDetection& star)
{
    QMessageBox::information(this, tr("Star detection"), starDetectionDetails(star));
}

bool CameraGUI::showStarDetectionContextMenu(const QPoint& viewportPos, const QPoint& globalPos)
{
    if (m_lastImage.isNull() || !m_imagePixmapItem || m_lastStarDetections.isEmpty()) {
        return false;
    }

    const QPointF imagePos = ui->imageView->mapToScene(viewportPos);
    const QRectF imageBounds(QPointF(0.0, 0.0), QSizeF(m_lastImage.size()));
    if (!imageBounds.contains(imagePos)) {
        return false;
    }

    const int starIndex = findStarDetectionAtImagePos(imagePos);
    if (starIndex < 0) {
        return false;
    }

    const CameraPipelineStarDetection star = m_lastStarDetections[starIndex];
    const QString target = starDetectionSearchTarget(star);
    const QUrl simbadUrl = simbadUrlForStarDetection(star, target);
    QMenu menu(this);
    menu.addSection(starDetectionDisplayName(star));
    QAction *infoAction = menu.addAction(tr("Star information..."));
    QAction *skyMapAction = menu.addAction(tr("Find in Sky Map"));
    QAction *simbadAction = menu.addAction(tr("Open in SIMBAD"));
    QAction *copyNameAction = menu.addAction(tr("Copy name"));
    QAction *copyDetailsAction = menu.addAction(tr("Copy details"));

    const bool hasTarget = !target.isEmpty();
    skyMapAction->setEnabled(hasTarget);
    simbadAction->setEnabled(simbadUrl.isValid() && !simbadUrl.isEmpty());
    copyNameAction->setEnabled(hasTarget);

    QAction *selectedAction = menu.exec(globalPos);
    if (!selectedAction) {
        return true;
    }

    if (selectedAction == infoAction)
    {
        showStarDetectionInfoDialog(star);
    }
    else if (selectedAction == skyMapAction)
    {
        if (!FeatureWebAPIUtils::skyMapFind(target)) {
            QMessageBox::warning(this, tr("Sky Map"), tr("No Sky Map feature could find \"%1\".").arg(target));
        }
    }
    else if (selectedAction == simbadAction)
    {
        QDesktopServices::openUrl(simbadUrl);
    }
    else if (selectedAction == copyNameAction)
    {
        QApplication::clipboard()->setText(target);
    }
    else if (selectedAction == copyDetailsAction)
    {
        QApplication::clipboard()->setText(starDetectionDetails(star));
    }

    return true;
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

void CameraGUI::on_postProcessWhiteBalanceHighlightProtectionSlider_valueChanged(int value)
{
    const double protection = sliderValueToDoubleSpinBox(settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin, value);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin->setValue(protection);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceHighlightProtection = protection;
    applySetting("postProcessWhiteBalanceHighlightProtection");
}

void CameraGUI::on_postProcessWhiteBalanceHighlightProtectionSpin_valueChanged(double value)
{
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSlider->blockSignals(true);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSlider->setValue(doubleSpinBoxValueToSlider(settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin, value));
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSlider->blockSignals(false);
    m_settings.m_postProcessWhiteBalanceHighlightProtection = value;
    applySetting("postProcessWhiteBalanceHighlightProtection");
}

void CameraGUI::on_postProcessUseCudaCheck_toggled(bool checked)
{
    m_settings.m_postProcessUseCuda = checked;
    applySetting("postProcessUseCuda");
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
        qBound(static_cast<int>(CameraSettings::HistogramStretchOff), index, static_cast<int>(CameraSettings::HistogramStretchCLAHE)));
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

void CameraGUI::on_edgeDisplayModeCombo_currentIndexChanged(int index)
{
    m_settings.m_edgeDisplayMode = static_cast<CameraSettings::EdgeDisplayMode>(
        qBound(static_cast<int>(CameraSettings::EdgeDisplayOverlay), index, static_cast<int>(CameraSettings::EdgeDisplayEdgesOnly)));
    applySetting("edgeDisplayMode");
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

void CameraGUI::on_cannyEdgeSlider_valueChanged(int value)
{
    m_settings.m_cannyEdge = value / 100.0;
    settingsUI()->cannyEdgeSpin->blockSignals(true);
    settingsUI()->cannyEdgeSpin->setValue(m_settings.m_cannyEdge);
    settingsUI()->cannyEdgeSpin->blockSignals(false);
    applySetting("cannyEdge");
}

void CameraGUI::on_cannyEdgeSpin_valueChanged(double value)
{
    settingsUI()->cannyEdgeSlider->blockSignals(true);
    settingsUI()->cannyEdgeSlider->setValue(static_cast<int>(value * 100.0));
    settingsUI()->cannyEdgeSlider->blockSignals(false);
    m_settings.m_cannyEdge = value;
    applySetting("cannyEdge");
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

void CameraGUI::on_imageRotationCombo_currentIndexChanged(int index)
{
    m_settings.m_imageRotation = qBound(0, index, 3) * 90;
    applySetting("imageRotation");
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
    const QColor color = QColorDialog::getColor(m_settings.m_dateTimeColor, this, tr("Select date/time text colour"), QColorDialog::ShowAlphaChannel);

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
    const QColor color = QColorDialog::getColor(m_settings.m_equatorialGridColor, this, tr("Select equatorial grid colour"), QColorDialog::ShowAlphaChannel);

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
    const QColor color = QColorDialog::getColor(m_settings.m_altAzGridColor, this, tr("Select alt-az grid colour"), QColorDialog::ShowAlphaChannel);

    if (color.isValid())
    {
        m_settings.m_altAzGridColor = color;
        updateColorButton(settingsUI()->altAzGridColorButton, m_settings.m_altAzGridColor);
        applySetting("altAzGridColor");
    }
}

void CameraGUI::on_constellationCheck_toggled(bool checked)
{
    m_settings.m_constellation = checked;
    applySetting("constellation");
}

void CameraGUI::on_constellationOverlayCombo_currentIndexChanged(int index)
{
    m_settings.m_constellationOverlay = static_cast<CameraSettings::ConstellationOverlay>(index);
    applySetting("constellationOverlay");
}

void CameraGUI::on_constellationColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_constellationColor, this, tr("Select constellation overlay colour"), QColorDialog::ShowAlphaChannel);

    if (color.isValid())
    {
        m_settings.m_constellationColor = color;
        updateColorButton(settingsUI()->constellationColorButton, m_settings.m_constellationColor);
        applySetting("constellationColor");
    }
}

void CameraGUI::on_trackObjectsCheck_toggled(bool checked)
{
    m_settings.m_trackObjects = checked;
    applySetting("trackObjects");
}

void CameraGUI::on_trackObjectTrailsCheck_toggled(bool checked)
{
    m_settings.m_trackObjectTrails = checked;
    applySetting("trackObjectTrails");
}

void CameraGUI::on_trackObjectHeatMapCheck_toggled(bool checked)
{
    m_settings.m_trackObjectHeatMap = checked;
    applySetting("trackObjectHeatMap");
}

void CameraGUI::on_trackObjectClearHeatMapButton_clicked()
{
    if (m_camera) {
        m_camera->getInputMessageQueue()->push(Camera::MsgClearTrackedObjectHeatMap::create());
    }
}

void CameraGUI::on_trackObjectMinElevationSpin_valueChanged(double value)
{
    m_settings.m_trackObjectMinElevation = value;
    applySetting("trackObjectMinElevation");
}

void CameraGUI::on_trackObjectColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_trackObjectColor, this, tr("Select tracked object colour"), QColorDialog::ShowAlphaChannel);

    if (color.isValid())
    {
        m_settings.m_trackObjectColor = color;
        updateColorButton(settingsUI()->trackObjectColorButton, m_settings.m_trackObjectColor);
        applySetting("trackObjectColor");
    }
}

void CameraGUI::on_trackObjectFontCombo_currentFontChanged(const QFont& font)
{
    m_settings.m_trackObjectFontFamily = font.family();
    applySetting("trackObjectFontFamily");
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
    const QColor color = QColorDialog::getColor(m_settings.m_overlayTextColor, this, tr("Select overlay text colour"), QColorDialog::ShowAlphaChannel);

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
    if (!m_settings.m_histogramVisible)
    {
        m_settings.m_histogramVisible = true;
        applySetting("histogramVisible");
    }

    if (!m_histogramDialog)
    {
        m_histogramDialog = new CameraHistogramDialog(m_lastHistogramData, this);
        m_histogramDialog->setAttribute(Qt::WA_DeleteOnClose);
        connect(m_histogramDialog, &QObject::destroyed, this, [this]() {
            m_histogramDialog = nullptr;
            if (m_settings.m_histogramVisible)
            {
                m_settings.m_histogramVisible = false;
                applySetting("histogramVisible");
            }
        });
    }
    else
    {
        m_histogramDialog->updateHistogram(m_lastHistogramData);
    }

    m_histogramDialog->show();
    m_histogramDialog->raise();
    m_histogramDialog->activateWindow();
}

void CameraGUI::on_detectionHistoryButton_clicked()
{
    if (!m_detectionHistoryDialog)
    {
        m_detectionHistoryDialog = new CameraDetectionHistory(m_detectionHistory, this);
        m_detectionHistoryDialog->setAttribute(Qt::WA_DeleteOnClose);
        connect(m_detectionHistoryDialog, &CameraDetectionHistory::clearHistoryRequested, this, &CameraGUI::on_detectionHistoryClearRequested);
        connect(m_detectionHistoryDialog, &CameraDetectionHistory::detectionActivated, this, &CameraGUI::on_detectionHistoryEntryActivated);
        connect(m_detectionHistoryDialog, &QObject::destroyed, this, [this]() { m_detectionHistoryDialog = nullptr; });
    }
    else
    {
        m_detectionHistoryDialog->updateHistory(m_detectionHistory);
    }

    m_detectionHistoryDialog->show();
    m_detectionHistoryDialog->raise();
    m_detectionHistoryDialog->activateWindow();
}

void CameraGUI::on_detectionHistoryClearRequested()
{
    MessageQueue *detectorQueue = m_camera ? m_camera->getDetectorInputMessageQueue() : nullptr;
    if (detectorQueue) {
        detectorQueue->push(CameraObjectDetector::MsgClearObjectDetectionHistory::create());
    }
}

void CameraGUI::on_detectionHistoryEntryActivated(const CameraDetectionHistoryEntry& entry)
{
    if (!m_settings.isFileCamera()) {
        return;
    }

    if (m_settings.isImageFileSequenceCamera() && (entry.m_playbackFrameNumber > 0))
    {
        const int frameIndex = entry.m_playbackFrameNumber - 1;
        if ((frameIndex < 0) || (frameIndex >= m_settings.m_imageFileCameraPaths.size())) {
            return;
        }

        m_imageSequenceTimer.stop();
        {
            QSignalBlocker blocker(ui->playPauseVideo);
            ui->playPauseVideo->setChecked(false);
        }
        showImageSequenceFrame(frameIndex);
        return;
    }

    if (m_settings.isVideoFileCamera() && (entry.m_playbackPositionMs >= 0))
    {
        const qint64 maxPosition = m_playbackDurationMs > 0 ? m_playbackDurationMs : entry.m_playbackPositionMs;
        const qint64 position = qBound<qint64>(0, entry.m_playbackPositionMs, maxPosition);
        updatePlaybackPositionLabel(position);
        sendVideoFileControl(CameraWorker::MsgVideoFileControl::Pause);
        sendVideoFileControl(CameraWorker::MsgVideoFileControl::Seek, position);
    }
}

void CameraGUI::on_defaultColorSettingsButton_clicked()
{
    settingsUI()->postProcessWhiteBalanceModeCombo->setCurrentIndex(0);
    settingsUI()->postProcessWhiteBalanceRedGainSpin->setValue(1);
    settingsUI()->postProcessWhiteBalanceGreenGainSpin->setValue(1);
    settingsUI()->postProcessWhiteBalanceBlueGainSpin->setValue(1);
    settingsUI()->postProcessWhiteBalanceHighlightProtectionSpin->setValue(0);
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
    settingsUI()->edgeDisplayModeCombo->setCurrentIndex(static_cast<int>(CameraSettings::EdgeDisplayOverlay));
    settingsUI()->sobelEdgeSpin->setValue(0.0);
    settingsUI()->cannyEdgeSpin->setValue(0.0);
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
    updateMotionExclusionPreview();
    applySetting("detectionRoiX");
}

void CameraGUI::on_detectionRoiYSpin_valueChanged(int value)
{
    m_settings.m_detectionRoiY = value;
    updateMotionExclusionPreview();
    applySetting("detectionRoiY");
}

void CameraGUI::on_detectionRoiWidthSpin_valueChanged(int value)
{
    m_settings.m_detectionRoiWidth = value;
    updateMotionExclusionPreview();
    applySetting("detectionRoiWidth");
}

void CameraGUI::on_detectionRoiHeightSpin_valueChanged(int value)
{
    m_settings.m_detectionRoiHeight = value;
    updateMotionExclusionPreview();
    applySetting("detectionRoiHeight");
}

void CameraGUI::on_detectionRoiShowButton_toggled(bool checked)
{
    m_settings.m_showDetectionRoi = checked;
    updateMotionExclusionPreview();
    applySetting("showDetectionRoi");
}

void CameraGUI::on_showStarDetectionBoxesCheck_toggled(bool checked)
{
    m_settings.m_showStarDetectionBoxes = checked;
    applySetting("showStarDetectionBoxes");
}

void CameraGUI::on_hideSyntheticNamesCheck_toggled(bool checked)
{
    m_settings.m_plateSolveLabelHideSyntheticNames = checked;
    applySetting("plateSolveLabelHideSyntheticNames");
}

void CameraGUI::on_detectionRoiDrawButton_clicked()
{
    setDetectionRoiDrawMode(true);
}

void CameraGUI::on_detectionRoiDeleteButton_clicked()
{
    setDetectionRoiDrawMode(false);

    m_settings.m_detectionRoiX = 0;
    m_settings.m_detectionRoiY = 0;
    m_settings.m_detectionRoiWidth = 0;
    m_settings.m_detectionRoiHeight = 0;

    settingsUI()->detectionRoiXSpin->blockSignals(true);
    settingsUI()->detectionRoiYSpin->blockSignals(true);
    settingsUI()->detectionRoiWidthSpin->blockSignals(true);
    settingsUI()->detectionRoiHeightSpin->blockSignals(true);
    settingsUI()->detectionRoiXSpin->setValue(0);
    settingsUI()->detectionRoiYSpin->setValue(0);
    settingsUI()->detectionRoiWidthSpin->setValue(0);
    settingsUI()->detectionRoiHeightSpin->setValue(0);
    settingsUI()->detectionRoiXSpin->blockSignals(false);
    settingsUI()->detectionRoiYSpin->blockSignals(false);
    settingsUI()->detectionRoiWidthSpin->blockSignals(false);
    settingsUI()->detectionRoiHeightSpin->blockSignals(false);

    updateMotionExclusionPreview();
    applySettings({"detectionRoiX", "detectionRoiY", "detectionRoiWidth", "detectionRoiHeight"});
}

void CameraGUI::on_detectionResetDefaultsButton_clicked()
{
    const CameraSettings defaults;

    m_settings.m_detectionRoiX = defaults.m_detectionRoiX;
    m_settings.m_detectionRoiY = defaults.m_detectionRoiY;
    m_settings.m_detectionRoiWidth = defaults.m_detectionRoiWidth;
    m_settings.m_detectionRoiHeight = defaults.m_detectionRoiHeight;
    m_settings.m_showDetectionRoi = defaults.m_showDetectionRoi;
    m_settings.m_showStarDetectionBoxes = defaults.m_showStarDetectionBoxes;
    m_settings.m_plateSolveLabelHideSyntheticNames = defaults.m_plateSolveLabelHideSyntheticNames;

    m_settings.m_motionDetect = defaults.m_motionDetect;
    m_settings.m_motionBackgroundSubtractor = defaults.m_motionBackgroundSubtractor;
    m_settings.m_motionMaskView = defaults.m_motionMaskView;
    m_settings.m_motionHistory = defaults.m_motionHistory;
    m_settings.m_motionVarThreshold = defaults.m_motionVarThreshold;
    m_settings.m_motionLearningRate = defaults.m_motionLearningRate;
    m_settings.m_motionConfirmFrames = defaults.m_motionConfirmFrames;
    m_settings.m_motionDownscale = defaults.m_motionDownscale;
    m_settings.m_motionDetectShadows = defaults.m_motionDetectShadows;
    m_settings.m_motionOpenSize = defaults.m_motionOpenSize;
    m_settings.m_motionCloseSize = defaults.m_motionCloseSize;
    m_settings.m_motionPersistenceFrames = defaults.m_motionPersistenceFrames;
    m_settings.m_motionBoxColor = defaults.m_motionBoxColor;
    m_settings.m_minContourArea = defaults.m_minContourArea;
    m_settings.m_motionExclusionRects = defaults.m_motionExclusionRects;
    m_settings.m_yoloIgnoredClassNames = defaults.m_yoloIgnoredClassNames;

    m_settings.m_starDetect = defaults.m_starDetect;
    m_settings.m_starThreshold = defaults.m_starThreshold;
    m_settings.m_starBackgroundBlur = defaults.m_starBackgroundBlur;
    m_settings.m_starMinArea = defaults.m_starMinArea;
    m_settings.m_starMaxArea = defaults.m_starMaxArea;
    m_settings.m_starMaxAspectRatio = defaults.m_starMaxAspectRatio;
    m_settings.m_starDebugView = defaults.m_starDebugView;
    m_settings.m_starColor = defaults.m_starColor;
    m_settings.m_plateSolveLabelMode = defaults.m_plateSolveLabelMode;
    m_settings.m_plateSolveMaxMagnitude = defaults.m_plateSolveMaxMagnitude;
    m_settings.m_plateSolveMinMatches = defaults.m_plateSolveMinMatches;
    m_settings.m_plateSolveMatchRadius = defaults.m_plateSolveMatchRadius;
    m_settings.m_plateSolveFinalMatchRadius = defaults.m_plateSolveFinalMatchRadius;
    m_settings.m_plateSolveAzElSearchRadius = defaults.m_plateSolveAzElSearchRadius;
    m_settings.m_plateSolveFovTolerance = defaults.m_plateSolveFovTolerance;
    m_settings.m_plateSolveStartMode = defaults.m_plateSolveStartMode;
    m_settings.m_plateSolveUseCaptureDateTime = defaults.m_plateSolveUseCaptureDateTime;
    m_settings.m_plateSolveDateTime = defaults.m_plateSolveDateTime;
    m_settings.m_plateSolveDateTimeUtc = defaults.m_plateSolveDateTimeUtc;
    m_settings.m_plateSolveCatalogSource = defaults.m_plateSolveCatalogSource;
    m_settings.m_plateSolveApplyMode = defaults.m_plateSolveApplyMode;
    m_settings.m_starCatalogDiskCacheSizeGb = defaults.m_starCatalogDiskCacheSizeGb;

    m_settings.m_diffMask = defaults.m_diffMask;
    m_settings.m_diffThreshold = defaults.m_diffThreshold;
    m_settings.m_diffMaskOpenSize = defaults.m_diffMaskOpenSize;
    m_settings.m_dilationSize = defaults.m_dilationSize;
    m_settings.m_diffMaskHistoryFrames = defaults.m_diffMaskHistoryFrames;
    m_settings.m_diffMaskCloseSize = defaults.m_diffMaskCloseSize;

    blockApplySettings(true);
    displaySettings();
    blockApplySettings(false);
    updateMotionExclusionPreview();

    applySettings({
        "detectionRoiX",
        "detectionRoiY",
        "detectionRoiWidth",
        "detectionRoiHeight",
        "showDetectionRoi",
        "showStarDetectionBoxes",
        "plateSolveLabelHideSyntheticNames",
        "motionDetect",
        "motionBackgroundSubtractor",
        "motionMaskView",
        "motionHistory",
        "motionVarThreshold",
        "motionLearningRate",
        "motionConfirmFrames",
        "motionDownscale",
        "motionDetectShadows",
        "motionOpenSize",
        "motionCloseSize",
        "motionPersistenceFrames",
        "motionBoxColor",
        "minContourArea",
        "showMotionExclusionRects",
        "motionExclusionRects",
        "yoloIgnoredClassNames",
        "starDetect",
        "starThreshold",
        "starBackgroundBlur",
        "starMinArea",
        "starMaxArea",
        "starMaxAspectRatio",
        "starDebugView",
        "starColor",
        "plateSolveLabelMode",
        "plateSolveMaxMagnitude",
        "plateSolveMinMatches",
        "plateSolveMatchRadius",
        "plateSolveFinalMatchRadius",
        "plateSolveSearchRadius",
        "plateSolveStartMode",
        "plateSolveUseCaptureDateTime",
        "plateSolveDateTime",
        "plateSolveDateTimeUtc",
        "plateSolveCatalogSource",
        "plateSolveApplyMode",
        "starCatalogDiskCacheSizeGb",
        "diffMask",
        "diffThreshold",
        "diffMaskOpenSize",
        "dilationSize",
        "diffMaskHistoryFrames",
        "diffMaskCloseSize"
    });
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

void CameraGUI::on_motionMaskViewCombo_currentIndexChanged(int index)
{
    m_settings.m_motionMaskView = static_cast<CameraSettings::MotionMaskView>(index);
    applySetting("motionMaskView");
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
    const QColor color = QColorDialog::getColor(m_settings.m_motionBoxColor, this, tr("Select bounding box colour"), QColorDialog::ShowAlphaChannel);

    if (color.isValid())
    {
        m_settings.m_motionBoxColor = color;
        updateColorButton(settingsUI()->motionBoxColorButton, color);
        applySetting("motionBoxColor");
    }
}

void CameraGUI::on_starDetectButton_toggled(bool checked)
{
    m_settings.m_starDetect = checked;
    m_settings.m_plateSolve = checked;
    applySettings({"starDetect", "plateSolve"});
}

void CameraGUI::on_starThresholdSpin_valueChanged(int value)
{
    m_settings.m_starThreshold = value;
    applySetting("starThreshold");
}

void CameraGUI::on_starBackgroundBlurSpin_valueChanged(int value)
{
    m_settings.m_starBackgroundBlur = value;
    applySetting("starBackgroundBlur");
}

void CameraGUI::on_starMinAreaSpin_valueChanged(int value)
{
    m_settings.m_starMinArea = value;
    applySetting("starMinArea");
}

void CameraGUI::on_starMaxAreaSpin_valueChanged(int value)
{
    m_settings.m_starMaxArea = value;
    applySetting("starMaxArea");
}

void CameraGUI::on_starMaxAspectRatioSpin_valueChanged(double value)
{
    m_settings.m_starMaxAspectRatio = value;
    applySetting("starMaxAspectRatio");
}

void CameraGUI::on_starDebugViewCombo_currentIndexChanged(int index)
{
    m_settings.m_starDebugView = static_cast<CameraSettings::StarDebugView>(index);
    applySetting("starDebugView");
}

void CameraGUI::on_plateSolveLabelModeCombo_currentIndexChanged(int index)
{
    m_settings.m_plateSolveLabelMode = static_cast<CameraSettings::PlateSolveLabelMode>(index);
    applySetting("plateSolveLabelMode");
}

void CameraGUI::on_starColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_starColor, this, tr("Select star colour"), QColorDialog::ShowAlphaChannel);

    if (color.isValid())
    {
        m_settings.m_starColor = color;
        updateColorButton(settingsUI()->starColorButton, color);
        applySetting("starColor");
    }
}

void CameraGUI::on_plateSolveMaxMagnitudeSpin_valueChanged(double value)
{
    m_settings.m_plateSolveMaxMagnitude = value;
    applySetting("plateSolveMaxMagnitude");
}

void CameraGUI::on_plateSolveMinMatchesSpin_valueChanged(int value)
{
    m_settings.m_plateSolveMinMatches = value;
    applySetting("plateSolveMinMatches");
}

void CameraGUI::on_plateSolveMatchRadiusSpin_valueChanged(double value)
{
    m_settings.m_plateSolveMatchRadius = value;
    applySetting("plateSolveMatchRadius");
}

void CameraGUI::on_plateSolveFinalMatchRadiusSpin_valueChanged(double value)
{
    m_settings.m_plateSolveFinalMatchRadius = value;
    applySetting("plateSolveFinalMatchRadius");
}

void CameraGUI::on_plateSolveSearchRadiusSpin_valueChanged(double value)
{
    m_settings.m_plateSolveAzElSearchRadius = value;
    applySetting("plateSolveSearchRadius");
}

void CameraGUI::on_plateSolveFovToleranceSpin_valueChanged(double value)
{
    m_settings.m_plateSolveFovTolerance = value;
    applySetting("plateSolveFovTolerance");
}

void CameraGUI::on_plateSolveStartModeCombo_currentIndexChanged(int index)
{
    m_settings.m_plateSolveStartMode = static_cast<CameraSettings::PlateSolveStartMode>(index);
    updatePlateSolveStartModeUi();
    applySetting("plateSolveStartMode");
}

void CameraGUI::on_plateSolveDateTimeModeCombo_currentIndexChanged(int index)
{
    m_settings.m_plateSolveUseCaptureDateTime = (index == 0);
    updatePlateSolveDateTimeEdit();
    applySetting("plateSolveUseCaptureDateTime");
}

void CameraGUI::on_plateSolveDateTimeEdit_dateTimeChanged(const QDateTime& dateTime)
{
    m_settings.m_plateSolveDateTime = m_settings.m_plateSolveDateTimeUtc
        ? QDateTime(dateTime.date(), dateTime.time(), Qt::UTC)
        : QDateTime(dateTime.date(), dateTime.time(), Qt::LocalTime);
    applySetting("plateSolveDateTime");
}

void CameraGUI::on_plateSolveDateTimeUtcButton_toggled(bool checked)
{
    const QDateTime dateTime = m_settings.m_plateSolveDateTime.isValid()
        ? m_settings.m_plateSolveDateTime
        : QDateTime::currentDateTime();

    m_settings.m_plateSolveDateTimeUtc = checked;
    // Reinterpret the displayed wall clock in the new spec (keep the same hh:mm:ss), matching
    // on_plateSolveDateTimeEdit_dateTimeChanged. The UTC button declares whether the entered time
    // IS UTC; it must NOT convert (toUTC would shift 21:53:53 local -> 20:53:53Z and silently change
    // the time the user typed).
    m_settings.m_plateSolveDateTime = QDateTime(
        dateTime.date(), dateTime.time(), checked ? Qt::UTC : Qt::LocalTime);
    updatePlateSolveDateTimeEdit();
    applySettings({"plateSolveDateTimeUtc", "plateSolveDateTime"});
}

void CameraGUI::on_plateSolveDateTimeNowButton_clicked()
{
    m_settings.m_plateSolveDateTime = m_settings.m_plateSolveDateTimeUtc
        ? QDateTime::currentDateTimeUtc()
        : QDateTime::currentDateTime();
    updatePlateSolveDateTimeEdit();
    applySetting("plateSolveDateTime");
}

void CameraGUI::on_plateSolveCatalogSourceCombo_currentIndexChanged(int index)
{
    m_settings.m_plateSolveCatalogSource = static_cast<CameraSettings::PlateSolveCatalogSource>(index);
    applySetting("plateSolveCatalogSource");
}

void CameraGUI::on_starCatalogDiskCacheSizeSpin_valueChanged(int value)
{
    m_settings.m_starCatalogDiskCacheSizeGb = value;
    applySetting("starCatalogDiskCacheSizeGb");
}

void CameraGUI::on_plateSolveApplyModeCombo_currentIndexChanged(int index)
{
    m_settings.m_plateSolveApplyMode = static_cast<CameraSettings::PlateSolveApplyMode>(index);
    applySetting("plateSolveApplyMode");
}

void CameraGUI::on_plateSolveDownloadCatalogButton_clicked()
{
    requestPlateSolveCatalogDownload();
}

void CameraGUI::on_plateSolveApplyButton_clicked()
{
    if (!m_lastPlateSolved) {
        return;
    }

    m_settings.m_azimuth = static_cast<float>(m_lastPlateSolveAzimuth);
    m_settings.m_elevation = static_cast<float>(m_lastPlateSolveElevation);

    QStringList settingsToApply {
        "azimuth", "elevation"
    };

    if (m_settings.m_plateSolveApplyMode >= CameraSettings::PlateSolveApplyAzElRoll) {
        m_settings.m_roll = static_cast<float>(m_lastPlateSolveRoll);
        settingsToApply.append("roll");
    }
    if (m_settings.m_plateSolveApplyMode >= CameraSettings::PlateSolveApplyAzElRollFov) {
        m_settings.m_fov = static_cast<float>(m_lastPlateSolveFov);
        settingsToApply.append("fov");
    }
    if (m_settings.m_plateSolveApplyMode >= CameraSettings::PlateSolveApplyAzElRollFovLens) {
        m_settings.m_lensCenterOffsetX = m_lastPlateSolveCenterOffsetX;
        m_settings.m_lensCenterOffsetY = m_lastPlateSolveCenterOffsetY;
        m_settings.m_lensDistortionK1 = m_lastPlateSolveDistortionK1;
        settingsToApply.append("lensCenterOffsetX");
        settingsToApply.append("lensCenterOffsetY");
        settingsToApply.append("lensDistortionK1");
    }

    blockApplySettings(true);
    displaySettings();
    blockApplySettings(false);

    applySettings(settingsToApply);
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

void CameraGUI::updateYoloButtonEnabled()
{
    const bool hasModelPath = !settingsUI()->yoloModelPathCombo->currentText().trimmed().isEmpty();
    const bool hasLabelsPath = !settingsUI()->yoloLabelsPathCombo->currentText().trimmed().isEmpty();
    const bool enabled = hasModelPath && hasLabelsPath;

    ui->yoloButton->setEnabled(enabled);

    if (!enabled && ui->yoloButton->isChecked()) {
        ui->yoloButton->setChecked(false);
    }
}

void CameraGUI::updateYouTubeStreamButtonEnabled()
{
    const bool enabled = !settingsUI()->youtubeStreamKeyEdit->text().trimmed().isEmpty();

    ui->youtubeStreamButton->setEnabled(enabled);

    if (!enabled && ui->youtubeStreamButton->isChecked())
    {
        const QSignalBlocker blocker(ui->youtubeStreamButton);
        ui->youtubeStreamButton->setChecked(false);
    }
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
    }
    else
    {
        return;
    }

    updateYoloButtonEnabled();
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

void CameraGUI::requestPlateSolveCatalogDownload()
{
    static const QString kHygCatalogUrl = QStringLiteral("https://codeberg.org/astronexus/hyg/media/branch/main/data/hyg/CURRENT/hyg_v42.csv.gz");
    static const QString kSirilAstroCatalogUrl = QStringLiteral("https://zenodo.org/records/14692304/files/siril_cat_healpix8_astro.dat.bz2?download=1");

    const bool downloadAuto = (m_settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogAuto);
    const bool downloadSirilAstro = downloadAuto
        || (m_settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogSirilAstroGaia);

    if (m_settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogSirilSpccGaia)
    {
        QMessageBox::information(this, tr("Catalog download"),
            tr("Siril Gaia DR3 SPCC is fetched online as needed and does not have a single local catalog file to download here."));
        return;
    }

    if (downloadAuto)
    {
        const QString hygArchiveFilename = CameraPlateSolver::downloadedCatalogArchivePath();

        if (!m_pendingPlateSolveDownloads.contains(hygArchiveFilename))
        {
            m_pendingPlateSolveDownloads.insert(hygArchiveFilename, kHygCatalogUrl);

            if (QFileInfo::exists(hygArchiveFilename))
            {
                if (!HttpDownloadManagerGUI::confirmDownload(hygArchiveFilename, this))
                {
                    handlePlateSolveCatalogDownloadComplete(hygArchiveFilename, true, kHygCatalogUrl, QString());
                }
                else
                {
                    m_dlm.download(QUrl(kHygCatalogUrl), hygArchiveFilename, this);
                }
            }
            else
            {
                m_dlm.download(QUrl(kHygCatalogUrl), hygArchiveFilename, this);
            }
        }
    }

    const QString catalogUrl = downloadSirilAstro ? kSirilAstroCatalogUrl : kHygCatalogUrl;
    const QString localArchiveFilename = downloadSirilAstro
        ? CameraPlateSolver::sirilAstroCompressedCatalogPath()
        : CameraPlateSolver::downloadedCatalogArchivePath();

    if (m_pendingPlateSolveDownloads.contains(localArchiveFilename)) {
        return;
    }

    m_pendingPlateSolveDownloads.insert(localArchiveFilename, catalogUrl);

    if (QFileInfo::exists(localArchiveFilename))
    {
        if (!HttpDownloadManagerGUI::confirmDownload(localArchiveFilename, this))
        {
            handlePlateSolveCatalogDownloadComplete(localArchiveFilename, true, catalogUrl, QString());
            return;
        }
    }

    m_dlm.download(QUrl(catalogUrl), localArchiveFilename, this);
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

void CameraGUI::handlePlateSolveCatalogDownloadComplete(const QString& filename, bool success, const QString& url, const QString& errorMessage)
{
    const QString requestedUrl = m_pendingPlateSolveDownloads.take(filename);
    if (requestedUrl.isEmpty()) {
        return;
    }

    if (!success)
    {
        QString error = errorMessage;
        if (error.isEmpty()) {
            error = tr("An unknown error occurred during download from %1 to %2.").arg(url, filename);
        }
        QMessageBox::warning(this, tr("Download failed"), error);
        return;
    }

    if (filename == CameraPlateSolver::sirilAstroCompressedCatalogPath())
    {
        QMessageBox::information(this, tr("Catalog downloaded"),
            tr("Downloaded Siril Gaia DR3 Astrometric catalog to %1.\n\nDecompress this .bz2 file to %2 before using the local astrometric catalog.")
                .arg(filename, CameraPlateSolver::sirilAstroCatalogPath()));
        return;
    }

    QString importError;
    if (!CameraPlateSolver::importDownloadedCatalogArchive(filename, &importError))
    {
        QMessageBox::warning(this, tr("Catalog import failed"),
            importError.isEmpty() ? tr("Failed to import downloaded HYG catalog.") : importError);
        return;
    }

    QMessageBox::information(this, tr("Catalog imported"),
        tr("Imported HYG catalog to %1").arg(CameraPlateSolver::downloadedCatalogCsvPath()));
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

void CameraGUI::on_yoloTileLargeImagesCheck_toggled(bool checked)
{
    m_settings.m_yoloTileLargeImages = checked;
    settingsUI()->yoloTileOverlapSpin->setEnabled(checked);
    applySetting("yoloTileLargeImages");
}

void CameraGUI::on_yoloTileOverlapSpin_valueChanged(int value)
{
    m_settings.m_yoloTileOverlapPercent = value;
    applySetting("yoloTileOverlapPercent");
}

void CameraGUI::on_yoloIgnoredClassNamesEdit_textChanged()
{
    QStringList ignoredClassNames;

    for (const QString& className : settingsUI()->yoloIgnoredClassNamesEdit->toPlainText().split(QRegularExpression("[\\r\\n]+"), Qt::SkipEmptyParts))
    {
        const QString trimmedClassName = className.trimmed();
        if (!trimmedClassName.isEmpty() && !ignoredClassNames.contains(trimmedClassName, Qt::CaseInsensitive)) {
            ignoredClassNames.append(trimmedClassName);
        }
    }

    m_settings.m_yoloIgnoredClassNames = ignoredClassNames;
    applySetting("yoloIgnoredClassNames");
}

void CameraGUI::on_yoloBoxColorButton_clicked()
{
    const QColor color = QColorDialog::getColor(m_settings.m_yoloBoxColor, this, tr("Select bounding box colour"), QColorDialog::ShowAlphaChannel);

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

        if ((m_previewDrawMode == PreviewDrawModeNone) && (event->type() == QEvent::MouseButtonPress))
        {
            const QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::RightButton)
            {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
                const QPoint globalPos = mouseEvent->globalPosition().toPoint();
#else
                const QPoint globalPos = mouseEvent->globalPos();
#endif
                if (showStarDetectionContextMenu(mouseEvent->pos(), globalPos)) {
                    return true;
                }
            }
        }

        if (m_previewDrawMode != PreviewDrawModeNone)
        {
            if (event->type() == QEvent::MouseButtonPress)
            {
                const QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if (mouseEvent->button() == Qt::LeftButton)
                {
                    m_previewDragStartImagePos = mapViewportPointToImage(mouseEvent->pos());
                    if (m_previewDragStartImagePos.x() < 0) {
                        return true;
                    }

                    m_previewDragging = true;

                    if (!m_previewDrawRectItem)
                    {
                        QPen pen(QColor(255, 255, 0));
                        pen.setWidth(2);
                        pen.setStyle(Qt::DashLine);
                        m_previewDrawRectItem = m_imageScene->addRect(QRectF(), pen);
                        m_previewDrawRectItem->setZValue(2.0);
                    }

                    m_previewDrawRectItem->setRect(QRectF(
                        QPointF(m_previewDragStartImagePos),
                        QSizeF(1.0, 1.0)));
                    return true;
                }
                if (mouseEvent->button() == Qt::RightButton)
                {
                    setPreviewDrawMode(PreviewDrawModeNone);
                    return true;
                }
            }
            else if (event->type() == QEvent::MouseMove)
            {
                const QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if (m_previewDragging && m_previewDrawRectItem)
                {
                    const QPoint current = mapViewportPointToImage(mouseEvent->pos());
                    const QRect rect(
                        QPoint(std::min(m_previewDragStartImagePos.x(), current.x()),
                               std::min(m_previewDragStartImagePos.y(), current.y())),
                        QPoint(std::max(m_previewDragStartImagePos.x(), current.x()),
                               std::max(m_previewDragStartImagePos.y(), current.y())));
                    m_previewDrawRectItem->setRect(QRectF(rect));
                    return true;
                }
            }
            else if (event->type() == QEvent::MouseButtonRelease)
            {
                const QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if (m_previewDragging && mouseEvent->button() == Qt::LeftButton)
                {
                    m_previewDragging = false;
                    const QPoint end = mapViewportPointToImage(mouseEvent->pos());
                    const int left = std::min(m_previewDragStartImagePos.x(), end.x());
                    const int top = std::min(m_previewDragStartImagePos.y(), end.y());
                    const int width = std::abs(end.x() - m_previewDragStartImagePos.x()) + 1;
                    const int height = std::abs(end.y() - m_previewDragStartImagePos.y()) + 1;

                    const PreviewDrawMode completedMode = m_previewDrawMode;

                    setPreviewDrawMode(PreviewDrawModeNone);

                    if ((width > 1) && (height > 1))
                    {
                        if (completedMode == PreviewDrawModeMotionExclusion)
                        {
                            m_settings.m_motionExclusionRects.append(QRect(left, top, width, height));
                            updateMotionExclusionRectsTable();
                            settingsUI()->motionExclusionTable->selectRow(settingsUI()->motionExclusionTable->rowCount() - 1);
                            applySetting("motionExclusionRects");
                        }
                        else if (completedMode == PreviewDrawModeDetectionRoi)
                        {
                            m_settings.m_detectionRoiX = left;
                            m_settings.m_detectionRoiY = top;
                            m_settings.m_detectionRoiWidth = width;
                            m_settings.m_detectionRoiHeight = height;

                            settingsUI()->detectionRoiXSpin->blockSignals(true);
                            settingsUI()->detectionRoiYSpin->blockSignals(true);
                            settingsUI()->detectionRoiWidthSpin->blockSignals(true);
                            settingsUI()->detectionRoiHeightSpin->blockSignals(true);
                            settingsUI()->detectionRoiXSpin->setValue(left);
                            settingsUI()->detectionRoiYSpin->setValue(top);
                            settingsUI()->detectionRoiWidthSpin->setValue(width);
                            settingsUI()->detectionRoiHeightSpin->setValue(height);
                            settingsUI()->detectionRoiXSpin->blockSignals(false);
                            settingsUI()->detectionRoiYSpin->blockSignals(false);
                            settingsUI()->detectionRoiWidthSpin->blockSignals(false);
                            settingsUI()->detectionRoiHeightSpin->blockSignals(false);

                            updateMotionExclusionPreview();
                            applySettings({"detectionRoiX", "detectionRoiY", "detectionRoiWidth", "detectionRoiHeight"});
                        }
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
    if (m_imagePixmapItem && !m_imagePixmapItem->image().isNull()) {
        ui->imageView->fitInView(m_imagePixmapItem, Qt::KeepAspectRatio);
    }
}

void CameraGUI::on_audioMute_toggled(bool checked)
{
    m_settings.m_audioMute = checked;
    applySetting("audioMute");
}

void CameraGUI::on_audioPreviewVolumeDial_valueChanged(int value)
{
    m_settings.m_audioPreviewVolume = value;
    ui->audioPreviewVolumeDial->setToolTip(tr("Audio preview volume: %1%").arg(value));
    applySetting("audioPreviewVolume");
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
    updateCameraSettingsVisibility();
    m_settingsDialog->shrinkToVisibleContent();
    m_settingsDialog->show();
    m_settingsDialog->shrinkToVisibleContent();
    QTimer::singleShot(0, m_settingsDialog, [this]() {
        m_settingsDialog->shrinkToVisibleContent();
    });
    m_settingsDialog->raise();
    m_settingsDialog->activateWindow();
}

void CameraGUI::applyImageToolTip()
{
    QStringList outputs;
    if (m_settings.m_recordRawFits) {
        outputs.append(QStringLiteral("raw FITS"));
    }
    if (m_settings.m_recordCalibratedMedia) {
        outputs.append(QStringLiteral("calibrated"));
    }
    if (m_settings.m_recordFilteredMedia) {
        outputs.append(QStringLiteral("filtered"));
    }
    if (m_settings.m_recordPostProcessedMedia) {
        outputs.append(QStringLiteral("post-processed"));
    }
    if (outputs.isEmpty()) {
        outputs.append(QStringLiteral("no"));
    }
    ui->saveImageCheck->setToolTip(QString("Save %1 images to %2")
        .arg(outputs.join(QStringLiteral(", ")), m_settings.m_imageFileName));
}

void CameraGUI::applyVideoToolTip()
{
    QStringList outputs;
    if (m_settings.m_recordCalibratedMedia) {
        outputs.append(QStringLiteral("calibrated"));
    }
    if (m_settings.m_recordFilteredMedia) {
        outputs.append(QStringLiteral("filtered"));
    }
    if (m_settings.m_recordPostProcessedMedia) {
        outputs.append(QStringLiteral("post-processed"));
    }
    if (outputs.isEmpty()) {
        outputs.append(QStringLiteral("no"));
    }
    ui->saveVideoCheck->setToolTip(QString("Record %1 video to %2")
        .arg(outputs.join(QStringLiteral(", ")), m_settings.m_videoFileName));
}

void CameraGUI::onSettingsDialogFinished(int result)
{
    Q_UNUSED(result)
}

void CameraGUI::updateStatus()
{
    if (!m_settings.m_directionSensor.isEmpty()) {
        syncFromDirectionSensors();
    } else if (!m_settings.m_rotator.isEmpty()) {
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
        populateDirectionSourceCombo();
        updatePositionControls();
    }
}

void CameraGUI::onFeatureRemoved(int featureSetIndex, Feature *feature)
{
    (void) featureSetIndex;
    if (feature && (feature->getURI() == QLatin1String("sdrangel.feature.gs232controller"))) {
        populateDirectionSourceCombo();
        if (!m_settings.m_rotator.isEmpty()
            && (settingsUI()->directionSourceCombo->findData(rotatorDirectionSourceId(m_settings.m_rotator)) < 0))
        {
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
