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

#ifndef INCLUDE_FEATURE_CAMERAGUI_H_
#define INCLUDE_FEATURE_CAMERAGUI_H_

#include <memory>

#include <QColor>
#include <QHash>
#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QNetworkReply>
#include <QPair>
#include <QSet>
#include <QSize>
#include <QTimer>
#include <QToolButton>
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QVideoFrame>
#else
#include <QAbstractVideoSurface>
#include <QAbstractVideoBuffer>
#include <QVideoFrame>
#endif
#ifdef QT_SENSORS_FOUND
#include <QSensor>
class QCompass;
class QRotationSensor;
class QTiltSensor;
#endif

#include "feature/featuregui.h"
#include "util/messagequeue.h"
#include "settings/rollupstate.h"
#include "camerainfo.h"
#include "cameradetectionhistoryentry.h"
#include "camerasettings.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"
#include "cameraimagepool.h"
#include "gui/httpdownloadmanagergui.h"

class PluginAPI;
class FeatureUISet;
class Camera;
class Feature;
class CameraSettingsDialog;
class CameraDetectionHistory;
class CameraHistogramDialog;
class Message;
class QLabel;
class QDialog;
class QDoubleSpinBox;
class QGraphicsRectItem;
class QMdiSubWindow;
class QProgressDialog;
class QPushButton;
class QTableWidget;
class QTableWidgetItem;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
class QCamera;
class QImageCapture;
class QVideoSink;
class QMediaCaptureSession;
#else
class QCamera;
class QCameraImageCapture;
class CameraGUI;
#endif

namespace Ui {
    class CameraGUI;
    class CameraSettingsDialog;
}

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
/// Qt5 video surface: receives raw frames from QCamera and emits them as QImage signals.
class CameraVideoSurface : public QAbstractVideoSurface
{
    Q_OBJECT
public:
    explicit CameraVideoSurface(QObject *parent = nullptr);

    QList<QVideoFrame::PixelFormat> supportedPixelFormats(
        QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle) const override;

    bool present(const QVideoFrame& frame) override;

signals:
    void frameAvailable(const QImage& image);

private:
    // Recycles the per-frame QImage backing buffer, instead of .copy() allocating
    // one for every presented frame.
    CameraImagePool m_imagePool;
};
#endif

/// Scene item that paints a QImage directly (via QPainter::drawImage) instead of
/// going through a per-frame QPixmap::fromImage conversion. That conversion was
/// the dominant always-on host allocation at 4K (~700 MB/s); painting the image
/// directly lets the (already pooled) decoder buffer upload straight to a GPU
/// texture under the OpenGL viewport, with no per-frame host allocation.
class CameraImageGraphicsItem : public QGraphicsItem
{
public:
    void setImage(const QImage& image);
    const QImage& image() const { return m_image; }
    QRectF boundingRect() const override { return QRectF(m_image.rect()); }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

private:
    QImage m_image;
};

/**
 * \brief Main feature GUI for the camera feature.
 *
 * The top-level FeatureGUI for the camera feature: it presents the live image/video display
 * (via a QGraphicsScene and CameraImageGraphicsItem), drives the settings dialog, and exposes the
 * full set of capture, image-processing, detection (motion / YOLO / star), plate-solving, recording,
 * streaming and overlay controls. It owns the Qt-side camera capture pipeline (QCamera / image
 * capture / video sink or, on Qt5, a CameraVideoSurface), serialises and restores CameraSettings,
 * and communicates with the camera worker/feature through message queues.
 *
 * \note GUI-thread class. Qt camera objects are deliberately created and torn down here on the GUI
 *       thread because cleaning them up on the worker thread can hang. Incoming worker messages are
 *       delivered via m_inputMessageQueue and handled in handleMessage().
 */
class CameraGUI : public FeatureGUI {
    Q_OBJECT

    struct FrameRateOptions {
        bool contiguous;
        int minFps;
        int maxFps;
        QList<int> values;
    };

public:

    static CameraGUI* create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature);
    void destroy() override;

    void resetToDefaults();
    QByteArray serialize() const override;
    bool deserialize(const QByteArray& data) override;
    MessageQueue *getInputMessageQueue() override { return &m_inputMessageQueue; }
    void setWorkspaceIndex(int index) override;
    int getWorkspaceIndex() const override { return m_settings.m_workspaceIndex; }
    void setGeometryBytes(const QByteArray& blob) override { m_settings.m_geometryBytes = blob; }
    QByteArray getGeometryBytes() const override { return m_settings.m_geometryBytes; }

private:
    enum PreviewDrawMode
    {
        PreviewDrawModeNone = 0,
        PreviewDrawModeMotionExclusion,
        PreviewDrawModeDetectionRoi
    };

    enum CameraComboRole
    {
        CameraProtocolRole = Qt::UserRole,
        CameraIdRole,
        CameraDescriptionRole,
        CameraAlpacaHostRole,
        CameraAlpacaPortRole
    };

    enum AccessoryComboRole
    {
        AccessoryDeviceNumberRole = Qt::UserRole + 100,
        AccessoryDescriptionRole,
        AccessoryAlpacaHostRole,
        AccessoryAlpacaPortRole
    };

    enum WindowOverlayComboRole
    {
        WindowOverlayClassRole = Qt::UserRole + 200,
        WindowOverlayTitleRole,
        WindowOverlayRegionObjectNameRole,
        WindowOverlayRegionTitleRole
    };

    Ui::CameraGUI* ui;
    PluginAPI* m_pluginAPI;
    FeatureUISet* m_featureUISet;
    CameraSettings m_settings;
    QList<QString> m_settingsKeys;
    RollupState m_rollupState;
    bool m_doApplySettings;
    bool m_forceSettings;
    bool m_captureActive = false;
    quint64 m_captureEpoch = 0;
    bool m_displayedMotionEventActive = false;
    QSet<QString> m_displayedObjectEventClasses;
    QSet<QString> m_displayedTrackedObjectsInView;
    QTimer m_updateTimer;
    bool m_updatingMotionExclusionRectsTable = false;
    QList<qint64> m_pipelineFrameTimes;
    double m_lastPipelineFps = 0.0;
    QSet<QString> m_reportedFeatureErrorKeys;
    bool m_lastPlateSolved = false;
    int m_lastPlateSolvedMatches = 0;
    int m_lastPlateSolveDetectedStarsConsidered = 0;
    int m_lastPlateSolveCatalogStarsLoaded = 0;
    int m_lastPlateSolveCatalogCandidateStars = 0;
    int m_lastPlateSolveOutlierStars = 0;
    double m_lastPlateSolveRmsError = 0.0;
    double m_lastPlateSolveMaxError = 0.0;
    double m_lastPlateSolveAzimuth = 0.0;
    double m_lastPlateSolveElevation = 0.0;
    double m_lastPlateSolveRoll = 0.0;
    double m_lastPlateSolveFov = 0.0;
    double m_lastPlateSolveCenterOffsetX = 0.0;
    double m_lastPlateSolveCenterOffsetY = 0.0;
    double m_lastPlateSolveDistortionK1 = 0.0;
    QString m_lastPlateSolveCatalogSource;

    Camera* m_camera;
    MessageQueue m_inputMessageQueue;
    QTimer m_statusTimer;
    int m_lastFeatureState;

    HttpDownloadManagerGUI m_dlm;
    QHash<QString, QString> m_pendingYoloDownloads;
    QHash<QString, QString> m_pendingPlateSolveDownloads;
    QStringList m_alpacaFilterWheelNames;
    QList<AlpacaDeviceInfo> m_discoveredAlpacaFocusers;
    QList<AlpacaDeviceInfo> m_discoveredAlpacaFilterWheels;
    int m_lastAlpacaCameraState;
    qint64 m_lastAlpacaCaptureTimeMs;
    QString m_lastAlpacaReceiveImageFormat;
    double m_lastAlpacaCcdTemperature;
    bool m_lastAlpacaCcdTemperatureValid;
    int m_lastAlpacaErrorNumber;
    QString m_lastAlpacaErrorMessage;

    QImage m_lastImage;     ///< Last processed image received from the worker (displayed in the GUI)
    CameraHistogramData m_lastHistogramData; ///< Last histogram computed after image processing but before detection/overlays
    QVector<CameraPipelineStarDetection> m_lastStarDetections;
    QVector<CameraPostProcessor::PreviewTextLabel> m_lastPreviewTextLabels;
    QVector<CameraPostProcessor::PreviewRectItem> m_lastPreviewRectItems;
    QVector<CameraPostProcessor::WindowOverlayFrame> m_lastPreviewImageOverlays;
    QList<CameraDetectionHistoryEntry> m_detectionHistory;
    int m_lastStackCount = 1;
    int m_lastStackQueuedCount = 0;
    int m_lastStackDroppedCount = 0;
    int m_lastStackRejectedCount = 0;
    CameraSettingsDialog *m_settingsDialog;
    CameraDetectionHistory *m_detectionHistoryDialog;
    CameraHistogramDialog *m_histogramDialog;
    QProgressDialog *m_tensorRtProgressDialog = nullptr;
    QWidget *m_windowOverlaysTab = nullptr;
    QTableWidget *m_spectrumOverlaysTable = nullptr;
    QPushButton *m_spectrumOverlayAddButton = nullptr;
    QPushButton *m_spectrumOverlayRemoveButton = nullptr;
    QPushButton *m_spectrumOverlayUpButton = nullptr;
    QPushButton *m_spectrumOverlayDownButton = nullptr;
    QTableWidget *m_windowOverlaysTable = nullptr;
    QPushButton *m_windowOverlayAddButton = nullptr;
    QPushButton *m_windowOverlayRemoveButton = nullptr;
    QPushButton *m_windowOverlayUpButton = nullptr;
    QPushButton *m_windowOverlayDownButton = nullptr;
    QTimer m_windowOverlayCaptureTimer;
    bool m_updatingSpectrumOverlaysTable = false;
    bool m_updatingWindowOverlaysTable = false;
    QStringList m_spectrumOverlayDeviceIds;
    QVector<qint64> m_windowOverlayLastCaptureMs;
    QVector<CameraPostProcessor::WindowOverlayFrame> m_windowOverlayCapturedFrames;
    bool m_alpacaHasNamedGains;   // true if gains list has named entries
    bool m_alpacaHasNamedOffsets; // true if offsets list has named entries
    int m_alpacaCameraSizeX;
    int m_alpacaCameraSizeY;
    bool m_qtZoomSupported;             // true when the active Qt camera reports zoom range > 1.0
    bool m_qtManualExposureSupported;   // true when the active Qt camera supports manual exposure time
    bool m_qtIsoSensitivitySupported;   // true when the active Qt camera supports manual ISO sensitivity
    bool m_qtWhiteBalanceModeSupported; // true when the active Qt camera supports white balance control
    bool m_qtExposureCompensationSupported; // true when the active Qt camera supports exposure compensation
    double m_exposureMinimumMs;
    double m_exposureMaximumMs;
    double m_exposureStepMs;
    int m_qtHdrExposureIndex = 0;
    bool m_asiCoolerSupported;
    bool m_asiTargetTempSupported;
    bool m_asiUsbBandwidthSupported;
    bool m_asiHighSpeedModeSupported;
    bool m_asiColorCameraActive;
    bool m_asiRgb24Supported;
    bool m_asiRaw16Supported;
    bool m_asiRaw8Supported;
    QHash<QString, FrameRateOptions> m_qtFrameRateOptionsByResolution;
    QGraphicsScene *m_imageScene;         ///< Scene used by the QGraphicsView image display
    CameraImageGraphicsItem *m_imagePixmapItem; ///< Scene item holding the camera frame
    QList<QGraphicsItem *> m_previewOverlayItems;
    QList<QGraphicsRectItem *> m_motionExclusionRectItems;
    QGraphicsRectItem *m_detectionRoiRectItem = nullptr;
    QGraphicsRectItem *m_previewDrawRectItem = nullptr;
    PreviewDrawMode m_previewDrawMode = PreviewDrawModeNone;
    bool m_previewDragging = false;
    QPoint m_previewDragStartImagePos;
    QDialog *m_keogramPreviewDialog = nullptr;
    QLabel *m_keogramPreviewLabel = nullptr;

#ifdef QT_SENSORS_FOUND
    QCompass *m_directionCompassSensor = nullptr;
    QRotationSensor *m_directionRotationSensor = nullptr;
    QTiltSensor *m_directionTiltSensor = nullptr;
    bool m_directionCompassReadingValid = false;
    bool m_directionRotationReadingValid = false;
    bool m_directionTiltReadingValid = false;
#endif

    // Qt camera code appears to need to be on GUI thread. Would hang on clean up in the worker thread.
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QCamera *m_qtCamera;
    QImageCapture *m_imageCapture;
    QVideoSink *m_videoSink;
    QMediaCaptureSession *m_captureSession;
    QVideoFrame m_pendingQtVideoFrame;
    bool m_processingQtVideoFrame = false;
    // Recycles the per-frame QImage backing buffers for direct (non-YUV) Qt
    // camera frames, instead of frame.toImage() allocating one each frame.
    CameraImagePool m_qtImagePool;
#else
    QCamera *m_qtCamera;
    QCameraImageCapture *m_imageCapture;
    CameraVideoSurface *m_videoSurface;
#endif
    qint64 m_playbackDurationMs = 0;
    qint64 m_videoFileVideoPositionMs = 0;
    QTimer m_imageSequenceTimer;
    bool m_imageSequenceLoaded = false;
    int m_imageSequenceIndex = 0;
    QTimer m_qtStillCaptureTimer;

    explicit CameraGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent = nullptr);
    ~CameraGUI() override;

    bool eventFilter(QObject *watched, QEvent *event) override;

    void blockApplySettings(bool block);
    void applySetting(const QString& settingsKey);
    void applySettings(const QStringList& settingsKeys, bool force = false);
    void applyAllSettings();
    void displaySettings();
    Ui::CameraSettingsDialog *settingsUI() const;
    bool handleMessage(const Message& message);
    void makeUIConnections();
    void updateCameraSettingsVisibility();
    void updateHistogramStretchControls();
    void updateMotionExclusionRectsTable();
    void applyMotionExclusionRectsFromTable();
    void updateMotionExclusionPreview();
    void updateYoloButtonEnabled();
    void updateYouTubeStreamButtonEnabled();
    void updatePlateSolveStartModeUi();
    void updatePlateSolveDateTimeEdit();
    void setPreviewDrawMode(PreviewDrawMode mode);
    void setMotionExclusionDrawMode(bool enabled);
    void setDetectionRoiDrawMode(bool enabled);
    QPoint mapViewportPointToImage(const QPoint& viewportPos) const;
    int findStarDetectionAtImagePos(const QPointF& imagePos) const;
    bool showStarDetectionContextMenu(const QPoint& viewportPos, const QPoint& globalPos);
    QString starDetectionDisplayName(const CameraPipelineStarDetection& star) const;
    QString starDetectionDetails(const CameraPipelineStarDetection& star) const;
    QString starDetectionSearchTarget(const CameraPipelineStarDetection& star) const;
    void showStarDetectionInfoDialog(const CameraPipelineStarDetection& star);
    void populateDirectionSourceCombo();
    void startDirectionSensors();
    void stopDirectionSensors();
    void syncFromDirectionSensors();
    void applyPositionSync();
    void updatePositionControls();
    void updateFovControls();
    void updateCalculatedFov();
    void updateKeogramPreview(const QImage& image, const QString& fileName, bool visible);
    void initialiseVideoRecordBitrateCombo();
    void updateVideoRecordBitrateCombo();
    void applyVideoRecordBitrateComboText();
    void initialiseYouTubeBitrateCombo();
    void updateYouTubeBitrateCombo();
    void applyYouTubeBitrateComboText();
    void syncFromMainSettings();
    void syncFromSelectedGs232Controller();
    QPair<int, int> selectedGs232ControllerIndices() const;
    void populateAlpacaAccessoryCombos();
    void updateAlpacaCapabilities(const CameraWorker::MsgReportAlpacaCameraInfo& info);
    void updateAsiCapabilities(const CameraWorker::MsgReportAsiCameraInfo& info);
    void updateCameraStatusDisplay();
    void updateCameraSubframeControls();
    void updateImageWidget();
    void updateImageViewSmoothing();
    void sendDisplayedFrameEvents(const QVector<QRect>& motionBoxes, const QVector<CameraPipelineDetection>& detections, const QVector<CameraPipelineMeteorPhotometry>& meteorPhotometry, const QVector<CameraPipelineTrackedObject>& trackedObjects, const QSize& imageSize, const QDateTime& captureDateTime);
    void updatePreviewOverlayItems();
    void clearPreviewOverlayItems();
    void updateCaptureModeControls();
    void updateCaptureIntervalWarning();
    void updateExposureControls();
    void updateHdrExposureControls();
    void updateHdrStackingControls();
    void updateScaleControls();
    void createWindowOverlaysTab();
    void updateSpectrumOverlaysTable();
    void updateSpectrumOverlayControls();
    void applySpectrumOverlaysFromTable();
    void updateWindowOverlaysTable();
    void updateWindowOverlayControls();
    void updateWindowOverlayCaptureTimer();
    void applyWindowOverlaysFromTable();
    void updateWindowOverlayRegionCombo(int row);
    void captureWindowOverlays();
    QList<QMdiSubWindow*> availableWindowOverlayWindows() const;
    QMdiSubWindow* findWindowOverlayWindow(const CameraSettings::WindowOverlay& overlay) const;
    QWidget* findWindowOverlayCaptureWidget(QMdiSubWindow* window, const CameraSettings::WindowOverlay& overlay) const;
    int windowOverlayRowForWidget(const QWidget *widget) const;
    static QString windowOverlayClassName(const QMdiSubWindow *window);
    static QString windowOverlayDisplayName(const QMdiSubWindow *window);
    void updateVideoFileControls();
    void updateVideoPreRecordBufferMemoryLabel();
    void submitQtImageFrame(const QImage& image, qint64 playbackPositionMs = -1, int playbackFrameNumber = -1);
    bool isHdrStackingSupported() const;
    bool isHdrStackingActiveForQt() const;
    void resetQtHdrBracketState();
    double currentQtCaptureExposureTimeMs() const;
    int currentQtHdrExposureIndex() const;
    int currentQtHdrExposureCount() const;
    void advanceQtHdrBracketState();
    void applyQtExposureTimeMs(double exposureTimeMs);
    void populateFrameExposureMetadata(CameraPipelineFrame& frame, double exposureTimeMs, int hdrExposureIndex, int hdrExposureCount, const QDateTime& captureDateTime = QDateTime()) const;
    void handleHdrExposureSliderChanged(int exposureIndex, int sliderValue);
    void handleHdrExposureSpinChanged(int exposureIndex, double value);
    void probeQtCameraCapabilities();
    void reportResolutions();
    void populateQtFormatControls(const QList<QSize>& resolutions, const QHash<QString, FrameRateOptions>& frameRateOptionsByResolution);
    void updateFrameRateControlForResolution(const QString& resolutionText);
    static void updateColorButton(QToolButton* btn, const QColor& color);
    void setupQtCapture();
    void cleanupQtCapture();
    bool ensureVideoFilePlayer(bool startPlayback);
    void sendVideoFileControl(CameraWorker::MsgVideoFileControl::Action action, qint64 positionMs = -1);
    int imageSequenceIntervalMs() const;
    qint64 imageSequenceDurationMs() const;
    void updatePlaybackPositionLabel(qint64 videoPositionMs = -1);
    bool loadImageSequenceFrame(int index, QImage& image) const;
    void showImageSequenceFrame(int index);
    void advanceImageSequenceFrame();
    void updateImageSequencePositionSlider();
    bool prepareImageSequenceManualStep(bool *wasLoaded);
    void reportFeatureError(const QString& errorKey, const QString& title, const QString& errorMessage);
    void applyQtCameraSettings(const QList<QString>& settingsKeys, bool force);
    void applyImageToolTip();
    void applyVideoToolTip();
    void updatePostProcessWhiteBalanceControls();
    void setSelectedCamera(const QString& protocol, const QString& cameraId, const QString& description,
                           const QString& alpacaHost = QString(), quint16 alpacaPort = 0);
    CameraInfo comboCameraInfo(int index) const;
    CameraInfo selectedCameraFromSettings() const;
    static bool sameCameraIdentity(const CameraInfo& lhs, const CameraInfo& rhs);
    static bool isSameHardwareCameraBackend(const CameraInfo& lhs, const CameraInfo& rhs);
    QStringList cameraSelectionSettingsKeys(const CameraInfo& cameraInfo) const;
    bool restorePreviousCameraSelection(const QString& previousCameraProtocol,
                                        const QString& previousCameraId,
                                        const QString& previousAlpacaHost,
                                        quint16 previousAlpacaPort);
    void resetCameraStatus();
    int findCameraComboIndex(const QString& protocol, const QString& cameraId,
                             const QString& alpacaHost = QString(), quint16 alpacaPort = 0) const;
    void applyYoloPathSetting(const QString& settingKey, const QString& path);
    void requestYoloDownload(const QString& settingKey, const QString& path);
    void handleYoloDownloadComplete(const QString& filename, bool success, const QString& url, const QString& errorMessage);
    void requestPlateSolveCatalogDownload();
    void handlePlateSolveCatalogDownloadComplete(const QString& filename, bool success, const QString& url, const QString& errorMessage);
    bool chooseVideoFileCameraFile(int comboIndex, const QString& previousCameraProtocol = QString(),
                                   const QString& previousCameraId = QString(),
                                   const QString& previousAlpacaHost = QString(),
                                   quint16 previousAlpacaPort = 0);
    bool chooseImageFileSequenceFiles(int comboIndex, const QString& previousCameraProtocol = QString(),
                                      const QString& previousCameraId = QString(),
                                      const QString& previousAlpacaHost = QString(),
                                      quint16 previousAlpacaPort = 0);
    bool chooseStreamUrl(int comboIndex, const QString& previousCameraProtocol = QString(),
                         const QString& previousCameraId = QString(),
                         const QString& previousAlpacaHost = QString(),
                         quint16 previousAlpacaPort = 0);
    static CameraGUI::FrameRateOptions makeFrameRateOptions(const QSet<int>& fpsValues);
    static QString resolutionKey(const QSize& size);
    static QString resolutionKey(int width, int height);
    static int decimalsForStepSize(double step);
    static int doubleSpinBoxSliderMaximum(const QDoubleSpinBox *spinBox);
    static int doubleSpinBoxValueToSlider(const QDoubleSpinBox *spinBox, double value);
    static double sliderValueToDoubleSpinBox(const QDoubleSpinBox *spinBox, int sliderValue);
    static int exposureValueToSlider(const QDoubleSpinBox *spinBox, double value);
    static double sliderToExposureValue(const QDoubleSpinBox *spinBox, int sliderValue);
    static double currentExposureUnitScaleMs(const Ui::CameraSettingsDialog *ui);
    static void appendFpsRange(QSet<int>& fpsValues, qreal minFps, qreal maxFps);
    static constexpr int PlaybackPositionSliderMaximum = 1000;

private slots:
    void onMenuDialogCalled(const QPoint &p);
    void handleInputMessages();
    void on_detectionHistoryClearRequested();
    void on_startStop_clicked(bool checked);
    void on_refreshCamerasButton_clicked();
    void on_cameraCombo_currentIndexChanged(int index);
    void on_browseVideoFileButton_clicked();
    void on_restartVideo_clicked();
    void on_stepBackVideo_clicked();
    void on_stepForwardVideo_clicked();
    void on_playPauseVideo_clicked(bool checked);
    void on_loopVideo_clicked(bool checked=false);
    void on_playbackRateSpin_valueChanged(double value);
    void on_playbackAudioOffsetSpin_valueChanged(int value);
    void on_playbackPositionSlider_sliderMoved(int value);
    void on_playbackPositionSlider_sliderReleased();
    void on_resolutionCombo_currentIndexChanged(int index);
    void on_fpsLabel_currentIndexChanged(int index);
    void on_fpsSpin_valueChanged(int value);
    void on_fpsCombo_currentIndexChanged(int index);
    void on_intervalSpin_valueChanged(double value);
    void on_intervalUnitsCombo_currentIndexChanged(int index);
    void on_exposureSlider_valueChanged(int value);
    void on_exposureSpin_valueChanged(double value);
    void on_exposureUnitsCombo_currentIndexChanged(int index);
    void on_isoSpin_valueChanged(int value);
    void on_alpacaDiscoveryCheck_toggled(bool checked);
    void on_alpacaApiLogCheck_toggled(bool checked);
    void on_alpacaHostEdit_editingFinished();
    void on_alpacaPortSpin_valueChanged(int value);
    void on_alpacaFocuserEnabledCheck_toggled(bool checked);
    void on_alpacaFocuserCombo_currentIndexChanged(int index);
    void on_alpacaFocuserHostEdit_editingFinished();
    void on_alpacaFocuserPortSpin_valueChanged(int value);
    void on_alpacaFocusPositionSpin_valueChanged(int value);
    void on_alpacaFocusStepSizeSpin_valueChanged(int value);
    void on_alpacaAutoFocusButton_clicked();
    void on_alpacaFilterWheelEnabledCheck_toggled(bool checked);
    void on_alpacaFilterWheelCombo_currentIndexChanged(int index);
    void on_alpacaFilterWheelHostEdit_editingFinished();
    void on_alpacaFilterWheelPortSpin_valueChanged(int value);
    void on_alpacaFilterWheelPositionCombo_currentIndexChanged(int index);
    void on_cameraBinXSpin_valueChanged(int value);
    void on_cameraBinYSpin_valueChanged(int value);
    void on_cameraNumXSpin_valueChanged(int value);
    void on_cameraNumYSpin_valueChanged(int value);
    void on_cameraStartXSpin_valueChanged(int value);
    void on_cameraStartYSpin_valueChanged(int value);
    void on_cameraGainCombo_currentIndexChanged(int index);
    void on_cameraGainSlider_valueChanged(int value);
    void on_cameraGainSpin_valueChanged(int value);
    void on_cameraOffsetCombo_currentIndexChanged(int index);
    void on_cameraOffsetSlider_valueChanged(int value);
    void on_cameraOffsetSpin_valueChanged(int value);
    void on_alpacaReadoutModeCombo_currentIndexChanged(int index);
    void on_asiCoolerOnCheck_toggled(bool checked);
    void on_asiTargetTempSpin_valueChanged(int value);
    void on_asiUsbBandwidthSpin_valueChanged(int value);
    void on_asiHighSpeedModeCheck_toggled(bool checked);
    void on_autoExposureGainCombo_currentIndexChanged(int index);
    void on_autoExposureGainModeCombo_currentIndexChanged(int index);
    void on_autoExposureTargetSpin_valueChanged(double value);
    void on_autoExposurePercentileSpin_valueChanged(double value);
    void on_autoExposureMinMsSpin_valueChanged(double value);
    void on_autoExposureMaxMsSpin_valueChanged(double value);
    void on_autoExposureMinGainSpin_valueChanged(int value);
    void on_autoExposureMaxGainSpin_valueChanged(int value);
    void on_autoExposureMaxChangeSpin_valueChanged(double value);
    void on_asiColorImageTypeCombo_currentIndexChanged(int index);
    void on_saveImageButton_clicked();
    void on_saveImageCheck_toggled(bool checked);
    void on_imagePathEdit_editingFinished();
    void on_imagePathButton_clicked();
    void on_saveVideoCheck_toggled(bool checked);
    void on_videoPathEdit_editingFinished();
    void on_videoPathButton_clicked();
    void on_keogramButton_toggled(bool checked);
    void on_keogramPathEdit_editingFinished();
    void on_keogramPathButton_clicked();
    void on_keogramDirectionCombo_currentIndexChanged(int index);
    void on_keogramDayModeCombo_currentIndexChanged(int index);
    void on_keogramSamplePeriodSpin_valueChanged(int value);
    void on_keogramPreviewCheck_toggled(bool checked);
    void on_youtubeStreamButton_toggled(bool checked);
    void on_youtubeStreamUrlEdit_editingFinished();
    void on_youtubeStreamKeyEdit_editingFinished();
    void on_youtubeStreamSourceCombo_currentIndexChanged(int index);
    void on_youtubeStreamBitrateCombo_activated(int index);
    void on_youtubeStreamBitrateCombo_editingFinished();
    void on_youtubeStreamFpsSpin_valueChanged(int value);
    void on_youtubeStreamWidthSpin_valueChanged(int value);
    void on_youtubeStreamHeightSpin_valueChanged(int value);
    void on_videoCodecCombo_currentIndexChanged(int index);
    void on_videoRecordBitrateCombo_activated(int index);
    void on_videoRecordBitrateCombo_editingFinished();
    void on_videoHwAccelerationCheck_toggled(bool checked);
    void on_streamBufferingSecondsSpin_valueChanged(double value);
    void on_videoPreRecordBufferSpin_valueChanged(int value);
    void on_imageRecordLimitSpin_valueChanged(int value);
    void on_videoRecordLimitSpin_valueChanged(int value);
    void on_recordRawFitsCheck_toggled(bool checked);
    void on_recordCalibratedMediaCheck_toggled(bool checked);
    void on_recordFilteredMediaCheck_toggled(bool checked);
    void on_recordPostProcessedMediaCheck_toggled(bool checked);
    void on_stackEnabledCheck_toggled(bool checked);
    void on_stackFrameCountSpin_valueChanged(int value);
    void on_stackMethodCombo_currentIndexChanged(int index);
    void on_stackAlignmentCombo_currentIndexChanged(int index);
    void on_stackDisplayModeCombo_currentIndexChanged(int index);
    void on_stackDisplayFrameSpin_valueChanged(int value);
    void on_stackDeleteFrameButton_clicked();
    void on_stackRejectBadFramesCheck_toggled(bool checked);
    void on_scaleEnabledCheck_toggled(bool checked);
    void on_scaleWidthSpin_valueChanged(int value);
    void on_scaleHeightSpin_valueChanged(int value);
    void on_scaleKeepAspectRatioCheck_toggled(bool checked);
    void on_scaleJustificationCombo_currentIndexChanged(int index);
    void on_stackDarkFileEdit_editingFinished();
    void on_stackDarkFileButton_clicked();
    void on_stackFlatFileEdit_editingFinished();
    void on_stackFlatFileButton_clicked();
    void on_stackBiasFileEdit_editingFinished();
    void on_stackBiasFileButton_clicked();
    void on_latitudeSpin_valueChanged(double value);
    void on_longitudeSpin_valueChanged(double value);
    void on_altitudeSpin_valueChanged(double value);
    void on_owmApiKeyEdit_editingFinished();
    void on_useMyPositionButton_clicked();
    void useMyPositionButton_rightClicked(const QPoint& p);
    void on_azimuthSpin_valueChanged(double value);
    void on_elevationSpin_valueChanged(double value);
    void on_rollSpin_valueChanged(double value);
    void on_azimuthOffsetSpin_valueChanged(double value);
    void on_elevationOffsetSpin_valueChanged(double value);
    void on_rollOffsetSpin_valueChanged(double value);
    void on_directionSourceCombo_currentIndexChanged(int index);
    void on_fovModeCombo_currentIndexChanged(int index);
    void on_fovSpin_valueChanged(double value);
    void on_fovSensorWidthSpin_valueChanged(double value);
    void on_fovSensorHeightSpin_valueChanged(double value);
    void on_fovFocalLengthSpin_valueChanged(double value);
    void on_lensProjectionCombo_currentIndexChanged(int index);
    void on_lensCenterOffsetXSpin_valueChanged(double value);
    void on_lensCenterOffsetYSpin_valueChanged(double value);
    void on_lensDistortionK1Spin_valueChanged(double value);
    void on_postProcessWhiteBalanceModeCombo_currentIndexChanged(int index);
    void on_postProcessWhiteBalanceRedGainSlider_valueChanged(int value);
    void on_postProcessWhiteBalanceRedGainSpin_valueChanged(double value);
    void on_postProcessWhiteBalanceGreenGainSlider_valueChanged(int value);
    void on_postProcessWhiteBalanceGreenGainSpin_valueChanged(double value);
    void on_postProcessWhiteBalanceBlueGainSlider_valueChanged(int value);
    void on_postProcessWhiteBalanceBlueGainSpin_valueChanged(double value);
    void on_postProcessWhiteBalanceHighlightProtectionSlider_valueChanged(int value);
    void on_postProcessWhiteBalanceHighlightProtectionSpin_valueChanged(double value);
    void on_postProcessUseCudaCheck_toggled(bool checked);
    void on_postProcessUnwarpCheck_toggled(bool checked);
    void on_histogramStretchModeCombo_currentIndexChanged(int index);
    void on_histogramStretchBlackPointSlider_valueChanged(int value);
    void on_histogramStretchBlackPointSpin_valueChanged(double value);
    void on_histogramStretchWhitePointSlider_valueChanged(int value);
    void on_histogramStretchWhitePointSpin_valueChanged(double value);
    void on_histogramStretchGammaSlider_valueChanged(int value);
    void on_histogramStretchGammaSpin_valueChanged(double value);
    void on_histogramStretchAsinhSlider_valueChanged(int value);
    void on_histogramStretchAsinhSpin_valueChanged(double value);
    void on_histogramStretchLogSlider_valueChanged(int value);
    void on_histogramStretchLogSpin_valueChanged(double value);
    void on_postProcessGreyscaleCheck_toggled(bool checked);
    void on_saturationSlider_valueChanged(int value);
    void on_saturationSpin_valueChanged(double value);
    void on_gammaSlider_valueChanged(int value);
    void on_gammaSpin_valueChanged(double value);
    void on_gaussianBlurSlider_valueChanged(int value);
    void on_gaussianBlurSpin_valueChanged(int value);
    void on_medianBlurSlider_valueChanged(int value);
    void on_medianBlurSpin_valueChanged(int value);
    void on_sharpenSlider_valueChanged(int value);
    void on_sharpenSpin_valueChanged(double value);
    void on_edgeDisplayModeCombo_currentIndexChanged(int index);
    void on_sobelEdgeSlider_valueChanged(int value);
    void on_sobelEdgeSpin_valueChanged(double value);
    void on_cannyEdgeSlider_valueChanged(int value);
    void on_cannyEdgeSpin_valueChanged(double value);
    void on_flipXButton_toggled(bool checked);
    void on_flipYButton_toggled(bool checked);
    void on_imageRotationCombo_currentIndexChanged(int index);
    void on_brightnessSlider_valueChanged(int value);
    void on_brightnessSpin_valueChanged(int value);
    void on_contrastSlider_valueChanged(int value);
    void on_contrastSpin_valueChanged(double value);
    void on_invertColorsButton_toggled(bool checked);
    void on_overlayDateTimeButton_toggled(bool checked);
    void on_dateTimeColorButton_clicked();
    void on_dateTimeFormatEdit_editingFinished();
    void on_dateTimeUtcButton_toggled(bool checked);
    void on_dateTimePosXSlider_valueChanged(int value);
    void on_dateTimePosYSlider_valueChanged(int value);
    void on_equatorialGridCheck_toggled(bool checked);
    void on_equatorialGridColorButton_clicked();
    void on_altAzGridCheck_toggled(bool checked);
    void on_altAzGridColorButton_clicked();
    void on_constellationCheck_toggled(bool checked);
    void on_constellationOverlayCombo_currentIndexChanged(int index);
    void on_constellationColorButton_clicked();
    void on_trackObjectsCheck_toggled(bool checked);
    void on_trackObjectTrailsCheck_toggled(bool checked);
    void on_trackObjectHeatMapCheck_toggled(bool checked);
    void on_trackObjectRangeCheck_toggled(bool checked);
    void on_trackObjectClearHeatMapButton_clicked();
    void on_trackObjectMinElevationSpin_valueChanged(double value);
    void on_trackObjectColorButton_clicked();
    void on_trackObjectFontCombo_currentFontChanged(const QFont& font);
    void on_trackObjectFontScaleSpin_valueChanged(double value);
    void on_gridLabelFontCombo_currentFontChanged(const QFont& font);
    void on_gridLabelFontScaleSpin_valueChanged(double value);
    void on_overlayTextButton_toggled(bool checked);
    void on_overlayTextColorButton_clicked();
    void on_overlayTextEdit_textChanged();
    void on_overlayTextPosXSlider_valueChanged(int value);
    void on_overlayTextPosYSlider_valueChanged(int value);
    void on_diffMaskButton_toggled(bool checked);
    void on_diffThresholdSpin_valueChanged(int value);
    void on_diffMaskOpenSizeSpin_valueChanged(int value);
    void on_dilationSpin_valueChanged(int value);
    void on_diffMaskHistoryFramesSpin_valueChanged(int value);
    void on_diffMaskCloseSizeSpin_valueChanged(int value);
    void on_detectionHistoryButton_clicked();
    void on_detectionHistoryEntryActivated(const CameraDetectionHistoryEntry& entry);
    void on_histogramButton_clicked();
    void on_defaultColorSettingsButton_clicked();
    void on_overlayFontCombo_currentFontChanged(const QFont& font);
    void on_overlayFontScaleSpin_valueChanged(double value);
    void on_overlayTextFontCombo_currentFontChanged(const QFont& font);
    void on_overlayTextFontScaleSpin_valueChanged(double value);
    void on_detectionRoiXSpin_valueChanged(int value);
    void on_detectionRoiYSpin_valueChanged(int value);
    void on_detectionRoiWidthSpin_valueChanged(int value);
    void on_detectionRoiHeightSpin_valueChanged(int value);
    void on_detectionRoiShowButton_toggled(bool checked);
    void on_showStarDetectionBoxesCheck_toggled(bool checked);
    void on_hideSyntheticNamesCheck_toggled(bool checked);
    void on_detectionRoiDrawButton_clicked();
    void on_detectionRoiDeleteButton_clicked();
    void on_detectionResetDefaultsButton_clicked();
    void on_motionDetectButton_toggled(bool checked);
    void on_motionBackgroundSubtractorCombo_currentIndexChanged(int index);
    void on_motionMaskViewCombo_currentIndexChanged(int index);
    void on_motionHistorySpin_valueChanged(int value);
    void on_motionVarThresholdSpin_valueChanged(double value);
    void on_motionLearningRateSpin_valueChanged(double value);
    void on_motionConfirmFramesSpin_valueChanged(int value);
    void on_motionDownscaleCombo_currentIndexChanged(int index);
    void on_motionDetectShadowsCheck_toggled(bool checked);
    void on_motionOpenSizeSpin_valueChanged(int value);
    void on_motionCloseSizeSpin_valueChanged(int value);
    void on_motionPersistenceFramesSpin_valueChanged(int value);
    void on_minContourAreaSpin_valueChanged(int value);
    void on_motionBoxColorButton_clicked();
    void on_starDetectButton_toggled(bool checked);
    void on_starThresholdSpin_valueChanged(int value);
    void on_starBackgroundBlurSpin_valueChanged(int value);
    void on_starMinAreaSpin_valueChanged(int value);
    void on_starMaxAreaSpin_valueChanged(int value);
    void on_starMaxAspectRatioSpin_valueChanged(double value);
    void on_starDebugViewCombo_currentIndexChanged(int index);
    void on_plateSolveLabelModeCombo_currentIndexChanged(int index);
    void on_starColorButton_clicked();
    void on_plateSolveMaxMagnitudeSpin_valueChanged(double value);
    void on_plateSolveMinMatchesSpin_valueChanged(int value);
    void on_plateSolveMatchRadiusSpin_valueChanged(double value);
    void on_plateSolveFinalMatchRadiusSpin_valueChanged(double value);
    void on_plateSolveSearchRadiusSpin_valueChanged(double value);
    void on_plateSolveFovToleranceSpin_valueChanged(double value);
    void on_plateSolveStartModeCombo_currentIndexChanged(int index);
    void on_plateSolveDateTimeModeCombo_currentIndexChanged(int index);
    void on_plateSolveDateTimeEdit_dateTimeChanged(const QDateTime& dateTime);
    void on_plateSolveDateTimeUtcButton_toggled(bool checked);
    void on_plateSolveDateTimeNowButton_clicked();
    void on_plateSolveCatalogSourceCombo_currentIndexChanged(int index);
    void on_starCatalogDiskCacheSizeSpin_valueChanged(int value);
    void on_plateSolveApplyModeCombo_currentIndexChanged(int index);
    void on_plateSolveDownloadCatalogButton_clicked();
    void on_plateSolveApplyButton_clicked();
    void on_motionExclusionAddButton_clicked();
    void on_motionExclusionRemoveButton_clicked();
    void on_motionExclusionTable_itemChanged(QTableWidgetItem *item);
    void on_spectrumOverlayButton_toggled(bool checked);
    void on_windowOverlayButton_toggled(bool checked);
    void on_yoloButton_toggled(bool checked);
    void on_yoloModelPathCombo_currentIndexChanged(int index);
    void on_yoloModelPathEdit_editingFinished();
    void on_yoloModelPathButton_clicked();
    void on_yoloLabelsPathCombo_currentIndexChanged(int index);
    void on_yoloLabelsPathEdit_editingFinished();
    void on_yoloLabelsPathButton_clicked();
    void on_yoloTargetCombo_currentIndexChanged(int index);
    void on_yoloConfSpin_valueChanged(double value);
    void on_yoloNmsSpin_valueChanged(double value);
    void on_yoloTileLargeImagesCheck_toggled(bool checked);
    void on_yoloTileOverlapSpin_valueChanged(int value);
    void on_yoloIgnoredClassNamesEdit_textChanged();
    void on_yoloBoxColorButton_clicked();
    void on_zoomInButton_clicked();
    void on_zoomOutButton_clicked();
    void on_fitInViewButton_clicked();
    void on_fitWindowToImageButton_clicked();
    void on_audioMute_toggled(bool checked);
    void on_audioPreviewVolumeDial_valueChanged(int value);
    void audioSelect(const QPoint& p);
    void on_whiteBalanceCombo_currentIndexChanged(int index);
    void on_exposureCompSpin_valueChanged(double value);
    void on_focusModeCombo_currentIndexChanged(int index);
    void on_focusDistSpin_valueChanged(double value);
    void on_zoomSpin_valueChanged(double value);
    void on_cameraSettingsButton_clicked();
    void updateStatus();
    void updateHardware();
    void preferenceChanged(int elementType);
    void onFeatureAdded(int featureSetIndex, Feature *feature);
    void onFeatureRemoved(int featureSetIndex, Feature *feature);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    void onQtVideoFrame(const QVideoFrame& frame);
    void processPendingQtVideoFrame();
#else
    void onQt5VideoFrame(const QImage& image);
#endif
    void onQtImageCaptured(int id, const QImage& image);
    void triggerQtStillCapture();
    void onSettingsDialogFinished(int result);
};

#endif // INCLUDE_FEATURE_CAMERAGUI_H_
