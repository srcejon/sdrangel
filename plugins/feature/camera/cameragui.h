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

#include <QColor>
#include <QHash>
#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QNetworkReply>
#include <QSize>
#include <QToolButton>
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QVideoFrame>
#else
#include <QAbstractVideoSurface>
#include <QAbstractVideoBuffer>
#include <QVideoFrame>
#endif

#include "feature/featuregui.h"
#include "util/messagequeue.h"
#include "settings/rollupstate.h"
#include "camerainfo.h"
#include "cameraobjectdevicesettingsgui.h"
#include "camerasettings.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"
#include "gui/httpdownloadmanagergui.h"

class PluginAPI;
class FeatureUISet;
class Camera;
class CameraSettingsDialog;
class CameraHistogramDialog;
class Message;
class QDoubleSpinBox;
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
};
#endif

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

    Ui::CameraGUI* ui;
    PluginAPI* m_pluginAPI;
    FeatureUISet* m_featureUISet;
    CameraSettings m_settings;
    QList<QString> m_settingsKeys;
    RollupState m_rollupState;
    bool m_doApplySettings;
    bool m_forceSettings;
    QTimer m_updateTimer;

    Camera* m_camera;
    MessageQueue m_inputMessageQueue;
    QTimer m_statusTimer;
    int m_lastFeatureState;

    HttpDownloadManagerGUI m_dlm;
    QHash<QString, QString> m_pendingYoloDownloads;
    QStringList m_alpacaFilterWheelNames;
    QList<AlpacaDeviceInfo> m_discoveredAlpacaFocusers;
    QList<AlpacaDeviceInfo> m_discoveredAlpacaFilterWheels;
    int m_lastAlpacaCameraState;
    qint64 m_lastAlpacaCaptureTimeMs;
    double m_lastAlpacaCcdTemperature;
    bool m_lastAlpacaCcdTemperatureValid;
    int m_lastAlpacaErrorNumber;
    QString m_lastAlpacaErrorMessage;

    QImage m_lastImage;     ///< Last processed image received from the worker (displayed in the GUI)
    CameraSettingsDialog *m_settingsDialog;
    CameraHistogramDialog *m_histogramDialog;
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
    QHash<QString, FrameRateOptions> m_qtFrameRateOptionsByResolution;
    QList<CameraObjectDeviceSettingsGUI *> m_actionDeviceSettingsGUIs;

    QGraphicsScene *m_imageScene;         ///< Scene used by the QGraphicsView image display
    QGraphicsPixmapItem *m_imagePixmapItem; ///< Pixmap item holding the camera frame

    // Qt camera code appears to need to be on GUI thread. Would hang on clean up in the worker thread.
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QCamera *m_qtCamera;
    QImageCapture *m_imageCapture;
    QVideoSink *m_videoSink;
    QMediaCaptureSession *m_captureSession;
#else
    QCamera *m_qtCamera;
    QCameraImageCapture *m_imageCapture;
    CameraVideoSurface *m_videoSurface;
#endif
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
    void updateAlpacaVisibility();
    void populateAlpacaAccessoryCombos();
    void updateAlpacaCapabilities(const CameraWorker::MsgReportAlpacaCameraInfo& info);
    void updateAlpacaStatusDisplay();
    void updateAlpacaSubframeControls();
    void updateImageWidget();
    void updateEnabledControls();
    void updateCaptureModeControls();
    void updateExposureControls();
    void probeQtCameraCapabilities();
    void reportResolutions();
    void populateQtFormatControls(const QList<QSize>& resolutions, const QHash<QString, FrameRateOptions>& frameRateOptionsByResolution);
    void updateFrameRateControlForResolution(const QString& resolutionText);
    static void updateColorButton(QToolButton* btn, const QColor& color);
    void setupQtCapture();
    void cleanupQtCapture();
    void applyQtCameraSettings(const QList<QString>& settingsKeys, bool force);
    void applyImagePath();
    void applyVideoPath();
    QStringList loadActionObjectClasses() const;
    void saveCurrentActionClassSettings();
    void populateActionClasses();
    void rebuildActionTabsForCurrentClass();
    void updateActionControls();
    void applyActionSettings();
    void updatePostProcessWhiteBalanceControls();
    void setSelectedCamera(const QString& protocol, const QString& cameraId, const QString& description,
                           const QString& alpacaHost = QString(), quint16 alpacaPort = 0);
    int findCameraComboIndex(const QString& protocol, const QString& cameraId, const QString& description,
                             const QString& alpacaHost = QString(), quint16 alpacaPort = 0) const;
    void applyYoloPathSetting(const QString& settingKey, const QString& path);
    void requestYoloDownload(const QString& settingKey, const QString& path);
    void handleYoloDownloadComplete(const QString& filename, bool success, const QString& url, const QString& errorMessage);
    static CameraGUI::FrameRateOptions makeFrameRateOptions(const QSet<int>& fpsValues);
    static QString resolutionKey(const QSize& size);
    static QString resolutionKey(int width, int height);
    static int decimalsForStepSize(double step);
    static int doubleSpinBoxSliderMaximum(const QDoubleSpinBox *spinBox);
    static int doubleSpinBoxValueToSlider(const QDoubleSpinBox *spinBox, double value);
    static double sliderValueToDoubleSpinBox(const QDoubleSpinBox *spinBox, int sliderValue);
    static double currentExposureUnitScaleMs(const Ui::CameraSettingsDialog *ui);
    static void appendFpsRange(QSet<int>& fpsValues, qreal minFps, qreal maxFps);

private slots:
    void handleInputMessages();
    void on_startStop_clicked(bool checked);
    void on_refreshCamerasButton_clicked();
    void on_cameraCombo_currentIndexChanged(int index);
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
    void on_alpacaFilterWheelEnabledCheck_toggled(bool checked);
    void on_alpacaFilterWheelCombo_currentIndexChanged(int index);
    void on_alpacaFilterWheelHostEdit_editingFinished();
    void on_alpacaFilterWheelPortSpin_valueChanged(int value);
    void on_alpacaFilterWheelPositionCombo_currentIndexChanged(int index);
    void on_alpacaBinXSpin_valueChanged(int value);
    void on_alpacaBinYSpin_valueChanged(int value);
    void on_alpacaNumXSpin_valueChanged(int value);
    void on_alpacaNumYSpin_valueChanged(int value);
    void on_alpacaStartXSpin_valueChanged(int value);
    void on_alpacaStartYSpin_valueChanged(int value);
    void on_alpacaGainCombo_currentIndexChanged(int index);
    void on_alpacaGainSlider_valueChanged(int value);
    void on_alpacaGainSpin_valueChanged(int value);
    void on_alpacaOffsetCombo_currentIndexChanged(int index);
    void on_alpacaOffsetSlider_valueChanged(int value);
    void on_alpacaOffsetSpin_valueChanged(int value);
    void on_alpacaReadoutModeCombo_currentIndexChanged(int index);
    void on_saveImageCheck_toggled(bool checked);
    void on_imagePathEdit_editingFinished();
    void on_imagePathButton_clicked();
    void on_saveVideoCheck_toggled(bool checked);
    void on_videoPathEdit_editingFinished();
    void on_videoPathButton_clicked();
    void on_videoHwAccelerationCheck_toggled(bool checked);
    void on_videoPostProcessCombo_currentIndexChanged(int index);
    void on_postProcessWhiteBalanceModeCombo_currentIndexChanged(int index);
    void on_postProcessWhiteBalanceRedGainSlider_valueChanged(int value);
    void on_postProcessWhiteBalanceRedGainSpin_valueChanged(double value);
    void on_postProcessWhiteBalanceGreenGainSlider_valueChanged(int value);
    void on_postProcessWhiteBalanceGreenGainSpin_valueChanged(double value);
    void on_postProcessWhiteBalanceBlueGainSlider_valueChanged(int value);
    void on_postProcessWhiteBalanceBlueGainSpin_valueChanged(double value);
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
    void on_sobelEdgeSlider_valueChanged(int value);
    void on_sobelEdgeSpin_valueChanged(double value);
    void on_flipXButton_toggled(bool checked);
    void on_flipYButton_toggled(bool checked);
    void on_brightnessSlider_valueChanged(int value);
    void on_brightnessSpin_valueChanged(int value);
    void on_contrastSlider_valueChanged(int value);
    void on_contrastSpin_valueChanged(double value);
    void on_invertColorsButton_toggled(bool checked);
    void on_overlayDateTimeButton_toggled(bool checked);
    void on_dateTimeColorButton_clicked();
    void on_dateTimeFormatEdit_editingFinished();
    void on_dateTimePosXSlider_valueChanged(int value);
    void on_dateTimePosYSlider_valueChanged(int value);
    void on_overlayTextButton_toggled(bool checked);
    void on_overlayTextColorButton_clicked();
    void on_overlayTextEdit_textChanged();
    void on_overlayTextPosXSlider_valueChanged(int value);
    void on_overlayTextPosYSlider_valueChanged(int value);
    void on_diffMaskButton_toggled(bool checked);
    void on_diffThresholdSpin_valueChanged(int value);
    void on_dilationSpin_valueChanged(int value);
    void on_diffMaskHistoryFramesSpin_valueChanged(int value);
    void on_diffMaskCloseSizeSpin_valueChanged(int value);
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
    void on_motionDetectButton_toggled(bool checked);
    void on_motionHistorySpin_valueChanged(int value);
    void on_motionVarThresholdSpin_valueChanged(double value);
    void on_motionDetectShadowsCheck_toggled(bool checked);
    void on_motionOpenSizeSpin_valueChanged(int value);
    void on_motionCloseSizeSpin_valueChanged(int value);
    void on_motionPersistenceFramesSpin_valueChanged(int value);
    void on_minContourAreaSpin_valueChanged(int value);
    void on_motionBoxColorButton_clicked();
    void on_spectrumOverlayButton_toggled(bool checked);
    void on_spectrumDeviceCombo_currentIndexChanged(int index);
    void on_spectrumOffsetXSlider_valueChanged(int value);
    void on_spectrumOffsetYSlider_valueChanged(int value);
    void on_spectrumScaleSpin_valueChanged(double value);
    void on_yoloButton_toggled(bool checked);
    void on_yoloModelPathCombo_currentIndexChanged(int index);
    void on_yoloModelPathEdit_editingFinished();
    void on_yoloModelPathButton_clicked();
    void on_yoloLabelsPathCombo_currentIndexChanged(int index);
    void on_yoloLabelsPathEdit_editingFinished();
    void on_yoloLabelsPathButton_clicked();
    void on_yoloTargetCombo_currentIndexChanged(int index);
    void on_actionsClassCombo_currentIndexChanged(int index);
    void on_actionsDisappearDebounceSpin_valueChanged(double value);
    void on_actionsAddButton_clicked();
    void on_actionsTabWidget_tabCloseRequested(int index);
    void on_yoloConfSpin_valueChanged(double value);
    void on_yoloNmsSpin_valueChanged(double value);
    void on_yoloBoxColorButton_clicked();
    void on_zoomInButton_clicked();
    void on_zoomOutButton_clicked();
    void on_fitInViewButton_clicked();
    void on_audioMute_toggled(bool checked);
    void audioSelect(const QPoint& p);
    void on_whiteBalanceCombo_currentIndexChanged(int index);
    void on_exposureCompSpin_valueChanged(double value);
    void on_focusModeCombo_currentIndexChanged(int index);
    void on_focusDistSpin_valueChanged(double value);
    void on_zoomSpin_valueChanged(double value);
    void on_cameraSettingsButton_clicked();
    void updateStatus();
    void updateHardware();

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    void onQtVideoFrame(const QVideoFrame& frame);
#else
    void onQt5VideoFrame(const QImage& image);
#endif
    void onQtImageCaptured(int id, const QImage& image);
    void triggerQtStillCapture();
    void onSettingsDialogFinished(int result);
};

#endif // INCLUDE_FEATURE_CAMERAGUI_H_
