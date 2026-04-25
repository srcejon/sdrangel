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

#ifndef INCLUDE_FEATURE_CAMERAGUI_H_
#define INCLUDE_FEATURE_CAMERAGUI_H_

#include <QColor>
#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QToolButton>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QVideoFrame>
class QCamera;
class QVideoSink;
class QMediaCaptureSession;
#else
#include <QAbstractVideoSurface>
#include <QAbstractVideoBuffer>
#include <QVideoFrame>
class QCamera;
class CameraGUI;

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

#include "feature/featuregui.h"
#include "util/messagequeue.h"
#include "settings/rollupstate.h"
#include "camerasettings.h"
#include "cameraworker.h"

class PluginAPI;
class FeatureUISet;
class Camera;
class CameraSettingsDialog;
class Message;

namespace Ui {
    class CameraGUI;
}

class CameraGUI : public FeatureGUI {
    Q_OBJECT
public:
    static CameraGUI* create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature);
    virtual void destroy();

    void resetToDefaults();
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    virtual MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    virtual void setWorkspaceIndex(int index);
    virtual int getWorkspaceIndex() const { return m_settings.m_workspaceIndex; }
    virtual void setGeometryBytes(const QByteArray& blob) { m_settings.m_geometryBytes = blob; }
    virtual QByteArray getGeometryBytes() const { return m_settings.m_geometryBytes; }

private:
    Ui::CameraGUI* ui;
    PluginAPI* m_pluginAPI;
    FeatureUISet* m_featureUISet;
    CameraSettings m_settings;
    QList<QString> m_settingsKeys;
    RollupState m_rollupState;
    bool m_doApplySettings;

    Camera* m_camera;
    MessageQueue m_inputMessageQueue;
    QImage m_lastImage;     ///< Last processed image received from the worker (displayed in the GUI)
    CameraSettingsDialog *m_settingsDialog;
    QToolButton *m_cameraSettingsButton;
    bool m_alpacaHasNamedGains;   // true if gains list has named entries
    bool m_alpacaHasNamedOffsets; // true if offsets list has named entries
    bool m_qtZoomSupported;             // true when the active Qt camera reports zoom range > 1.0
    bool m_qtManualExposureSupported;   // true when the active Qt camera supports manual exposure time
    bool m_qtIsoSensitivitySupported;   // true when the active Qt camera supports manual ISO sensitivity
    bool m_qtWhiteBalanceModeSupported; // true when the active Qt camera supports white balance control

    QGraphicsScene *m_imageScene;         ///< Scene used by the QGraphicsView image display
    QGraphicsPixmapItem *m_imagePixmapItem; ///< Pixmap item holding the camera frame

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QCamera *m_qtCamera;
    QVideoSink *m_videoSink;
    QMediaCaptureSession *m_captureSession;
#else
    QCamera *m_qtCamera;
    CameraVideoSurface *m_videoSurface;
#endif

    explicit CameraGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent = nullptr);
    virtual ~CameraGUI();

    bool eventFilter(QObject *watched, QEvent *event) override;

    void blockApplySettings(bool block);
    void applySettings(bool force = false);
    void displaySettings();
    bool handleMessage(const Message& message);
    void makeUIConnections();
    void moveSettingsWidgetsToDialog();
    void updateAlpacaVisibility();
    void updateAlpacaCapabilities(const CameraWorker::MsgReportAlpacaCameraInfo& info);
    void updateImageWidget();
    void updateEnabledControls();
    static void updateColorButton(QToolButton* btn, const QColor& color);
    void setupQtCapture();
    void cleanupQtCapture();
    void applyQtCameraSettings(const QList<QString>& settingsKeys, bool force);

private slots:
    void handleInputMessages();
    void on_startStop_clicked(bool checked);
    void on_refreshCamerasButton_clicked();
    void on_cameraCombo_currentTextChanged(const QString& text);
    void on_resolutionCombo_currentIndexChanged(int index);
    void on_fpsSpin_valueChanged(int value);
    void on_exposureSpin_valueChanged(int value);
    void on_isoSpin_valueChanged(int value);
    void on_alpacaHostEdit_editingFinished();
    void on_alpacaPortSpin_valueChanged(int value);
    void on_alpacaBinXSpin_valueChanged(int value);
    void on_alpacaBinYSpin_valueChanged(int value);
    void on_alpacaGainCombo_currentIndexChanged(int index);
    void on_alpacaGainSpin_valueChanged(int value);
    void on_alpacaOffsetCombo_currentIndexChanged(int index);
    void on_alpacaOffsetSpin_valueChanged(int value);
    void on_alpacaReadoutModeCombo_currentIndexChanged(int index);
    void on_saveImageCheck_toggled(bool checked);
    void on_imagePathEdit_editingFinished();
    void on_imagePathButton_clicked();
    void on_saveVideoCheck_toggled(bool checked);
    void on_videoPathEdit_editingFinished();
    void on_videoPathButton_clicked();
    void on_videoPostProcessButton_toggled(bool checked);
    void on_brightnessSlider_valueChanged(int value);
    void on_contrastSlider_valueChanged(int value);
    void on_invertColorsButton_toggled(bool checked);
    void on_overlayDateTimeButton_toggled(bool checked);
    void on_dateTimeColorButton_clicked();
    void on_dateTimeFormatEdit_editingFinished();
    void on_dateTimePosXSlider_valueChanged(int value);
    void on_dateTimePosYSlider_valueChanged(int value);
    void on_diffMaskButton_toggled(bool checked);
    void on_dilationSpin_valueChanged(int value);
    void on_histogramButton_clicked();
    void on_overlayFontCombo_currentFontChanged(const QFont& font);
    void on_overlayFontScaleSpin_valueChanged(double value);
    void on_motionDetectButton_toggled(bool checked);
    void on_minContourAreaSpin_valueChanged(int value);
    void on_motionBoxColorButton_clicked();
    void on_spectrumOverlayButton_toggled(bool checked);
    void on_spectrumDeviceCombo_currentIndexChanged(int index);
    void on_spectrumOffsetXSlider_valueChanged(int value);
    void on_spectrumOffsetYSlider_valueChanged(int value);
    void on_spectrumScaleSpin_valueChanged(double value);
    void on_yoloButton_toggled(bool checked);
    void on_yoloModelPathEdit_editingFinished();
    void on_yoloModelPathButton_clicked();
    void on_yoloLabelsPathEdit_editingFinished();
    void on_yoloLabelsPathButton_clicked();
    void on_yoloObjectControlButton_clicked();
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

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    void onQtVideoFrame(const QVideoFrame& frame);
#else
    void onQt5VideoFrame(const QImage& image);
#endif
};

#endif // INCLUDE_FEATURE_CAMERAGUI_H_
