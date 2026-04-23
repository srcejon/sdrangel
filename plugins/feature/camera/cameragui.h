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

#include "feature/featuregui.h"
#include "util/messagequeue.h"
#include "settings/rollupstate.h"
#include "camerasettings.h"
#include "cameraworker.h"

class PluginAPI;
class FeatureUISet;
class Camera;
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
    bool m_alpacaHasNamedGains;   // true if gains list has named entries
    bool m_alpacaHasNamedOffsets; // true if offsets list has named entries

    explicit CameraGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent = nullptr);
    virtual ~CameraGUI();

    void blockApplySettings(bool block);
    void applySettings(bool force = false);
    void displaySettings();
    bool handleMessage(const Message& message);
    void makeUIConnections();
    void updateAlpacaVisibility();
    void updateAlpacaCapabilities(const CameraWorker::MsgReportAlpacaCameraInfo& info);
    void updateImageWidget();
    static void updateColorButton(QToolButton* btn, const QColor& color);

private slots:
    void handleInputMessages();
    void on_startStop_clicked(bool checked);
    void on_refreshCamerasButton_clicked();
    void on_apiCombo_currentIndexChanged(int index);
    void on_cameraCombo_currentTextChanged(const QString& text);
    void on_resolutionCombo_currentIndexChanged(int index);
    void on_fpsSpin_valueChanged(int value);
    void on_exposureSpin_valueChanged(int value);
    void on_isoSpin_valueChanged(int value);
    void on_alpacaHostEdit_editingFinished();
    void on_alpacaPortSpin_valueChanged(int value);
    void on_alpacaCameraIdSpin_valueChanged(int value);
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
    void on_diffMaskButton_toggled(bool checked);
    void on_dilationSpin_valueChanged(int value);
    void on_histogramButton_clicked();
    void on_overlayFontCombo_currentIndexChanged(int index);
    void on_overlayFontScaleSpin_valueChanged(double value);
    void on_motionDetectButton_toggled(bool checked);
    void on_minContourAreaSpin_valueChanged(int value);
    void on_motionBoxColorButton_clicked();
};

#endif // INCLUDE_FEATURE_CAMERAGUI_H_
