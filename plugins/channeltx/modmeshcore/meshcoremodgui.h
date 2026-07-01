///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Alejandro Aleman                                           //
// Copyright (C) 2016-2026 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2021-2022 Jon Beniston, M7RCE <jon@beniston.com>                //
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

#ifndef PLUGINS_CHANNELTX_MODMESHCORE_MESHCOREMODGUI_H_
#define PLUGINS_CHANNELTX_MODMESHCORE_MESHCOREMODGUI_H_

#include "channel/channelgui.h"
#include "dsp/channelmarker.h"
#include "util/movingaverage.h"
#include "util/messagequeue.h"
#include "settings/rollupstate.h"

#include "meshcoremod.h"
#include "meshcoremodsettings.h"

class PluginAPI;
class DeviceUISet;
class BasebandSampleSource;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QWidget;

namespace Ui {
    class MeshcoreModGUI;
}

class MeshcoreModGUI : public ChannelGUI {
    Q_OBJECT

public:
    static MeshcoreModGUI* create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx);
    virtual void destroy();

    void resetToDefaults();
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    virtual MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    virtual void setWorkspaceIndex(int index) { m_settings.m_workspaceIndex = index; };
    virtual int getWorkspaceIndex() const { return m_settings.m_workspaceIndex; };
    virtual void setGeometryBytes(const QByteArray& blob) { m_settings.m_geometryBytes = blob; };
    virtual QByteArray getGeometryBytes() const { return m_settings.m_geometryBytes; };
    virtual QString getTitle() const { return m_settings.m_title; };
    virtual QColor getTitleColor() const  { return m_settings.m_rgbColor; };
    virtual void zetHidden(bool hidden) { m_settings.m_hidden = hidden; }
    virtual bool getHidden() const { return m_settings.m_hidden; }
    virtual ChannelMarker& getChannelMarker() { return m_channelMarker; }
    virtual int getStreamIndex() const { return m_settings.m_streamIndex; }
    virtual void setStreamIndex(int streamIndex) { m_settings.m_streamIndex = streamIndex; }

public slots:
    void channelMarkerChangedByCursor();

private:
    Ui::MeshcoreModGUI* ui;
    PluginAPI* m_pluginAPI;
    DeviceUISet* m_deviceUISet;
    ChannelMarker m_channelMarker;
    RollupState m_rollupState;
    MeshcoreModSettings m_settings;
    qint64 m_deviceCenterFrequency;
    int m_basebandSampleRate;
    bool m_doApplySettings;
    bool m_meshControlsUpdating;

    // MeshCore Identity / Message-Type panel widgets (built programmatically
    // in setupMeshcoreIdentityControls — avoids a heavy UI XML edit).
    QWidget*     m_meshIdPanel;
    QLabel*      m_meshIdPubLabel;
    QLineEdit*   m_meshIdNodeNameEdit;
    QPushButton* m_meshIdGenerateButton;
    QPushButton* m_meshIdCopyPubkeyButton;
    QComboBox*   m_meshIdMessageTypeCombo;
    QLineEdit*   m_meshIdDestPubEdit;
    QLineEdit*   m_meshIdChannelEdit;
    QPushButton* m_meshIdSendNowButton;

    MeshcoreMod* m_meshcoreMod;
    MovingAverageUtil<double, double, 20> m_channelPowerDbAvg;

    std::size_t m_tickCount;
    MessageQueue m_inputMessageQueue;

    explicit MeshcoreModGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx, QWidget* parent = nullptr);
    virtual ~MeshcoreModGUI();

    void blockApplySettings(bool block);
    void applySettings(bool force = false);
    void displaySettings();
    void displayCurrentPayloadMessage();
    void displayBinaryMessage();
    void setBandwidths();
    QString getActivePayloadText() const;
    int findBandwidthIndex(int bandwidthHz) const;
    bool retuneDeviceToFrequency(qint64 centerFrequencyHz);
    void setupMeshcoreAutoProfileControls();
    void rebuildMeshcoreChannelOptions();
    void applyMeshcoreProfileFromSelection();
    void setupMeshcoreIdentityControls();
    void displayMeshcoreIdentity();
    void updateMeshcoreMessageTypeFields();
    bool handleMessage(const Message& message);
    void makeUIConnections();
    void updateAbsoluteCenterFrequency();

    void leaveEvent(QEvent*);
    void enterEvent(EnterEventType*);

private slots:
    void handleSourceMessages();
    void on_deltaFrequency_changed(qint64 value);
    void on_bw_valueChanged(int value);
	void on_spread_valueChanged(int value);
    void on_deBits_valueChanged(int value);
    void on_preambleChirps_valueChanged(int value);
    void on_idleTime_valueChanged(int value);
    void on_syncWord_editingFinished();
    void on_channelMute_toggled(bool checked);
    void on_fecParity_valueChanged(int value);
    void on_playMessage_clicked(bool checked);
    void on_repeatMessage_valueChanged(int value);
    void on_messageText_editingFinished();
    void on_hexText_editingFinished();
    void on_udpEnabled_clicked(bool checked);
    void on_udpAddress_editingFinished();
    void on_udpPort_editingFinished();
    void on_invertRamps_stateChanged(int state);
    void on_meshRegion_currentIndexChanged(int index);
    void on_meshPreset_currentIndexChanged(int index);
    void on_meshChannel_currentIndexChanged(int index);
    void on_meshApply_clicked(bool checked);
    void onMeshIdMessageTypeChanged(int index);
    void onMeshIdGenerateClicked();
    void onMeshIdCopyPubkeyClicked();
    void onMeshIdNodeNameEdited();
    void onMeshIdDestPubEdited();
    void onMeshIdChannelEdited();
    void onMeshIdSendNowClicked();
    void onWidgetRolled(QWidget* widget, bool rollDown);
    void onMenuDialogCalled(const QPoint& p);
    void tick();
};

#endif /* PLUGINS_CHANNELTX_MODMESHCORE_MESHCOREMODGUI_H_ */
