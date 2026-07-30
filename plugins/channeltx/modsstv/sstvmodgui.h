///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by Copilot / Claude Sonnet                                          //
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

#ifndef PLUGINS_CHANNELTX_MODSSTV_SSTVMODGUI_H_
#define PLUGINS_CHANNELTX_MODSSTV_SSTVMODGUI_H_

#include "channel/channelgui.h"
#include "dsp/channelmarker.h"
#include "util/movingaverage.h"
#include "util/messagequeue.h"
#include "settings/rollupstate.h"
#include "gui/qtcompatibility.h"

#include "sstvmod.h"
#include "sstvmodsettings.h"

class PluginAPI;
class DeviceUISet;
class BasebandSampleSource;
class SpectrumVis;
class ScopeVis;

namespace Ui {
    class SSTVModGUI;
}

class SSTVModGUI : public ChannelGUI
{
    Q_OBJECT

public:
    static SSTVModGUI* create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx);
    virtual void destroy();

    void resetToDefaults() override;
    QByteArray serialize() const override;
    bool deserialize(const QByteArray& data) override;
    MessageQueue *getInputMessageQueue() override { return &m_inputMessageQueue; }
    void setWorkspaceIndex(int index) override { m_settings.m_workspaceIndex = index; }
    int getWorkspaceIndex() const override { return m_settings.m_workspaceIndex; }
    void setGeometryBytes(const QByteArray& blob) override { m_settings.m_geometryBytes = blob; }
    QByteArray getGeometryBytes() const override { return m_settings.m_geometryBytes; }
    QString getTitle() const override { return m_settings.m_title; }
    QColor getTitleColor() const override { return m_settings.m_rgbColor; }
    void zetHidden(bool hidden) override { m_settings.m_hidden = hidden; }
    bool getHidden() const override { return m_settings.m_hidden; }
    ChannelMarker& getChannelMarker() override { return m_channelMarker; }
    int getStreamIndex() const override { return m_settings.m_streamIndex; }
    void setStreamIndex(int streamIndex) override { m_settings.m_streamIndex = streamIndex; }

public slots:
    void channelMarkerChangedByCursor();

private:
    Ui::SSTVModGUI* ui;
    PluginAPI* m_pluginAPI;
    DeviceUISet* m_deviceUISet;
    ChannelMarker m_channelMarker;
    RollupState m_rollupState;
    SSTVModSettings m_settings;
    qint64 m_deviceCenterFrequency = 0;
    bool m_doApplySettings = true;

    SSTVMod* m_sstvMod;
    SpectrumVis* m_spectrumVis;
    ScopeVis* m_scopeVis;
    MovingAverageUtil<double, double, 20> m_channelPowerDbAvg;
    MessageQueue m_inputMessageQueue;

    explicit SSTVModGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSource *channelTx, QWidget* parent = nullptr);
    ~SSTVModGUI() override;

    void blockApplySettings(bool block);
    void applySettings(const QStringList& settingsKeys, bool force = false);
    void displaySettings();
    bool handleMessage(const Message& message);
    void makeUIConnections();
    void updateAbsoluteCenterFrequency();
    void loadImage();

    void leaveEvent(QEvent*) override;
    void enterEvent(EnterEventType*) override;

private slots:
    void handleSourceMessages();
    void on_deltaFrequency_changed(qint64 value);
    void on_rfBW_valueChanged(int value);
    void on_modulation_currentIndexChanged(int index);
    void on_sstvMode_currentIndexChanged(int index);
    void on_fmDeviation_valueChanged(int value);
    void on_loadImage_clicked(bool checked);
    void on_startStop_toggled(bool checked);
    void on_repeat_toggled(bool checked);
    void onWidgetRolled(QWidget* widget, bool rollDown);
    void onMenuDialogCalled(const QPoint& p);
    void tick();
};

#endif // PLUGINS_CHANNELTX_MODSSTV_SSTVMODGUI_H_
