///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016 Edouard Griffiths, F4EXB                                   //
// Copyright (C) 2021-2026 Jon Beniston, M7RCE <jon@beniston.com>                //
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

#ifndef INCLUDE_SSTVDEMODGUI_H
#define INCLUDE_SSTVDEMODGUI_H

#include <QImage>
#include <QPixmap>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>

#include "channel/channelgui.h"
#include "dsp/channelmarker.h"
#include "dsp/scopevis.h"
#include "util/messagequeue.h"
#include "settings/rollupstate.h"

#include "sstvdemodsettings.h"

class PluginAPI;
class DeviceUISet;
class BasebandSampleSink;
class SSTVDemod;
class SpectrumVis;

namespace Ui {
    class SSTVDemodGUI;
}

class SSTVDemodGUI : public ChannelGUI {
    Q_OBJECT

public:
    static SSTVDemodGUI* create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel);
    virtual void destroy();

    void resetToDefaults();
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    virtual MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    virtual void setWorkspaceIndex(int index) { m_settings.m_workspaceIndex = index; }
    virtual int getWorkspaceIndex() const { return m_settings.m_workspaceIndex; }
    virtual void setGeometryBytes(const QByteArray& blob) { m_settings.m_geometryBytes = blob; }
    virtual QByteArray getGeometryBytes() const { return m_settings.m_geometryBytes; }
    virtual QString getTitle() const { return m_settings.m_title; }
    virtual QColor getTitleColor() const { return m_settings.m_rgbColor; }
    virtual void zetHidden(bool hidden) { m_settings.m_hidden = hidden; }
    virtual bool getHidden() const { return m_settings.m_hidden; }
    virtual ChannelMarker& getChannelMarker() { return m_channelMarker; }
    virtual int getStreamIndex() const { return m_settings.m_streamIndex; }
    virtual void setStreamIndex(int streamIndex) { m_settings.m_streamIndex = streamIndex; }

public slots:
    void channelMarkerChangedByCursor();
    void channelMarkerHighlightedByCursor();

private:
    Ui::SSTVDemodGUI *ui;
    PluginAPI *m_pluginAPI;
    DeviceUISet *m_deviceUISet;
    ChannelMarker m_channelMarker;
    RollupState m_rollupState;
    SSTVDemodSettings m_settings;
    qint64 m_deviceCenterFrequency;
    bool m_doApplySettings;

    SSTVDemod *m_sstvDemod;
    int m_basebandSampleRate;
    uint32_t m_tickCount;
    MessageQueue m_inputMessageQueue;

    ScopeVis *m_scopeVis;
    SpectrumVis *m_spectrumVis;

    QImage m_image;
    QPixmap m_pixmap;
    QGraphicsScene *m_scene;
    QGraphicsPixmapItem *m_pixmapItem;

    explicit SSTVDemodGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel, QWidget *parent = nullptr);
    virtual ~SSTVDemodGUI();

    void blockApplySettings(bool block);
    void applySettings(const QStringList& settingsKeys, bool force = false);
    void displaySettings();
    bool handleMessage(const Message& message);
    void makeUIConnections();
    void updateAbsoluteCenterFrequency();
    void resetImage();

    void leaveEvent(QEvent*);
    void enterEvent(EnterEventType*);

private slots:
    void on_deltaFrequency_changed(qint64 value);
    void on_rfBW_valueChanged(int index);
    void on_fmDev_valueChanged(int value);
    void on_modulation_currentIndexChanged(int index);
    void on_startStop_clicked(bool checked = false);
    void on_resetDecoder_clicked();
    void on_saveImage_clicked();
    void on_zoomIn_clicked();
    void on_zoomOut_clicked();
    void on_zoomAll_clicked(bool checked = false);
    void onWidgetRolled(QWidget *widget, bool rollDown);
    void onMenuDialogCalled(const QPoint& p);
    void handleInputMessages();
    void tick();
};

#endif // INCLUDE_SSTVDEMODGUI_H
