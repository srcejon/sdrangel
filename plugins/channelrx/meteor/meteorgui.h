///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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

#ifndef INCLUDE_METEORGUI_H
#define INCLUDE_METEORGUI_H

#include <QDate>
#include <QDateTime>
#include <QList>
#include <QMap>
#include <QSet>
#include <QVector>
#include <QtCharts>

#include "channel/channelgui.h"
#include "dsp/channelmarker.h"
#include "settings/rollupstate.h"
#include "util/messagequeue.h"

#include "meteor.h"
#include "meteordemodsink.h"
#include "meteorsettings.h"

class BasebandSampleSink;
class ButtonSwitch;
class DeviceUISet;
class GLSpectrum;
class GLSpectrumGUI;
class GLSpectrumView;
class PluginAPI;
class RollupContents;
class SpectrumVis;
class ValueDialZ;

namespace Ui {
    class MeteorGUI;
}

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QWidget;

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

class MeteorGUI : public ChannelGUI {
    Q_OBJECT

public:
    static MeteorGUI* create(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel);
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
    void channelMarkerHighlightedByCursor();

private:
    Ui::MeteorGUI* ui;
    PluginAPI* m_pluginAPI;
    DeviceUISet* m_deviceUISet;
    ChannelMarker m_channelMarker;
    RollupState m_rollupState;
    MeteorSettings m_settings;
    QStringList m_settingsKeys;
    qint64 m_deviceCenterFrequency;
    int m_basebandSampleRate;
    bool m_doApplySettings;
    uint32_t m_tickCount;

    Meteor* m_meteor;
    SpectrumVis* m_spectrumVis;
    MessageQueue m_inputMessageQueue;

    QComboBox *m_frequencyMode;
    ValueDialZ *m_deltaFrequency;
    QLabel *m_deltaUnits;
    QComboBox *m_sampleRate;
    QDoubleSpinBox *m_powerLPFCutoff;
    QDoubleSpinBox *m_detectionThreshold;
    QSpinBox *m_minDuration;
    QSpinBox *m_maxDuration;
    QDoubleSpinBox *m_maxFrequencyDrift;
    ButtonSwitch *m_highlightAllDetections;
    QSpinBox *m_detectionBoxPadding;
    QComboBox *m_detectionLabels;
    QLabel *m_totalCountText;
    QLabel *m_hourCountText;
    QPushButton *m_saveDetections;
    QPushButton *m_saveColorgramme;
    QPushButton *m_clearDetections;
    QTableWidget *m_detectionsTable;
    QTableWidget *m_colorgrammeTable;
    QChartView *m_hourlyChartView;
    QChart *m_hourlyChart;
    GLSpectrum *m_glSpectrum;
    GLSpectrumGUI *m_spectrumGUI;
    QVector<QLabel *> m_detectionOverlayLabels;

    int m_totalCount;
    bool m_highlightAllDetectionOverlays;
    quint64 m_nextDetectionOverlayId;
    bool m_detectionOverlayWindowValid;
    QDateTime m_detectionOverlayWindowStartUtc;
    QDateTime m_detectionOverlayWindowEndUtc;
    QMap<QDate, QVector<int> > m_hourlyCounts;
    QMap<QDate, QVector<bool> > m_hourlyData;
    QSet<QDate> m_dirtyRMOBMonths;
    QDateTime m_lastAutomaticRMOBSaveUtc;

    struct DetectionOverlay
    {
        quint64 m_id;
        QDateTime m_startTimeUtc;
        double m_durationS;
        double m_centerFrequency;
        double m_frequencySpan;
        double m_frequencyDrift;
        double m_peakPowerDB;
    };

    QVector<DetectionOverlay> m_detectionOverlays;

    explicit MeteorGUI(PluginAPI* pluginAPI, DeviceUISet *deviceUISet, BasebandSampleSink *rxChannel, QWidget* parent = nullptr);
    virtual ~MeteorGUI();

    void setupUi(RollupContents *rollupContents);
    void setupSpectrum();
    void blockApplySettings(bool block);
    void applySetting(const QString& settingsKey);
    void applySettings(const QStringList& settingsKeys, bool force = false);
    void applyAllSettings();
    void displaySettings();
    bool handleMessage(const Message& message);
    void makeUIConnections();
    void calcOffset();
    void updateAbsoluteCenterFrequency();
    void updateVisualSampleRate();
    void addDetection(const MeteorDemodSink::MsgMeteorDetected& detection);
    void addCameraDetection(const Meteor::MsgCameraMeteorDetected& detection);
    void updateCounters();
    void updateHistogram();
    void updateColorgramme();
    bool markHourData(const QDate& date, int hour);
    bool hasHourData(const QDate& date, int hour) const;
    bool loadRMOBReport(const QString& fileName);
    bool saveRMOBReport(const QString& fileName, QString *error = nullptr) const;
    bool saveRMOBReport(const QString& fileName, const QDate& monthDate, QString *error = nullptr) const;
    bool saveAutomaticRMOBReports() const;
    void markRMOBDirty(const QDate& date);
    QString automaticRMOBReportFileName(const QDate& date) const;
    QColor colorgrammeColor(int count, int maxCount) const;
    QDate colorgrammeMonthDate() const;
    QList<QDate> colorgrammeMonthDates() const;
    qint64 rmobReportFrequency() const;
    QString rmobReceiverName() const;
    void drawDetectionOverlays(GLSpectrumView *spectrumView);
    void hideDetectionOverlayLabels();
    QString detectionOverlayLabel(const DetectionOverlay& detection) const;
    void applyDetectionsColumnVisibility();
    void saveDetectionsColumnVisibility();
    QSet<quint64> selectedDetectionOverlayIds() const;
    void deleteSelectedDetections();
    int sampleRateIndex(int sampleRate) const;
    QTableWidgetItem *makeTableItem(const QString& text, const QVariant& sortValue = QVariant()) const;

    void leaveEvent(QEvent*);
    void enterEvent(EnterEventType*);

private slots:
    void on_frequencyMode_currentIndexChanged(int index);
    void on_deltaFrequency_changed(qint64 value);
    void on_sampleRate_currentIndexChanged(int index);
    void on_powerLPFCutoff_valueChanged(double value);
    void on_detectionThreshold_valueChanged(double value);
    void on_minDuration_valueChanged(int value);
    void on_maxDuration_valueChanged(int value);
    void on_maxFrequencyDrift_valueChanged(double value);
    void on_highlightAllDetections_toggled(bool checked);
    void on_detectionBoxPadding_valueChanged(int value);
    void on_detectionLabels_currentIndexChanged(int index);
    void on_saveDetections_clicked();
    void on_saveColorgramme_clicked();
    void on_clearDetections_clicked();
    void on_detectionsTable_itemSelectionChanged();
    void on_detectionsTable_customContextMenuRequested(const QPoint& pos);
    void on_detectionsTableHeader_customContextMenuRequested(const QPoint& pos);
    void onWidgetRolled(QWidget* widget, bool rollDown);
    void onMenuDialogCalled(const QPoint& p);
    void handleInputMessages();
    void tick();
};

#endif // INCLUDE_METEORGUI_H
