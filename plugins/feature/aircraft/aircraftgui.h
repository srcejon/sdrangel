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

#ifndef INCLUDE_FEATURE_AIRCRAFTGUI_H_
#define INCLUDE_FEATURE_AIRCRAFTGUI_H_

#include <QTimer>
#include <QMenu>
#include <QDateTime>
#include <QtCharts>

#include "feature/featuregui.h"
#include "util/messagequeue.h"
#include "util/planespotters.h"
#include "settings/rollupstate.h"

#include "aircraftsettings.h"
#include "aircrafttracker.h"
#include "aircrafttablemodels.h"

class PluginAPI;
class FeatureUISet;
class Aircraft;
class QTableWidget;
class QTableWidgetItem;
class QTextToSpeech;
class QSortFilterProxyModel;

namespace Ui {
    class AircraftGUI;
}

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

// View onto the AircraftTracker, which runs on its own thread inside the
// feature and does all the processing; this GUI just displays the snapshots
// the tracker sends it and forwards user actions back
class AircraftGUI : public FeatureGUI {
    Q_OBJECT

public:
    static AircraftGUI* create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature);
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
    Ui::AircraftGUI* ui;
    PluginAPI* m_pluginAPI;
    FeatureUISet* m_featureUISet;
    AircraftSettings m_settings;
    // The statistics records, serialised by the tracker and held ready for serialize(),
    // which is const and cannot go and ask for them
    QByteArray m_statisticsBlob;

    // All three splitters in one blob - see AircraftSettings::m_splitterStates
    QByteArray saveSplitters() const;
    void restoreSplitters(const QByteArray& data);
    QString m_selectedFlightNumber;  // Drives the Past Aircraft tab
    // updateHistoryTables() replaces the rows, and clearing them emits a selection
    // change with nothing selected, which would otherwise stop watching the flight
    // the selection that triggered the rebuild had just chosen
    bool m_updatingHistoryTables = false;
    // Which column Find looks in, in the order the combo lists them
    enum FindColumn { FindAll = 0, FindIcao, FindReg, FindType, FindFlight };
    // Where the last match was, so Enter steps on to the next one rather than finding
    // the same row again
    int m_findRow = -1;
    // Typing searches, but not on every keystroke: each match selects a row, and a
    // selection change reloads the message tables, the profile chart and the aircraft
    // photograph - the last of which can go to the network
    QTimer m_findTimer;
    QList<QString> m_settingsKeys;
    RollupState m_rollupState;
    bool m_doApplySettings;

    Aircraft* m_aircraft;
    MessageQueue m_inputMessageQueue;

    // One model each for aircraft and flights, shown through two views apiece: what we
    // are hearing now, and the archive of everything else
    AircraftTableModel *m_aircraftModel;
    ActiveFilterProxy *m_aircraftProxy;
    ActiveFilterProxy *m_oldAircraftProxy;
    FlightTableModel *m_flightModel;
    ActiveFilterProxy *m_flightProxy;
    ActiveFilterProxy *m_oldFlightProxy;

    QMenu *m_columnMenu;
    QMenu *m_flightColumnMenu;

    // Selection driving the Documents/ATC filter, the photo and the profile
    quint64 m_selectedAircraftId = 0;
    QString m_selectedRegistration;
    quint32 m_selectedIcao = 0;
    quint64 m_selectedFlightId = 0;

    // Message rate chart, driven by the tracker's rate samples
    static const int CHART_SERIES = AircraftReport::ProtocolCount + 1;
    QChart *m_chart = nullptr;
    QDateTimeAxis *m_chartXAxis = nullptr;
    QValueAxis *m_chartYAxis = nullptr;
    QLineSeries *m_chartSeries[CHART_SERIES];

    // Flight profile chart, from the tracker's profile snapshots
    QChart *m_profileChart = nullptr;
    quint64 m_profileFlightId = 0;
    QString m_profileTitle;
    QList<qint64> m_profileTimes;
    QList<float> m_profileAltFt;
    QList<float> m_profileSpeedKts;

    QTextToSpeech *m_speech = nullptr;

    // Aircraft photos from planespotters.net for the selected aircraft
    PlaneSpotters m_planeSpotters;
    QString m_photoLink;
    QString m_photoKey;                 // Registration/ICAO the current request is for

    explicit AircraftGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent = nullptr);
    virtual ~AircraftGUI();

    void blockApplySettings(bool block);
    void applySettings(bool force = false);
    void displaySettings();
    bool handleMessage(const Message& message);
    void makeUIConnections();

    AircraftTracker *tracker() const;
    void setDefaultColumnWidths();
    void showRowNumbers(QTableView *table);
    void trackerStatistics(const AircraftStatistics& statistics);
    bool messageRowVisible(const QTableWidgetItem *firstItem) const;
    void applyMessageFilter();
    void tableContextMenu(QTableWidget *table, QPoint pos);
    void viewContextMenu(QTableView *view, bool aircraftView, QPoint pos);
    void showContextMenu(QWidget *parent, const QString& cellText, const QString& mapName,
                         QPoint globalPos, const QString& showFlight = QString(),
                         quint64 showAircraftId = 0);
    void selectFlight(const QString& flight);
    void selectAircraft(quint64 aircraftId);
    void plotChart();
    void plotProfileChart();
    void watchFlight(quint64 flightId, const QString& title);
    void updatePhoto(const QString& registration, quint32 icao);
    void updateAircraftInfo(const QString& registration, quint32 icao);
    bool showingArchive() const;
    void updatePhotoText(const QString& registration, quint32 icao);
    QAction *createCheckableItem(QString& text, int idx, bool checked, const char *slot);

private slots:
    void onMenuDialogCalled(const QPoint &p);
    void onWidgetRolled(QWidget* widget, bool rollDown);
    void handleInputMessages();
    void on_deleteAircraft_clicked();
    void on_settings_clicked();
    void on_notifications_clicked();
    void on_atcLabels_clicked(bool checked);
    void on_stats_clicked(bool checked);
    void statsContextMenu(const QPoint& pos);
    void updateHistoryTables();
    void trackerWeatherReports(const QList<WeatherEvent>& reports);
    void weatherSelectionChanged();
    void on_find_returnPressed();
    void on_find_textChanged(const QString& text);
    bool findFrom(int startRow, const QString& text);
    void pastFlightSelectionChanged();
    void chartContextMenu(const QPoint& pos);
    void on_displayChart_clicked(bool checked);
    void documentSelectionChanged();
    void aircraftSelectionChanged();
    void flightSelectionChanged();
    void aircraft_sectionMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex);
    void flights_sectionMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex);
    void flights_sectionResized(int logicalIndex, int oldSize, int newSize);
    void flightColumnSelectMenu(QPoint pos);
    void flightColumnSelectMenuChecked(bool checked = false);
    void aircraft_sectionResized(int logicalIndex, int oldSize, int newSize);
    void aircraftColumnSelectMenu(QPoint pos);
    void aircraftColumnSelectMenuChecked(bool checked = false);
    void legendMarkerClicked();
    void aircraftPhoto(const PlaneSpottersPhoto *photo);
    void photoClicked();

    // From the tracker
    void trackerAircraftUpdated(const QList<AircraftDisplay>& aircraft);
    void trackerAircraftRemoved(const QList<quint64>& ids);
    void trackerFlightsUpdated(const QList<FlightDisplay>& flights);
    void trackerFlightsRemoved(const QList<quint64>& ids);
    void trackerDocumentsAdded(const QList<DocumentEvent>& documents);
    void trackerAtcMessages(const QList<AtcEvent>& messages);
    void trackerProfileUpdated(quint64 flightId, const QList<qint64>& times, const QList<float>& altitudeFt, const QList<float>& speedKts);
    void trackerMessageRates(const QList<float>& rates);
    void trackerSpeechNotification(const QString& speech);
    void trackerAllCleared();
    void trackerDatabaseFilenameReverted(const QString& filename);

    // Column enums for the Documents and ATC tables (still QTableWidgets)
private:
    enum DocumentCol {
        DOCUMENT_COL_TIME,
        DOCUMENT_COL_FLIGHT,
        DOCUMENT_COL_REG,
        DOCUMENT_COL_TYPE,
        DOCUMENT_COL_TITLE
    };

    enum WeatherCol {
        WEATHER_COL_TIME,
        WEATHER_COL_AIRPORT,
        WEATHER_COL_TYPE,
        WEATHER_COL_VIA,
        WEATHER_COL_REPORT
    };

    enum AtcCol {
        ATC_COL_TIME,
        ATC_COL_PROTOCOL,
        ATC_COL_DIR,
        ATC_COL_FROM,
        ATC_COL_TO,
        ATC_COL_MESSAGE
    };
};

#endif // INCLUDE_FEATURE_AIRCRAFTGUI_H_
