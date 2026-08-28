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
#include <cmath>

#include <QAction>
#include <QClipboard>
#include <QDebug>
#include <QDesktopServices>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QHeaderView>
#include <QSortFilterProxyModel>
#include <QTableWidgetItem>
#include <QUrl>
#ifdef QT_TEXTTOSPEECH_FOUND
#include <QTextToSpeech>
#endif

#include "feature/featureuiset.h"
#include "feature/featurewebapiutils.h"
#include "gui/basicfeaturesettingsdialog.h"
#include "gui/dialogpositioner.h"
#include "maincore.h"
#include "util/osndb.h"

#include "ui_aircraftgui.h"
#include "aircraft.h"
#include "aircraftgui.h"
#include "aircraftnotificationdialog.h"
#include "aircraftsettingsdialog.h"

AircraftGUI* AircraftGUI::create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature)
{
    AircraftGUI* gui = new AircraftGUI(pluginAPI, featureUISet, feature);
    return gui;
}

void AircraftGUI::destroy()
{
    delete this;
}

void AircraftGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applySettings(true);
}

// The three splitters travel together in one blob, each holding its own
// QSplitter::saveState(). A version leads it so that adding a fourth later can be read
// by an older build without it making nonsense of the ones it does understand.
QByteArray AircraftGUI::saveSplitters() const
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    stream.setVersion(QDataStream::Qt_5_15);

    stream << (quint32) 1;
    stream << ui->splitter->saveState()
           << ui->aircraftSplitter->saveState()
           << ui->weatherSplitter->saveState();
    return data;
}

void AircraftGUI::restoreSplitters(const QByteArray& data)
{
    if (data.isEmpty()) {
        return;
    }
    QDataStream stream(data);
    stream.setVersion(QDataStream::Qt_5_15);

    quint32 version = 0;
    stream >> version;
    if ((version != 1) || (stream.status() != QDataStream::Ok)) {
        return;             // Written by something else, or truncated: leave the defaults
    }

    QByteArray outer, aircraft, weather;
    stream >> outer >> aircraft >> weather;
    if (stream.status() != QDataStream::Ok) {
        return;
    }
    ui->splitter->restoreState(outer);
    ui->aircraftSplitter->restoreState(aircraft);
    ui->weatherSplitter->restoreState(weather);
}

QByteArray AircraftGUI::serialize() const
{
    // The records live in the tracker, which is where they are set. serialize() is const
    // and is the actual persistence path for a feature, so take a copy of the settings
    // and drop the current blob into that.
    AircraftSettings settings = m_settings;
    settings.m_statistics = m_statisticsBlob;
    settings.m_splitterStates = saveSplitters();
    return settings.serialize();
}

bool AircraftGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        // Hold on to the records that were just read: serialize() writes this back, and
        // until the tracker sends its first update there is nothing else to write
        m_statisticsBlob = m_settings.m_statistics;
        m_feature->setWorkspaceIndex(m_settings.m_workspaceIndex);
        displaySettings();
        applySettings(true);
        return true;
    }
    else
    {
        resetToDefaults();
        return false;
    }
}

void AircraftGUI::setWorkspaceIndex(int index)
{
    m_settings.m_workspaceIndex = index;
    m_feature->setWorkspaceIndex(index);
    m_settingsKeys.append("workspaceIndex");
    applySettings();
}

bool AircraftGUI::handleMessage(const Message& message)
{
    if (Aircraft::MsgConfigureAircraft::match(message))
    {
        const Aircraft::MsgConfigureAircraft& cfg = (Aircraft::MsgConfigureAircraft&) message;

        if (cfg.getForce()) {
            m_settings = cfg.getSettings();
        } else {
            m_settings.applySettings(cfg.getSettingsKeys(), cfg.getSettings());
        }

        blockApplySettings(true);
        displaySettings();
        blockApplySettings(false);

        return true;
    }

    return false;
}

void AircraftGUI::handleInputMessages()
{
    Message* message;

    while ((message = getInputMessageQueue()->pop()))
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void AircraftGUI::onWidgetRolled(QWidget* widget, bool rollDown)
{
    (void) widget;
    (void) rollDown;

    RollupContents *rollupContents = getRollupContents();

    rollupContents->saveState(m_rollupState);
    applySettings();
}

AircraftTracker *AircraftGUI::tracker() const
{
    return m_aircraft->getTracker();
}

AircraftGUI::AircraftGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent) :
    FeatureGUI(parent),
    ui(new Ui::AircraftGUI),
    m_pluginAPI(pluginAPI),
    m_featureUISet(featureUISet),
    m_doApplySettings(true)
{
    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/aircraft/readme.md";
    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    rollupContents->arrangeRollups();
    connect(rollupContents, SIGNAL(widgetRolled(QWidget*,bool)), this, SLOT(onWidgetRolled(QWidget*,bool)));

    m_aircraft = reinterpret_cast<Aircraft*>(feature);
    m_aircraft->setMessageQueueToGUI(&m_inputMessageQueue);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    AircraftInformation::init();

    // Aircraft and flights tables: model/view, with the data coming as
    // snapshots from the tracker thread
    m_aircraftModel = new AircraftTableModel(this);
    m_aircraftProxy = new ActiveFilterProxy(true, this);
    m_oldAircraftProxy = new ActiveFilterProxy(false, this);
    m_flightModel = new FlightTableModel(this);
    m_flightProxy = new ActiveFilterProxy(true, this);
    m_oldFlightProxy = new ActiveFilterProxy(false, this);

    for (auto pair : { qMakePair(ui->aircraft, m_aircraftProxy), qMakePair(ui->oldAircraft, m_oldAircraftProxy) })
    {
        pair.second->setSourceModel(m_aircraftModel);
        pair.second->setSortRole(Qt::DisplayRole);
        pair.first->setModel(pair.second);
        pair.first->setSortingEnabled(true);
        pair.first->horizontalHeader()->setSectionsMovable(true);
        pair.first->setIconSize(QSize(85, 20));
        showRowNumbers(pair.first);
        connect(pair.first->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &AircraftGUI::aircraftSelectionChanged);
    }

    for (auto pair : { qMakePair(ui->flights, m_flightProxy), qMakePair(ui->oldFlights, m_oldFlightProxy) })
    {
        pair.second->setSourceModel(m_flightModel);
        pair.first->setModel(pair.second);
        pair.first->setSortingEnabled(true);
        pair.first->horizontalHeader()->setSectionsMovable(true);
        showRowNumbers(pair.first);
        connect(pair.first->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &AircraftGUI::flightSelectionChanged);
    }

    setDefaultColumnWidths();

    // Documents and ATC log tables
    ui->documents->setSortingEnabled(true);
    ui->documents->horizontalHeader()->setSectionsMovable(true);
    ui->documents->horizontalHeader()->setStretchLastSection(true);
    connect(ui->documents, &QTableWidget::itemSelectionChanged, this, &AircraftGUI::documentSelectionChanged);
    connect(ui->weather, &QTableWidget::itemSelectionChanged, this, &AircraftGUI::weatherSelectionChanged);
    ui->weather->setSortingEnabled(true);
    ui->weather->horizontalHeader()->setSectionsMovable(true);
    ui->weather->horizontalHeader()->setStretchLastSection(true);
    ui->weather->verticalHeader()->setVisible(false);
    connect(ui->pastFlights, &QTableWidget::itemSelectionChanged, this, &AircraftGUI::pastFlightSelectionChanged);
    connect(ui->find, &QLineEdit::returnPressed, this, &AircraftGUI::on_find_returnPressed);
    m_findTimer.setSingleShot(true);
    connect(&m_findTimer, &QTimer::timeout, this, [this]() {
        ui->find->setStyleSheet(findFrom(-1, ui->find->text()) ? "" : "QLineEdit { color: red; }");
    });

    // A row found while the keyboard stays in the Find box is a selection in a view
    // that does not have focus, which most styles draw in a washed out grey. Make the
    // unfocused highlight the same as the focused one so the match is actually visible.
    for (QTableView *view : {ui->aircraft, ui->oldAircraft, ui->flights, ui->oldFlights})
    {
        QPalette palette = view->palette();
        palette.setColor(QPalette::Inactive, QPalette::Highlight,
                         palette.color(QPalette::Active, QPalette::Highlight));
        palette.setColor(QPalette::Inactive, QPalette::HighlightedText,
                         palette.color(QPalette::Active, QPalette::HighlightedText));
        view->setPalette(palette);
    }
    connect(ui->find, &QLineEdit::textChanged, this, &AircraftGUI::on_find_textChanged);
    // Changing which column to look in searches again from the top
    connect(ui->findColumn, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) { on_find_textChanged(ui->find->text()); });
    ui->atcMessages->setSortingEnabled(true);
    ui->atcMessages->horizontalHeader()->setSectionsMovable(true);
    ui->atcMessages->horizontalHeader()->setStretchLastSection(true);

    // Selecting an aircraft or a flight filters the Documents and ATC tables
    connect(ui->aircraftTabs, &QTabWidget::currentChanged, this, [this](int) {
        QWidget *tab = ui->aircraftTabs->currentWidget();

        applyMessageFilter();

        // Only a tab that IS one of the selection tables may re-derive the selection
        // from it. Weather is not one, and falling through to the aircraft table there
        // used to silently discard a flight selection - which empties Documents and
        // ATC, since with nothing selected they show nothing.
        if ((tab == ui->flightsTab) || (tab == ui->oldFlightsTab)) {
            flightSelectionChanged();
        } else if ((tab == ui->aircraftTab) || (tab == ui->oldAircraftTab)) {
            aircraftSelectionChanged();
        }
    });

    // Context menus: copy cell contents or find the aircraft on the Map
    for (auto pair : { qMakePair((QTableView *) ui->aircraft, true), qMakePair((QTableView *) ui->oldAircraft, true),
                       qMakePair((QTableView *) ui->flights, false), qMakePair((QTableView *) ui->oldFlights, false) })
    {
        QTableView *view = pair.first;
        bool isAircraft = pair.second;
        view->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(view, &QTableView::customContextMenuRequested, this,
            [this, view, isAircraft](QPoint pos) { viewContextMenu(view, isAircraft, pos); });
    }
    for (QTableWidget *table : {ui->documents, ui->atcMessages})
    {
        table->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(table, &QTableWidget::customContextMenuRequested, this,
            [this, table](QPoint pos) { tableContextMenu(table, pos); });
    }

    // Aircraft photos from planespotters.net, shown for the selected aircraft
    connect(&m_planeSpotters, &PlaneSpotters::aircraftPhoto, this, &AircraftGUI::aircraftPhoto);
    connect(ui->photo, &ClickableLabel::clicked, this, &AircraftGUI::photoClicked);
    ui->photoHeader->setVisible(false);
    ui->photoFlag->setVisible(false);
    ui->photo->setVisible(false);
    ui->flightDetails->setVisible(false);

    // Charts
    plotChart();
    plotProfileChart();

    // Reuse the ADS-B demodulator's control tower icon (its resources are
    // process wide); the "ATC" text shows if it isn't available
    QIcon towerIcon(":/icons/controltower.png");
    if (!towerIcon.isNull()) {
        ui->atcLabels->setIcon(towerIcon);
    }

    // Column select menu
    m_columnMenu = new QMenu(ui->aircraft);
    for (int i = 0; i < AIRCRAFT_COLUMNS; i++)
    {
        QString text = m_aircraftModel->headerData(i, Qt::Horizontal).toString();
        m_columnMenu->addAction(createCheckableItem(text, i, true, SLOT(aircraftColumnSelectMenuChecked())));
    }
    ui->aircraft->horizontalHeader()->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->aircraft->horizontalHeader(), SIGNAL(customContextMenuRequested(QPoint)), SLOT(aircraftColumnSelectMenu(QPoint)));
    connect(ui->aircraft->horizontalHeader(), SIGNAL(sectionMoved(int, int, int)), SLOT(aircraft_sectionMoved(int, int, int)));
    connect(ui->aircraft->horizontalHeader(), SIGNAL(sectionResized(int, int, int)), SLOT(aircraft_sectionResized(int, int, int)));

    m_flightColumnMenu = new QMenu(ui->flights);
    for (int i = 0; i < FLIGHT_COLUMNS; i++)
    {
        QString text = m_flightModel->headerData(i, Qt::Horizontal).toString();
        m_flightColumnMenu->addAction(createCheckableItem(text, i, true, SLOT(flightColumnSelectMenuChecked())));
    }
    ui->flights->horizontalHeader()->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->flights->horizontalHeader(), SIGNAL(customContextMenuRequested(QPoint)), SLOT(flightColumnSelectMenu(QPoint)));
    connect(ui->flights->horizontalHeader(), SIGNAL(sectionMoved(int, int, int)), SLOT(flights_sectionMoved(int, int, int)));
    connect(ui->flights->horizontalHeader(), SIGNAL(sectionResized(int, int, int)), SLOT(flights_sectionResized(int, int, int)));

    // The tracker does all the processing on its own thread; these queued
    // connections deliver its batched snapshots
    AircraftTracker *t = tracker();
    connect(t, &AircraftTracker::aircraftUpdated, this, &AircraftGUI::trackerAircraftUpdated);
    connect(t, &AircraftTracker::aircraftRemoved, this, &AircraftGUI::trackerAircraftRemoved);
    connect(t, &AircraftTracker::flightsUpdated, this, &AircraftGUI::trackerFlightsUpdated);
    connect(t, &AircraftTracker::flightsRemoved, this, &AircraftGUI::trackerFlightsRemoved);
    connect(t, &AircraftTracker::documentsAdded, this, &AircraftGUI::trackerDocumentsAdded);
    connect(t, &AircraftTracker::atcMessages, this, &AircraftGUI::trackerAtcMessages);
    connect(t, &AircraftTracker::weatherReports, this, &AircraftGUI::trackerWeatherReports);
    connect(t, &AircraftTracker::profileUpdated, this, &AircraftGUI::trackerProfileUpdated);
    connect(t, &AircraftTracker::messageRates, this, &AircraftGUI::trackerMessageRates);
    connect(t, &AircraftTracker::statisticsUpdated, this, &AircraftGUI::trackerStatistics);
    connect(t, &AircraftTracker::speechNotification, this, &AircraftGUI::trackerSpeechNotification);
    connect(t, &AircraftTracker::allCleared, this, &AircraftGUI::trackerAllCleared);
    connect(t, &AircraftTracker::databaseFilenameReverted,
            this, &AircraftGUI::trackerDatabaseFilenameReverted);

    m_settings.setRollupState(&m_rollupState);

    displaySettings();
    applySettings(true);
    makeUIConnections();
    m_resizer.enableChildMouseTracking();

    // Get the current state - the tracker may have restored a session before
    // this GUI connected
    t->getInputMessageQueue()->push(AircraftTracker::MsgResync::create());
}

AircraftGUI::~AircraftGUI()
{
    delete ui;
}

void AircraftGUI::makeUIConnections()
{
    QObject::connect(
        ui->deleteAircraft,
        &QToolButton::clicked,
        this,
        &AircraftGUI::on_deleteAircraft_clicked
    );
    QObject::connect(
        ui->settings,
        &QToolButton::clicked,
        this,
        &AircraftGUI::on_settings_clicked
    );
    QObject::connect(
        ui->notifications,
        &QToolButton::clicked,
        this,
        &AircraftGUI::on_notifications_clicked
    );
    QObject::connect(
        ui->atcLabels,
        &ButtonSwitch::clicked,
        this,
        &AircraftGUI::on_atcLabels_clicked
    );
    // The statistics sit to the right of the tables. Give the tables the bulk of the
    // width by default - the statistics are two narrow columns and a date - while still
    // letting the handle be dragged.
    ui->aircraftSplitter->setStretchFactor(0, 4);
    ui->aircraftSplitter->setStretchFactor(1, 1);

    QObject::connect(ui->stats, &ButtonSwitch::clicked, this, &AircraftGUI::on_stats_clicked);
    ui->stats->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(ui->stats, &QWidget::customContextMenuRequested, this, &AircraftGUI::statsContextMenu);
    QObject::connect(ui->displayChart, &ButtonSwitch::clicked, this, &AircraftGUI::on_displayChart_clicked);
    ui->displayChart->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(ui->displayChart, &QWidget::customContextMenuRequested, this, &AircraftGUI::chartContextMenu);

    // Nothing is selected yet, so the two history tabs have nothing to be about. They
    // are visible in the .ui so that they can be seen while it is being edited.
    updateHistoryTables();
}

void AircraftGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void AircraftGUI::applySettings(bool force)
{
    if (m_doApplySettings)
    {
        Aircraft::MsgConfigureAircraft* message = Aircraft::MsgConfigureAircraft::create(m_settings, m_settingsKeys, force);
        m_aircraft->getInputMessageQueue()->push(message);
    }

    m_settingsKeys.clear();
}

// Default column widths from representative content
// Row numbers down the left of a table. The numbers themselves come from
// ActiveFilterProxy::headerData(), which counts displayed position rather than source
// row so that the top row is 1 whatever the sort order.
//
// The header is left non-clickable: clicking a vertical section selects the row, which
// the table already does when a cell is clicked, and making it look interactive invites
// the assumption that it sorts.
void AircraftGUI::showRowNumbers(QTableView *table)
{
    QHeaderView *header = table->verticalHeader();
    header->setVisible(true);
    header->setSectionsClickable(false);
    header->setHighlightSections(false);
    // The resize mode is deliberately left alone. Setting ResizeToContents here - which
    // seemed reasonable, since the rows carry airline logos of differing heights - makes
    // the header re-measure EVERY row whenever anything in the table changes. On a table
    // holding a session's worth of restored aircraft, each measurement pulls in that
    // row's logo and sideview images, and start up went from seconds to over a minute of
    // a blocked GUI. Row numbering does not need it: the view sizes its own rows exactly
    // as it did when the header was hidden.
}

// The Statistics tab. Rebuilt wholesale on each update - it is a couple of dozen rows
// arriving a few times a second at most, and a table that small is cheaper to refill
// than to diff.
//
// Every figure is given twice, for this session and for all time, because the pair is
// what makes either readable: 210 km on its own says nothing, 210 km against an all time
// best of 340 km says the band is poor today. The session columns come first, as the
// ones that change while you watch.
void AircraftGUI::trackerStatistics(const AircraftStatistics& statistics)
{
    m_statisticsBlob = AircraftTracker::serializeStatistics(statistics);

    // Sized by the enum, so adding a protocol fails to compile here rather than
    // reading past the end of the array on the very next line
    static const char *protocolNames[AircraftReport::ProtocolCount] =
        { "ADS-B", "ACARS", "VDL-2", "HFDL", "Aero" };

    QTableWidget *table = ui->statistics;
    const bool first = (table->columnCount() == 0);
    if (first)
    {
        table->setColumnCount(5);
        table->setHorizontalHeaderLabels(
            { "Statistic", "Session", "Session set by", "All time", "All time set by" });
        table->horizontalHeader()->setSectionsMovable(true);
        table->verticalHeader()->setVisible(false);
    }

    QList<QStringList> rows;

    auto record = [](const AircraftStatistics::Record& r, const QString& suffix)
    {
        if (!r.m_valid) {
            return QStringList{ QString(), QString() };
        }
        return QStringList{
            QString("%1 %2").arg(r.m_value, 0, 'f', 1).arg(suffix),
            QString("%1, %2").arg(r.m_aircraft,
                r.m_when.isValid() ? r.m_when.toString("yyyy-MM-dd hh:mm") : QString("?"))
        };
    };

    // Interleaves a statistic computed for each scope into one row: label, then the
    // session's value and detail, then all time's
    auto both = [&rows](const QString& label, const QStringList& session,
                        const QStringList& allTime)
    {
        rows.append(QStringList{ label } + session + allTime);
    };

    const AircraftStatistics::Scope& ses = statistics.m_session;
    const AircraftStatistics::Scope& all = statistics.m_allTime;

    // Furthest per protocol. They are wildly different by nature - line of sight VHF
    // against HF skywave against a satellite downlink - so one combined figure would
    // say nothing useful.
    for (int i = 0; i < AircraftReport::ProtocolCount; i++)
    {
        both(QString("Maximum range, %1").arg(protocolNames[i]),
             record(ses.m_maxRange[i], "km"), record(all.m_maxRange[i], "km"));
    }

    both("Fastest ground speed", record(ses.m_fastest, "kn"), record(all.m_fastest, "kn"));
    both("Highest altitude", record(ses.m_highest, "ft"), record(all.m_highest, "ft"));

    auto concurrent = [](const AircraftStatistics::Scope& scope)
    {
        return QStringList{ QString::number(scope.m_maxConcurrent),
            scope.m_maxConcurrentWhen.isValid()
                ? scope.m_maxConcurrentWhen.toString("yyyy-MM-dd hh:mm") : QString() };
    };
    both(QString("Most aircraft at once (%1 min)").arg(AircraftStatistics::ConcurrentWindowMins),
         concurrent(ses), concurrent(all));

    both("Aircraft heard",
         QStringList{ QString::number(ses.m_distinctAircraft), QString() },
         QStringList{ QString::number(all.m_distinctAircraft), QString() });
    both("Total messages",
         QStringList{ QString::number(ses.m_totalMessages), QString() },
         QStringList{ QString::number(all.m_totalMessages), QString() });

    // The share is of that scope's own total, so a protocol that has been quiet today
    // shows its reduced share against its long run one rather than against nothing
    auto messages = [](const AircraftStatistics::Scope& scope, int protocol)
    {
        const quint64 n = scope.m_messagesByProtocol[protocol];
        QString share;
        if (scope.m_totalMessages > 0) {
            share = QString("%1%").arg(100.0 * n / scope.m_totalMessages, 0, 'f', 1);
        }
        return QStringList{ QString::number(n), share };
    };
    for (int i = 0; i < AircraftReport::ProtocolCount; i++) {
        both(QString("Messages, %1").arg(protocolNames[i]), messages(ses, i), messages(all, i));
    }

    auto elapsed = [](qint64 secs, const QDateTime& since)
    {
        if (secs <= 0) {
            return QStringList{ QString(), QString() };
        }
        return QStringList{ QString("%1h %2m").arg(secs / 3600).arg((secs % 3600) / 60),
            since.isValid() ? QString("since %1").arg(since.toString("yyyy-MM-dd hh:mm"))
                            : QString() };
    };
    both("Listening time", elapsed(ses.m_seconds, statistics.m_sessionStart),
         elapsed(all.m_seconds, statistics.m_firstStart));

    auto perMinute = [](const AircraftStatistics::Scope& scope)
    {
        if (scope.m_seconds <= 0) {
            return QStringList{ QString(), QString() };
        }
        return QStringList{
            QString("%1").arg(scope.m_totalMessages * 60.0 / scope.m_seconds, 0, 'f', 1),
            QString() };
    };
    both("Messages per minute", perMinute(ses), perMinute(all));

    table->setRowCount(rows.size());
    for (int row = 0; row < rows.size(); row++)
    {
        for (int col = 0; col < table->columnCount(); col++)
        {
            QTableWidgetItem *item = table->item(row, col);
            if (!item)
            {
                item = new QTableWidgetItem();
                table->setItem(row, col, item);
            }
            item->setText(rows[row].value(col));
        }
    }
    if (first) {
        table->resizeColumnsToContents();
    }
}

void AircraftGUI::setDefaultColumnWidths()
{

    const QFontMetrics fm(font());
    auto width = [&fm](const char *sample) { return fm.horizontalAdvance(sample) + 12; };

    ui->aircraft->setColumnWidth(AircraftTableModel::COL_ICAO, width("ABCDEF"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_REG, width("G-ABCDEF"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_TYPE, width("B77W"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_FLIGHT, width("RYR33KQW"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_AIRLINE, std::max(90, width("Airline name")));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_SIDEVIEW, 90);
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_COUNTRY, 45);
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_LATITUDE, width("-90.00000"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_LONGITUDE, width("-180.00000"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_DISTANCE, width("Dist (km)"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_ALTITUDE, width("40000"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_HEADING, width("360"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_SPEED, width("500"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_PROTOCOLS, width("ACARS 131.725; HFDL 10.081"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_MESSAGES, width("10000"));
    ui->aircraft->setColumnWidth(AircraftTableModel::COL_LAST_SEEN, width("12/12/2026 12:00:00"));

    ui->flights->setColumnWidth(FlightTableModel::COL_FLIGHT, width("RYR33KQW"));
    ui->flights->setColumnWidth(FlightTableModel::COL_REG, width("G-ABCDEF"));
    ui->flights->setColumnWidth(FlightTableModel::COL_FROM, width("EGKK"));
    ui->flights->setColumnWidth(FlightTableModel::COL_TO, width("EGKK"));
    ui->flights->setColumnWidth(FlightTableModel::COL_FIRST_SEEN, width("12/12/2026 12:00:00"));
    ui->flights->setColumnWidth(FlightTableModel::COL_LAST_SEEN, width("12/12/2026 12:00:00"));
    ui->flights->setColumnWidth(FlightTableModel::COL_DOCS, width("Docs"));
    ui->flights->setColumnWidth(FlightTableModel::COL_PROTOCOLS, width("ACARS 131.725; HFDL 10.081"));

    // The archive views show the same columns, so give them the same widths
    for (auto pair : { qMakePair((QTableView *) ui->aircraft, (QTableView *) ui->oldAircraft),
                       qMakePair((QTableView *) ui->flights, (QTableView *) ui->oldFlights) })
    {
        for (int col = 0; col < pair.first->model()->columnCount(); col++) {
            pair.second->setColumnWidth(col, pair.first->columnWidth(col));
        }
    }
}

void AircraftGUI::displaySettings()
{
    setTitleColor(m_settings.m_rgbColor);
    setWindowTitle(m_settings.m_title);
    setTitle(m_settings.m_title);
    blockApplySettings(true);

    ui->atcLabels->setChecked(m_settings.m_atcLabels);
    ui->stats->setChecked(m_settings.m_displayStatistics);
    ui->statistics->setVisible(m_settings.m_displayStatistics);
    ui->displayChart->setChecked(m_settings.m_displayChart);
    ui->chart->setVisible(m_settings.m_displayChart);

    // Order and size columns
    QHeaderView *header = ui->aircraft->horizontalHeader();

    for (int i = 0; i < AIRCRAFT_COLUMNS; i++)
    {
        bool hidden = m_settings.m_columnSizes[i] == 0;
        header->setSectionHidden(i, hidden);
        if (i < m_columnMenu->actions().size()) {
            m_columnMenu->actions().at(i)->setChecked(!hidden);
        }
        if (m_settings.m_columnSizes[i] > 0) {
            ui->aircraft->setColumnWidth(i, m_settings.m_columnSizes[i]);
        }
        header->moveSection(header->visualIndex(i), m_settings.m_columnIndexes[i]);
    }

    QHeaderView *flightHeader = ui->flights->horizontalHeader();
    for (int i = 0; i < FLIGHT_COLUMNS; i++)
    {
        bool hidden = m_settings.m_flightColumnSizes[i] == 0;
        flightHeader->setSectionHidden(i, hidden);
        if (i < m_flightColumnMenu->actions().size()) {
            m_flightColumnMenu->actions().at(i)->setChecked(!hidden);
        }
        if (m_settings.m_flightColumnSizes[i] > 0) {
            ui->flights->setColumnWidth(i, m_settings.m_flightColumnSizes[i]);
        }
        flightHeader->moveSection(flightHeader->visualIndex(i), m_settings.m_flightColumnIndexes[i]);
    }

    // After the column work, so the tables are their final shape before the splitters
    // are asked to divide the space between them
    restoreSplitters(m_settings.m_splitterStates);

    getRollupContents()->restoreState(m_rollupState);
    blockApplySettings(false);
    getRollupContents()->arrangeRollups();
}

void AircraftGUI::onMenuDialogCalled(const QPoint &p)
{
    if (m_contextMenuType == ContextMenuType::ContextMenuChannelSettings)
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

        m_settings.m_title = dialog.getTitle();
        m_settings.m_useReverseAPI = dialog.useReverseAPI();
        m_settings.m_reverseAPIAddress = dialog.getReverseAPIAddress();
        m_settings.m_reverseAPIPort = dialog.getReverseAPIPort();
        m_settings.m_reverseAPIFeatureSetIndex = dialog.getReverseAPIFeatureSetIndex();
        m_settings.m_reverseAPIFeatureIndex = dialog.getReverseAPIFeatureIndex();

        setTitle(m_settings.m_title);
        setTitleColor(m_settings.m_rgbColor);

        m_settingsKeys.append("title");
        m_settingsKeys.append("useReverseAPI");
        m_settingsKeys.append("reverseAPIAddress");
        m_settingsKeys.append("reverseAPIPort");
        m_settingsKeys.append("reverseAPIFeatureSetIndex");
        m_settingsKeys.append("reverseAPIFeatureIndex");

        applySettings();
    }

    resetContextMenuType();
}

// ---- Snapshots from the tracker ----

void AircraftGUI::trackerAircraftUpdated(const QList<AircraftDisplay>& aircraft)
{
    m_aircraftModel->upsert(aircraft);

    // Keep the selection-derived state fresh
    for (const AircraftDisplay& d : aircraft)
    {
        if (d.m_id == m_selectedAircraftId)
        {
            m_selectedRegistration = d.m_registration;
            m_selectedIcao = d.m_icao;
        }
    }
}

void AircraftGUI::trackerAircraftRemoved(const QList<quint64>& ids)
{
    m_aircraftModel->remove(ids);
    if (ids.contains(m_selectedAircraftId)) {
        m_selectedAircraftId = 0;
    }
}

void AircraftGUI::trackerFlightsUpdated(const QList<FlightDisplay>& flights)
{
    m_flightModel->upsert(flights);
    // A new flight for the aircraft on screen, or a new aircraft on the flight on screen,
    // has to show up without the user clicking away and back
    if ((m_selectedAircraftId != 0) || !m_selectedFlightNumber.isEmpty()) {
        updateHistoryTables();
    }
}

void AircraftGUI::trackerFlightsRemoved(const QList<quint64>& ids)
{
    m_flightModel->remove(ids);
    if (ids.contains(m_selectedFlightId)) {
        m_selectedFlightId = 0;
    }
}

void AircraftGUI::trackerDocumentsAdded(const QList<DocumentEvent>& documents)
{
    ui->documents->setSortingEnabled(false);
    for (const DocumentEvent& e : documents)
    {
        int row = ui->documents->rowCount();
        ui->documents->setRowCount(row + 1);
        QTableWidgetItem *timeItem = new QTableWidgetItem();
        timeItem->setData(Qt::DisplayRole, e.m_received);
        timeItem->setData(Qt::UserRole, e.m_text);
        timeItem->setData(Qt::UserRole + 1, (qulonglong) e.m_aircraftId);
        timeItem->setData(Qt::UserRole + 2, (qulonglong) e.m_flightId);
        timeItem->setData(Qt::UserRole + 3, e.m_mapName);
        ui->documents->setItem(row, DOCUMENT_COL_TIME, timeItem);
        ui->documents->setItem(row, DOCUMENT_COL_FLIGHT, new QTableWidgetItem(e.m_flight));
        ui->documents->setItem(row, DOCUMENT_COL_REG, new QTableWidgetItem(e.m_reg));
        ui->documents->setItem(row, DOCUMENT_COL_TYPE, new QTableWidgetItem(e.m_kind));
        ui->documents->setItem(row, DOCUMENT_COL_TITLE, new QTableWidgetItem(e.m_title));
    }
    // setSortingEnabled(true) sorts immediately, and a row's hidden flag belongs to the
    // VIEW's row number rather than to the item, so it does not travel with the row.
    // Hiding inside the loop above therefore left the flags on whichever rows the sort
    // happened to move into those positions - documents vanishing or being replaced by
    // unrelated ones, with nothing touched.
    ui->documents->setSortingEnabled(true);
    applyMessageFilter();
}

// Weather is about airports, so unlike the Documents and ATC tables it is not filtered
// by the selected aircraft - the aircraft that carried a METAR is incidental to it.
void AircraftGUI::trackerWeatherReports(const QList<WeatherEvent>& reports)
{
    ui->weather->setSortingEnabled(false);
    for (const WeatherEvent& e : reports)
    {
        int row = ui->weather->rowCount();
        ui->weather->setRowCount(row + 1);

        QTableWidgetItem *timeItem = new QTableWidgetItem();
        timeItem->setData(Qt::DisplayRole, e.m_received);
        timeItem->setData(Qt::UserRole, e.m_text);      // Read back by the detail pane
        ui->weather->setItem(row, WEATHER_COL_TIME, timeItem);
        ui->weather->setItem(row, WEATHER_COL_AIRPORT, new QTableWidgetItem(e.m_airport));
        ui->weather->setItem(row, WEATHER_COL_TYPE, new QTableWidgetItem(e.m_kind));
        ui->weather->setItem(row, WEATHER_COL_VIA, new QTableWidgetItem(e.m_from));
        QTableWidgetItem *reportItem = new QTableWidgetItem(e.m_summary);
        reportItem->setToolTip(e.m_text);
        ui->weather->setItem(row, WEATHER_COL_REPORT, reportItem);
    }
    ui->weather->setSortingEnabled(true);
    ui->weather->scrollToBottom();
}

void AircraftGUI::weatherSelectionChanged()
{
    const QList<QTableWidgetItem *> selected = ui->weather->selectedItems();
    for (const auto *selItem : selected)
    {
        QTableWidgetItem *timeItem = ui->weather->item(selItem->row(), WEATHER_COL_TIME);
        if (timeItem)
        {
            ui->weatherText->setPlainText(timeItem->data(Qt::UserRole).toString());
            return;
        }
    }
    ui->weatherText->clear();
}

void AircraftGUI::trackerAtcMessages(const QList<AtcEvent>& messages)
{
    ui->atcMessages->setSortingEnabled(false);
    for (const AtcEvent& e : messages)
    {
        int row = ui->atcMessages->rowCount();
        ui->atcMessages->setRowCount(row + 1);
        QTableWidgetItem *timeItem = new QTableWidgetItem();
        timeItem->setData(Qt::DisplayRole, e.m_received);
        timeItem->setData(Qt::UserRole + 1, (qulonglong) e.m_aircraftId);
        timeItem->setData(Qt::UserRole + 2, (qulonglong) e.m_flightId);
        timeItem->setData(Qt::UserRole + 3, e.m_mapName);
        ui->atcMessages->setItem(row, ATC_COL_TIME, timeItem);
        ui->atcMessages->setItem(row, ATC_COL_PROTOCOL, new QTableWidgetItem(e.m_protocol));
        ui->atcMessages->setItem(row, ATC_COL_DIR,
            new QTableWidgetItem(QString("%1").arg(QChar(e.m_uplink ? 0x2191 : 0x2193))));
        ui->atcMessages->setItem(row, ATC_COL_FROM, new QTableWidgetItem(e.m_from));
        ui->atcMessages->setItem(row, ATC_COL_TO, new QTableWidgetItem(e.m_to));
        QTableWidgetItem *messageItem = new QTableWidgetItem(e.m_message);
        messageItem->setToolTip(e.m_tooltip);
        ui->atcMessages->setItem(row, ATC_COL_MESSAGE, messageItem);
    }
    // As in trackerDocumentsAdded: hide after the sort, not before
    ui->atcMessages->setSortingEnabled(true);
    applyMessageFilter();
    ui->atcMessages->scrollToBottom();
}

// The tracker could not move to the database that was asked for and has stayed with the
// one it had. These settings are the ones that get serialised, so they have to agree -
// otherwise the preset would name a file nothing is writing to.
void AircraftGUI::trackerDatabaseFilenameReverted(const QString& filename)
{
    if (m_settings.m_databaseFilename == filename) {
        return;
    }
    m_settings.m_databaseFilename = filename;
    m_settingsKeys.append("databaseFilename");
    applySettings();
}

void AircraftGUI::trackerAllCleared()
{
    m_aircraftModel->clear();
    m_flightModel->clear();
    ui->documents->setRowCount(0);
    ui->documentText->clear();
    ui->atcMessages->setRowCount(0);
    ui->weather->setRowCount(0);
    ui->weatherText->clear();
    m_selectedAircraftId = 0;
    m_selectedFlightId = 0;
    m_selectedRegistration.clear();
    m_selectedIcao = 0;
    plotProfileChart();
}

void AircraftGUI::trackerSpeechNotification(const QString& speech)
{
#ifdef QT_TEXTTOSPEECH_FOUND
    if (!m_speech) {
        m_speech = new QTextToSpeech(this);
    }
    m_speech->say(speech);
#else
    qWarning() << "AircraftGUI::trackerSpeechNotification: TextToSpeech not supported. Unable to say " << speech;
#endif
}

// ---- Selection, filtering, photo and profile ----

// Whether a Documents/ATC row matches the selected aircraft (Aircraft tab
// active) or flight (Flights tab active); everything shows with no selection
bool AircraftGUI::messageRowVisible(const QTableWidgetItem *firstItem) const
{
    if (m_selectedFlightId) {
        return firstItem->data(Qt::UserRole + 2).toULongLong() == m_selectedFlightId;
    }
    if (m_selectedAircraftId) {
        return firstItem->data(Qt::UserRole + 1).toULongLong() == m_selectedAircraftId;
    }
    return false;
}

void AircraftGUI::applyMessageFilter()
{
    for (QTableWidget *table : {ui->documents, ui->atcMessages})
    {
        for (int row = 0; row < table->rowCount(); row++) {
            table->setRowHidden(row, !messageRowVisible(table->item(row, 0)));
        }
    }
}

// The archive tabs show what we are no longer hearing from
bool AircraftGUI::showingArchive() const
{
    QWidget *tab = ui->aircraftTabs->currentWidget();
    return (tab == ui->oldAircraftTab) || (tab == ui->oldFlightsTab);
}

// What the aircraft database knows about this airframe, which does not change as it flies
void AircraftGUI::updateAircraftInfo(const QString& registration, quint32 icao)
{
    ui->aircraftInfo->setRowCount(0);

    QSharedPointer<const QHash<QString, AircraftInformation *>> byReg = OsnDB::getAircraftInformationByReg();
    QSharedPointer<const QHash<int, AircraftInformation *>> byIcao = OsnDB::getAircraftInformation();
    const AircraftInformation *info = nullptr;

    if (byReg && byReg->contains(registration)) {
        info = byReg->value(registration);
    } else if (icao && byIcao && byIcao->contains((int) icao)) {
        info = byIcao->value((int) icao);
    }
    if (!info) {
        return;
    }

    const QList<QPair<QString, QString>> details = {
        { "ICAO", QString("%1").arg(info->m_icao, 6, 16, QChar('0')).toUpper() },
        { "Registration", info->m_registration },
        { "Manufacturer", info->m_manufacturerName },
        { "Model", info->m_model },
        { "Type", info->m_type },
        { "Owner", info->m_owner },
        { "Operator", info->m_operator },
        { "Operator ICAO", info->m_operatorICAO },
        { "Registered", info->m_registered },
    };

    for (const auto& detail : details)
    {
        if (detail.second.isEmpty()) {
            continue;
        }
        int row = ui->aircraftInfo->rowCount();
        ui->aircraftInfo->setRowCount(row + 1);
        ui->aircraftInfo->setItem(row, 0, new QTableWidgetItem(detail.first));
        ui->aircraftInfo->setItem(row, 1, new QTableWidgetItem(detail.second));
    }

    // The country is shown as its flag, as it is in the aircraft table
    const QString flag = info->getFlag();
    if (!flag.isEmpty())
    {
        int row = ui->aircraftInfo->rowCount();
        ui->aircraftInfo->setRowCount(row + 1);
        ui->aircraftInfo->setItem(row, 0, new QTableWidgetItem("Country"));

        QTableWidgetItem *country = new QTableWidgetItem();
        QIcon *icon = AircraftInformation::getFlagIcon(flag);
        if (icon)
        {
            country->setIcon(*icon);
            country->setToolTip(flag);
        }
        else
        {
            country->setText(flag);
        }
        ui->aircraftInfo->setItem(row, 1, country);
    }
    ui->aircraftInfo->resizeColumnToContents(0);
}

// The two history tabs, both built from the flight model rather than from the tracker.
//
// Every flight the feature knows about is already in that model, live and archived alike,
// and a FlightDisplay carries both the aircraft it was flown by and the flight number -
// so "which flights has this aircraft operated" and "which aircraft have operated this
// flight" are two readings of the same list, and neither needs the tracker to be asked.
// Highlight the next row matching what was typed, in whichever table is on show.
//
// A filter would hide everything else, and what is wanted here is to pick one aircraft
// out of a busy table while still seeing the rest of the traffic around it - so this
// selects and scrolls to the match instead. Enter steps on to the next one and wraps.
bool AircraftGUI::findFrom(int startRow, const QString& text)
{
    if (text.isEmpty()) {
        return false;
    }

    const bool flights = (ui->aircraftTabs->currentWidget() == ui->flightsTab)
                      || (ui->aircraftTabs->currentWidget() == ui->oldFlightsTab);
    QTableView *view = flights ? (showingArchive() ? ui->oldFlights : ui->flights)
                               : (showingArchive() ? ui->oldAircraft : ui->aircraft);
    ActiveFilterProxy *proxy = flights ? (showingArchive() ? m_oldFlightProxy : m_flightProxy)
                                       : (showingArchive() ? m_oldAircraftProxy : m_aircraftProxy);
    const int rows = proxy->rowCount();
    if (rows == 0) {
        return false;
    }
    const int column = ui->findColumn->currentIndex();

    for (int i = 0; i < rows; i++)
    {
        // Walk in the order the rows are displayed, so stepping through the matches
        // follows the sort the user is looking at rather than the model's own order
        const int row = (startRow + 1 + i) % rows;
        const int sourceRow = proxy->mapToSource(proxy->index(row, 0)).row();

        // Type and airline are not on either snapshot - they come from the database,
        // which is what the table itself displays in those columns, so searching them
        // matches what is on screen
        auto typeFields = [](const QString& registration) {
            QStringList f;
            QSharedPointer<const QHash<QString, AircraftInformation *>> byReg
                = OsnDB::getAircraftInformationByReg();
            if (byReg && byReg->contains(registration))
            {
                const AircraftInformation *info = byReg->value(registration);
                f << info->m_type << info->m_model << info->m_manufacturerName;
            }
            return f;
        };

        QStringList fields;
        if (flights)
        {
            const FlightDisplay *d = m_flightModel->flightAt(sourceRow);
            if (!d) {
                continue;
            }
            // A flight has no ICAO address of its own - the airframe flying it does,
            // and that is what the Aircraft table is for, so ICAO simply finds nothing
            switch (column)
            {
            case FindIcao:   break;
            case FindReg:    fields << d->m_reg; break;
            case FindType:   fields << typeFields(d->m_reg); break;
            case FindFlight: fields << d->m_flight << d->m_aliases; break;
            default:
                fields << d->m_flight << d->m_aliases << d->m_reg
                       << d->m_departure << d->m_arrival << d->m_route
                       << typeFields(d->m_reg);
                break;
            }
        }
        else
        {
            const AircraftDisplay *d = m_aircraftModel->aircraftAt(sourceRow);
            if (!d) {
                continue;
            }
            const QString icao = QString("%1").arg(d->m_icao, 6, 16, QChar('0')).toUpper();
            switch (column)
            {
            case FindIcao:   fields << icao; break;
            case FindReg:    fields << d->m_registration; break;
            case FindType:   fields << typeFields(d->m_registration); break;
            case FindFlight: fields << d->m_flight; break;
            default:
                fields << icao << d->m_registration << d->m_flight << d->m_mapName
                       << typeFields(d->m_registration);
                QSharedPointer<const QHash<QString, AircraftInformation *>> byReg
                    = OsnDB::getAircraftInformationByReg();
                if (byReg && byReg->contains(d->m_registration))
                {
                    const AircraftInformation *info = byReg->value(d->m_registration);
                    fields << info->m_operator << info->m_operatorICAO << info->m_owner;
                }
                break;
            }
        }

        for (const QString& field : fields)
        {
            if (!field.isEmpty() && field.contains(text, Qt::CaseInsensitive))
            {
                m_findRow = row;
                view->selectRow(row);
                view->scrollTo(proxy->index(row, 0), QAbstractItemView::PositionAtCenter);
                return true;
            }
        }
    }
    return false;
}

// Typing searches from the top, so the match follows what has been typed so far
void AircraftGUI::on_find_textChanged(const QString& text)
{
    m_findRow = -1;
    if (text.isEmpty())
    {
        m_findTimer.stop();
        ui->find->setStyleSheet("");
        return;
    }
    // Wait for a pause in typing. Each match selects a row, and selecting reloads the
    // message tables, the flight profile and the photograph, so searching on every
    // keystroke would do all of that for each half-typed registration
    m_findTimer.start(300);
}

// Enter steps on to the next match
void AircraftGUI::on_find_returnPressed()
{
    const QString text = ui->find->text();
    ui->find->setStyleSheet(findFrom(m_findRow, text) ? "" : "QLineEdit { color: red; }");
}

void AircraftGUI::updateHistoryTables()
{
    m_updatingHistoryTables = true;
    static const QStringList flightHeadings =
        { "Flight", "From", "To", "First seen", "Last seen", "Msgs" };
    static const QStringList aircraftHeadings =
        { "Reg", "From", "To", "First seen", "Last seen", "Msgs" };

    auto prepare = [](QTableWidget *table, const QStringList& headings)
    {
        if (table->columnCount() == 0)
        {
            table->setColumnCount(headings.size());
            table->setHorizontalHeaderLabels(headings);
            table->horizontalHeader()->setSectionsMovable(true);
            table->verticalHeader()->setVisible(false);
        }
        // Sorting has to be off while the rows are replaced, or Qt re-sorts after each
        // cell is set and the rows shuffle out from under the loop
        table->setSortingEnabled(false);
        table->setRowCount(0);
    };

    auto fill = [](QTableWidget *table, int row, const QStringList& values)
    {
        for (int col = 0; col < values.size(); col++) {
            table->setItem(row, col, new QTableWidgetItem(values[col]));
        }
    };

    // QTableWidgetItem sorts on the display role, so the dates have to sort correctly as
    // TEXT. This format does - year first, zero padded throughout - which is why it is
    // used here rather than a friendlier local format.
    const QString dateFormat = "yyyy-MM-dd hh:mm";

    // The tracker flushes every 300 ms and every flush that touches a flight rebuilds
    // these tables, so a selection the user has just made would be wiped within a frame
    // or two of making it. Remember it and put it back.
    const quint64 keepSelected = m_selectedFlightId;

    // Past Flights: everything this aircraft has operated
    prepare(ui->pastFlights, flightHeadings);
    if (m_selectedAircraftId != 0)
    {
        int row = 0;
        for (int i = 0; i < m_flightModel->rowCount(); i++)
        {
            const FlightDisplay *f = m_flightModel->flightAt(i);
            if (!f || (f->m_aircraftId != m_selectedAircraftId)) {
                continue;
            }
            ui->pastFlights->setRowCount(row + 1);
            fill(ui->pastFlights, row,
                 { f->m_flight, f->m_departure, f->m_arrival,
                   f->m_firstSeen.toString(dateFormat), f->m_lastSeen.toString(dateFormat),
                   QString::number(f->m_messages) });
            ui->pastFlights->item(row, 0)->setData(Qt::UserRole, f->m_id);
            row++;
        }
    }
    ui->pastFlights->setSortingEnabled(true);
    ui->pastFlights->sortByColumn(4, Qt::DescendingOrder);

    // After the sort, because sorting moves the rows around
    if (keepSelected)
    {
        for (int row = 0; row < ui->pastFlights->rowCount(); row++)
        {
            const QTableWidgetItem *first = ui->pastFlights->item(row, 0);
            if (first && (first->data(Qt::UserRole).toULongLong() == keepSelected))
            {
                ui->pastFlights->selectRow(row);
                break;
            }
        }
    }

    // Past Aircraft: everything that has operated this flight number
    prepare(ui->pastAircraft, aircraftHeadings);
    if (!m_selectedFlightNumber.isEmpty())
    {
        int row = 0;
        for (int i = 0; i < m_flightModel->rowCount(); i++)
        {
            const FlightDisplay *f = m_flightModel->flightAt(i);
            if (!f || f->m_flight.compare(m_selectedFlightNumber, Qt::CaseInsensitive)) {
                continue;
            }
            ui->pastAircraft->setRowCount(row + 1);
            fill(ui->pastAircraft, row,
                 { f->m_reg, f->m_departure, f->m_arrival,
                   f->m_firstSeen.toString(dateFormat), f->m_lastSeen.toString(dateFormat),
                   QString::number(f->m_messages) });
            row++;
        }
    }
    ui->pastAircraft->setSortingEnabled(true);
    ui->pastAircraft->sortByColumn(4, Qt::DescendingOrder);

    // These two tabs live in messageTabs, NOT aircraftTabs - asking aircraftTabs for
    // them returns -1, and setTabVisible(-1) does nothing at all, which is why they
    // used to stay visible whatever was selected.
    ui->messageTabs->setTabVisible(ui->messageTabs->indexOf(ui->pastFlightsTab),
                                   m_selectedAircraftId != 0);
    ui->messageTabs->setTabVisible(ui->messageTabs->indexOf(ui->pastAircraftTab),
                                   !m_selectedFlightNumber.isEmpty());
    m_updatingHistoryTables = false;
}

// Selecting one of the aircraft's past flights charts its profile and draws its track
// on the map. Nothing is selected once the table is rebuilt, which returns both to the
// flight the aircraft is on now.
void AircraftGUI::pastFlightSelectionChanged()
{
    if (m_updatingHistoryTables) {
        return;
    }

    quint64 flightId = 0;
    QString title;
    const QList<QTableWidgetItem *> selected = ui->pastFlights->selectedItems();
    if (!selected.isEmpty())
    {
        const QTableWidgetItem *first = ui->pastFlights->item(selected.first()->row(), 0);
        if (first)
        {
            flightId = first->data(Qt::UserRole).toULongLong();
            title = first->text();
        }
    }
    // With no past flight selected the messages widen back out to the whole aircraft,
    // while the track and the chart go back to the flight it is on now
    m_selectedFlightId = flightId;
    applyMessageFilter();

    if (flightId == 0)
    {
        for (int i = 0; i < m_aircraftModel->rowCount(); i++)
        {
            const AircraftDisplay *d = m_aircraftModel->aircraftAt(i);
            if (d && (d->m_id == m_selectedAircraftId))
            {
                flightId = d->m_currentFlightId;
                title = d->m_flight.isEmpty() ? d->m_mapName : d->m_flight;
                break;
            }
        }
    }
    watchFlight(flightId, title);
}

void AircraftGUI::aircraftSelectionChanged()
{
    m_selectedAircraftId = 0;
    m_selectedRegistration.clear();
    m_selectedIcao = 0;
    // An airframe is selected, so "what else has flown this number" no longer has a
    // number to be about
    m_selectedFlightId = 0;
    m_selectedFlightNumber.clear();
    quint64 currentFlightId = 0;
    QString title;

    QTableView *view = showingArchive() ? ui->oldAircraft : ui->aircraft;
    ActiveFilterProxy *proxy = showingArchive() ? m_oldAircraftProxy : m_aircraftProxy;
    QModelIndexList selected = view->selectionModel()->selectedRows();
    if (!selected.isEmpty())
    {
        int sourceRow = proxy->mapToSource(selected.first()).row();
        const AircraftDisplay *d = m_aircraftModel->aircraftAt(sourceRow);
        if (d)
        {
            m_selectedAircraftId = d->m_id;
            m_selectedRegistration = d->m_registration;
            m_selectedIcao = d->m_icao;
            currentFlightId = d->m_currentFlightId;
            title = d->m_flight.isEmpty() ? d->m_mapName : d->m_flight;
        }
    }
    applyMessageFilter();
    updateHistoryTables();
    updatePhoto(m_selectedRegistration, m_selectedIcao);
    watchFlight(currentFlightId, title);
}

void AircraftGUI::flightSelectionChanged()
{
    m_selectedFlightId = 0;
    m_selectedFlightNumber.clear();
    m_selectedAircraftId = 0;
    quint64 watchId = 0;
    QString title;
    QString registration;

    QTableView *view = showingArchive() ? ui->oldFlights : ui->flights;
    ActiveFilterProxy *proxy = showingArchive() ? m_oldFlightProxy : m_flightProxy;
    QModelIndexList selected = view->selectionModel()->selectedRows();
    if (!selected.isEmpty())
    {
        int sourceRow = proxy->mapToSource(selected.first()).row();
        const FlightDisplay *d = m_flightModel->flightAt(sourceRow);
        if (d)
        {
            m_selectedFlightId = d->m_id;
            m_selectedFlightNumber = d->m_flight;
            watchId = d->m_id;
            title = d->m_flight.isEmpty() ? d->m_reg : d->m_flight;
            registration = d->m_reg;
        }
    }
    applyMessageFilter();
    updateHistoryTables();
    updatePhoto(registration, 0);
    watchFlight(watchId, title);
}

// Tell the tracker which flight's profile to send us
void AircraftGUI::watchFlight(quint64 flightId, const QString& title)
{
    m_profileFlightId = flightId;
    m_profileTitle = title;
    m_profileTimes.clear();
    m_profileAltFt.clear();
    m_profileSpeedKts.clear();
    plotProfileChart();
    tracker()->getInputMessageQueue()->push(AircraftTracker::MsgWatchFlight::create(flightId));
}

void AircraftGUI::trackerProfileUpdated(quint64 flightId, const QList<qint64>& times, const QList<float>& altitudeFt, const QList<float>& speedKts)
{
    if (flightId != m_profileFlightId) {
        return;
    }
    m_profileTimes = times;
    m_profileAltFt = altitudeFt;
    m_profileSpeedKts = speedKts;
    plotProfileChart();
}

void AircraftGUI::documentSelectionChanged()
{
    QList<QTableWidgetItem *> selected = ui->documents->selectedItems();
    for (const auto *selItem : selected)
    {
        QTableWidgetItem *timeItem = ui->documents->item(selItem->row(), DOCUMENT_COL_TIME);
        if (timeItem)
        {
            ui->documentText->setPlainText(timeItem->data(Qt::UserRole).toString());
            return;
        }
    }
    // Nothing selected - the previous message is not the selected one, and leaving it
    // there reads as though it is
    ui->documentText->clear();
}

// Fetch and display the planespotters.net photo for an aircraft
void AircraftGUI::updatePhoto(const QString& registration, quint32 icao)
{
    QString key = !registration.isEmpty() ? registration
        : (icao ? QString("%1").arg(icao, 6, 16, QChar('0')).toUpper() : QString());
    if (key == m_photoKey) {
        return;
    }
    m_photoKey = key;

    // Hide the previous aircraft's photo
    ui->photoHeader->setVisible(false);
    ui->photoFlag->setVisible(false);
    ui->photo->setVisible(false);
    ui->photo->setPixmap(QPixmap());
    ui->flightDetails->setVisible(false);
    m_photoLink.clear();

    if (key.isEmpty()) {
        return;
    }

    updatePhotoText(registration, icao);
    updateAircraftInfo(registration, icao);
    if (!registration.isEmpty()) {
        m_planeSpotters.getAircraftPhotoByRegistration(registration);
    } else {
        m_planeSpotters.getAircraftPhoto(key);
    }
}

void AircraftGUI::updatePhotoText(const QString& registration, quint32 icao)
{
    ui->photoHeader->setText(registration.isEmpty()
        ? QString("%1").arg(icao, 6, 16, QChar('0')).toUpper() : registration);

    ui->photoFlag->setPixmap(QPixmap());
    ui->flightDetails->setPixmap(QPixmap());

    QSharedPointer<const QHash<QString, AircraftInformation *>> aircraftInfoDB = OsnDB::getAircraftInformationByReg();
    if (aircraftInfoDB && aircraftInfoDB->contains(registration))
    {
        const AircraftInformation *aircraftInfo = aircraftInfoDB->value(registration);

        QString flag = aircraftInfo->getFlag();
        if (!flag.isEmpty())
        {
            QIcon *icon = AircraftInformation::getFlagIcon(flag);
            if (icon)
            {
                QList<QSize> sizes = icon->availableSizes();
                if (sizes.size() > 0) {
                    ui->photoFlag->setPixmap(icon->pixmap(sizes[0]));
                }
            }
        }

        // The airline's logo under the photograph. The manufacturer, model, owner,
        // operator and registration date that used to sit beside the photograph are
        // gone - the aircraftInfo table next to it lists all five, so they were on
        // screen twice.
        QIcon *airlineIcon = AircraftInformation::getAirlineIcon(aircraftInfo->m_operatorICAO);
        if (airlineIcon)
        {
            QList<QSize> sizes = airlineIcon->availableSizes();
            if (sizes.size() > 0) {
                ui->flightDetails->setPixmap(airlineIcon->pixmap(sizes[0]));
            }
        }
    }
}

void AircraftGUI::aircraftPhoto(const PlaneSpottersPhoto *photo)
{
    // The download takes a while, and the selection can move on while it runs. Checking
    // only that SOMETHING is selected would put aircraft A's photograph next to aircraft
    // B's details, which reads as a wrong answer rather than a slow one.
    if (!photo->m_pixmap.isNull() && !m_photoKey.isEmpty() && (photo->m_id == m_photoKey))
    {
        ui->photo->setPixmap(photo->m_pixmap);
        ui->photo->setToolTip(QString("Photographer: %1").arg(photo->m_photographer)); // Required by terms of use
        ui->photoHeader->setVisible(true);
        ui->photoFlag->setVisible(true);
        ui->photo->setVisible(true);
        ui->flightDetails->setVisible(true);
        m_photoLink = photo->m_link;
    }
}

void AircraftGUI::photoClicked()
{
    // Photo needs to link back to PlaneSpotters, as per terms of use
    if (!m_photoLink.isEmpty()) {
        QDesktopServices::openUrl(QUrl(m_photoLink));
    }
}

// ---- Context menus ----

// Jump to a flight in the Flights table, from the aircraft that is flying it
void AircraftGUI::selectFlight(const QString& flight)
{
    ui->aircraftTabs->setCurrentWidget(ui->flightsTab);

    ActiveFilterProxy *proxy = m_flightProxy;
    QTableView *view = ui->flights;
    for (int row = 0; row < proxy->rowCount(); row++)
    {
        const FlightDisplay *d = m_flightModel->flightAt(proxy->mapToSource(proxy->index(row, 0)).row());
        if (d && !d->m_flight.compare(flight, Qt::CaseInsensitive))
        {
            view->selectRow(row);
            view->scrollTo(proxy->index(row, 0), QAbstractItemView::PositionAtCenter);
            return;
        }
    }
}

// And the other way: the airframe that flew a flight
void AircraftGUI::selectAircraft(quint64 aircraftId)
{
    ui->aircraftTabs->setCurrentWidget(ui->aircraftTab);

    ActiveFilterProxy *proxy = m_aircraftProxy;
    QTableView *view = ui->aircraft;
    for (int row = 0; row < proxy->rowCount(); row++)
    {
        const AircraftDisplay *d = m_aircraftModel->aircraftAt(proxy->mapToSource(proxy->index(row, 0)).row());
        if (d && (d->m_id == aircraftId))
        {
            view->selectRow(row);
            view->scrollTo(proxy->index(row, 0), QAbstractItemView::PositionAtCenter);
            return;
        }
    }
}

void AircraftGUI::showContextMenu(QWidget *parent, const QString& cellText, const QString& mapName,
                                  QPoint globalPos, const QString& showFlight, quint64 showAircraftId)
{
    QMenu *menu = new QMenu(parent);
    menu->setAttribute(Qt::WA_DeleteOnClose);

    if (!cellText.isEmpty())
    {
        QAction *copyAction = new QAction(QString("Copy %1").arg(cellText), menu);
        connect(copyAction, &QAction::triggered, this, [cellText]() {
            QGuiApplication::clipboard()->setText(cellText);
        });
        menu->addAction(copyAction);
    }
    if (!mapName.isEmpty())
    {
        QAction *findAction = new QAction(QString("Find %1 on map").arg(mapName), menu);
        connect(findAction, &QAction::triggered, this, [mapName]() {
            FeatureWebAPIUtils::mapFind(mapName);
        });
        menu->addAction(findAction);
    }
    if (!showFlight.isEmpty())
    {
        QAction *action = new QAction(QString("Display flight %1").arg(showFlight), menu);
        connect(action, &QAction::triggered, this, [this, showFlight]() {
            selectFlight(showFlight);
        });
        menu->addAction(action);
    }
    if (showAircraftId != 0)
    {
        QAction *action = new QAction("Display aircraft", menu);
        connect(action, &QAction::triggered, this, [this, showAircraftId]() {
            selectAircraft(showAircraftId);
        });
        menu->addAction(action);
    }
    if (menu->actions().isEmpty())
    {
        delete menu;
        return;
    }
    menu->popup(globalPos);
}

void AircraftGUI::viewContextMenu(QTableView *view, bool aircraftView, QPoint pos)
{
    QSortFilterProxyModel *proxy = qobject_cast<QSortFilterProxyModel *>(view->model());
    QModelIndex index = view->indexAt(pos);
    if (!index.isValid()) {
        return;
    }

    QString cellText = index.data(Qt::DisplayRole).toString();
    QString mapName;
    QString showFlight;
    quint64 showAircraftId = 0;
    if (aircraftView)
    {
        const AircraftDisplay *d = m_aircraftModel->aircraftAt(proxy->mapToSource(index).row());
        if (d)
        {
            mapName = d->m_mapName;
            showFlight = d->m_flight;
        }
    }
    else
    {
        const FlightDisplay *d = m_flightModel->flightAt(proxy->mapToSource(index).row());
        if (d)
        {
            mapName = d->m_reg;
            showAircraftId = d->m_aircraftId;
        }
    }
    showContextMenu(view, cellText, mapName, view->viewport()->mapToGlobal(pos),
                    showFlight, showAircraftId);
}

void AircraftGUI::tableContextMenu(QTableWidget *table, QPoint pos)
{
    QTableWidgetItem *item = table->itemAt(pos);
    if (!item) {
        return;
    }
    QString mapName = table->item(item->row(), 0)->data(Qt::UserRole + 3).toString();
    showContextMenu(table, item->text(), mapName, table->viewport()->mapToGlobal(pos));
}

// ---- Charts ----

void AircraftGUI::plotChart()
{
    QChart *oldChart = m_chart;

    m_chart = new QChart();
    m_chart->layout()->setContentsMargins(0, 0, 0, 0);
    m_chart->setMargins(QMargins(1, 1, 1, 1));
    m_chart->setTheme(QChart::ChartThemeDark);
    m_chart->legend()->setAlignment(Qt::AlignRight);

    m_chartXAxis = new QDateTimeAxis();
    m_chartXAxis->setFormat("hh:mm:ss");
    m_chartYAxis = new QValueAxis();
    m_chartYAxis->setTitleText("Msgs/s");

    m_chart->addAxis(m_chartXAxis, Qt::AlignBottom);
    m_chart->addAxis(m_chartYAxis, Qt::AlignLeft);

    static const char *seriesNames[CHART_SERIES] = {"ADS-B", "ACARS", "VDL2", "HFDL", "Aero", "Total"};
    for (int i = 0; i < CHART_SERIES; i++)
    {
        m_chartSeries[i] = new QLineSeries();
        m_chartSeries[i]->setName(seriesNames[i]);
        m_chart->addSeries(m_chartSeries[i]);
        m_chartSeries[i]->attachAxis(m_chartXAxis);
        m_chartSeries[i]->attachAxis(m_chartYAxis);
    }

    const QDateTime now = QDateTime::currentDateTime();
    m_chartXAxis->setMin(now.addSecs(-60));
    m_chartXAxis->setMax(now);
    m_chartYAxis->setRange(0.0, 10.0);

    ui->chart->setChart(m_chart);

    // Click a legend entry to show/hide its series
    const auto markers = m_chart->legend()->markers();
    for (QLegendMarker *marker : markers) {
        connect(marker, &QLegendMarker::clicked, this, &AircraftGUI::legendMarkerClicked);
    }

    delete oldChart;
}

void AircraftGUI::legendMarkerClicked()
{
    QLegendMarker *marker = qobject_cast<QLegendMarker *>(sender());
    if (marker)
    {
        marker->series()->setVisible(!marker->series()->isVisible());
        marker->setVisible(true);
    }
}

void AircraftGUI::trackerMessageRates(const QList<float>& rates)
{
    const QDateTime now = QDateTime::currentDateTime();

    for (int i = 0; (i < CHART_SERIES) && (i < rates.size()); i++)
    {
        m_chartSeries[i]->append(now.toMSecsSinceEpoch(), rates[i]);
        if (m_chartYAxis->max() < rates[i]) {
            m_chartYAxis->setMax(std::ceil(rates[i]) + 1.0);
        }
        // Bound long-running growth
        if (m_chartSeries[i]->count() > 1000) {
            m_chartSeries[i]->removePoints(0, m_chartSeries[i]->count() - 1000);
        }
    }
    m_chartXAxis->setMax(now);
    if (m_chartSeries[0]->count() > 0) {
        m_chartXAxis->setMin(QDateTime::fromMSecsSinceEpoch((qint64) m_chartSeries[0]->at(0).x()));
    }
}

// Altitude and speed against time for the watched flight
void AircraftGUI::plotProfileChart()
{
    QChart *oldChart = m_profileChart;
    m_profileChart = new QChart();
    m_profileChart->layout()->setContentsMargins(0, 0, 0, 0);
    m_profileChart->setMargins(QMargins(1, 1, 1, 1));
    m_profileChart->setTheme(QChart::ChartThemeDark);
    m_profileChart->legend()->setAlignment(Qt::AlignRight);
    m_profileChart->setTitle(m_profileFlightId ? m_profileTitle : "No flight selected");

    QDateTimeAxis *xAxis = new QDateTimeAxis();
    xAxis->setFormat("hh:mm");
    QValueAxis *altAxis = new QValueAxis();
    altAxis->setTitleText("Alt (ft)");
    QValueAxis *speedAxis = new QValueAxis();
    speedAxis->setTitleText("Speed (kn)");
    m_profileChart->addAxis(xAxis, Qt::AlignBottom);
    m_profileChart->addAxis(altAxis, Qt::AlignLeft);
    m_profileChart->addAxis(speedAxis, Qt::AlignRight);

    QLineSeries *altSeries = new QLineSeries();
    altSeries->setName("Altitude");
    QLineSeries *speedSeries = new QLineSeries();
    speedSeries->setName("Speed");

    float maxAlt = 0.0f;
    float maxSpeed = 0.0f;
    for (int i = 0; i < m_profileTimes.size(); i++)
    {
        if (!std::isnan(m_profileAltFt[i]))
        {
            altSeries->append(m_profileTimes[i], m_profileAltFt[i]);
            maxAlt = std::max(maxAlt, m_profileAltFt[i]);
        }
        if (!std::isnan(m_profileSpeedKts[i]))
        {
            speedSeries->append(m_profileTimes[i], m_profileSpeedKts[i]);
            maxSpeed = std::max(maxSpeed, m_profileSpeedKts[i]);
        }
    }

    m_profileChart->addSeries(altSeries);
    altSeries->attachAxis(xAxis);
    altSeries->attachAxis(altAxis);
    m_profileChart->addSeries(speedSeries);
    speedSeries->attachAxis(xAxis);
    speedSeries->attachAxis(speedAxis);

    if (!m_profileTimes.isEmpty())
    {
        xAxis->setMin(QDateTime::fromMSecsSinceEpoch(m_profileTimes.first()));
        xAxis->setMax(QDateTime::fromMSecsSinceEpoch(std::max(m_profileTimes.first() + 60000, m_profileTimes.last())));
    }
    else
    {
        const QDateTime now = QDateTime::currentDateTime();
        xAxis->setMin(now.addSecs(-60));
        xAxis->setMax(now);
    }
    altAxis->setRange(0.0, std::max(1000.0, std::ceil(maxAlt / 1000.0) * 1000.0));
    speedAxis->setRange(0.0, std::max(100.0, std::ceil(maxSpeed / 100.0) * 100.0));

    ui->profileChart->setChart(m_profileChart);
    delete oldChart;
}

// ---- Toolbar ----

void AircraftGUI::on_deleteAircraft_clicked()
{
    tracker()->getInputMessageQueue()->push(AircraftTracker::MsgDeleteAll::create());
}

void AircraftGUI::on_settings_clicked()
{
    AircraftSettingsDialog dialog(&m_settings, this);

    if (dialog.exec() == QDialog::Accepted)
    {
        const QStringList& changed = dialog.changedSettings();

        if (!changed.isEmpty())
        {
            m_settingsKeys.append(changed);
            applySettings();
        }
    }
}

void AircraftGUI::on_notifications_clicked()
{
    AircraftNotificationDialog dialog(&m_settings, this);
    new DialogPositioner(&dialog, true);
    if (dialog.exec() == QDialog::Accepted)
    {
        m_settingsKeys.append("notificationSettings");
        applySettings();
    }
}

// The statistics table and the message rate chart live in the same vertical splitter as
// the tables, rather than in tabs, so they can be watched at the same time as the traffic
// they describe - the same arrangement the ADS-B demodulator uses.
// Right clicking either toggle resets what it shows, which is how the same two buttons
// behave in the ADS-B demodulator
void AircraftGUI::statsContextMenu(const QPoint& pos)
{
    QMenu menu(this);
    QAction *session = menu.addAction("Reset session statistics");
    QAction *allTime = menu.addAction("Reset all time statistics");
    QAction *chosen = menu.exec(ui->stats->mapToGlobal(pos));

    if (chosen == session)
    {
        tracker()->getInputMessageQueue()->push(
            AircraftTracker::MsgResetStatistics::create(false));
    }
    else if (chosen == allTime)
    {
        // The stored blob has to be cleared as well as the tracker's copy, or the next
        // update would serialise the old figures straight back out of the preset
        m_statisticsBlob = QByteArray();
        m_settings.m_statistics = QByteArray();
        m_settingsKeys.append("statistics");
        tracker()->getInputMessageQueue()->push(
            AircraftTracker::MsgResetStatistics::create(true));
        applySettings();
    }
}

void AircraftGUI::on_stats_clicked(bool checked)
{
    m_settings.m_displayStatistics = checked;
    ui->statistics->setVisible(checked);
    m_settingsKeys.append("displayStatistics");
    applySettings();
}

void AircraftGUI::chartContextMenu(const QPoint& pos)
{
    QMenu menu(this);
    QAction *reset = menu.addAction("Reset chart data");
    if (menu.exec(ui->displayChart->mapToGlobal(pos)) == reset)
    {
        for (int i = 0; i < CHART_SERIES; i++)
        {
            if (m_chartSeries[i]) {
                m_chartSeries[i]->clear();
            }
        }
    }
}

void AircraftGUI::on_displayChart_clicked(bool checked)
{
    m_settings.m_displayChart = checked;
    ui->chart->setVisible(checked);
    m_settingsKeys.append("displayChart");
    applySettings();
}

void AircraftGUI::on_atcLabels_clicked(bool checked)
{
    m_settings.m_atcLabels = checked;
    m_settingsKeys.append("atcLabels");
    applySettings();    // The tracker refreshes the Map labels
}

// ---- Column management ----

void AircraftGUI::aircraft_sectionMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex)
{
    (void) oldVisualIndex;

    m_settings.m_columnIndexes[logicalIndex] = newVisualIndex;
    m_settingsKeys.append("columnIndexes");
    applySettings();
}

void AircraftGUI::aircraft_sectionResized(int logicalIndex, int oldSize, int newSize)
{
    (void) oldSize;

    m_settings.m_columnSizes[logicalIndex] = newSize;
    m_settingsKeys.append("columnSizes");
    applySettings();
}

void AircraftGUI::flights_sectionMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex)
{
    (void) oldVisualIndex;

    m_settings.m_flightColumnIndexes[logicalIndex] = newVisualIndex;
    m_settingsKeys.append("flightColumnIndexes");
    applySettings();
}

void AircraftGUI::flights_sectionResized(int logicalIndex, int oldSize, int newSize)
{
    (void) oldSize;

    m_settings.m_flightColumnSizes[logicalIndex] = newSize;
    m_settingsKeys.append("flightColumnSizes");
    applySettings();
}

void AircraftGUI::flightColumnSelectMenu(QPoint pos)
{
    m_flightColumnMenu->popup(ui->flights->horizontalHeader()->viewport()->mapToGlobal(pos));
}

void AircraftGUI::flightColumnSelectMenuChecked(bool checked)
{
    (void) checked;

    QAction* action = qobject_cast<QAction*>(sender());
    if (action != nullptr)
    {
        int idx = action->data().toInt(nullptr);
        ui->flights->setColumnHidden(idx, !action->isChecked());
    }
}

void AircraftGUI::aircraftColumnSelectMenu(QPoint pos)
{
    m_columnMenu->popup(ui->aircraft->horizontalHeader()->viewport()->mapToGlobal(pos));
}

void AircraftGUI::aircraftColumnSelectMenuChecked(bool checked)
{
    (void) checked;

    QAction* action = qobject_cast<QAction*>(sender());
    if (action != nullptr)
    {
        int idx = action->data().toInt(nullptr);
        ui->aircraft->setColumnHidden(idx, !action->isChecked());
    }
}

QAction *AircraftGUI::createCheckableItem(QString &text, int idx, bool checked, const char *slot)
{
    QAction *action = new QAction(text, this);
    action->setCheckable(true);
    action->setChecked(checked);
    action->setData(QVariant(idx));
    connect(action, SIGNAL(triggered()), this, slot);
    return action;
}
