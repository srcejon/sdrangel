///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021-2023 Jon Beniston, M7RCE <jon@beniston.com>                //
// Copyright (C) 2021-2022 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2021 DreamNik <dreamnik@mail.ru>                                //
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

#include <cmath>
#include <limits>
#include <QAbstractTableModel>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QMessageBox>
#include <QLineEdit>
#include <QSignalBlocker>
#include <QSortFilterProxyModel>
#include <QSet>
#include <QTableView>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>

#include "channel/channelwebapiutils.h"
#include "feature/featureset.h"
#include "feature/featureuiset.h"
#include "feature/featureutils.h"
#include "feature/featurewebapiutils.h"
#include "gui/basicfeaturesettingsdialog.h"
#include "gui/dialogpositioner.h"
#include "util/units.h"
#include "util/profiler.h"
#include "device/deviceapi.h"
#include "device/deviceset.h"
#include "maincore.h"

#include "ui_satellitetrackergui.h"
#include "satellitetracker.h"
#include "satellitetrackergui.h"
#include "satellitetrackerreport.h"
#include "satellitetrackersettingsdialog.h"
#include "satelliteselectiondialog.h"
#include "satelliteradiocontroldialog.h"
#include "satellitetrackersgp4.h"

namespace
{
    enum SatelliteTableRole {
        SortRole = Qt::UserRole + 1
    };

    struct SatelliteTableColumn {
        const char *m_title;
        const char *m_toolTip;
        const char *m_widthHint;
    };

    const SatelliteTableColumn satelliteTableColumns[SAT_COL_COLUMNS] = {
        {"Satellite", "Satellite name", "Satellite123"},
        {"Az", "Azimuth in degrees to satellite from antenna location", "360"},
        {"El", "Elevation in degrees to satellite from antenna location", "-90"},
        {"Next", "Time until next event", "99:99:99 AOS"},
        {"Dur", "Duration of next pass", "9:99:99"},
        {"AOS", "Time of next AOS (Acquisition of signal)", "+1 10:17"},
        {"LOS", "Time of next LOS (Loss of signal)", "+1 10:17"},
        {"Max El.", "Maximum elevation in degrees of next satellite pass", "90"},
        {"Dir", "Direction of the next pass", "^"},
        {"Latitude (\302\260)", "", "-90.0"},
        {"Longitude (\302\260)", "", "-180.0"},
        {"Alt (km)", "Satellite altitude in kilometres", "50000"},
        {"Range (km)", "Range to satellite in kilometres", "50000"},
        {"Range rate (km/s)", "Speed of satellite towards antenna location in kilometers per second", "10.0"},
        {"Doppler (Hz)", "Receive Doppler shift in Hertz (At frequency set in settings)", "10000"},
        {"Path loss (dB)", "Free space loss of signal in decibels (At frequency set in settings)", "100"},
        {"Delay (ms)", "Propagation delay of a signal from the antenna to the satellite in milliseconds (assuming line-of-sight)", "100"},
        {"Norad ID", "Norad catalog identifier for the satellite", "123456"},
        {"Error", "Details any errors calculating satellites position", "Decayed"}
    };

    QString formatDaysTimeForTable(qint64 days, const QDateTime& dateTime, bool utc, const QString& dateFormat)
    {
        QDateTime dt = utc ? dateTime.toUTC() : dateTime.toLocalTime();

        if (std::abs(days) > 10) {
            return dt.date().toString(dateFormat);
        } else if (days == 0) {
            return dt.time().toString("hh:mm");
        } else if (days > 0) {
            return dt.time().toString(QString("hh:mm +%1").arg(days));
        } else {
            return dt.time().toString(QString("hh:mm %1").arg(days));
        }
    }

    QString formatSecondsAsHHMMSSForTable(qint64 seconds)
    {
        char const* sign = "";

        if (seconds < 0)
        {
            sign = "-";
            seconds = -seconds;
        }

        int minutes = seconds / 60;
        seconds = seconds % 60;
        int hours = minutes / 60;
        minutes = minutes % 60;

        if (hours > 0) {
            return QString("%1%2:%3:%4").arg(sign).arg(hours).arg(minutes, 2, 10, QChar('0')).arg(seconds, 2, 10, QChar('0'));
        } else {
            return QString("%1%2:%3").arg(sign).arg(minutes).arg(seconds, 2, 10, QChar('0'));
        }
    }
}

#define SPEED_OF_LIGHT 299792458.0

// Frequency in Hz, speed in m/s
static double doppler(double frequency, double speed)
{
    return frequency * speed / SPEED_OF_LIGHT;
}

// Frequency in Hz, speed in m/s
static double freeSpaceLoss(double frequency, double distance)
{
    return 20.0 * log10(distance) + 20 * log10(frequency) + 20 * log10(4*M_PI/SPEED_OF_LIGHT);
}

// Distance in m, delay in s
static double propagationDelay(double distance)
{
    return distance / SPEED_OF_LIGHT;
}

class SatelliteTrackerGUI::SatelliteTableModel : public QAbstractTableModel
{
public:
    explicit SatelliteTableModel(QObject *parent = nullptr) :
        QAbstractTableModel(parent),
        m_utc(false),
        m_defaultFrequency(0.0)
    {
    }

    void configure(bool utc, const QString& dateFormat, double defaultFrequency)
    {
        m_utc = utc;
        m_dateFormat = dateFormat;
        m_defaultFrequency = defaultFrequency;
    }

    int rowCount(const QModelIndex& parent = QModelIndex()) const override
    {
        return parent.isValid() ? 0 : m_rows.size();
    }

    int columnCount(const QModelIndex& parent = QModelIndex()) const override
    {
        return parent.isValid() ? 0 : SAT_COL_COLUMNS;
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role) const override
    {
        if ((orientation != Qt::Horizontal) || (section < 0) || (section >= SAT_COL_COLUMNS)) {
            return QVariant();
        }

        if (role == Qt::DisplayRole) {
            return satelliteTableColumns[section].m_title;
        } else if (role == Qt::ToolTipRole) {
            return satelliteTableColumns[section].m_toolTip;
        } else {
            return QVariant();
        }
    }

    QVariant data(const QModelIndex& index, int role) const override
    {
        if (!index.isValid() || (index.row() < 0) || (index.row() >= m_rows.size())) {
            return QVariant();
        }

        const SatelliteTableRow& row = m_rows[index.row()];
        const int column = index.column();

        if (role == Qt::TextAlignmentRole)
        {
            switch (column)
            {
            case SAT_COL_AZ:
            case SAT_COL_EL:
            case SAT_COL_TNE:
            case SAT_COL_DUR:
            case SAT_COL_MAX_EL:
            case SAT_COL_LATITUDE:
            case SAT_COL_LONGITUDE:
            case SAT_COL_ALT:
            case SAT_COL_RANGE:
            case SAT_COL_RANGE_RATE:
            case SAT_COL_DOPPLER:
            case SAT_COL_PATH_LOSS:
            case SAT_COL_DELAY:
                return QVariant::fromValue(Qt::Alignment(Qt::AlignRight | Qt::AlignVCenter));
            default:
                return QVariant();
            }
        }

        if (role == SortRole) {
            return sortData(row, column);
        }

        if (role != Qt::DisplayRole) {
            return QVariant();
        }

        switch (column)
        {
        case SAT_COL_NAME:
            return row.m_name;
        case SAT_COL_AZ:
            return row.m_hasPosition ? QVariant(row.m_azimuth) : QVariant();
        case SAT_COL_EL:
            return row.m_hasPosition ? QVariant(row.m_elevation) : QVariant();
        case SAT_COL_TNE:
            return row.m_hasPosition ? row.m_nextEvent : QString();
        case SAT_COL_DUR:
            return row.m_hasPosition ? row.m_duration : QString();
        case SAT_COL_AOS:
            return row.m_hasPosition ? row.m_aos : QString();
        case SAT_COL_LOS:
            return row.m_hasPosition ? row.m_los : QString();
        case SAT_COL_MAX_EL:
            return row.m_hasMaxElevation ? QVariant(row.m_maxElevation) : QVariant();
        case SAT_COL_DIR:
            return row.m_hasPosition ? row.m_direction : QString();
        case SAT_COL_LATITUDE:
            return row.m_hasPosition ? QVariant(row.m_latitude) : QVariant();
        case SAT_COL_LONGITUDE:
            return row.m_hasPosition ? QVariant(row.m_longitude) : QVariant();
        case SAT_COL_ALT:
            return row.m_hasPosition ? QVariant(row.m_altitude) : QVariant();
        case SAT_COL_RANGE:
            return row.m_hasPosition ? QVariant(row.m_range) : QVariant();
        case SAT_COL_RANGE_RATE:
            return row.m_hasPosition ? row.m_rangeRate : QString();
        case SAT_COL_DOPPLER:
            return row.m_hasPosition ? QVariant(row.m_doppler) : QVariant();
        case SAT_COL_PATH_LOSS:
            return row.m_hasPosition ? row.m_pathLoss : QString();
        case SAT_COL_DELAY:
            return row.m_hasPosition ? row.m_delay : QString();
        case SAT_COL_NORAD_ID:
            return row.m_hasNoradId ? QVariant(row.m_noradId) : QVariant();
        case SAT_COL_ERROR:
            return row.m_error;
        default:
            return QVariant();
        }
    }

    QString satelliteNameAt(int row) const
    {
        if ((row < 0) || (row >= m_rows.size())) {
            return QString();
        }

        return m_rows[row].m_name;
    }

    void removeSatellitesNotIn(const QList<QString>& selectedSatellites)
    {
        QSet<QString> selectedSatelliteSet;
        selectedSatelliteSet.reserve(selectedSatellites.size());

        for (const QString& satellite : selectedSatellites) {
            selectedSatelliteSet.insert(satellite);
        }

        for (int row = m_rows.size() - 1; row >= 0; )
        {
            if (selectedSatelliteSet.contains(m_rows[row].m_name))
            {
                row--;
                continue;
            }

            int firstRow = row;

            while ((firstRow > 0) && !selectedSatelliteSet.contains(m_rows[firstRow - 1].m_name)) {
                firstRow--;
            }

            beginRemoveRows(QModelIndex(), firstRow, row);
            m_rows.remove(firstRow, row - firstRow + 1);
            endRemoveRows();

            row = firstRow - 1;
        }

        rebuildRowIndex();
    }

    void updateSatellite(const SatelliteState& satState, const SatNogsSatellite *satellite, const QDateTime& currentDateTime)
    {
        SatelliteTableRow row = makeRow(satState, satellite, currentDateTime);
        auto it = m_rowByName.find(row.m_name);

        if (it == m_rowByName.end())
        {
            const int newRow = m_rows.size();
            beginInsertRows(QModelIndex(), newRow, newRow);
            m_rows.append(row);
            m_rowByName.insert(row.m_name, newRow);
            endInsertRows();
        }
        else
        {
            const int existingRow = it.value();
            m_rows[existingRow] = row;
            emit dataChanged(index(existingRow, 0), index(existingRow, SAT_COL_COLUMNS - 1),
                {Qt::DisplayRole, Qt::TextAlignmentRole, SortRole});
        }
    }

private:
    void rebuildRowIndex()
    {
        m_rowByName.clear();

        for (int i = 0; i < m_rows.size(); ++i) {
            m_rowByName.insert(m_rows[i].m_name, i);
        }
    }

    struct SatelliteTableRow
    {
        QString m_name;
        bool m_hasPosition = false;
        bool m_hasPass = false;
        bool m_hasMaxElevation = false;
        bool m_hasNoradId = false;
        int m_azimuth = 0;
        int m_elevation = 0;
        QString m_nextEvent;
        qint64 m_nextEventSeconds = std::numeric_limits<qint64>::max();
        QString m_duration;
        qint64 m_durationSeconds = std::numeric_limits<qint64>::max();
        QString m_aos;
        QDateTime m_aosSort;
        QString m_los;
        QDateTime m_losSort;
        int m_maxElevation = 0;
        QString m_direction;
        double m_latitude = 0.0;
        double m_longitude = 0.0;
        int m_altitude = 0;
        int m_range = 0;
        QString m_rangeRate;
        double m_rangeRateSort = 0.0;
        int m_doppler = 0;
        QString m_pathLoss;
        double m_pathLossSort = 0.0;
        QString m_delay;
        double m_delaySort = 0.0;
        int m_noradId = 0;
        QString m_error;
    };

    SatelliteTableRow makeRow(const SatelliteState& satState, const SatNogsSatellite *satellite, const QDateTime& currentDateTime) const
    {
        SatelliteTableRow row;
        row.m_name = satState.m_name;

        if (satellite != nullptr)
        {
            row.m_hasNoradId = true;
            row.m_noradId = satellite->m_noradCatId;
        }

        row.m_error = satState.m_error;

        if (!satState.m_error.isEmpty()) {
            return row;
        }

        row.m_hasPosition = true;
        row.m_azimuth = (int) round(satState.m_azimuth);
        row.m_elevation = (int) round(satState.m_elevation);

        if (!satState.m_passes.isEmpty())
        {
            const SatellitePass& pass = satState.m_passes[0];
            const qint64 secondsToAOS = currentDateTime.secsTo(pass.m_aos);
            const qint64 secondsToLOS = currentDateTime.secsTo(pass.m_los);
            const int daysToAOS = currentDateTime.daysTo(pass.m_aos);
            const int daysToLOS = currentDateTime.daysTo(pass.m_los);

            row.m_hasPass = true;
            row.m_hasMaxElevation = true;

            if (pass.m_aos > currentDateTime)
            {
                row.m_nextEventSeconds = secondsToAOS;
                row.m_nextEvent = formatSecondsAsHHMMSSForTable(secondsToAOS) + " AOS";
            }
            else
            {
                row.m_nextEventSeconds = secondsToLOS;
                row.m_nextEvent = formatSecondsAsHHMMSSForTable(secondsToLOS) + " LOS";
            }

            row.m_durationSeconds = pass.m_aos.secsTo(pass.m_los);
            row.m_duration = formatSecondsAsHHMMSSForTable(row.m_durationSeconds);
            row.m_aos = formatDaysTimeForTable(daysToAOS, pass.m_aos, m_utc, m_dateFormat);
            row.m_aosSort = pass.m_aos;
            row.m_los = formatDaysTimeForTable(daysToLOS, pass.m_los, m_utc, m_dateFormat);
            row.m_losSort = pass.m_los;
            row.m_maxElevation = (int) round(pass.m_maxElevation);
            row.m_direction = pass.m_northToSouth ? QString(QChar(0x2193)) : QString(QChar(0x2191));
        }

        row.m_latitude = satState.m_latitude;
        row.m_longitude = satState.m_longitude;
        row.m_altitude = (int) round(satState.m_altitude);
        row.m_range = (int) round(satState.m_range);
        row.m_rangeRateSort = satState.m_rangeRate;
        row.m_rangeRate = QString::number(satState.m_rangeRate, 'f', 3);
        row.m_doppler = (int) round(-doppler(m_defaultFrequency, satState.m_rangeRate * 1000.0));
        row.m_pathLossSort = freeSpaceLoss(m_defaultFrequency, satState.m_range * 1000.0);
        row.m_pathLoss = QString::number(row.m_pathLossSort, 'f', 1);
        row.m_delaySort = propagationDelay(satState.m_range * 1000.0) * 1000.0;
        row.m_delay = QString::number(row.m_delaySort, 'f', 1);

        return row;
    }

    QVariant sortData(const SatelliteTableRow& row, int column) const
    {
        if ((column != SAT_COL_NAME) && (column != SAT_COL_NORAD_ID) && (column != SAT_COL_ERROR) && !row.m_hasPosition) {
            return QVariant();
        }

        switch (column)
        {
        case SAT_COL_NAME:
            return row.m_name;
        case SAT_COL_AZ:
            return row.m_azimuth;
        case SAT_COL_EL:
            return row.m_elevation;
        case SAT_COL_TNE:
            return row.m_hasPass ? QVariant(row.m_nextEventSeconds) : QVariant();
        case SAT_COL_DUR:
            return row.m_hasPass ? QVariant(row.m_durationSeconds) : QVariant();
        case SAT_COL_AOS:
            return row.m_hasPass ? QVariant(row.m_aosSort) : QVariant();
        case SAT_COL_LOS:
            return row.m_hasPass ? QVariant(row.m_losSort) : QVariant();
        case SAT_COL_MAX_EL:
            return row.m_hasMaxElevation ? QVariant(row.m_maxElevation) : QVariant();
        case SAT_COL_DIR:
            return row.m_direction;
        case SAT_COL_LATITUDE:
            return row.m_latitude;
        case SAT_COL_LONGITUDE:
            return row.m_longitude;
        case SAT_COL_ALT:
            return row.m_altitude;
        case SAT_COL_RANGE:
            return row.m_range;
        case SAT_COL_RANGE_RATE:
            return row.m_rangeRateSort;
        case SAT_COL_DOPPLER:
            return row.m_doppler;
        case SAT_COL_PATH_LOSS:
            return row.m_pathLossSort;
        case SAT_COL_DELAY:
            return row.m_delaySort;
        case SAT_COL_NORAD_ID:
            return row.m_hasNoradId ? QVariant(row.m_noradId) : QVariant();
        case SAT_COL_ERROR:
            return row.m_error;
        default:
            return QVariant();
        }
    }

    QVector<SatelliteTableRow> m_rows;
    QHash<QString, int> m_rowByName;
    bool m_utc;
    QString m_dateFormat;
    double m_defaultFrequency;
};

class SatelliteTrackerGUI::SatelliteTableSortProxy : public QSortFilterProxyModel
{
public:
    explicit SatelliteTableSortProxy(QObject *parent = nullptr) :
        QSortFilterProxyModel(parent)
    {
    }

protected:
    bool lessThan(const QModelIndex& sourceLeft, const QModelIndex& sourceRight) const override
    {
        const QVariant left = sourceModel()->data(sourceLeft, SortRole);
        const QVariant right = sourceModel()->data(sourceRight, SortRole);

        if (!left.isValid() || !right.isValid()) {
            return left.isValid() && !right.isValid();
        }

        switch (sourceLeft.column())
        {
        case SAT_COL_NAME:
        case SAT_COL_DIR:
        case SAT_COL_ERROR:
            return QString::localeAwareCompare(left.toString(), right.toString()) < 0;
        case SAT_COL_AOS:
        case SAT_COL_LOS:
            return left.toDateTime() < right.toDateTime();
        default:
            return left.toDouble() < right.toDouble();
        }
    }
};

SatelliteTrackerGUI* SatelliteTrackerGUI::create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature)
{
    SatelliteTrackerGUI* gui = new SatelliteTrackerGUI(pluginAPI, featureUISet, feature);
    return gui;
}

void SatelliteTrackerGUI::destroy()
{
    delete this;
}

void SatelliteTrackerGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applySettings(true);
}

QByteArray SatelliteTrackerGUI::serialize() const
{
    return m_settings.serialize();
}

bool SatelliteTrackerGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        m_feature->setWorkspaceIndex(m_settings.m_workspaceIndex);
        updateSelectedSats();
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

QString SatelliteTrackerGUI::convertDegreesToText(double degrees)
{
    if (m_settings.m_azElUnits == SatelliteTrackerSettings::DMS)
        return Units::decimalDegreesToDegreeMinutesAndSeconds(degrees);
    else if (m_settings.m_azElUnits == SatelliteTrackerSettings::DM)
        return Units::decimalDegreesToDegreesAndMinutes(degrees);
    else if (m_settings.m_azElUnits == SatelliteTrackerSettings::D)
        return Units::decimalDegreesToDegrees(degrees);
    else
        return QString("%1").arg(degrees, 0, 'f', 2);
}

bool SatelliteTrackerGUI::handleMessage(const Message& message)
{
    if (SatelliteTracker::MsgConfigureSatelliteTracker::match(message))
    {
        qDebug("SatelliteTrackerGUI::handleMessage: SatelliteTracker::MsgConfigureSatelliteTracker");
        const SatelliteTracker::MsgConfigureSatelliteTracker& cfg = (SatelliteTracker::MsgConfigureSatelliteTracker&) message;

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
    else if (SatelliteTrackerReport::MsgReportSat::match(message))
    {
        PROFILER_START();
        SatelliteTrackerReport::MsgReportSat& satReport = (SatelliteTrackerReport::MsgReportSat&) message;
        const QList<SatelliteState>& satStates = satReport.getSatelliteStates();
        QSet<QString> selectedSatelliteSet;
        selectedSatelliteSet.reserve(m_settings.m_satellites.size());

        for (const QString& satellite : m_settings.m_satellites) {
            selectedSatelliteSet.insert(satellite);
        }

        const QDateTime currentDateTime = m_satelliteTracker->currentDateTime();
        ui->satTable->setUpdatesEnabled(false);
        m_satTableProxy->setDynamicSortFilter(false);

        for (const SatelliteState& satState : satStates)
        {
            if (!selectedSatelliteSet.contains(satState.m_name)) {
                continue;
            }

            if (satState.m_name == m_settings.m_target)
            {
                delete m_targetSatState;
                m_targetSatState = new SatelliteState(satState);

                ui->azimuth->setText(convertDegreesToText(satState.m_azimuth));
                ui->elevation->setText(convertDegreesToText(satState.m_elevation));
                plotChart();

                if (satState.m_passes.size() > 0)
                {
                    const SatellitePass &pass = satState.m_passes[0];
                    bool geostationary = !pass.m_aos.isValid() && !pass.m_los.isValid();

                    if ((m_nextTargetAOS != pass.m_aos) || (m_nextTargetLOS != pass.m_los) || (geostationary != m_geostationarySatVisible))
                    {
                        m_nextTargetAOS = pass.m_aos;
                        m_nextTargetLOS = pass.m_los;
                        m_geostationarySatVisible = geostationary;
                        updateTimeToAOS();
                    }
                }
            }

            updateTable(&satState, currentDateTime);
        }

        m_satTableProxy->setDynamicSortFilter(true);
        if (m_settings.m_columnSort >= 0) {
            m_satTableProxy->sort(m_settings.m_columnSort, m_settings.m_columnSortOrder);
        }
        ui->satTable->setUpdatesEnabled(true);

        PROFILER_STOP("SatelliteTrackerGUI::handleMessage::MsgReportSat");
        return true;
    }
    else if (SatelliteTrackerReport::MsgReportAOS::match(message))
    {
        SatelliteTrackerReport::MsgReportAOS& aosReport = (SatelliteTrackerReport::MsgReportAOS&) message;
        aos(aosReport.getName(), aosReport.getSpeech());
        return true;
    }
    else if (SatelliteTrackerReport::MsgReportTarget::match(message))
    {
        SatelliteTrackerReport::MsgReportTarget& targetReport = (SatelliteTrackerReport::MsgReportTarget&) message;
        setTarget(targetReport.getName());
        return true;
    }
    else if (SatelliteTrackerReport::MsgReportLOS::match(message))
    {
        SatelliteTrackerReport::MsgReportLOS& losReport = (SatelliteTrackerReport::MsgReportLOS&) message;
        los(losReport.getSpeech());
        return true;
    }
    else if (SatelliteTracker::MsgSatData::match(message))
    {
        SatelliteTracker::MsgSatData& satData = (SatelliteTracker::MsgSatData&) message;
        m_satellites = satData.getSatellites();
        // Remove satellites that no longer exist
        QMutableListIterator<QString> itr(m_settings.m_satellites);

        while (itr.hasNext())
        {
            QString satellite = itr.next();
            if (!m_satellites.contains(satellite))
                itr.remove();
        }

        if (!m_satellites.contains(m_settings.m_target)) {
            setTarget("");
        }

        // Update GUI
        updateSelectedSats();

        return true;
    }
    else if (SatelliteTracker::MsgError::match(message))
    {
        SatelliteTracker::MsgError& errorMsg = (SatelliteTracker::MsgError&) message;
        QString error = errorMsg.getError();
        QMessageBox::critical(this, "Satellite Tracker", error);
        return true;
    }

    return false;
}

// Call when m_settings.m_satellites changes
void SatelliteTrackerGUI::updateSelectedSats()
{
    // Remove unselected satellites from target combo and table.
    m_satTableModel->removeSatellitesNotIn(m_settings.m_satellites);

    for (int i = 0; i < ui->target->count(); )
    {
        QString name = ui->target->itemText(i);
        int idx = m_settings.m_satellites.indexOf(name);

        if (idx == -1) {
            ui->target->removeItem(i);
        } else {
            i++;
        }
    }

    // Add new satellites to target combo
    for (int i = 0; i < m_settings.m_satellites.size(); i++)
    {
        if (ui->target->findText(m_settings.m_satellites[i], Qt::MatchExactly) == -1) {
            ui->target->addItem(m_settings.m_satellites[i]);
        }
    }
    // Select current target, if it still exists
    int idx = ui->target->findText(m_settings.m_target);

    if (idx != -1) {
        ui->target->setCurrentIndex(idx);
    } else {
        setTarget("");
    }
}

void SatelliteTrackerGUI::handleInputMessages()
{
    Message* message;

    while ((message = getInputMessageQueue()->pop()))
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void SatelliteTrackerGUI::onWidgetRolled(QWidget* widget, bool rollDown)
{
    (void) widget;
    (void) rollDown;

    RollupContents *rollupContents = getRollupContents();
    rollupContents->saveState(m_rollupState);
}

SatelliteTrackerGUI::SatelliteTrackerGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent) :
    FeatureGUI(parent),
    ui(new Ui::SatelliteTrackerGUI),
    m_pluginAPI(pluginAPI),
    m_featureUISet(featureUISet),
    m_doApplySettings(true),
    m_lastFeatureState(0),
    m_lastUpdatingSatData(false),
    m_targetSatState(nullptr),
    m_satTableModel(nullptr),
    m_satTableProxy(nullptr),
    m_plotPass(0),
    m_lineChart(nullptr),
    m_polarChart(nullptr),
    m_geostationarySatVisible(false)
{
    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/satellitetracker/readme.md";
    RollupContents *rollupContents = getRollupContents();
	ui->setupUi(rollupContents);
    rollupContents->arrangeRollups();
	connect(rollupContents, SIGNAL(widgetRolled(QWidget*,bool)), this, SLOT(onWidgetRolled(QWidget*,bool)));

    m_satelliteTracker = reinterpret_cast<SatelliteTracker*>(feature);
    m_satelliteTracker->setMessageQueueToGUI(&m_inputMessageQueue);

    m_settings.setRollupState(&m_rollupState);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    connect(&m_statusTimer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    m_statusTimer.start(1000);

    connect(&m_redrawTimer, &QTimer::timeout, this, &SatelliteTrackerGUI::plotChart);

    // Initialise charts
    m_emptyChart.layout()->setContentsMargins(0, 0, 0, 0);
    m_emptyChart.setMargins(QMargins(1, 1, 1, 1));
    ui->passChart->setChart(&m_emptyChart);
    ui->passChart->setRenderHint(QPainter::Antialiasing);

    ui->dateTime->setDateTime(m_satelliteTracker->currentDateTime());
    ui->deviceFeatureSelect->setVisible(false);

    m_satTableModel = new SatelliteTableModel(this);
    m_satTableModel->configure(m_settings.m_utc, m_settings.m_dateFormat, m_settings.m_defaultFrequency);
    m_satTableProxy = new SatelliteTableSortProxy(this);
    m_satTableProxy->setSourceModel(m_satTableModel);
    m_satTableProxy->setDynamicSortFilter(true);
    ui->satTable->setModel(m_satTableProxy);
    ui->satTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->satTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
    // Allow user to reorder columns
    ui->satTable->horizontalHeader()->setSectionsMovable(true);
    // Allow user to sort table by clicking on headers
    ui->satTable->setSortingEnabled(true);
    resizeTable();
    // Add context menu to allow hiding/showing of columns
    menu = new QMenu(ui->satTable);

    for (int i = 0; i < m_satTableModel->columnCount(); i++)
    {
        QString text = m_satTableModel->headerData(i, Qt::Horizontal, Qt::DisplayRole).toString();
        menu->addAction(createCheckableItem(text, i, true));
    }

    ui->satTable->horizontalHeader()->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->satTable->horizontalHeader(), SIGNAL(customContextMenuRequested(QPoint)), SLOT(columnSelectMenu(QPoint)));
    ui->satTable->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->satTable, SIGNAL(customContextMenuRequested(QPoint)), SLOT(satTableMenu(QPoint)));
    // Get signals when columns change
    connect(ui->satTable->horizontalHeader(), SIGNAL(sectionMoved(int, int, int)), SLOT(satTable_sectionMoved(int, int, int)));
    connect(ui->satTable->horizontalHeader(), SIGNAL(sectionResized(int, int, int)), SLOT(satTable_sectionResized(int, int, int)));

#ifdef QT_TEXTTOSPEECH_FOUND
    m_speech = new QTextToSpeech(this);
#endif

    displaySettings();
    applySettings(true);
    makeUIConnections();
    m_resizer.enableChildMouseTracking();

    // Get initial list of satellites
    on_updateSatData_clicked();

    // Use My Position from preferences, if none set
    if ((m_settings.m_latitude == 0.0) && (m_settings.m_longitude == 0.0)) {
        on_useMyPosition_clicked();
    }
}

SatelliteTrackerGUI::~SatelliteTrackerGUI()
{
    delete m_targetSatState;
    delete ui;
}

void SatelliteTrackerGUI::setWorkspaceIndex(int index)
{
    m_settings.m_workspaceIndex = index;
    m_settingsKeys.append("workspaceIndex");
    m_feature->setWorkspaceIndex(index);
}

void SatelliteTrackerGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void SatelliteTrackerGUI::displaySettings()
{
    setTitleColor(m_settings.m_rgbColor);
    setWindowTitle(m_settings.m_title);
    setTitle(m_settings.m_title);
    blockApplySettings(true);
    ui->latitude->setValue(m_settings.m_latitude);
    ui->longitude->setValue(m_settings.m_longitude);

    ui->target->blockSignals(true);
    ui->target->clear();
    for (const QString& s : m_settings.m_satellites) {
        ui->target->addItem(s);
    }
    ui->target->blockSignals(false);
    ui->target->setCurrentIndex(ui->target->findText(m_settings.m_target));

    ui->dateTime->setDateTime(QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs));
    ui->dateTimeSelect->setCurrentIndex((int)m_settings.m_dateTimeSelect);
    ui->dateTime->setVisible(m_settings.m_dateTimeSelect == SatelliteTrackerSettings::CUSTOM);
    ui->autoTarget->setChecked(m_settings.m_autoTarget);
    ui->darkTheme->setChecked(m_settings.m_chartsDarkTheme);
    m_satTableModel->configure(m_settings.m_utc, m_settings.m_dateFormat, m_settings.m_defaultFrequency);
    if (m_settings.m_columnSort >= 0)
    {
        ui->satTable->horizontalHeader()->setSortIndicator(m_settings.m_columnSort, m_settings.m_columnSortOrder);
        m_satTableProxy->sort(m_settings.m_columnSort, m_settings.m_columnSortOrder);
    }
    getRollupContents()->restoreState(m_rollupState);
    plotChart();
    blockApplySettings(false);
}

void SatelliteTrackerGUI::onMenuDialogCalled(const QPoint &p)
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
        m_settingsKeys.append("rgbColor");
        m_settingsKeys.append("useReverseAPI");
        m_settingsKeys.append("reverseAPIAddress");
        m_settingsKeys.append("reverseAPIPort");
        m_settingsKeys.append("reverseAPIFeatureSetIndex");
        m_settingsKeys.append("reverseAPIFeatureIndex");

        applySettings();
    }

    resetContextMenuType();
}

void SatelliteTrackerGUI::aos(const QString &satName, const QString &speech)
{
    if (satName == m_settings.m_target)
    {
        // Call plotChart() to start the periodic updates with sat position in polar chart
        plotChart();
    }
    // Give speech notification of pass
    if (!speech.isEmpty()) 
    {
#ifdef QT_TEXTTOSPEECH_FOUND
        m_speech->say(speech);
#else
        qWarning() << "SatelliteTrackerGUI::aos: No TextToSpeech: " << speech;
#endif
    }
}

void SatelliteTrackerGUI::los(const QString &speech)
{
    // Give speech notification of end of pass
    if (!speech.isEmpty()) 
    {
#ifdef QT_TEXTTOSPEECH_FOUND
        m_speech->say(speech);
#else
        qWarning() << "SatelliteTrackerGUI::los: No TextToSpeech: " << speech;
#endif
    }
}

void SatelliteTrackerGUI::on_startStop_toggled(bool checked)
{
    if (m_doApplySettings)
    {
        SatelliteTracker::MsgStartStop *message = SatelliteTracker::MsgStartStop::create(checked);
        m_satelliteTracker->getInputMessageQueue()->push(message);
    }
}

void SatelliteTrackerGUI::on_latitude_valueChanged(double value)
{
    m_settings.m_latitude = value;
    m_settingsKeys.append("latitude");
    applySettings();
    plotChart();
}

void SatelliteTrackerGUI::on_longitude_valueChanged(double value)
{
    m_settings.m_longitude = value;
    m_settingsKeys.append("longitude");
    applySettings();
    plotChart();
}

void SatelliteTrackerGUI::setTarget(const QString& target)
{
    if (target != m_settings.m_target)
    {
        m_settings.m_target = target;
        m_settingsKeys.append("target");
        ui->azimuth->setText("");
        ui->elevation->setText("");
        ui->aos->setText("");
        ui->target->setCurrentIndex(ui->target->findText(m_settings.m_target));
        m_nextTargetAOS = QDateTime();
        m_nextTargetLOS = QDateTime();
        m_geostationarySatVisible = false;
        applySettings();
        delete m_targetSatState;
        m_targetSatState = nullptr;
        m_plotPass = 0;
        ui->passLabel->setText(QString("%1").arg(m_plotPass));
        plotChart();
    }
}

void SatelliteTrackerGUI::on_target_currentTextChanged(const QString &text)
{
    setTarget(text);
}

void SatelliteTrackerGUI::on_useMyPosition_clicked(bool checked)
{
    (void) checked;
    double stationLatitude = MainCore::instance()->getSettings().getLatitude();
    double stationLongitude = MainCore::instance()->getSettings().getLongitude();
    double stationAltitude = MainCore::instance()->getSettings().getAltitude();

    ui->latitude->setValue(stationLatitude);
    ui->longitude->setValue(stationLongitude);
    m_settings.m_heightAboveSeaLevel = stationAltitude;
    m_settingsKeys.append("heightAboveSeaLevel");
    applySettings();
    plotChart();
}

// Show settings dialog
void SatelliteTrackerGUI::on_displaySettings_clicked()
{
    SatelliteTrackerSettingsDialog dialog(&m_settings);
    new DialogPositioner(&dialog, true);
    if (dialog.exec() == QDialog::Accepted)
    {
        m_settingsKeys.append("heightAboveSeaLevel");
        m_settingsKeys.append("predictionPeriod");
        m_settingsKeys.append("passStartTime");
        m_settingsKeys.append("passFinishTime");
        m_settingsKeys.append("minAOSElevation");
        m_settingsKeys.append("minPassElevation");
        m_settingsKeys.append("rotatorMaxAzimuth");
        m_settingsKeys.append("rotatorMaxElevation");
        m_settingsKeys.append("azimuthOffset");
        m_settingsKeys.append("elevationOffset");
        m_settingsKeys.append("aosSpeech");
        m_settingsKeys.append("losSpeech");
        m_settingsKeys.append("aosCommand");
        m_settingsKeys.append("losCommand");
        m_settingsKeys.append("updatePeriod");
        m_settingsKeys.append("dopplerPeriod");
        m_settingsKeys.append("defaultFrequency");
        m_settingsKeys.append("azElUnits");
        m_settingsKeys.append("groundTrackPoints");
        m_settingsKeys.append("drawRotators");
        m_settingsKeys.append("dateFormat");
        m_settingsKeys.append("utc");
        m_settingsKeys.append("drawOnMap");
        m_settingsKeys.append("tles");
        m_settingsKeys.append("replayEnabled");
        m_settingsKeys.append("replayStartDateTime");
        m_settingsKeys.append("sendTimeToMap");
        applySettings();
        plotChart();
    }
}

void SatelliteTrackerGUI::on_dateTimeSelect_currentIndexChanged(int index)
{
    m_settings.m_dateTimeSelect = (SatelliteTrackerSettings::DateTimeSelect)index;

    if (m_settings.m_dateTimeSelect != SatelliteTrackerSettings::CUSTOM)
    {
        m_settings.m_dateTime = "";
        ui->dateTime->setVisible(false);
    }
    else
    {
        m_settings.m_dateTime = ui->dateTime->dateTime().toString(Qt::ISODateWithMs);
        ui->dateTime->setVisible(true);
    }

    ui->deviceFeatureSelect->setVisible(m_settings.m_dateTimeSelect >= SatelliteTrackerSettings::FROM_MAP);
    updateDeviceFeatureCombo();

    m_settingsKeys.append("dateTimeSelect");
    m_settingsKeys.append("dateTime");

    applySettings();
    plotChart();
}

void SatelliteTrackerGUI::on_dateTime_dateTimeChanged(const QDateTime &datetime)
{
    (void) datetime;

    if (ui->dateTimeSelect->currentIndex() == 1)
    {
        m_settings.m_dateTime = ui->dateTime->dateTime().toString(Qt::ISODateWithMs);
        m_settingsKeys.append("dateTime");
        applySettings();
        plotChart();
    }
}

// Find target on the Map
void SatelliteTrackerGUI::on_viewOnMap_clicked()
{
    if (!m_settings.m_target.isEmpty()) {
        FeatureWebAPIUtils::mapFind(m_settings.m_target);
    }
}

void SatelliteTrackerGUI::on_updateSatData_clicked()
{
    m_satelliteTracker->getInputMessageQueue()->push(SatelliteTracker::MsgUpdateSatData::create());
}

void SatelliteTrackerGUI::on_selectSats_clicked()
{
    SatelliteSelectionDialog dialog(&m_settings, m_satellites);
    new DialogPositioner(&dialog, true);
    if (dialog.exec() == QDialog::Accepted)
    {
        updateSelectedSats();
        m_settingsKeys.append("satellites");
        applySettings();
    }
}

void SatelliteTrackerGUI::on_radioControl_clicked()
{
    SatelliteRadioControlDialog dialog(&m_settings, m_satellites);
    new DialogPositioner(&dialog, true);
    if (dialog.exec() == QDialog::Accepted)
    {
        m_settingsKeys.append("deviceSettings");
        applySettings();
    }
}

void SatelliteTrackerGUI::on_autoTarget_clicked(bool checked)
{
    m_settings.m_autoTarget = checked;
    m_settingsKeys.append("autoTarget");
    applySettings();
}

void SatelliteTrackerGUI::updateStatus()
{
    int state = m_satelliteTracker->getState();

    if (m_lastFeatureState != state)
    {
        // We set checked state of start/stop button, in case it was changed via API
        bool oldState;
        switch (state)
        {
            case Feature::StNotStarted:
                ui->startStop->setStyleSheet("QToolButton { background:rgb(79,79,79); }");
                break;
            case Feature::StIdle:
                oldState = ui->startStop->blockSignals(true);
                ui->startStop->setChecked(false);
                ui->startStop->blockSignals(oldState);
                ui->startStop->setStyleSheet("QToolButton { background-color : blue; }");
                break;
            case Feature::StRunning:
                oldState = ui->startStop->blockSignals(true);
                ui->startStop->setChecked(true);
                ui->startStop->blockSignals(oldState);
                ui->startStop->setStyleSheet("QToolButton { background-color : green; }");
                break;
            case Feature::StError:
                ui->startStop->setStyleSheet("QToolButton { background-color : red; }");
                QMessageBox::information(this, tr("Message"), m_satelliteTracker->getErrorMessage());
                break;
            default:
                break;
        }

        m_lastFeatureState = state;
    }

    // Indicate if satellite data is being updated
    bool updatingSatData = m_satelliteTracker->isUpdatingSatData();

    if (updatingSatData != m_lastUpdatingSatData)
    {
        if (updatingSatData) {
            ui->updateSatData->setStyleSheet("QToolButton { background-color : green; }");
        } else {
            ui->updateSatData->setStyleSheet("QToolButton { background: none; }");
        }

        m_lastUpdatingSatData = updatingSatData;
    }

    updateTimeToAOS();
    updateDeviceFeatureCombo();
}

// Update time to AOS
void SatelliteTrackerGUI::updateTimeToAOS()
{
    if (m_geostationarySatVisible)
    {
        ui->aos->setText("Now");
    }
    else if (m_nextTargetAOS.isValid())
    {
        QDateTime currentTime = m_satelliteTracker->currentDateTime();            // FIXME: UTC
        int secondsToAOS = m_nextTargetAOS.toSecsSinceEpoch() - currentTime.toSecsSinceEpoch();

        if (secondsToAOS > 0)
        {
            int seconds = secondsToAOS % 60;
            int minutes = (secondsToAOS / 60) % 60;
            int hours = (secondsToAOS / (60 * 60)) % 24;
            int days = secondsToAOS / (60 * 60 * 24);

            if (days == 1)
            {
                ui->aos->setText(QString("1 day"));
            }
            else if (days > 0)
            {
                ui->aos->setText(QString("%1 days").arg(days));
            }
            else
            {
                ui->aos->setText(QString("%1:%2:%3")
                    .arg(hours, 2, 10, QLatin1Char('0'))
                    .arg(minutes, 2, 10, QLatin1Char('0'))
                    .arg(seconds, 2, 10, QLatin1Char('0')));
            }
        }
        else if (m_nextTargetLOS < currentTime)
        {
            ui->aos->setText("");
        }
        else
        {
            ui->aos->setText("Now");
        }
    }
    else
        ui->aos->setText("");
}

void SatelliteTrackerGUI::applySettings(bool force)
{
    if (m_doApplySettings)
    {
        SatelliteTracker::MsgConfigureSatelliteTracker* message = SatelliteTracker::MsgConfigureSatelliteTracker::create(
            m_settings, m_settingsKeys, force);
        m_satelliteTracker->getInputMessageQueue()->push(message);
    }

    m_settingsKeys.clear();
}

void SatelliteTrackerGUI::on_nextPass_clicked()
{
    if (m_targetSatState != nullptr)
    {
        if (m_plotPass < m_targetSatState->m_passes.size() - 1)
        {
            m_plotPass++;
            ui->passLabel->setText(QString("%1").arg(m_plotPass));
            plotChart();
        }
    }
}

void SatelliteTrackerGUI::on_prevPass_clicked()
{
    if (m_plotPass > 0)
    {
        m_plotPass--;
        ui->passLabel->setText(QString("%1").arg(m_plotPass));
        plotChart();
    }
}

void SatelliteTrackerGUI::on_darkTheme_clicked(bool checked)
{
    m_settings.m_chartsDarkTheme = checked;
    plotChart();
    m_settingsKeys.append("chartsDarkTheme");
    applySettings();
}

void SatelliteTrackerGUI::on_chartSelect_currentIndexChanged(int index)
{
    (void) index;
    plotChart();
}

void SatelliteTrackerGUI::plotChart()
{
    if (ui->chartSelect->currentIndex() == 0) {
        plotPolarChart();
    } else {
        plotAzElChart();
    }
}

// Linear interpolation
static double interpolate(double x0, double y0, double x1, double y1, double x)
{
    return (y0*(x1-x) + y1*(x-x0)) / (x1-x0);
}

// Reduce az/el range from 450,180 to 360,90
void SatelliteTrackerGUI::limitAzElRange(double& azimuth, double& elevation) const
{
    if (elevation > 90.0)
    {
        elevation = 180.0 - elevation;
        if (azimuth < 180.0) {
            azimuth += 180.0;
        } else {
            azimuth -= 180.0;
        }
    }
    if (azimuth > 360.0) {
        azimuth -= 360.0f;
    }
    if (azimuth == 0) {
        azimuth = 360.0;
    }
}

// Plot pass in polar coords
void SatelliteTrackerGUI::plotPolarChart()
{
    if ((m_targetSatState == nullptr) || !m_satellites.contains(m_settings.m_target) || (m_targetSatState->m_passes.size() == 0))
    {
        ui->passChart->setChart(&m_emptyChart);
        return;
    }

    QChart *oldChart = m_polarChart;

    if (m_plotPass >= m_targetSatState->m_passes.size() - 1)
    {
        m_plotPass = m_targetSatState->m_passes.size() - 1;
        ui->passLabel->setText(QString("%1").arg(m_plotPass));
    }

    const SatellitePass &pass = m_targetSatState->m_passes[m_plotPass];

    // Always create a new chart, otherwise sometimes they aren't drawn properly
    m_polarChart = new QPolarChart();
    m_polarChart->setTheme(m_settings.m_chartsDarkTheme ? QChart::ChartThemeDark : QChart::ChartThemeLight);
    QValueAxis *angularAxis = new QValueAxis();
    QCategoryAxis *radialAxis = new QCategoryAxis();

    angularAxis->setTickCount(9);
    angularAxis->setMinorTickCount(1);
    angularAxis->setLabelFormat("%d");
    angularAxis->setRange(0, 360);

    radialAxis->setMin(0);
    radialAxis->setMax(90);
    radialAxis->append("90", 0);
    radialAxis->append("60", 30);
    radialAxis->append("30", 60);
    radialAxis->append("0", 90);
    radialAxis->setLabelsPosition(QCategoryAxis::AxisLabelsPositionOnValue);

    m_polarChart->addAxis(angularAxis, QPolarChart::PolarOrientationAngular);
    m_polarChart->addAxis(radialAxis, QPolarChart::PolarOrientationRadial);
    m_polarChart->legend()->hide();
    m_polarChart->layout()->setContentsMargins(0, 0, 0, 0);
    m_polarChart->setMargins(QMargins(1, 1, 1, 1));

    SatNogsSatellite *sat = m_satellites.value(m_settings.m_target);

    if (pass.m_aos.isValid() && pass.m_los.isValid())
    {
        QString title;

        if (m_settings.m_utc) {
            title = pass.m_aos.date().toString(m_settings.m_dateFormat);
        } else {
            title = pass.m_aos.toLocalTime().date().toString(m_settings.m_dateFormat);
        }

        m_polarChart->setTitle(QString("%1").arg(title));
        QLineSeries *polarSeries = new QLineSeries();

        getPassAzEl(
            nullptr,
            nullptr,
            polarSeries,
            sat->m_tle->m_tle0,
            sat->m_tle->m_tle1,
            sat->m_tle->m_tle2,
            m_settings.m_latitude,
            m_settings.m_longitude,
            m_settings.m_heightAboveSeaLevel/1000.0,
            pass.m_aos, pass.m_los
        );

        // Polar charts can't handle points that are more than 180 degrees apart, so
        // we need to split passes that cross from 359 -> 0 degrees (or the reverse)
        QList<QLineSeries *> series;
        series.append(new QLineSeries());
        QLineSeries *s = series.first();
        QPen pen(QColor(32, 159, 223), 2, Qt::SolidLine);
        s->setPen(pen);

        qreal prevAz = polarSeries->at(0).x();
        qreal prevEl = polarSeries->at(0).y();

        for (int i = 1; i < polarSeries->count(); i++)
        {
            qreal az = polarSeries->at(i).x();
            qreal el = polarSeries->at(i).y();

            if ((prevAz > 270.0) && (az <= 90.0))
            {
                double elMid = interpolate(prevAz, prevEl, az+360.0, el, 360.0);
                s->append(360.0, elMid);
                series.append(new QLineSeries());
                s = series.last();
                s->setPen(pen);
                s->append(0.0, elMid);
                s->append(az, el);
            }
            else if ((prevAz <= 90.0) && (az > 270.0))
            {
                double elMid = interpolate(prevAz, prevEl, az-360.0, el, 0.0);
                s->append(0.0, elMid);
                series.append(new QLineSeries());
                s = series.last();
                s->setPen(pen);
                s->append(360.0, elMid);
                s->append(az, el);
            }
            else
            {
                s->append(polarSeries->at(i));
            }

            prevAz = az;
            prevEl = el;
        }

        for (int i = 0; i < series.length(); i++)
        {
            m_polarChart->addSeries(series[i]);
            series[i]->attachAxis(angularAxis);
            series[i]->attachAxis(radialAxis);
        }

        int redrawTime = 0;

        if (m_settings.m_drawRotators != SatelliteTrackerSettings::NO_ROTATORS)
        {
            // Plot rotator position
            QString ourSourceName = QString("F:%1 %2").arg(m_satelliteTracker->getIndexInFeatureSet()).arg(m_satelliteTracker->getIdentifier());
            std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();
            for (int featureSetIndex = 0; featureSetIndex < (int)featureSets.size(); featureSetIndex++)
            {
                FeatureSet *featureSet = featureSets[featureSetIndex];
                for (int featureIndex = 0; featureIndex < featureSet->getNumberOfFeatures(); featureIndex++)
                {
                    Feature *feature = featureSet->getFeatureAt(featureIndex);
                    if (FeatureUtils::compareFeatureURIs(feature->getURI(), "sdrangel.feature.gs232controller"))
                    {
                        QString source;
                        ChannelWebAPIUtils::getFeatureSetting(featureSetIndex, featureIndex, "source", source); // Will return false if source isn't set in Controller
                        int track = 0;
                        ChannelWebAPIUtils::getFeatureSetting(featureSetIndex, featureIndex, "track", track);
                        if ((m_settings.m_drawRotators == SatelliteTrackerSettings::ALL_ROTATORS) || ((source == ourSourceName) && track))
                        {
                            int onTarget = 0;
                            ChannelWebAPIUtils::getFeatureReportValue(featureSetIndex, featureIndex, "onTarget", onTarget);

                            if (!onTarget)
                            {
                                // Target azimuth red dotted line
                                double targetAzimuth, targetElevation;
                                bool targetAzimuthOk = ChannelWebAPIUtils::getFeatureReportValue(featureSetIndex, featureIndex, "targetAzimuth", targetAzimuth);
                                bool targetElevationOk = ChannelWebAPIUtils::getFeatureReportValue(featureSetIndex, featureIndex, "targetElevation", targetElevation);
                                if (targetAzimuthOk && targetElevationOk)
                                {
                                    limitAzElRange(targetAzimuth, targetElevation);

                                    QScatterSeries *rotatorSeries = new QScatterSeries();
                                    QColor color(255, 0, 0, 150);
                                    QPen pen(color);
                                    rotatorSeries->setPen(pen);
                                    rotatorSeries->setColor(color.darker());
                                    rotatorSeries->setMarkerSize(20);
                                    rotatorSeries->append(targetAzimuth, 90-targetElevation);
                                    m_polarChart->addSeries(rotatorSeries);
                                    rotatorSeries->attachAxis(angularAxis);
                                    rotatorSeries->attachAxis(radialAxis);

                                    redrawTime = 333;
                                }
                            }

                            // Current azimuth line. Yellow while off target, green on target.
                            double currentAzimuth, currentElevation;
                            bool currentAzimuthOk = ChannelWebAPIUtils::getFeatureReportValue(featureSetIndex, featureIndex, "currentAzimuth", currentAzimuth);
                            bool currentElevationOk = ChannelWebAPIUtils::getFeatureReportValue(featureSetIndex, featureIndex, "currentElevation", currentElevation);
                            if (currentAzimuthOk && currentElevationOk)
                            {
                                limitAzElRange(currentAzimuth, currentElevation);

                                QScatterSeries *rotatorSeries = new QScatterSeries();
                                QColor color;
                                if (onTarget) {
                                    color = QColor(0, 255, 0, 150);
                                } else {
                                    color = QColor(255, 255, 0, 150);
                                }
                                rotatorSeries->setPen(QPen(color));
                                rotatorSeries->setColor(color.darker());
                                rotatorSeries->setMarkerSize(20);
                                rotatorSeries->append(currentAzimuth, 90-currentElevation);
                                m_polarChart->addSeries(rotatorSeries);
                                rotatorSeries->attachAxis(angularAxis);
                                rotatorSeries->attachAxis(radialAxis);

                                redrawTime = 333;
                            }
                        }
                    }
                }
            }
        }

        // Create series with single point, so we can plot time of AOS
        QLineSeries *aosSeries = new QLineSeries();
        aosSeries->append(polarSeries->at(0));
        QTime time;

        if (m_settings.m_utc) {
            time = pass.m_aos.time();
        } else {
            time = pass.m_aos.toLocalTime().time();
        }

        if (m_settings.m_utc) {
            aosSeries->setPointLabelsFormat(QString("AOS %1").arg(time.toString("hh:mm")));
        } else {
            aosSeries->setPointLabelsFormat(QString("AOS %1").arg(time.toString("hh:mm")));
        }

        aosSeries->setPointLabelsVisible(true);
        aosSeries->setPointLabelsClipping(false);
        m_polarChart->addSeries(aosSeries);
        aosSeries->attachAxis(angularAxis);
        aosSeries->attachAxis(radialAxis);
        // Create series with single point, so we can plot time of LOS
        QLineSeries *losSeries = new QLineSeries();
        losSeries->append(polarSeries->at(polarSeries->count()-1));

        if (m_settings.m_utc) {
            time = pass.m_los.time();
        } else {
            time = pass.m_los.toLocalTime().time();
        }

        losSeries->setPointLabelsFormat(QString("LOS %1").arg(time.toString("hh:mm")));
        losSeries->setPointLabelsVisible(true);
        losSeries->setPointLabelsClipping(false);
        m_polarChart->addSeries(losSeries);
        losSeries->attachAxis(angularAxis);
        losSeries->attachAxis(radialAxis);
        QDateTime currentTime;

        if (m_settings.m_dateTime == "") {
            currentTime = m_satelliteTracker->currentDateTimeUtc();
        } else if (m_settings.m_utc) {
            currentTime = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs);
        } else {
            currentTime = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs).toUTC();
        }

        if ((currentTime >= pass.m_aos) && (currentTime <= pass.m_los))
        {
            // Create series with single point, so we can plot current time
            QScatterSeries *nowSeries = new QScatterSeries();
            nowSeries->setMarkerSize(3);
            // Find closest point to current time
            int idx = std::round(polarSeries->count() * (currentTime.toMSecsSinceEpoch() - pass.m_aos.toMSecsSinceEpoch())
                                                       / (pass.m_los.toMSecsSinceEpoch() - pass.m_aos.toMSecsSinceEpoch()));
            nowSeries->append(polarSeries->at(idx));
            nowSeries->setPointLabelsFormat(m_settings.m_target);
            nowSeries->setPointLabelsVisible(true);
            nowSeries->setPointLabelsClipping(false);
            m_polarChart->addSeries(nowSeries);
            nowSeries->attachAxis(angularAxis);
            nowSeries->attachAxis(radialAxis);
        }

        if (redrawTime > 0)
        {
            // Redraw to show updated rotator position
            m_redrawTimer.setSingleShot(true);
            m_redrawTimer.start(redrawTime);
        }

        delete polarSeries;
    }
    else
    {
        // Possibly geostationary, just plot current position
        QDateTime currentTime;

        if (m_settings.m_dateTime == "") {
            currentTime = m_satelliteTracker->currentDateTimeUtc();
        } else if (m_settings.m_utc) {
            currentTime = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs);
        } else {
            currentTime = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs).toUTC();
        }

        QString title;

        if (m_settings.m_utc) {
            title = currentTime.date().toString(m_settings.m_dateFormat);
        } else {
            title = currentTime.toLocalTime().date().toString(m_settings.m_dateFormat);
        }

        m_polarChart->setTitle(QString("%1").arg(title));
        QLineSeries *nowSeries = new QLineSeries();
        QDateTime endTime = currentTime.addSecs(1);

        getPassAzEl(
            nullptr,
            nullptr,
            nowSeries,
            sat->m_tle->m_tle0,
            sat->m_tle->m_tle1,
            sat->m_tle->m_tle2,
            m_settings.m_latitude,
            m_settings.m_longitude,
            m_settings.m_heightAboveSeaLevel/1000.0,
            currentTime, endTime
        );

        nowSeries->setPointLabelsFormat(m_settings.m_target);
        nowSeries->setPointLabelsVisible(true);
        nowSeries->setPointLabelsClipping(false);
        m_polarChart->addSeries(nowSeries);
        nowSeries->attachAxis(angularAxis);
        nowSeries->attachAxis(radialAxis);
    }

    ui->passChart->setChart(m_polarChart);
    delete oldChart;
}

// Plot target elevation/azimuth for the next pass
void SatelliteTrackerGUI::plotAzElChart()
{
    if ((m_targetSatState == nullptr) || !m_satellites.contains(m_settings.m_target) || (m_targetSatState->m_passes.size() == 0))
    {
        ui->passChart->setChart(&m_emptyChart);
        return;
    }

    QChart *oldChart = m_lineChart;

    if (m_plotPass >= m_targetSatState->m_passes.size() - 1)
    {
        m_plotPass = m_targetSatState->m_passes.size() - 1;
        ui->passLabel->setText(QString("%1").arg(m_plotPass));
    }

    const SatellitePass &pass = m_targetSatState->m_passes[m_plotPass];
    // Always create a new chart, otherwise sometimes they aren't drawn properly
    m_lineChart = new QChart();
    m_lineChart->setTheme(m_settings.m_chartsDarkTheme ? QChart::ChartThemeDark : QChart::ChartThemeLight);
    QDateTimeAxis *xAxis = new QDateTimeAxis();
    QValueAxis *yLeftAxis = new QValueAxis();
    QValueAxis *yRightAxis = new QValueAxis();
    QString title;

    if (m_settings.m_utc) {
        title = pass.m_aos.date().toString(m_settings.m_dateFormat);
    } else {
        title = pass.m_aos.toLocalTime().date().toString(m_settings.m_dateFormat);
    }

    m_lineChart->setTitle(QString("%1").arg(title));
    m_lineChart->legend()->hide();
    m_lineChart->addAxis(xAxis, Qt::AlignBottom);
    m_lineChart->addAxis(yLeftAxis, Qt::AlignLeft);
    m_lineChart->addAxis(yRightAxis, Qt::AlignRight);
    m_lineChart->layout()->setContentsMargins(0, 0, 0, 0);
    m_lineChart->setMargins(QMargins(1, 1, 1, 1));
    SatNogsSatellite *sat = m_satellites.value(m_settings.m_target);
    QLineSeries *azSeries = new QLineSeries();
    QLineSeries *elSeries = new QLineSeries();

    getPassAzEl(
        azSeries,
        elSeries,
        nullptr,
        sat->m_tle->m_tle0,
        sat->m_tle->m_tle1,
        sat->m_tle->m_tle2,
        m_settings.m_latitude,
        m_settings.m_longitude,
        m_settings.m_heightAboveSeaLevel/1000.0,
        pass.m_aos,
        pass.m_los
    );

    // Split crossing of 360/0 degrees in to multiple series in the same colour
    QList<QLineSeries *> azSeriesList;
    QPen pen(QColor(153, 202, 83), 2, Qt::SolidLine);
    QLineSeries *s = new QLineSeries();
    azSeriesList.append(s);
    s->setPen(pen);
    qreal prevAz = azSeries->at(0).y();

    for (int i = 0; i < azSeries->count(); i++)
    {
        qreal az = azSeries->at(i).y();

        if (((prevAz >= 270) && (az < 90)) || ((prevAz < 90) && (az >= 270)))
        {
            s = new QLineSeries();
            azSeriesList.append(s);
            s->setPen(pen);
        }

        s->append(azSeries->at(i).x(), az);
        prevAz = az;
    }

    m_lineChart->addSeries(elSeries);
    elSeries->attachAxis(xAxis);
    elSeries->attachAxis(yLeftAxis);

    for (int i = 0; i < azSeriesList.size(); i++)
    {
        m_lineChart->addSeries(azSeriesList[i]);
        azSeriesList[i]->attachAxis(xAxis);
        azSeriesList[i]->attachAxis(yRightAxis);
    }

    // Plot current target on elevation series
    if (m_targetSatState && (m_targetSatState->m_elevation > 0.0))
    {
        QDateTime currentTime;

        if (m_settings.m_dateTime == "") {
            currentTime = m_satelliteTracker->currentDateTimeUtc();
        } else if (m_settings.m_utc) {
            currentTime = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs);
        } else {
            currentTime = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs).toUTC();
        }

        QScatterSeries *posSeries = new QScatterSeries();
        posSeries->setMarkerSize(3);
        posSeries->append(currentTime.toMSecsSinceEpoch(), m_targetSatState->m_elevation);
        posSeries->setPointLabelsVisible(true);
        posSeries->setPointLabelsFormat(m_settings.m_target);
        posSeries->setPointLabelsClipping(false);
        m_lineChart->addSeries(posSeries);
        posSeries->attachAxis(xAxis);
        posSeries->attachAxis(yLeftAxis);
    }

    xAxis->setRange(pass.m_aos, pass.m_los);
    xAxis->setFormat("hh:mm");
    yLeftAxis->setRange(0.0, 90.0);
    yLeftAxis->setTickCount(7);
    yLeftAxis->setLabelFormat("%d");
    yLeftAxis->setTitleText(QString("Elevation (%1)").arg(QChar(0xb0)));
    yRightAxis->setRange(0.0, 360.0);
    yRightAxis->setTickCount(7);
    yRightAxis->setLabelFormat("%d");
    yRightAxis->setTitleText(QString("Azimuth (%1)").arg(QChar(0xb0)));

    ui->passChart->setChart(m_lineChart);

    delete azSeries;
    delete oldChart;
}

void SatelliteTrackerGUI::resizeTable()
{
    QHeaderView *header = ui->satTable->horizontalHeader();
    QSignalBlocker blocker(header);
    const QFontMetrics metrics(ui->satTable->font());

    for (int column = 0; column < SAT_COL_COLUMNS; ++column)
    {
        int width = m_settings.m_columnSizes[column];

        if (width <= 0)
        {
            width = qMax(
                metrics.horizontalAdvance(QString::fromUtf8(satelliteTableColumns[column].m_title)),
                metrics.horizontalAdvance(QString::fromUtf8(satelliteTableColumns[column].m_widthHint))
            ) + 24;
        }

        header->resizeSection(column, width);
    }

    for (int logical = 0; logical < SAT_COL_COLUMNS; ++logical)
    {
        const int visual = m_settings.m_columnIndexes[logical];

        if ((visual >= 0) && (visual < SAT_COL_COLUMNS) && (header->visualIndex(logical) != visual)) {
            header->moveSection(header->visualIndex(logical), visual);
        }
    }
}

// As we only have limited space in table, display time plus number of days to AOS/LOS
// unless it's greater than 10 days, in which case just display the date
QString SatelliteTrackerGUI::formatDaysTime(qint64 days, QDateTime dateTime)
{
    QDateTime dt;

    if (m_settings.m_utc) {
        dt = dateTime.toUTC();
    } else {
        dt = dateTime.toLocalTime();
    }

    if (abs(days) > 10) {
        return dt.date().toString(m_settings.m_dateFormat);
    } else if (days == 0) {
        return dt.time().toString("hh:mm");
    } else if (days > 0) {
        return dt.time().toString(QString("hh:mm +%1").arg(days));
    } else {
        return dt.time().toString(QString("hh:mm %1").arg(days));
    }
}

QString SatelliteTrackerGUI::formatSecondsAsHHMMSS(qint64 seconds)
{
    char const* sign = "";

    if (seconds < 0)
    {
        sign    = "-";
        seconds = -seconds;
    }

    int minutes = seconds / 60;
    seconds = seconds % 60;
    int hours = minutes / 60;
    minutes = minutes % 60;

    if (hours > 0) {
        return QString("%1%2:%3:%4").arg(sign).arg(hours).arg(minutes, 2, 10, QChar('0')).arg(seconds, 2, 10, QChar('0'));
    } else {
        return QString("%1%2:%3").arg(sign).arg(minutes).arg(seconds, 2, 10, QChar('0'));
    }
}

// Update satellite data table with latest data for the satellite
void SatelliteTrackerGUI::updateTable(const SatelliteState *satState, const QDateTime& currentDateTime)
{
    m_satTableModel->updateSatellite(*satState, m_satellites.value(satState->m_name, nullptr), currentDateTime);
}

void SatelliteTrackerGUI::on_satTable_doubleClicked(const QModelIndex& index)
{
    if (!index.isValid()) {
        return;
    }

    const QModelIndex sourceIndex = m_satTableProxy->mapToSource(index);
    const QString sat = m_satTableModel->satelliteNameAt(sourceIndex.row());

    if (!sat.isEmpty()) {
        FeatureWebAPIUtils::mapFind(sat);
    }
}

void SatelliteTrackerGUI::on_satTableHeader_sortIndicatorChanged(int logicalIndex, Qt::SortOrder order)
{
    m_settings.m_columnSort = logicalIndex;
    m_settings.m_columnSortOrder = order;
    m_satTableProxy->sort(logicalIndex, order);
    m_settingsKeys.append("columnSort");
    m_settingsKeys.append("columnSortOrder");

    applySettings();
}

// Columns in table reordered
void SatelliteTrackerGUI::satTable_sectionMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex)
{
    (void) oldVisualIndex;
    m_settings.m_columnIndexes[logicalIndex] = newVisualIndex;
}

// Column in table resized (when hidden size is 0)
void SatelliteTrackerGUI::satTable_sectionResized(int logicalIndex, int oldSize, int newSize)
{
    (void) oldSize;
    m_settings.m_columnSizes[logicalIndex] = newSize;
}

void SatelliteTrackerGUI::satTableMenu(QPoint pos)
{
    QModelIndex index = ui->satTable->indexAt(pos);
    QString targetSatellite;

    if (index.isValid() && !ui->satTable->selectionModel()->isSelected(index))
    {
        ui->satTable->selectionModel()->select(
            index,
            QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows
        );
        ui->satTable->setCurrentIndex(index);
    }

    if (index.isValid())
    {
        const QModelIndex sourceIndex = m_satTableProxy->mapToSource(index);
        targetSatellite = m_satTableModel->satelliteNameAt(sourceIndex.row());
    }

    QList<QString> selectedSatellites;
    QSet<QString> selectedSatelliteSet;
    const QModelIndexList selectedRows = ui->satTable->selectionModel()->selectedRows();

    for (const QModelIndex& selectedRow : selectedRows)
    {
        const QModelIndex sourceIndex = m_satTableProxy->mapToSource(selectedRow);
        const QString satellite = m_satTableModel->satelliteNameAt(sourceIndex.row());

        if (!satellite.isEmpty() && !selectedSatelliteSet.contains(satellite))
        {
            selectedSatelliteSet.insert(satellite);
            selectedSatellites.append(satellite);
        }
    }

    if (targetSatellite.isEmpty() && (selectedSatellites.size() == 1)) {
        targetSatellite = selectedSatellites[0];
    }

    QMenu menu(this);
    QAction *setTargetAction = menu.addAction("Set as target");
    setTargetAction->setEnabled(!targetSatellite.isEmpty());
    menu.addSeparator();
    QAction *deleteAction = menu.addAction("Delete");
    deleteAction->setEnabled(!selectedSatellites.isEmpty());
    QAction *selectedAction = menu.exec(ui->satTable->viewport()->mapToGlobal(pos));

    if (selectedAction == setTargetAction)
    {
        setTarget(targetSatellite);
        return;
    }

    if (selectedAction == deleteAction)
    {
        for (const QString& satellite : selectedSatellites) {
            m_settings.m_satellites.removeAll(satellite);
        }

        updateSelectedSats();
        m_settingsKeys.append("satellites");
        applySettings();
    }
}

// Right click in table header - show column select menu
void SatelliteTrackerGUI::columnSelectMenu(QPoint pos)
{
    menu->popup(ui->satTable->horizontalHeader()->viewport()->mapToGlobal(pos));
}

// Hide/show column when menu selected
void SatelliteTrackerGUI::columnSelectMenuChecked(bool checked)
{
    (void) checked;
    QAction* action = qobject_cast<QAction*>(sender());

    if (action != nullptr)
    {
        int idx = action->data().toInt(nullptr);
        ui->satTable->setColumnHidden(idx, !action->isChecked());
    }
}

// Create column select menu item
QAction *SatelliteTrackerGUI::createCheckableItem(QString &text, int idx, bool checked)
{
    QAction *action = new QAction(text, this);
    action->setCheckable(true);
    action->setChecked(checked);
    action->setData(QVariant(idx));
    connect(action, SIGNAL(triggered()), this, SLOT(columnSelectMenuChecked()));
    return action;
}

void SatelliteTrackerGUI::updateDeviceFeatureCombo()
{
    if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::FROM_MAP) {
        updateMapList();
    } else if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::FROM_FILE) {
        updateFileInputList();
    } else if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::FROM_CAMERA) {
        updateCameraList();
    }
}

void SatelliteTrackerGUI::updateDeviceFeatureCombo(const QStringList &items, const QString &selected)
{
    // Remove items no longer in list
    int i = 0;
    while (i < ui->deviceFeatureSelect->count())
    {
        if (!items.contains(ui->deviceFeatureSelect->itemText(i))) {
            ui->deviceFeatureSelect->removeItem(i);
        } else {
            i++;
        }
    }
    // Add new items to list
    for (auto item : items)
    {
        int idx = ui->deviceFeatureSelect->findText(item);

        if (idx == -1) {
            ui->deviceFeatureSelect->addItem(item);
        }
    }

    ui->deviceFeatureSelect->setCurrentIndex(ui->deviceFeatureSelect->findText(selected));
}

void SatelliteTrackerGUI::updateFileInputList()
{
    // Create list of File Input devices
    std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
    int deviceIndex = 0;
    QStringList items;

    for (std::vector<DeviceSet*>::const_iterator it = deviceSets.begin(); it != deviceSets.end(); ++it, deviceIndex++)
    {
        if ((*it)->m_deviceAPI && (*it)->m_deviceAPI->getHardwareId() == "FileInput") {
           items.append(QString("R%1").arg(deviceIndex));
        }
    }

    updateDeviceFeatureCombo(items, m_settings.m_fileInputDevice);
}

void SatelliteTrackerGUI::updateMapList()
{
    // Create list of Map features
    std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();
    int featureIndex = 0;
    QStringList items;

    for (std::vector<FeatureSet*>::const_iterator it = featureSets.begin(); it != featureSets.end(); ++it, featureIndex++)
    {
        for (int fi = 0; fi < (*it)->getNumberOfFeatures(); fi++)
        {
            Feature *feature = (*it)->getFeatureAt(fi);

            if (feature->getURI() == "sdrangel.feature.map") {
                items.append(QString("F%1:%2").arg(featureIndex).arg(fi));
            }
        }
    }

    updateDeviceFeatureCombo(items, m_settings.m_mapFeature);
}

void SatelliteTrackerGUI::updateCameraList()
{
    // Create list of Camera features
    std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();
    int featureIndex = 0;
    QStringList items;

    for (std::vector<FeatureSet*>::const_iterator it = featureSets.begin(); it != featureSets.end(); ++it, featureIndex++)
    {
        for (int fi = 0; fi < (*it)->getNumberOfFeatures(); fi++)
        {
            Feature *feature = (*it)->getFeatureAt(fi);

            if (feature->getURI() == "sdrangel.feature.camera") {
                items.append(QString("F%1:%2").arg(featureIndex).arg(fi));
            }
        }
    }

    updateDeviceFeatureCombo(items, m_settings.m_cameraFeature);
}

void SatelliteTrackerGUI::on_deviceFeatureSelect_currentIndexChanged(int index)
{
    (void) index;

    if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::FROM_MAP)
    {
        m_settings.m_mapFeature = ui->deviceFeatureSelect->currentText();
        m_settingsKeys.append("mapFeature");
    }
    else if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::FROM_FILE)
    {
        m_settings.m_fileInputDevice = ui->deviceFeatureSelect->currentText();
        m_settingsKeys.append("fileInputDevice");
    }
    else if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::FROM_CAMERA)
    {
        m_settings.m_cameraFeature = ui->deviceFeatureSelect->currentText();
        m_settingsKeys.append("cameraFeature");
    }

    applySettings();
}

void SatelliteTrackerGUI::makeUIConnections()
{
    QObject::connect(ui->startStop, &ButtonSwitch::toggled, this, &SatelliteTrackerGUI::on_startStop_toggled);
    QObject::connect(ui->useMyPosition, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_useMyPosition_clicked);
    QObject::connect(ui->latitude, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &SatelliteTrackerGUI::on_latitude_valueChanged);
    QObject::connect(ui->longitude, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &SatelliteTrackerGUI::on_longitude_valueChanged);
    QObject::connect(ui->target, &QComboBox::currentTextChanged, this, &SatelliteTrackerGUI::on_target_currentTextChanged);
    QObject::connect(ui->displaySettings, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_displaySettings_clicked);
    QObject::connect(ui->radioControl, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_radioControl_clicked);
    QObject::connect(ui->dateTimeSelect, qOverload<int>(&QComboBox::currentIndexChanged), this, &SatelliteTrackerGUI::on_dateTimeSelect_currentIndexChanged);
    QObject::connect(ui->dateTime, &WrappingDateTimeEdit::dateTimeChanged, this, &SatelliteTrackerGUI::on_dateTime_dateTimeChanged);
    QObject::connect(ui->viewOnMap, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_viewOnMap_clicked);
    QObject::connect(ui->updateSatData, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_updateSatData_clicked);
    QObject::connect(ui->selectSats, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_selectSats_clicked);
    QObject::connect(ui->autoTarget, &ButtonSwitch::clicked, this, &SatelliteTrackerGUI::on_autoTarget_clicked);
    QObject::connect(ui->chartSelect, qOverload<int>(&QComboBox::currentIndexChanged), this, &SatelliteTrackerGUI::on_chartSelect_currentIndexChanged);
    QObject::connect(ui->nextPass, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_nextPass_clicked);
    QObject::connect(ui->prevPass, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_prevPass_clicked);
    QObject::connect(ui->darkTheme, &QToolButton::clicked, this, &SatelliteTrackerGUI::on_darkTheme_clicked);
    QObject::connect(ui->satTable, &QTableView::doubleClicked, this, &SatelliteTrackerGUI::on_satTable_doubleClicked);
    QObject::connect(ui->satTable->horizontalHeader(), &QHeaderView::sortIndicatorChanged, this, &SatelliteTrackerGUI::on_satTableHeader_sortIndicatorChanged);
    QObject::connect(ui->deviceFeatureSelect, qOverload<int>(&QComboBox::currentIndexChanged), this, &SatelliteTrackerGUI::on_deviceFeatureSelect_currentIndexChanged);
}
