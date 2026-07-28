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

#ifndef INCLUDE_FEATURE_CAMERASETTINGSDIALOG_H_
#define INCLUDE_FEATURE_CAMERASETTINGSDIALOG_H_

#include <QDateTime>
#include <QDialog>

QT_FORWARD_DECLARE_CLASS(QShowEvent)

#include "ui_camerasettingsdialog.h"

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
namespace QtCharts {
    class QChart;
    class QDateTimeAxis;
    class QLineSeries;
    class QValueAxis;
}
using CameraSettingsChart = QtCharts::QChart;
using CameraSettingsDateTimeAxis = QtCharts::QDateTimeAxis;
using CameraSettingsLineSeries = QtCharts::QLineSeries;
using CameraSettingsValueAxis = QtCharts::QValueAxis;
#else
QT_FORWARD_DECLARE_CLASS(QChart)
QT_FORWARD_DECLARE_CLASS(QDateTimeAxis)
QT_FORWARD_DECLARE_CLASS(QLineSeries)
QT_FORWARD_DECLARE_CLASS(QValueAxis)
using CameraSettingsChart = QChart;
using CameraSettingsDateTimeAxis = QDateTimeAxis;
using CameraSettingsLineSeries = QLineSeries;
using CameraSettingsValueAxis = QValueAxis;
#endif

/**
 * \brief Dialog hosting the camera's detailed settings UI and live history charts.
 *
 * Wraps the generated Ui::CameraSettingsDialog (the large multi-page settings form) and adds
 * QtCharts line charts that plot CCD temperature and cloud coverage over time.
 * appendTemperatureSample() and appendCloudCoverageSample() feed new samples into the charts,
 * clearCameraStatus() resets the status display, and fitToAvailableScreen() ensures the
 * dialog does not exceed the available screen geometry.
 *
 * \note The owning CameraGUI accesses the underlying UI directly via getUI(); this dialog is a thin
 *       container around that form plus the charts.
 */
class CameraSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraSettingsDialog(QWidget *parent = nullptr);
    ~CameraSettingsDialog() override;

    Ui::CameraSettingsDialog *getUI() const { return ui; }
    void appendTemperatureSample(const QDateTime& timestamp, double temperatureC);
    void appendCloudCoverageSample(const QDateTime& timestamp, double coveragePercent);
    void appendThermalSample(const QDateTime& timestamp, double markerTemperatureC,
        double minimumTemperatureC, double maximumTemperatureC, bool showMinMax,
        int historySeconds, int sampleIntervalMs, bool fahrenheit);
    void setThermalUnits(bool fahrenheit);
    void clearCameraStatus();
    void fitToAvailableScreen();

protected:
    void showEvent(QShowEvent *event) override;

private:
    Ui::CameraSettingsDialog *ui;
    CameraSettingsChart *m_tempChart;
    CameraSettingsLineSeries *m_tempSeries;
    CameraSettingsDateTimeAxis *m_tempAxisX;
    CameraSettingsValueAxis *m_tempAxisY;
    CameraSettingsChart *m_cloudChart;
    CameraSettingsLineSeries *m_cloudSeries;
    CameraSettingsDateTimeAxis *m_cloudAxisX;
    CameraSettingsValueAxis *m_cloudAxisY;
    qint64 m_lastCloudSampleMs;
    CameraSettingsChart *m_thermalChart;
    CameraSettingsLineSeries *m_thermalSeries;
    CameraSettingsLineSeries *m_thermalMinimumSeries;
    CameraSettingsLineSeries *m_thermalMaximumSeries;
    CameraSettingsDateTimeAxis *m_thermalAxisX;
    CameraSettingsValueAxis *m_thermalAxisY;
    qint64 m_lastThermalSampleMs;
    bool m_thermalValuesFahrenheit;

    void updateCloudAxes();

#if defined(Q_OS_ANDROID)
    void logAndroidSizeDiagnostics() const;
#endif

private slots:
    void on_clearChart_clicked();
    void on_clearCloudChart_clicked();
    void on_thermalClearChartButton_clicked();
};

#endif // INCLUDE_FEATURE_CAMERASETTINGSDIALOG_H_
