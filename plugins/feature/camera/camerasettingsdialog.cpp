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

#include "camerasettingsdialog.h"

#include <algorithm>
#include <QApplication>
#include <QPainter>
#include <QScreen>
#include <QShowEvent>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QGraphicsLayout>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

namespace {

#if defined(Q_OS_ANDROID)
void logWidePageWidgets(const char *pageName, const QWidget *page)
{
    if (!page || page->minimumSizeHint().width() < 380) {
        return;
    }

    const QList<QWidget *> widgets = page->findChildren<QWidget *>();

    for (const QWidget *widget : widgets)
    {
        if (widget->objectName().isEmpty() || widget->minimumSizeHint().width() < 150) {
            continue;
        }

        qDebug() << "CameraSettingsDialog Android wide widget" << pageName
                 << widget->metaObject()->className()
                 << widget->objectName()
                 << "minimum" << widget->minimumSize()
                 << "minimumHint" << widget->minimumSizeHint()
                 << "sizeHint" << widget->sizeHint();
    }
}

void logTabWidgetSizeDiagnostics(const char *name, const QTabWidget *tabWidget)
{
    if (!tabWidget) {
        return;
    }

    qDebug() << "CameraSettingsDialog Android size" << name
             << "size" << tabWidget->size()
             << "minimum" << tabWidget->minimumSize()
             << "minimumHint" << tabWidget->minimumSizeHint()
             << "sizeHint" << tabWidget->sizeHint()
             << "current" << tabWidget->currentIndex()
             << "tabs" << tabWidget->count();

    for (int index = 0; index < tabWidget->count(); ++index)
    {
        const QWidget *page = tabWidget->widget(index);

        if (!page) {
            continue;
        }

        qDebug() << "CameraSettingsDialog Android page" << name << index
                 << tabWidget->tabText(index)
                 << "visible" << page->isVisible()
                 << "size" << page->size()
                 << "minimum" << page->minimumSize()
                 << "minimumHint" << page->minimumSizeHint()
                 << "sizeHint" << page->sizeHint();
        logWidePageWidgets(page->objectName().toUtf8().constData(), page);
    }
}
#endif

void updateTemperatureAxes(const QList<QLineSeries*>& seriesList, QDateTimeAxis* axisX, QValueAxis* axisY);

void updateTemperatureAxes(QLineSeries* series, QDateTimeAxis* axisX, QValueAxis* axisY)
{
    updateTemperatureAxes(QList<QLineSeries*>{series}, axisX, axisY);
}

void updateTemperatureAxes(const QList<QLineSeries*>& seriesList, QDateTimeAxis* axisX, QValueAxis* axisY)
{
    if (!axisX || !axisY) {
        return;
    }

    bool havePoint = false;
    qreal minX = 0.0;
    qreal maxX = 0.0;
    qreal minY = 0.0;
    qreal maxY = 0.0;
    for (const QLineSeries *series : seriesList)
    {
        if (!series || !series->isVisible()) {
            continue;
        }
        const QList<QPointF> points = series->points();
        for (const QPointF& point : points)
        {
            if (!havePoint)
            {
                minX = maxX = point.x();
                minY = maxY = point.y();
                havePoint = true;
            }
            else
            {
                minX = std::min(minX, point.x());
                maxX = std::max(maxX, point.x());
                minY = std::min(minY, point.y());
                maxY = std::max(maxY, point.y());
            }
        }
    }

    if (!havePoint)
    {
        const QDateTime now = QDateTime::currentDateTime();
        axisX->setRange(now.addSecs(-60), now);
        axisY->setRange(0.0, 1.0);
        return;
    }

    if (maxX <= minX) {
        maxX = minX + 1000.0;
    }

    qreal yPadding = (maxY - minY) * 0.1;
    if (yPadding < 0.5) {
        yPadding = 0.5;
    }

    axisX->setRange(QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(minX)),
                    QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(maxX)));
    axisY->setRange(minY - yPadding, maxY + yPadding);
}

}

CameraSettingsDialog::CameraSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraSettingsDialog),
    m_tempChart(nullptr),
    m_tempSeries(nullptr),
    m_tempAxisX(nullptr),
    m_tempAxisY(nullptr),
    m_cloudChart(nullptr),
    m_cloudSeries(nullptr),
    m_cloudAxisX(nullptr),
    m_cloudAxisY(nullptr),
    m_lastCloudSampleMs(0),
    m_thermalChart(nullptr),
    m_thermalSeries(nullptr),
    m_thermalMinimumSeries(nullptr),
    m_thermalMaximumSeries(nullptr),
    m_thermalAxisX(nullptr),
    m_thermalAxisY(nullptr),
    m_lastThermalSampleMs(0),
    m_thermalValuesFahrenheit(false)
{
    ui->setupUi(this);

#if defined(Q_OS_ANDROID)
    // The tab widget now fits the available Android screen width exactly. The
    // dialog's default outer margins would add 8 px on each side and push the
    // window off-screen, while each page already supplies its own padding.
    if (QLayout *dialogLayout = layout())
    {
        const QMargins margins = dialogLayout->contentsMargins();
        dialogLayout->setContentsMargins(0, margins.top(), 0, margins.bottom());
    }
#endif

    m_tempChart = new QChart();
    m_tempChart->setTheme(QChart::ChartThemeDark);
    m_tempChart->setTitle(tr("CCD temperature vs time"));
    m_tempChart->legend()->hide();
    m_tempChart->layout()->setContentsMargins(0, 0, 0, 0);

    m_tempSeries = new QLineSeries(m_tempChart);
    m_tempChart->addSeries(m_tempSeries);

    m_tempAxisX = new QDateTimeAxis(m_tempChart);
    m_tempAxisX->setFormat("HH:mm:ss");

    m_tempAxisY = new QValueAxis(m_tempChart);
    m_tempAxisY->setTitleText(tr("Temperature (C)"));
    m_tempAxisY->setLabelFormat("%.1f");

    m_tempChart->addAxis(m_tempAxisX, Qt::AlignBottom);
    m_tempChart->addAxis(m_tempAxisY, Qt::AlignLeft);
    m_tempSeries->attachAxis(m_tempAxisX);
    m_tempSeries->attachAxis(m_tempAxisY);

    auto* chartView = new QChartView(m_tempChart, ui->cameraTempChartContainer);
    chartView->setRenderHint(QPainter::Antialiasing);

    auto* layout = new QVBoxLayout(ui->cameraTempChartContainer);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(chartView);

    m_cloudChart = new QChart();
    m_cloudChart->setTheme(QChart::ChartThemeDark);
    m_cloudChart->setTitle(tr("Cloud coverage vs time"));
    m_cloudChart->legend()->hide();
    m_cloudChart->layout()->setContentsMargins(0, 0, 0, 0);

    m_cloudSeries = new QLineSeries(m_cloudChart);
    m_cloudChart->addSeries(m_cloudSeries);

    m_cloudAxisX = new QDateTimeAxis(m_cloudChart);
    m_cloudAxisX->setFormat("HH:mm:ss");

    m_cloudAxisY = new QValueAxis(m_cloudChart);
    m_cloudAxisY->setTitleText(tr("Coverage (%)"));
    m_cloudAxisY->setLabelFormat("%.0f");
    m_cloudAxisY->setRange(0.0, 100.0);

    m_cloudChart->addAxis(m_cloudAxisX, Qt::AlignBottom);
    m_cloudChart->addAxis(m_cloudAxisY, Qt::AlignLeft);
    m_cloudSeries->attachAxis(m_cloudAxisX);
    m_cloudSeries->attachAxis(m_cloudAxisY);

    auto* cloudChartView = new QChartView(m_cloudChart, ui->cloudChartContainer);
    cloudChartView->setRenderHint(QPainter::Antialiasing);

    auto* cloudLayout = new QVBoxLayout(ui->cloudChartContainer);
    cloudLayout->setContentsMargins(0, 0, 0, 0);
    cloudLayout->addWidget(cloudChartView);

    updateCloudAxes();

    m_thermalChart = new QChart();
    m_thermalChart->setTheme(QChart::ChartThemeDark);
    m_thermalChart->setTitle(tr("Thermal temperature vs time"));
    m_thermalChart->legend()->hide();
    m_thermalChart->layout()->setContentsMargins(0, 0, 0, 0);
    m_thermalSeries = new QLineSeries(m_thermalChart);
    m_thermalSeries->setName(tr("Marker"));
    m_thermalSeries->setColor(Qt::white);
    m_thermalMinimumSeries = new QLineSeries(m_thermalChart);
    m_thermalMinimumSeries->setName(tr("Minimum"));
    m_thermalMinimumSeries->setColor(QColor(80, 160, 255));
    m_thermalMinimumSeries->setVisible(false);
    m_thermalMaximumSeries = new QLineSeries(m_thermalChart);
    m_thermalMaximumSeries->setName(tr("Maximum"));
    m_thermalMaximumSeries->setColor(QColor(255, 80, 80));
    m_thermalMaximumSeries->setVisible(false);
    m_thermalChart->addSeries(m_thermalSeries);
    m_thermalChart->addSeries(m_thermalMinimumSeries);
    m_thermalChart->addSeries(m_thermalMaximumSeries);
    m_thermalAxisX = new QDateTimeAxis(m_thermalChart);
    m_thermalAxisX->setFormat("HH:mm:ss");
    m_thermalAxisY = new QValueAxis(m_thermalChart);
    m_thermalAxisY->setTitleText(tr("Temperature (C)"));
    m_thermalAxisY->setLabelFormat("%.1f");
    m_thermalChart->addAxis(m_thermalAxisX, Qt::AlignBottom);
    m_thermalChart->addAxis(m_thermalAxisY, Qt::AlignLeft);
    m_thermalSeries->attachAxis(m_thermalAxisX);
    m_thermalSeries->attachAxis(m_thermalAxisY);
    m_thermalMinimumSeries->attachAxis(m_thermalAxisX);
    m_thermalMinimumSeries->attachAxis(m_thermalAxisY);
    m_thermalMaximumSeries->attachAxis(m_thermalAxisX);
    m_thermalMaximumSeries->attachAxis(m_thermalAxisY);
    auto *thermalChartView = new QChartView(m_thermalChart, ui->thermalChartContainer);
    thermalChartView->setRenderHint(QPainter::Antialiasing);
    auto *thermalChartLayout = new QVBoxLayout(ui->thermalChartContainer);
    thermalChartLayout->setContentsMargins(0, 0, 0, 0);
    thermalChartLayout->addWidget(thermalChartView);
    updateTemperatureAxes(m_thermalSeries, m_thermalAxisX, m_thermalAxisY);

    clearCameraStatus();
}

CameraSettingsDialog::~CameraSettingsDialog()
{
    delete ui;
}

void CameraSettingsDialog::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);

#if defined(Q_OS_ANDROID)
    // Run after Qt has applied the platform window geometry, rather than reporting
    // the pre-show size used by fitToAvailableScreen().
    QTimer::singleShot(0, this, [this]() {
        logAndroidSizeDiagnostics();
    });
#endif
}

#if defined(Q_OS_ANDROID)
void CameraSettingsDialog::logAndroidSizeDiagnostics() const
{
    const QScreen *dialogScreen = screen() ? screen() : QApplication::primaryScreen();

    qDebug() << "CameraSettingsDialog Android screen"
             << (dialogScreen ? dialogScreen->name() : QStringLiteral("none"))
             << "available" << (dialogScreen ? dialogScreen->availableGeometry() : QRect())
             << "geometry" << (dialogScreen ? dialogScreen->geometry() : QRect())
             << "devicePixelRatio" << (dialogScreen ? dialogScreen->devicePixelRatio() : 0.0);
    qDebug() << "CameraSettingsDialog Android dialog"
             << "size" << size()
             << "geometry" << geometry()
             << "frameGeometry" << frameGeometry()
             << "minimum" << minimumSize()
             << "minimumHint" << minimumSizeHint()
             << "sizeHint" << sizeHint()
             << "maximum" << maximumSize();

    logTabWidgetSizeDiagnostics("settings", ui->tabWidget);
    logTabWidgetSizeDiagnostics("overlay", ui->overlayTabWidget);
    logTabWidgetSizeDiagnostics("detection", ui->detectionTabWidget);
}
#endif

void CameraSettingsDialog::appendTemperatureSample(const QDateTime& timestamp, double temperatureC)
{
    if (!m_tempSeries) {
        return;
    }

    m_tempSeries->append(timestamp.toMSecsSinceEpoch(), temperatureC);
    updateTemperatureAxes(m_tempSeries, m_tempAxisX, m_tempAxisY);
}

void CameraSettingsDialog::appendThermalSample(const QDateTime& timestamp, double markerTemperatureC,
    double minimumTemperatureC, double maximumTemperatureC, bool showMinMax,
    int historySeconds, int sampleIntervalMs, bool fahrenheit)
{
    if (!m_thermalSeries || !m_thermalMinimumSeries || !m_thermalMaximumSeries || !timestamp.isValid()) {
        return;
    }
    setThermalUnits(fahrenheit);
    m_thermalMinimumSeries->setVisible(showMinMax);
    m_thermalMaximumSeries->setVisible(showMinMax);
    m_thermalChart->legend()->setVisible(showMinMax);
    const qint64 timestampMs = timestamp.toMSecsSinceEpoch();
    if ((m_lastThermalSampleMs > 0) && (timestampMs - m_lastThermalSampleMs < sampleIntervalMs))
    {
        updateTemperatureAxes(
            {m_thermalSeries, m_thermalMinimumSeries, m_thermalMaximumSeries},
            m_thermalAxisX,
            m_thermalAxisY);
        return;
    }
    m_lastThermalSampleMs = timestampMs;
    auto displayValue = [fahrenheit](double temperatureC) {
        return fahrenheit ? temperatureC * 9.0 / 5.0 + 32.0 : temperatureC;
    };
    m_thermalSeries->append(timestampMs, displayValue(markerTemperatureC));
    if (showMinMax)
    {
        m_thermalMinimumSeries->append(timestampMs, displayValue(minimumTemperatureC));
        m_thermalMaximumSeries->append(timestampMs, displayValue(maximumTemperatureC));
    }
    const qint64 oldestMs = timestampMs - static_cast<qint64>(historySeconds) * 1000;
    for (QLineSeries *series : {m_thermalSeries, m_thermalMinimumSeries, m_thermalMaximumSeries})
    {
        const QList<QPointF> points = series->points();
        int removeCount = 0;
        while ((removeCount < points.size()) && (points.at(removeCount).x() < oldestMs)) {
            ++removeCount;
        }
        if (removeCount > 0) {
            series->removePoints(0, removeCount);
        }
    }
    updateTemperatureAxes(
        {m_thermalSeries, m_thermalMinimumSeries, m_thermalMaximumSeries},
        m_thermalAxisX,
        m_thermalAxisY);
}

void CameraSettingsDialog::setThermalUnits(bool fahrenheit)
{
    if (!m_thermalSeries || !m_thermalMinimumSeries || !m_thermalMaximumSeries
        || (fahrenheit == m_thermalValuesFahrenheit))
    {
        return;
    }

    for (QLineSeries *series : {m_thermalSeries, m_thermalMinimumSeries, m_thermalMaximumSeries})
    {
        QList<QPointF> points = series->points();
        for (QPointF& point : points) {
            point.setY(fahrenheit ? point.y() * 9.0 / 5.0 + 32.0 : (point.y() - 32.0) * 5.0 / 9.0);
        }
        series->replace(points);
    }
    m_thermalValuesFahrenheit = fahrenheit;
    m_thermalAxisY->setTitleText(fahrenheit ? tr("Temperature (F)") : tr("Temperature (C)"));
    updateTemperatureAxes(
        {m_thermalSeries, m_thermalMinimumSeries, m_thermalMaximumSeries},
        m_thermalAxisX,
        m_thermalAxisY);
}

void CameraSettingsDialog::clearCameraStatus()
{
    ui->cameraStateLabel->setText("-");
    ui->captureTimeLabel->setText("-");
    ui->receiveImageFormatLabel->setText("-");
    ui->cameraNameLabel->setText("-");
    ui->cameraDescriptionLabel->setText("-");
    ui->sensorNameLabel->setText("-");
    ui->sensorTypeLabel->setText("-");
    ui->pixelSizeLabel->setText("-");
    ui->cameraSizeLabel->setText("-");
    ui->ccdTempLabel->setText("-");
    ui->alpacaErrorCodeLabel->setText("0");
    ui->alpacaErrorMessageLabel->setText("-");

    if (m_tempSeries) {
        m_tempSeries->clear();
    }
    if (m_thermalSeries) {
        m_thermalSeries->clear();
    }
    if (m_thermalMinimumSeries) {
        m_thermalMinimumSeries->clear();
    }
    if (m_thermalMaximumSeries) {
        m_thermalMaximumSeries->clear();
    }
    m_lastThermalSampleMs = 0;

    updateTemperatureAxes(m_tempSeries, m_tempAxisX, m_tempAxisY);
    updateTemperatureAxes(
        {m_thermalSeries, m_thermalMinimumSeries, m_thermalMaximumSeries},
        m_thermalAxisX,
        m_thermalAxisY);
}

void CameraSettingsDialog::on_thermalClearChartButton_clicked()
{
    if (m_thermalSeries) {
        m_thermalSeries->clear();
    }
    if (m_thermalMinimumSeries) {
        m_thermalMinimumSeries->clear();
    }
    if (m_thermalMaximumSeries) {
        m_thermalMaximumSeries->clear();
    }
    m_lastThermalSampleMs = 0;
    updateTemperatureAxes(
        {m_thermalSeries, m_thermalMinimumSeries, m_thermalMaximumSeries},
        m_thermalAxisX,
        m_thermalAxisY);
}

void CameraSettingsDialog::fitToAvailableScreen()
{
    if (layout()) {
        layout()->activate();
    }

    updateGeometry();

    QScreen *dialogScreen = screen() ? screen() : QApplication::primaryScreen();
    if (!dialogScreen) {
        return;
    }

    const QRect availableGeometry = dialogScreen->availableGeometry();
    const QSize frameOverhead = frameGeometry().isValid()
        ? frameGeometry().size() - geometry().size()
        : QSize();
    const QSize maximumContentSize = (availableGeometry.size() - frameOverhead - QSize(16, 16))
        .expandedTo(QSize(320, 240));
    const QSize targetSize = size().boundedTo(maximumContentSize);

    if (targetSize != size()) {
        resize(targetSize);
    }
}

void CameraSettingsDialog::on_clearChart_clicked()
{
    if (m_tempSeries) {
        m_tempSeries->clear();
    }

    updateTemperatureAxes(m_tempSeries, m_tempAxisX, m_tempAxisY);
}

void CameraSettingsDialog::appendCloudCoverageSample(const QDateTime& timestamp, double coveragePercent)
{
    if (!m_cloudSeries || !timestamp.isValid()) {
        return;
    }

    // Coverage arrives per displayed frame; sample the chart every few seconds so a long
    // session stays readable and cheap to append to
    constexpr qint64 sampleIntervalMs = 5000;
    // A day of samples at the sample interval
    constexpr int maxSamples = 17280;

    const qint64 sampleMs = timestamp.toMSecsSinceEpoch();
    if (m_cloudSeries->count() > 0)
    {
        // Playback was restarted or seeked backwards: restart the history rather than
        // drawing a line that doubles back on itself
        if (sampleMs + sampleIntervalMs < m_lastCloudSampleMs) {
            m_cloudSeries->clear();
        } else if (sampleMs - m_lastCloudSampleMs < sampleIntervalMs) {
            return;
        }
    }

    m_cloudSeries->append(sampleMs, coveragePercent);
    m_lastCloudSampleMs = sampleMs;
    if (m_cloudSeries->count() > maxSamples) {
        m_cloudSeries->removePoints(0, m_cloudSeries->count() - maxSamples);
    }

    updateCloudAxes();
}

void CameraSettingsDialog::updateCloudAxes()
{
    if (!m_cloudSeries || !m_cloudAxisX || !m_cloudAxisY) {
        return;
    }

    // Coverage is a percentage, so the Y axis stays fixed at 0-100; only time auto-ranges
    if (m_cloudSeries->count() == 0)
    {
        const QDateTime now = QDateTime::currentDateTime();
        m_cloudAxisX->setRange(now.addSecs(-60), now);
        return;
    }

    qreal minX = m_cloudSeries->at(0).x();
    qreal maxX = m_cloudSeries->at(m_cloudSeries->count() - 1).x();
    if (maxX <= minX) {
        maxX = minX + 1000.0;
    }

    m_cloudAxisX->setRange(QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(minX)),
                           QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(maxX)));
}

void CameraSettingsDialog::on_clearCloudChart_clicked()
{
    if (m_cloudSeries) {
        m_cloudSeries->clear();
    }
    m_lastCloudSampleMs = 0;

    updateCloudAxes();
}
