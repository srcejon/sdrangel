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

void updateTemperatureAxes(QLineSeries* series, QDateTimeAxis* axisX, QValueAxis* axisY)
{
    if (!series || !axisX || !axisY) {
        return;
    }

    const QList<QPointF> points = series->points();

    if (points.isEmpty())
    {
        const QDateTime now = QDateTime::currentDateTime();
        axisX->setRange(now.addSecs(-60), now);
        axisY->setRange(0.0, 1.0);
        return;
    }

    qreal minX = points.first().x();
    qreal maxX = points.first().x();
    qreal minY = points.first().y();
    qreal maxY = points.first().y();

    for (const QPointF& point : points)
    {
        minX = std::min(minX, point.x());
        maxX = std::max(maxX, point.x());
        minY = std::min(minY, point.y());
        maxY = std::max(maxY, point.y());
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
    m_lastCloudSampleMs(0)
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
    // the pre-show size used by shrinkToVisibleContent().
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

    updateTemperatureAxes(m_tempSeries, m_tempAxisX, m_tempAxisY);
}

void CameraSettingsDialog::shrinkToVisibleContent()
{
    if (layout()) {
        layout()->activate();
    }

    updateGeometry();

    QSize targetSize = minimumSizeHint();
    targetSize = targetSize.expandedTo(minimumSize());

    if (QScreen *screen = this->screen() ? this->screen() : QApplication::primaryScreen())
    {
        const QRect availableGeometry = screen->availableGeometry();
        const QSize frameOverhead = frameGeometry().isValid()
            ? frameGeometry().size() - geometry().size()
            : QSize();
        const QSize maximumContentSize = (availableGeometry.size() - frameOverhead - QSize(16, 16))
            .expandedTo(QSize(320, 240));
        targetSize = targetSize.boundedTo(maximumContentSize);
    }

    resize(targetSize);
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
