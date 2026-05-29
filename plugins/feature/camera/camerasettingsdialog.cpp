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
#include <QPainter>
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
    m_tempAxisY(nullptr)
{
    ui->setupUi(this);

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

    clearCameraStatus();
}

CameraSettingsDialog::~CameraSettingsDialog()
{
    delete ui;
}

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

void CameraSettingsDialog::on_clearChart_clicked()
{
    if (m_tempSeries) {
        m_tempSeries->clear();
    }

    updateTemperatureAxes(m_tempSeries, m_tempAxisX, m_tempAxisY);
}
