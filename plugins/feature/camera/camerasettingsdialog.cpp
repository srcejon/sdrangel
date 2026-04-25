///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

namespace {

QLineSeries* temperatureSeries(const CameraSettingsDialog* dialog)
{
    return dialog->findChild<QLineSeries*>("alpacaTempSeries");
}

QDateTimeAxis* temperatureAxisX(const CameraSettingsDialog* dialog)
{
    return dialog->findChild<QDateTimeAxis*>("alpacaTempAxisX");
}

QValueAxis* temperatureAxisY(const CameraSettingsDialog* dialog)
{
    return dialog->findChild<QValueAxis*>("alpacaTempAxisY");
}

void updateTemperatureAxes(const CameraSettingsDialog* dialog)
{
    QLineSeries* series = temperatureSeries(dialog);
    QDateTimeAxis* axisX = temperatureAxisX(dialog);
    QValueAxis* axisY = temperatureAxisY(dialog);

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
    ui(new Ui::CameraSettingsDialog)
{
    ui->setupUi(this);

    auto* chart = new QChart();
    chart->setObjectName("alpacaTempChart");
    chart->setTitle(tr("CCD temperature vs time"));
    chart->legend()->hide();

    auto* series = new QLineSeries(chart);
    series->setObjectName("alpacaTempSeries");
    chart->addSeries(series);

    auto* axisX = new QDateTimeAxis(chart);
    axisX->setObjectName("alpacaTempAxisX");
    axisX->setFormat("HH:mm:ss");
    axisX->setTitleText(tr("Time"));

    auto* axisY = new QValueAxis(chart);
    axisY->setObjectName("alpacaTempAxisY");
    axisY->setTitleText(tr("Temperature (C)"));
    axisY->setLabelFormat("%.1f");

    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisX);
    series->attachAxis(axisY);

    auto* chartView = new QChartView(chart, ui->alpacaTempChartContainer);
    chartView->setRenderHint(QPainter::Antialiasing);

    auto* layout = new QVBoxLayout(ui->alpacaTempChartContainer);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(chartView);

    clearAlpacaStatus();
}

CameraSettingsDialog::~CameraSettingsDialog()
{
    delete ui;
}

void CameraSettingsDialog::appendTemperatureSample(const QDateTime& timestamp, double temperatureC)
{
    QLineSeries* series = temperatureSeries(this);

    if (!series) {
        return;
    }

    series->append(timestamp.toMSecsSinceEpoch(), temperatureC);
    updateTemperatureAxes(this);
}

void CameraSettingsDialog::clearAlpacaStatus()
{
    ui->cameraStateLabel->setText("-");
    ui->sensorNameLabel->setText("-");
    ui->sensorTypeLabel->setText("-");
    ui->pixelSizeLabel->setText("-");
    ui->cameraSizeLabel->setText("-");
    ui->ccdTempLabel->setText("-");

    if (QLineSeries* series = temperatureSeries(this)) {
        series->clear();
    }

    updateTemperatureAxes(this);
}
