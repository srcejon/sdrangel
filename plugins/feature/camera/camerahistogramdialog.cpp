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

#include <QPushButton>
#include <QVBoxLayout>
#include "util/profiler.h"

#include "camerahistogramdialog.h"

CameraHistogramDialog::CameraHistogramDialog(const CameraHistogramData& histogramData, QWidget* parent)
    : QDialog(parent),
      m_chart(new QChart()),
      m_chartView(new QChartView(m_chart, this)),
      m_axisX(new QValueAxis()),
      m_axisY(new QValueAxis())
{
    setWindowTitle(tr("Histogram"));
    resize(600, 400);
    setModal(false);

    m_chart->setTheme(QChart::ChartThemeDark);
    m_chart->legend()->setVisible(true);

    m_axisX->setRange(0, 255);
    m_axisX->setTitleText(tr("Pixel value"));
    m_axisX->setLabelFormat("%d");

    m_axisY->setRange(0, 1.0);
    m_axisY->setTitleText(tr("Count"));

    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    m_chartView->setRenderHint(QPainter::Antialiasing);

    m_chartView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* closeButton = new QPushButton(tr("Close"), this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::close);

    auto buttonSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding);

    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->addItem(buttonSpacer);
    buttonLayout->addWidget(closeButton);

    auto* layout = new QVBoxLayout();
    layout->addWidget(m_chartView);
    layout->addLayout(buttonLayout);
    setLayout(layout);

    updateHistogram(histogramData);
}

void CameraHistogramDialog::updateHistogram(const CameraHistogramData& histogramData)
{
    PROFILER_START();
    m_chart->removeAllSeries();

    if (!histogramData.isValid()) {
        m_axisY->setRange(0, 1.0);
        return;
    }

    const struct { const QVector<float>* bins; const char* name; QColor color; } channelDefs[] = {
        {&histogramData.m_redBins, "Red",   Qt::red},
        {&histogramData.m_greenBins, "Green", Qt::green},
        {&histogramData.m_blueBins, "Blue",  Qt::blue}
    };

    double maxCount = 0.0;
    for (const auto& def : channelDefs)
    {
        auto* series = new QLineSeries();
        series->setName(tr(def.name));
        QPen pen(def.color);
        pen.setWidth(1);
        series->setPen(pen);

        for (int j = 0; j < def.bins->size(); ++j)
        {
            const float v = def.bins->at(j);
            series->append(j, v);
            if (v > maxCount) {
                maxCount = v;
            }
        }

        m_chart->addSeries(series);
        series->attachAxis(m_axisX);
        series->attachAxis(m_axisY);
    }

    m_axisY->setRange(0, maxCount > 0 ? maxCount : 1.0);
    PROFILER_STOP("CameraHistogram");
}
