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

#include <QPushButton>
#include <QVBoxLayout>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camerahistogramdialog.h"

CameraHistogramDialog::CameraHistogramDialog(const QImage& image, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Histogram"));
    resize(600, 400);

    // Convert QImage to BGR cv::Mat
    const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgrMat;
    mat.copyTo(bgrMat);
    cv::cvtColor(bgrMat, bgrMat, cv::COLOR_RGB2BGR);

    // Split into B, G, R channels
    std::vector<cv::Mat> channels;
    cv::split(bgrMat, channels);

    constexpr int histSize = 256;
    const float range[] = {0.0f, 256.0f};
    const float* histRange = range;

    auto* chart = new QChart();
    chart->setTheme(QChart::ChartThemeDark);
    chart->setTitle(tr("RGB histogram"));
    chart->legend()->setVisible(true);

    // Channel order from cv::split on BGR: 0=B, 1=G, 2=R
    const struct { int idx; const char* name; QColor colour; } channelDefs[] = {
        {2, "Red",   Qt::red},
        {1, "Green", Qt::green},
        {0, "Blue",  Qt::blue}
    };

    double maxCount = 0.0;
    for (const auto& def : channelDefs)
    {
        cv::Mat hist;
        cv::calcHist(&channels[def.idx], 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange);

        auto* series = new QLineSeries();
        series->setName(tr(def.name));
        QPen pen(def.colour);
        pen.setWidth(1);
        series->setPen(pen);

        for (int j = 0; j < histSize; ++j)
        {
            const float v = hist.at<float>(j);
            series->append(j, v);
            if (v > maxCount) {
                maxCount = v;
            }
        }

        chart->addSeries(series);
    }

    auto* axisX = new QValueAxis();
    axisX->setRange(0, 255);
    axisX->setTitleText(tr("Pixel value"));
    axisX->setLabelFormat("%d");

    auto* axisY = new QValueAxis();
    axisY->setRange(0, maxCount > 0 ? maxCount : 1.0);
    axisY->setTitleText(tr("Count"));

    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);

    for (auto* s : chart->series())
    {
        s->attachAxis(axisX);
        s->attachAxis(axisY);
    }

    auto* chartView = new QChartView(chart, this);
    chartView->setRenderHint(QPainter::Antialiasing);

    auto* closeButton = new QPushButton(tr("Close"), this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(chartView);
    layout->addWidget(closeButton);
    setLayout(layout);
}
