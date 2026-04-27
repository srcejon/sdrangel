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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camerahistogramdialog.h"

CameraHistogramDialog::CameraHistogramDialog(const QImage& image, QWidget* parent)
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
    m_chart->setTitle(tr("RGB histogram"));
    m_chart->legend()->setVisible(true);

    m_axisX->setRange(0, 255);
    m_axisX->setTitleText(tr("Pixel value"));
    m_axisX->setLabelFormat("%d");

    m_axisY->setRange(0, 1.0);
    m_axisY->setTitleText(tr("Count"));

    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    m_chartView->setRenderHint(QPainter::Antialiasing);

    auto* closeButton = new QPushButton(tr("Close"), this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::close);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(m_chartView);
    layout->addWidget(closeButton);
    setLayout(layout);

    updateImage(image);
}

void CameraHistogramDialog::updateImage(const QImage& image)
{
    m_chart->removeAllSeries();

    if (image.isNull()) {
        m_axisY->setRange(0, 1.0);
        return;
    }

    const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    std::vector<cv::Mat> channels;
    cv::split(bgrMat, channels);

    constexpr int histSize = 256;
    const float range[] = {0.0f, 256.0f};
    const float* histRange = range;

    const struct { int idx; const char* name; QColor color; } channelDefs[] = {
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
        QPen pen(def.color);
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

        m_chart->addSeries(series);
        series->attachAxis(m_axisX);
        series->attachAxis(m_axisY);
    }

    m_axisY->setRange(0, maxCount > 0 ? maxCount : 1.0);
}
