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

#ifndef INCLUDE_FEATURE_CAMERAHISTOGRAMDIALOG_H_
#define INCLUDE_FEATURE_CAMERAHISTOGRAMDIALOG_H_

#include <QDialog>
#include <QImage>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

/**
 * @brief Dialog that displays per-channel histogram of a camera frame.
 *
 * Uses OpenCV to compute the histogram and Qt Charts to render it.
 */
class CameraHistogramDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CameraHistogramDialog(const QImage& image, QWidget* parent = nullptr);
    void updateImage(const QImage& image);

private:
    QChart* m_chart;
    QChartView* m_chartView;
    QValueAxis* m_axisX;
    QValueAxis* m_axisY;
};

#endif // INCLUDE_FEATURE_CAMERAHISTOGRAMDIALOG_H_
