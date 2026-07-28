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

#ifndef INCLUDE_FEATURE_CAMERAHISTOGRAMDIALOG_H_
#define INCLUDE_FEATURE_CAMERAHISTOGRAMDIALOG_H_

#include <array>

#include <QDialog>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QValueAxis>

#include "camerapipelineframe.h"

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

class ButtonSwitch;

/**
 * @brief Dialog that displays precomputed per-channel histogram data.
 */
class CameraHistogramDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CameraHistogramDialog(
        const CameraHistogramData& histogramData,
        bool useDetectionRoi,
        bool logScale,
        QWidget* parent = nullptr);
    void updateHistogram(const CameraHistogramData& histogramData);
    void setUseDetectionRoi(bool enabled);
    void setLogScale(bool enabled);

signals:
    void useDetectionRoiChanged(bool enabled);
    void logScaleChanged(bool enabled);

private:
    CameraHistogramData m_histogramData;
    QChart* m_chart;
    QChartView* m_chartView;
    QValueAxis* m_axisX;
    QValueAxis* m_linearAxisY;
    QLogValueAxis* m_logAxisY;
    ButtonSwitch* m_useDetectionRoiButton;
    ButtonSwitch* m_logScaleButton;
    bool m_logScale;
    std::array<bool, 3> m_seriesVisible {{true, true, true}};
};

#endif // INCLUDE_FEATURE_CAMERAHISTOGRAMDIALOG_H_
