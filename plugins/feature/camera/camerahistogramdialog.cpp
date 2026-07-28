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
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QtCharts/QLegendMarker>
#include "util/profiler.h"

#include "gui/buttonswitch.h"
#include "gui/dialogpositioner.h"

#include "camerahistogramdialog.h"

CameraHistogramDialog::CameraHistogramDialog(
    const CameraHistogramData& histogramData,
    bool useDetectionRoi,
    bool logScale,
    QWidget* parent)
    : QDialog(parent),
      m_chart(new QChart()),
      m_chartView(new QChartView(m_chart, this)),
      m_axisX(new QValueAxis()),
      m_linearAxisY(new QValueAxis()),
      m_logAxisY(new QLogValueAxis()),
      m_useDetectionRoiButton(new ButtonSwitch(this)),
      m_logScaleButton(new ButtonSwitch(this)),
      m_logScale(logScale)
{
    setWindowTitle(tr("Histogram"));
    resize(600, 400);
    setModal(false);
    new DialogPositioner(this, true);

    m_chart->setTheme(QChart::ChartThemeDark);
    m_chart->legend()->setVisible(true);

    m_axisX->setRange(0, 255);
    m_axisX->setTitleText(tr("Pixel value"));
    m_axisX->setLabelFormat("%d");

    m_linearAxisY->setRange(0, 1.0);
    m_linearAxisY->setTitleText(tr("Count"));

    m_logAxisY->setBase(10.0);
    m_logAxisY->setRange(1.0, 10.0);
    m_logAxisY->setTitleText(tr("Count"));
    m_logAxisY->setLabelFormat(QStringLiteral("%.0e"));
    m_logAxisY->setVisible(logScale);
    m_linearAxisY->setVisible(!logScale);

    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_linearAxisY, Qt::AlignLeft);
    m_chart->addAxis(m_logAxisY, Qt::AlignLeft);

    m_chartView->setRenderHint(QPainter::Antialiasing);

    m_chartView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* closeButton = new QPushButton(tr("Close"), this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::close);

    m_useDetectionRoiButton->setText(tr("Use RoI"));
    m_useDetectionRoiButton->setChecked(useDetectionRoi);
    m_useDetectionRoiButton->setToolTip(tr("Compute the histogram using only pixels inside the shared detection RoI"));
    connect(m_useDetectionRoiButton, &ButtonSwitch::toggled, this, &CameraHistogramDialog::useDetectionRoiChanged);

    QIcon histogramScaleIcon;
    histogramScaleIcon.addFile(QStringLiteral(":/linear.png"), QSize(), QIcon::Normal, QIcon::Off);
    histogramScaleIcon.addFile(QStringLiteral(":/logarithmic.png"), QSize(), QIcon::Normal, QIcon::On);
    m_logScaleButton->setIcon(histogramScaleIcon);
    m_logScaleButton->setChecked(logScale);
    m_logScaleButton->setToolTip(tr("Linear / logarithmic vertical axis"));
    connect(m_logScaleButton, &ButtonSwitch::toggled, this, [this](bool enabled) {
        setLogScale(enabled);
        emit logScaleChanged(enabled);
    });

    auto buttonSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding);

    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(m_useDetectionRoiButton);
    buttonLayout->addWidget(m_logScaleButton);
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
    m_histogramData = histogramData;
    m_chart->removeAllSeries();

    if (!histogramData.isValid()) {
        m_linearAxisY->setRange(0, 1.0);
        m_logAxisY->setRange(1.0, 10.0);
        return;
    }

    // QT_TR_NOOP marks the literals for lupdate extraction; the matching tr() below performs
    // the runtime lookup (tr() on a bare runtime const char* is not extractable).
    const struct { const QVector<float>* bins; const char* name; QColor color; } channelDefs[] = {
        {&histogramData.m_redBins, QT_TR_NOOP("Red"),   Qt::red},
        {&histogramData.m_greenBins, QT_TR_NOOP("Green"), Qt::green},
        {&histogramData.m_blueBins, QT_TR_NOOP("Blue"),  Qt::blue}
    };

    double maxCount = 0.0;
    for (int channelIndex = 0; channelIndex < 3; ++channelIndex)
    {
        const auto& def = channelDefs[channelIndex];
        auto* series = new QLineSeries();
        series->setName(tr(def.name));
        QPen pen(def.color);
        pen.setWidth(1);
        series->setPen(pen);

        for (int j = 0; j < def.bins->size(); ++j)
        {
            const float v = def.bins->at(j);
            if (!m_logScale || (v > 0.0f)) {
                series->append(j, v);
            }
            if (v > maxCount) {
                maxCount = v;
            }
        }

        m_chart->addSeries(series);
        series->attachAxis(m_axisX);
        series->attachAxis(m_logScale
            ? static_cast<QAbstractAxis*>(m_logAxisY)
            : static_cast<QAbstractAxis*>(m_linearAxisY));
        series->setVisible(m_seriesVisible[channelIndex]);

        const auto markers = m_chart->legend()->markers(series);
        for (QLegendMarker* marker : markers)
        {
            const auto updateMarkerAppearance = [](QLegendMarker* legendMarker, bool visible) {
                const int alpha = visible ? 255 : 100;

                QBrush labelBrush = legendMarker->labelBrush();
                QColor labelColor = labelBrush.color();
                labelColor.setAlpha(alpha);
                labelBrush.setColor(labelColor);
                legendMarker->setLabelBrush(labelBrush);

                QBrush brush = legendMarker->brush();
                QColor brushColor = brush.color();
                brushColor.setAlpha(alpha);
                brush.setColor(brushColor);
                legendMarker->setBrush(brush);

                QPen markerPen = legendMarker->pen();
                QColor penColor = markerPen.color();
                penColor.setAlpha(alpha);
                markerPen.setColor(penColor);
                legendMarker->setPen(markerPen);
            };

            marker->setVisible(true);
            updateMarkerAppearance(marker, m_seriesVisible[channelIndex]);
            connect(marker, &QLegendMarker::clicked, this, [this, series, marker, channelIndex, updateMarkerAppearance]() {
                m_seriesVisible[channelIndex] = !m_seriesVisible[channelIndex];
                series->setVisible(m_seriesVisible[channelIndex]);
                marker->setVisible(true);
                updateMarkerAppearance(marker, m_seriesVisible[channelIndex]);
            });
        }
    }

    if (m_logScale) {
        m_logAxisY->setRange(1.0, maxCount > 1.0 ? maxCount : 10.0);
    } else {
        m_linearAxisY->setRange(0, maxCount > 0 ? maxCount : 1.0);
    }
    PROFILER_STOP("CameraHistogram");
}

void CameraHistogramDialog::setUseDetectionRoi(bool enabled)
{
    const QSignalBlocker blocker(m_useDetectionRoiButton);
    m_useDetectionRoiButton->setChecked(enabled);
}

void CameraHistogramDialog::setLogScale(bool enabled)
{
    {
        const QSignalBlocker blocker(m_logScaleButton);
        m_logScaleButton->setChecked(enabled);
    }
    if (m_logScale == enabled) {
        return;
    }

    m_logScale = enabled;
    m_linearAxisY->setVisible(!enabled);
    m_logAxisY->setVisible(enabled);
    updateHistogram(m_histogramData);
}
