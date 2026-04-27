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

#ifndef INCLUDE_FEATURE_CAMERASETTINGSDIALOG_H_
#define INCLUDE_FEATURE_CAMERASETTINGSDIALOG_H_

#include <QDateTime>
#include <QDialog>

#include "ui_camerasettingsdialog.h"

QT_FORWARD_DECLARE_CLASS(QChart)
QT_FORWARD_DECLARE_CLASS(QDateTimeAxis)
QT_FORWARD_DECLARE_CLASS(QLineSeries)
QT_FORWARD_DECLARE_CLASS(QValueAxis)

class CameraSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraSettingsDialog(QWidget *parent = nullptr);
    ~CameraSettingsDialog() override;

    Ui::CameraSettingsDialog *getUI() const { return ui; }
    void appendTemperatureSample(const QDateTime& timestamp, double temperatureC);
    void clearAlpacaStatus();

private:
    Ui::CameraSettingsDialog *ui;
    QChart *m_tempChart;
    QLineSeries *m_tempSeries;
    QDateTimeAxis *m_tempAxisX;
    QValueAxis *m_tempAxisY;

private slots:
    void on_clearChart_clicked();
};

#endif // INCLUDE_FEATURE_CAMERASETTINGSDIALOG_H_
