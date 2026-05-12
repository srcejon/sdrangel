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

#include <QFileDialog>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSaveFile>
#include <QTableWidget>
#include <QTextStream>
#include <QVBoxLayout>

#include "cameradetectionhistory.h"

namespace {
QString formatDateTime(const QDateTime& dateTime)
{
    if (!dateTime.isValid()) {
        return QObject::tr("Active");
    }

    return dateTime.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
}
}

CameraDetectionHistory::CameraDetectionHistory(const QList<CameraDetectionHistoryEntry>& history, QWidget* parent) :
    QDialog(parent),
    m_table(new QTableWidget(this)),
    m_clearButton(new QPushButton(tr("Clear history"), this)),
    m_saveCsvButton(new QPushButton(tr("Save to CSV"), this))
{
    setWindowTitle(tr("Detection History"));
    resize(720, 360);
    setModal(false);

    m_table->setColumnCount(4);
    m_table->setHorizontalHeaderLabels({tr("Class"), tr("First detected"), tr("Disappeared"), tr("Peak confidence")});
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setAlternatingRowColors(true);
    m_table->setSortingEnabled(false);
    m_table->verticalHeader()->setVisible(false);
    m_table->horizontalHeader()->setStretchLastSection(false);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    auto* closeButton = new QPushButton(tr("Close"), this);
    connect(m_clearButton, &QPushButton::clicked, this, &CameraDetectionHistory::clearHistoryRequested);
    connect(m_saveCsvButton, &QPushButton::clicked, this, &CameraDetectionHistory::saveHistoryToCsv);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::close);

    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(m_clearButton);
    buttonLayout->addWidget(m_saveCsvButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(closeButton);

    auto* layout = new QVBoxLayout();
    layout->addWidget(m_table);
    layout->addLayout(buttonLayout);
    setLayout(layout);

    updateHistory(history);
}

void CameraDetectionHistory::updateHistory(const QList<CameraDetectionHistoryEntry>& history)
{
    m_history = history;
    m_table->setSortingEnabled(false);
    m_table->setRowCount(history.size());

    for (int row = 0; row < history.size(); ++row)
    {
        const CameraDetectionHistoryEntry& entry = history.at(row);
        m_table->setItem(row, 0, new QTableWidgetItem(entry.m_label));
        m_table->setItem(row, 1, new QTableWidgetItem(formatDateTime(entry.m_firstDetected)));
        m_table->setItem(row, 2, new QTableWidgetItem(formatDateTime(entry.m_disappeared)));
        m_table->setItem(row, 3, new QTableWidgetItem(QString::number(entry.m_peakConfidence, 'f', 3)));
    }

    m_table->setSortingEnabled(true);
    m_table->sortItems(1, Qt::DescendingOrder);
}

void CameraDetectionHistory::saveHistoryToCsv()
{
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Save Detection History"),
        QStringLiteral("camera-detection-history.csv"),
        tr("CSV files (*.csv)"));

    if (fileName.isEmpty()) {
        return;
    }

    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }

    QTextStream stream(&file);
    stream << "\"Class\",\"First detected\",\"Disappeared\",\"Peak confidence\"\n";

    for (const CameraDetectionHistoryEntry& entry : m_history)
    {
        auto escapeCsv = [](QString value) {
            value.replace('"', QStringLiteral("\"\""));
            return QStringLiteral("\"%1\"").arg(value);
        };

        stream << escapeCsv(entry.m_label) << ','
               << escapeCsv(formatDateTime(entry.m_firstDetected)) << ','
               << escapeCsv(formatDateTime(entry.m_disappeared)) << ','
               << escapeCsv(QString::number(entry.m_peakConfidence, 'f', 3)) << '\n';
    }

    file.commit();
}
