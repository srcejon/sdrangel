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

#ifndef INCLUDE_FEATURE_CAMERADETECTIONHISTORY_H_
#define INCLUDE_FEATURE_CAMERADETECTIONHISTORY_H_

#include <QDialog>
#include <QList>

#include "cameradetectionhistoryentry.h"

class QTableWidget;
class QPushButton;

class CameraDetectionHistory : public QDialog
{
    Q_OBJECT
public:
    explicit CameraDetectionHistory(const QList<CameraDetectionHistoryEntry>& history, QWidget* parent = nullptr);
    void updateHistory(const QList<CameraDetectionHistoryEntry>& history);

signals:
    void clearHistoryRequested();

private:
    void saveHistoryToCsv();

    QList<CameraDetectionHistoryEntry> m_history;
    QTableWidget* m_table;
    QPushButton* m_clearButton;
    QPushButton* m_saveCsvButton;
};

#endif // INCLUDE_FEATURE_CAMERADETECTIONHISTORY_H_
