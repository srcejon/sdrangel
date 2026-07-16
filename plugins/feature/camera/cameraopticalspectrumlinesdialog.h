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

#ifndef INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMLINESDIALOG_H_
#define INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMLINESDIALOG_H_

#include <QDialog>
#include <QSet>

#include "cameraopticalspectrum.h"

class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

/**
 * \brief Dialog for selecting which reference lines the optical spectrum chart overlays.
 *
 * Shows an expandable tree of the reference line sets (Balmer, He I, Na/Ca, telluric
 * O2 and the user's custom lines) with tristate checkboxes: checking a series selects
 * every line in it, or individual lines can be picked. Changes are emitted immediately
 * via selectionChanged() (in the opticalSpectrumRefLines token format: a set key selects
 * the whole set, "key:label" an individual line) so the chart updates live.
 *
 * The Custom series is editable via Add/Remove buttons; edits are emitted through
 * customLinesChanged() in the opticalSpectrumCustomLines format ("label:nm;label:nm").
 */
class CameraOpticalSpectrumLinesDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CameraOpticalSpectrumLinesDialog(const QString& refLines, const QString& customLines, QWidget* parent = nullptr);

signals:
    void selectionChanged(const QString& refLines);
    void customLinesChanged(const QString& customLines);

private:
    [[nodiscard]] QString refLinesString() const;
    [[nodiscard]] QString customLinesString() const;
    QTreeWidgetItem* addSeries(const QString& key, const QString& name, const QVector<CameraOpticalSpectrumRefLine>& lines, const QStringList& tokens);
    void rebuildCustomNode(const QSet<QString>& checkedLabels);
    void addCustomLine();
    void removeSelectedCustomLines();
    void updateEditButtons();
    [[nodiscard]] bool promptCustomLine(QString& name, double& nm) const;

    QTreeWidget* m_tree;
    QTreeWidgetItem* m_customItem = nullptr;
    QVector<CameraOpticalSpectrumRefLine> m_customLineList;
    QPushButton* m_addButton = nullptr;
    QPushButton* m_removeButton = nullptr;
    bool m_populating = false;
};

#endif // INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMLINESDIALOG_H_
