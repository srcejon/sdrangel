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

#include <algorithm>

#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "gui/dialogpositioner.h"

#include "cameraopticalspectrumlinesdialog.h"

namespace {
const QString kCustomKey = QStringLiteral("custom");
}

CameraOpticalSpectrumLinesDialog::CameraOpticalSpectrumLinesDialog(const QString& refLines, const QString& customLines, QWidget* parent)
    : QDialog(parent),
      m_tree(new QTreeWidget(this))
{
    setWindowTitle(tr("Reference lines"));
    resize(360, 420);
    new DialogPositioner(this, true);

    m_tree->setColumnCount(2);
    m_tree->setHeaderLabels({tr("Line"), tr("nm")});
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_tree->setRootIsDecorated(true);

    m_customLineList = CameraOpticalSpectrumExtractor::parseCustomLines(customLines);

    m_populating = true;
    const QStringList tokens = refLines.split(',', Qt::SkipEmptyParts);
    for (const CameraOpticalSpectrumRefLineSet& set : CameraOpticalSpectrumExtractor::referenceLineSets()) {
        addSeries(set.m_key, set.m_name, set.m_lines, tokens);
    }
    m_customItem = addSeries(kCustomKey, tr("Custom"), m_customLineList, tokens);
    m_populating = false;

    connect(m_tree, &QTreeWidget::itemChanged, this, [this]() {
        if (!m_populating) {
            emit selectionChanged(refLinesString());
        }
    });
    connect(m_tree, &QTreeWidget::itemSelectionChanged, this, [this]() { updateEditButtons(); });

    m_addButton = new QPushButton(tr("Add..."), this);
    m_addButton->setToolTip(tr("Add a custom reference line"));
    connect(m_addButton, &QPushButton::clicked, this, [this]() { addCustomLine(); });
    m_removeButton = new QPushButton(tr("Remove"), this);
    m_removeButton->setToolTip(tr("Remove the selected custom reference line(s)"));
    connect(m_removeButton, &QPushButton::clicked, this, [this]() { removeSelectedCustomLines(); });
    updateEditButtons();

    auto* closeButton = new QPushButton(tr("Close"), this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::close);

    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(m_addButton);
    buttonLayout->addWidget(m_removeButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(closeButton);

    auto* layout = new QVBoxLayout();
    layout->addWidget(m_tree, 1);
    layout->addLayout(buttonLayout);
    setLayout(layout);
}

QTreeWidgetItem* CameraOpticalSpectrumLinesDialog::addSeries(const QString& key, const QString& name, const QVector<CameraOpticalSpectrumRefLine>& lines, const QStringList& tokens)
{
    auto* seriesItem = new QTreeWidgetItem(m_tree, {name});
    seriesItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsAutoTristate);
    seriesItem->setData(0, Qt::UserRole, key);

    const bool wholeSet = tokens.contains(key);
    bool anyChecked = false;
    for (const CameraOpticalSpectrumRefLine& line : lines)
    {
        auto* lineItem = new QTreeWidgetItem(seriesItem, {line.m_label, QString::number(line.m_nm, 'f', 2)});
        // ItemIsSelectable is required for the row to select on click; without it the
        // Remove button (enabled on selecting a custom line) can never activate.
        lineItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
        lineItem->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
        const bool checked = wholeSet || tokens.contains(key + ':' + line.m_label);
        lineItem->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
        anyChecked = anyChecked || checked;
    }

    if (lines.isEmpty()) {
        // No children to derive a tristate from (e.g. no custom lines defined yet).
        // Keep the whole-set token's state so it is not dropped from the setting by
        // an unrelated checkbox change.
        seriesItem->setCheckState(0, wholeSet ? Qt::Checked : Qt::Unchecked);
        seriesItem->setDisabled(true);
    }
    seriesItem->setExpanded(anyChecked && !wholeSet);
    return seriesItem;
}

void CameraOpticalSpectrumLinesDialog::rebuildCustomNode(const QSet<QString>& checkedLabels)
{
    m_populating = true;
    const auto children = m_customItem->takeChildren();
    qDeleteAll(children);

    for (const CameraOpticalSpectrumRefLine& line : m_customLineList)
    {
        auto* lineItem = new QTreeWidgetItem(m_customItem, {line.m_label, QString::number(line.m_nm, 'f', 2)});
        lineItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
        lineItem->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
        lineItem->setCheckState(0, checkedLabels.contains(line.m_label) ? Qt::Checked : Qt::Unchecked);
    }

    if (m_customLineList.isEmpty()) {
        m_customItem->setCheckState(0, Qt::Unchecked);
        m_customItem->setDisabled(true);
    } else {
        m_customItem->setDisabled(false);
        m_customItem->setExpanded(true);
    }
    m_populating = false;
    updateEditButtons();
}

void CameraOpticalSpectrumLinesDialog::addCustomLine()
{
    QString name;
    double nm = 0.0;
    if (!promptCustomLine(name, nm)) {
        return;
    }

    // Capture the current custom selection so existing choices survive the rebuild;
    // select the newly added line by default so it shows immediately.
    QSet<QString> checked;
    const bool wholeSet = (m_customItem->checkState(0) == Qt::Checked) && !m_customLineList.isEmpty();
    for (int j = 0; j < m_customItem->childCount(); j++)
    {
        const QTreeWidgetItem* child = m_customItem->child(j);
        if (wholeSet || (child->checkState(0) == Qt::Checked)) {
            checked.insert(child->text(0));
        }
    }
    checked.insert(name);

    m_customLineList.append({name, nm, false});
    rebuildCustomNode(checked);
    emit customLinesChanged(customLinesString());
    emit selectionChanged(refLinesString());
}

void CameraOpticalSpectrumLinesDialog::removeSelectedCustomLines()
{
    QSet<QString> removeLabels;
    for (const QTreeWidgetItem* item : m_tree->selectedItems())
    {
        if (item->parent() == m_customItem) {
            removeLabels.insert(item->text(0));
        }
    }
    if (removeLabels.isEmpty()) {
        return;
    }

    // Preserve the checked state of the lines that remain
    QSet<QString> checked;
    const bool wholeSet = (m_customItem->checkState(0) == Qt::Checked);
    for (int j = 0; j < m_customItem->childCount(); j++)
    {
        const QTreeWidgetItem* child = m_customItem->child(j);
        if ((wholeSet || (child->checkState(0) == Qt::Checked)) && !removeLabels.contains(child->text(0))) {
            checked.insert(child->text(0));
        }
    }

    m_customLineList.erase(
        std::remove_if(m_customLineList.begin(), m_customLineList.end(),
            [&removeLabels](const CameraOpticalSpectrumRefLine& line) { return removeLabels.contains(line.m_label); }),
        m_customLineList.end());

    rebuildCustomNode(checked);
    emit customLinesChanged(customLinesString());
    emit selectionChanged(refLinesString());
}

void CameraOpticalSpectrumLinesDialog::updateEditButtons()
{
    bool customSelected = false;
    for (const QTreeWidgetItem* item : m_tree->selectedItems())
    {
        if (item->parent() == m_customItem) {
            customSelected = true;
            break;
        }
    }
    if (m_removeButton) {
        m_removeButton->setEnabled(customSelected);
    }
}

bool CameraOpticalSpectrumLinesDialog::promptCustomLine(QString& name, double& nm) const
{
    QDialog dialog(const_cast<CameraOpticalSpectrumLinesDialog*>(this));
    dialog.setWindowTitle(tr("Custom reference line"));

    auto* nameEdit = new QLineEdit(&dialog);
    nameEdit->setPlaceholderText(tr("e.g. Fe I 527"));
    auto* nmSpin = new QDoubleSpinBox(&dialog);
    nmSpin->setRange(100.0, 2000.0);
    nmSpin->setDecimals(2);
    nmSpin->setValue(550.0);
    nmSpin->setSuffix(tr(" nm"));

    auto* form = new QFormLayout();
    form->addRow(tr("Name"), nameEdit);
    form->addRow(tr("Wavelength"), nmSpin);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(buttons);

    while (dialog.exec() == QDialog::Accepted)
    {
        const QString trimmed = nameEdit->text().trimmed();
        // The label is stored in the ";"/":"/"," delimited settings, so those
        // characters are not allowed, and labels must be unique among custom lines.
        if (trimmed.isEmpty() || trimmed.contains(':') || trimmed.contains(';') || trimmed.contains(','))
        {
            QMessageBox::warning(const_cast<CameraOpticalSpectrumLinesDialog*>(this), tr("Custom reference line"),
                tr("Enter a name that does not contain ':', ';' or ','."));
            continue;
        }
        const bool duplicate = std::any_of(m_customLineList.cbegin(), m_customLineList.cend(),
            [&trimmed](const CameraOpticalSpectrumRefLine& line) { return line.m_label == trimmed; });
        if (duplicate)
        {
            QMessageBox::warning(const_cast<CameraOpticalSpectrumLinesDialog*>(this), tr("Custom reference line"),
                tr("A custom line named '%1' already exists.").arg(trimmed));
            continue;
        }
        name = trimmed;
        nm = nmSpin->value();
        return true;
    }
    return false;
}

QString CameraOpticalSpectrumLinesDialog::customLinesString() const
{
    QStringList entries;
    for (const CameraOpticalSpectrumRefLine& line : m_customLineList) {
        entries.append(line.m_label + ':' + QString::number(line.m_nm, 'g', 10));
    }
    return entries.join(';');
}

QString CameraOpticalSpectrumLinesDialog::refLinesString() const
{
    QStringList tokens;
    for (int i = 0; i < m_tree->topLevelItemCount(); i++)
    {
        const QTreeWidgetItem* seriesItem = m_tree->topLevelItem(i);
        const QString key = seriesItem->data(0, Qt::UserRole).toString();
        if (seriesItem->childCount() == 0)
        {
            // Childless series (no custom lines yet) keeps its whole-set token
            if (seriesItem->checkState(0) == Qt::Checked) {
                tokens.append(key);
            }
            continue;
        }
        if (seriesItem->checkState(0) == Qt::Checked)
        {
            tokens.append(key);
            continue;
        }
        for (int j = 0; j < seriesItem->childCount(); j++)
        {
            const QTreeWidgetItem* lineItem = seriesItem->child(j);
            if (lineItem->checkState(0) == Qt::Checked) {
                tokens.append(key + ':' + lineItem->text(0));
            }
        }
    }
    return tokens.join(',');
}
