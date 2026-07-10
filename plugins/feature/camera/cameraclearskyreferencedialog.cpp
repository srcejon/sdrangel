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
// GNU General Public License V3 for more details.                              //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include <QDialogButtonBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include "cameraclearskyreference.h"
#include "cameraclearskyreferencedialog.h"

namespace {

QLabel* imageLabel(const QImage& image, QWidget *parent)
{
    QLabel *label = new QLabel(parent);
    if (!image.isNull())
    {
        label->setPixmap(QPixmap::fromImage(image));
        label->setFixedSize(image.size());
    }
    else
    {
        label->setText(QStringLiteral("-"));
        label->setAlignment(Qt::AlignCenter);
    }
    return label;
}

} // namespace

CameraClearSkyReferenceDialog::CameraClearSkyReferenceDialog(const QString& storageKey, QWidget *parent) :
    QDialog(parent),
    m_storageKey(storageKey),
    m_grid(nullptr)
{
    setWindowTitle(tr("Clear-sky references"));
    setAttribute(Qt::WA_DeleteOnClose);

    QVBoxLayout *layout = new QVBoxLayout(this);

    QScrollArea *scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    QWidget *content = new QWidget(scroll);
    m_grid = new QGridLayout(content);
    scroll->setWidget(content);
    layout->addWidget(scroll);

    QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    QPushButton *refreshButton = buttons->addButton(tr("Refresh"), QDialogButtonBox::ActionRole);
    connect(refreshButton, &QPushButton::clicked, this, &CameraClearSkyReferenceDialog::refresh);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    resize(1000, 700);
    refresh();
}

void CameraClearSkyReferenceDialog::refresh()
{
    // Clear the grid
    while (QLayoutItem *item = m_grid->takeAt(0))
    {
        delete item->widget();
        delete item;
    }

    // Read the persisted store fresh from disk
    CameraClearSkyReference reference;
    reference.ensureLoaded(m_storageKey);

    const QStringList headers{tr("Slot"), tr("Clear-sky brightness"), tr("Colour (R/B)"), tr("Texture"), tr("Sky mask"), tr("Filled")};
    for (int col = 0; col < headers.size(); ++col)
    {
        QLabel *header = new QLabel(QStringLiteral("<b>%1</b>").arg(headers[col]));
        m_grid->addWidget(header, 0, col);
    }

    int row = 1;
    int filled = 0;
    for (int slot = 0; slot < CameraClearSkyReference::kSlotCount; ++slot)
    {
        const CameraClearSkyReference::SlotPreview preview = reference.slotPreview(slot);
        if (!preview.valid)
        {
            QLabel *empty = new QLabel(QStringLiteral("%1: %2").arg(CameraClearSkyReference::slotName(slot), tr("empty")));
            empty->setStyleSheet(QStringLiteral("color: gray"));
            m_grid->addWidget(empty, row, 0, 1, 6);
            ++row;
            continue;
        }

        QLabel *info = new QLabel(preview.info);
        info->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        m_grid->addWidget(info, row, 0);
        m_grid->addWidget(imageLabel(preview.brightness, this), row, 1);
        m_grid->addWidget(imageLabel(preview.ratio, this), row, 2);
        m_grid->addWidget(imageLabel(preview.texture, this), row, 3);
        m_grid->addWidget(imageLabel(preview.sky, this), row, 4);
        m_grid->addWidget(imageLabel(preview.filled, this), row, 5);
        ++row;
        ++filled;
    }

    const QImage foreground = reference.foregroundPreview();
    QLabel *foregroundTitle = new QLabel(QStringLiteral("<b>%1</b>").arg(tr("Derived foreground (excluded from evaluation)")));
    m_grid->addWidget(foregroundTitle, row, 0);
    m_grid->addWidget(imageLabel(foreground, this), row, 1);
    ++row;

    if (filled == 0)
    {
        QLabel *none = new QLabel(tr("No references saved for this camera yet. Enable 'Clear-sky ref' and press 'Save ref' on a cloud-free sky, or enable 'Auto learn ref'."));
        none->setWordWrap(true);
        m_grid->addWidget(none, row, 0, 1, 6);
    }
    m_grid->setRowStretch(row + 1, 1);
}
