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

#include "camerafilesequencedialog.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QDialogButtonBox>
#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QImageReader>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QSplitter>
#include <QVariant>
#include <QVBoxLayout>

#include "util/fits.h"

namespace
{
bool isFitsFile(const QString& fileName)
{
    const QString suffix = QFileInfo(fileName).suffix().toLower();
    return (suffix == QLatin1String("fits"))
        || (suffix == QLatin1String("fit"))
        || (suffix == QLatin1String("fts"));
}

QImage loadFitsImage(const QString& fileName)
{
    FITS fits(fileName);
    if (!fits.valid()) {
        return QImage();
    }

    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::lowest();
    for (int y = 0; y < fits.height(); ++y)
    {
        for (int x = 0; x < fits.width(); ++x)
        {
            const float value = fits.scaledValue(x, y);
            if (std::isfinite(value))
            {
                minValue = std::min(minValue, value);
                maxValue = std::max(maxValue, value);
            }
        }
    }

    if (!std::isfinite(minValue) || !std::isfinite(maxValue) || (maxValue <= minValue)) {
        return QImage();
    }

    QImage image(fits.width(), fits.height(), QImage::Format_Grayscale8);
    const float scale = 255.0f / (maxValue - minValue);
    for (int y = 0; y < fits.height(); ++y)
    {
        uchar *line = image.scanLine(y);
        for (int x = 0; x < fits.width(); ++x)
        {
            const float value = fits.scaledValue(x, y);
            const int pixel = std::isfinite(value)
                ? qBound(0, static_cast<int>((value - minValue) * scale + 0.5f), 255)
                : 0;
            line[x] = static_cast<uchar>(pixel);
        }
    }
    return image;
}
}

CameraFileSequenceDialog::CameraFileSequenceDialog(const QStringList& fileNames, QWidget *parent) :
    QDialog(parent),
    m_fileList(new QListWidget(this)),
    m_previewLabel(new QLabel(this)),
    m_removeButton(new QPushButton(tr("Remove"), this)),
    m_moveUpButton(new QPushButton(tr("Up"), this)),
    m_moveDownButton(new QPushButton(tr("Down"), this))
{
    setWindowTitle(tr("Image Sequence Files"));
    resize(760, 420);

    m_fileList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_fileList->setDragDropMode(QAbstractItemView::InternalMove);
    m_fileList->setDefaultDropAction(Qt::MoveAction);
    m_fileList->setAlternatingRowColors(true);
    for (const QString& fileName : fileNames) {
        addFileItem(fileName);
    }
    updateFileListRowNumbers();

    m_previewLabel->setAlignment(Qt::AlignCenter);
    m_previewLabel->setMinimumSize(260, 220);
    m_previewLabel->setFrameShape(QFrame::StyledPanel);
    m_previewLabel->setText(tr("No image selected"));

    QPushButton *addButton = new QPushButton(tr("Add..."), this);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);

    QVBoxLayout *listButtonsLayout = new QVBoxLayout;
    listButtonsLayout->addWidget(addButton);
    listButtonsLayout->addWidget(m_removeButton);
    listButtonsLayout->addSpacing(12);
    listButtonsLayout->addWidget(m_moveUpButton);
    listButtonsLayout->addWidget(m_moveDownButton);
    listButtonsLayout->addStretch();

    QWidget *fileListPane = new QWidget(this);
    QHBoxLayout *fileListLayout = new QHBoxLayout(fileListPane);
    fileListLayout->setContentsMargins(0, 0, 0, 0);
    fileListLayout->addWidget(m_fileList, 1);
    fileListLayout->addLayout(listButtonsLayout);

    QSplitter *bodySplitter = new QSplitter(Qt::Horizontal, this);
    bodySplitter->addWidget(fileListPane);
    bodySplitter->addWidget(m_previewLabel);
    bodySplitter->setStretchFactor(0, 1);
    bodySplitter->setStretchFactor(1, 1);
    bodySplitter->setSizes(QList<int>() << 440 << 300);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(bodySplitter, 1);
    mainLayout->addWidget(buttonBox);

    connect(addButton, &QPushButton::clicked, this, [this]() { addFiles(); });
    connect(m_removeButton, &QPushButton::clicked, this, [this]() { removeSelectedFiles(); });
    connect(m_moveUpButton, &QPushButton::clicked, this, [this]() { moveSelectedFiles(-1); });
    connect(m_moveDownButton, &QPushButton::clicked, this, [this]() { moveSelectedFiles(1); });
    connect(m_fileList, &QListWidget::itemSelectionChanged, this, [this]() {
        updateButtons();
        updatePreview();
    });
    connect(m_fileList->model(), &QAbstractItemModel::rowsMoved, this, [this]() {
        updateFileListRowNumbers();
        updateButtons();
        updatePreview();
    });
    connect(bodySplitter, &QSplitter::splitterMoved, this, [this]() { updatePreviewPixmap(); });
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    if (m_fileList->count() > 0) {
        m_fileList->setCurrentRow(0);
    }
    updateButtons();
    updatePreview();
}

QStringList CameraFileSequenceDialog::fileNames() const
{
    QStringList names;
    names.reserve(m_fileList->count());
    for (int i = 0; i < m_fileList->count(); ++i) {
        names.append(fileNameForItem(m_fileList->item(i)));
    }
    return names;
}

QImage CameraFileSequenceDialog::loadPreviewImage(const QString& fileName)
{
    if (isFitsFile(fileName)) {
        return loadFitsImage(fileName);
    }

    QImageReader reader(fileName);
    reader.setAutoTransform(true);
    return reader.read();
}

void CameraFileSequenceDialog::resizeEvent(QResizeEvent *event)
{
    QDialog::resizeEvent(event);
    updatePreviewPixmap();
}

void CameraFileSequenceDialog::addFileItem(const QString& fileName)
{
    QListWidgetItem *item = new QListWidgetItem(m_fileList);
    item->setData(Qt::UserRole, fileName);
    item->setToolTip(fileName);
}

QString CameraFileSequenceDialog::fileNameForItem(const QListWidgetItem *item) const
{
    if (!item) {
        return QString();
    }

    const QString fileName = item->data(Qt::UserRole).toString();
    return fileName.isEmpty() ? item->text() : fileName;
}

bool CameraFileSequenceDialog::containsFileName(const QString& fileName) const
{
    for (int i = 0; i < m_fileList->count(); ++i)
    {
        if (fileNameForItem(m_fileList->item(i)) == fileName) {
            return true;
        }
    }
    return false;
}

void CameraFileSequenceDialog::updateFileListRowNumbers()
{
    for (int i = 0; i < m_fileList->count(); ++i)
    {
        QListWidgetItem *item = m_fileList->item(i);
        item->setText(QStringLiteral("%1. %2").arg(i + 1).arg(fileNameForItem(item)));
    }
}

void CameraFileSequenceDialog::addFiles()
{
    const QString startPath = m_fileList->currentItem()
        ? QFileInfo(fileNameForItem(m_fileList->currentItem())).absolutePath()
        : QString();
    const QStringList fileNames = QFileDialog::getOpenFileNames(
        this,
        tr("Add Image Files"),
        startPath,
        tr("Image Files (*.jpg *.jpeg *.png *.fits *.fit *.fts);;All Files (*.*)"));
    for (const QString& fileName : fileNames)
    {
        if (!containsFileName(fileName)) {
            addFileItem(fileName);
        }
    }
    if (!fileNames.isEmpty() && !m_fileList->currentItem()) {
        m_fileList->setCurrentRow(0);
    }
    updateFileListRowNumbers();
    updateButtons();
    updatePreview();
}

void CameraFileSequenceDialog::removeSelectedFiles()
{
    const QList<QListWidgetItem *> selectedItems = m_fileList->selectedItems();
    for (QListWidgetItem *item : selectedItems) {
        delete m_fileList->takeItem(m_fileList->row(item));
    }
    if ((m_fileList->count() > 0) && !m_fileList->currentItem()) {
        m_fileList->setCurrentRow(0);
    }
    updateFileListRowNumbers();
    updateButtons();
    updatePreview();
}

void CameraFileSequenceDialog::moveSelectedFiles(int direction)
{
    if ((direction != -1) && (direction != 1)) {
        return;
    }

    QList<int> rows;
    const QList<QListWidgetItem *> selectedItems = m_fileList->selectedItems();
    for (QListWidgetItem *item : selectedItems) {
        rows.append(m_fileList->row(item));
    }
    std::sort(rows.begin(), rows.end());
    if (direction > 0) {
        std::reverse(rows.begin(), rows.end());
    }

    for (int row : rows)
    {
        const int newRow = row + direction;
        if ((newRow < 0) || (newRow >= m_fileList->count())) {
            continue;
        }
        QListWidgetItem *item = m_fileList->takeItem(row);
        m_fileList->insertItem(newRow, item);
        item->setSelected(true);
    }
    updateFileListRowNumbers();
    updateButtons();
    updatePreview();
}

void CameraFileSequenceDialog::updateButtons()
{
    const bool hasSelection = !m_fileList->selectedItems().isEmpty();
    m_removeButton->setEnabled(hasSelection);
    m_moveUpButton->setEnabled(hasSelection && (m_fileList->currentRow() > 0));
    m_moveDownButton->setEnabled(hasSelection && (m_fileList->currentRow() >= 0) && (m_fileList->currentRow() < m_fileList->count() - 1));
}

void CameraFileSequenceDialog::updatePreview()
{
    QListWidgetItem *item = m_fileList->currentItem();
    if (!item)
    {
        m_previewImage = QImage();
        m_previewLabel->setPixmap(QPixmap());
        m_previewLabel->setText(tr("No image selected"));
        return;
    }

    m_previewImage = loadPreviewImage(fileNameForItem(item));
    if (m_previewImage.isNull())
    {
        m_previewLabel->setPixmap(QPixmap());
        m_previewLabel->setText(tr("Preview unavailable"));
        return;
    }

    m_previewLabel->setText(QString());
    updatePreviewPixmap();
}

void CameraFileSequenceDialog::updatePreviewPixmap()
{
    if (m_previewImage.isNull()) {
        return;
    }

    m_previewLabel->setPixmap(QPixmap::fromImage(m_previewImage).scaled(
        m_previewLabel->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation));
}
