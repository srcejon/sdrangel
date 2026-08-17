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
#include <QHeaderView>
#include <QHBoxLayout>
#include <QImageReader>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPixelFormat>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVariant>
#include <QVBoxLayout>

#include "gui/dialogpositioner.h"
#include "util/fits.h"
#include "cameramediametadata.h"
#include "camerasettings.h"

namespace
{
bool isFitsFile(const QString& fileName)
{
    const QString suffix = QFileInfo(fileName).suffix().toLower();
    return (suffix == QLatin1String("fits"))
        || (suffix == QLatin1String("fit"))
        || (suffix == QLatin1String("fts"));
}

QImage loadFitsImage(const QString& fileName, QVariantMap *headers = nullptr)
{
    FITS fits(fileName);
    if (!fits.valid()) {
        return QImage();
    }
    if (headers) {
        *headers = fits.headers();
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

QString projectionName(int projection)
{
    switch (static_cast<CameraSettings::LensProjection>(projection))
    {
    case CameraSettings::LensProjectionEquidistant:
        return QObject::tr("Equidistant");
    case CameraSettings::LensProjectionEquisolid:
        return QObject::tr("Equisolid");
    case CameraSettings::LensProjectionRectilinear:
    default:
        return QObject::tr("Rectilinear");
    }
}

QString imageFormatName(QImage::Format format)
{
    switch (format)
    {
    case QImage::Format_Mono: return QStringLiteral("Mono MSB");
    case QImage::Format_MonoLSB: return QStringLiteral("Mono LSB");
    case QImage::Format_Indexed8: return QStringLiteral("Indexed 8-bit");
    case QImage::Format_RGB32: return QStringLiteral("RGB32");
    case QImage::Format_ARGB32: return QStringLiteral("ARGB32");
    case QImage::Format_ARGB32_Premultiplied: return QStringLiteral("ARGB32 premultiplied");
    case QImage::Format_RGB16: return QStringLiteral("RGB565");
    case QImage::Format_RGB888: return QStringLiteral("RGB888");
    case QImage::Format_RGB444: return QStringLiteral("RGB444");
    case QImage::Format_ARGB4444_Premultiplied: return QStringLiteral("ARGB4444 premultiplied");
    case QImage::Format_RGBX8888: return QStringLiteral("RGBX8888");
    case QImage::Format_RGBA8888: return QStringLiteral("RGBA8888");
    case QImage::Format_RGBA8888_Premultiplied: return QStringLiteral("RGBA8888 premultiplied");
    case QImage::Format_BGR30: return QStringLiteral("BGR30");
    case QImage::Format_A2BGR30_Premultiplied: return QStringLiteral("A2BGR30 premultiplied");
    case QImage::Format_RGB30: return QStringLiteral("RGB30");
    case QImage::Format_A2RGB30_Premultiplied: return QStringLiteral("A2RGB30 premultiplied");
    case QImage::Format_Alpha8: return QStringLiteral("Alpha8");
    case QImage::Format_Grayscale8: return QStringLiteral("Grayscale8");
    case QImage::Format_RGBX64: return QStringLiteral("RGBX64");
    case QImage::Format_RGBA64: return QStringLiteral("RGBA64");
    case QImage::Format_RGBA64_Premultiplied: return QStringLiteral("RGBA64 premultiplied");
    case QImage::Format_Grayscale16: return QStringLiteral("Grayscale16");
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    case QImage::Format_BGR888: return QStringLiteral("BGR888");
#endif
    case QImage::Format_Invalid:
    default:
        return QObject::tr("Format %1").arg(static_cast<int>(format));
    }
}

int imageColorChannelCount(const QImage& image)
{
    const QPixelFormat pixelFormat = image.pixelFormat();
    switch (pixelFormat.colorModel())
    {
    case QPixelFormat::RGB:
    case QPixelFormat::BGR:
    case QPixelFormat::HSL:
    case QPixelFormat::HSV:
    case QPixelFormat::YUV:
        return 3;
    case QPixelFormat::CMYK:
        return 4;
    case QPixelFormat::Indexed:
    case QPixelFormat::Grayscale:
    case QPixelFormat::Alpha:
    default:
        return 1;
    }
}

int imageComponentBitDepth(const QImage& image)
{
    const QPixelFormat pixelFormat = image.pixelFormat();
    return std::max({
        static_cast<int>(pixelFormat.redSize()),
        static_cast<int>(pixelFormat.greenSize()),
        static_cast<int>(pixelFormat.blueSize()),
        static_cast<int>(pixelFormat.blackSize()),
        static_cast<int>(pixelFormat.alphaSize())});
}

QString byteSizeString(qint64 bytes)
{
    if (bytes >= 1024 * 1024) {
        return QObject::tr("%1 MiB (%2 bytes)").arg(bytes / (1024.0 * 1024.0), 0, 'f', 2).arg(bytes);
    }
    if (bytes >= 1024) {
        return QObject::tr("%1 KiB (%2 bytes)").arg(bytes / 1024.0, 0, 'f', 2).arg(bytes);
    }
    return QObject::tr("%1 bytes").arg(bytes);
}
}

CameraFileSequenceDialog::CameraFileSequenceDialog(const QStringList& fileNames, QWidget *parent) :
    QDialog(parent),
    m_fileList(new QListWidget(this)),
    m_previewLabel(new QLabel(this)),
    m_metadataTable(new QTableWidget(this)),
    m_removeButton(new QPushButton(tr("Remove"), this)),
    m_moveUpButton(new QPushButton(tr("Up"), this)),
    m_moveDownButton(new QPushButton(tr("Down"), this))
{
    setWindowTitle(tr("Image Sequence Files"));
    resize(760, 420);
    new DialogPositioner(this, true);

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

    m_metadataTable->setColumnCount(2);
    m_metadataTable->setHorizontalHeaderLabels({tr("Property"), tr("Value")});
    m_metadataTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_metadataTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_metadataTable->verticalHeader()->setVisible(false);
    m_metadataTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_metadataTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_metadataTable->setAlternatingRowColors(true);
    m_metadataTable->setWordWrap(false);
    m_metadataTable->setMinimumHeight(120);
    m_metadataTable->setToolTip(tr("Metadata embedded in the selected image file"));

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
    QSplitter *previewSplitter = new QSplitter(Qt::Vertical, this);
    previewSplitter->addWidget(m_previewLabel);
    previewSplitter->addWidget(m_metadataTable);
    previewSplitter->setStretchFactor(0, 1);
    previewSplitter->setStretchFactor(1, 0);
    previewSplitter->setSizes(QList<int>() << 270 << 130);
    bodySplitter->addWidget(fileListPane);
    bodySplitter->addWidget(previewSplitter);
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
    connect(previewSplitter, &QSplitter::splitterMoved, this, [this]() { updatePreviewPixmap(); });
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
        setMetadataMessage(tr("No image selected"));
        return;
    }

    const QString fileName = fileNameForItem(item);
    QVariantMap fitsHeaders;
    m_previewImage = isFitsFile(fileName)
        ? loadFitsImage(fileName, &fitsHeaders)
        : loadPreviewImage(fileName);
    updateMetadata(fileName, m_previewImage, fitsHeaders);
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

void CameraFileSequenceDialog::setMetadataMessage(const QString& message)
{
    m_metadataTable->setRowCount(0);
    addMetadataRow(tr("Metadata"), message);
}

void CameraFileSequenceDialog::addMetadataRow(const QString& property, const QString& value)
{
    const int row = m_metadataTable->rowCount();
    m_metadataTable->insertRow(row);
    QTableWidgetItem *propertyItem = new QTableWidgetItem(property);
    QTableWidgetItem *valueItem = new QTableWidgetItem(value);
    propertyItem->setToolTip(property);
    valueItem->setToolTip(value);
    m_metadataTable->setItem(row, 0, propertyItem);
    m_metadataTable->setItem(row, 1, valueItem);
}

void CameraFileSequenceDialog::updateMetadata(
    const QString& fileName,
    const QImage& image,
    const QVariantMap& fitsHeaders)
{
    m_metadataTable->setRowCount(0);

    const QFileInfo fileInfo(fileName);
    addMetadataRow(tr("File type"), fileInfo.suffix().toUpper());
    if (fileInfo.exists()) {
        addMetadataRow(tr("File size"), byteSizeString(fileInfo.size()));
    }

    if (isFitsFile(fileName))
    {
        const int width = fitsHeaders.value(QStringLiteral("NAXIS1"), image.width()).toInt();
        const int height = fitsHeaders.value(QStringLiteral("NAXIS2"), image.height()).toInt();
        const int channels = fitsHeaders.value(QStringLiteral("NAXIS3"), 1).toInt();
        const int bitPix = fitsHeaders.value(QStringLiteral("BITPIX"), 0).toInt();
        addMetadataRow(tr("Image size"), tr("%1 x %2 px").arg(width).arg(height));
        addMetadataRow(tr("Channels"), QString::number(channels));
        if (bitPix != 0)
        {
            addMetadataRow(
                tr("Bit depth per channel"),
                bitPix < 0
                    ? tr("%1-bit floating point").arg(std::abs(bitPix))
                    : tr("%1-bit integer").arg(bitPix));
        }
        if (fitsHeaders.isEmpty())
        {
            addMetadataRow(tr("FITS metadata"), tr("Unavailable"));
        }
        else
        {
            for (auto it = fitsHeaders.cbegin(); it != fitsHeaders.cend(); ++it) {
                addMetadataRow(it.key(), it.value().toString());
            }
        }
        m_metadataTable->resizeRowsToContents();
        return;
    }

    if (!image.isNull())
    {
        const int colorChannels = imageColorChannelCount(image);
        const int channels = colorChannels + (image.hasAlphaChannel() ? 1 : 0);
        addMetadataRow(tr("Image size"), tr("%1 x %2 px").arg(image.width()).arg(image.height()));
        addMetadataRow(tr("Pixel format"), imageFormatName(image.format()));
        addMetadataRow(tr("Channels"), QString::number(channels));
        addMetadataRow(tr("Color channels"), QString::number(colorChannels));
        addMetadataRow(tr("Bit depth per channel"), tr("%1 bits").arg(imageComponentBitDepth(image)));
        addMetadataRow(tr("Storage depth"), tr("%1 bits/pixel").arg(image.depth()));
        addMetadataRow(tr("Alpha channel"), image.hasAlphaChannel() ? tr("Yes") : tr("No"));
        addMetadataRow(tr("Bytes per line"), QString::number(image.bytesPerLine()));
        addMetadataRow(tr("Decoded image size"), byteSizeString(image.sizeInBytes()));
    }

    QString metadataError;
    const CameraMediaMetadata metadata = CameraMediaMetadata::fromImage(image, &metadataError);
    if (!metadata.isValid())
    {
        addMetadataRow(
            tr("Camera metadata"),
            metadataError.isEmpty() ? tr("Not present") : tr("Invalid: %1").arg(metadataError));
    }
    else
    {
        if (metadata.captureDateTimeUtc().isValid()) {
            addMetadataRow(tr("Capture date/time (UTC)"), metadata.captureDateTimeUtc().toString(Qt::ISODateWithMs));
        }
        addMetadataRow(tr("Latitude"), tr("%1 deg").arg(metadata.latitude(), 0, 'f', 8));
        addMetadataRow(tr("Longitude"), tr("%1 deg").arg(metadata.longitude(), 0, 'f', 8));
        addMetadataRow(tr("Altitude"), tr("%1 m").arg(metadata.altitude(), 0, 'f', 2));
        addMetadataRow(tr("Azimuth"), tr("%1 deg").arg(metadata.azimuth(), 0, 'f', 6));
        addMetadataRow(tr("Elevation"), tr("%1 deg").arg(metadata.elevation(), 0, 'f', 6));
        addMetadataRow(tr("Roll"), tr("%1 deg").arg(metadata.roll(), 0, 'f', 6));
        addMetadataRow(tr("Field of view"), tr("%1 deg").arg(metadata.fov(), 0, 'f', 6));
        addMetadataRow(tr("Lens projection"), projectionName(metadata.lensProjection()));
        addMetadataRow(tr("Lens center X"), tr("%1 px").arg(metadata.lensCenterOffsetX(), 0, 'f', 3));
        addMetadataRow(tr("Lens center Y"), tr("%1 px").arg(metadata.lensCenterOffsetY(), 0, 'f', 3));
        addMetadataRow(tr("Lens distortion K1"), QString::number(metadata.lensDistortionK1(), 'g', 10));
        addMetadataRow(tr("Lens mirror"), metadata.lensMirror() ? tr("Yes") : tr("No"));
        if (metadata.imageTransformValid())
        {
            const QTransform& transform = metadata.opticalToImage();
            addMetadataRow(
                tr("Optical image size"),
                tr("%1 x %2 px").arg(metadata.opticalSize().width()).arg(metadata.opticalSize().height()));
            addMetadataRow(
                tr("Optical transform"),
                QStringLiteral("[%1, %2; %3, %4]")
                    .arg(transform.m11(), 0, 'g', 8)
                    .arg(transform.m12(), 0, 'g', 8)
                    .arg(transform.m21(), 0, 'g', 8)
                    .arg(transform.m22(), 0, 'g', 8));
            addMetadataRow(
                tr("Optical translation"),
                tr("%1, %2 px")
                    .arg(transform.dx(), 0, 'g', 8)
                    .arg(transform.dy(), 0, 'g', 8));
        }
    }

    for (const QString& key : image.textKeys())
    {
        if (key != CameraMediaMetadata::metadataKey()) {
            addMetadataRow(key, image.text(key));
        }
    }
    m_metadataTable->resizeRowsToContents();
}
