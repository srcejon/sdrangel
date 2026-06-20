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

#ifndef INCLUDE_FEATURE_CAMERAFILESEQUENCEDIALOG_H_
#define INCLUDE_FEATURE_CAMERAFILESEQUENCEDIALOG_H_

#include <QDialog>
#include <QImage>
#include <QStringList>

class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;

/**
 * \brief Dialog for assembling and ordering a sequence of image files to play back as a camera source.
 *
 * Lets the user build an ordered list of image files (add/remove/reorder) that will be used as a
 * synthetic camera input, with a live preview of the selected file. The resulting ordered list is
 * retrieved via fileNames(). loadPreviewImage() is a static helper that loads a single image for
 * preview thumbnails.
 */
class CameraFileSequenceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraFileSequenceDialog(const QStringList& fileNames, QWidget *parent = nullptr);

    QStringList fileNames() const;

    static QImage loadPreviewImage(const QString& fileName);

private:
    void resizeEvent(QResizeEvent *event) override;
    void addFileItem(const QString& fileName);
    QString fileNameForItem(const QListWidgetItem *item) const;
    bool containsFileName(const QString& fileName) const;
    void updateFileListRowNumbers();
    void addFiles();
    void removeSelectedFiles();
    void moveSelectedFiles(int direction);
    void updateButtons();
    void updatePreview();
    void updatePreviewPixmap();

    QListWidget *m_fileList;
    QLabel *m_previewLabel;
    QPushButton *m_removeButton;
    QPushButton *m_moveUpButton;
    QPushButton *m_moveDownButton;
    QImage m_previewImage;
};

#endif // INCLUDE_FEATURE_CAMERAFILESEQUENCEDIALOG_H_
