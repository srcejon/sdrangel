///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERAFILESEQUENCEDIALOG_H_
#define INCLUDE_FEATURE_CAMERAFILESEQUENCEDIALOG_H_

#include <QDialog>
#include <QImage>
#include <QStringList>

class QLabel;
class QListWidget;
class QPushButton;

class CameraFileSequenceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraFileSequenceDialog(const QStringList& fileNames, QWidget *parent = nullptr);

    QStringList fileNames() const;

    static QImage loadPreviewImage(const QString& fileName);

private:
    void addFiles();
    void removeSelectedFiles();
    void moveSelectedFiles(int direction);
    void updateButtons();
    void updatePreview();

    QListWidget *m_fileList;
    QLabel *m_previewLabel;
    QPushButton *m_removeButton;
    QPushButton *m_moveUpButton;
    QPushButton *m_moveDownButton;
};

#endif // INCLUDE_FEATURE_CAMERAFILESEQUENCEDIALOG_H_
