///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#include <QDialogButtonBox>
#include <QFrame>
#include <QGridLayout>
#include <QScrollArea>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

#include "camerasettingsdialog.h"

CameraSettingsDialog::CameraSettingsDialog(QWidget *parent) :
    QDialog(parent),
    m_cameraSettingsLayout(nullptr),
    m_postProcessingLayout(nullptr)
{
    setWindowTitle(tr("Camera Settings"));
    resize(900, 650);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    QTabWidget *tabWidget = new QTabWidget(this);

    QWidget *cameraScrollContents = new QWidget(tabWidget);
    m_cameraSettingsLayout = new QGridLayout(cameraScrollContents);
    m_cameraSettingsLayout->setContentsMargins(6, 6, 6, 6);
    m_cameraSettingsLayout->setHorizontalSpacing(8);
    m_cameraSettingsLayout->setVerticalSpacing(6);
    m_cameraSettingsLayout->setColumnStretch(1, 1);

    QScrollArea *cameraScrollArea = new QScrollArea(tabWidget);
    cameraScrollArea->setWidgetResizable(true);
    cameraScrollArea->setFrameShape(QFrame::NoFrame);
    cameraScrollArea->setWidget(cameraScrollContents);

    QWidget *postScrollContents = new QWidget(tabWidget);
    m_postProcessingLayout = new QVBoxLayout(postScrollContents);
    m_postProcessingLayout->setContentsMargins(6, 6, 6, 6);
    m_postProcessingLayout->setSpacing(6);

    QScrollArea *postScrollArea = new QScrollArea(tabWidget);
    postScrollArea->setWidgetResizable(true);
    postScrollArea->setFrameShape(QFrame::NoFrame);
    postScrollArea->setWidget(postScrollContents);

    tabWidget->addTab(cameraScrollArea, tr("Camera"));
    tabWidget->addTab(postScrollArea, tr("Post Processing"));
    mainLayout->addWidget(tabWidget);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::close);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::close);
    mainLayout->addWidget(buttonBox);
}
