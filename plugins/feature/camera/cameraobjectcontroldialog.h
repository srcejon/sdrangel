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

#ifndef INCLUDE_FEATURE_CAMERAOBJECTCONTROLDIALOG_H
#define INCLUDE_FEATURE_CAMERAOBJECTCONTROLDIALOG_H

#include <QDialog>
#include <QHash>
#include <QList>

#include "cameraobjectdevicesettingsgui.h"
#include "camerasettings.h"

class QComboBox;
class QDialogButtonBox;
class QLabel;
class QPushButton;
class QDoubleSpinBox;
class QTabWidget;

class CameraObjectControlDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraObjectControlDialog(CameraSettings *settings, QWidget *parent = nullptr);
    ~CameraObjectControlDialog() override = default;

private slots:
    void accept() override;
    void onAddClicked();
    void onTabCloseRequested(int index);
    void onClassCurrentIndexChanged(int index);

private:
    QStringList loadObjectClasses() const;
    void saveCurrentClassSettings();
    void rebuildTabsForCurrentClass();
    void updateControls();

    CameraSettings *m_settings;
    QHash<QString, QList<CameraSettings::ObjectDeviceSettings *> *> m_objectDeviceSettings;
    QList<CameraObjectDeviceSettingsGUI *> m_devSettingsGUIs;

    QComboBox *m_classSelect;
    QDoubleSpinBox *m_disappearDebounceSpin;
    QPushButton *m_addButton;
    QTabWidget *m_tabWidget;
    QLabel *m_statusLabel;
    QDialogButtonBox *m_buttonBox;
};

#endif // INCLUDE_FEATURE_CAMERAOBJECTCONTROLDIALOG_H
