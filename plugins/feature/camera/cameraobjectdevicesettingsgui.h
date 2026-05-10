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

#ifndef INCLUDE_FEATURE_CAMERAOBJECTDEVICESETTINGSGUI_H
#define INCLUDE_FEATURE_CAMERAOBJECTDEVICESETTINGSGUI_H

#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>
#include <QTabWidget>
#include <QWidget>

#include "camerasettings.h"

class Preset;

class CameraObjectDeviceSettingsGUI : public QWidget
{
    Q_OBJECT

public:
    explicit CameraObjectDeviceSettingsGUI(
        CameraSettings::ObjectDeviceSettings *devSettings,
        QTabWidget *tab,
        QWidget *parent = nullptr);

    void accept();

signals:
    void settingsChanged();

private slots:
    void onDeviceSetChanged(const QString& text);

private:
    void addDeviceSets();
    void addPresets(QChar deviceSetType);
    const Preset *getSelectedPreset() const;
    void loadFromSettings(const CameraSettings::ObjectDeviceSettings& settings);

    QTabWidget *m_tab;
    QComboBox *m_deviceSetWidget;
    QComboBox *m_presetWidget;
    QCheckBox *m_startOnDetectWidget;
    QCheckBox *m_stopOnDisappearWidget;
    QCheckBox *m_startStopFileSinkWidget;
    QCheckBox *m_recordVideoWidget;
    QLineEdit *m_detectCommandWidget;
    QLineEdit *m_disappearCommandWidget;
    QLineEdit *m_detectSpeechWidget;
    QLineEdit *m_disappearSpeechWidget;
    QChar m_currentPresetType;
    CameraSettings::ObjectDeviceSettings *m_devSettings;
};

#endif // INCLUDE_FEATURE_CAMERAOBJECTDEVICESETTINGSGUI_H
