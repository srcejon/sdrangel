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

#include <QFormLayout>

#include "device/deviceset.h"
#include "maincore.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"

#include "cameraobjectdevicesettingsgui.h"

CameraObjectDeviceSettingsGUI::CameraObjectDeviceSettingsGUI(
    CameraSettings::ObjectDeviceSettings *devSettings,
    QTabWidget *tab,
    QWidget *parent) :
    QWidget(parent),
    m_tab(tab),
    m_deviceSetWidget(new QComboBox(this)),
    m_presetWidget(new QComboBox(this)),
    m_startOnDetectWidget(new QCheckBox(this)),
    m_stopOnDisappearWidget(new QCheckBox(this)),
    m_startStopFileSinkWidget(new QCheckBox(this)),
    m_recordVideoWidget(new QCheckBox(this)),
    m_detectCommandWidget(new QLineEdit(this)),
    m_disappearCommandWidget(new QLineEdit(this)),
    m_currentPresetType('R'),
    m_devSettings(devSettings)
{
    QFormLayout *formLayout = new QFormLayout(this);

    m_deviceSetWidget->setEditable(true);
    m_deviceSetWidget->setToolTip(tr("Device set to control when the selected object class is detected"));
    formLayout->addRow(tr("Device set"), m_deviceSetWidget);

    m_presetWidget->setToolTip(tr("Preset to load when the selected object class is detected"));
    formLayout->addRow(tr("Preset"), m_presetWidget);

    m_startOnDetectWidget->setChecked(devSettings->m_startOnDetect);
    m_startOnDetectWidget->setToolTip(tr("Start acquisition when the selected object class is detected"));
    formLayout->addRow(tr("Start acquisition on detection"), m_startOnDetectWidget);

    m_stopOnDisappearWidget->setChecked(devSettings->m_stopOnDisappear);
    m_stopOnDisappearWidget->setToolTip(tr("Stop acquisition when the selected object class disappears"));
    formLayout->addRow(tr("Stop acquisition on disappearance"), m_stopOnDisappearWidget);

    m_startStopFileSinkWidget->setChecked(devSettings->m_startStopFileSink);
    m_startStopFileSinkWidget->setToolTip(tr("Start file sinks on detection and stop them on disappearance"));
    formLayout->addRow(tr("Start/stop file sinks"), m_startStopFileSinkWidget);

    m_recordVideoWidget->setChecked(devSettings->m_recordVideo);
    m_recordVideoWidget->setToolTip(tr("Start recording video on detection and stop when the object disappears"));
    formLayout->addRow(tr("Record video on detection"), m_recordVideoWidget);

    m_detectCommandWidget->setText(devSettings->m_detectCommand);
    m_detectCommandWidget->setToolTip(tr("Command to execute when the selected object class is detected"));
    formLayout->addRow(tr("Detect command"), m_detectCommandWidget);

    m_disappearCommandWidget->setText(devSettings->m_disappearCommand);
    m_disappearCommandWidget->setToolTip(tr("Command to execute when the selected object class disappears"));
    formLayout->addRow(tr("Disappear command"), m_disappearCommandWidget);

    addDeviceSets();

    int deviceSetIndex = m_deviceSetWidget->findData(devSettings->m_deviceSetIndex);
    if (deviceSetIndex >= 0) {
        m_deviceSetWidget->setCurrentIndex(deviceSetIndex);
    } else if (m_deviceSetWidget->count() > 0) {
        m_deviceSetWidget->setCurrentIndex(0);
    }

    if (m_deviceSetWidget->count() > 0) {
        onDeviceSetChanged(m_deviceSetWidget->currentText());
    }

    connect(m_deviceSetWidget, &QComboBox::currentTextChanged, this, &CameraObjectDeviceSettingsGUI::onDeviceSetChanged);
}

void CameraObjectDeviceSettingsGUI::addDeviceSets()
{
    m_deviceSetWidget->clear();

    const std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
    for (int deviceIndex = 0; deviceIndex < static_cast<int>(deviceSets.size()); ++deviceIndex)
    {
        const DeviceSet *deviceSet = deviceSets[deviceIndex];

        if (deviceSet->m_deviceSourceEngine != nullptr) {
            m_deviceSetWidget->addItem(QString("R%1").arg(deviceIndex), deviceIndex);
        } else if (deviceSet->m_deviceSinkEngine != nullptr) {
            m_deviceSetWidget->addItem(QString("T%1").arg(deviceIndex), deviceIndex);
        } else if (deviceSet->m_deviceMIMOEngine != nullptr) {
            m_deviceSetWidget->addItem(QString("M%1").arg(deviceIndex), deviceIndex);
        }
    }
}

void CameraObjectDeviceSettingsGUI::addPresets(QChar deviceSetType)
{
    m_presetWidget->clear();
    m_presetWidget->addItem(QString());
    m_currentPresetType = deviceSetType;

    const MainSettings& mainSettings = MainCore::instance()->getSettings();
    const int count = mainSettings.getPresetCount();

    for (int i = 0; i < count; ++i)
    {
        const Preset *preset = mainSettings.getPreset(i);
        if (((preset->isSourcePreset()) && (m_currentPresetType == 'R'))
            || ((preset->isSinkPreset()) && (m_currentPresetType == 'T'))
            || ((preset->isMIMOPreset()) && (m_currentPresetType == 'M')))
        {
            m_presetWidget->addItem(QString("%1: %2 - %3")
                .arg(preset->getGroup())
                .arg(preset->getCenterFrequency()/1000000.0, 0, 'f', 3)
                .arg(preset->getDescription()));
        }
    }

    int presetIdx = 1;
    for (int i = 0; i < count; ++i)
    {
        const Preset *preset = mainSettings.getPreset(i);
        if (((preset->isSourcePreset()) && (m_currentPresetType == 'R'))
            || ((preset->isSinkPreset()) && (m_currentPresetType == 'T'))
            || ((preset->isMIMOPreset()) && (m_currentPresetType == 'M')))
        {
            if ((m_devSettings->m_presetGroup == preset->getGroup())
                && (m_devSettings->m_presetFrequency == preset->getCenterFrequency())
                && (m_devSettings->m_presetDescription == preset->getDescription()))
            {
                m_presetWidget->setCurrentIndex(presetIdx);
                break;
            }

            ++presetIdx;
        }
    }
}

const Preset *CameraObjectDeviceSettingsGUI::getSelectedPreset() const
{
    const int listIdx = m_presetWidget->currentIndex();
    if (listIdx <= 0) {
        return nullptr;
    }

    const MainSettings& mainSettings = MainCore::instance()->getSettings();
    const int count = mainSettings.getPresetCount();
    int presetIdx = 1;

    for (int i = 0; i < count; ++i)
    {
        const Preset *preset = mainSettings.getPreset(i);
        if (((preset->isSourcePreset()) && (m_currentPresetType == 'R'))
            || ((preset->isSinkPreset()) && (m_currentPresetType == 'T'))
            || ((preset->isMIMOPreset()) && (m_currentPresetType == 'M')))
        {
            if (listIdx == presetIdx) {
                return preset;
            }

            ++presetIdx;
        }
    }

    return nullptr;
}

void CameraObjectDeviceSettingsGUI::onDeviceSetChanged(const QString& text)
{
    if (text.isEmpty()) {
        return;
    }

    addPresets(text.at(0));

    const int currentTabIndex = m_tab->indexOf(this);
    if (currentTabIndex >= 0) {
        m_tab->setTabText(currentTabIndex, text);
    }
}

void CameraObjectDeviceSettingsGUI::accept()
{
    m_devSettings->m_deviceSetIndex = m_deviceSetWidget->currentData().toInt();

    const Preset *preset = getSelectedPreset();
    if (preset != nullptr)
    {
        m_devSettings->m_presetGroup = preset->getGroup();
        m_devSettings->m_presetFrequency = preset->getCenterFrequency();
        m_devSettings->m_presetDescription = preset->getDescription();
    }
    else
    {
        m_devSettings->m_presetGroup.clear();
        m_devSettings->m_presetFrequency = 0;
        m_devSettings->m_presetDescription.clear();
    }

    m_devSettings->m_startOnDetect = m_startOnDetectWidget->isChecked();
    m_devSettings->m_stopOnDisappear = m_stopOnDisappearWidget->isChecked();
    m_devSettings->m_startStopFileSink = m_startStopFileSinkWidget->isChecked();
    m_devSettings->m_recordVideo = m_recordVideoWidget->isChecked();
    m_devSettings->m_detectCommand = m_detectCommandWidget->text();
    m_devSettings->m_disappearCommand = m_disappearCommandWidget->text();
}
