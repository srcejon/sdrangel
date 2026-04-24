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

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QTextStream>
#include <QVBoxLayout>
#include <QtAlgorithms>

#include "cameraobjectcontroldialog.h"

CameraObjectControlDialog::CameraObjectControlDialog(CameraSettings *settings, QWidget *parent) :
    QDialog(parent),
    m_settings(settings),
    m_objectDeviceSettings(settings->m_objectDeviceSettings),
    m_classSelect(new QComboBox(this)),
    m_disappearDebounceSpin(new QDoubleSpinBox(this)),
    m_addButton(new QPushButton(tr("Add device set"), this)),
    m_tabWidget(new QTabWidget(this)),
    m_statusLabel(new QLabel(this)),
    m_buttonBox(new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this))
{
    setWindowTitle(tr("YOLO Object Control"));
    resize(900, 600);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    QHBoxLayout *headerLayout = new QHBoxLayout();

    headerLayout->addWidget(new QLabel(tr("Object class"), this));
    headerLayout->addWidget(m_classSelect, 1);
    headerLayout->addWidget(new QLabel(tr("Disappear debounce (s)"), this));

    m_disappearDebounceSpin->setDecimals(1);
    m_disappearDebounceSpin->setRange(0.0, 60.0);
    m_disappearDebounceSpin->setSingleStep(0.5);
    m_disappearDebounceSpin->setValue(settings->m_yoloDisappearDebounce);
    m_disappearDebounceSpin->setToolTip(tr("How long a class must stay absent before disappearance actions run"));
    headerLayout->addWidget(m_disappearDebounceSpin);

    m_addButton->setToolTip(tr("Add device set control settings for the selected object class"));
    headerLayout->addWidget(m_addButton);

    mainLayout->addLayout(headerLayout);

    m_statusLabel->setWordWrap(true);
    mainLayout->addWidget(m_statusLabel);

    m_tabWidget->setTabsClosable(true);
    mainLayout->addWidget(m_tabWidget, 1);
    mainLayout->addWidget(m_buttonBox);

    const QStringList classes = loadObjectClasses();
    for (const QString& className : classes) {
        m_classSelect->addItem(className);
    }

    connect(m_addButton, &QPushButton::clicked, this, &CameraObjectControlDialog::onAddClicked);
    connect(m_tabWidget, &QTabWidget::tabCloseRequested, this, &CameraObjectControlDialog::onTabCloseRequested);
    connect(m_classSelect, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraObjectControlDialog::onClassCurrentIndexChanged);
    connect(m_buttonBox, &QDialogButtonBox::accepted, this, &CameraObjectControlDialog::accept);
    connect(m_buttonBox, &QDialogButtonBox::rejected, this, &CameraObjectControlDialog::reject);

    rebuildTabsForCurrentClass();
    updateControls();
}

QStringList CameraObjectControlDialog::loadObjectClasses() const
{
    QStringList classes;

    if (m_settings->m_yoloLabelsPath.isEmpty()) {
        return classes;
    }

    QFile f(m_settings->m_yoloLabelsPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return classes;
    }

    QTextStream ts(&f);
    while (!ts.atEnd())
    {
        const QString line = ts.readLine().trimmed();
        if (!line.isEmpty() && !classes.contains(line)) {
            classes.append(line);
        }
    }

    return classes;
}

void CameraObjectControlDialog::saveCurrentClassSettings()
{
    for (CameraObjectDeviceSettingsGUI *gui : m_devSettingsGUIs) {
        gui->accept();
    }
}

void CameraObjectControlDialog::rebuildTabsForCurrentClass()
{
    m_tabWidget->clear();
    qDeleteAll(m_devSettingsGUIs);
    m_devSettingsGUIs.clear();

    const QString className = m_classSelect->currentText();
    if (className.isEmpty()) {
        updateControls();
        return;
    }

    if (!m_objectDeviceSettings.contains(className)) {
        m_objectDeviceSettings.insert(className, new QList<CameraSettings::ObjectDeviceSettings *>());
    }

    QList<CameraSettings::ObjectDeviceSettings *> *devSettingsList = m_objectDeviceSettings.value(className);
    for (CameraSettings::ObjectDeviceSettings *devSettings : *devSettingsList)
    {
        CameraObjectDeviceSettingsGUI *devSettingsGUI =
            new CameraObjectDeviceSettingsGUI(devSettings, m_tabWidget, m_tabWidget);
        const int index = m_tabWidget->addTab(devSettingsGUI, QString("R%1").arg(devSettings->m_deviceSetIndex));
        m_tabWidget->setCurrentIndex(index);
        m_devSettingsGUIs.append(devSettingsGUI);
    }

    updateControls();
}

void CameraObjectControlDialog::updateControls()
{
    const bool hasClasses = m_classSelect->count() > 0;
    const bool hasClassSelection = !m_classSelect->currentText().isEmpty();

    m_classSelect->setEnabled(hasClasses);
    m_addButton->setEnabled(hasClassSelection);
    m_tabWidget->setEnabled(hasClassSelection);

    if (!hasClasses)
    {
        if (m_settings->m_yoloLabelsPath.isEmpty()) {
            m_statusLabel->setText(tr("Select a YOLO labels file first to configure per-class actions."));
        } else {
            m_statusLabel->setText(tr("No class names could be loaded from the labels file."));
        }
    }
    else
    {
        m_statusLabel->setText(tr("Configure what each device set should do when the selected YOLO class is detected or disappears."));
    }
}

void CameraObjectControlDialog::accept()
{
    saveCurrentClassSettings();
    m_settings->m_yoloDisappearDebounce = m_disappearDebounceSpin->value();
    m_settings->m_objectDeviceSettings = m_objectDeviceSettings;
    QDialog::accept();
}

void CameraObjectControlDialog::onAddClicked()
{
    const QString className = m_classSelect->currentText();
    if (className.isEmpty()) {
        return;
    }

    if (!m_objectDeviceSettings.contains(className)) {
        m_objectDeviceSettings.insert(className, new QList<CameraSettings::ObjectDeviceSettings *>());
    }

    CameraSettings::ObjectDeviceSettings *devSettings = new CameraSettings::ObjectDeviceSettings();
    CameraObjectDeviceSettingsGUI *devSettingsGUI =
        new CameraObjectDeviceSettingsGUI(devSettings, m_tabWidget, m_tabWidget);

    const int index = m_tabWidget->addTab(devSettingsGUI, QStringLiteral("R0"));
    m_tabWidget->setCurrentIndex(index);
    m_devSettingsGUIs.append(devSettingsGUI);
    m_objectDeviceSettings.value(className)->append(devSettings);
}

void CameraObjectControlDialog::onTabCloseRequested(int index)
{
    const QString className = m_classSelect->currentText();
    if (className.isEmpty() || !m_objectDeviceSettings.contains(className)) {
        return;
    }

    m_tabWidget->removeTab(index);
    delete m_devSettingsGUIs.takeAt(index);

    QList<CameraSettings::ObjectDeviceSettings *> *devSettingsList = m_objectDeviceSettings.value(className);
    delete devSettingsList->takeAt(index);
}

void CameraObjectControlDialog::onClassCurrentIndexChanged(int index)
{
    (void) index;
    saveCurrentClassSettings();
    rebuildTabsForCurrentClass();
}
