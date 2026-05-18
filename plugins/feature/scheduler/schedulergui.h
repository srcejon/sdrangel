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

#ifndef INCLUDE_FEATURE_SCHEDULERGUI_H_
#define INCLUDE_FEATURE_SCHEDULERGUI_H_

#include "availablechannelorfeaturehandler.h"
#include "feature/featuregui.h"
#include "settings/rollupstate.h"
#include "util/messagequeue.h"

#include "schedulersettings.h"

class Feature;
class FeatureUISet;
class PluginAPI;
class Preset;
class QComboBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QSpinBox;
class QTableWidgetItem;
class Scheduler;

namespace Ui {
    class SchedulerGUI;
}

class SchedulerGUI : public FeatureGUI
{
    Q_OBJECT

public:
    static SchedulerGUI* create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature);
    void destroy() override;

    bool handleMessage(const Message& message);

    void resetToDefaults() override;
    QByteArray serialize() const override;
    bool deserialize(const QByteArray& data) override;
    MessageQueue *getInputMessageQueue() override { return &m_inputMessageQueue; }
    void setWorkspaceIndex(int index) override;
    int getWorkspaceIndex() const override { return m_settings.m_workspaceIndex; }
    void setGeometryBytes(const QByteArray& blob) override { m_settings.m_geometryBytes = blob; }
    QByteArray getGeometryBytes() const override { return m_settings.m_geometryBytes; }

private:
    Ui::SchedulerGUI* ui;
    Scheduler* m_scheduler;
    SchedulerSettings m_settings;
    RollupState m_rollupState;
    MessageQueue m_inputMessageQueue;
    AvailableChannelOrFeatureHandler m_eventSourceHandler;
    bool m_doApplySettings;
    bool m_populating;
    int m_currentRule;
    int m_currentDeviceAction;
    int m_currentFeatureAction;
    QGroupBox *m_cameraActionGroup;
    QLineEdit *m_cameraFilename;
    QComboBox *m_cameraRecordMode;
    QLabel *m_cameraImageCountLabel;
    QSpinBox *m_cameraImageCount;
    QLabel *m_cameraVideoDurationLabel;
    QSpinBox *m_cameraVideoDuration;
    QGroupBox *m_findActionGroup;
    QLineEdit *m_findTarget;

    explicit SchedulerGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent = nullptr);
    ~SchedulerGUI() override;

    void makeUIConnections();
    void blockApplySettings(bool block);
    void applySettings(const QStringList& settingsKeys, bool force = false);
    void applyRules();
    void displaySettings();
    void refreshRulesTable();
    void refreshDeviceActionsTable();
    void refreshFeatureActionsTable();
    void displayRuleEditor();
    void displayDeviceActionEditor();
    void displayFeatureActionEditor();
    void createFeatureActionParameterEditors();
    void updateFeatureActionParameterVisibility();
    bool updateCurrentRuleFromWidgets();
    void updateCurrentDeviceActionFromWidgets();
    void updateCurrentFeatureActionFromWidgets();
    void updateEventSourceList(const QString& selectedSource);
    void updateDeviceSetList(const SchedulerSettings::DeviceSetAction *selectedAction);
    void updatePresetList(const SchedulerSettings::DeviceSetAction *selectedAction);
    void updateFeatureList(const SchedulerSettings::FeatureAction *selectedAction);
    void updateTriggerVisibility();
    void updateRegexState();
    void selectRule(int row);
    void selectDeviceAction(int row);
    void selectFeatureAction(int row);
    SchedulerSettings::ScheduleRule *currentRule();
    SchedulerSettings::DeviceSetAction *currentDeviceAction();
    SchedulerSettings::FeatureAction *currentFeatureAction();
    const Preset *selectedPreset() const;
    QString ruleTriggerText(const SchedulerSettings::ScheduleRule& rule) const;
    QString ruleRecurrenceDelayText(const SchedulerSettings::ScheduleRule& rule) const;
    QString ruleActionSummary(const SchedulerSettings::ScheduleRule& rule) const;
    QString deviceActionText(const SchedulerSettings::DeviceSetAction& action) const;
    QString featureActionText(const SchedulerSettings::FeatureAction& action) const;
    QString runActionText(SchedulerSettings::RunAction action) const;
    QString recurrenceText(SchedulerSettings::Recurrence recurrence) const;
    QString deviceSetId(int deviceSetIndex) const;

private slots:
    void onMenuDialogCalled(const QPoint &p);
    void onWidgetRolled(QWidget* widget, bool rollDown);
    void handleInputMessages();
    void channelsOrFeaturesChanged(const QStringList& renameFrom, const QStringList& renameTo, const QStringList& removed, const QStringList& added);
    void onRuleSelectionChanged();
    void onRuleItemChanged(QTableWidgetItem *item);
    void onDeviceActionSelectionChanged();
    void onFeatureActionSelectionChanged();
    void onAddRule();
    void onDeleteRule();
    void onAddDeviceAction();
    void onDeleteDeviceAction();
    void onAddFeatureAction();
    void onDeleteFeatureAction();
    void onRuleEditorChanged();
    void onDeviceEditorChanged();
    void onFeatureEditorChanged();
    void onRefreshLists();
};

#endif // INCLUDE_FEATURE_SCHEDULERGUI_H_
