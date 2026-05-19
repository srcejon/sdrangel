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

#include <QDate>

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
class QLayout;
class QLineEdit;
class QSpinBox;
class QTableWidget;
class QTableWidgetItem;
class QToolButton;
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
    int m_currentChannelAction;
    int m_currentFeatureAction;
    QGroupBox *m_channelTextGroup;
    QLineEdit *m_channelText;
    QGroupBox *m_channelDataGroup;
    QLineEdit *m_channelData;
    QGroupBox *m_packetActionGroup;
    QLineEdit *m_packetCallsign;
    QLineEdit *m_packetTo;
    QLineEdit *m_packetVia;
    QLineEdit *m_packetData;
    QGroupBox *m_cameraActionGroup;
    QLineEdit *m_cameraFilename;
    QComboBox *m_cameraRecordMode;
    QLabel *m_cameraImageCountLabel;
    QSpinBox *m_cameraImageCount;
    QLabel *m_cameraVideoDurationLabel;
    QSpinBox *m_cameraVideoDuration;
    QGroupBox *m_findActionGroup;
    QLineEdit *m_findTarget;
    QGroupBox *m_deviceSettingsGroup;
    QTableWidget *m_deviceSettingsTable;
    QToolButton *m_addDeviceSetting;
    QToolButton *m_deleteDeviceSetting;
    QGroupBox *m_channelSettingsGroup;
    QTableWidget *m_channelSettingsTable;
    QToolButton *m_addChannelSetting;
    QToolButton *m_deleteChannelSetting;
    QGroupBox *m_featureSettingsGroup;
    QTableWidget *m_featureSettingsTable;
    QToolButton *m_addFeatureSetting;
    QToolButton *m_deleteFeatureSetting;

    explicit SchedulerGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent = nullptr);
    ~SchedulerGUI() override;

    void makeUIConnections();
    void blockApplySettings(bool block);
    void applySettings(const QStringList& settingsKeys, bool force = false);
    void applyRules();
    void displaySettings();
    void refreshRulesTable();
    void refreshDeviceActionsTable();
    void refreshChannelActionsTable();
    void refreshFeatureActionsTable();
    void displayRuleEditor();
    void displayDeviceActionEditor();
    void displayChannelActionEditor();
    void displayFeatureActionEditor();
    void createChannelActionParameterEditors();
    void createFeatureActionParameterEditors();
    void createDeviceActionParameterEditors();
    void createApplySettingsEditor(QGroupBox *&group, QTableWidget *&table, QToolButton *&addButton, QToolButton *&deleteButton, QWidget *parent, QLayout *targetLayout);
    void updateChannelActionParameterVisibility();
    void updateFeatureActionParameterVisibility();
    void updateDeviceActionParameterVisibility();
    bool updateCurrentRuleFromWidgets();
    void updateCurrentDeviceActionFromWidgets();
    void updateCurrentChannelActionFromWidgets();
    void updateCurrentFeatureActionFromWidgets();
    QList<SchedulerSettings::SettingValue> settingValuesFromTable(QTableWidget *table) const;
    void setSettingValuesToTable(QTableWidget *table, const QList<SchedulerSettings::SettingValue>& settings, void (SchedulerGUI::*editorChanged)());
    void connectSettingTable(QTableWidget *table, QToolButton *addButton, QToolButton *deleteButton, void (SchedulerGUI::*editorChanged)());
    void updateEventSourceList(const QString& selectedSource);
    void updateDeviceSetList(const SchedulerSettings::DeviceSetAction *selectedAction);
    void updatePresetList(const SchedulerSettings::DeviceSetAction *selectedAction);
    void updateChannelDeviceSetList(const SchedulerSettings::ChannelAction *selectedAction);
    void updateChannelList(const SchedulerSettings::ChannelAction *selectedAction);
    void updateChannelActionList(const SchedulerSettings::ChannelAction *selectedAction);
    void updateFeatureList(const SchedulerSettings::FeatureAction *selectedAction);
    void updateFeatureActionList(const SchedulerSettings::FeatureAction *selectedAction);
    void updateTriggerVisibility();
    void updateTimeScheduleVisibility();
    void updateRegexState();
    void selectRule(int row);
    void selectDeviceAction(int row);
    void selectChannelAction(int row);
    void selectFeatureAction(int row);
    SchedulerSettings::ScheduleRule *currentRule();
    SchedulerSettings::DeviceSetAction *currentDeviceAction();
    SchedulerSettings::ChannelAction *currentChannelAction();
    SchedulerSettings::FeatureAction *currentFeatureAction();
    const Preset *selectedPreset() const;
    const SchedulerSettings::DeviceSetAction *deviceSetActionForChannelAction(const SchedulerSettings::ChannelAction *action) const;
    const Preset *presetForDeviceSetAction(const SchedulerSettings::DeviceSetAction *action) const;
    QString ruleTriggerText(const SchedulerSettings::ScheduleRule& rule) const;
    QString ruleRecurrenceDelayText(const SchedulerSettings::ScheduleRule& rule) const;
    QString ruleActionSummary(const SchedulerSettings::ScheduleRule& rule) const;
    QString durationText(const SchedulerSettings::ScheduleRule& rule) const;
    QString deviceActionText(const SchedulerSettings::DeviceSetAction& action) const;
    QString deviceActionText(const SchedulerSettings::ChannelAction& action) const;
    QString channelActionText(const SchedulerSettings::ChannelAction& action) const;
    QString featureActionText(const SchedulerSettings::FeatureAction& action) const;
    QString runActionText(SchedulerSettings::RunAction action) const;
    QString recurrenceText(SchedulerSettings::Recurrence recurrence) const;
    QString deviceSetId(int deviceSetIndex) const;
    static constexpr int PresetNone = -1;
    static constexpr int PresetUnresolved = -2;
    static constexpr int DefaultWeekdayMask = 0x7f;
    static QDate noDateUntil();
    static QString presetText(const QString& group, quint64 frequency, const QString& description);
    static QString featureKey(int featureSetIndex, int featureIndex);
    static QString channelKey(int channelIndex, const QString& channelId);
    static bool channelSupportsAction(const QString& channelId, SchedulerSettings::RunAction action);
    static bool featureSupportsAction(const QString& featureId, SchedulerSettings::RunAction action);
    static int weekdayMaskFromWidgets(const Ui::SchedulerGUI *ui);
    static void setWeekdayWidgets(Ui::SchedulerGUI *ui, int mask);
    static bool ruleHasDeviceSetAction(const SchedulerSettings::ScheduleRule& rule, int deviceSetIndex);
    static void pruneChannelActionsForDeviceSets(SchedulerSettings::ScheduleRule& rule);
    static bool parseFeatureKey(const QString& key, int& featureSetIndex, int& featureIndex);

private slots:
    void onMenuDialogCalled(const QPoint &p);
    void onWidgetRolled(QWidget* widget, bool rollDown);
    void handleInputMessages();
    void channelsOrFeaturesChanged(const QStringList& renameFrom, const QStringList& renameTo, const QStringList& removed, const QStringList& added);
    void onRuleSelectionChanged();
    void onRuleItemChanged(QTableWidgetItem *item);
    void onDeviceActionSelectionChanged();
    void onChannelActionSelectionChanged();
    void onFeatureActionSelectionChanged();
    void onAddRule();
    void onDeleteRule();
    void onAddDeviceAction();
    void onDeleteDeviceAction();
    void onAddChannelAction();
    void onDeleteChannelAction();
    void onAddFeatureAction();
    void onDeleteFeatureAction();
    void onRuleEditorChanged();
    void onDeviceEditorChanged();
    void onChannelEditorChanged();
    void onFeatureEditorChanged();
    void onRefreshLists();
};

#endif // INCLUDE_FEATURE_SCHEDULERGUI_H_
