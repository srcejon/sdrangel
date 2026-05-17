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

#include <QDateTimeEdit>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QLineEdit>
#include <QRegularExpression>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolButton>

#include "device/deviceset.h"
#include "feature/feature.h"
#include "feature/featureset.h"
#include "feature/featureuiset.h"
#include "gui/basicfeaturesettingsdialog.h"
#include "gui/dialogpositioner.h"
#include "maincore.h"
#include "settings/mainsettings.h"
#include "settings/preset.h"

#include "ui_schedulergui.h"
#include "scheduler.h"
#include "schedulergui.h"

namespace
{
constexpr int PresetNone = -1;
constexpr int PresetUnresolved = -2;

QString presetText(const QString& group, quint64 frequency, const QString& description)
{
    if (group.isEmpty()) {
        return QString();
    }

    return QStringLiteral("%1: %2 - %3")
        .arg(group)
        .arg(frequency / 1000000.0, 0, 'f', 3)
        .arg(description);
}

QString featureKey(int featureSetIndex, int featureIndex)
{
    return QStringLiteral("%1:%2").arg(featureSetIndex).arg(featureIndex);
}

bool parseFeatureKey(const QString& key, int& featureSetIndex, int& featureIndex)
{
    const QStringList parts = key.split(':');
    if (parts.size() != 2) {
        return false;
    }

    bool okSet = false;
    bool okFeature = false;
    const int parsedSet = parts[0].toInt(&okSet);
    const int parsedFeature = parts[1].toInt(&okFeature);

    if (okSet && okFeature)
    {
        featureSetIndex = parsedSet;
        featureIndex = parsedFeature;
        return true;
    }

    return false;
}
}

SchedulerGUI* SchedulerGUI::create(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature)
{
    return new SchedulerGUI(pluginAPI, featureUISet, feature);
}

void SchedulerGUI::destroy()
{
    delete this;
}

SchedulerGUI::SchedulerGUI(PluginAPI* pluginAPI, FeatureUISet *featureUISet, Feature *feature, QWidget* parent) :
    FeatureGUI(parent),
    ui(new Ui::SchedulerGUI),
    m_scheduler(reinterpret_cast<Scheduler*>(feature)),
    m_eventSourceHandler(QStringList(), QStringLiteral("RTMF")),
    m_doApplySettings(true),
    m_populating(false),
    m_currentRule(-1),
    m_currentDeviceAction(-1),
    m_currentFeatureAction(-1)
{
    (void) pluginAPI;
    (void) featureUISet;

    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/scheduler/readme.md";

    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    rollupContents->arrangeRollups();
    connect(rollupContents, SIGNAL(widgetRolled(QWidget*,bool)), this, SLOT(onWidgetRolled(QWidget*,bool)));
    sizeToContents();

    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onMenuDialogCalled(const QPoint &)));

    m_scheduler->setMessageQueueToGUI(&m_inputMessageQueue);
    connect(getInputMessageQueue(), SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));

    connect(
        &m_eventSourceHandler,
        &AvailableChannelOrFeatureHandler::channelsOrFeaturesChanged,
        this,
        &SchedulerGUI::channelsOrFeaturesChanged
    );
    m_eventSourceHandler.scanAvailableChannelsAndFeatures();

    ui->rulesTable->setColumnCount(7);
    ui->rulesTable->setHorizontalHeaderLabels(QStringList({
        tr("On"),
        tr("Name"),
        tr("Trigger"),
        tr("Recurrence/Delay"),
        tr("Next run"),
        tr("Last run"),
        tr("Actions")
    }));
    ui->rulesTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->rulesTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->rulesTable->horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch);

    ui->deviceActionsTable->setColumnCount(4);
    ui->deviceActionsTable->setHorizontalHeaderLabels(QStringList({
        tr("Device set"),
        tr("Preset"),
        tr("Acq."),
        tr("File sinks")
    }));
    ui->deviceActionsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->deviceActionsTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);

    ui->featureActionsTable->setColumnCount(2);
    ui->featureActionsTable->setHorizontalHeaderLabels(QStringList({
        tr("Feature"),
        tr("Action")
    }));
    ui->featureActionsTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui->featureActionsTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);

    ui->triggerType->addItem(tr("Time"), SchedulerSettings::TriggerTime);
    ui->triggerType->addItem(tr("Event"), SchedulerSettings::TriggerEvent);

    ui->recurrence->addItem(tr("Once"), SchedulerSettings::RecurrenceOnce);
    ui->recurrence->addItem(tr("Daily"), SchedulerSettings::RecurrenceDaily);
    ui->recurrence->addItem(tr("Weekly"), SchedulerSettings::RecurrenceWeekly);
    ui->recurrence->addItem(tr("Monthly"), SchedulerSettings::RecurrenceMonthly);

    const QStringList eventNames = Scheduler::eventTypeNames();
    for (int i = 0; i < eventNames.size(); ++i) {
        ui->eventType->addItem(eventNames[i], i);
    }

    ui->eventDelayUnit->addItem(tr("seconds"), SchedulerSettings::DelaySeconds);
    ui->eventDelayUnit->addItem(tr("minutes"), SchedulerSettings::DelayMinutes);

    ui->acquisitionAction->addItem(tr("No change"), SchedulerSettings::ActionNoChange);
    ui->acquisitionAction->addItem(tr("Start"), SchedulerSettings::ActionStart);
    ui->acquisitionAction->addItem(tr("Stop"), SchedulerSettings::ActionStop);
    ui->fileSinkAction->addItem(tr("No change"), SchedulerSettings::ActionNoChange);
    ui->fileSinkAction->addItem(tr("Start"), SchedulerSettings::ActionStart);
    ui->fileSinkAction->addItem(tr("Stop"), SchedulerSettings::ActionStop);
    ui->featureAction->addItem(tr("Start"), SchedulerSettings::ActionStart);
    ui->featureAction->addItem(tr("Stop"), SchedulerSettings::ActionStop);

    ui->dateTime->setDisplayFormat(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
    ui->command->setToolTip(tr("Detached command. Supports ${rule}, ${trigger}, ${dateTime}, ${event}, ${source} and ${data}."));
    ui->speech->setToolTip(tr("Text to speak. Supports ${rule}, ${trigger}, ${dateTime}, ${event}, ${source} and ${data}."));
    ui->eventSource->setToolTip(tr("Optional event source. Leave empty to match all producers on the event pipe."));
    ui->eventDataRegex->setToolTip(tr("Optional regular expression matched against the event data string."));

    m_settings.setRollupState(&m_rollupState);

    makeUIConnections();
    displaySettings();
    applySettings(QStringList(), true);
    m_resizer.enableChildMouseTracking();
}

SchedulerGUI::~SchedulerGUI()
{
    delete ui;
}

void SchedulerGUI::makeUIConnections()
{
    connect(ui->rulesTable, &QTableWidget::itemChanged, this, &SchedulerGUI::onRuleItemChanged);
    connect(ui->rulesTable->selectionModel(), &QItemSelectionModel::selectionChanged, this, &SchedulerGUI::onRuleSelectionChanged);
    connect(ui->deviceActionsTable->selectionModel(), &QItemSelectionModel::selectionChanged, this, &SchedulerGUI::onDeviceActionSelectionChanged);
    connect(ui->featureActionsTable->selectionModel(), &QItemSelectionModel::selectionChanged, this, &SchedulerGUI::onFeatureActionSelectionChanged);

    connect(ui->addRule, &QToolButton::clicked, this, &SchedulerGUI::onAddRule);
    connect(ui->deleteRule, &QToolButton::clicked, this, &SchedulerGUI::onDeleteRule);
    connect(ui->refreshLists, &QToolButton::clicked, this, &SchedulerGUI::onRefreshLists);
    connect(ui->addDeviceAction, &QToolButton::clicked, this, &SchedulerGUI::onAddDeviceAction);
    connect(ui->deleteDeviceAction, &QToolButton::clicked, this, &SchedulerGUI::onDeleteDeviceAction);
    connect(ui->addFeatureAction, &QToolButton::clicked, this, &SchedulerGUI::onAddFeatureAction);
    connect(ui->deleteFeatureAction, &QToolButton::clicked, this, &SchedulerGUI::onDeleteFeatureAction);

    connect(ui->ruleName, &QLineEdit::editingFinished, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->ruleEnabled, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->triggerType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->dateTime, &QDateTimeEdit::dateTimeChanged, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->recurrence, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->eventType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->eventSource, &QComboBox::currentTextChanged, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->eventDataRegex, &QLineEdit::textChanged, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->eventDelay, QOverload<int>::of(&QSpinBox::valueChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->eventDelayUnit, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->command, &QLineEdit::editingFinished, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->speech, &QLineEdit::editingFinished, this, &SchedulerGUI::onRuleEditorChanged);

    connect(ui->deviceSet, &QComboBox::currentTextChanged, this, [this](const QString&) {
        if (!m_populating) {
            updatePresetList(currentDeviceAction());
            onDeviceEditorChanged();
        }
    });
    connect(ui->preset, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onDeviceEditorChanged);
    connect(ui->acquisitionAction, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onDeviceEditorChanged);
    connect(ui->fileSinkAction, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onDeviceEditorChanged);
    connect(ui->overrideFrequency, &QCheckBox::toggled, this, &SchedulerGUI::onDeviceEditorChanged);
    connect(ui->centerFrequency, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SchedulerGUI::onDeviceEditorChanged);

    connect(ui->featureSelect, &QComboBox::currentTextChanged, this, &SchedulerGUI::onFeatureEditorChanged);
    connect(ui->featureAction, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onFeatureEditorChanged);
}

bool SchedulerGUI::handleMessage(const Message& message)
{
    if (Scheduler::MsgConfigureScheduler::match(message))
    {
        const Scheduler::MsgConfigureScheduler& cfg = (const Scheduler::MsgConfigureScheduler&) message;
        if (cfg.getForce()) {
            m_settings = cfg.getSettings();
        } else {
            m_settings.applySettings(cfg.getSettingsKeys(), cfg.getSettings());
        }

        blockApplySettings(true);
        displaySettings();
        blockApplySettings(false);
        return true;
    }

    return false;
}

void SchedulerGUI::handleInputMessages()
{
    Message* message;

    while ((message = getInputMessageQueue()->pop()))
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void SchedulerGUI::resetToDefaults()
{
    m_settings.resetToDefaults();
    displaySettings();
    applySettings(QStringList(), true);
}

QByteArray SchedulerGUI::serialize() const
{
    return m_settings.serialize();
}

bool SchedulerGUI::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        m_feature->setWorkspaceIndex(m_settings.m_workspaceIndex);
        displaySettings();
        applySettings(QStringList(), true);
        return true;
    }

    resetToDefaults();
    return false;
}

void SchedulerGUI::setWorkspaceIndex(int index)
{
    m_settings.m_workspaceIndex = index;
    m_feature->setWorkspaceIndex(index);
}

void SchedulerGUI::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void SchedulerGUI::applySettings(const QStringList& settingsKeys, bool force)
{
    if (m_doApplySettings) {
        m_scheduler->getInputMessageQueue()->push(Scheduler::MsgConfigureScheduler::create(m_settings, settingsKeys, force));
    }
}

void SchedulerGUI::applyRules()
{
    applySettings(QStringList({QStringLiteral("rules")}));
}

void SchedulerGUI::displaySettings()
{
    setTitleColor(m_settings.m_rgbColor);
    setWindowTitle(m_settings.m_title);
    setTitle(m_settings.m_title);

    m_populating = true;
    getRollupContents()->restoreState(m_rollupState);
    refreshRulesTable();

    if (m_settings.m_rules.isEmpty()) {
        m_currentRule = -1;
    } else if ((m_currentRule < 0) || (m_currentRule >= m_settings.m_rules.size())) {
        m_currentRule = 0;
    }

    m_populating = false;
    selectRule(m_currentRule);
    displayRuleEditor();
}

void SchedulerGUI::refreshRulesTable()
{
    const int selectedRow = m_currentRule;
    const QDateTime now = QDateTime::currentDateTime();

    m_populating = true;
    ui->rulesTable->setRowCount(m_settings.m_rules.size());

    for (int row = 0; row < m_settings.m_rules.size(); ++row)
    {
        const SchedulerSettings::ScheduleRule& rule = m_settings.m_rules[row];

        QTableWidgetItem *enabledItem = new QTableWidgetItem();
        enabledItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        enabledItem->setCheckState(rule.m_enabled ? Qt::Checked : Qt::Unchecked);
        ui->rulesTable->setItem(row, 0, enabledItem);

        ui->rulesTable->setItem(row, 1, new QTableWidgetItem(rule.m_name));
        ui->rulesTable->setItem(row, 2, new QTableWidgetItem(ruleTriggerText(rule)));
        ui->rulesTable->setItem(row, 3, new QTableWidgetItem(ruleRecurrenceDelayText(rule)));

        const QDateTime nextRun = rule.m_triggerType == SchedulerSettings::TriggerTime
            ? SchedulerSettings::nextDateTime(rule, now)
            : QDateTime();
        ui->rulesTable->setItem(row, 4, new QTableWidgetItem(nextRun.isValid() ? nextRun.toString(Qt::ISODateWithMs) : QString()));
        ui->rulesTable->setItem(row, 5, new QTableWidgetItem(rule.m_lastRun.isValid() ? rule.m_lastRun.toString(Qt::ISODateWithMs) : QString()));
        ui->rulesTable->setItem(row, 6, new QTableWidgetItem(ruleActionSummary(rule)));
    }

    m_populating = false;

    if ((selectedRow >= 0) && (selectedRow < ui->rulesTable->rowCount())) {
        ui->rulesTable->selectRow(selectedRow);
    }
}

void SchedulerGUI::refreshDeviceActionsTable()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    const int actionCount = rule ? rule->m_deviceSetActions.size() : 0;

    m_populating = true;
    ui->deviceActionsTable->setRowCount(actionCount);

    for (int row = 0; row < actionCount; ++row)
    {
        const SchedulerSettings::DeviceSetAction& action = rule->m_deviceSetActions[row];
        ui->deviceActionsTable->setItem(row, 0, new QTableWidgetItem(deviceActionText(action)));
        ui->deviceActionsTable->setItem(row, 1, new QTableWidgetItem(presetText(action.m_presetGroup, action.m_presetFrequency, action.m_presetDescription)));
        ui->deviceActionsTable->setItem(row, 2, new QTableWidgetItem(runActionText(action.m_acquisitionAction)));
        ui->deviceActionsTable->setItem(row, 3, new QTableWidgetItem(runActionText(action.m_fileSinkAction)));
    }

    m_populating = false;

    if ((m_currentDeviceAction >= 0) && (m_currentDeviceAction < actionCount)) {
        ui->deviceActionsTable->selectRow(m_currentDeviceAction);
    }
}

void SchedulerGUI::refreshFeatureActionsTable()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    const int actionCount = rule ? rule->m_featureActions.size() : 0;

    m_populating = true;
    ui->featureActionsTable->setRowCount(actionCount);

    for (int row = 0; row < actionCount; ++row)
    {
        const SchedulerSettings::FeatureAction& action = rule->m_featureActions[row];
        ui->featureActionsTable->setItem(row, 0, new QTableWidgetItem(featureActionText(action)));
        ui->featureActionsTable->setItem(row, 1, new QTableWidgetItem(runActionText(action.m_action)));
    }

    m_populating = false;

    if ((m_currentFeatureAction >= 0) && (m_currentFeatureAction < actionCount)) {
        ui->featureActionsTable->selectRow(m_currentFeatureAction);
    }
}

void SchedulerGUI::displayRuleEditor()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    const bool hasRule = rule != nullptr;

    m_populating = true;
    ui->ruleEditor->setEnabled(hasRule);
    ui->timeGroup->setEnabled(hasRule);
    ui->eventGroup->setEnabled(hasRule);
    ui->commandGroup->setEnabled(hasRule);
    ui->deviceGroup->setEnabled(hasRule);
    ui->featureGroup->setEnabled(hasRule);

    if (hasRule)
    {
        ui->ruleName->setText(rule->m_name);
        ui->ruleEnabled->setChecked(rule->m_enabled);
        ui->triggerType->setCurrentIndex(ui->triggerType->findData(rule->m_triggerType));
        ui->dateTime->setDateTime(rule->m_time.isValid() ? rule->m_time : QDateTime::currentDateTime().addSecs(60));
        ui->recurrence->setCurrentIndex(ui->recurrence->findData(rule->m_recurrence));
        ui->eventType->setCurrentIndex(ui->eventType->findData(rule->m_eventType));
        updateEventSourceList(rule->m_eventSourceId);
        ui->eventDataRegex->setText(rule->m_eventDataRegex);
        ui->eventDelay->setValue(rule->m_eventDelay);
        ui->eventDelayUnit->setCurrentIndex(ui->eventDelayUnit->findData(rule->m_eventDelayUnit));
        ui->command->setText(rule->m_command);
        ui->speech->setText(rule->m_speech);
    }
    else
    {
        ui->ruleName->clear();
        ui->ruleEnabled->setChecked(false);
        ui->dateTime->setDateTime(QDateTime::currentDateTime().addSecs(60));
        updateEventSourceList(QString());
        ui->eventDataRegex->clear();
        ui->eventDelay->setValue(0);
        ui->command->clear();
        ui->speech->clear();
    }

    m_currentDeviceAction = hasRule && !rule->m_deviceSetActions.isEmpty() ? qBound(0, m_currentDeviceAction, rule->m_deviceSetActions.size() - 1) : -1;
    m_currentFeatureAction = hasRule && !rule->m_featureActions.isEmpty() ? qBound(0, m_currentFeatureAction, rule->m_featureActions.size() - 1) : -1;

    m_populating = false;
    updateTriggerVisibility();
    updateRegexState();
    refreshDeviceActionsTable();
    refreshFeatureActionsTable();
    selectDeviceAction(m_currentDeviceAction);
    selectFeatureAction(m_currentFeatureAction);
    displayDeviceActionEditor();
    displayFeatureActionEditor();
}

void SchedulerGUI::displayDeviceActionEditor()
{
    SchedulerSettings::DeviceSetAction *action = currentDeviceAction();
    const bool hasAction = action != nullptr;

    m_populating = true;
    ui->deviceSet->setEnabled(hasAction);
    ui->preset->setEnabled(hasAction);
    ui->acquisitionAction->setEnabled(hasAction);
    ui->fileSinkAction->setEnabled(hasAction);
    ui->overrideFrequency->setEnabled(hasAction);
    ui->centerFrequency->setEnabled(hasAction);

    updateDeviceSetList(action);
    updatePresetList(action);

    if (hasAction)
    {
        ui->acquisitionAction->setCurrentIndex(ui->acquisitionAction->findData(action->m_acquisitionAction));
        ui->fileSinkAction->setCurrentIndex(ui->fileSinkAction->findData(action->m_fileSinkAction));
        ui->overrideFrequency->setChecked(action->m_overrideCenterFrequency);
        ui->centerFrequency->setValue(action->m_centerFrequency / 1000000.0);
    }
    else
    {
        ui->acquisitionAction->setCurrentIndex(ui->acquisitionAction->findData(SchedulerSettings::ActionNoChange));
        ui->fileSinkAction->setCurrentIndex(ui->fileSinkAction->findData(SchedulerSettings::ActionNoChange));
        ui->overrideFrequency->setChecked(false);
        ui->centerFrequency->setValue(0.0);
    }

    m_populating = false;
}

void SchedulerGUI::displayFeatureActionEditor()
{
    SchedulerSettings::FeatureAction *action = currentFeatureAction();
    const bool hasAction = action != nullptr;

    m_populating = true;
    ui->featureSelect->setEnabled(hasAction);
    ui->featureAction->setEnabled(hasAction);
    updateFeatureList(action);

    if (hasAction) {
        ui->featureAction->setCurrentIndex(ui->featureAction->findData(action->m_action));
    } else {
        ui->featureAction->setCurrentIndex(ui->featureAction->findData(SchedulerSettings::ActionStart));
    }

    m_populating = false;
}

bool SchedulerGUI::updateCurrentRuleFromWidgets()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (!rule) {
        return false;
    }

    const int triggerType = ui->triggerType->currentData().toInt();
    const QString regex = ui->eventDataRegex->text();

    if ((triggerType == SchedulerSettings::TriggerEvent) && !regex.isEmpty())
    {
        const QRegularExpression re(regex);
        if (!re.isValid())
        {
            updateRegexState();
            return false;
        }
    }

    rule->m_name = ui->ruleName->text();
    rule->m_enabled = ui->ruleEnabled->isChecked();
    rule->m_triggerType = static_cast<SchedulerSettings::TriggerType>(triggerType);
    rule->m_time = ui->dateTime->dateTime();
    rule->m_recurrence = static_cast<SchedulerSettings::Recurrence>(ui->recurrence->currentData().toInt());
    rule->m_eventType = ui->eventType->currentData().toInt();

    const QVariant sourceData = ui->eventSource->currentData();
    if (sourceData.isValid() && !sourceData.toString().isEmpty()) {
        rule->m_eventSourceId = sourceData.toString();
    } else {
        rule->m_eventSourceId = ui->eventSource->currentText() == tr("Any") ? QString() : ui->eventSource->currentText().trimmed();
    }

    rule->m_eventDataRegex = regex;
    rule->m_eventDelay = ui->eventDelay->value();
    rule->m_eventDelayUnit = static_cast<SchedulerSettings::DelayUnit>(ui->eventDelayUnit->currentData().toInt());
    rule->m_command = ui->command->text();
    rule->m_speech = ui->speech->text();

    updateRegexState();
    return true;
}

void SchedulerGUI::updateCurrentDeviceActionFromWidgets()
{
    SchedulerSettings::DeviceSetAction *action = currentDeviceAction();
    if (!action) {
        return;
    }

    const QVariant deviceData = ui->deviceSet->currentData();
    bool parsedDeviceSet = false;

    if (deviceData.isValid())
    {
        action->m_deviceSetIndex = deviceData.toInt(&parsedDeviceSet);
        if (parsedDeviceSet) {
            action->m_deviceSetId = deviceSetId(action->m_deviceSetIndex);
        }
    }

    if (!parsedDeviceSet)
    {
        unsigned int deviceSetIndex = 0;
        if (MainCore::getDeviceSetIndexFromId(ui->deviceSet->currentText(), deviceSetIndex))
        {
            action->m_deviceSetIndex = static_cast<int>(deviceSetIndex);
            action->m_deviceSetId = ui->deviceSet->currentText();
        }
        else
        {
            action->m_deviceSetId = ui->deviceSet->currentText();
        }
    }

    const int presetIndex = ui->preset->currentData().toInt();
    if (presetIndex >= 0)
    {
        const Preset *preset = MainCore::instance()->getSettings().getPreset(presetIndex);
        action->m_presetGroup = preset->getGroup();
        action->m_presetFrequency = preset->getCenterFrequency();
        action->m_presetDescription = preset->getDescription();
    }
    else if (presetIndex == PresetNone)
    {
        action->m_presetGroup.clear();
        action->m_presetFrequency = 0;
        action->m_presetDescription.clear();
    }

    action->m_acquisitionAction = static_cast<SchedulerSettings::RunAction>(ui->acquisitionAction->currentData().toInt());
    action->m_fileSinkAction = static_cast<SchedulerSettings::RunAction>(ui->fileSinkAction->currentData().toInt());
    action->m_overrideCenterFrequency = ui->overrideFrequency->isChecked();
    action->m_centerFrequency = qRound64(ui->centerFrequency->value() * 1000000.0);
}

void SchedulerGUI::updateCurrentFeatureActionFromWidgets()
{
    SchedulerSettings::FeatureAction *action = currentFeatureAction();
    if (!action) {
        return;
    }

    const QString key = ui->featureSelect->currentData().toString();
    int featureSetIndex = 0;
    int featureIndex = 0;

    if (parseFeatureKey(key, featureSetIndex, featureIndex))
    {
        action->m_featureSetIndex = featureSetIndex;
        action->m_featureIndex = featureIndex;

        std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();
        if ((featureSetIndex >= 0) && (featureSetIndex < (int) featureSets.size())
            && (featureIndex >= 0) && (featureIndex < featureSets[featureSetIndex]->getNumberOfFeatures()))
        {
            action->m_featureId = featureSets[featureSetIndex]->getFeatureAt(featureIndex)->getURI();
        }
    }
    else
    {
        unsigned int parsedSet = 0;
        unsigned int parsedFeature = 0;
        if (MainCore::getFeatureIndexFromId(ui->featureSelect->currentText(), parsedSet, parsedFeature))
        {
            action->m_featureSetIndex = static_cast<int>(parsedSet);
            action->m_featureIndex = static_cast<int>(parsedFeature);
        }
    }

    action->m_action = static_cast<SchedulerSettings::RunAction>(ui->featureAction->currentData().toInt());
}

void SchedulerGUI::updateEventSourceList(const QString& selectedSource)
{
    ui->eventSource->clear();
    ui->eventSource->addItem(tr("Any"), QString());

    bool found = selectedSource.isEmpty();
    const AvailableChannelOrFeatureList& entries = m_eventSourceHandler.getAvailableChannelOrFeatureList();
    for (const AvailableChannelOrFeature& entry : entries)
    {
        const QString longId = entry.getLongId();
        ui->eventSource->addItem(longId, longId);
        if (longId == selectedSource) {
            found = true;
        }
    }

    if (!found) {
        ui->eventSource->addItem(selectedSource, selectedSource);
    }

    const int index = selectedSource.isEmpty() ? 0 : ui->eventSource->findData(selectedSource);
    ui->eventSource->setCurrentIndex(index >= 0 ? index : 0);
}

void SchedulerGUI::updateDeviceSetList(const SchedulerSettings::DeviceSetAction *selectedAction)
{
    ui->deviceSet->clear();

    const std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
    bool found = false;

    for (int i = 0; i < (int) deviceSets.size(); ++i)
    {
        const QString id = deviceSetId(i);
        ui->deviceSet->addItem(id, i);

        if (selectedAction && (selectedAction->m_deviceSetIndex == i)) {
            found = true;
        }
    }

    if (selectedAction && !found && !selectedAction->m_deviceSetId.isEmpty()) {
        ui->deviceSet->addItem(selectedAction->m_deviceSetId, QVariant());
    }

    if (selectedAction)
    {
        const int index = ui->deviceSet->findData(selectedAction->m_deviceSetIndex);
        if (index >= 0) {
            ui->deviceSet->setCurrentIndex(index);
        } else if (!selectedAction->m_deviceSetId.isEmpty()) {
            ui->deviceSet->setCurrentText(selectedAction->m_deviceSetId);
        }
    }
}

void SchedulerGUI::updatePresetList(const SchedulerSettings::DeviceSetAction *selectedAction)
{
    ui->preset->clear();
    ui->preset->addItem(QString(), PresetNone);

    QChar deviceType;
    const QString text = ui->deviceSet->currentText();
    if (text.isEmpty()) {
        deviceType = 'R';
    } else {
        deviceType = text.at(0);
    }

    const MainSettings& mainSettings = MainCore::instance()->getSettings();
    const int count = mainSettings.getPresetCount();
    bool found = false;

    for (int i = 0; i < count; ++i)
    {
        const Preset *preset = mainSettings.getPreset(i);
        if (((preset->isSourcePreset()) && (deviceType == 'R'))
            || ((preset->isSinkPreset()) && (deviceType == 'T'))
            || ((preset->isMIMOPreset()) && (deviceType == 'M')))
        {
            ui->preset->addItem(presetText(preset->getGroup(), preset->getCenterFrequency(), preset->getDescription()), i);
            if (selectedAction
                && (selectedAction->m_presetGroup == preset->getGroup())
                && (selectedAction->m_presetFrequency == preset->getCenterFrequency())
                && (selectedAction->m_presetDescription == preset->getDescription()))
            {
                found = true;
            }
        }
    }

    if (selectedAction && !selectedAction->m_presetGroup.isEmpty() && !found)
    {
        ui->preset->addItem(
            presetText(selectedAction->m_presetGroup, selectedAction->m_presetFrequency, selectedAction->m_presetDescription) + tr(" (unresolved)"),
            PresetUnresolved);
    }

    if (selectedAction && !selectedAction->m_presetGroup.isEmpty())
    {
        int selectedIndex = -1;
        for (int i = 0; i < ui->preset->count(); ++i)
        {
            if (ui->preset->itemData(i).toInt() >= 0)
            {
                const Preset *preset = mainSettings.getPreset(ui->preset->itemData(i).toInt());
                if ((selectedAction->m_presetGroup == preset->getGroup())
                    && (selectedAction->m_presetFrequency == preset->getCenterFrequency())
                    && (selectedAction->m_presetDescription == preset->getDescription()))
                {
                    selectedIndex = i;
                    break;
                }
            }
        }

        if (selectedIndex < 0) {
            selectedIndex = ui->preset->findData(PresetUnresolved);
        }

        ui->preset->setCurrentIndex(selectedIndex >= 0 ? selectedIndex : 0);
    }
    else
    {
        ui->preset->setCurrentIndex(0);
    }
}

void SchedulerGUI::updateFeatureList(const SchedulerSettings::FeatureAction *selectedAction)
{
    ui->featureSelect->clear();

    std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();
    bool found = false;

    for (int fsi = 0; fsi < (int) featureSets.size(); ++fsi)
    {
        FeatureSet *featureSet = featureSets[fsi];
        for (int fi = 0; fi < featureSet->getNumberOfFeatures(); ++fi)
        {
            Feature *feature = featureSet->getFeatureAt(fi);
            QString title;
            feature->getTitle(title);
            const QString text = QStringLiteral("F%1:%2 %3 (%4)")
                .arg(fsi)
                .arg(fi)
                .arg(feature->getIdentifier())
                .arg(title);

            ui->featureSelect->addItem(text, featureKey(fsi, fi));

            if (selectedAction && (selectedAction->m_featureSetIndex == fsi) && (selectedAction->m_featureIndex == fi)) {
                found = true;
            }
        }
    }

    if (selectedAction && !found)
    {
        ui->featureSelect->addItem(
            QStringLiteral("F%1:%2 %3 (unresolved)")
                .arg(selectedAction->m_featureSetIndex)
                .arg(selectedAction->m_featureIndex)
                .arg(selectedAction->m_featureId),
            QString());
    }

    if (selectedAction)
    {
        const int index = ui->featureSelect->findData(featureKey(selectedAction->m_featureSetIndex, selectedAction->m_featureIndex));
        if (index >= 0) {
            ui->featureSelect->setCurrentIndex(index);
        } else if (ui->featureSelect->count() > 0) {
            ui->featureSelect->setCurrentIndex(ui->featureSelect->count() - 1);
        }
    }
}

void SchedulerGUI::updateTriggerVisibility()
{
    const bool isTime = ui->triggerType->currentData().toInt() == SchedulerSettings::TriggerTime;
    ui->timeGroup->setVisible(isTime);
    ui->eventGroup->setVisible(!isTime);
    getRollupContents()->arrangeRollups();
}

void SchedulerGUI::updateRegexState()
{
    const QString regex = ui->eventDataRegex->text();
    const bool eventRule = ui->triggerType->currentData().toInt() == SchedulerSettings::TriggerEvent;
    const bool valid = !eventRule || regex.isEmpty() || QRegularExpression(regex).isValid();

    ui->eventDataRegex->setStyleSheet(valid ? QString() : QStringLiteral("QLineEdit { background-color: rgb(120, 40, 40); }"));
    ui->eventDataRegex->setToolTip(valid
        ? tr("Optional regular expression matched against the event data string.")
        : tr("Invalid regular expression. The rule will not be applied until this is fixed."));
}

void SchedulerGUI::selectRule(int row)
{
    if ((row >= 0) && (row < ui->rulesTable->rowCount()))
    {
        m_currentRule = row;
        ui->rulesTable->selectRow(row);
    }
    else
    {
        m_currentRule = -1;
        ui->rulesTable->clearSelection();
    }
}

void SchedulerGUI::selectDeviceAction(int row)
{
    if ((row >= 0) && (row < ui->deviceActionsTable->rowCount()))
    {
        m_currentDeviceAction = row;
        ui->deviceActionsTable->selectRow(row);
    }
    else
    {
        m_currentDeviceAction = -1;
        ui->deviceActionsTable->clearSelection();
    }
}

void SchedulerGUI::selectFeatureAction(int row)
{
    if ((row >= 0) && (row < ui->featureActionsTable->rowCount()))
    {
        m_currentFeatureAction = row;
        ui->featureActionsTable->selectRow(row);
    }
    else
    {
        m_currentFeatureAction = -1;
        ui->featureActionsTable->clearSelection();
    }
}

SchedulerSettings::ScheduleRule *SchedulerGUI::currentRule()
{
    if ((m_currentRule >= 0) && (m_currentRule < m_settings.m_rules.size())) {
        return &m_settings.m_rules[m_currentRule];
    }

    return nullptr;
}

SchedulerSettings::DeviceSetAction *SchedulerGUI::currentDeviceAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (rule && (m_currentDeviceAction >= 0) && (m_currentDeviceAction < rule->m_deviceSetActions.size())) {
        return &rule->m_deviceSetActions[m_currentDeviceAction];
    }

    return nullptr;
}

SchedulerSettings::FeatureAction *SchedulerGUI::currentFeatureAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (rule && (m_currentFeatureAction >= 0) && (m_currentFeatureAction < rule->m_featureActions.size())) {
        return &rule->m_featureActions[m_currentFeatureAction];
    }

    return nullptr;
}

const Preset *SchedulerGUI::selectedPreset() const
{
    const int presetIndex = ui->preset->currentData().toInt();
    if (presetIndex < 0) {
        return nullptr;
    }

    return MainCore::instance()->getSettings().getPreset(presetIndex);
}

QString SchedulerGUI::ruleTriggerText(const SchedulerSettings::ScheduleRule& rule) const
{
    if (rule.m_triggerType == SchedulerSettings::TriggerTime) {
        return rule.m_time.toString(Qt::ISODateWithMs);
    }

    QString text = Scheduler::eventTypeName(rule.m_eventType);
    if (!rule.m_eventSourceId.isEmpty()) {
        text += QStringLiteral(" from %1").arg(rule.m_eventSourceId);
    }
    if (!rule.m_eventDataRegex.isEmpty()) {
        text += QStringLiteral(" /%1/").arg(rule.m_eventDataRegex);
    }
    return text;
}

QString SchedulerGUI::ruleRecurrenceDelayText(const SchedulerSettings::ScheduleRule& rule) const
{
    if (rule.m_triggerType == SchedulerSettings::TriggerTime) {
        return recurrenceText(rule.m_recurrence);
    }

    const int delay = SchedulerSettings::delaySeconds(rule);
    if (delay == 0) {
        return QString();
    }
    if (rule.m_eventDelayUnit == SchedulerSettings::DelayMinutes) {
        return tr("%1 min").arg(rule.m_eventDelay);
    }
    return tr("%1 s").arg(delay);
}

QString SchedulerGUI::ruleActionSummary(const SchedulerSettings::ScheduleRule& rule) const
{
    QStringList parts;

    if (!rule.m_deviceSetActions.isEmpty()) {
        parts.append(tr("%1 device").arg(rule.m_deviceSetActions.size()));
    }
    if (!rule.m_featureActions.isEmpty()) {
        parts.append(tr("%1 feature").arg(rule.m_featureActions.size()));
    }
    if (!rule.m_command.isEmpty()) {
        parts.append(tr("command"));
    }
    if (!rule.m_speech.isEmpty()) {
        parts.append(tr("speech"));
    }

    return parts.join(QStringLiteral(", "));
}

QString SchedulerGUI::deviceActionText(const SchedulerSettings::DeviceSetAction& action) const
{
    if (!action.m_deviceSetId.isEmpty()) {
        return action.m_deviceSetId;
    }

    return deviceSetId(action.m_deviceSetIndex);
}

QString SchedulerGUI::featureActionText(const SchedulerSettings::FeatureAction& action) const
{
    std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();
    if ((action.m_featureSetIndex >= 0) && (action.m_featureSetIndex < (int) featureSets.size())
        && (action.m_featureIndex >= 0) && (action.m_featureIndex < featureSets[action.m_featureSetIndex]->getNumberOfFeatures()))
    {
        Feature *feature = featureSets[action.m_featureSetIndex]->getFeatureAt(action.m_featureIndex);
        return QStringLiteral("F%1:%2 %3")
            .arg(action.m_featureSetIndex)
            .arg(action.m_featureIndex)
            .arg(feature->getIdentifier());
    }

    return QStringLiteral("F%1:%2 %3")
        .arg(action.m_featureSetIndex)
        .arg(action.m_featureIndex)
        .arg(action.m_featureId);
}

QString SchedulerGUI::runActionText(SchedulerSettings::RunAction action) const
{
    switch (action)
    {
    case SchedulerSettings::ActionStart:
        return tr("Start");
    case SchedulerSettings::ActionStop:
        return tr("Stop");
    case SchedulerSettings::ActionNoChange:
    default:
        return tr("No change");
    }
}

QString SchedulerGUI::recurrenceText(SchedulerSettings::Recurrence recurrence) const
{
    switch (recurrence)
    {
    case SchedulerSettings::RecurrenceDaily:
        return tr("Daily");
    case SchedulerSettings::RecurrenceWeekly:
        return tr("Weekly");
    case SchedulerSettings::RecurrenceMonthly:
        return tr("Monthly");
    case SchedulerSettings::RecurrenceOnce:
    default:
        return tr("Once");
    }
}

QString SchedulerGUI::deviceSetId(int deviceSetIndex) const
{
    const std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
    if ((deviceSetIndex >= 0) && (deviceSetIndex < (int) deviceSets.size())) {
        return MainCore::instance()->getDeviceSetId(deviceSets[deviceSetIndex]);
    }

    return QStringLiteral("R%1").arg(deviceSetIndex);
}

void SchedulerGUI::onMenuDialogCalled(const QPoint &p)
{
    if (m_contextMenuType == ContextMenuType::ContextMenuChannelSettings)
    {
        BasicFeatureSettingsDialog dialog(this);
        dialog.setTitle(m_settings.m_title);
        dialog.setUseReverseAPI(false);
        dialog.setReverseAPIAddress(QStringLiteral("127.0.0.1"));
        dialog.setReverseAPIPort(8888);
        dialog.setReverseAPIFeatureSetIndex(0);
        dialog.setReverseAPIFeatureIndex(0);
        dialog.setDefaultTitle(m_displayedName);

        dialog.move(p);
        new DialogPositioner(&dialog, false);
        dialog.exec();

        if (dialog.hasChanged())
        {
            m_settings.m_title = dialog.getTitle();
            setTitle(m_settings.m_title);
            setWindowTitle(m_settings.m_title);
            applySettings(QStringList({QStringLiteral("title")}));
        }
    }

    resetContextMenuType();
}

void SchedulerGUI::onWidgetRolled(QWidget* widget, bool rollDown)
{
    (void) widget;
    (void) rollDown;

    getRollupContents()->saveState(m_rollupState);
}

void SchedulerGUI::channelsOrFeaturesChanged(const QStringList& renameFrom, const QStringList& renameTo, const QStringList& removed, const QStringList& added)
{
    (void) removed;
    (void) added;

    bool renamed = false;
    for (SchedulerSettings::ScheduleRule& rule : m_settings.m_rules)
    {
        const int renameIndex = renameFrom.indexOf(rule.m_eventSourceId);
        if (renameIndex >= 0)
        {
            rule.m_eventSourceId = renameTo[renameIndex];
            renamed = true;
        }
    }

    displayRuleEditor();

    if (renamed) {
        applyRules();
    }
}

void SchedulerGUI::onRuleSelectionChanged()
{
    if (m_populating) {
        return;
    }

    const QList<QTableWidgetSelectionRange> ranges = ui->rulesTable->selectedRanges();
    m_currentRule = ranges.isEmpty() ? -1 : ranges.first().topRow();
    m_currentDeviceAction = -1;
    m_currentFeatureAction = -1;
    displayRuleEditor();
}

void SchedulerGUI::onRuleItemChanged(QTableWidgetItem *item)
{
    if (m_populating || !item || (item->column() != 0)) {
        return;
    }

    const int row = item->row();
    if ((row < 0) || (row >= m_settings.m_rules.size())) {
        return;
    }

    m_settings.m_rules[row].m_enabled = item->checkState() == Qt::Checked;
    m_currentRule = row;
    applyRules();
    refreshRulesTable();
}

void SchedulerGUI::onDeviceActionSelectionChanged()
{
    if (m_populating) {
        return;
    }

    const QList<QTableWidgetSelectionRange> ranges = ui->deviceActionsTable->selectedRanges();
    m_currentDeviceAction = ranges.isEmpty() ? -1 : ranges.first().topRow();
    displayDeviceActionEditor();
}

void SchedulerGUI::onFeatureActionSelectionChanged()
{
    if (m_populating) {
        return;
    }

    const QList<QTableWidgetSelectionRange> ranges = ui->featureActionsTable->selectedRanges();
    m_currentFeatureAction = ranges.isEmpty() ? -1 : ranges.first().topRow();
    displayFeatureActionEditor();
}

void SchedulerGUI::onAddRule()
{
    SchedulerSettings::ScheduleRule rule;
    rule.m_name = tr("Rule %1").arg(m_settings.m_rules.size() + 1);
    m_settings.m_rules.append(rule);
    m_currentRule = m_settings.m_rules.size() - 1;
    m_currentDeviceAction = -1;
    m_currentFeatureAction = -1;
    refreshRulesTable();
    selectRule(m_currentRule);
    displayRuleEditor();
    applyRules();
}

void SchedulerGUI::onDeleteRule()
{
    if ((m_currentRule < 0) || (m_currentRule >= m_settings.m_rules.size())) {
        return;
    }

    m_settings.m_rules.removeAt(m_currentRule);
    if (m_currentRule >= m_settings.m_rules.size()) {
        m_currentRule = m_settings.m_rules.size() - 1;
    }
    m_currentDeviceAction = -1;
    m_currentFeatureAction = -1;
    refreshRulesTable();
    selectRule(m_currentRule);
    displayRuleEditor();
    applyRules();
}

void SchedulerGUI::onAddDeviceAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (!rule) {
        return;
    }

    SchedulerSettings::DeviceSetAction action;
    const std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
    if (!deviceSets.empty())
    {
        action.m_deviceSetIndex = 0;
        action.m_deviceSetId = deviceSetId(0);
    }

    rule->m_deviceSetActions.append(action);
    m_currentDeviceAction = rule->m_deviceSetActions.size() - 1;
    refreshDeviceActionsTable();
    selectDeviceAction(m_currentDeviceAction);
    displayDeviceActionEditor();
    refreshRulesTable();
    applyRules();
}

void SchedulerGUI::onDeleteDeviceAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (!rule || (m_currentDeviceAction < 0) || (m_currentDeviceAction >= rule->m_deviceSetActions.size())) {
        return;
    }

    rule->m_deviceSetActions.removeAt(m_currentDeviceAction);
    if (m_currentDeviceAction >= rule->m_deviceSetActions.size()) {
        m_currentDeviceAction = rule->m_deviceSetActions.size() - 1;
    }
    refreshDeviceActionsTable();
    selectDeviceAction(m_currentDeviceAction);
    displayDeviceActionEditor();
    refreshRulesTable();
    applyRules();
}

void SchedulerGUI::onAddFeatureAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (!rule) {
        return;
    }

    SchedulerSettings::FeatureAction action;
    std::vector<FeatureSet*>& featureSets = MainCore::instance()->getFeatureeSets();

    for (int fsi = 0; fsi < (int) featureSets.size(); ++fsi)
    {
        if (featureSets[fsi]->getNumberOfFeatures() > 0)
        {
            action.m_featureSetIndex = fsi;
            action.m_featureIndex = 0;
            action.m_featureId = featureSets[fsi]->getFeatureAt(0)->getURI();
            break;
        }
    }

    rule->m_featureActions.append(action);
    m_currentFeatureAction = rule->m_featureActions.size() - 1;
    refreshFeatureActionsTable();
    selectFeatureAction(m_currentFeatureAction);
    displayFeatureActionEditor();
    refreshRulesTable();
    applyRules();
}

void SchedulerGUI::onDeleteFeatureAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (!rule || (m_currentFeatureAction < 0) || (m_currentFeatureAction >= rule->m_featureActions.size())) {
        return;
    }

    rule->m_featureActions.removeAt(m_currentFeatureAction);
    if (m_currentFeatureAction >= rule->m_featureActions.size()) {
        m_currentFeatureAction = rule->m_featureActions.size() - 1;
    }
    refreshFeatureActionsTable();
    selectFeatureAction(m_currentFeatureAction);
    displayFeatureActionEditor();
    refreshRulesTable();
    applyRules();
}

void SchedulerGUI::onRuleEditorChanged()
{
    if (!m_doApplySettings || m_populating) {
        return;
    }

    const int row = m_currentRule;
    if (!updateCurrentRuleFromWidgets()) {
        return;
    }

    updateTriggerVisibility();
    refreshRulesTable();
    selectRule(row);
    applyRules();
}

void SchedulerGUI::onDeviceEditorChanged()
{
    if (!m_doApplySettings || m_populating) {
        return;
    }

    const int row = m_currentDeviceAction;
    updateCurrentDeviceActionFromWidgets();
    refreshDeviceActionsTable();
    selectDeviceAction(row);
    refreshRulesTable();
    selectRule(m_currentRule);
    applyRules();
}

void SchedulerGUI::onFeatureEditorChanged()
{
    if (!m_doApplySettings || m_populating) {
        return;
    }

    const int row = m_currentFeatureAction;
    updateCurrentFeatureActionFromWidgets();
    refreshFeatureActionsTable();
    selectFeatureAction(row);
    refreshRulesTable();
    selectRule(m_currentRule);
    applyRules();
}

void SchedulerGUI::onRefreshLists()
{
    m_eventSourceHandler.scanAvailableChannelsAndFeatures();
    displayRuleEditor();
}
