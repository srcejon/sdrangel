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

#include <QDateEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QAbstractItemView>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QItemSelectionModel>
#include <QLineEdit>
#include <QRegularExpression>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimeEdit>
#include <QToolButton>
#include <QVBoxLayout>

#include "device/deviceset.h"
#include "channel/channelapi.h"
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

QDate SchedulerGUI::noDateUntil()
{
    return QDate(1900, 1, 1);
}

QString SchedulerGUI::presetText(const QString& group, quint64 frequency, const QString& description)
{
    if (group.isEmpty()) {
        return QString();
    }

    return QStringLiteral("%1: %2 - %3")
        .arg(group)
        .arg(frequency / 1000000.0, 0, 'f', 3)
        .arg(description);
}

QString SchedulerGUI::featureKey(int featureSetIndex, int featureIndex)
{
    return QStringLiteral("%1:%2").arg(featureSetIndex).arg(featureIndex);
}

QString SchedulerGUI::channelKey(int channelIndex, const QString& channelId)
{
    return QStringLiteral("%1:%2").arg(channelIndex).arg(channelId);
}

bool SchedulerGUI::channelSupportsAction(const QString& channelId, SchedulerSettings::RunAction action)
{
    switch (action)
    {
    case SchedulerSettings::ActionApplySetting:
        return !channelId.isEmpty();
    case SchedulerSettings::ActionFileSinkRecordStart:
    case SchedulerSettings::ActionFileSinkRecordStop:
        return channelId == "sdrangel.channel.filesink";
    case SchedulerSettings::ActionSigMFRecordStart:
    case SchedulerSettings::ActionSigMFRecordStop:
        return channelId == "sdrangel.channel.sigmffilesink";
    case SchedulerSettings::ActionRTTYTransmit:
        return channelId == "sdrangel.channeltx.modrtty";
    case SchedulerSettings::ActionPSK31Transmit:
        return channelId == "sdrangel.channeltx.modpsk31";
    case SchedulerSettings::ActionPacketTransmit:
        return channelId == "sdrangel.channeltx.modpacket";
    case SchedulerSettings::ActionIEEE_802_15_4Transmit:
        return channelId == "sdrangel.channeltx.mod802.15.4";
    case SchedulerSettings::ActionAISTransmit:
        return channelId == "sdrangel.channel.modais";
    case SchedulerSettings::ActionFreqScannerRun:
    case SchedulerSettings::ActionFreqScannerStop:
        return channelId == "sdrangel.channel.freqscanner";
    case SchedulerSettings::ActionRadioAstronomyStart:
        return channelId == "sdrangel.channel.radioastronomy";
    default:
        return false;
    }
}

bool SchedulerGUI::featureSupportsAction(const QString& featureId, SchedulerSettings::RunAction action)
{
    switch (action)
    {
    case SchedulerSettings::ActionStart:
    case SchedulerSettings::ActionStop:
        return !featureId.isEmpty();
    case SchedulerSettings::ActionCameraSaveImage:
    case SchedulerSettings::ActionCameraRecordVideo:
        return featureId == "sdrangel.feature.camera";
    case SchedulerSettings::ActionMapFind:
        return (featureId == "sdrangel.feature.map") || (featureId == "sdrangel.feature.skymap");
    case SchedulerSettings::ActionApplySetting:
        return !featureId.isEmpty();
    default:
        return false;
    }
}

int SchedulerGUI::weekdayMaskFromWidgets(const Ui::SchedulerGUI *ui)
{
    int mask = 0;

    if (ui->monday->isChecked()) {
        mask |= 1 << 0;
    }
    if (ui->tuesday->isChecked()) {
        mask |= 1 << 1;
    }
    if (ui->wednesday->isChecked()) {
        mask |= 1 << 2;
    }
    if (ui->thursday->isChecked()) {
        mask |= 1 << 3;
    }
    if (ui->friday->isChecked()) {
        mask |= 1 << 4;
    }
    if (ui->saturday->isChecked()) {
        mask |= 1 << 5;
    }
    if (ui->sunday->isChecked()) {
        mask |= 1 << 6;
    }

    return mask;
}

void SchedulerGUI::setWeekdayWidgets(Ui::SchedulerGUI *ui, int mask)
{
    ui->monday->setChecked((mask & (1 << 0)) != 0);
    ui->tuesday->setChecked((mask & (1 << 1)) != 0);
    ui->wednesday->setChecked((mask & (1 << 2)) != 0);
    ui->thursday->setChecked((mask & (1 << 3)) != 0);
    ui->friday->setChecked((mask & (1 << 4)) != 0);
    ui->saturday->setChecked((mask & (1 << 5)) != 0);
    ui->sunday->setChecked((mask & (1 << 6)) != 0);
}

bool SchedulerGUI::ruleHasDeviceSetAction(const SchedulerSettings::ScheduleRule& rule, int deviceSetIndex)
{
    for (const SchedulerSettings::DeviceSetAction& action : rule.m_deviceSetActions)
    {
        if (action.m_deviceSetIndex == deviceSetIndex) {
            return true;
        }
    }

    return false;
}

void SchedulerGUI::pruneChannelActionsForDeviceSets(SchedulerSettings::ScheduleRule& rule)
{
    for (int i = rule.m_channelActions.size() - 1; i >= 0; --i)
    {
        if (!ruleHasDeviceSetAction(rule, rule.m_channelActions[i].m_deviceSetIndex)) {
            rule.m_channelActions.removeAt(i);
        }
    }
}

bool SchedulerGUI::parseFeatureKey(const QString& key, int& featureSetIndex, int& featureIndex)
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
    m_currentChannelAction(-1),
    m_currentFeatureAction(-1),
    m_channelTextGroup(nullptr),
    m_channelText(nullptr),
    m_channelDataGroup(nullptr),
    m_channelData(nullptr),
    m_packetActionGroup(nullptr),
    m_packetCallsign(nullptr),
    m_packetTo(nullptr),
    m_packetVia(nullptr),
    m_packetData(nullptr),
    m_cameraActionGroup(nullptr),
    m_cameraFilename(nullptr),
    m_cameraRecordMode(nullptr),
    m_cameraImageCountLabel(nullptr),
    m_cameraImageCount(nullptr),
    m_cameraVideoDurationLabel(nullptr),
    m_cameraVideoDuration(nullptr),
    m_findActionGroup(nullptr),
    m_findTarget(nullptr),
    m_deviceSettingsGroup(nullptr),
    m_deviceSettingsTable(nullptr),
    m_addDeviceSetting(nullptr),
    m_deleteDeviceSetting(nullptr),
    m_channelSettingsGroup(nullptr),
    m_channelSettingsTable(nullptr),
    m_addChannelSetting(nullptr),
    m_deleteChannelSetting(nullptr),
    m_featureSettingsGroup(nullptr),
    m_featureSettingsTable(nullptr),
    m_addFeatureSetting(nullptr),
    m_deleteFeatureSetting(nullptr)
{
    (void) pluginAPI;
    (void) featureUISet;

    m_feature = feature;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_helpURL = "plugins/feature/scheduler/readme.md";

    RollupContents *rollupContents = getRollupContents();
    ui->setupUi(rollupContents);
    createDeviceActionParameterEditors();
    createChannelActionParameterEditors();
    createFeatureActionParameterEditors();
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

    ui->channelActionsTable->setColumnCount(3);
    ui->channelActionsTable->setHorizontalHeaderLabels(QStringList({
        tr("Device set"),
        tr("Channel"),
        tr("Action")
    }));
    ui->channelActionsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->channelActionsTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);

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
    ui->recurrence->addItem(tr("Monthly"), SchedulerSettings::RecurrenceMonthly);

    const QStringList eventNames = Scheduler::eventTypeNames();
    for (int i = 0; i < eventNames.size(); ++i) {
        ui->eventType->addItem(eventNames[i], i);
    }

    ui->eventDelayUnit->addItem(tr("seconds"), SchedulerSettings::DelaySeconds);
    ui->eventDelayUnit->addItem(tr("minutes"), SchedulerSettings::DelayMinutes);
    ui->durationUnit->addItem(tr("seconds"), SchedulerSettings::DelaySeconds);
    ui->durationUnit->addItem(tr("minutes"), SchedulerSettings::DelayMinutes);

    ui->acquisitionAction->addItem(tr("No change"), SchedulerSettings::ActionNoChange);
    ui->acquisitionAction->addItem(tr("Start"), SchedulerSettings::ActionStart);
    ui->acquisitionAction->addItem(tr("Stop"), SchedulerSettings::ActionStop);
    ui->fileSinkAction->addItem(tr("No change"), SchedulerSettings::ActionNoChange);
    ui->fileSinkAction->addItem(tr("Start"), SchedulerSettings::ActionStart);
    ui->fileSinkAction->addItem(tr("Stop"), SchedulerSettings::ActionStop);

    m_cameraRecordMode->addItem(tr("Raw"), 0);
    m_cameraRecordMode->addItem(tr("Processed"), 1);
    m_cameraRecordMode->addItem(tr("Both"), 2);

    ui->dateFrom->setDisplayFormat(QStringLiteral("yyyy-MM-dd"));
    ui->dateUntil->setDisplayFormat(QStringLiteral("yyyy-MM-dd"));
    ui->dateUntil->setMinimumDate(noDateUntil());
    ui->dateUntil->setSpecialValueText(tr("None"));
    ui->time->setDisplayFormat(QStringLiteral("HH:mm:ss"));
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
    connect(ui->channelActionsTable->selectionModel(), &QItemSelectionModel::selectionChanged, this, &SchedulerGUI::onChannelActionSelectionChanged);
    connect(ui->featureActionsTable->selectionModel(), &QItemSelectionModel::selectionChanged, this, &SchedulerGUI::onFeatureActionSelectionChanged);

    connect(ui->addRule, &QToolButton::clicked, this, &SchedulerGUI::onAddRule);
    connect(ui->deleteRule, &QToolButton::clicked, this, &SchedulerGUI::onDeleteRule);
    connect(ui->refreshLists, &QToolButton::clicked, this, &SchedulerGUI::onRefreshLists);
    connect(ui->addDeviceAction, &QToolButton::clicked, this, &SchedulerGUI::onAddDeviceAction);
    connect(ui->deleteDeviceAction, &QToolButton::clicked, this, &SchedulerGUI::onDeleteDeviceAction);
    connect(ui->addChannelAction, &QToolButton::clicked, this, &SchedulerGUI::onAddChannelAction);
    connect(ui->deleteChannelAction, &QToolButton::clicked, this, &SchedulerGUI::onDeleteChannelAction);
    connect(ui->addFeatureAction, &QToolButton::clicked, this, &SchedulerGUI::onAddFeatureAction);
    connect(ui->deleteFeatureAction, &QToolButton::clicked, this, &SchedulerGUI::onDeleteFeatureAction);

    connect(ui->ruleName, &QLineEdit::editingFinished, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->ruleEnabled, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->triggerType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->dateFrom, &QDateEdit::dateChanged, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->time, &QTimeEdit::timeChanged, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->dateUntil, &QDateEdit::dateChanged, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->recurrence, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->monday, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->tuesday, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->wednesday, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->thursday, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->friday, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->saturday, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->sunday, &QCheckBox::toggled, this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->duration, QOverload<int>::of(&QSpinBox::valueChanged), this, &SchedulerGUI::onRuleEditorChanged);
    connect(ui->durationUnit, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onRuleEditorChanged);
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
    connect(ui->centerFrequency, &QLineEdit::editingFinished, this, &SchedulerGUI::onDeviceEditorChanged);
    connectSettingTable(m_deviceSettingsTable, m_addDeviceSetting, m_deleteDeviceSetting, &SchedulerGUI::onDeviceEditorChanged);

    connect(ui->channelDeviceSet, &QComboBox::currentTextChanged, this, [this](const QString&) {
        if (!m_populating) {
            updateChannelList(currentChannelAction());
            updateChannelActionList(currentChannelAction());
            onChannelEditorChanged();
        }
    });
    connect(ui->channelSelect, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        if (!m_populating) {
            updateChannelActionList(currentChannelAction());
            onChannelEditorChanged();
        }
    });
    connect(ui->channelAction, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onChannelEditorChanged);
    connect(m_channelText, &QLineEdit::editingFinished, this, &SchedulerGUI::onChannelEditorChanged);
    connect(m_channelData, &QLineEdit::editingFinished, this, &SchedulerGUI::onChannelEditorChanged);
    connect(m_packetCallsign, &QLineEdit::editingFinished, this, &SchedulerGUI::onChannelEditorChanged);
    connect(m_packetTo, &QLineEdit::editingFinished, this, &SchedulerGUI::onChannelEditorChanged);
    connect(m_packetVia, &QLineEdit::editingFinished, this, &SchedulerGUI::onChannelEditorChanged);
    connect(m_packetData, &QLineEdit::editingFinished, this, &SchedulerGUI::onChannelEditorChanged);
    connectSettingTable(m_channelSettingsTable, m_addChannelSetting, m_deleteChannelSetting, &SchedulerGUI::onChannelEditorChanged);

    connect(ui->featureSelect, &QComboBox::currentTextChanged, this, [this](const QString&) {
        if (!m_populating) {
            updateFeatureActionList(currentFeatureAction());
            onFeatureEditorChanged();
        }
    });
    connect(ui->featureAction, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onFeatureEditorChanged);
    connect(m_cameraFilename, &QLineEdit::editingFinished, this, &SchedulerGUI::onFeatureEditorChanged);
    connect(m_cameraRecordMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SchedulerGUI::onFeatureEditorChanged);
    connect(m_cameraImageCount, QOverload<int>::of(&QSpinBox::valueChanged), this, &SchedulerGUI::onFeatureEditorChanged);
    connect(m_cameraVideoDuration, QOverload<int>::of(&QSpinBox::valueChanged), this, &SchedulerGUI::onFeatureEditorChanged);
    connect(m_findTarget, &QLineEdit::editingFinished, this, &SchedulerGUI::onFeatureEditorChanged);
    connectSettingTable(m_featureSettingsTable, m_addFeatureSetting, m_deleteFeatureSetting, &SchedulerGUI::onFeatureEditorChanged);
}

void SchedulerGUI::createDeviceActionParameterEditors()
{
    createApplySettingsEditor(
        m_deviceSettingsGroup,
        m_deviceSettingsTable,
        m_addDeviceSetting,
        m_deleteDeviceSetting,
        ui->deviceGroup,
        ui->deviceGroupLayout);
}

void SchedulerGUI::createFeatureActionParameterEditors()
{
    m_cameraActionGroup = new QGroupBox(tr("Camera Action Parameters"), ui->featureGroup);
    QFormLayout *layout = new QFormLayout(m_cameraActionGroup);

    m_cameraFilename = new QLineEdit(m_cameraActionGroup);
    m_cameraFilename->setToolTip(tr("Optional filename. Leave empty to use the Camera feature setting."));

    m_cameraRecordMode = new QComboBox(m_cameraActionGroup);

    m_cameraImageCountLabel = new QLabel(tr("Images"), m_cameraActionGroup);
    m_cameraImageCount = new QSpinBox(m_cameraActionGroup);
    m_cameraImageCount->setRange(0, 1000000);
    m_cameraImageCount->setToolTip(tr("Number of images to save. 0 records until stopped."));

    m_cameraVideoDurationLabel = new QLabel(tr("Duration"), m_cameraActionGroup);
    m_cameraVideoDuration = new QSpinBox(m_cameraActionGroup);
    m_cameraVideoDuration->setRange(0, 86400);
    m_cameraVideoDuration->setSuffix(tr(" s"));
    m_cameraVideoDuration->setToolTip(tr("Video duration in seconds. 0 records until stopped."));

    layout->addRow(tr("Filename"), m_cameraFilename);
    layout->addRow(tr("Record mode"), m_cameraRecordMode);
    layout->addRow(m_cameraImageCountLabel, m_cameraImageCount);
    layout->addRow(m_cameraVideoDurationLabel, m_cameraVideoDuration);

    ui->featureGroupLayout->addWidget(m_cameraActionGroup);

    m_findActionGroup = new QGroupBox(tr("Find Action Parameters"), ui->featureGroup);
    QFormLayout *findLayout = new QFormLayout(m_findActionGroup);
    m_findTarget = new QLineEdit(m_findActionGroup);
    findLayout->addRow(tr("Target"), m_findTarget);
    ui->featureGroupLayout->addWidget(m_findActionGroup);

    createApplySettingsEditor(
        m_featureSettingsGroup,
        m_featureSettingsTable,
        m_addFeatureSetting,
        m_deleteFeatureSetting,
        ui->featureGroup,
        ui->featureGroupLayout);
}

void SchedulerGUI::createChannelActionParameterEditors()
{
    m_channelTextGroup = new QGroupBox(tr("Text Parameters"), ui->channelGroup);
    QFormLayout *textLayout = new QFormLayout(m_channelTextGroup);
    m_channelText = new QLineEdit(m_channelTextGroup);
    textLayout->addRow(tr("Text"), m_channelText);
    ui->channelGroupLayout->addWidget(m_channelTextGroup);

    m_channelDataGroup = new QGroupBox(tr("Data Parameters"), ui->channelGroup);
    QFormLayout *dataLayout = new QFormLayout(m_channelDataGroup);
    m_channelData = new QLineEdit(m_channelDataGroup);
    dataLayout->addRow(tr("Data"), m_channelData);
    ui->channelGroupLayout->addWidget(m_channelDataGroup);

    m_packetActionGroup = new QGroupBox(tr("Packet Parameters"), ui->channelGroup);
    QFormLayout *packetLayout = new QFormLayout(m_packetActionGroup);
    m_packetCallsign = new QLineEdit(m_packetActionGroup);
    m_packetTo = new QLineEdit(m_packetActionGroup);
    m_packetVia = new QLineEdit(m_packetActionGroup);
    m_packetData = new QLineEdit(m_packetActionGroup);
    packetLayout->addRow(tr("Callsign"), m_packetCallsign);
    packetLayout->addRow(tr("To"), m_packetTo);
    packetLayout->addRow(tr("Via"), m_packetVia);
    packetLayout->addRow(tr("Data"), m_packetData);
    ui->channelGroupLayout->addWidget(m_packetActionGroup);

    createApplySettingsEditor(
        m_channelSettingsGroup,
        m_channelSettingsTable,
        m_addChannelSetting,
        m_deleteChannelSetting,
        ui->channelGroup,
        ui->channelGroupLayout);
}

void SchedulerGUI::createApplySettingsEditor(
        QGroupBox *&group,
        QTableWidget *&table,
        QToolButton *&addButton,
        QToolButton *&deleteButton,
        QWidget *parent,
        QLayout *targetLayout)
{
    group = new QGroupBox(tr("Apply Settings"), parent);
    QVBoxLayout *layout = new QVBoxLayout(group);
    table = new QTableWidget(group);
    table->setColumnCount(3);
    table->setHorizontalHeaderLabels(QStringList({tr("Setting"), tr("Value"), tr("Type")}));
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    table->verticalHeader()->setVisible(false);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    addButton = new QToolButton(group);
    deleteButton = new QToolButton(group);
    addButton->setText(QStringLiteral("+"));
    deleteButton->setText(QStringLiteral("-"));
    addButton->setToolTip(tr("Add setting"));
    deleteButton->setToolTip(tr("Delete setting"));
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(deleteButton);
    buttonLayout->addStretch(1);

    layout->addWidget(table);
    layout->addLayout(buttonLayout);
    targetLayout->addWidget(group);
}

QList<SchedulerSettings::SettingValue> SchedulerGUI::settingValuesFromTable(QTableWidget *table) const
{
    QList<SchedulerSettings::SettingValue> settings;

    for (int row = 0; row < table->rowCount(); ++row)
    {
        SchedulerSettings::SettingValue setting;
        QTableWidgetItem *nameItem = table->item(row, 0);
        QTableWidgetItem *valueItem = table->item(row, 1);
        QComboBox *type = qobject_cast<QComboBox *>(table->cellWidget(row, 2));

        setting.m_name = nameItem ? nameItem->text().trimmed() : QString();
        setting.m_value = valueItem ? valueItem->text() : QString();
        setting.m_type = type
            ? static_cast<SchedulerSettings::SettingValueType>(type->currentData().toInt())
            : SchedulerSettings::SettingString;

        if (!setting.m_name.isEmpty()) {
            settings.append(setting);
        }
    }

    return settings;
}

void SchedulerGUI::setSettingValuesToTable(QTableWidget *table, const QList<SchedulerSettings::SettingValue>& settings, void (SchedulerGUI::*editorChanged)())
{
    table->setRowCount(settings.size());

    for (int row = 0; row < settings.size(); ++row)
    {
        const SchedulerSettings::SettingValue& setting = settings[row];
        table->setItem(row, 0, new QTableWidgetItem(setting.m_name));
        table->setItem(row, 1, new QTableWidgetItem(setting.m_value));

        QComboBox *type = new QComboBox(table);
        type->addItem(tr("String"), SchedulerSettings::SettingString);
        type->addItem(tr("Integer"), SchedulerSettings::SettingInteger);
        type->addItem(tr("Double"), SchedulerSettings::SettingDouble);
        type->setCurrentIndex(type->findData(setting.m_type));
        connect(type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, editorChanged](int) {
            if (!m_populating) {
                (this->*editorChanged)();
            }
        });
        table->setCellWidget(row, 2, type);
    }
}

void SchedulerGUI::connectSettingTable(QTableWidget *table, QToolButton *addButton, QToolButton *deleteButton, void (SchedulerGUI::*editorChanged)())
{
    connect(table, &QTableWidget::itemChanged, this, [this, editorChanged](QTableWidgetItem *) {
        if (!m_populating) {
            (this->*editorChanged)();
        }
    });

    connect(addButton, &QToolButton::clicked, this, [this, table, editorChanged]() {
        if (m_populating) {
            return;
        }

        const int row = table->rowCount();
        table->insertRow(row);
        table->setItem(row, 0, new QTableWidgetItem());
        table->setItem(row, 1, new QTableWidgetItem());

        QComboBox *type = new QComboBox(table);
        type->addItem(tr("String"), SchedulerSettings::SettingString);
        type->addItem(tr("Integer"), SchedulerSettings::SettingInteger);
        type->addItem(tr("Double"), SchedulerSettings::SettingDouble);
        connect(type, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, editorChanged](int) {
            if (!m_populating) {
                (this->*editorChanged)();
            }
        });
        table->setCellWidget(row, 2, type);
        table->selectRow(row);
        (this->*editorChanged)();
    });

    connect(deleteButton, &QToolButton::clicked, this, [this, table, editorChanged]() {
        if (m_populating) {
            return;
        }

        const int row = table->currentRow();
        if (row >= 0) {
            table->removeRow(row);
            (this->*editorChanged)();
        }
    });
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

void SchedulerGUI::refreshChannelActionsTable()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    const int actionCount = rule ? rule->m_channelActions.size() : 0;

    m_populating = true;
    ui->channelActionsTable->setRowCount(actionCount);

    for (int row = 0; row < actionCount; ++row)
    {
        const SchedulerSettings::ChannelAction& action = rule->m_channelActions[row];
        ui->channelActionsTable->setItem(row, 0, new QTableWidgetItem(deviceActionText(action)));
        ui->channelActionsTable->setItem(row, 1, new QTableWidgetItem(channelActionText(action)));
        ui->channelActionsTable->setItem(row, 2, new QTableWidgetItem(runActionText(action.m_action)));
    }

    m_populating = false;

    if ((m_currentChannelAction >= 0) && (m_currentChannelAction < actionCount)) {
        ui->channelActionsTable->selectRow(m_currentChannelAction);
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
    ui->channelGroup->setEnabled(hasRule);
    ui->featureGroup->setEnabled(hasRule);

    if (hasRule)
    {
        ui->ruleName->setText(rule->m_name);
        ui->ruleEnabled->setChecked(rule->m_enabled);
        ui->triggerType->setCurrentIndex(ui->triggerType->findData(rule->m_triggerType));
        const QDateTime time = rule->m_time.isValid() ? rule->m_time : QDateTime::currentDateTime().addSecs(60);
        ui->dateFrom->setDate(time.date());
        ui->time->setTime(time.time());
        ui->dateUntil->setDate(rule->m_dateUntil.isValid() ? rule->m_dateUntil : noDateUntil());
        ui->recurrence->setCurrentIndex(ui->recurrence->findData(rule->m_recurrence));
        setWeekdayWidgets(ui, rule->m_weekdayMask);
        ui->duration->setValue(rule->m_duration);
        ui->durationUnit->setCurrentIndex(ui->durationUnit->findData(rule->m_durationUnit));
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
        const QDateTime time = QDateTime::currentDateTime().addSecs(60);
        ui->dateFrom->setDate(time.date());
        ui->time->setTime(time.time());
        ui->dateUntil->setDate(noDateUntil());
        setWeekdayWidgets(ui, DefaultWeekdayMask);
        ui->duration->setValue(0);
        ui->durationUnit->setCurrentIndex(ui->durationUnit->findData(SchedulerSettings::DelaySeconds));
        updateEventSourceList(QString());
        ui->eventDataRegex->clear();
        ui->eventDelay->setValue(0);
        ui->command->clear();
        ui->speech->clear();
    }

    m_currentDeviceAction = hasRule && !rule->m_deviceSetActions.isEmpty() ? qBound(0, m_currentDeviceAction, rule->m_deviceSetActions.size() - 1) : -1;
    m_currentChannelAction = hasRule && !rule->m_channelActions.isEmpty() ? qBound(0, m_currentChannelAction, rule->m_channelActions.size() - 1) : -1;
    m_currentFeatureAction = hasRule && !rule->m_featureActions.isEmpty() ? qBound(0, m_currentFeatureAction, rule->m_featureActions.size() - 1) : -1;

    m_populating = false;
    updateTriggerVisibility();
    updateRegexState();
    refreshDeviceActionsTable();
    refreshChannelActionsTable();
    refreshFeatureActionsTable();
    selectDeviceAction(m_currentDeviceAction);
    selectChannelAction(m_currentChannelAction);
    selectFeatureAction(m_currentFeatureAction);
    displayDeviceActionEditor();
    displayChannelActionEditor();
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
    ui->centerFrequency->setEnabled(hasAction);

    updateDeviceSetList(action);
    updatePresetList(action);

    if (hasAction)
    {
        const int acquisitionIndex = ui->acquisitionAction->findData(action->m_acquisitionAction);
        ui->acquisitionAction->setCurrentIndex(acquisitionIndex >= 0
            ? acquisitionIndex
            : ui->acquisitionAction->findData(SchedulerSettings::ActionNoChange));
        ui->fileSinkAction->setCurrentIndex(ui->fileSinkAction->findData(action->m_fileSinkAction));
        ui->centerFrequency->setText(action->m_centerFrequency);
        setSettingValuesToTable(m_deviceSettingsTable, action->m_settings, &SchedulerGUI::onDeviceEditorChanged);
    }
    else
    {
        ui->acquisitionAction->setCurrentIndex(ui->acquisitionAction->findData(SchedulerSettings::ActionNoChange));
        ui->fileSinkAction->setCurrentIndex(ui->fileSinkAction->findData(SchedulerSettings::ActionNoChange));
        ui->centerFrequency->clear();
        setSettingValuesToTable(m_deviceSettingsTable, QList<SchedulerSettings::SettingValue>(), &SchedulerGUI::onDeviceEditorChanged);
    }

    updateDeviceActionParameterVisibility();
    m_populating = false;
}

void SchedulerGUI::displayChannelActionEditor()
{
    SchedulerSettings::ChannelAction *action = currentChannelAction();
    const bool hasAction = action != nullptr;

    m_populating = true;
    ui->channelDeviceSet->setEnabled(hasAction);
    ui->channelSelect->setEnabled(hasAction);
    ui->channelAction->setEnabled(hasAction);

    updateChannelDeviceSetList(action);
    updateChannelList(action);
    updateChannelActionList(action);

    if (hasAction)
    {
        ui->channelAction->setCurrentIndex(ui->channelAction->findData(action->m_action));
        m_channelText->setText(action->m_text);
        m_channelData->setText(action->m_data);
        m_packetCallsign->setText(action->m_callsign);
        m_packetTo->setText(action->m_to);
        m_packetVia->setText(action->m_via);
        m_packetData->setText(action->m_data);
        setSettingValuesToTable(m_channelSettingsTable, action->m_settings, &SchedulerGUI::onChannelEditorChanged);
    }
    else
    {
        m_channelText->clear();
        m_channelData->clear();
        m_packetCallsign->clear();
        m_packetTo->clear();
        m_packetVia->clear();
        m_packetData->clear();
        setSettingValuesToTable(m_channelSettingsTable, QList<SchedulerSettings::SettingValue>(), &SchedulerGUI::onChannelEditorChanged);
    }

    updateChannelActionParameterVisibility();
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
    updateFeatureActionList(action);

    if (hasAction)
    {
        const int actionIndex = ui->featureAction->findData(action->m_action);
        if (actionIndex >= 0) {
            ui->featureAction->setCurrentIndex(actionIndex);
        }
        m_cameraFilename->setText(action->m_cameraFilename);
        m_cameraRecordMode->setCurrentIndex(m_cameraRecordMode->findData(action->m_cameraRecordMode));
        m_cameraImageCount->setValue(action->m_cameraImageCount);
        m_cameraVideoDuration->setValue(action->m_cameraVideoDuration);
        m_findTarget->setText(action->m_findTarget);
        setSettingValuesToTable(m_featureSettingsTable, action->m_settings, &SchedulerGUI::onFeatureEditorChanged);
    }
    else
    {
        const int actionIndex = ui->featureAction->findData(SchedulerSettings::ActionStart);
        if (actionIndex >= 0) {
            ui->featureAction->setCurrentIndex(actionIndex);
        }
        m_cameraFilename->clear();
        m_cameraRecordMode->setCurrentIndex(m_cameraRecordMode->findData(0));
        m_cameraImageCount->setValue(1);
        m_cameraVideoDuration->setValue(0);
        m_findTarget->clear();
        setSettingValuesToTable(m_featureSettingsTable, QList<SchedulerSettings::SettingValue>(), &SchedulerGUI::onFeatureEditorChanged);
    }

    ui->featureAction->setEnabled(hasAction && (ui->featureAction->count() > 0));
    updateFeatureActionParameterVisibility();
    m_populating = false;
}

void SchedulerGUI::updateChannelActionParameterVisibility()
{
    const int action = ui->channelAction->currentData().toInt();
    const bool text = (action == SchedulerSettings::ActionRTTYTransmit)
        || (action == SchedulerSettings::ActionPSK31Transmit);
    const bool data = (action == SchedulerSettings::ActionIEEE_802_15_4Transmit)
        || (action == SchedulerSettings::ActionAISTransmit);
    const bool packet = action == SchedulerSettings::ActionPacketTransmit;
    const bool settings = action == SchedulerSettings::ActionApplySetting;

    m_channelTextGroup->setVisible(text);
    m_channelTextGroup->setEnabled(text);
    m_channelDataGroup->setVisible(data);
    m_channelDataGroup->setEnabled(data);
    m_packetActionGroup->setVisible(packet);
    m_packetActionGroup->setEnabled(packet);
    m_channelSettingsGroup->setVisible(settings);
    m_channelSettingsGroup->setEnabled(settings);
    getRollupContents()->arrangeRollups();
}

void SchedulerGUI::updateFeatureActionParameterVisibility()
{
    SchedulerSettings::FeatureAction *action = currentFeatureAction();
    const int featureAction = ui->featureAction->currentData().toInt();
    const bool cameraFeature = action && (action->m_featureId == "sdrangel.feature.camera");
    const bool findFeature = action
        && ((action->m_featureId == "sdrangel.feature.map") || (action->m_featureId == "sdrangel.feature.skymap"));
    const bool saveImage = cameraFeature && (featureAction == SchedulerSettings::ActionCameraSaveImage);
    const bool recordVideo = cameraFeature && (featureAction == SchedulerSettings::ActionCameraRecordVideo);
    const bool find = findFeature && (featureAction == SchedulerSettings::ActionMapFind);
    const bool settings = featureAction == SchedulerSettings::ActionApplySetting;
    const bool showCameraParams = saveImage || recordVideo;

    m_cameraActionGroup->setVisible(showCameraParams);
    m_cameraActionGroup->setEnabled(showCameraParams);
    m_cameraImageCountLabel->setVisible(saveImage);
    m_cameraImageCount->setVisible(saveImage);
    m_cameraVideoDurationLabel->setVisible(recordVideo);
    m_cameraVideoDuration->setVisible(recordVideo);
    m_findActionGroup->setVisible(find);
    m_findActionGroup->setEnabled(find);
    m_featureSettingsGroup->setVisible(settings);
    m_featureSettingsGroup->setEnabled(settings);
    getRollupContents()->arrangeRollups();
}

void SchedulerGUI::updateDeviceActionParameterVisibility()
{
    const bool hasAction = currentDeviceAction() != nullptr;
    m_deviceSettingsGroup->setVisible(hasAction);
    m_deviceSettingsGroup->setEnabled(hasAction);
    getRollupContents()->arrangeRollups();
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
    rule->m_time = QDateTime(ui->dateFrom->date(), ui->time->time());
    rule->m_dateUntil = ui->dateUntil->date() == noDateUntil() ? QDate() : ui->dateUntil->date();
    rule->m_recurrence = static_cast<SchedulerSettings::Recurrence>(ui->recurrence->currentData().toInt());
    rule->m_weekdayMask = weekdayMaskFromWidgets(ui);
    rule->m_duration = ui->duration->value();
    rule->m_durationUnit = static_cast<SchedulerSettings::DelayUnit>(ui->durationUnit->currentData().toInt());
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
    action->m_centerFrequency = ui->centerFrequency->text().trimmed();
    action->m_settings = settingValuesFromTable(m_deviceSettingsTable);
}

void SchedulerGUI::updateCurrentChannelActionFromWidgets()
{
    SchedulerSettings::ChannelAction *action = currentChannelAction();
    if (!action) {
        return;
    }

    const QVariant deviceData = ui->channelDeviceSet->currentData();
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
        if (MainCore::getDeviceSetIndexFromId(ui->channelDeviceSet->currentText(), deviceSetIndex))
        {
            action->m_deviceSetIndex = static_cast<int>(deviceSetIndex);
            action->m_deviceSetId = ui->channelDeviceSet->currentText();
        }
        else
        {
            action->m_deviceSetId = ui->channelDeviceSet->currentText();
        }
    }

    bool parsedChannel = false;
    const int channelIndex = ui->channelSelect->currentData().toInt(&parsedChannel);
    if (parsedChannel)
    {
        action->m_channelIndex = channelIndex;
        action->m_channelId = ui->channelSelect->currentData(Qt::UserRole + 1).toString();
    }

    action->m_action = static_cast<SchedulerSettings::RunAction>(ui->channelAction->currentData().toInt());
    action->m_text = m_channelText->text();
    action->m_callsign = m_packetCallsign->text();
    action->m_to = m_packetTo->text();
    action->m_via = m_packetVia->text();
    action->m_data = (action->m_action == SchedulerSettings::ActionPacketTransmit) ? m_packetData->text() : m_channelData->text();
    action->m_settings = settingValuesFromTable(m_channelSettingsTable);
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
    action->m_cameraFilename = m_cameraFilename->text();
    action->m_cameraRecordMode = m_cameraRecordMode->currentData().toInt();
    action->m_cameraImageCount = m_cameraImageCount->value();
    action->m_cameraVideoDuration = m_cameraVideoDuration->value();
    action->m_findTarget = m_findTarget->text();
    action->m_settings = settingValuesFromTable(m_featureSettingsTable);
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

void SchedulerGUI::updateChannelDeviceSetList(const SchedulerSettings::ChannelAction *selectedAction)
{
    ui->channelDeviceSet->clear();

    const SchedulerSettings::ScheduleRule *rule = currentRule();
    bool found = false;

    if (rule)
    {
        for (const SchedulerSettings::DeviceSetAction& deviceAction : rule->m_deviceSetActions)
        {
            if (ui->channelDeviceSet->findData(deviceAction.m_deviceSetIndex) >= 0) {
                continue;
            }

            const QString id = deviceActionText(deviceAction);
            ui->channelDeviceSet->addItem(id, deviceAction.m_deviceSetIndex);

            if (selectedAction && (selectedAction->m_deviceSetIndex == deviceAction.m_deviceSetIndex)) {
                found = true;
            }
        }
    }

    if (selectedAction && !found && !selectedAction->m_deviceSetId.isEmpty()) {
        ui->channelDeviceSet->addItem(selectedAction->m_deviceSetId, QVariant());
    }

    if (selectedAction)
    {
        const int index = ui->channelDeviceSet->findData(selectedAction->m_deviceSetIndex);
        if (index >= 0) {
            ui->channelDeviceSet->setCurrentIndex(index);
        } else if (!selectedAction->m_deviceSetId.isEmpty()) {
            ui->channelDeviceSet->setCurrentText(selectedAction->m_deviceSetId);
        }
    }
}

void SchedulerGUI::updateChannelList(const SchedulerSettings::ChannelAction *selectedAction)
{
    ui->channelSelect->clear();
    bool found = false;

    const SchedulerSettings::DeviceSetAction *deviceAction = deviceSetActionForChannelAction(selectedAction);
    const Preset *preset = presetForDeviceSetAction(deviceAction);
    if (preset)
    {
        for (int i = 0; i < preset->getChannelCount(); ++i)
        {
            const QString channelId = preset->getChannelConfig(i).m_channelIdURI;
            ui->channelSelect->addItem(QStringLiteral("%1 %2").arg(i).arg(channelId), i);
            ui->channelSelect->setItemData(ui->channelSelect->count() - 1, channelId, Qt::UserRole + 1);
            if (selectedAction && (selectedAction->m_channelIndex == i)) {
                found = true;
            }
        }
    }
    else
    {
        const int deviceSetIndex = ui->channelDeviceSet->currentData().toInt();
        const std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
        if ((deviceSetIndex >= 0) && (deviceSetIndex < (int) deviceSets.size()))
        {
            DeviceSet *deviceSet = deviceSets[deviceSetIndex];
            for (int i = 0; i < deviceSet->getNumberOfChannels(); ++i)
            {
                ChannelAPI *channel = deviceSet->getChannelAt(i);
                const QString channelId = channel ? channel->getURI() : QString();
                QString text = QStringLiteral("%1 %2").arg(i).arg(channelId);
                if (channel)
                {
                    text += QStringLiteral(" (%1)").arg(channel->getIdentifier());
                }
                ui->channelSelect->addItem(text, i);
                ui->channelSelect->setItemData(ui->channelSelect->count() - 1, channelId, Qt::UserRole + 1);
                if (selectedAction && (selectedAction->m_channelIndex == i)) {
                    found = true;
                }
            }
        }
    }

    if (selectedAction && !found)
    {
        ui->channelSelect->addItem(
            QStringLiteral("%1 %2 (unresolved)")
                .arg(selectedAction->m_channelIndex)
                .arg(selectedAction->m_channelId),
            selectedAction->m_channelIndex);
        ui->channelSelect->setItemData(ui->channelSelect->count() - 1, selectedAction->m_channelId, Qt::UserRole + 1);
    }

    if (selectedAction)
    {
        const int index = ui->channelSelect->findData(selectedAction->m_channelIndex);
        if (index >= 0) {
            ui->channelSelect->setCurrentIndex(index);
        } else if (ui->channelSelect->count() > 0) {
            ui->channelSelect->setCurrentIndex(0);
        }
    }
}

void SchedulerGUI::updateChannelActionList(const SchedulerSettings::ChannelAction *selectedAction)
{
    const SchedulerSettings::RunAction selected = selectedAction ? selectedAction->m_action : SchedulerSettings::ActionNoChange;
    const QString channelId = ui->channelSelect->currentData(Qt::UserRole + 1).toString();
    ui->channelAction->clear();

    auto addAction = [this, &channelId](const QString& text, SchedulerSettings::RunAction action) {
        if (channelSupportsAction(channelId, action)) {
            ui->channelAction->addItem(text, action);
        }
    };

    addAction(tr("Start recording"), SchedulerSettings::ActionFileSinkRecordStart);
    addAction(tr("Stop recording"), SchedulerSettings::ActionFileSinkRecordStop);
    addAction(tr("Start SigMF recording"), SchedulerSettings::ActionSigMFRecordStart);
    addAction(tr("Stop SigMF recording"), SchedulerSettings::ActionSigMFRecordStop);
    addAction(tr("Transmit RTTY"), SchedulerSettings::ActionRTTYTransmit);
    addAction(tr("Transmit PSK31"), SchedulerSettings::ActionPSK31Transmit);
    addAction(tr("Transmit packet"), SchedulerSettings::ActionPacketTransmit);
    addAction(tr("Transmit IEEE 802.15.4"), SchedulerSettings::ActionIEEE_802_15_4Transmit);
    addAction(tr("Transmit AIS"), SchedulerSettings::ActionAISTransmit);
    addAction(tr("Run scanner"), SchedulerSettings::ActionFreqScannerRun);
    addAction(tr("Stop scanner"), SchedulerSettings::ActionFreqScannerStop);
    addAction(tr("Start sweep"), SchedulerSettings::ActionRadioAstronomyStart);
    addAction(tr("Apply setting"), SchedulerSettings::ActionApplySetting);

    const int index = ui->channelAction->findData(selected);
    if (index >= 0) {
        ui->channelAction->setCurrentIndex(index);
    } else if (ui->channelAction->count() > 0) {
        ui->channelAction->setCurrentIndex(0);
    }

    ui->channelAction->setEnabled(ui->channelAction->count() > 0);
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
            ui->featureSelect->setItemData(ui->featureSelect->count() - 1, feature->getURI(), Qt::UserRole + 1);

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
        ui->featureSelect->setItemData(ui->featureSelect->count() - 1, selectedAction->m_featureId, Qt::UserRole + 1);
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

void SchedulerGUI::updateFeatureActionList(const SchedulerSettings::FeatureAction *selectedAction)
{
    const SchedulerSettings::RunAction selected = selectedAction ? selectedAction->m_action : SchedulerSettings::ActionStart;
    const QString featureId = ui->featureSelect->currentData(Qt::UserRole + 1).toString();
    ui->featureAction->clear();

    auto addAction = [this, &featureId](const QString& text, SchedulerSettings::RunAction action) {
        if (featureSupportsAction(featureId, action)) {
            ui->featureAction->addItem(text, action);
        }
    };

    addAction(tr("Start"), SchedulerSettings::ActionStart);
    addAction(tr("Stop"), SchedulerSettings::ActionStop);
    addAction(tr("Save image"), SchedulerSettings::ActionCameraSaveImage);
    addAction(tr("Record video"), SchedulerSettings::ActionCameraRecordVideo);
    addAction(tr("Find"), SchedulerSettings::ActionMapFind);
    addAction(tr("Apply setting"), SchedulerSettings::ActionApplySetting);

    const int index = ui->featureAction->findData(selected);
    if (index >= 0) {
        ui->featureAction->setCurrentIndex(index);
    } else if (ui->featureAction->count() > 0) {
        ui->featureAction->setCurrentIndex(0);
    }

    ui->featureAction->setEnabled(ui->featureAction->count() > 0);
}

void SchedulerGUI::updateTriggerVisibility()
{
    const bool isTime = ui->triggerType->currentData().toInt() == SchedulerSettings::TriggerTime;
    ui->timeGroup->setVisible(isTime);
    ui->eventGroup->setVisible(!isTime);
    updateTimeScheduleVisibility();
    getRollupContents()->arrangeRollups();
}

void SchedulerGUI::updateTimeScheduleVisibility()
{
    const bool isDaily = ui->recurrence->currentData().toInt() == SchedulerSettings::RecurrenceDaily;
    ui->weekdaysLabel->setVisible(isDaily);
    ui->weekdaysWidget->setVisible(isDaily);
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

void SchedulerGUI::selectChannelAction(int row)
{
    if ((row >= 0) && (row < ui->channelActionsTable->rowCount()))
    {
        m_currentChannelAction = row;
        ui->channelActionsTable->selectRow(row);
    }
    else
    {
        m_currentChannelAction = -1;
        ui->channelActionsTable->clearSelection();
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

SchedulerSettings::ChannelAction *SchedulerGUI::currentChannelAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (rule && (m_currentChannelAction >= 0) && (m_currentChannelAction < rule->m_channelActions.size())) {
        return &rule->m_channelActions[m_currentChannelAction];
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

const SchedulerSettings::DeviceSetAction *SchedulerGUI::deviceSetActionForChannelAction(const SchedulerSettings::ChannelAction *action) const
{
    const SchedulerSettings::ScheduleRule *rule = const_cast<SchedulerGUI *>(this)->currentRule();
    if (!rule) {
        return nullptr;
    }

    bool ok = false;
    int deviceSetIndex = ui->channelDeviceSet->currentData().toInt(&ok);
    if (!ok && action) {
        deviceSetIndex = action->m_deviceSetIndex;
        ok = true;
    }
    if (!ok) {
        return nullptr;
    }

    for (const SchedulerSettings::DeviceSetAction& deviceAction : rule->m_deviceSetActions)
    {
        if (deviceAction.m_deviceSetIndex == deviceSetIndex) {
            return &deviceAction;
        }
    }

    return nullptr;
}

const Preset *SchedulerGUI::presetForDeviceSetAction(const SchedulerSettings::DeviceSetAction *action) const
{
    if (!action || action->m_presetGroup.isEmpty()) {
        return nullptr;
    }

    QChar deviceType;
    if (!action->m_deviceSetId.isEmpty()) {
        deviceType = action->m_deviceSetId.at(0);
    } else {
        deviceType = deviceSetId(action->m_deviceSetIndex).at(0);
    }

    QString presetType;
    if (deviceType == 'R') {
        presetType = QStringLiteral("R");
    } else if (deviceType == 'T') {
        presetType = QStringLiteral("T");
    } else if (deviceType == 'M') {
        presetType = QStringLiteral("M");
    }

    return MainCore::instance()->getSettings().getPreset(
        action->m_presetGroup,
        action->m_presetFrequency,
        action->m_presetDescription,
        presetType);
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
    QString text;

    if (rule.m_triggerType == SchedulerSettings::TriggerTime)
    {
        if ((rule.m_recurrence == SchedulerSettings::RecurrenceDaily) && (rule.m_weekdayMask != DefaultWeekdayMask))
        {
            QStringList days;
            if ((rule.m_weekdayMask & (1 << 0)) != 0) {
                days.append(tr("Mon"));
            }
            if ((rule.m_weekdayMask & (1 << 1)) != 0) {
                days.append(tr("Tue"));
            }
            if ((rule.m_weekdayMask & (1 << 2)) != 0) {
                days.append(tr("Wed"));
            }
            if ((rule.m_weekdayMask & (1 << 3)) != 0) {
                days.append(tr("Thu"));
            }
            if ((rule.m_weekdayMask & (1 << 4)) != 0) {
                days.append(tr("Fri"));
            }
            if ((rule.m_weekdayMask & (1 << 5)) != 0) {
                days.append(tr("Sat"));
            }
            if ((rule.m_weekdayMask & (1 << 6)) != 0) {
                days.append(tr("Sun"));
            }
            text = days.isEmpty() ? tr("Daily (no days)") : tr("Daily (%1)").arg(days.join(QStringLiteral(", ")));
        }
        else
        {
            text = recurrenceText(rule.m_recurrence);
        }
    }
    else
    {
        const int delay = SchedulerSettings::delaySeconds(rule);
        if (delay > 0)
        {
            if (rule.m_eventDelayUnit == SchedulerSettings::DelayMinutes) {
                text = tr("%1 min").arg(rule.m_eventDelay);
            } else {
                text = tr("%1 s").arg(delay);
            }
        }
    }

    const QString duration = durationText(rule);
    if (!duration.isEmpty())
    {
        if (!text.isEmpty()) {
            text += QStringLiteral(", ");
        }
        text += tr("for %1").arg(duration);
    }

    return text;
}

QString SchedulerGUI::ruleActionSummary(const SchedulerSettings::ScheduleRule& rule) const
{
    QStringList parts;

    if (!rule.m_deviceSetActions.isEmpty()) {
        parts.append(tr("%1 device").arg(rule.m_deviceSetActions.size()));
    }
    if (!rule.m_channelActions.isEmpty()) {
        parts.append(tr("%1 channel").arg(rule.m_channelActions.size()));
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

QString SchedulerGUI::durationText(const SchedulerSettings::ScheduleRule& rule) const
{
    const int duration = SchedulerSettings::durationSeconds(rule);
    if (duration == 0) {
        return QString();
    }
    if (rule.m_durationUnit == SchedulerSettings::DelayMinutes) {
        return tr("%1 min").arg(rule.m_duration);
    }
    return tr("%1 s").arg(duration);
}

QString SchedulerGUI::deviceActionText(const SchedulerSettings::DeviceSetAction& action) const
{
    if (!action.m_deviceSetId.isEmpty()) {
        return action.m_deviceSetId;
    }

    return deviceSetId(action.m_deviceSetIndex);
}

QString SchedulerGUI::deviceActionText(const SchedulerSettings::ChannelAction& action) const
{
    if (!action.m_deviceSetId.isEmpty()) {
        return action.m_deviceSetId;
    }

    return deviceSetId(action.m_deviceSetIndex);
}

QString SchedulerGUI::channelActionText(const SchedulerSettings::ChannelAction& action) const
{
    return QStringLiteral("%1 %2")
        .arg(action.m_channelIndex)
        .arg(action.m_channelId);
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
    case SchedulerSettings::ActionCameraSaveImage:
        return tr("Save image");
    case SchedulerSettings::ActionCameraRecordVideo:
        return tr("Record video");
    case SchedulerSettings::ActionMapFind:
        return tr("Find");
    case SchedulerSettings::ActionFileSinkRecordStart:
        return tr("Start recording");
    case SchedulerSettings::ActionFileSinkRecordStop:
        return tr("Stop recording");
    case SchedulerSettings::ActionSigMFRecordStart:
        return tr("Start SigMF recording");
    case SchedulerSettings::ActionSigMFRecordStop:
        return tr("Stop SigMF recording");
    case SchedulerSettings::ActionRTTYTransmit:
        return tr("Transmit RTTY");
    case SchedulerSettings::ActionPSK31Transmit:
        return tr("Transmit PSK31");
    case SchedulerSettings::ActionPacketTransmit:
        return tr("Transmit packet");
    case SchedulerSettings::ActionIEEE_802_15_4Transmit:
        return tr("Transmit IEEE 802.15.4");
    case SchedulerSettings::ActionAISTransmit:
        return tr("Transmit AIS");
    case SchedulerSettings::ActionFreqScannerRun:
        return tr("Run scanner");
    case SchedulerSettings::ActionFreqScannerStop:
        return tr("Stop scanner");
    case SchedulerSettings::ActionRadioAstronomyStart:
        return tr("Start sweep");
    case SchedulerSettings::ActionApplySetting:
        return tr("Apply setting");
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
    m_currentChannelAction = -1;
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

void SchedulerGUI::onChannelActionSelectionChanged()
{
    if (m_populating) {
        return;
    }

    const QList<QTableWidgetSelectionRange> ranges = ui->channelActionsTable->selectedRanges();
    m_currentChannelAction = ranges.isEmpty() ? -1 : ranges.first().topRow();
    displayChannelActionEditor();
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
    m_currentChannelAction = -1;
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
    m_currentChannelAction = -1;
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
    displayChannelActionEditor();
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
    pruneChannelActionsForDeviceSets(*rule);
    if (m_currentDeviceAction >= rule->m_deviceSetActions.size()) {
        m_currentDeviceAction = rule->m_deviceSetActions.size() - 1;
    }
    if (m_currentChannelAction >= rule->m_channelActions.size()) {
        m_currentChannelAction = rule->m_channelActions.size() - 1;
    }
    refreshDeviceActionsTable();
    selectDeviceAction(m_currentDeviceAction);
    displayDeviceActionEditor();
    refreshChannelActionsTable();
    selectChannelAction(m_currentChannelAction);
    displayChannelActionEditor();
    refreshRulesTable();
    applyRules();
}

void SchedulerGUI::onAddChannelAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (!rule || rule->m_deviceSetActions.isEmpty()) {
        return;
    }

    SchedulerSettings::ChannelAction action;
    const SchedulerSettings::DeviceSetAction& deviceAction = rule->m_deviceSetActions.first();
    action.m_deviceSetIndex = deviceAction.m_deviceSetIndex;
    action.m_deviceSetId = deviceActionText(deviceAction);

    const std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
    if ((action.m_deviceSetIndex >= 0) && (action.m_deviceSetIndex < (int) deviceSets.size())
        && (deviceSets[action.m_deviceSetIndex]->getNumberOfChannels() > 0))
    {
        ChannelAPI *channel = deviceSets[action.m_deviceSetIndex]->getChannelAt(0);
        action.m_channelIndex = 0;
        action.m_channelId = channel ? channel->getURI() : QString();
    }

    rule->m_channelActions.append(action);
    m_currentChannelAction = rule->m_channelActions.size() - 1;
    refreshChannelActionsTable();
    selectChannelAction(m_currentChannelAction);
    displayChannelActionEditor();
    refreshRulesTable();
    applyRules();
}

void SchedulerGUI::onDeleteChannelAction()
{
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (!rule || (m_currentChannelAction < 0) || (m_currentChannelAction >= rule->m_channelActions.size())) {
        return;
    }

    rule->m_channelActions.removeAt(m_currentChannelAction);
    if (m_currentChannelAction >= rule->m_channelActions.size()) {
        m_currentChannelAction = rule->m_channelActions.size() - 1;
    }
    refreshChannelActionsTable();
    selectChannelAction(m_currentChannelAction);
    displayChannelActionEditor();
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
    SchedulerSettings::ScheduleRule *rule = currentRule();
    if (rule) {
        pruneChannelActionsForDeviceSets(*rule);
        if (m_currentChannelAction >= rule->m_channelActions.size()) {
            m_currentChannelAction = rule->m_channelActions.size() - 1;
        }
    }
    refreshDeviceActionsTable();
    selectDeviceAction(row);
    updateDeviceActionParameterVisibility();
    refreshChannelActionsTable();
    selectChannelAction(m_currentChannelAction);
    displayChannelActionEditor();
    refreshRulesTable();
    selectRule(m_currentRule);
    applyRules();
}

void SchedulerGUI::onChannelEditorChanged()
{
    if (!m_doApplySettings || m_populating) {
        return;
    }

    const int row = m_currentChannelAction;
    updateCurrentChannelActionFromWidgets();
    updateChannelActionParameterVisibility();
    refreshChannelActionsTable();
    selectChannelAction(row);
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
    updateFeatureActionParameterVisibility();
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
