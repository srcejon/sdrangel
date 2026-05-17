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

#include <QtPlugin>

#include "plugin/pluginapi.h"

#ifndef SERVER_MODE
#include "schedulergui.h"
#endif

#include "scheduler.h"
#include "schedulerplugin.h"
#include "schedulerwebapiadapter.h"

const PluginDescriptor SchedulerPlugin::m_pluginDescriptor = {
    Scheduler::m_featureId,
    QStringLiteral("Scheduler"),
    QStringLiteral("7.25.0"),
    QStringLiteral("(c) Jon Beniston, M7RCE"),
    QStringLiteral("https://github.com/f4exb/sdrangel"),
    true,
    QStringLiteral("https://github.com/f4exb/sdrangel")
};

SchedulerPlugin::SchedulerPlugin(QObject* parent) :
    QObject(parent),
    m_pluginAPI(nullptr)
{
}

const PluginDescriptor& SchedulerPlugin::getPluginDescriptor() const
{
    return m_pluginDescriptor;
}

void SchedulerPlugin::initPlugin(PluginAPI* pluginAPI)
{
    m_pluginAPI = pluginAPI;
    m_pluginAPI->registerFeature(Scheduler::m_featureIdURI, Scheduler::m_featureId, this);
}

#ifdef SERVER_MODE
FeatureGUI* SchedulerPlugin::createFeatureGUI(FeatureUISet *featureUISet, Feature *feature) const
{
    (void) featureUISet;
    (void) feature;
    return nullptr;
}
#else
FeatureGUI* SchedulerPlugin::createFeatureGUI(FeatureUISet *featureUISet, Feature *feature) const
{
    return SchedulerGUI::create(m_pluginAPI, featureUISet, feature);
}
#endif

Feature* SchedulerPlugin::createFeature(WebAPIAdapterInterface* webAPIAdapterInterface) const
{
    return new Scheduler(webAPIAdapterInterface);
}

FeatureWebAPIAdapter* SchedulerPlugin::createFeatureWebAPIAdapter() const
{
    return new SchedulerWebAPIAdapter();
}
