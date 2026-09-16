///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by Copilot / Claude Sonnet                                          //
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
#include "sstvmodgui.h"
#endif
#include "sstvmod.h"
#include "sstvmodwebapiadapter.h"
#include "sstvmodplugin.h"

const PluginDescriptor SSTVModPlugin::m_pluginDescriptor = {
    SSTVMod::m_channelId,
    QStringLiteral("SSTV Modulator"),
    QStringLiteral("7.25.0"),
    QStringLiteral("(c) Jon Beniston, M7RCE"),
    QStringLiteral("https://github.com/srcejon/sdrangel"),
    true,
    QStringLiteral("https://github.com/srcejon/sdrangel")
};

SSTVModPlugin::SSTVModPlugin(QObject* parent) :
    QObject(parent)
{}

const PluginDescriptor& SSTVModPlugin::getPluginDescriptor() const
{
    return m_pluginDescriptor;
}

void SSTVModPlugin::initPlugin(PluginAPI* pluginAPI)
{
    m_pluginAPI = pluginAPI;
    m_pluginAPI->registerTxChannel(SSTVMod::m_channelIdURI, SSTVMod::m_channelId, this);
}

void SSTVModPlugin::createTxChannel(DeviceAPI *deviceAPI, BasebandSampleSource **bs, ChannelAPI **cs) const
{
    if (bs || cs)
    {
        SSTVMod *instance = new SSTVMod(deviceAPI);
        if (bs) {
            *bs = instance;
        }
        if (cs) {
            *cs = instance;
        }
    }
}

#ifdef SERVER_MODE
ChannelGUI* SSTVModPlugin::createTxChannelGUI(DeviceUISet *deviceUISet, BasebandSampleSource *txChannel) const
{
    (void) deviceUISet;
    (void) txChannel;
    return nullptr;
}
#else
ChannelGUI* SSTVModPlugin::createTxChannelGUI(DeviceUISet *deviceUISet, BasebandSampleSource *txChannel) const
{
    return SSTVModGUI::create(m_pluginAPI, deviceUISet, txChannel);
}
#endif

ChannelWebAPIAdapter* SSTVModPlugin::createChannelWebAPIAdapter() const
{
    return new SSTVModWebAPIAdapter();
}
