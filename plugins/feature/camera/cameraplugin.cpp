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
#include "cameragui.h"
#endif
#include "camera.h"
#include "cameraplugin.h"
#include "camerawebapiadapter.h"

const PluginDescriptor CameraPlugin::m_pluginDescriptor = {
    Camera::m_featureId,
    QStringLiteral("Camera"),
    QStringLiteral("7.24.0"),
    QStringLiteral("(c) Jon Beniston, M7RCE"),
    QStringLiteral("https://github.com/srcejon/sdrangel"),
    true,
    QStringLiteral("https://github.com/srcejon/sdrangel")
};

CameraPlugin::CameraPlugin(QObject* parent) :
    QObject(parent),
    m_pluginAPI(nullptr)
{
}

const PluginDescriptor& CameraPlugin::getPluginDescriptor() const
{
    return m_pluginDescriptor;
}

void CameraPlugin::initPlugin(PluginAPI* pluginAPI)
{
    m_pluginAPI = pluginAPI;
    m_pluginAPI->registerFeature(Camera::m_featureIdURI, Camera::m_featureId, this);
}

#ifdef SERVER_MODE
FeatureGUI* CameraPlugin::createFeatureGUI(FeatureUISet *featureUISet, Feature *feature) const
{
    (void) featureUISet;
    (void) feature;
    return nullptr;
}
#else
FeatureGUI* CameraPlugin::createFeatureGUI(FeatureUISet *featureUISet, Feature *feature) const
{
    return CameraGUI::create(m_pluginAPI, featureUISet, feature);
}
#endif

Feature* CameraPlugin::createFeature(WebAPIAdapterInterface* webAPIAdapterInterface) const
{
    return new Camera(webAPIAdapterInterface);
}

FeatureWebAPIAdapter* CameraPlugin::createFeatureWebAPIAdapter() const
{
    return new CameraWebAPIAdapter();
}
