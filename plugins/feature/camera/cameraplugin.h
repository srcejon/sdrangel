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

#ifndef INCLUDE_FEATURE_CAMERAPLUGIN_H
#define INCLUDE_FEATURE_CAMERAPLUGIN_H

#include <QObject>
#include "plugin/plugininterface.h"

class FeatureGUI;
class WebAPIAdapterInterface;

/**
 * \brief PluginInterface implementation that registers the Camera feature with SDRangel.
 *
 * CameraPlugin is the entry point that the SDRangel plugin manager loads. It advertises
 * the feature via its PluginDescriptor, registers it under Camera::m_featureIdURI during
 * initPlugin(), and acts as a factory for the feature's collaborators: the Camera Feature
 * object, the CameraGUI (non-server builds only), and the CameraWebAPIAdapter.
 *
 * \note A single CameraPlugin instance is created by the plugin framework; the factory
 *       methods are const and produce new objects owned by their callers.
 * \see Camera, CameraGUI, CameraWebAPIAdapter
 */
class CameraPlugin : public QObject, PluginInterface {
    Q_OBJECT
    Q_INTERFACES(PluginInterface)
    Q_PLUGIN_METADATA(IID "sdrangel.feature.camera")

public:
    explicit CameraPlugin(QObject* parent = nullptr);

    const PluginDescriptor& getPluginDescriptor() const override;
    void initPlugin(PluginAPI* pluginAPI) override;

    FeatureGUI* createFeatureGUI(FeatureUISet *featureUISet, Feature *feature) const override;
    Feature* createFeature(WebAPIAdapterInterface *webAPIAdapterInterface) const override;
    FeatureWebAPIAdapter* createFeatureWebAPIAdapter() const override;

private:
    static const PluginDescriptor m_pluginDescriptor;
    PluginAPI* m_pluginAPI;
};

#endif // INCLUDE_FEATURE_CAMERAPLUGIN_H
