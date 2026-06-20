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

#ifndef INCLUDE_FEATURE_CAMERA_WEBAPIADAPTER_H
#define INCLUDE_FEATURE_CAMERA_WEBAPIADAPTER_H

#include "feature/featurewebapiadapter.h"
#include "camerasettings.h"

/**
 * \brief Lightweight REST/WebAPI bridge for the Camera feature's settings.
 *
 * CameraWebAPIAdapter lets the camera feature's settings be read and written over the
 * SDRangel WebAPI without instantiating a full Camera feature (e.g. for offline settings
 * import/export). It holds its own CameraSettings instance and delegates serialization,
 * deserialization and SWG formatting to that object and to the static Camera::webapi*
 * helpers, so the on-the-wire format stays identical to the live feature.
 *
 * \note This adapter is standalone state: it does not drive the capture pipeline and is
 *       not connected to a running Camera; it only holds and converts settings.
 * \see Camera, CameraSettings, FeatureWebAPIAdapter
 */
class CameraWebAPIAdapter : public FeatureWebAPIAdapter {
public:
    CameraWebAPIAdapter() = default;
    ~CameraWebAPIAdapter() override = default;

    QByteArray serialize() const override { return m_settings.serialize(); }
    bool deserialize(const QByteArray& data) override { return m_settings.deserialize(data); }

    virtual int webapiSettingsGet(
            SWGSDRangel::SWGFeatureSettings& response,
            QString& errorMessage) override;

    virtual int webapiSettingsPutPatch(
            bool force,
            const QStringList& featureSettingsKeys,
            SWGSDRangel::SWGFeatureSettings& response,
            QString& errorMessage) override;

private:
    CameraSettings m_settings;
};

#endif // INCLUDE_FEATURE_CAMERA_WEBAPIADAPTER_H
