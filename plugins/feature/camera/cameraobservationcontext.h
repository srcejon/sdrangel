///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as version 3 of the       //
// License, or                                                                   //
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

#ifndef INCLUDE_FEATURE_CAMERA_OBSERVATION_CONTEXT_H_
#define INCLUDE_FEATURE_CAMERA_OBSERVATION_CONTEXT_H_

#include "camerapipelineframe.h"
#include "camerasettings.h"

namespace CameraObservationContext {

inline CameraSettings projectionSettingsForFrame(
    const CameraSettings& settings,
    const CameraPipelineFrame& frame)
{
    CameraSettings result = settings;

    if ((settings.m_siteSource == CameraSettings::SiteSourceMediaMetadata)
        && frame.m_mediaMetadata.isValid())
    {
        result.m_latitude = static_cast<float>(frame.m_mediaMetadata.latitude());
        result.m_longitude = static_cast<float>(frame.m_mediaMetadata.longitude());
        result.m_altitude = static_cast<float>(frame.m_mediaMetadata.altitude());
    }
    else if (!settings.m_siteApplyToCurrentImage && frame.m_observationContext.m_valid)
    {
        result.m_latitude = frame.m_observationContext.m_latitude;
        result.m_longitude = frame.m_observationContext.m_longitude;
        result.m_altitude = frame.m_observationContext.m_altitude;
    }

    if ((settings.m_directionSource == CameraSettings::DirectionSourceMediaMetadata)
        && frame.m_mediaMetadata.isValid())
    {
        result.m_azimuth = static_cast<float>(frame.m_mediaMetadata.azimuth());
        result.m_elevation = static_cast<float>(frame.m_mediaMetadata.elevation());
        result.m_roll = static_cast<float>(frame.m_mediaMetadata.roll());
    }
    else if (!settings.m_directionApplyToCurrentImage && frame.m_captureDirection.m_valid)
    {
        result.m_azimuth = frame.m_captureDirection.m_azimuth;
        result.m_elevation = frame.m_captureDirection.m_elevation;
        result.m_roll = frame.m_captureDirection.m_roll;
    }

    if ((settings.m_projectionSource == CameraSettings::ProjectionSourceMediaMetadata)
        && frame.m_mediaMetadata.isValid())
    {
        result.m_fov = static_cast<float>(frame.m_mediaMetadata.fov());
        result.m_lensProjection = static_cast<CameraSettings::LensProjection>(frame.m_mediaMetadata.lensProjection());
        result.m_lensCenterOffsetX = frame.m_mediaMetadata.lensCenterOffsetX();
        result.m_lensCenterOffsetY = frame.m_mediaMetadata.lensCenterOffsetY();
        result.m_lensDistortionK1 = frame.m_mediaMetadata.lensDistortionK1();
        result.m_lensMirror = frame.m_mediaMetadata.lensMirror();
    }
    else if (!settings.m_projectionApplyToCurrentImage && frame.m_observationContext.m_valid)
    {
        result.m_fov = frame.m_observationContext.m_fov;
        result.m_lensProjection = static_cast<CameraSettings::LensProjection>(frame.m_observationContext.m_lensProjection);
        result.m_lensCenterOffsetX = frame.m_observationContext.m_lensCenterOffsetX;
        result.m_lensCenterOffsetY = frame.m_observationContext.m_lensCenterOffsetY;
        result.m_lensDistortionK1 = frame.m_observationContext.m_lensDistortionK1;
        result.m_lensMirror = frame.m_observationContext.m_lensMirror;
    }

    return result;
}

inline QDateTime dateTimeForFrame(const CameraSettings& settings, const CameraPipelineFrame& frame)
{
    CameraSettings::ObservationTimeSource source = settings.m_observationTimeSource;
    // Keep direct legacy users working until every external caller has moved to the enum.
    if ((source == CameraSettings::ObservationTimeCapture) && !settings.m_plateSolveUseCaptureDateTime) {
        source = CameraSettings::ObservationTimeCustom;
    }

    switch (source)
    {
    case CameraSettings::ObservationTimeCurrent:
        if (settings.m_observationTimeApplyToCurrentImage) {
            return QDateTime::currentDateTime();
        }
        if (frame.m_observationContext.m_valid && frame.m_observationContext.m_dateTime.isValid()) {
            return frame.m_observationContext.m_dateTime;
        }
        return frame.m_captureDateTime;
    case CameraSettings::ObservationTimeCustom:
        if (settings.m_observationTimeApplyToCurrentImage && settings.m_plateSolveDateTime.isValid()) {
            return settings.m_plateSolveDateTime;
        }
        if (frame.m_observationContext.m_valid && frame.m_observationContext.m_dateTime.isValid()) {
            return frame.m_observationContext.m_dateTime;
        }
        return settings.m_plateSolveDateTime.isValid() ? settings.m_plateSolveDateTime : frame.m_captureDateTime;
    case CameraSettings::ObservationTimeCapture:
    default:
        if (frame.m_captureDateTime.isValid()) {
            return frame.m_captureDateTime;
        }
        return (frame.m_observationContext.m_valid && frame.m_observationContext.m_dateTime.isValid())
            ? frame.m_observationContext.m_dateTime
            : QDateTime();
    }
}

} // namespace CameraObservationContext

#endif // INCLUDE_FEATURE_CAMERA_OBSERVATION_CONTEXT_H_
