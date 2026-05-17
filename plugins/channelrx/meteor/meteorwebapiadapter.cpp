///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include "SWGChannelSettings.h"
#include "SWGMeteorSettings.h"

#include "meteor.h"
#include "meteorwebapiadapter.h"

int MeteorWebAPIAdapter::webapiSettingsGet(
        SWGSDRangel::SWGChannelSettings& response,
        QString& errorMessage)
{
    (void) errorMessage;
    response.setMeteorSettings(new SWGSDRangel::SWGMeteorSettings());
    response.getMeteorSettings()->init();
    Meteor::webapiFormatChannelSettings(response, m_settings);
    return 200;
}

int MeteorWebAPIAdapter::webapiSettingsPutPatch(
        bool force,
        const QStringList& channelSettingsKeys,
        SWGSDRangel::SWGChannelSettings& response,
        QString& errorMessage)
{
    (void) force;
    (void) errorMessage;
    Meteor::webapiUpdateChannelSettings(m_settings, channelSettingsKeys, response);
    return 200;
}
