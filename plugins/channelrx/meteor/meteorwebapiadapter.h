///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_METEOR_WEBAPIADAPTER_H
#define INCLUDE_METEOR_WEBAPIADAPTER_H

#include "channel/channelwebapiadapter.h"

#include "meteorsettings.h"

class MeteorWebAPIAdapter : public ChannelWebAPIAdapter
{
public:
    MeteorWebAPIAdapter() = default;
    ~MeteorWebAPIAdapter() override = default;

    QByteArray serialize() const override { return m_settings.serialize(); }
    bool deserialize(const QByteArray& data) override { return m_settings.deserialize(data); }

    int webapiSettingsGet(
            SWGSDRangel::SWGChannelSettings& response,
            QString& errorMessage) override;

    int webapiSettingsPutPatch(
            bool force,
            const QStringList& channelSettingsKeys,
            SWGSDRangel::SWGChannelSettings& response,
            QString& errorMessage) override;

private:
    MeteorSettings m_settings;
};

#endif // INCLUDE_METEOR_WEBAPIADAPTER_H
