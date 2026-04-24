///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by Copilot                                                          //
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

#include "SWGFeatureSettings.h"
#include "SWGFreqDisplaySettings.h"
#include "freqdisplay.h"
#include "freqdisplaywebapiadapter.h"

int FreqDisplayWebAPIAdapter::webapiSettingsGet(
    SWGSDRangel::SWGFeatureSettings& response,
    QString& errorMessage)
{
    (void) errorMessage;
    response.setFreqDisplaySettings(new SWGSDRangel::SWGFreqDisplaySettings());
    response.getFreqDisplaySettings()->init();
    FreqDisplay::webapiFormatFeatureSettings(response, m_settings);

    return 200;
}

int FreqDisplayWebAPIAdapter::webapiSettingsPutPatch(
    bool force,
    const QStringList& featureSettingsKeys,
    SWGSDRangel::SWGFeatureSettings& response,
    QString& errorMessage)
{
    (void) force; // no action
    (void) errorMessage;
    FreqDisplay::webapiUpdateFeatureSettings(m_settings, featureSettingsKeys, response);

    return 200;
}
