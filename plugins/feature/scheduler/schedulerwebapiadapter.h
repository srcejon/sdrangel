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

#ifndef INCLUDE_SCHEDULER_WEBAPIADAPTER_H_
#define INCLUDE_SCHEDULER_WEBAPIADAPTER_H_

#include "feature/featurewebapiadapter.h"

#include "schedulersettings.h"

class SchedulerWebAPIAdapter : public FeatureWebAPIAdapter
{
public:
    SchedulerWebAPIAdapter() = default;
    ~SchedulerWebAPIAdapter() override = default;

    QByteArray serialize() const override { return m_settings.serialize(); }
    bool deserialize(const QByteArray& data) override { return m_settings.deserialize(data); }

private:
    SchedulerSettings m_settings;
};

#endif // INCLUDE_SCHEDULER_WEBAPIADAPTER_H_
