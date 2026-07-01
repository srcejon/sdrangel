///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018-2019 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
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

#include <QStringList>
#include "devicesoapysdr.h"

DeviceSoapySDR::DeviceSoapySDR()
{
    m_scanner.scan();
}

DeviceSoapySDR::~DeviceSoapySDR()
{}

DeviceSoapySDR& DeviceSoapySDR::instance()
{
    static DeviceSoapySDR inst;
    return inst;
}

SoapySDR::Device *DeviceSoapySDR::openSoapySDR(uint32_t sequence, const QString& hardwareUserArguments)
{
    instance();
    return openopenSoapySDRFromSequence(sequence, hardwareUserArguments);
}

void DeviceSoapySDR::closeSoapySdr(SoapySDR::Device *device)
{
    SoapySDR::Device::unmake(device);
}

SoapySDR::Device *DeviceSoapySDR::openopenSoapySDRFromSequence(uint32_t sequence, const QString& hardwareUserArguments)
{
    if (sequence > m_scanner.getNbDevices())
    {
        return 0;
    }
    else
    {
        const DeviceSoapySDRScan::SoapySDRDeviceEnum& deviceEnum = m_scanner.getDevicesEnumeration()[sequence];

        try
        {
            SoapySDR::Kwargs kwargs;
            kwargs["driver"] = deviceEnum.m_driverName.toStdString();

            if (hardwareUserArguments.size() > 0)
            {
                QStringList kvArgs = hardwareUserArguments.split(',');

                for (int i = 0; i < kvArgs.size(); i++)
                {
                    QStringList kv = kvArgs.at(i).split('=');

                    if (kv.size() > 1) {
                        kwargs[kv.at(0).toStdString()] = kv.at(1).toStdString();
                    }
                }
            }
            else if (deviceEnum.m_idKey.size() > 0)
            {
                kwargs[deviceEnum.m_idKey.toStdString()] = deviceEnum.m_idValue.toStdString();
            }

            // Optional master_clock_rate override via environment.
            // Device-agnostic: applies to any SoapySDR device that honors the
            // master_clock_rate device-arg (SoapyUHD for B200/B210, etc.).
            // Empty/unset = SoapyUHD picks MCR via auto_tick_rate (default).
            // Used by headless harnesses to pin MCR for clean-decim TX/RX rates.
            if (const char *mcr_env = std::getenv("SDRANGEL_USRP_MASTER_CLOCK_RATE_HZ"))
            {
                if (mcr_env[0] != '\0' && kwargs.find("master_clock_rate") == kwargs.end())
                {
                    kwargs["master_clock_rate"] = mcr_env;
                    qDebug("DeviceSoapySDR::openopenSoapySDRFromSequence:"
                           " SDRANGEL_USRP_MASTER_CLOCK_RATE_HZ=%s", mcr_env);
                }
            }

            // Disable auto_tick_rate for SoapyUHD so that the MCR pinned above
            // (or via device args) is preserved across setSampleRate() calls.
            // Without this, UHD re-derives the MCR on setSampleRate/set_tx_rate
            // even when MCR was already set, breaking the rate decimator chain.
            if (deviceEnum.m_driverName == "uhd"
                && kwargs.find("auto_tick_rate") == kwargs.end())
            {
                kwargs["auto_tick_rate"] = "0";
                qDebug("DeviceSoapySDR::openopenSoapySDRFromSequence:"
                       " forced auto_tick_rate=0 for SoapyUHD");
            }

            SoapySDR::Kwargs::const_iterator it = kwargs.begin();

            for (; it != kwargs.end(); ++it) {
                qDebug("DeviceSoapySDR::openopenSoapySDRFromSequence: %s=%s", it->first.c_str(), it->second.c_str());
            }

            SoapySDR::Device *device = SoapySDR::Device::make(kwargs);
            return device;
        }
        catch (const std::exception &ex)
        {
            qWarning("DeviceSoapySDR::openopenSoapySDRFromSequence: %s cannot be opened: %s",
                    deviceEnum.m_label.toStdString().c_str(), ex.what());
            return 0;
        }
    }
}

void DeviceSoapySDR::enumOriginDevices(const QString& hardwareId, PluginInterface::OriginDevices& originDevices)
{
    const std::vector<DeviceSoapySDRScan::SoapySDRDeviceEnum>& devicesEnumeration = getDevicesEnumeration();
    qDebug("SoapySDROutputPlugin::enumOriginDevices: %lu SoapySDR devices", devicesEnumeration.size());
    std::vector<DeviceSoapySDRScan::SoapySDRDeviceEnum>::const_iterator it = devicesEnumeration.begin();

    for (int idev = 0; it != devicesEnumeration.end(); ++it, idev++)
    {
        QString displayedName(QString("SoapySDR[%1:$1] %2").arg(idev).arg(it->m_label));
        QString serial(QString("%1-%2").arg(it->m_driverName).arg(it->m_sequence));

        originDevices.append(PluginInterface::OriginDevice(
            displayedName,
            hardwareId,
            serial,
            idev, // Sequence
            it->m_nbRx, // nb Rx
            it->m_nbTx  // nb Tx
        ));
    }
}
