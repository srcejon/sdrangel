///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024-2026 Jon Beniston <jon@beniston.com>                       //
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

#ifndef SDRBASE_AVAILABLEDEVICEHANDLER_H_
#define SDRBASE_AVAILABLEDEVICEHANDLER_H_

#include "availabledevice.h"
#include "export.h"
#include "util/messagequeue.h"

class DeviceAPI;

// Utility class to help keeping track of list of available devices and optionally register pipes to them
class SDRBASE_API AvailableDeviceHandler : public QObject
{
    Q_OBJECT

public:

    // Use this constructor to just keep track of available devices with specified URIs and kinds
    AvailableDeviceHandler(QStringList uris) :
        m_uris(uris)
    {
        init();
    }

    // Use this constructor to keep track of available devices with specified URIs and kinds and register pipes with the given names to them
    AvailableDeviceHandler(QStringList uris, QStringList pipeNames) :
        m_uris(uris),
        m_pipeNames(pipeNames)
    {
        init();
    }

    void scanAvailableDevices();

    const AvailableDeviceList& getAvailableDeviceList() const {
        return m_availableDeviceList;
    }

    QObject* registerPipes(const QString& longIdFrom, const QStringList& pipeNames);
    void deregisterPipes(QObject* from, const QStringList& pipeNames);

private:

    AvailableDeviceList m_availableDeviceList;

    QStringList m_uris;             //!< URIs of devices we want to create a list for
    QStringList m_pipeNames;        //!< List of pipe names to register

    void init();
    void registerPipe(const QString& pipeName, QObject *device);

private slots:

    void handleDeviceSetAdded(int deviceSetIndex, DeviceAPI *device);
    void handleDeviceSetRemoved(int deviceSetIndex);

signals:
    void devicesChanged(const QStringList& renameFrom, const QStringList& renameTo, const QStringList& removed, const QStringList& added);  //!< Emitted when list of devices has changed
    void messageEnqueued(MessageQueue *messageQueue);   //!< Emitted when message enqueued to a pipe

};

#endif // SDRBASE_AVAILABLEDEVICEHANDLER_H_
