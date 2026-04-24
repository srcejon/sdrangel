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

#include "availabledevicehandler.h"
#include "device/deviceapi.h"
#include "maincore.h"

void AvailableDeviceHandler::init()
{
    QObject::connect(MainCore::instance(), &MainCore::deviceSetAdded, this, &AvailableDeviceHandler::handleDeviceSetAdded);
    QObject::connect(MainCore::instance(), &MainCore::deviceSetRemoved, this, &AvailableDeviceHandler::handleDeviceSetRemoved);
    // Don't call scanAvailableDevices() here, as devicesChanged slot will not yet be connected
    // Owner should call scanAvailableDevices after connection
}

void AvailableDeviceHandler::scanAvailableDevices()
{
    // Get current list of devices with specified URIs and kinds
    AvailableDeviceList availableDeviceList = MainCore::instance()->getAvailableDevices(m_uris, m_kinds);

    // Look for new devices
    for (const auto& device : availableDeviceList)
    {
        if (!m_availableDeviceList.contains(device))
        {
            // Register pipes for any new devices
            for (const auto& pipeName: m_pipeNames) {
                registerPipe(pipeName, device.m_object);
            }
        }
    }

    // Check to see if list has changed
    bool changes = m_availableDeviceList != availableDeviceList;

    // Check to see if anything has been renamed, due to indexes changing after device removal
    QStringList renameFrom;
    QStringList renameTo;
    for (const auto& device : availableDeviceList)
    {
        int index = m_availableDeviceList.indexOfObject(device.m_object);
        if (index >= 0)
        {
            const AvailableDevice& oldEntry = m_availableDeviceList.at(index);
            if (oldEntry.m_index != device.m_index)
            {
                renameFrom.append(oldEntry.getId());
                renameTo.append(device.getId());
                renameFrom.append(oldEntry.getLongId());
                renameTo.append(device.getLongId());
            }
        }
    }

    // Create lists of which devices have been added or removed
    QStringList added;
    QStringList removed;

    for (const auto& device : availableDeviceList)
    {
        if (m_availableDeviceList.indexOfObject(device.m_object) < 0) {
            added.append(device.getId());
        }
    }
    for (const auto& device : m_availableDeviceList)
    {
        if (availableDeviceList.indexOfObject(device.m_object) < 0) {
            removed.append(device.getId());
        }
    }

    m_availableDeviceList = availableDeviceList;

    // Signal if list has changed
    if (changes) {
        emit devicesChanged(renameFrom, renameTo, removed, added);
    }
}

QObject* AvailableDeviceHandler::registerPipes(const QString& longIdFrom, const QStringList& pipeNames)
{
    int index = m_availableDeviceList.indexOfLongId(longIdFrom);
    if (index >= 0)
    {
        QObject *object = m_availableDeviceList[index].m_object;
        for (const auto& pipeName : pipeNames) {
            registerPipe(pipeName, object);
        }
        return object;
    }
    else
    {
        return nullptr;
    }
}

void AvailableDeviceHandler::deregisterPipes(QObject* from, const QStringList& pipeNames)
{
    // Don't dereference 'from' here, as it may have been deleted
    if (from)
    {
        qDebug("AvailableDeviceHandler::deregisterPipes: unregister (%p)", from);
        MessagePipes& messagePipes = MainCore::instance()->getMessagePipes();
        for (const auto& pipeName : pipeNames) {
            messagePipes.unregisterProducerToConsumer(from, this, pipeName);
        }
    }
}

void AvailableDeviceHandler::registerPipe(const QString& pipeName, QObject *device)
{
    qDebug("AvailableDeviceHandler::registerPipe: register %s (%p)", qPrintable(device->objectName()), device);
    MessagePipes& messagePipes = MainCore::instance()->getMessagePipes();

    ObjectPipe *pipe = messagePipes.registerProducerToConsumer(device, this, pipeName);
    MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
    QObject::connect(
        messageQueue,
        &MessageQueue::messageEnqueued,
        this,
        [=](){ emit messageEnqueued(messageQueue); },
        Qt::QueuedConnection
    );
}

void AvailableDeviceHandler::handleDeviceSetAdded(int index, DeviceAPI *device)
{
    (void) index;
    (void) device;

    scanAvailableDevices();
}

void AvailableDeviceHandler::handleDeviceSetRemoved(int index)
{
    (void) index;

    scanAvailableDevices();
}
