///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024-2026 Jon Beniston <jon@beniston.com>                        //
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

#ifndef SDRBASE_AVAILABLEDEVICE_H_
#define SDRBASE_AVAILABLEDEVICE_H_

#include <QString>
#include <QObject>

#include "export.h"

struct SDRBASE_API AvailableDevice
{
    QChar m_kind;           //!< 'R' or 'T' for channel, 'M' for MIMO
    int m_index;            //!< Device set index
    QString m_type;         //!< Device type (E.g. HackRF)
    QObject *m_object;      //!< Pointer to the object (DeviceAPI object)

    AvailableDevice() = default;
    AvailableDevice(const AvailableDevice&) = default;
    AvailableDevice& operator=(const AvailableDevice&) = default;

    bool operator==(const AvailableDevice& a) const;
    QString getId() const; //!< Eg: "R3"
    QString getLongId() const; //!< Eg: "T1 HackRF"
};

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
inline uint qHash(const AvailableDevice &c, uint seed = 0) noexcept
{
    return qHash(c.getLongId(), seed);
}
#else
inline size_t qHash(const AvailableDevice &c, size_t seed = 0) noexcept
{
    return qHash(c.getLongId(), seed);
}
#endif

class SDRBASE_API AvailableDeviceList : public QList<AvailableDevice>
{
public:
    AvailableDeviceList() {}
    inline explicit AvailableDeviceList(const AvailableDevice &i) { append(i); }

    int indexOfObject(const QObject *object, int from=0) const;     //!< // Find index of entry containing specified object. -1 if not found.
    int indexOfId(const QString& longId, int from=0) const;         //!< // Find index of entry with specified Id. -1 if not found.
    int indexOfLongId(const QString& longId, int from=0) const;     //!< // Find index of entry with specified long Id. -1 if not found.
};

#endif // SDRBASE_AVAILABLEDEVICE_H_
