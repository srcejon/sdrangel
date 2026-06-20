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

#ifndef INCLUDE_FEATURE_CAMERAASISDK_H_
#define INCLUDE_FEATURE_CAMERAASISDK_H_

#include <QRecursiveMutex>

inline QRecursiveMutex& cameraAsiSdkMutex()
{
    static QRecursiveMutex mutex;
    return mutex;
}

/**
 * \brief Scoped (RAII) lock guarding the process-global ZWO ASI SDK.
 *
 * The ASICamera2 SDK keeps global per-process state and is not safe to call concurrently from
 * multiple threads. This locker acquires a single shared QRecursiveMutex (cameraAsiSdkMutex()) on
 * construction and releases it on destruction, so any block of SDK calls can be serialised simply
 * by declaring a CameraAsiSdkLocker at the top of the scope.
 *
 * \note The underlying mutex is recursive, so nested locks on the same thread are permitted.
 * \note Non-copyable and non-movable; intended purely as a stack guard.
 */
class CameraAsiSdkLocker
{
public:
    CameraAsiSdkLocker()
    {
        cameraAsiSdkMutex().lock();
    }

    ~CameraAsiSdkLocker()
    {
        cameraAsiSdkMutex().unlock();
    }

    CameraAsiSdkLocker(const CameraAsiSdkLocker&) = delete;
    CameraAsiSdkLocker& operator=(const CameraAsiSdkLocker&) = delete;
};

#endif // INCLUDE_FEATURE_CAMERAASISDK_H_
