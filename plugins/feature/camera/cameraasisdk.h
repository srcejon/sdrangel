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
