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

#ifndef INCLUDE_FEATURE_CAMERADETECTIONHISTORYENTRY_H_
#define INCLUDE_FEATURE_CAMERADETECTIONHISTORYENTRY_H_

#include <QDateTime>
#include <QString>

struct CameraDetectionHistoryEntry
{
    QString m_label;
    QDateTime m_firstDetected;
    QDateTime m_disappeared;
    qint64 m_playbackPositionMs = -1;
    int m_playbackFrameNumber = -1;
    float m_peakConfidence = 0.0f;
};

#endif // INCLUDE_FEATURE_CAMERADETECTIONHISTORYENTRY_H_
