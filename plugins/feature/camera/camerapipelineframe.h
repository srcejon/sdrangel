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

#ifndef INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_
#define INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_

#include <QDateTime>
#include <QImage>
#include <QRect>
#include <QSharedPointer>
#include <QString>
#include <QVector>

struct CameraPipelineDetection
{
    QRect m_box;
    QString m_label;
    float m_score;
};

struct CameraPipelineFrame
{
    QImage m_image;
    QImage m_unprocessedImage;
    QDateTime m_captureDateTime;
    QVector<QRect> m_motionBoxes;
    QVector<CameraPipelineDetection> m_detections;
};

using CameraPipelineFramePtr = QSharedPointer<CameraPipelineFrame>;

#endif // INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_
