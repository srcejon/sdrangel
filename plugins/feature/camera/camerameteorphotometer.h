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

#ifndef INCLUDE_FEATURE_CAMERAMETEORPHOTOMETER_H_
#define INCLUDE_FEATURE_CAMERAMETEORPHOTOMETER_H_

#include "camerapipelineframe.h"

/**
 * \brief Measures meteor flux and apparent magnitude from detected meteor boxes.
 *
 * The photometer uses the current frame image, YOLO detections labelled "meteor",
 * solved reference stars and their catalog magnitudes. Magnitude is reported only
 * when enough unsaturated reference stars produce a stable zero point.
 */
class CameraMeteorPhotometer
{
public:
    void processFrame(CameraPipelineFrame& frame) const;
};

#endif // INCLUDE_FEATURE_CAMERAMETEORPHOTOMETER_H_
