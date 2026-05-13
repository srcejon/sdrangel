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
#include <QPointF>
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

struct CameraHistogramData
{
    QVector<float> m_redBins;
    QVector<float> m_greenBins;
    QVector<float> m_blueBins;

    bool isValid() const
    {
        return !m_redBins.isEmpty() && (m_redBins.size() == m_greenBins.size()) && (m_redBins.size() == m_blueBins.size());
    }
};

struct CameraPipelineStarDetection
{
    QPointF m_center;
    QPointF m_projectedCenter;
    float m_peakValue;
    float m_radius;
    float m_qualityScore = 0.0f;
    float m_roundness = 0.0f;
    float m_fillRatio = 0.0f;
    float m_aspectRatio = 0.0f;
    bool m_saturated = false;
    QString m_label;
    float m_matchDistancePixels = 0.0f;
    float m_catalogMagnitude = 0.0f;
    QString m_catalogSpectralType;
    bool m_solved = false;
};

struct CameraPipelineFrame
{
    enum BayerPattern
    {
        BayerNone = 0,
        BayerRGGB,
        BayerBGGR,
        BayerGRBG,
        BayerGBRG
    };

    QImage m_image;
    QImage m_unprocessedImage;
    CameraHistogramData m_histogramData;
    QDateTime m_captureDateTime;
    QVector<QRect> m_motionBoxes;
    QVector<CameraPipelineDetection> m_detections;
    QVector<CameraPipelineStarDetection> m_starDetections;
    bool m_plateSolved = false;
    int m_plateSolvedMatches = 0;
    int m_plateSolveDetectedStarsConsidered = 0;
    int m_plateSolveCatalogStarsLoaded = 0;
    int m_plateSolveCatalogCandidateStars = 0;
    int m_plateSolveOutlierStars = 0;
    float m_plateSolveRmsError = 0.0f;
    float m_plateSolveMaxError = 0.0f;
    float m_plateSolveAzimuth = 0.0f;
    float m_plateSolveElevation = 0.0f;
    float m_plateSolveRoll = 0.0f;
    float m_plateSolveFov = 0.0f;
    float m_plateSolveCenterOffsetX = 0.0f;
    float m_plateSolveCenterOffsetY = 0.0f;
    float m_plateSolveDistortionK1 = 0.0f;
    QString m_plateSolveCatalogSource;
    bool m_saveCurrentImage = false;
    int m_stackCount = 1;
    BayerPattern m_bayerPattern = BayerNone;
};

using CameraPipelineFramePtr = QSharedPointer<CameraPipelineFrame>;

#endif // INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_
