///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERADRAWING_H_
#define INCLUDE_FEATURE_CAMERADRAWING_H_

#include <QColor>
#include <QFont>
#include <QList>
#include <QPainterPath>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QSize>
#include <QString>
#include <QVector>

class QPainter;

struct CameraDrawing
{
    static constexpr int MaxDrawingCount = 1000;
    static constexpr int MaxPointsPerDrawing = 20000;

    enum Type
    {
        Line,
        Arrow,
        Rectangle,
        Ellipse,
        Freehand,
        Text
    };

    Type m_type = Line;
    QVector<QPointF> m_points; // Normalized image coordinates.
    double m_lineWidth = 3.0;
    QColor m_strokeColor = QColor(255, 255, 0);
    bool m_fillEnabled = false;
    QColor m_fillColor = QColor(255, 255, 0, 64);
    QString m_text;
    QString m_fontFamily;
    int m_fontPixelSize = 24;
    bool m_fontBold = false;
    bool m_fontItalic = false;
};

class CameraDrawingRenderer
{
public:
    static QPointF imagePoint(const QPointF& normalizedPoint, const QSize& imageSize);
    static QPainterPath path(const CameraDrawing& drawing, const QSize& imageSize);
    static QRectF bounds(const CameraDrawing& drawing, const QSize& imageSize);
    static void draw(QPainter& painter, const CameraDrawing& drawing, const QSize& imageSize);
    static void drawAll(QPainter& painter, const QList<CameraDrawing>& drawings, const QSize& imageSize);

private:
    static QFont font(const CameraDrawing& drawing);
    static QPolygonF arrowHead(const QPointF& start, const QPointF& end, double lineWidth);
};

#endif // INCLUDE_FEATURE_CAMERADRAWING_H_
