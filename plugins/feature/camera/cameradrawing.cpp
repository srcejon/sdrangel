///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>

#include <QFontMetricsF>
#include <QPainter>
#include <QPolygonF>

#include "cameradrawing.h"

namespace {
QRectF normalizedRect(const CameraDrawing& drawing, const QSize& imageSize)
{
    if (drawing.m_points.size() < 2) {
        return QRectF();
    }

    return QRectF(
        CameraDrawingRenderer::imagePoint(drawing.m_points[0], imageSize),
        CameraDrawingRenderer::imagePoint(drawing.m_points[1], imageSize)).normalized();
}

QRectF drawingTextRect(const CameraDrawing& drawing, const QSize& imageSize, const QFont& font)
{
    const QFontMetricsF metrics(font);
    const QStringList lines = drawing.m_text.split(QChar('\n'));
    qreal width = 0.0;
    for (const QString& line : lines) {
        width = std::max(width, metrics.horizontalAdvance(line));
    }
    const qreal height = std::max(1, static_cast<int>(lines.size())) * metrics.lineSpacing();
    return QRectF(
        CameraDrawingRenderer::imagePoint(drawing.m_points.first(), imageSize),
        QSizeF(std::max<qreal>(1.0, width), std::max<qreal>(1.0, height)));
}
}

QPointF CameraDrawingRenderer::imagePoint(const QPointF& normalizedPoint, const QSize& imageSize)
{
    return QPointF(normalizedPoint.x() * imageSize.width(), normalizedPoint.y() * imageSize.height());
}

QFont CameraDrawingRenderer::font(const CameraDrawing& drawing)
{
    QFont result;
    if (!drawing.m_fontFamily.isEmpty()) {
        result.setFamily(drawing.m_fontFamily);
    }
    result.setPixelSize(std::max(1, drawing.m_fontPixelSize));
    result.setBold(drawing.m_fontBold);
    result.setItalic(drawing.m_fontItalic);
    return result;
}

QPolygonF CameraDrawingRenderer::arrowHead(const QPointF& start, const QPointF& end, double lineWidth)
{
    const QPointF direction = end - start;
    const double length = std::hypot(direction.x(), direction.y());
    if (length <= 0.0) {
        return QPolygonF();
    }

    const QPointF unit(direction.x() / length, direction.y() / length);
    const QPointF normal(-unit.y(), unit.x());
    const double headLength = std::max(10.0, lineWidth * 4.5);
    const double headWidth = headLength * 0.55;
    const QPointF base = end - unit * headLength;
    return QPolygonF() << end << base + normal * headWidth << base - normal * headWidth;
}

QPainterPath CameraDrawingRenderer::path(const CameraDrawing& drawing, const QSize& imageSize)
{
    QPainterPath result;
    if (drawing.m_points.isEmpty() || imageSize.isEmpty()) {
        return result;
    }

    const QPointF first = imagePoint(drawing.m_points.first(), imageSize);
    switch (drawing.m_type)
    {
    case CameraDrawing::Line:
    case CameraDrawing::Arrow:
        if (drawing.m_points.size() >= 2)
        {
            result.moveTo(first);
            result.lineTo(imagePoint(drawing.m_points[1], imageSize));
        }
        break;
    case CameraDrawing::Rectangle:
        result.addRect(normalizedRect(drawing, imageSize));
        break;
    case CameraDrawing::Ellipse:
        result.addEllipse(normalizedRect(drawing, imageSize));
        break;
    case CameraDrawing::Freehand:
        result.moveTo(first);
        for (int i = 1; i < drawing.m_points.size(); ++i) {
            result.lineTo(imagePoint(drawing.m_points[i], imageSize));
        }
        break;
    case CameraDrawing::Text:
        break;
    }
    return result;
}

QRectF CameraDrawingRenderer::bounds(const CameraDrawing& drawing, const QSize& imageSize)
{
    if (drawing.m_points.isEmpty() || imageSize.isEmpty()) {
        return QRectF();
    }

    QRectF result;
    if (drawing.m_type == CameraDrawing::Text)
    {
        result = drawingTextRect(drawing, imageSize, font(drawing));
    }
    else
    {
        result = path(drawing, imageSize).boundingRect();
        if ((drawing.m_type == CameraDrawing::Arrow) && (drawing.m_points.size() >= 2))
        {
            result = result.united(arrowHead(
                imagePoint(drawing.m_points[0], imageSize),
                imagePoint(drawing.m_points[1], imageSize),
                drawing.m_lineWidth).boundingRect());
        }
    }

    const double margin = std::max(2.0, drawing.m_lineWidth * 0.75);
    return result.adjusted(-margin, -margin, margin, margin);
}

void CameraDrawingRenderer::draw(QPainter& painter, const CameraDrawing& drawing, const QSize& imageSize)
{
    if (drawing.m_points.isEmpty() || imageSize.isEmpty()) {
        return;
    }

    painter.save();
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    painter.setPen(QPen(drawing.m_strokeColor, std::max(0.5, drawing.m_lineWidth), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    const bool fillShape = drawing.m_fillEnabled
        && ((drawing.m_type == CameraDrawing::Rectangle) || (drawing.m_type == CameraDrawing::Ellipse));
    painter.setBrush(fillShape ? QBrush(drawing.m_fillColor) : Qt::NoBrush);

    if (drawing.m_type == CameraDrawing::Text)
    {
        const QFont textFont = font(drawing);
        painter.setFont(textFont);
        const QRectF textBounds = drawingTextRect(drawing, imageSize, textFont);
        if (drawing.m_fillEnabled) {
            painter.fillRect(textBounds, drawing.m_fillColor);
        }
        painter.drawText(textBounds, Qt::AlignLeft | Qt::AlignTop, drawing.m_text);
    }
    else
    {
        painter.drawPath(path(drawing, imageSize));
        if ((drawing.m_type == CameraDrawing::Arrow) && (drawing.m_points.size() >= 2))
        {
            const QPolygonF head = arrowHead(
                imagePoint(drawing.m_points[0], imageSize),
                imagePoint(drawing.m_points[1], imageSize),
                drawing.m_lineWidth);
            painter.setBrush(drawing.m_strokeColor);
            painter.drawPolygon(head);
        }
    }

    painter.restore();
}

void CameraDrawingRenderer::drawAll(QPainter& painter, const QList<CameraDrawing>& drawings, const QSize& imageSize)
{
    for (const CameraDrawing& drawing : drawings) {
        draw(painter, drawing, imageSize);
    }
}
