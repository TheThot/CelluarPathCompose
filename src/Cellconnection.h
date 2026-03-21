//
// Created by Admin on 07.03.2026.
//
#include <QRectF>
#include <QPolygonF>

#ifndef DECOMPOSERLIB_CELLCONNECTION_H
#define DECOMPOSERLIB_CELLCONNECTION_H
class Cellconnection
{
    QRectF _pathArea;

    QPointF _pointFrom2d;
    QPointF _pointTo2d;
    const QList<QPolygonF>* _holes;
    const QPolygonF* _survPoly;
    QPolygonF _resPath;

    void _buildPath2d();

public:
    Cellconnection();
    ~Cellconnection();

    void init(const QList<QPolygonF>* holes, const QPolygonF* survPoly);
    QList<QPointF> getPath() const;
    void perform(const QPointF &pointFrom, const QPointF &pointTo);
};
#endif //DECOMPOSERLIB_CELLCONNECTION_H