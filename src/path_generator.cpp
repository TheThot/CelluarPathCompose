//
// Created by Admin on 19.12.2025.
//
#include "path_generator.h"
#include "utils.h"

using namespace baseFunc;

PathGenerator::PathGenerator(QObject* parent) :
                            QObject(parent)
{

}

PathGenerator::PathGenerator(QPolygonF& inPolygon, QPolygonF& inBoundaryPoly, double inStep, double inAngle, QObject *parent) :
                            QObject(parent),
                            _survPolygon(inPolygon),
                            _polyBoundary(inBoundaryPoly),
                            _gridSpace(inStep),
                            _gridAngle(inAngle)
{
    _init();
}

void PathGenerator::_init()
{

    // формируем полное покрытие полигона
    QPointF center;
    for(const auto& currP: _survPolygon){
        center += currP;
    }
    center /= _survPolygon.size();
    QRectF bR = _survPolygon.boundingRect();

    QList<QLineF> lineList;
    double maxWidth = qMax(bR.width(), bR.height());
    double halfWidth = maxWidth / 2.0;
    double transectX = center.x() - halfWidth;
    double transectXMax = transectX + maxWidth;
    while (transectX < transectXMax) {
        double transectYTop = center.y() - halfWidth;
        double transectYBottom = center.y() + halfWidth;

        lineList += QLineF(rotatePoint(QPointF(transectX, transectYTop), center, _gridAngle + 90),
                           rotatePoint(QPointF(transectX, transectYBottom), center, _gridAngle + 90));
        transectX += _gridSpace;
    }

    // Now intersect the lines with the polygon
    QList<QLineF> intersectLines;
    intersectLinesWithPolygon(lineList, _survPolygon, intersectLines);
    _path = intersectLines;

}

PathGenerator::~PathGenerator()
{
    _path.clear();
}

QVariantList PathGenerator::pathTraj() const
{
    QVariantList pathTraj;

    for (const auto& path : _path)
    {
        QVariantMap lineMap;

        // Точка p1
        QVariantMap point1Map;
        point1Map["x"] = path.p1().x();
        point1Map["y"] = path.p1().y();
        lineMap["p1"] = point1Map;

        // Точка p2
        QVariantMap point2Map;
        point2Map["x"] = path.p2().x();
        point2Map["y"] = path.p2().y();
        lineMap["p2"] = point2Map;

        pathTraj.append(lineMap);
    }

    return pathTraj;
}

void PathGenerator::setGridAngle(double in)
{
    _gridAngle = in;
}

void PathGenerator::pathUpdation()
{
    _init();
}