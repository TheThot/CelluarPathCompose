//
// Created by Admin on 19.12.2025.
//
#include "path_generator.h"
#include "utils.h"

using namespace baseFunc;

PathGenerator::PathGenerator(QPolygonF& inPolygon, QPolygonF& inBoundaryPoly, double inStep, double inAngle) :
                            _survPolygon(inPolygon),
                            _polyBoundary(inBoundaryPoly),
                            _gridSpace(inStep),
                            _gridAngle(inAngle)
{
    _init();
}

PathGenerator& PathGenerator::operator=(const PathGenerator& other) {
    if (this != &other) {
        _gridSpace = other._gridSpace;
        _gridAngle = other._gridAngle;
        _isPathShow = other._isPathShow;
        _polyBoundary = other._polyBoundary;
        _survPolygon = other._survPolygon;
        _path = other._path;
    }
    return *this;
}


void PathGenerator::_init()
{

    // формируем полное покрытие полигона
    QPointF center;
    for(const auto& currP: _polyBoundary){
        center += currP;
    }
    center /= _polyBoundary.size();
    QRectF bR = _polyBoundary.boundingRect();

    QList<QLineF> lineList;
    double maxWidth = qMax(bR.width(), bR.height()) + 2000.0;
    double halfWidth = maxWidth / 2.0;
    double transectX = center.x() - halfWidth;
    double transectXMax = transectX + maxWidth;
    while (transectX < transectXMax) {
        double transectYTop = center.y() - halfWidth;
        double transectYBottom = center.y() + halfWidth;

        lineList += QLineF(rotatePoint(QPointF(transectX, transectYTop), center, _gridAngle),
                           rotatePoint(QPointF(transectX, transectYBottom), center, _gridAngle));
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
        pathTraj.append(QVariant::fromValue(path));
    }

    return pathTraj;
}

bool PathGenerator::showPathCoverage() const {
    return _isPathShow;
}

void PathGenerator::setShowPathCoverage(bool in) {
    if(_isPathShow == in)
        return;

    _isPathShow = in;
    emit showPathCoverageChanged();
}

void PathGenerator::pathUpdation()
{

}