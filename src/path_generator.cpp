//
// Created by Admin on 19.12.2025.
//
#include <iostream>
#include "path_generator.h"

#include "utils.h"

using namespace baseFunc;

PathGenerator::PathGenerator(QObject* parent) :
                            QObject(parent)
{

}

PathGenerator::PathGenerator(double inStep, double inAngle, QObject *parent) :
                            QObject(parent),
                            _gridSpace(inStep),
                            _gridAngle(inAngle)
{
//    _initNonRespectInnerHoles();
}

void PathGenerator::_initNonRespectInnerHoles()
{

    if(_survPolygon == nullptr)
        return;

    // формируем полное покрытие полигона
    QPointF center;
    for(const auto& currP: *_survPolygon){
        center += currP;
    }
    center /= _survPolygon->size();
    QRectF bR = _survPolygon->boundingRect();

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
    intersectLinesWithPolygon(lineList, *_survPolygon, intersectLines);
    _path = intersectLines;

}

PathGenerator::~PathGenerator()
{
    _path.clear();
}

QVariantMap PathGenerator::oneLoopTraj(const QLineF& in) const
{
    QVariantMap lineMap;

    // Точка p1
    QVariantMap point1Map;
    point1Map["x"] = in.p1().x();
    point1Map["y"] = in.p1().y();
    lineMap["p1"] = point1Map;

    // Точка p2
    QVariantMap point2Map;
    point2Map["x"] = in.p2().x();
    point2Map["y"] = in.p2().y();
    lineMap["p2"] = point2Map;

    return lineMap;
}

/*
QVariantMap PathGenerator::twoLoopTraj(const QLineF& in) const
{
    QVariantList result;
    QVariantList row;

    // комбинируем в одном QVariantList все остальные QVariantList'ы
    for (const auto &segments : in) {
        for(const auto &segment : segments) {
            QVariantMap pointMap;
            pointMap["x"] = p.x();
            pointMap["y"] = p.y();
            row.append(pointMap);
        }
        result.append(QVariant::fromValue(row));
        row.clear();
    }
}*/

QVariantList PathGenerator::pathTraj() const
{
    QVariantList pathTraj;

    // определяем какой у нас активен режим ? от этого будет зависеть возвращаемое в qml значение
    if (!_isHolesActive)
        for (const auto& path : _path)
        {
            auto lineMap = oneLoopTraj(path);
            pathTraj.append(lineMap);
        }
    else {

        QVariantList row;
        for (const auto &lineS: _pathRespectHoles) {
            for (const auto &p: lineS) {
                QVariantMap pointMap;
                pointMap["x"] = p.x();
                pointMap["y"] = p.y();
                row.append(pointMap);
            }
            pathTraj.append(QVariant::fromValue(row));
            row.clear();
        }
    }

    return pathTraj;
}

void PathGenerator::setGridAngle(double in)
{
    _gridAngle = in;
}

void PathGenerator::pathUpdation()
{
    _path.clear();
    _pathRespectHoles.clear();

    _initNonRespectInnerHoles();

    if(_holes != nullptr)
        _initLinesRespectHoles();
}

void PathGenerator::setPolyHolesList(const QList<QPolygonF>& in)
{
    _holes = &in;
    _isHolesActive = !_isHolesActive;
}

void PathGenerator::setSurvPoly(const QPolygonF& in)
{
    _survPolygon = &in;
}

void PathGenerator::setPolyBoundary(const QPolygonF& in)
{
    _polyBoundary = &in;
}

void PathGenerator::_initLinesRespectHoles()
{
    if(_holes == nullptr)
        return;

    QLineF holeL;
    for (const auto& currTransect : _path)
    {
        QList<QPointF> res;
        QList<QPointF> buff;
        QList<double> dist;
        buff.append(currTransect.p1());
        for (const auto& currHolePoly: *_holes)
        {
            for (int i = 0; i < currHolePoly.count(); ++i)
            {
                int k = (i + 1) % currHolePoly.count();
                holeL = QLineF(currHolePoly[i], currHolePoly[k]);
                //res - все пересечения линии с любыми _holes
                intersectionListFormimgRoutine(currTransect, holeL, res, QLineF::BoundedIntersection);
            }
        }
        // сразу buff пересечений класть в _pathRespectHoles нельзя
        // сперва отсортировать по расстояниям
        for(const auto& oneIntersection:res){
            holeL = QLineF(currTransect.p1(), oneIntersection);
            dist.append(holeL.length());
        }
        auto idx = sort_indexes<double>(dist);
        for(const auto& currI:idx){
            buff.append(res[currI]);
        }
        buff.append(currTransect.p2());
//        std::cout << "[_initLinesRespectHoles] buff length is " << buff.count() << std::endl;
        _pathRespectHoles.append(buff);
    }
}