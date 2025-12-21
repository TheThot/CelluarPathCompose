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

QList<QLineF> PathGenerator::_initNonRespectInnerHoles()
{
    QList<QLineF> res = {};
    if(_survPolygon == nullptr)
        return res;

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
    res = intersectLines;

    return res;
}

PathGenerator::~PathGenerator()
{
    _path.clear();
}

QVariantList PathGenerator::_oneLoopTraj(const QList<QList<QPointF>>& in) const
{
    QVariantList pathTraj;

    QVariantList row;
    for (const auto &lineS: in) {
        for (const auto &p: lineS) {
            QVariantMap pointMap;
            pointMap["x"] = p.x();
            pointMap["y"] = p.y();
            row.append(pointMap);
        }
        pathTraj.append(QVariant::fromValue(row));
        row.clear();
    }

    return pathTraj;
}

QVariantList PathGenerator::pathTraj() const
{
    QVariantList pathTraj;

    // определяем какой у нас активен режим ? от этого будет зависеть возвращаемое в qml значение
    if (!_isHolesActive) {
        pathTraj = _oneLoopTraj(_orientedPathSimpl);
    }
    else {
        pathTraj = _oneLoopTraj(_pathRespectHoles);
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

    _path = _initNonRespectInnerHoles();
    _orientedPathSimpl = _orientNonRespectPath();

    if(_holes != nullptr)
        _pathRespectHoles = _initLinesRespectHoles();
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

QList<QList<QPointF>> PathGenerator::_initLinesRespectHoles()
{
    QList<QList<QPointF>> res = {};
    if(_holes == nullptr)
        return res;

    QLineF holeL;
    for (const auto& currTransect : _path)
    {
        QList<QPointF> resI;
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
                intersectionListFormimgRoutine(currTransect, holeL, resI, QLineF::BoundedIntersection);
            }
        }
        // сразу buff пересечений класть в _pathRespectHoles нельзя
        // сперва отсортировать по расстояниям
        for(const auto& oneIntersection:resI){
            holeL = QLineF(currTransect.p1(), oneIntersection);
            dist.append(holeL.length());
        }
        auto idx = sort_indexes<double>(dist);
        for(const auto& currI:idx){
            buff.append(resI[currI]);
        }
        buff.append(currTransect.p2());
//        std::cout << "[_initLinesRespectHoles] buff length is " << buff.count() << std::endl;
        res.append(buff);
    }

    return res;
}

QPointF PathGenerator::startP() const{
    return _startEndP.first;
}

QPointF PathGenerator::endP() const{
    return _startEndP.second;
}

template<typename Type>
void PathGenerator::_orientLineOneDirection(const QList<Type>& lineList, QList<Type>& resultLines) {
    qreal firstAngle = 0;
    for (const Type & currLine : lineList) {
        Type res_line;
        for(int k = 0; k < currLine.count()-1; ++(++k)) {
            const QLineF &line = QLineF(currLine[k], currLine[k+1]);
            QLineF adjustedLine;

            if (lineList.first() == currLine) {
                firstAngle = line.angle();
            }

            if (qAbs(line.angle() - firstAngle) > 1.0) {
                adjustedLine.setP1(line.p2());
                adjustedLine.setP2(line.p1());
            } else {
                adjustedLine = line;
            }

            res_line << adjustedLine.p1();
            res_line << adjustedLine.p2();
        }
        resultLines.append(res_line);
    }
}

// обратный ход для вектора
template<typename Type>
void PathGenerator::_adjustToLawnower_oneVectorCase(const Type &lineList, Type &resultLines, bool &reverseVertices) {

    Type transectVertices = lineList;
    if (reverseVertices) {
        reverseVertices = false;
        Type reversedVertices;
        for (int j = transectVertices.count() - 1; j >= 0; j--) {
            reversedVertices += transectVertices[j];
        }
        transectVertices = reversedVertices;
    } else {
        reverseVertices = true;
    }
    resultLines = transectVertices;

}

QList<QList<QPointF>> PathGenerator::_orientNonRespectPath(){
    QList<QList<QPointF>> res1 = {};

    if(_path.count() < 1){
        _startEndP.first = QPointF{-1e4, -1e4};
        _startEndP.second = _startEndP.first;
        return res1;
    }

    for(const auto& currL:_path){
        QList<QPointF> buffL;
        buffL   << currL.p1()
                << currL.p2();
        res1.append(buffL);
    }
    // убеждаемся что все линии в одном направлении
    QList<QList<QPointF>> res2 = {};
    _orientLineOneDirection<QList<QPointF>>(res1, res2);

    res1.clear();

    // чередование направления линий змейкой
    bool reverse = false;
    for(const auto& row: res2){
        QList<QPointF> buffL;
        _adjustToLawnower_oneVectorCase(row, buffL, reverse);
        res1.append(buffL);
    }

    // формируем точку входа выхода
    _startEndP.first    = res1[0][0];
    const auto& lastRow =  res1[res1.count()-1];
    _startEndP.second   = lastRow[lastRow.count()-1];

    return res1;
}