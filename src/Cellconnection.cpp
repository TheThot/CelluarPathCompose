//
// Created by Admin on 07.03.2026.
//
#include "Cellconnection.h"
#include "utils.h"

using namespace baseFunc;

Cellconnection::Cellconnection() :
          _pathArea()
        , _pointFrom2d()
        , _pointTo2d()
        , _resPath()
        , _holes(nullptr)
        , _survPoly(nullptr)
{

}

Cellconnection::~Cellconnection()
{

}

void Cellconnection::init(const QList<QPolygonF> *holes, const QPolygonF* survPoly)
{
    if(_holes == nullptr || survPoly == nullptr) {
        _holes = holes;
        _survPoly = survPoly;
    }
}

QList<QPointF> Cellconnection::getPath() const
{
    return _resPath.toList();
}

void Cellconnection::perform(const QPointF &pointFrom, const QPointF &pointTo)
{
    _pointFrom2d = pointFrom;
    _pointTo2d = pointTo;
    _pathArea.setTopLeft(_pointFrom2d);
    _pathArea.setBottomRight(_pointTo2d);

    _buildPath2d();
}

void Cellconnection::_buildPath2d()
{
    // базовая кратчайшая линия от А к Б
    bool flag = false;
    QLineF fastTrack(_pointFrom2d, _pointTo2d);

    _resPath.clear();

    QList<QPointF> intersections;
    QList<QList<QLineF>> polyRepHolesV;

    for (const auto& currHole:*_holes) {
        QList<QLineF> polyRep;
        for (int i = 0; i < currHole.size(); ++i) {
            auto p1 = currHole[i];
            auto p2 = currHole[(i + 1) % currHole.size()];
            QLineF holeLine(p1, p2);
            polyRep.push_back(holeLine);
            // накапливаем возможные пересечения
            intersectionListFormimgRoutine(fastTrack, holeLine, intersections, QLineF::BoundedIntersection);
        }
        polyRepHolesV.push_back(polyRep);
    }
    QList<QLineF> survPolyV;
    for (int i = 0; i < _survPoly->size(); ++i) {
        auto p1 = _survPoly->at(i);
        auto p2 = _survPoly->at((i + 1) % _survPoly->size());
        QLineF survLine(p1, p2);
        intersectionListFormimgRoutine(fastTrack, survLine, intersections, QLineF::BoundedIntersection);
        survPolyV.append(survLine);
    }
    polyRepHolesV.push_back(survPolyV);

    // сортируем по дальности
    std::sort(intersections.begin(), intersections.end(),
              [fastTrack](QPointF& a, QPointF& b)
    {
        auto distA = QLineF(fastTrack.p1(), a).length();
        auto distB = QLineF(fastTrack.p1(), b).length();
        return distA < distB;
    });

    intersections.append(fastTrack.p2()); intersections.prepend(fastTrack.p1());

    for (int i = 0; i < intersections.size()-1; ++i)
    {
        QLineF intersectionLine(intersections[i], intersections[i+1]);
        // определяем как обойти пересечение
        for(const auto& polyRep: polyRepHolesV) {
            QList<QPointF> res;
            flag = extraDangerPointsRoutine(polyRep, intersectionLine, res);
            if (flag) {
                for (auto &curr: res)
                    _resPath.append(curr);
                _resPath.append(intersectionLine.p2());
            }
        }
    }

    if(!_resPath.contains(_pointFrom2d))
        _resPath.prepend(_pointFrom2d);

    if(!_resPath.contains(_pointTo2d))
        _resPath.append(_pointTo2d);
}