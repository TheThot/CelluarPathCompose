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
{

}

Cellconnection::~Cellconnection()
{

}

void Cellconnection::init(const QList<QPolygonF>* holes)
{
    if (_holes == nullptr)
        _holes = holes;
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

    /*if (_isPointInObstacle(_pointFrom2d)) {
        // Ищем ближайшую свободную точку
        _pointFrom2d = _findNearestFreePoint(_pointFrom2d);
    }

    if (_isPointInObstacle(_pointTo2d)) {
        _pointTo2d = _findNearestFreePoint(_pointTo2d);
    }
    */

    _buildPath2d();
}

void Cellconnection::_buildPath2d()
{
    // базовая кратчайшая линия от А к Б
    QLineF fastTrack(_pointFrom2d, _pointTo2d);
    _resPath.clear();
    _resPath.append(_pointFrom2d);
    for (const auto& currHole:*_holes)
    {
        QList<QPointF> intersections;
        QList<QLineF> polyRep;
        QList<QPointF> res;
        for (int i = 0; i < currHole.size(); ++i)
        {
            auto p1 = currHole[i];
            auto p2 = currHole[(i+1)%currHole.size()];
            QLineF holeLine(p1, p2);
            polyRep.push_back(holeLine);
            // накапливаем возможные пересечения
            intersectionListFormimgRoutine(fastTrack, holeLine, intersections,QLineF::BoundedIntersection);
        }
        if (!intersections.isEmpty() && intersections.size() >= 2)
        {
            QLineF intersectionLine(intersections[0], intersections[1]);
            // определяем как обойти пересечение
            extraDangerPointsRoutine(polyRep, intersectionLine, res);
            if (!res.isEmpty() && res.size() >= 1)
            {
                for (auto& curr: res)
                {
                    _resPath.append(curr);
                }
            }
        }
    }
    _resPath.append(_pointTo2d);
}

bool Cellconnection::_isPointInObstacle(const QPointF& point) const {
    PolyBuilder pb = PolyBuilder();
    for (const auto& obstacle : *_holes) {
        QList<QPolygonF> temp;
        temp.push_back(obstacle);
        temp = pb.unitedListWrp(temp, pb.getScale()*4);
        if (temp[0].containsPoint(point, Qt::OddEvenFill)) {
            return true;
        }
    }
    return false;
}

QPointF Cellconnection::_findNearestFreePoint(const QPointF& point) {
    // Если точка внутри препятствия, ищем ближайшую свободную
    if (!_isPointInObstacle(point)) {
        return point;  // Точка уже свободна
    }

    // Поиск в радиусе
    double radius = 1.0;
    const int maxAttempts = 100;

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        // Проверяем точки по окружности
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 8) {
            QPointF candidate(
                    point.x() + radius * cos(angle),
                    point.y() + radius * sin(angle)
            );

            if (!_isPointInObstacle(candidate)) {
                //                qDebug() << "Исправлена точка из" << point << "в" << candidate;
                return candidate;
            }
        }

        // Увеличиваем радиус для следующей попытки
        radius *= 1.5;
    }

    //    qWarning() << "Не удалось найти свободную точку рядом с" << point;
    return point;  // Возвращаем оригинальную
}