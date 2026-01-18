#include "PathFinderCalculator.h"
#include <QDebug>
#include <cmath>
#include <iostream>
#include <ostream>
#include <QLineF>
#include <PolyBuilder.h>

const int PathFinderCalculator::_worldOffset{50};
const int PathFinderCalculator::_size = 200;

PathFinderCalculator::PathFinderCalculator() {
    clear();
}

PathFinderCalculator::~PathFinderCalculator() {
}

bool PathFinderCalculator::isEqual(const QPointF &a, const QPointF &b) {
    return QLineF{a, b}.length() < 1e-2;
}

void PathFinderCalculator::init(const QList<QPolygonF> &obstacles) {
    clear();
    _obstacles2d = obstacles;
}

bool PathFinderCalculator::isPointInObstacle(const QPointF& point) const {
    PolyBuilder pb = PolyBuilder();
    for (const auto& obstacle : _obstacles2d) {
        QList<QPolygonF> temp;
        temp.push_back(obstacle);
        temp = pb.unitedListWrp(temp, pb.getScale()*4);
        if (temp[0].containsPoint(point, Qt::OddEvenFill)) {
            return true;
        }
    }
    return false;
}

QPointF PathFinderCalculator::findNearestFreePoint(const QPointF& point) {
    // Если точка внутри препятствия, ищем ближайшую свободную
    if (!isPointInObstacle(point)) {
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

            if (!isPointInObstacle(candidate)) {
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


void PathFinderCalculator::perform(const QPointF &pointFrom, const QPointF &pointTo) {
    if (pointFrom.isNull() || pointTo.isNull())
        return;
    _pointFrom2d = pointFrom;
    _pointTo2d = pointTo;

    // ПРОВЕРКА: не находятся ли точки внутри препятствий
    if (isPointInObstacle(_pointFrom2d)) {
        // Ищем ближайшую свободную точку
        _pointFrom2d = findNearestFreePoint(_pointFrom2d);
    }

    if (isPointInObstacle(_pointTo2d)) {
        _pointTo2d = findNearestFreePoint(_pointTo2d);
    }

    buildPath2d();
}

void PathFinderCalculator::clear() {
    _pointFrom2d = QPointF();
    _pointTo2d = QPointF();
    _obstacles2d.clear();
    _path2d.clear();
    _generator.clearCollisions();
}

void PathFinderCalculator::initGenerator() {
    _generator.setWorldSize({_size, _size});
    _generator.setHeuristic(AStar::Heuristic::manhattan);
    _generator.setDiagonalMovement(true);
    _generator.allowMovementAlongBorders(true);
    _generator.clearCollisions();
}

void PathFinderCalculator::getWorldCoordinate(double x, double y, int &xWorld, int &yWorld, const QRectF& iniArea) {
    xWorld = static_cast<int>((x - iniArea.left()) * _scale);
    yWorld = static_cast<int>((y - iniArea.top()) * _scale);
    xWorld = std::max(0, std::min(_size-1, xWorld));
    yWorld = std::max(0, std::min(_size-1, yWorld));
}


QPointF PathFinderCalculator::backsideCoordConversion(int xWorld, int yWorld, const QRectF& iniArea)
{
    double x = xWorld / _scale + iniArea.left();
    double y = yWorld / _scale + iniArea.top();
    return QPointF(x, y);
}

void PathFinderCalculator::buildPath2d() {
    initGenerator();

    // Находим bounding box всех объектов
    QRectF totalBounds;
    totalBounds.setTopLeft(_pointFrom2d);
    totalBounds.setBottomRight(_pointTo2d);

    for (const auto& obstacle : _obstacles2d) {
        totalBounds = totalBounds.united(obstacle.boundingRect());
    }

    // Добавляем margin
    totalBounds.adjust(-_worldOffset, -_worldOffset, _worldOffset, _worldOffset);

    // Масштабируем
    double scaleX = _size / totalBounds.width();
    double scaleY = _size / totalBounds.height();
    _scale = std::min(scaleX, scaleY);

    // Добавляем препятствия
    initCollisions(totalBounds);

    // Ищем путь
    AStar::Vec2i start, end;
    getWorldCoordinate(_pointFrom2d.x(), _pointFrom2d.y(), start.x, start.y, totalBounds);
    getWorldCoordinate(_pointTo2d.x(), _pointTo2d.y(), end.x, end.y, totalBounds);

    auto path = _generator.findPath(start, end);

    // Преобразуем обратно
    _path2d.clear();
    for (const auto& p : path) {
        _path2d.append(backsideCoordConversion(p.x, p.y, totalBounds));
    }

}

QList<QPoint> PathFinderCalculator::bresenham_line(const QPoint& p0, const QPoint& p1){
    QList<QPoint> temp = {};
    QLine line = QLine(p0, p1);
    int dx = std::abs(line.dx());
    int dy = std::abs(line.dy());
    int sx = p0.x() < p1.x() ? 1 : - 1;
    int sy = p0.y() < p1.y() ? 1 : - 1;
    int err = dx - dy;

    bool flag = true;
    QPoint curr = p0;
    while(flag){
        temp.append(curr);

        if(curr.x() == p1.x() && curr.y() == p1.y()){
            flag = false;
        }

        int e2 = 2 * err;
        if( e2 > -dy ) {
            err -= dy;
            curr.setX(curr.x() + sx);
        }
        if( e2 < dx ) {
            err += dx;
            curr.setY(curr.y() + sy);
        }
    }
    return temp;
}

void PathFinderCalculator::initCollisions(const QRectF& area) {
    for (const auto& obstacle : _obstacles2d) {
        QRectF bounds = obstacle.boundingRect();

        int minX, minY, maxX, maxY;
        AStar::Vec2i tl, br;
        getWorldCoordinate(bounds.topLeft().x(), bounds.topLeft().y(), tl.x, tl.y, area);
        getWorldCoordinate(bounds.bottomRight().x(), bounds.bottomRight().y(), br.x, br.y, area);

        minX = std::min(tl.x, br.x);
        maxX = std::max(tl.x, br.x);
        minY = std::min(tl.y, br.y);
        maxY = std::max(tl.y, br.y);

        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                QPointF realPoint = backsideCoordConversion(x, y, area);
                if (obstacle.containsPoint(realPoint, Qt::OddEvenFill)) {
                    _generator.addCollision({x, y});
                }
            }
        }
    }
}

void PathFinderCalculator::initCollisions() {
    // Очищаем старые препятствия
    _generator.clearCollisions();

    int offSet = 0;

    // Для каждого препятствия
    for (const auto& obstacle : _obstacles2d) {
        // Получаем bounding box препятствия в мировых координатах
        QRectF obstacleBounds = obstacle.boundingRect();

        // Преобразуем в координаты AStar
        int minX, minY, maxX, maxY;
        getWorldCoordinate(obstacleBounds.left(), obstacleBounds.top(),
                          minX, minY, _pathArea);
        getWorldCoordinate(obstacleBounds.right(), obstacleBounds.bottom(),
                          maxX, maxY, _pathArea);

        // Добавляем ВСЕ точки внутри bounding box, которые принадлежат препятствию
        for (int x = minX - offSet; x <= maxX + offSet; ++x) {
            for (int y = minY - offSet; y <= maxY + offSet; ++y) {
                // Преобразуем обратно в реальные координаты для проверки
                QPointF realPoint = backsideCoordConversion(x, y, _pathArea);

                if (obstacle.containsPoint(realPoint, Qt::OddEvenFill)) {
                    _generator.addCollision({x, y});
                }
            }
        }
    }
}

QList<QPointF> PathFinderCalculator::getPath2d() {
    return _path2d.toList();
}
