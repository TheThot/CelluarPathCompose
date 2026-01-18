#include "PathFinderCalculator.h"
#include <QDebug>
#include <cmath>
#include <iostream>
#include <ostream>
#include <QLineF>
#include <PolyBuilder.h>

const int PathFinderCalculator::_worldOffset{10};
const double PathFinderCalculator::_scale{0.5};

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

void PathFinderCalculator::init(const QList<QPolygonF> &obstacles, int wrldX, int wrldY) {
    clear();
    _obstacles2d = obstacles;
    _wrldX = wrldX;
    _wrldY = wrldY;
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
                qDebug() << "Исправлена точка из" << point << "в" << candidate;
                return candidate;
            }
        }

        // Увеличиваем радиус для следующей попытки
        radius *= 1.5;
    }

    qWarning() << "Не удалось найти свободную точку рядом с" << point;
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
    _generator.setWorldSize(specifyVolume(_pathArea));
    _generator.setHeuristic(AStar::Heuristic::octagonal);
    _generator.setDiagonalMovement(true);
    _generator.allowMovementAlongBorders(true);
}

AStar::Vec2i PathFinderCalculator::specifyVolume(const QRectF& intoArea)
{
    //так как в алгоритме Astar числа веществ проводим масштабирование
    int resX, resY;
    resX = std::abs(intoArea.width()) + _worldOffset;
    resY = std::abs(intoArea.height()) + _worldOffset;
    resX *= _scale;
    resY *= _scale;
    return AStar::Vec2i{resX, resY};
}

void PathFinderCalculator::getWorldCoordinate(double x, double y, int &xWorld, int &yWorld) {
    xWorld = x  * _scale;
    yWorld = y * _scale;
}

void PathFinderCalculator::getWorldCoordinate(double x, double y, int &xWorld, int &yWorld, const QRectF& iniArea) {
    xWorld = x - iniArea.left();
    yWorld =  y - iniArea.top();
    xWorld *= _scale;
    yWorld *= _scale;
}


QPointF PathFinderCalculator::backsideCoordConversion(int xWorld, int yWorld)
{
    return QPointF(xWorld / _scale, yWorld / _scale);
}

QPointF PathFinderCalculator::backsideCoordConversion(int xWorld, int yWorld, const QRectF& iniArea)
{
    return QPointF(xWorld / _scale + iniArea.left(),  yWorld / _scale + iniArea.top());
}

void PathFinderCalculator::buildPath2d() {

    const int WORLD_SIZE = 200;
    _generator.setWorldSize({WORLD_SIZE, WORLD_SIZE});
    _generator.setHeuristic(AStar::Heuristic::manhattan);
    _generator.setDiagonalMovement(true);
    _generator.clearCollisions();

    // Находим bounding box всех объектов
    QRectF totalBounds;
    totalBounds.setTopLeft(_pointFrom2d);
    totalBounds.setBottomRight(_pointTo2d);

    for (const auto& obstacle : _obstacles2d) {
        totalBounds = totalBounds.united(obstacle.boundingRect());
    }

    // Добавляем margin
    totalBounds.adjust(-50, -50, 50, 50);

    // Масштабируем в мир 1000x1000
    double scaleX = WORLD_SIZE / totalBounds.width();
    double scaleY = WORLD_SIZE / totalBounds.height();
    double scale = std::min(scaleX, scaleY);

    // Функции преобразования
    auto toWorld = [&](const QPointF& p) -> AStar::Vec2i {
        int x = static_cast<int>((p.x() - totalBounds.left()) * scale);
        int y = static_cast<int>((p.y() - totalBounds.top()) * scale);
        x = std::max(0, std::min(WORLD_SIZE-1, x));
        y = std::max(0, std::min(WORLD_SIZE-1, y));
        return {x, y};
    };

    auto fromWorld = [&](const AStar::Vec2i& p) -> QPointF {
        double x = p.x / scale + totalBounds.left();
        double y = p.y / scale + totalBounds.top();
        return QPointF(x, y);
    };

    // Добавляем препятствия
    for (const auto& obstacle : _obstacles2d) {
        QRectF bounds = obstacle.boundingRect();

        int minX, minY, maxX, maxY;
        AStar::Vec2i tl = toWorld(bounds.topLeft());
        AStar::Vec2i br = toWorld(bounds.bottomRight());

        minX = std::min(tl.x, br.x);
        maxX = std::max(tl.x, br.x);
        minY = std::min(tl.y, br.y);
        maxY = std::max(tl.y, br.y);

        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                QPointF realPoint = fromWorld({x, y});
                if (obstacle.containsPoint(realPoint, Qt::OddEvenFill)) {
                    _generator.addCollision({x, y});
                }
            }
        }
    }

    // Ищем путь
    AStar::Vec2i start = toWorld(_pointFrom2d);
    AStar::Vec2i end = toWorld(_pointTo2d);

    auto path = _generator.findPath(start, end);

    // Преобразуем обратно
    _path2d.clear();
    for (const auto& p : path) {
        _path2d.append(fromWorld(p));
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
    QList<QPoint> temp = {};
    int xWrld, yWrld;
    for(const auto & obst: _obstacles2d){
        for(int i = 0; i < obst.count(); ++i){
            QLineF buff = QLineF(obst[i], obst[(i+1)%obst.count()]);
            auto bl = bresenham_line(QPoint(buff.p1().x(), buff.p1().y()), QPoint(buff.p2().x(), buff.p2().y()));
            for(const auto& res: bl){
                if(area.contains(res)){
                    getWorldCoordinate(res.x(), res.y(), xWrld, yWrld, area);
                    QPoint resI = QPoint(xWrld, yWrld);
                    if(!temp.contains(resI))
                        temp.append(resI);
                }
            }
        }
        for(const auto& currP:temp){
            _generator.addCollision({currP.x(), currP.y()});
        }
    }
}

/*
void PathFinderCalculator::initCollisions() {
    QList<QPoint> temp = {};
    int xWrld, yWrld;
    for(const auto & obst: _obstacles2d){
        for(int i = 0; i < obst.count(); ++i){
            QLineF buff = QLineF(obst[i], obst[(i+1)%obst.count()]);
            auto bl = bresenham_line(QPoint(buff.p1().x(), buff.p1().y()), QPoint(buff.p2().x(), buff.p2().y()));
            for(const auto& res: bl){
                getWorldCoordinate(res.x(), res.y(), xWrld, yWrld);
                QPoint resI = QPoint(xWrld, yWrld);
                if(!temp.contains(resI))
                    temp.append(resI);
            }
        }
        for(const auto& currP:temp){
            int x = currP.x();
            int y = currP.y();
            _generator.addCollision({x, y});
        }
    }
}*/

void PathFinderCalculator::initCollisions() {
    // Очищаем старые препятствия
    _generator.clearCollisions();

    int offSet = 0;

    // Для каждого препятствия
    for (const auto& obstacle : _obstacles2d) {
        // 1. Получаем bounding box препятствия в мировых координатах
        QRectF obstacleBounds = obstacle.boundingRect();

        // 2. Преобразуем в координаты AStar
        int minX, minY, maxX, maxY;
        getWorldCoordinate(obstacleBounds.left(), obstacleBounds.top(),
                          minX, minY, _pathArea);
        getWorldCoordinate(obstacleBounds.right(), obstacleBounds.bottom(),
                          maxX, maxY, _pathArea);

        // 3. Добавляем ВСЕ точки внутри bounding box, которые принадлежат препятствию
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
