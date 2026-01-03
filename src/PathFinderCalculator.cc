#include "PathFinderCalculator.h"
#include <QDebug>
#include <cmath>
#include <iostream>
#include <ostream>
#include <QLineF>

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

void PathFinderCalculator::perform(const QPointF &pointFrom, const QPointF &pointTo) {
    if (pointFrom.isNull() || pointTo.isNull())
        return;
    _pointFrom2d = pointFrom;
    _pointTo2d = pointTo;
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
    resX = std::trunc(std::abs(intoArea.width())) + _worldOffset;
    resY = std::trunc(std::abs(intoArea.height())) + _worldOffset;
    resX *= _scale;
    resY *= _scale;
    return AStar::Vec2i{resX, resY};
}

void PathFinderCalculator::getWorldCoordinate(double x, double y, int &xWorld, int &yWorld) {
    xWorld = std::trunc(x  * _scale);
    yWorld = std::trunc(y * _scale);
}

void PathFinderCalculator::getWorldCoordinate(double x, double y, int &xWorld, int &yWorld, const QRectF& iniArea) {
    xWorld = std::trunc( x - iniArea.left() );
    yWorld = std::trunc(  y - iniArea.top() );
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
    /*if(isEqual(_pointFrom2d, _pointTo2d))
        return;*/

    double maxX, maxY, minY, minX;
    maxX = std::max(_pointFrom2d.x(), _pointTo2d.x());
    maxY = std::max(_pointFrom2d.y(), _pointTo2d.y());
    minY = std::min(_pointFrom2d.y(), _pointTo2d.y());
    minX = std::min(_pointFrom2d.x(), _pointTo2d.x());
    _pathArea = QRectF(QPointF{minX,minY}, QPointF{maxX,maxY});
//    _pathArea = QRectF(0, 0, _wrldX, _wrldY);

    initGenerator();
    initCollisions();
    // initCollisions(_pathArea);

    int fromWorldX, fromWorldY, toWorldX, toWorldY;
    /*getWorldCoordinate(_pointFrom2d.x(), _pointFrom2d.y(), fromWorldX, fromWorldY);
    getWorldCoordinate(_pointTo2d.x(), _pointTo2d.y(), toWorldX, toWorldY);*/
    getWorldCoordinate(_pointFrom2d.x(), _pointFrom2d.y(), fromWorldX, fromWorldY, _pathArea);
    getWorldCoordinate(_pointTo2d.x(), _pointTo2d.y(), toWorldX, toWorldY, _pathArea);

    auto path = _generator.findPath({fromWorldX, fromWorldY}, {toWorldX, toWorldY});
    // QList<QPointF> pathPoints;
    for(const auto& p: path){
        _path2d.append(backsideCoordConversion(p.x, p.y, _pathArea));
        // std::cout << pathPoints.last().x() << ", " << pathPoints.last().y() << std::endl;
        // _path2d.append(QPointF(p.x * _scaleX/* + _centerArea.x()*/, p.y * _scaleY /*+ _centerArea.y()*/));
    }
    //_path2d std::movepathPoints;
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
    double step = 1e-5;
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