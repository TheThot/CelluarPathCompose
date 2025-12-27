#include "PathFinderCalculator.h"
#include <QDebug>
#include <cmath>
#include <QLineF>

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

void PathFinderCalculator::init(const QList<QPolygonF> &obstacles, int worldSizeX, int worldSizeY) {
    clear();
    _obstacles2d = obstacles;
    _wrldX = worldSizeX;
    _wrldY = worldSizeY;
}

void PathFinderCalculator::perform(const QPointF &pointFrom, const QPointF &pointTo, double pathGridMargins) {
    if (pointFrom.isNull() || pointTo.isNull())
        return;
    _pointFrom2d = pointFrom;
    _pointTo2d = pointTo;
    _pathGridMargins = pathGridMargins;
    buildPath2d();
//    simplifyPath2dByIntersect();
}

void PathFinderCalculator::clear() {
    _pointFrom2d = QPointF();
    _pointTo2d = QPointF();
    _obstacles2d.clear();
    _path2d.clear();
    _generator.clearCollisions();
}

void PathFinderCalculator::initGenerator(double rectSizeX, double rectSizeY, double pathGridMargins) {
    double minObstclSize = 1e10;
    for(const auto& currObstcl : _obstacles2d){
        auto currBoundR = currObstcl.boundingRect();
        minObstclSize = std::min(minObstclSize, std::min(currBoundR.height(), currBoundR.width()));
    }
    _generator.setWorldSize(specifyVolume(rectSizeX, rectSizeY, minObstclSize, pathGridMargins));
    _generator.setHeuristic(AStar::Heuristic::octagonal);
    _generator.setDiagonalMovement(true);
}

AStar::Vec2i PathFinderCalculator::specifyVolume(double rectSizeX, double rectSizeY, double obstclSize, double pathGridMargins)
{
    //так как в алгоритме Astar числа веществ проводим масштабирование
    int scaleFact = std::ceil(obstclSize / pathGridMargins);
    _scaleX = _scaleY = scaleFact;
//    _scaleX = _scaleY = 1;
    int resX, resY;
    resX = std::ceil(rectSizeX / _scaleX);
    resY = std::ceil(rectSizeY / _scaleY);
    return AStar::Vec2i{resX, resY};
}

void PathFinderCalculator::getWorldCoordinate(double x, double y, int &xWorld, int &yWorld) {
    xWorld = std::floor(x / _scaleX);
    yWorld = std::floor(y / _scaleY);
}

void PathFinderCalculator::buildPath2d() {
    /*if(isEqual(_pointFrom2d, _pointTo2d))
        return;*/

    double maxX, maxY, minY, minX;
    maxX = std::max(_pointFrom2d.x(), _pointTo2d.x());
    maxY = std::max(_pointFrom2d.y(), _pointTo2d.y());
    minY = std::min(_pointFrom2d.y(), _pointTo2d.y());
    minX = std::min(_pointFrom2d.x(), _pointTo2d.x());
    QRectF pathBounds = QRectF(QPointF{minX,maxY}, QPointF{maxX,minY});

    _centerArea = pathBounds.center();

    initGenerator(_wrldX, _wrldY, _pathGridMargins);
//    initGenerator(std::abs(pathBounds.width()) + _pathGridMargins, std::abs(pathBounds.height()) + _pathGridMargins, _pathGridMargins);
//    initCollisions(QPoint(_centerArea.x(), _centerArea.y()));
    initCollisions();

    int fromWorldX, fromWorldY, toWorldX, toWorldY;
    getWorldCoordinate(_pointFrom2d.x(), _pointFrom2d.y(), fromWorldX, fromWorldY);
    getWorldCoordinate(_pointTo2d.x(), _pointTo2d.y(), toWorldX, toWorldY);

    auto path = _generator.findPath({fromWorldX, fromWorldY}, {toWorldX, toWorldY});
    for(const auto& p: path){
        _path2d.append(QPointF(p.x * _scaleX/* + _centerArea.x()*/, p.y * _scaleY /*+ _centerArea.y()*/));
    }
}

void PathFinderCalculator::initCollisions() {
    QList<QPoint> temp = {};
    double step = 1 / _pathGridMargins;
    for(const auto & obst: _obstacles2d){
        for(int i = 0; i < obst.count(); ++i){
            QLineF buff = QLineF(obst[i], obst[(i+1)%obst.count()]);
            for(double j = 0; j <= 1; j+=step){
                auto res = buff.pointAt(j);
                QPoint resI = QPoint(res.x(), res.y());
                if(!temp.contains(resI))
                    temp.append(resI);
            }
        }
        for(const auto& currP:temp){
            int x = currP.x() / _scaleX;
            int y = currP.y() / _scaleY;
            _generator.addCollision({x, y});
        }
    }
}

void PathFinderCalculator::initCollisions(const QPoint& centerArea) {
    QList<QPoint> temp = {};
    double step = 1 / _pathGridMargins;
    for(const auto & obst: _obstacles2d){
        for(int i = 0; i < obst.count(); ++i){
            QLineF buff = QLineF(obst[i], obst[(i+1)%obst.count()]);
            for(double j = 0; j <= 1; j+=step){
                auto res = buff.pointAt(j);
                QPoint resI = QPoint(res.x(), res.y()) - centerArea;
                if(!temp.contains(resI))
                    temp.append(resI);
            }
        }
        for(const auto& currP:temp){
            int x = currP.x() / _scaleX;
            int y = currP.y() / _scaleY;
            _generator.addCollision({x, y});
        }
    }
}

QList<QPointF> PathFinderCalculator::getPath2d() {
    return _path2d.toList();
}


void PathFinderCalculator::simplifyPath2dByIntersect() {
    QPolygonF newPath;
    newPath.append(_path2d.first());
    int p1Index = 1;
    while (p1Index < _path2d.count() - 1) {
        int p2Index = _path2d.count() - 1;
        QPointF p1 = _path2d[p1Index];
        QPointF p2 = _path2d[p2Index];
        while (isIntersect(p1, p2)) {
            p2Index--;
            p2 = _path2d[p2Index];
        }
        newPath.append(_path2d[p2Index]);
        p1Index = p2Index;
    }
    _path2d = newPath;
}

bool PathFinderCalculator::isIntersect(const QPointF &p1, const QPointF &p2) {
    bool intersected = false;

    QPolygonF line;
    line.append(p1);
    line.append(p2);

    for (int itemIndex = 0; !intersected && (itemIndex < _obstacles2d.count()); itemIndex++) {
        QPolygonF polygon = _obstacles2d[itemIndex];
        intersected = polygon.intersects(line);
    }

    return intersected;
}