#include "PathFinderCalculator.h"
#include <QDebug>
#include <cmath>
#include <QLineF>

const double PathFinderCalculator::_gridSize = 3;
const int PathFinderCalculator::_worldOffset = 0;

PathFinderCalculator::PathFinderCalculator() {
    clear();
}

PathFinderCalculator::~PathFinderCalculator() {
}

bool PathFinderCalculator::isEqual(const QPointF &a, const QPointF &b) {
    return QLineF{a, b}.length() < 1e6;
}

void PathFinderCalculator::init(const QList<QPolygonF> &obstacles) {
    clear();
    _obstacles2d = obstacles;
}

void PathFinderCalculator::perform(const QPointF &pointFrom, const QPointF &pointTo) {
    if (pointFrom.isNull() || pointTo.isNull())
        return;
    _pointFrom2d = pointFrom;
    _pointTo2d = pointTo;
    buildPath2d();
    simplifyPath2dByIntersect();
}

void PathFinderCalculator::clear() {
    _pointFrom2d = QPointF();
    _pointTo2d = QPointF();
    _obstacles2d.clear();
    _path2d.clear();
    _generator.clearCollisions();
}

void PathFinderCalculator::buildPath2d() {
    initGenerator();
    initCollisions();

    int fromWorldX, fromWorldY, toWorldX, toWorldY;
    getWorldCoordinate(_pointFrom2d.x(), _pointFrom2d.y(), fromWorldX, fromWorldY);
    getWorldCoordinate(_pointTo2d.x(), _pointTo2d.y(), toWorldX, toWorldY);

    auto path = _generator.findPath({fromWorldX, fromWorldY}, {toWorldX, toWorldY});
    std::reverse(path.begin(), path.end());

    size_t pointIndex = 0;
    while (pointIndex < path.size()) {
        if ((pointIndex == 0) && (path.front().x == fromWorldX) && (path.front().y == fromWorldY)) {
            _path2d.append(_pointFrom2d);
        } else if ((pointIndex == (path.size() - 1)) && (path.back().x == toWorldX) && (path.back().y == toWorldY)) {
            _path2d.append(_pointTo2d);
        } else {
            auto point = path[pointIndex];
            double x, y;
            worldCellLeftBottomCoordinate(point.x, point.y, x, y);
            x += _gridSize / 2.0;
            y += _gridSize / 2.0;
            _path2d.append({x, y});
        }
        pointIndex++;
    }
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

void PathFinderCalculator::initGenerator() {
    _generator.setWorldSize(getWorldSize());
    _generator.setHeuristic(AStar::Heuristic::euclidean);
    _generator.setDiagonalMovement(false);
}

AStar::Vec2i PathFinderCalculator::getWorldSize() {
    double xMax = std::max(_pointFrom2d.x(), _pointTo2d.x());
    double yMax = std::max(_pointFrom2d.y(), _pointTo2d.y());

    for (int itemIndex = 0; itemIndex < _obstacles2d.count(); itemIndex++) {
        QPolygonF polygon = _obstacles2d[itemIndex];
        double polygonXMin, polygonXMax, polygonYMin, polygonYMax;
        getPolygonBoundary(polygon, polygonXMin, polygonYMin, polygonXMax, polygonYMax);
        xMax = std::max(xMax, polygonXMax);
        yMax = std::max(yMax, polygonYMax);
    }

    AStar::Vec2i result{(int) std::floor(xMax / _gridSize) + 1 + 2 * _worldOffset, (int) std::floor(yMax / _gridSize) + 1 + 2 * _worldOffset};

    return result;
}

void PathFinderCalculator::getPolygonBoundary(const QPolygonF &polygon, double &xMin, double &yMin, double &xMax, double &yMax) {
    xMin = polygon.value(0).x();
    yMin = polygon.value(0).y();
    xMax = polygon.value(0).x();
    yMax = polygon.value(0).y();

    for (int pointIndex = 0; pointIndex < polygon.count(); pointIndex++) {
        QPointF point = polygon.value(pointIndex);
        if (point.x() < xMin) {
            xMin = point.x();
        }
        if (point.y() < yMin) {
            yMin = point.y();
        }
        if (xMax < point.x()) {
            xMax = point.x();
        }
        if (yMax < point.y()) {
            yMax = point.y();
        }
    }
}

void PathFinderCalculator::getWorldCoordinate(double x, double y, int &xWorld, int &yWorld) {
    xWorld = std::floor(x / _gridSize) + _worldOffset;
    yWorld = std::floor(y / _gridSize) + _worldOffset;
}

void PathFinderCalculator::worldCellLeftBottomCoordinate(int xWorld, int yWorld, double &x, double &y) {
    x = ((double) (xWorld - _worldOffset)) * _gridSize;
    y = ((double) (yWorld - _worldOffset)) * _gridSize;
}

void PathFinderCalculator::initCollisions() {
    for (int polygonIndex = 0; polygonIndex < _obstacles2d.count(); polygonIndex++) {
        QPolygonF polygon = _obstacles2d[polygonIndex];
        double polygonXMin, polygonYMin, polygonXMax, polygonYMax;
        getPolygonBoundary(polygon, polygonXMin, polygonYMin, polygonXMax, polygonYMax);
        int worldXMin, worldYMin, worldXMax, worldYMax;
        getWorldCoordinate(polygonXMin, polygonYMin, worldXMin, worldYMin);
        getWorldCoordinate(polygonXMax, polygonYMax, worldXMax, worldYMax);
        for (int worldX = worldXMin; worldX <= worldXMax; worldX++) {
            for (int worldY = worldYMin; worldY <= worldYMax; worldY++) {
                double xMin, yMin, xMax, yMax;
                worldCellLeftBottomCoordinate(worldX, worldY, xMin, yMin);
                xMax = xMin + _gridSize;
                yMax = yMin + _gridSize;
                QPolygonF collisionCell;
                collisionCell.append({xMin, yMin});
                collisionCell.append({xMin, yMax});
                collisionCell.append({xMax, yMax});
                collisionCell.append({xMax, yMin});
                collisionCell.append({xMin, yMin});
                if (polygon.intersects(collisionCell)) {
                    _generator.addCollision({worldX, worldY});
                }
            }
        }
    }
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

QList<QPointF> PathFinderCalculator::getPath2d() {
    return _path2d.toList();
}
