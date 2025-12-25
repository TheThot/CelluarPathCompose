#pragma once

#include <QVariantList>
#include <QPolygonF>
#include <QPointF>
#include "../libs/a-star/source/AStar.hpp"

class PathFinderCalculator {
public:
    PathFinderCalculator();

    virtual ~PathFinderCalculator();

    static bool isEqual(const QPointF &a, const QPointF &b);

    bool init(const QPointF &pointFrom, const QPointF &pointTo, const QVector<QPolygonF> &obstacles);

    void perform();

    QVariantList collisions();

    QVariantList path();

private:
    static const double _gridSize;  //Размер ячейки области
    static const int _worldOffset;  //Увеличение размера области, для обхода препятствий

    QPointF _pointFrom;
    QPointF _pointTo;
    QVariantList _obstacles;
    QVariantList _collisions;
    QVariantList _path;

    QPointF _pointFrom2d;
    QPointF _pointTo2d;
    QVector<QPolygonF> _obstacles2d;
    QPolygonF _path2d;
    AStar::Generator _generator;

    void clear();

    void initPoints2d();

    void initObstacles2d();

    void buildPath2d();

    bool isIntersect(const QPointF &p1, const QPointF &p2);

    void initGenerator();

    AStar::Vec2i getWorldSize();

    static void getPolygonBoundary(const QPolygonF &polygon, double &xMin, double &yMin, double &xMax, double &yMax);

    static void getWorldCoordinate(double x, double y, int &xWorld, int &yWorld);

    static void worldCellLeftBottomCoordinate(int xWorld, int yWorld, double &x, double &y);

    void initCollisions();

    void simplifyPath2dByIntersect();
};
