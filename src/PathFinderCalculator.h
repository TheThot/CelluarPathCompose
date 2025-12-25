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

    void init(const QList<QPolygonF> &obstacles);

    void perform(const QPointF &pointFrom, const QPointF &pointTo);

    QList<QPointF> getPath2d();

private:
    static const double _gridSize;  //Размер ячейки области
    static const int _worldOffset;  //Увеличение размера области, для обхода препятствий

    QPointF _pointFrom2d;
    QPointF _pointTo2d;
    QList<QPolygonF> _obstacles2d;
    QPolygonF _path2d;
    AStar::Generator _generator;

    void clear();

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
