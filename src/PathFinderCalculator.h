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
    void init(const QList<QPolygonF> &obstacles, int worldSizeX, int worldSizeY);

    void perform(const QPointF &pointFrom, const QPointF &pointTo, double pathGridMargins);

    QList<QPointF> getPath2d();

private:
    double _pathGridMargins;  //Размер ячейки области
    double _scaleX;
    double _scaleY;
    double _wrldX;
    double _wrldY;
    QPointF _centerArea;

    QPointF _pointFrom2d;
    QPointF _pointTo2d;
    QList<QPolygonF> _obstacles2d;
    QPolygonF _path2d;
    AStar::Generator _generator;

    void clear();

    void buildPath2d();

    void initGenerator(double rectSizeX, double rectSizeY, double pathGridMargins);

    AStar::Vec2i specifyVolume(double rectSizeX, double rectSizeY, double maxObstclSize, double pathGridMargins);

    void initCollisions(const QPoint& centerArea);
    void initCollisions();

    void simplifyPath2dByIntersect();
    bool isIntersect(const QPointF &p1, const QPointF &p2);

    void getWorldCoordinate(double x, double y, int &xWorld, int &yWorld);
};
