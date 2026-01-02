#pragma once

#include <QPolygonF>
#include <QPointF>
#include <QList>
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
//    double _pathGridMargins;  //Размер ячейки области
    /*double _scaleX;
    double _scaleY;*/
    double _wrldX;
    double _wrldY;
    QRectF _pathArea;
    static const int _worldOffset;
    static const double _scale;

    QPointF _pointFrom2d;
    QPointF _pointTo2d;
    QList<QPolygonF> _obstacles2d;
    QPolygonF _path2d;
    AStar::Generator _generator;

    void clear();

    void buildPath2d();

    void initGenerator();

    AStar::Vec2i specifyVolume(const QRectF& intoArea);

    void initCollisions();
    void initCollisions(const QRectF& area);

    void simplifyPath2dByIntersect();
    bool isIntersect(const QPointF &p1, const QPointF &p2);

    void getWorldCoordinate(double x, double y, int &xWorld, int &yWorld);
    void getWorldCoordinate(double x, double y, int &xWorld, int &yWorld, const QRectF& iniArea);
    QPointF backsideCoordConversion(int xWorld, int yWorld);
    QPointF backsideCoordConversion(int xWorld, int yWorld, const QRectF& iniArea);

    QList<QPoint> bresenham_line(const QPoint& p0, const QPoint& p1);
};
