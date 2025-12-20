//
// Created by Admin on 19.12.2025.
//

#ifndef TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H
#define TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H

#include <QObject>
#include <QPolygonF>
#include <QLineF>
#include <QVariantList>

class PathGenerator : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList pathTraj READ pathTraj NOTIFY pathTrajChanged)

public:
    PathGenerator() = default;
    PathGenerator(QObject *parent);
    PathGenerator(QPolygonF& inPolygon, QPolygonF& inBoundaryPoly, double inStep, double inAngle, QObject *parent);
    ~PathGenerator();

    QVariantList pathTraj() const;

    void pathUpdation();
    void setGridAngle(double in);

signals:
    void pathTrajChanged();
    void updatePath();

private:

    void _init();

    QList<QLineF>   _path;
    QPolygonF       _survPolygon;
    QPolygonF       _polyBoundary;
    double          _gridSpace;
    double          _gridAngle;
};
#endif //TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H