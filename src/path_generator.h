//
// Created by Admin on 19.12.2025.
//

#ifndef TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H
#define TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H

#include <memory>
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
    PathGenerator(double inStep, double inAngle, QObject *parent);
    ~PathGenerator();

    QVariantList pathTraj() const;

    void setGridAngle(double in);

    void setPolyHolesList(const QList<QPolygonF>& in);
    void setSurvPoly(const QPolygonF& in);
    void setPolyBoundary(const QPolygonF& in);

public slots:

    void pathUpdation();

signals:
    void pathTrajChanged();
    void updatePath();

private:

    void _initNonRespectInnerHoles();
    void _initLinesRespectHoles();

    QList<QLineF>   _path;
    double          _gridSpace;
    double          _gridAngle;
    QList<QList<QLineF>>   _pathRespectHoles;

    const QList<QPolygonF>* _holes;
    const QPolygonF* _survPolygon = nullptr;
    const QPolygonF* _polyBoundary = nullptr;
};
#endif //TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H