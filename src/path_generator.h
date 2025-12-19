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
    Q_PROPERTY(bool showPathCoverage READ showPathCoverage WRITE setShowPathCoverage NOTIFY showPathCoverageChanged)

public:
    PathGenerator() = default;
    ~PathGenerator();

    PathGenerator(QPolygonF& inPolygon, QPolygonF& inBoundaryPoly, double inStep, double inAngle);

    PathGenerator& operator=(const PathGenerator& other);


    QVariantList pathTraj() const;
    bool         showPathCoverage() const;

    void pathUpdation();

    Q_INVOKABLE void setShowPathCoverage(bool in);

signals:
    void pathTrajChanged();
    void updatePath();
    void showPathCoverageChanged();

private:
    void _init();

    QList<QLineF>   _path;
    QPolygonF       _survPolygon;
    QPolygonF       _polyBoundary;
    double          _gridSpace;
    double          _gridAngle;
    bool            _isPathShow;
};
#endif //TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H