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
    ~PathGenerator();

    PathGenerator(QPolygonF& inPolygon);

    QVariantList pathTraj() const;

signals:
    void pathTrajChanged();

private:
    QList<QLineF>   _path;
    QPolygonF       _survPolygon;
};
#endif //TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H