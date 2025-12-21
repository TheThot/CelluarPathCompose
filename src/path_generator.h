//
// Created by Admin on 19.12.2025.
//

#ifndef TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H
#define TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H

#include <QPolygonF>
#include <QLineF>
#include <QVariantList>

class PathGenerator : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList pathTraj READ pathTraj NOTIFY pathTrajChanged)
    Q_PROPERTY(QPointF startP READ startP NOTIFY startEndChanged)
    Q_PROPERTY(QPointF endP READ endP NOTIFY startEndChanged)

public:
    PathGenerator() = default;
    PathGenerator(QObject *parent);
    PathGenerator(double inStep, double inAngle, QObject *parent);
    ~PathGenerator();

    QVariantList pathTraj() const;
    QPointF startP() const;
    QPointF endP() const;

    void setGridAngle(double in);
    void setTransectWidth(double in);

    void setPolyHolesList(const QList<QPolygonF>& in);
    void setSurvPoly(const QPolygonF& in);
    void setPolyBoundary(const QPolygonF& in);

public slots:

    void pathUpdation();

signals:
    void pathTrajChanged();
    void updatePath();
    void startEndChanged();

private:

    QList<QLineF>           _initNonRespectInnerHoles();
    QList<QList<QPointF>>   _initLinesRespectHoles();
    QVariantList             _oneLoopTraj(const QList<QList<QPointF>>& in) const;
    QList<QList<QPointF>>   _orientNonRespectPath();
    template<typename Type>
    void                    _orientLineOneDirection(const QList<Type>& lineList, QList<Type>& resultLines);
    template<typename Type>
    void                    _adjustToLawnower_oneVectorCase(const Type &lineList, Type &resultLines, bool &reverseVertices);

    QPair<QPointF,QPointF> _startEndP;

    bool            _isHolesActive = false;
    QList<QLineF>   _path;
    double          _gridSpace;
    double          _gridAngle;
    QList<QList<QPointF>>   _pathRespectHoles;
    QList<QList<QPointF>>   _orientedPathSimpl; // non-Inner Incl path oriented _path var above

    const QList<QPolygonF>* _holes = nullptr;
    const QPolygonF* _survPolygon = nullptr;
    const QPolygonF* _polyBoundary = nullptr;
};
#endif //TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H