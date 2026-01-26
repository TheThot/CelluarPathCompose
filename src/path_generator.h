//
// Created by Admin on 19.12.2025.
//

#ifndef TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H
#define TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H

#include "utils.h"
#include "PathFinderCalculator.h"
#include <QPolygonF>
#include <QLineF>
#include <QVariantList>

class PathGenerator : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList pathTraj READ pathTraj NOTIFY pathTrajChanged)
    Q_PROPERTY(QVariantList pathConnection READ pathConnection NOTIFY pathTrajChanged)
    Q_PROPERTY(QVariantList connPList READ connPList NOTIFY connPListChanged)

public:

    PathGenerator() = default;
    PathGenerator(QObject *parent);
    PathGenerator(double inStep, double inAngle, QObject *parent);
    ~PathGenerator();

    void changeHolesActiveState();

    QVariantList pathTraj() const;
    QVariantList pathConnection() const;
    QVariantList connPList() const;

    void setGridAngle(double in);
    void setTransectWidth(double in);

    void setPolyHolesList(const QList<QPolygonF>& in);
    void setSurvPoly(const QPolygonF& in);
    void setPolyBoundary(const QPolygonF& in);
    void setDecomposeStruct(const holesInfoIn& in);
    void setPathSegments(const QList<QPolygonF> &in);

    void setIsHoleActive(bool in);

    QList<QLineF>           initNonRespectInnerHoles(const QPolygonF* inPoly);
    QList<QList<QPointF>>   orientNonRespectPath(const QList<QLineF>& inPath);
    QList<QList<QPointF>>   performRespectPathCnstr(const QList<QPolygonF>& dcmpsRes, const QList<QPolygonF>& holes);

public slots:

    void pathUpdation();

signals:

    void pathTrajChanged();
    void updatePath();
    void connPListChanged();

private:

    enum class cellNaming{
        NonHolesPathSeg = 0,
        HolesPathSeg = 1,
    };

    QList<QList<QPointF>>   _improvePathRespectCell(const QList<QList<QPointF>>& resPointList, const QPolygonF& cell);
    QList<QLineF>           _pathSegmRelationToCell(const QPolygonF& inPoly);
    QList<QLineF>           _initNonRespectInnerHoles(const QPolygonF* inPoly);
    QVariantList             _oneLoopTraj(const QList<QList<QPointF>>& in) const;
    QList<QList<QPointF>>   _orientNonRespectPath(const QList<QLineF>& inPath);
    template<typename Type>
    void                    _orientLineOneDirection(const QList<Type>& lineList, QList<Type>& resultLines);
    template<>
    void                    _orientLineOneDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines);

    QHash< const QPolygonF*,
    QList<QList<QPointF>> >  _pathRouteCells(const QHash< const QPolygonF*, QList<QList<QPointF>> >& inPath);
    QHash< const QPolygonF*,
    QList<QList<QPointF>> > _configurePathIntoCell(QVector< const QPolygonF* >& order, QVector< QPair<QPointF, QPointF> >& flP);
    QList<QList<QPointF>>   _pathRouteConnections(const QVector<QPair<QPointF, QPointF>>& inPath);

    // Debug utilz
    void _debugPrintHolesInfo(const holesInfoIn& info);
    void _qDebugPrintPathRespectHoles(const QList<QList<QPair<QPointF, int>>>& pathData);
    void _qDebugPrintPath(const QList<QList<QPointF>>& pathData);

    bool            _isHolesActive = false;
    QList<QLineF>   _path;
    double          _gridSpace;
    double          _gridAngle;

    QList<QList<QPointF>>   _pathConnectionLines;
    QList<QList<QPointF>>   _pathRespectHoles; // для выдачи в qml используем такое представление
    QList<QList<QPointF>>   _orientedPathSimpl; // non-Inner Incl path oriented _path var above
    QVector<int>            _holesNumStack;
    QHash< const QPolygonF*, QList<QList<QPointF>> >   _pathIntoCell;
    QVector< QPair<QPointF, QPointF> >              _startEndPointsIntoCell;
    QVector< const QPolygonF* >                     _ordering;

    // указатели на переменные в decompose
    const QList<QPolygonF>*     _holes = nullptr;
    const QPolygonF*            _survPolygon = nullptr;
    const QPolygonF*            _polyBoundary = nullptr;
    // два указателя на переменные хранящие информацию по ячейкам cells
    const holesInfoIn*          _decompose = nullptr;
    const QList<QPolygonF>*     _bpd_decompositionCells = nullptr;

    PathFinderCalculator *pfc;
};

#endif //TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H