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
    void setDecomposeStruct(const holesInfoIn& in);
    void setPathSegments(const QList<QPolygonF> &in);

public slots:

    void pathUpdation();

signals:

    void pathTrajChanged();
    void updatePath();
    void startEndChanged();

private:

    enum class cellNaming{
        NonHolesPathSeg = 0,
        HolesPathSeg = 1,
    };

    struct pathCoverProgress{
        int totalTransects;
        int numRows; // число галсов если бы не было holes
        int currTransect;
        int currHoleNum; // для зон Holes U и D справедливо для holeToBCD
        QPolygonF* currHole; // для зон Holes U и D справедливо для holeToBCD
        QPolygonF* activeCell; // в этой переменной адрес активной в обработке cell след var со всеми cell
        QHash<const QPolygonF*,QPair<int,double>> zoneLoadRate;   // QPolygonF* ссылка на cell из decomposer и так же упорядочены с границами holes в struct holesInfoIn
        // [double, int] - [доля обработанных трансект, число трансект в этой cell]
    } _pcp;

    struct pathSegmentsAdj{

    };

    QList<QLineF>           _pathSegmRelationToCell(const QPolygonF& inPoly);
    QList<QLineF>           _initNonRespectInnerHoles(const QPolygonF* inPoly);
    QList<QList<QPointF>>   _initLinesRespectHoles(QList<QList<QPair<QPointF,int>>>& pathRespectHolesWithNum);
    QVariantList             _oneLoopTraj(const QList<QList<QPointF>>& in) const;
    QList<QList<QPointF>>   _orientNonRespectPath(const QList<QLineF>& inPath);
    void                    _orientLineOneDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines);
    template<typename Type>
    void                    _orientLineOneDirection(const QList<Type>& lineList, QList<Type>& resultLines);
    template<typename Type>
    void                    _adjustToLawnower_oneVectorCase(const Type &lineList, Type &resultLines, bool &reverseVertices);
    QList<QList<QPointF>>   _pathProcRespectInnerHoles();
    void _debugPrintHolesInfo(const holesInfoIn& info);
    void _qDebugPrintPathRespectHoles(const QList<QList<QPair<QPointF, int>>>& pathData);
    void _qDebugPrintPath(const QList<QList<QPointF>>& pathData);
    QList<QList<QPointF>> _drawComplexCoverPathSequence();
    QMap<int,QList<QList<QPair<QPointF,int>>>> _preProcRespectInnerHoles();
    int goTroughtHoleDirectProc(QVector<int>& holesNumStack, QList<QList<QPointF>>& res);
    bool _updateCountRule();
    QList<QList<QPointF>> _pathRouteBetweenCells(const QHash< const QPolygonF*, QList<QList<QPointF>> >& inPath);

    QPair<QPointF,QPointF> _startEndP;

    bool            _isHolesActive = false;
    QList<QLineF>   _path;
    double          _gridSpace;
    double          _gridAngle;
    QList<QList<QPair<QPointF,int>>>   _pathRespectHolesWithNum; // сопроводительная переменная у каждой точки есть номер QPointF,int зоны с которой пересечение либо -1 если пересеч с внеш границей
    QList<QList<QPointF>>   _pathRespectHoles; // для выдачи в qml используем такое представление
    QList<QLineF>   _linesMannerRespectHoles; // но в обработке при preprocessing лучше держать их так
    QList<QList<QPointF>>   _orientedPathSimpl; // non-Inner Incl path oriented _path var above
    QVector<int>            _holesNumStack;
    QMap<int,
    QList<QList< QPair<QPointF,int> >>    >    _holeMannerPathSegm;
    QHash< const QPolygonF*, QList<QList<QPointF>> >   _pathIntoCell;

    // указатели на переменные в decompose
    const QList<QPolygonF>*     _holes = nullptr;
    const QPolygonF*            _survPolygon = nullptr;
    const QPolygonF*            _polyBoundary = nullptr;
    // два указателя на переменные хранящие информацию по ячейкам cells
    const holesInfoIn*          _decompose = nullptr;
    const QList<QPolygonF>*     _bpd_decompositionCells = nullptr;
    bool                        _currRule;
    double                      _ruleRate;

    PathFinderCalculator *pfc;
};

#endif //TRYCELLUARPATHCOMPOSE_PATH_GENERATOR_H