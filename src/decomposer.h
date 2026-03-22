//
// Created by Admin on 08.12.2025.
//

#ifndef DECOMPOSER_H
#define DECOMPOSER_H

#include "path_generator.h"
#include "PolyBuilder.h"
#include <QObject>
#include <QVector>
#include <QPointF>

class Decomposer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList originalPolygon READ originalPolygon WRITE setOriginalPolygon NOTIFY originalPolygonChanged)
    Q_PROPERTY(QVariantList decompositionCells READ decompositionCells NOTIFY decompositionCellsChanged)
    Q_PROPERTY(QVariantList bpdDecompositionCells READ bpdDecompositionCells NOTIFY decompositionCellsChanged)
    Q_PROPERTY(QVariantList orientedRect READ orientedRect NOTIFY orientedRectChanged)
    Q_PROPERTY(double transectWidth READ transectWidth WRITE setTransectWidth NOTIFY sweepAngleChanged)
    Q_PROPERTY(double sweepAngle READ sweepAngle WRITE setSweepAngle NOTIFY trWdthChanged)
    Q_PROPERTY(bool showDecomposition READ showDecomposition WRITE setShowDecomposition NOTIFY showDecompositionChanged)
    Q_PROPERTY(bool showOrientedRect READ showOrientedRect WRITE setShowOrientedRect NOTIFY showOrientedRectChanged)
    Q_PROPERTY(QVariantList test_2Darray READ test_2Darray WRITE setTest_2Darray)
    Q_PROPERTY(QVariantList holes_2Darray READ holes_2Darray NOTIFY holesPolygonsChanged)
    Q_PROPERTY(QVariantList orientedHoleRects READ orientedHoleRects NOTIFY orientedHoleRectsChanged)
    Q_PROPERTY(PathGenerator* transects  READ transects  CONSTANT)
    Q_PROPERTY(bool showPathCoverage READ showPathCoverage WRITE setShowPathCoverage NOTIFY showPathCoverageChanged)
    Q_PROPERTY(bool decmpsKind READ decmpsKind WRITE setDecmpsKind)

public:

    explicit Decomposer(QObject *parent = nullptr);

    explicit Decomposer(double pathSpace, double angle, QObject *parent = nullptr);

    // Основные свойства
    double      transectWidth() const;
    PathGenerator* transects() const;
    QVariantList holes_2Darray() const;
    QVariantList originalPolygon() const;
    QVariantList decompositionCells() const;
    QVariantList bpdDecompositionCells() const;
    QVariantList orientedRect() const;
    QVariantList orientedHoleRects() const;
    double sweepAngle() const;
    bool showDecomposition() const;
    bool showOrientedRect() const;
    bool showPathCoverage() const;
    bool decmpsKind() const;
    QVariantList test_2Darray() const {
        QVariantList result;

        // Создаем QList<QVariantList>
        QList<QVariantList> data = {
            {1, 2, 3},
            {5, 6, 7, 8},
            {9, 10}
        };

        // комбинируем в одном QVariantList все остальные QVariantList'ы
        for (const QVariantList &row : data) {
            result.append(QVariant::fromValue(row));
        }

        return result;
    }

    QList<QList<QPointF>> config(const QPolygonF& survPoly, const QList<QPolygonF>& holes = QList<QPolygonF>{});
    // Методы для вызова из QML
    Q_INVOKABLE void setTransectWidth(double w);
    Q_INVOKABLE void setShowPathCoverage(bool in);
    Q_INVOKABLE void setDecmpsKind(bool in);
    Q_INVOKABLE void setOriginalPolygon(const QVariantList &polygon);
    Q_INVOKABLE void setSweepAngle(double angle);
    Q_INVOKABLE void setShowDecomposition(bool show);
    Q_INVOKABLE void setShowOrientedRect(bool show);
    Q_INVOKABLE void updateDecomposition();
    Q_INVOKABLE void resetPolygon(int idx);
    Q_INVOKABLE void resetToPolygonWithHoleState();
    Q_INVOKABLE void setTest_2Darray(const QVariantList &polygon)
    {
        for (const QVariant &pointVar : polygon) {
            auto curr = pointVar.toList();
            QPolygonF poly;
            for (const QVariant &currVar : curr)
            {
                poly << currVar.toPointF();
            }
        }
    }

    void preprocPolys(QPolygonF& survPoly, QList<QPolygonF>& holes);
    QList<QPolygonF> getOrientedBoundingHoleRects(const QPolygonF& polygon, const QList<QPolygonF>& holes, double angleDegrees);

signals:
    void updateCanvas();
    void originalPolygonChanged();
    void decompositionCellsChanged();
    void showDecompositionChanged();
    void orientedRectChanged();
    void sweepAngleChanged();
    void showOrientedRectChanged();
    void trWdthChanged();

    void holesPolygonsChanged();
    void orientedHoleRectsChanged();
    void showPathCoverageChanged();

private:
    void _init();
    // Алгоритмы декомпозиции
    QList<QPolygonF> trapezoidalDecomposition(const QPolygonF& polygon, double sweepAngle);
    QPolygonF getOrientedBoundingRect(const QPolygonF& polygon, double angleDegrees);
    QList<QPolygonF> boustrophedonDecomposition_compact(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                        double sweepAngle);
    QList<QPolygonF> boustrophedonDecomposition(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                double sweepAngle);

    // Утилиты
    template <typename T>
    T rotationStruct(const T& v, double sweepAngle);
    QPointF updateCenterBeforeRot(QPolygonF& poly);
    QPointF rotatePoint(const QPointF& point, double angle);
    QPointF inverseRotatePoint(const QPointF& point, double angle);
    double computePolygonArea(const QPolygonF& polygon) const;

    template< typename Type > QVariantList configListVariantLists(Type in_array) const{
        QVariantList result;
        QVariantList row;

        // комбинируем в одном QVariantList все остальные QVariantList'ы
        for (const auto &one_elem : in_array) {
            for(const auto &p : one_elem) {
                QVariantMap pointMap;
                pointMap["x"] = p.x();
                pointMap["y"] = p.y();
                row.append(pointMap);
            }
            result.append(QVariant::fromValue(row));
            row.clear();
        }
        return result;
    }

    void _debugPolyListToConsole(const QList<QPolygonF>& polygons);

    // Данные
    QPolygonF m_originalPolygon;
    QList<QPolygonF> m_decompositionCells;
    QList<QPolygonF> m_bpd_decompositionCells;
    QPolygonF m_orientedRect;
    double m_sweepAngle;
    bool m_showDecomposition;
    bool m_showOrientedRect;
    bool _decmpsKind;
    QList<QPolygonF> m_holes;
    QList<QPolygonF> m_orientedHoleRects;

    PolyBuilder    _pb;
    PathGenerator* _transects;
    bool            _isPathShow;
    double          _trWidth;

    // Предопределенные полигоны
    void createDefaultPolygon();
    void createPolygonWithHoles();
    void createPolygonWithHoles(int idx);
};

#endif