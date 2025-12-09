//
// Created by Admin on 08.12.2025.
//

#ifndef DECOMPOSER_H
#define DECOMPOSER_H

#include <QObject>
#include <QPolygonF>
#include <QVariantList>
#include <QVector>
#include <QPointF>

class Decomposer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList originalPolygon READ originalPolygon WRITE setOriginalPolygon NOTIFY originalPolygonChanged)
    Q_PROPERTY(QVariantList decompositionCells READ decompositionCells NOTIFY decompositionCellsChanged)
    Q_PROPERTY(QVariantList orientedRect READ orientedRect NOTIFY orientedRectChanged)
    Q_PROPERTY(double sweepAngle READ sweepAngle WRITE setSweepAngle NOTIFY sweepAngleChanged)
    Q_PROPERTY(bool showDecomposition READ showDecomposition WRITE setShowDecomposition NOTIFY showDecompositionChanged)
    Q_PROPERTY(bool showOrientedRect READ showOrientedRect WRITE setShowOrientedRect NOTIFY showOrientedRectChanged)

public:
    explicit Decomposer(QObject *parent = nullptr);

    // Основные свойства
    QVariantList originalPolygon() const;
    QVariantList decompositionCells() const;
    QVariantList orientedRect() const;
    double sweepAngle() const;
    bool showDecomposition() const;
    bool showOrientedRect() const;

    // Методы для вызова из QML
    Q_INVOKABLE void setOriginalPolygon(const QVariantList &polygon);
    Q_INVOKABLE void setSweepAngle(double angle);
    Q_INVOKABLE void setShowDecomposition(bool show);
    Q_INVOKABLE void setShowOrientedRect(bool show);
    Q_INVOKABLE void updateDecomposition();
    Q_INVOKABLE void resetPolygon();

signals:
    void originalPolygonChanged();
    void decompositionCellsChanged();
    void orientedRectChanged();
    void sweepAngleChanged();
    void showDecompositionChanged();
    void showOrientedRectChanged();

private:
    // Алгоритмы декомпозиции
    std::vector<QPolygonF> trapezoidalDecomposition(const QPolygonF& polygon, double sweepAngle);
    QPolygonF getOrientedBoundingRect(const QPolygonF& polygon, double angleDegrees);

    // Утилиты
    QPointF rotatePoint(const QPointF& point, double angle);
    QPointF inverseRotatePoint(const QPointF& point, double angle);
    double computePolygonArea(const QPolygonF& polygon) const;
    bool isPointInPolygon(const QPointF& point, const QPolygonF& polygon);

    // Данные
    QPolygonF m_originalPolygon;
    QVector<QPolygonF> m_decompositionCells;
    QPolygonF m_orientedRect;
    double m_sweepAngle;
    bool m_showDecomposition;
    bool m_showOrientedRect;

    // Предопределенные полигоны
    void createDefaultPolygon();
    void createStarPolygon();
    void createComplexPolygon();
};

#endif