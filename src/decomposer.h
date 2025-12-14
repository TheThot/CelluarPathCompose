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
#include <QLineF>

class Decomposer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList originalPolygon READ originalPolygon WRITE setOriginalPolygon NOTIFY originalPolygonChanged)
    Q_PROPERTY(QVariantList decompositionCells READ decompositionCells NOTIFY decompositionCellsChanged)
    Q_PROPERTY(QVariantList orientedRect READ orientedRect NOTIFY orientedRectChanged)
    Q_PROPERTY(double sweepAngle READ sweepAngle WRITE setSweepAngle NOTIFY sweepAngleChanged)
    Q_PROPERTY(bool showDecomposition READ showDecomposition WRITE setShowDecomposition NOTIFY showDecompositionChanged)
    Q_PROPERTY(bool showOrientedRect READ showOrientedRect WRITE setShowOrientedRect NOTIFY showOrientedRectChanged)
    Q_PROPERTY(QVariantList test_2Darray READ test_2Darray WRITE setTest_2Darray)
    Q_PROPERTY(QVariantList holes_2Darray READ holes_2Darray NOTIFY holesPolygonsChanged)
    Q_PROPERTY(QVariantList orientedHoleRects READ orientedHoleRects NOTIFY orientedHoleRectsChanged)

public:
    explicit Decomposer(QObject *parent = nullptr);

    // Основные свойства
    QVariantList holes_2Darray() const;
    QVariantList originalPolygon() const;
    QVariantList decompositionCells() const;
    QVariantList orientedRect() const;
    QVariantList orientedHoleRects() const;
    double sweepAngle() const;
    bool showDecomposition() const;
    bool showOrientedRect() const;
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

    // Методы для вызова из QML
    Q_INVOKABLE void setOriginalPolygon(const QVariantList &polygon);
    Q_INVOKABLE void setSweepAngle(double angle);
    Q_INVOKABLE void setShowDecomposition(bool show);
    Q_INVOKABLE void setShowOrientedRect(bool show);
    Q_INVOKABLE void updateDecomposition();
    Q_INVOKABLE void resetPolygon();
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

signals:
    void originalPolygonChanged();
    void decompositionCellsChanged();
    void orientedRectChanged();
    void sweepAngleChanged();
    void showDecompositionChanged();
    void showOrientedRectChanged();
    void holesPolygonsChanged();
    void orientedHoleRectsChanged();

private:
    enum class OrientPointNames
    {
        LeftBottom = 0,     // [0] - лев ниж
        RightBottom = 1,    // [1] - прав ниж
        RightTop = 2,       // [2] - прав верх
        LeftTop = 3         // [3] - лев верх
    };
    // против часовой стрелки ориентация правосторонняя сист коорд
    enum class OrientedLine
    {
        ParallelSweepL = 2,
        PerpendiclSweepU = 1,
        ParallelSweepR = 0,
        PerpendiclSweepD = 3
    }; // обозначает каждую сторону описывающего прямоугольника
    // Алгоритмы декомпозиции
    std::vector<QPolygonF> trapezoidalDecomposition(const QPolygonF& polygon, double sweepAngle);
    QPolygonF getOrientedBoundingRect(const QPolygonF& polygon, QMap<OrientedLine, QLineF>& currOrient, double angleDegrees);
    QList<QPolygonF> getOrientedBoundingHoleRects(const QPolygonF& polygon, const QList<QPolygonF>& holes, double angleDegrees);
    QList<QPolygonF> boustrophedonDecomposition(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                const QList<QPolygonF>& orientedHoleRects, double sweepAngle);

    // Утилиты
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

    // Данные
    QPolygonF m_originalPolygon;
    QVector<QPolygonF> m_decompositionCells;
    QPolygonF m_orientedRect;
    double m_sweepAngle;
    bool m_showDecomposition;
    bool m_showOrientedRect;
    QList<QPolygonF> m_holes;
    QList<QPolygonF> m_orientedHoleRects;
    QList<QMap<OrientedLine, QLineF>> m_mapOriendtedHoleRectLines; // переменная с информацией какая из ограничивающего holes фигуры паралельна галсу или нет

    // Предопределенные полигоны
    void createDefaultPolygon();
    void createPolygonWithHoles();
};

#endif