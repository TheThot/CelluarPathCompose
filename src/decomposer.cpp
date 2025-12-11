//
// Created by Admin on 08.12.2025.
//
#define _USE_MATH_DEFINES
#include "decomposer.h"
#include <cmath>
#include <algorithm>
#include <vector>


Decomposer::Decomposer(QObject *parent)
    : QObject(parent)
    , m_sweepAngle(0.0)
    , m_showDecomposition(true)
    , m_showOrientedRect(true)
{
    createDefaultPolygon();
    updateDecomposition();
}

QVariantList Decomposer::originalPolygon() const {
    QVariantList list;
    for (const QPointF &point : m_originalPolygon) {
        QVariantMap pointMap;
        pointMap["x"] = point.x();
        pointMap["y"] = point.y();
        list.append(pointMap);
    }
    return list;
}

QVariantList Decomposer::decompositionCells() const {
    QVariantList cellsList;
    for (const QPolygonF &cell : m_decompositionCells) {
        QVariantList polygonList;
        for (const QPointF &point : cell) {
            QVariantMap pointMap;
            pointMap["x"] = point.x();
            pointMap["y"] = point.y();
            polygonList.append(pointMap);
        }
        QVariantMap cellMap;
        cellMap["points"] = polygonList;
        cellMap["area"] = computePolygonArea(cell);
        cellsList.append(cellMap);
    }
    return cellsList;
}

QVariantList Decomposer::orientedRect() const {
    QVariantList list;
    for (const QPointF &point : m_orientedRect) {
        QVariantMap pointMap;
        pointMap["x"] = point.x();
        pointMap["y"] = point.y();
        list.append(pointMap);
    }
    return list;
}

double Decomposer::sweepAngle() const {
    return m_sweepAngle;
}

bool Decomposer::showDecomposition() const {
    return m_showDecomposition;
}

bool Decomposer::showOrientedRect() const {
    return m_showOrientedRect;
}

void Decomposer::setOriginalPolygon(const QVariantList &polygon) {
    m_originalPolygon.clear();
    for (const QVariant &pointVar : polygon) {
        QVariantMap pointMap = pointVar.toMap();
        if (pointMap.contains("x") && pointMap.contains("y")) {
            m_originalPolygon << QPointF(pointMap["x"].toDouble(),
                                         pointMap["y"].toDouble());
        }
    }
    emit originalPolygonChanged();
    updateDecomposition();
}

void Decomposer::setSweepAngle(double angle) {
    if (qFuzzyCompare(m_sweepAngle, angle))
        return;

    m_sweepAngle = angle;
    emit sweepAngleChanged();
    updateDecomposition();
}

void Decomposer::setShowDecomposition(bool show) {
    if (m_showDecomposition == show)
        return;

    m_showDecomposition = show;
    emit showDecompositionChanged();
}

void Decomposer::setShowOrientedRect(bool show) {
    if (m_showOrientedRect == show)
        return;

    m_showOrientedRect = show;
    emit showOrientedRectChanged();
}

void Decomposer::updateDecomposition() {
    // Очищаем предыдущие результаты
    m_decompositionCells.clear();
    m_orientedRect.clear();

    if (m_originalPolygon.size() < 3) {
        emit decompositionCellsChanged();
        emit orientedRectChanged();
        return;
    }

    // Вычисляем ориентированный ограничивающий прямоугольник
    m_orientedRect = getOrientedBoundingRect(m_originalPolygon, m_sweepAngle);

    // Выполняем трапецоидальную декомпозицию
    std::vector<QPolygonF> cells = trapezoidalDecomposition(m_originalPolygon, m_sweepAngle);
    m_decompositionCells = QVector<QPolygonF>(cells.begin(), cells.end());

    emit decompositionCellsChanged();
    emit orientedRectChanged();
}

void Decomposer::resetPolygon() {
    createDefaultPolygon();
    emit originalPolygonChanged();
    updateDecomposition();
}

// Алгоритм трапецоидальной декомпозиции
std::vector<QPolygonF> Decomposer::trapezoidalDecomposition(const QPolygonF& polygon, double sweepAngle) {
    std::vector<QPolygonF> cells;

    if (polygon.size() < 3) {
        return cells;
    }

    // 1. Поворачиваем полигон
    QPolygonF rotatedPolygon;
    for (const QPointF& point : polygon) {
        rotatedPolygon.append(rotatePoint(point, -sweepAngle));
    }

    // 2. Находим все уникальные Y-координаты вершин
    QList<qreal> yLevels;
    for (const QPointF& point : rotatedPolygon) {
        if (!yLevels.contains(point.y())) {
            yLevels.append(point.y());
        }
    }
    std::sort(yLevels.begin(), yLevels.end());

    // 3. Для каждого уровня Y находим пересечения с ребрами
    for (int i = 0; i < yLevels.size() - 1; ++i) {
        double y1 = yLevels[i];
        double y2 = yLevels[i + 1];

        if (std::abs(y2 - y1) < 1e-9) {
            continue;
        }

        // Находим все пересечения с горизонтальной линией на середине уровня
        double yMid = (y1 + y2) / 2;
        QList<qreal> intersections;

        for (int j = 0; j < rotatedPolygon.size(); ++j) {
            int k = (j + 1) % rotatedPolygon.size();
            QPointF p1 = rotatedPolygon[j];
            QPointF p2 = rotatedPolygon[k];

            // Проверяем пересечение
            if ((p1.y() <= yMid && p2.y() >= yMid) ||
                (p1.y() >= yMid && p2.y() <= yMid)) {

                if (std::abs(p2.y() - p1.y()) < 1e-9) {
                    intersections.append(p1.x());
                    intersections.append(p2.x());
                } else {
                    double t = (yMid - p1.y()) / (p2.y() - p1.y());
                    double x = p1.x() + t * (p2.x() - p1.x());
                    intersections.append(x);
                }
            }
        }

        // Сортируем пересечения
        std::sort(intersections.begin(), intersections.end());

        // Создаем трапеции между пересечениями
        for (int j = 0; j < intersections.size() - 1; j += 2) {
            double x1 = intersections[j];
            double x2 = intersections[j + 1];

            if (x2 - x1 < 1e-9) {
                continue;
            }

            // Создаем трапецию
            QPolygonF trapezoid;

            // Верхние точки
            trapezoid << QPointF(x1, y1) << QPointF(x2, y1);

            // Находим нижние точки (на уровне y2)
            // Для простоты используем линейную интерполяцию
            trapezoid << QPointF(x2, y2) << QPointF(x1, y2);

            // Поворачиваем обратно
            QPolygonF rotatedTrapezoid;
            for (const QPointF& point : trapezoid) {
                rotatedTrapezoid.append(inverseRotatePoint(point, -sweepAngle));
            }

            cells.push_back(rotatedTrapezoid);
        }
    }

    return cells;
}

// Ориентированный ограничивающий прямоугольник
QPolygonF Decomposer::getOrientedBoundingRect(const QPolygonF& polygon, double angleDegrees) {
    if (polygon.size() < 3) {
        return QPolygonF();
    }

    // Вычисляем центр полигона
    QPointF centroid(0, 0);
    for (const QPointF& p : polygon) {
        centroid += p;
    }
    centroid /= polygon.size();

    // Поворачиваем полигон на -angle (выравниваем)
    qreal minX = std::numeric_limits<qreal>::max();
    qreal maxX = -std::numeric_limits<qreal>::max();
    qreal minY = std::numeric_limits<qreal>::max();
    qreal maxY = -std::numeric_limits<qreal>::max();

    QPolygonF polygonR;
    double xAligned, yAligned;
    for (const QPointF& p : polygon) {
        auto pR = rotatePoint(QPointF(p.x() - centroid.x(),p.y() - centroid.y()),
                              angleDegrees);
        polygonR << pR;
        xAligned = pR.x() + centroid.x();
        yAligned = pR.y() + centroid.y();

        minX = xAligned < minX ? xAligned : minX;
        maxX = xAligned > maxX ? xAligned : maxX;
        minY = yAligned < minY ? yAligned : minY;
        maxY = yAligned > maxY ? yAligned : maxY;
    }

    // Углы выровненного прямоугольника
    QPointF corners[4] = {
        QPointF(minX, minY),
        QPointF(maxX, minY),
        QPointF(maxX, maxY),
        QPointF(minX, maxY)
    };

    // Поворачиваем углы обратно на angle
    QPolygonF orientedRect;
    for (auto currCorn : corners)
    {
        auto pR = inverseRotatePoint(QPointF(currCorn.x() - centroid.x(), currCorn.y() - centroid.y()),
                                        angleDegrees);

        orientedRect << QPointF(pR.x() + centroid.x(), pR.y() + centroid.y());
    }

    return orientedRect;
}

// Утилиты
QPointF Decomposer::rotatePoint(const QPointF& point, double angle) {
    double rad = angle * M_PI / 180.0;
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);

    return QPointF{point.x() * cosA - point.y() * sinA,
        point.x() * sinA + point.y() * cosA};
}

QPointF Decomposer::inverseRotatePoint(const QPointF& point, double angle) {
    return rotatePoint(point, -angle);
}

double Decomposer::computePolygonArea(const QPolygonF& polygon) const{
    if (polygon.size() < 3) return 0;

    double area = 0;
    int n = polygon.size();

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        area += polygon[i].x() * polygon[j].y();
        area -= polygon[j].x() * polygon[i].y();
    }

    return std::abs(area) / 2.0;
}

// Предопределенные полигоны
void Decomposer::createDefaultPolygon() {
    m_originalPolygon.clear();
    // Шестиугольник
    m_originalPolygon << QPointF(200, 100)
                      << QPointF(400, 100)
                      << QPointF(500, 250)
                      << QPointF(400, 400)
                      << QPointF(200, 400)
                      << QPointF(100, 250);
}