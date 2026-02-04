//
// Created by Admin on 19.12.2025.
//

#ifndef TRYCELLUARPATHCOMPOSE_UTILS_H
#define TRYCELLUARPATHCOMPOSE_UTILS_H

#include <cmath>
#include <QPointF>
#include <QPolygonF>
#include <QtCore/QLineF>
#include <QHash>
#include <QList>
#include <QPair>
#include <QPainterPath>
#include <iostream>
#include "PolyBuilder.h"

#ifdef __clang__
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#pragma clang diagnostic ignored "-Wunused"
#endif

namespace baseFunc {

    // Вспомогательная функция для вычисления площади
    static double polygonArea(const QPolygonF &poly) {
        if (poly.size() < 3) return 0.0;

        double area = 0.0;
        int n = poly.size();

        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            area += poly[i].x() * poly[j].y();
            area -= poly[j].x() * poly[i].y();
        }

        return fabs(area) / 2.0;
    }

    //проверка лежит ли точка на прямой
    static bool isPointOnLineF(const QPointF& p, const QLineF& line){
        double len = line.length(); //исходная длина прямой
        QLineF lineP1(line.p1(), p), lineP2(p, line.p2());
        double len1, len2;
        len1 = lineP1.length();
        len2 = lineP2.length();
        double sum = len1 + len2;
        bool isEq = std::abs(len - sum) < 1e-1;
        //в случае если сумма len1 + len2 == len значит точка на прямой
        return isEq;
    }

    /**
     * @brief добавляет промежуточные точки полигона @param polyRep между двумя точками линий @param betweenLines
     * @param polyRep Линии полигона
     * @param betweenLine точки между которыми будет вестись поиск
     * result @param orderExtraPolyline добавляем в новый массив всё вместе
     */
    static bool extraDangerPointsRoutine(const QList<QLineF>& polyRep, const QLineF& betweenLine,
                                          QList<QPointF>& orderExtraPolyline) {
        // находим на каких линиях полигона polyRep inner hole две наших пары точек
        int firstLidx = -1, secondLidx = -1;
        orderExtraPolyline.clear();
        orderExtraPolyline.append(betweenLine.p1());

        // Ищем индексы линий, содержащих точки
        for(int j = 0; j < polyRep.size(); ++j) {
            if(isPointOnLineF(betweenLine.p1(), polyRep[j]))
                firstLidx = j;
            if(isPointOnLineF(betweenLine.p2(), polyRep[j]))
                secondLidx = j;
        }

        if(firstLidx == -1 || secondLidx == -1)
            return false;

        // Если точки на одной линии, промежуточных точек нет
        if(firstLidx == secondLidx) {
            // Ничего не добавляем
            return true;
        }

        // Проверяем, являются ли линии соседними с учетом замкнутости
        int size = polyRep.size();
        bool areAdjacent = false;

        // Проверка соседности
        if(std::abs(firstLidx - secondLidx) == 1 || (firstLidx == 0 && secondLidx == size-1) ||
                (firstLidx == size-1 && secondLidx == 0))
            areAdjacent = true;

        if(areAdjacent) {
            // Для соседних линий добавляем только точку соединения между ними
            // Нужно определить правильную точку соединения
            if((firstLidx + 1) % size == secondLidx) {
                // firstLidx предшествует secondLidx
                orderExtraPolyline.append(polyRep[firstLidx].p2());
            } else {
                // secondLidx предшествует firstLidx
                // Добавляем точки в обратном порядке
                orderExtraPolyline.append(polyRep[secondLidx].p2());
            }
            return true;
        }

        // Для линий собираем все промежуточные вершины
        // Определяем направление обхода
        int currentIdx = firstLidx;
        int endIdx = secondLidx;

        // Проверяем какое направление короче
        int forwardDistance = (endIdx > currentIdx) ?
                              (endIdx - currentIdx) :
                              (size - currentIdx + endIdx);
        int backwardDistance = (currentIdx > endIdx) ?
                               (currentIdx - endIdx) :
                               (size - endIdx + currentIdx);

        // Выбираем направление с меньшим расстоянием
        bool goForward = forwardDistance <= backwardDistance;

        if(goForward) {
            // Обход вперед
            while(true) {
                currentIdx = (currentIdx + 1) % size;
                if(currentIdx == endIdx) break;

                // Добавляем начальную точку текущей линии
                orderExtraPolyline.append(polyRep[currentIdx].p1());
            }
        } else {
            // Обход назад
            while(true) {
                // Добавляем конечную точку предыдущей линии
                orderExtraPolyline.append(polyRep[currentIdx].p1());

                currentIdx = (currentIdx - 1 + size) % size;
                if(currentIdx == endIdx) break;
            }
        }

        return true;
    }

    static double lineEquationKoeff(const QPointF& p1, const QPointF& p2){
        double res;
        res = (p2.x() - p1.x()) / (p2.y() + p1.y());
        return res;
    }

    //проверка на направление вращения полигона true - clockwise; false - counter-clockwise
    static bool isPolyInClockwiseMannerRot(const QPointF* polygonPoints, int size){
        if (size < 3) {
            // Для полигона нужно минимум 3 точки
            return false;
        }

        double sum = 0;
        for (int i = 0; i < size; i++){
            const QPointF& p1 = polygonPoints[i];
            const QPointF& p2 = polygonPoints[(i + 1) % size];
            sum += lineEquationKoeff(p1, p2);
        }
        bool res = sum > 0;
        return res;
    }

    static int updateCellRule(const QList<QPolygonF>* _holes, const QList<QPolygonF>* _bpd_decompositionCells){
        int rule = 0;

        if(_holes == nullptr){
            return rule;
        }

        // первые holes.count * 2 _bpd_decompositionCells это U и D каждой Cell

        QList<QPolygonF> check;
        for(int i = 0; i < _holes->count() * 2; ++i){
            check.append(_bpd_decompositionCells->at(i));
        }
        QList<QRectF> check2;
        for(int i = 0; i < _holes->count() * 2; i+=2){
            check2.append(check[i].boundingRect().united(check[i+1].boundingRect()));
        }
        QList<double> squareHolesSection;
        for(int i = 0; i < check2.count(); ++i) {
            squareHolesSection.append(polygonArea(check2[i]));
        }
        double sumSq = 0;
        double min = 1e10;
        for(int i = 0; i < check2.count(); ++i) {
            if(min > polygonArea(check2[i]))
                min = polygonArea(check2[i]);
            sumSq += squareHolesSection[i];
        }
        QPolygonF whole;
        PolyBuilder pb = PolyBuilder();
        for(int i = 0; i < check2.count()-1; ++i) {
            if(i == 0)
                whole = pb.snglIntersctnWrp(check2[i], check2[i+1]);
        }
        double squareWhole = polygonArea(whole);

//        std::cout << "utilz [updateCellRule] squareWhole is " << squareWhole << " sumSq is " << sumSq << " min is " << min << std::endl;

        rule = (squareWhole == 0) ? 0 : (std::abs(squareWhole - min) < 1e-1) ? 2 : 1; // squareWhole == min входит в 2

        return rule;
    }


    static void intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines, const QPolygonF& bounds, const QList<QPolygonF>* _holes)
    {
        resultLines.clear();

        for (const QLineF& line : lineList) {
            QList<QPointF> intersections;

            // Intersect the line with all the polygon edges
            for (int i=0; i<polygon.count(); ++i) {
                QLineF polygonEdge(polygon[i], polygon[(i+1) % polygon.count()]);
                QPointF intersectPoint;

                if (line.intersect(polygonEdge, &intersectPoint) == QLineF::BoundedIntersection) {
                    if (!intersections.contains(intersectPoint) && bounds.containsPoint(intersectPoint, Qt::WindingFill))
                        intersections.append(intersectPoint);
                }
            }

            for(const auto& currHole:*_holes){
                for (int i=0; i<currHole.count(); ++i) {
                    QLineF polygonEdge(currHole[i], currHole[(i+1) % currHole.count()]);
                    QPointF intersectPoint;

                    if (line.intersect(polygonEdge, &intersectPoint) == QLineF::BoundedIntersection) {
                        if (!intersections.contains(intersectPoint) && bounds.containsPoint(intersectPoint, Qt::WindingFill))
                            intersections.append(intersectPoint);
                    }
                }
            }

            std::sort(intersections.begin(), intersections.end(), [&line](const QPointF& pointStart, const QPointF& pointEnd) {
                return QLineF(line.p1(), pointStart).length() < QLineF(line.p1(), pointEnd).length();
            });

            for (int j=0; j+1<intersections.count(); j+=2) {
                auto temp = QLineF(intersections[j], intersections[j + 1]);
                resultLines.append(temp);
            }
        }

        if (!resultLines.isEmpty()) {
            QList<QLineF> linesList;
            linesList.append(resultLines.first());
            resultLines.removeFirst();

            while (!resultLines.isEmpty()) {
                QLineF& tempLine = linesList.last();
                QPointF tempPoint = tempLine.p2();

                int indexNear = -1;
                double minDistance = std::numeric_limits<double>::max();
                bool reverse = false;

                for (int i = 0; i < resultLines.count(); ++i) {
                    double distanceP1 = QLineF(tempPoint, resultLines[i].p1()).length();
                    double distanceP2 = QLineF(tempPoint, resultLines[i].p2()).length();

                    if (distanceP1 < minDistance) {
                        minDistance = distanceP1;
                        indexNear = i;
                        reverse = false;
                    }
                    if (distanceP2 < minDistance) {
                        minDistance = distanceP2;
                        indexNear = i;
                        reverse = true;
                    }
                }

                if (indexNear != -1) {
                    QLineF nextLine = resultLines.takeAt(indexNear);
                    if (reverse) {
                        nextLine = QLineF(nextLine.p2(), nextLine.p1());
                    }
                    linesList.append(nextLine);
                }
                else
                    break;
            }
            resultLines = linesList;
        }
    }

    static void intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines)
    {
        resultLines.clear();

        for (const QLineF& line : lineList) {
            QList<QPointF> intersections;

            // Intersect the line with all the polygon edges
            for (int i=0; i<polygon.count(); ++i) {
                QLineF polygonEdge(polygon[i], polygon[(i+1) % polygon.count()]);
                QPointF intersectPoint;

                if (line.intersect(polygonEdge, &intersectPoint) == QLineF::BoundedIntersection) {
                    if (!intersections.contains(intersectPoint))
                        intersections.append(intersectPoint);
                }
            }

            std::sort(intersections.begin(), intersections.end(), [&line](const QPointF& pointStart, const QPointF& pointEnd) {
                return QLineF(line.p1(), pointStart).length() < QLineF(line.p1(), pointEnd).length();
            });

            for (int j=0; j+1<intersections.count(); j+=2)
                resultLines.append(QLineF(intersections[j], intersections[j+1]));
        }

        if (!resultLines.isEmpty()) {
            QList<QLineF> linesList;
            linesList.append(resultLines.first());
            resultLines.removeFirst();

            while (!resultLines.isEmpty()) {
                QLineF& tempLine = linesList.last();
                QPointF tempPoint = tempLine.p2();

                int indexNear = -1;
                double minDistance = std::numeric_limits<double>::max();
                bool reverse = false;

                for (int i = 0; i < resultLines.count(); ++i) {
                    double distanceP1 = QLineF(tempPoint, resultLines[i].p1()).length();
                    double distanceP2 = QLineF(tempPoint, resultLines[i].p2()).length();

                    if (distanceP1 < minDistance) {
                        minDistance = distanceP1;
                        indexNear = i;
                        reverse = false;
                    }
                    if (distanceP2 < minDistance) {
                        minDistance = distanceP2;
                        indexNear = i;
                        reverse = true;
                    }
                }

                if (indexNear != -1) {
                    QLineF nextLine = resultLines.takeAt(indexNear);
                    if (reverse) {
                        nextLine = QLineF(nextLine.p2(), nextLine.p1());
                    }
                    linesList.append(nextLine);
                }
                else
                    break;
            }
            resultLines = linesList;
        }
    }

    static QPointF rotatePoint(const QPointF& point, const QPointF& origin, double angle)
    {
        QPointF rotated;

        double radians = (M_PI / 180.0) * -angle;

        rotated.setX(((point.x() - origin.x()) * cos(radians)) - ((point.y() - origin.y()) * sin(radians)) + origin.x());
        rotated.setY(((point.x() - origin.x()) * sin(radians)) + ((point.y() - origin.y()) * cos(radians)) + origin.y());

        return rotated;
    }

    static bool intersectionListFormimgRoutine(const QLineF& l1, const QLineF& l2, QList<QPointF>& intersections_list, int foundingType)
    {
        // на случай QLineF::UnboundedIntersection ограничиваем возможные пересечения за пределами области определения линий самым большим размером = max(SurvPolyBoundRect)
        QPointF resIntersection{0,0};
        int flag = l1.intersect(l2, &resIntersection);

        if (flag == foundingType)
            if (!intersections_list.contains(resIntersection))
                intersections_list.append(resIntersection);

        return flag == foundingType;
    }


    template <typename T>
    static std::vector<int> sort_indexes(const QList<T> &v) {

        // initialize original index locations
        std::vector<int> idx(v.size());
        std::iota(idx.begin(), idx.end(), 0);

        std::sort(idx.begin(), idx.end(),
                  [&v](int i1, int i2) {return v[i1] < v[i2];});

        return idx;
    }

    static QPolygonF sortPolygonClockwise(QPolygonF polygon) {
        if (polygon.size() < 3) return polygon;

        // Находим самую нижнюю-левую точку (опорную точку)
        QPointF pivot = polygon[0];
        for (const QPointF& p : polygon) {
            if (p.y() < pivot.y() ||
                (std::abs(p.y() - pivot.y()) < 1e-9 && p.x() < pivot.x())) {
                pivot = p;
                }
        }

        // Сортируем по полярному углу относительно опорной точки
        std::sort(polygon.begin(), polygon.end(),
                  [pivot](const QPointF& a, const QPointF& b) {
                      if (a == pivot) return true;
                      if (b == pivot) return false;

                      double angleA = std::atan2(a.y() - pivot.y(), a.x() - pivot.x());
                      double angleB = std::atan2(b.y() - pivot.y(), b.x() - pivot.x());

                      if (std::abs(angleA - angleB) < 1e-9) {
                          // Для коллинеарных точек сортируем по расстоянию
                          double distA = std::hypot(a.x() - pivot.x(), a.y() - pivot.y());
                          double distB = std::hypot(b.x() - pivot.x(), b.y() - pivot.y());
                          return distA < distB;
                      }

                      return angleA < angleB;
                  });

        return polygon;
    }


    static QPolygonF sortPolygonCounterClockwise(QPolygonF polygon) {
        // Сначала сортируем по часовой стрелке
        polygon = sortPolygonClockwise(polygon);
        // Затем реверсируем (исключая опорную точку)
        std::reverse(polygon.begin() + 1, polygon.end());
        return polygon;
    }

    /**
     * @brief Пролонгирует линию в обе стороны на заданное расстояние
     * @param line Исходная линия
     * @param delta Расстояние для удлинения в каждую сторону
     * @return Пролонгированная линия
     */
    static QLineF extendLineBothWays(const QLineF& line, qreal delta)
    {
        // Получаем начальную и конечную точки линии
        QPointF p1 = line.p1();
        QPointF p2 = line.p2();

        // Вычисляем длину и угол линии
        qreal length = line.length();

        if (qFuzzyCompare(length, 0)) {
            // Если линия является точкой, возвращаем её без изменений
            return line;
        }

        // Вычисляем коэффициенты для направления линии
        qreal dx = (p2.x() - p1.x()) / length;
        qreal dy = (p2.y() - p1.y()) / length;

        // Удлиняем линию в обе стороны
        QPointF newP1 = p1 - QPointF(dx * delta, dy * delta);
        QPointF newP2 = p2 + QPointF(dx * delta, dy * delta);

        return QLineF{newP1, newP2};
    }

    static QList<double> distanceToPointRoutine(const QPointF& point, const QList<QPointF>& intersections_list)
    {
        QList<double> result;
        for(int i = 0; i < intersections_list.count(); ++i)
            result.append(QLineF(point, intersections_list[i]).length());
        return result;
    }

    static QLineF findLineBetweenLines(const QLineF& parall1, const QLineF& parall2, const QPointF& coord)
    {

        // Получаем нормальный единичный вектор
        QLineF normal = parall1.normalVector();
        QPointF unitNormal = normal.p2() - normal.p1();

        // Создаем линию через точку coord в перпендикулярном направлении
        qreal length = 1e3;

        auto approxToPrecise = QLineF{coord - unitNormal * length, coord + unitNormal * length};
        QPointF preciseP;
        parall1.intersect(approxToPrecise, &preciseP);
        approxToPrecise.setP1(preciseP);
        parall2.intersect(approxToPrecise, &preciseP);
        approxToPrecise.setP2(preciseP);
        return approxToPrecise;

    }

    // Равномерная выборка точек с фиксированным шагом
    static QList<QPointF> uniformSample(const QList<QPointF>& points,
                                        int targetCount) {
        if (points.size() <= targetCount) return points;

        QList<QPointF> result;
        double step = static_cast<double>(points.size() - 1) / (targetCount - 1);

        for (int i = 0; i < targetCount; i++) {
            double index = i * step;
            int idx1 = static_cast<int>(index);
            double fraction = index - idx1;

            if (idx1 == points.size() - 1) {
                result.append(points.last());
            } else {
                // Линейная интерполяция между точками
                QPointF p1 = points[idx1];
                QPointF p2 = points[idx1 + 1];

                QPointF interpolated(
                        p1.x() + fraction * (p2.x() - p1.x()),
                        p1.y() + fraction * (p2.y() - p1.y())
                );
                result.append(interpolated);
            }
        }

        return result;
    }

    // Адаптивная выборка: сохраняем точки с большим изменением направления
    static QList<QPointF> adaptiveSample(const QList<QPointF>& points,
                                         double angleThreshold = 15.0,
                                         int minPoints = 10) {
        if (points.size() < 3) return points;

        QList<QPointF> result;
        result.append(points.first());

        // Всегда сохраняем первую и последнюю точки
        for (int i = 1; i < points.size() - 1; i++) {
            QPointF prev = points[i-1];
            QPointF curr = points[i];
            QPointF next = points[i+1];

            // Вычисляем угол между векторами
            QPointF v1(curr.x() - prev.x(), curr.y() - prev.y());
            QPointF v2(next.x() - curr.x(), next.y() - curr.y());

            double dot = v1.x() * v2.x() + v1.y() * v2.y();
            double mag1 = std::sqrt(v1.x() * v1.x() + v1.y() * v1.y());
            double mag2 = std::sqrt(v2.x() * v2.x() + v2.y() * v2.y());

            if (mag1 > 0 && mag2 > 0) {
                double cosAngle = dot / (mag1 * mag2);
                cosAngle = qBound(-1.0, cosAngle, 1.0);
                double angle = std::acos(cosAngle) * 180.0 / M_PI;

                if (angle > angleThreshold) {
                    // Резкое изменение направления - сохраняем точку
                    result.append(curr);
                }
            }

            // Гарантируем минимальное количество точек
            // Исправление: проверяем, что деление возможно
            if (result.size() < minPoints && minPoints > 0) {
                // Равномерно распределяем точки
                int step = qMax(1, points.size() / minPoints);
                if (i % step == 0) {
                    result.append(curr);
                }
            }
        }

        result.append(points.last());

        // Дополнительная гарантия минимального количества точек
        if (result.size() < minPoints && minPoints <= points.size()) {
            // Просто равномерно выбираем точки
            result.clear();
            result.append(points.first());
            int step = qMax(1, (points.size() - 2) / (minPoints - 2));
            for (int i = 1; i < points.size() - 1; i++) {
                if (i % step == 0 || result.size() < minPoints) {
                    result.append(points[i]);
                }
            }
            result.append(points.last());
        }

        return result;
    }


    static bool polygonsAreEqualWithinDelta(const QPolygonF& poly1, const QPolygonF& poly2, double delta)
    {
        // Проверяем количество точек
        if (poly1.size() != poly2.size()) {
            return false;
        }

        // Проверяем каждую точку в пределах погрешности
        for (int i = 0; i < poly1.size(); ++i) {
            double dx = poly1[i].x() - poly2[i].x();
            double dy = poly1[i].y() - poly2[i].y();
            double distanceSquared = dx * dx + dy * dy;

            if (distanceSquared > delta * delta) {
                return false;
            }
        }

        return true;
    }

    static QList<QPolygonF> removeDuplicatePolygons(QList<QPolygonF>& polygons, double delta = 10)
    {
        QList<QPolygonF> result;

        for (int i = 0; i < polygons.size(); ++i) {
            bool isDuplicate = false;

            // Проверяем, есть ли уже такой полигон в результате
            for (int j = 0; j < result.size(); ++j) {
                if (polygonsAreEqualWithinDelta(polygons[i], result[j], delta)) {
                    isDuplicate = true;
                    break;
                }
            }

            // Если не дубликат, добавляем в результат
            if (!isDuplicate) {
                result.append(polygons[i]);
            }
        }

        return result;
    }

}
#endif //TRYCELLUARPATHCOMPOSE_UTILS_H
