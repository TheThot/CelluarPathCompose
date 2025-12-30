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

struct holesInfoIn{
    QHash<const QPolygonF*, QPair<QList<QPointF>,QList<QPointF>>>  holeBorderSegm;
    QHash<const QPolygonF*, QPair<QPolygonF*,QPolygonF*>>          holeToBCD;  // BCD levels только для holes
};

namespace baseFunc {

    static QList<QPolygonF> simpleSubtracted(const QPolygonF &poly1, const QPolygonF &poly2)
    {
        // Создаем QPainterPath для обоих полигонов
        QPainterPath path1;
        path1.addPolygon(poly1);

        QPainterPath path2;
        path2.addPolygon(poly2);

        // Вычитаем
        QPainterPath subtracted = path1.subtracted(path2);

        // Получаем все подконтуры
        QList<QPolygonF> polygons;

        // Проходим по всем элементам пути
        for (int i = 0; i < subtracted.elementCount(); ) {
            QPolygonF poly;

            // Начинаем новый подконтур
            const QPainterPath::Element &elem = subtracted.elementAt(i);
            if (elem.type == QPainterPath::MoveToElement) {
                poly << QPointF(elem.x, elem.y);
                i++;

                // Добавляем все последующие LineTo/CurveTo элементы
                while (i < subtracted.elementCount()) {
                    const QPainterPath::Element &nextElem = subtracted.elementAt(i);
                    if (nextElem.type == QPainterPath::MoveToElement) {
                        break;  // Начался новый подконтур
                    }
                    poly << QPointF(nextElem.x, nextElem.y);
                    i++;
                }

                // Если полигон не пустой, добавляем его
                if (poly.size() > 2) {
                    // Проверяем, замкнут ли полигон
                    if (poly.first() != poly.last()) {
                        poly << poly.first();
                    }
                    polygons.append(poly);
                }
            } else {
                i++;
            }
        }

        return polygons;
    }

    static double lineEquationKoeff(const QPointF& p1, const QPointF& p2){
        double res;
        res = (p2.x() - p1.x()) / (p2.y() + p1.y());
        return res;
    }

    //проверка на направление вращения полигона true - clockwise; false - counter-clockwise
    static bool isPolyInClockwiseMannerRot(const QPointF* polygonPoints, uint size){
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

    static int _updateCellRule(const QList<QPolygonF>* _holes, const QList<QPolygonF>* _bpd_decompositionCells){
        int rule = 0;

        if(_holes == nullptr){
            return rule;
        }

        // первые holes.count * 2 _bpd_decompositionCells это U и D каждой Cell

        QList<QPolygonF> check;
        for(int i = 0; i < _holes->count() * 2; ++i){
            check.append(_bpd_decompositionCells->at(i));
        }
        QList<QPolygonF> check2;
        for(int i = 0; i < _holes->count() * 2; i+=2){
            check2.append(check[i].united(check[i+1]));
        }
        QList<double> squareHolesSection;
        for(int i = 0; i < check2.count(); ++i) {
            squareHolesSection.append(check2[i].boundingRect().height()*check2[i].boundingRect().width());
        }
        double sumSq = 0;
        double min = 1e10;
        for(int i = 0; i < check2.count(); ++i) {
            if(min > squareHolesSection[i])
                min = squareHolesSection[i];
            sumSq += squareHolesSection[i];
        }
        QPolygonF whole;
        for(int i = 0; i < check2.count()-1; ++i) {
            if(i == 0)
                whole = check2[i].intersected(check2[i+1]);
            else
                whole = whole.intersected(check2[i+1]);
        }
        double squareWhole = whole.boundingRect().width()*whole.boundingRect().height();

        rule = (squareWhole == 0) ? 0 : (squareWhole < min) ? 1 : 2; // squareWhole == min входит в 2

        return rule;
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

    /*static void intersectionListFormimgRoutine2(const QLineF& l1, const QLineF& l2, QList<QPointF>& intersections_list, QLineF::IntersectionType foundingType)
    {
        // на случай QLineF::UnboundedIntersection ограничиваем возможные пересечения за пределами области определения линий самым большим размером = max(SurvPolyBoundRect)
        QPointF resIntersection{0,0};
        QLineF::IntersectionType flag = l1.intersects(l2, &resIntersection);


        if (flag == foundingType)
            if (!intersections_list.contains(resIntersection))
                intersections_list.append(resIntersection);
    }*/

    static bool intersectionListFormimgRoutine(const QLineF& l1, const QLineF& l2, QList<QPointF>& intersections_list, int foundingType,
                                        uint64_t maxBoundSurvPolyRad = 1e3)
    {
        // на случай QLineF::UnboundedIntersection ограничиваем возможные пересечения за пределами области определения линий самым большим размером = max(SurvPolyBoundRect)
        QPointF resIntersection{0,0};
        int flag = l1.intersect(l2, &resIntersection);

        if(flag != 0) // берём l2 в условие потому что это линия должна быть со структуры полигона with holes
            if(static_cast<uint64_t>(QLineF(l2.p1(), resIntersection).length()) > maxBoundSurvPolyRad)
                return false;

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

    //проверка лежит ли точка на прямой
    static bool isPointOnLineF(const QPointF& p, const QLineF& line){
        double len = line.length(); //исходная длина прямой
        QLineF lineP1(line.p1(), p), lineP2(p, line.p2());
        double len1, len2;
        len1 = lineP1.length();
        len2 = lineP2.length();
        double sum = len1 + len2;
        bool isEq = std::abs(len - sum) < 0.1;
        //в случае если сумма len1 + len2 == len значит точка на прямой
        return isEq;
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
        qreal angle = line.angle() * M_PI / 180.0; // Конвертируем в радианы

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

}
#endif //TRYCELLUARPATHCOMPOSE_UTILS_H
