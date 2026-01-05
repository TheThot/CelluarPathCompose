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

struct holesInfoIn{
    QHash<const QPolygonF*, QPair<QList<QPointF>,QList<QPointF>>>  holeBorderSegm;
    QHash<const QPolygonF*, QPair<QPolygonF*,QPolygonF*>>          holeToBCD;  // BCD levels только для holes
};

namespace baseFunc {

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

    static bool segmentsIntersect(const QPointF &p1, const QPointF &p2,
                                         const QPointF &q1, const QPointF &q2,
                                         QPointF &intersection, double &t, double &u)
    {
        QPointF r = p2 - p1;
        QPointF s = q2 - q1;
        QPointF qp = q1 - p1;

        auto crossProduct = [](const QPointF &p1, const QPointF &p2, const QPointF &p3)
        {
            return (p2.x() - p1.x()) * (p3.y() - p1.y()) -
                   (p2.y() - p1.y()) * (p3.x() - p1.x());
        };

        double rxs = crossProduct(QPointF(0, 0), r, s);
        double qpxr = crossProduct(QPointF(0, 0), qp, r);

        if (fabs(rxs) < 1e-10) {
            return false; // Коллинеарны или параллельны
        }

        t = crossProduct(QPointF(0, 0), qp, s) / rxs;
        u = crossProduct(QPointF(0, 0), qp, r) / rxs;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            intersection = p1 + r * t;
            return true;
        }

        return false;
    }

    static QList<QPolygonF> subtractPolygonPrecise(const QPolygonF &subject, const QPolygonF &clip)
    {
        QList<QPolygonF> result;

        if (subject.isEmpty()) return result;
        if (clip.isEmpty()) {
            result.append(subject);
            return result;
        }

        // Создаем путь с операцией вычитания
        QPainterPath subjectPath;
        subjectPath.addPolygon(subject);

        QPainterPath clipPath;
        clipPath.addPolygon(clip);

        QPainterPath subtractedPath = subjectPath.subtracted(clipPath);

        // ОШИБКА 1: toFillPolygon() может возвращать пустой полигон
        // Используем toSubpathPolygons() для получения всех подпутей
        QList<QPolygonF> allPolygons = subtractedPath.toSubpathPolygons();

        // ОШИБКА 2: Проверяем, есть ли что-то в результате
        if (allPolygons.isEmpty()) {
            // Если нет полигонов, возможно результат пустой
            return result;
        }

        // ОШИБКА 3: Алгоритм разделения был слишком сложным и неправильным
        // Упрощаем: возвращаем все полигоны из toSubpathPolygons()
        for (const QPolygonF &poly : allPolygons) {
            if (poly.size() >= 3) {
                // Проверяем, замкнут ли полигон
                QPolygonF closedPoly = poly;
                if (closedPoly.first() != closedPoly.last()) {
                    closedPoly.append(closedPoly.first());
                }

                // Проверяем площадь
                double area = 0;
                for (int i = 0; i < closedPoly.size(); i++) {
                    int j = (i + 1) % closedPoly.size();
                    area += closedPoly[i].x() * closedPoly[j].y();
                    area -= closedPoly[j].x() * closedPoly[i].y();
                }
                area = fabs(area) / 2.0;

                if (area > 1e-6) { // Минимальная площадь
                    result.append(closedPoly);
                }
            }
        }

        // ОШИБКА 4: Если все еще пусто, возможно нужно альтернативное решение
        if (result.isEmpty()) {
            // Проверяем, может быть clip полностью внутри subject
            bool allInside = true;
            for (const QPointF &p : clip) {
                if (!subject.containsPoint(p, Qt::OddEvenFill)) {
                    allInside = false;
                    break;
                }
            }

            if (allInside) {
                // Clip внутри subject - создаем полигон с дырой
                QPainterPath pathWithHole;
                pathWithHole.addPolygon(subject);
                pathWithHole.addPolygon(clip);

                QList<QPolygonF> holePolygons = pathWithHole.toSubpathPolygons();
                for (const QPolygonF &poly : holePolygons) {
                    if (poly.size() >= 3) {
                        QPolygonF closedPoly = poly;
                        if (closedPoly.first() != closedPoly.last()) {
                            closedPoly.append(closedPoly.first());
                        }
                        result.append(closedPoly);
                    }
                }
            } else {
                // Возвращаем subject как есть
                result.append(subject);
            }
        }

        return result;
    }

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

    static QPolygonF simpleUnited(const QPolygonF &poly1, const QPolygonF &poly2) {

        // Создаем QPainterPath для обоих полигонов
        QPainterPath path1;
        path1.addPolygon(poly1);

        QPainterPath path2;
        path2.addPolygon(poly2);

        // Объединяем
        QPainterPath united = path1.united(path2);

        // Используем toSubpathPolygons() для получения всех контуров
        QList<QPolygonF> polygons = united.toSubpathPolygons();
        QList<QPolygonF> result;

        // Обрабатываем каждый полигон
        for (QPolygonF poly : polygons) {
            // Удаляем дубликаты точек
            QPolygonF cleanPoly;
            for (int i = 0; i < poly.size(); i++) {
                if (i == 0 || QLineF(poly[i-1], poly[i]).length() > 0.001) {
                    cleanPoly.append(poly[i]);
                }
            }

            if (cleanPoly.size() >= 3) {
                // Замыкаем полигон если нужно
                if (cleanPoly.first() != cleanPoly.last()) {
                    cleanPoly.append(cleanPoly.first());
                }

                // Вычисляем площадь для проверки корректности
                double area = 0;
                int n = cleanPoly.size();
                for (int i = 0; i < n; i++) {
                    int j = (i + 1) % n;
                    area += cleanPoly[i].x() * cleanPoly[j].y();
                    area -= cleanPoly[j].x() * cleanPoly[i].y();
                }
                area = fabs(area) / 2.0;

                // Добавляем если площадь достаточна
                if (area > 0.000001) {
                    result.append(cleanPoly);
                }
            }
        }

        // Если результат пустой, но хотя бы один полигон был не пустой
        if (result.isEmpty() ) {
            // Возвращаем наибольший из полигонов
            return QPolygonF();
        }
        return result[0];
    }

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


// Упрощение полигона (удаление дубликатов и коллинеарных точек)
    static QPolygonF simplifyPolygon(const QPolygonF &poly, double epsilon) {
        if (poly.size() < 3) return poly;

        QPolygonF result;
        result.append(poly[0]);

        for (int i = 1; i < poly.size(); i++) {
            QPointF prev = result.last();
            QPointF curr = poly[i];

            // Проверяем расстояние
            double dx = curr.x() - prev.x();
            double dy = curr.y() - prev.y();
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist > epsilon) {
                // Проверяем, не коллинеарна ли точка
                if (result.size() >= 2) {
                    QPointF prev2 = result[result.size() - 2];
                    double area = std::abs(
                            (prev2.x() - prev.x()) * (curr.y() - prev.y()) -
                            (prev2.y() - prev.y()) * (curr.x() - prev.x())
                    );

                    if (area > epsilon * epsilon) {
                        result.append(curr);
                    } else {
                        // Заменяем предыдущую точку
                        result.last() = curr;
                    }
                } else {
                    result.append(curr);
                }
            }
        }

        // Замыкаем если нужно
        if (result.size() > 2) {
            QPointF first = result.first();
            QPointF last = result.last();

            double dx = last.x() - first.x();
            double dy = last.y() - first.y();
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist > epsilon) {
                result.append(first);
            }
        }

        return result;
    }

// "Раздутие" полигона (смещение границ наружу)
    static QPolygonF inflatePolygon(const QPolygonF &poly, double amount) {
        if (poly.size() < 3) return poly;

        QPolygonF result;
        int n = poly.size();

        for (int i = 0; i < n; i++) {
            QPointF prev = poly[(i - 1 + n) % n];
            QPointF curr = poly[i];
            QPointF next = poly[(i + 1) % n];

            // Вычисляем нормали к ребрам
            QPointF inEdge = curr - prev;
            QPointF outEdge = next - curr;

            // Нормализуем
            double lenIn = std::sqrt(inEdge.x() * inEdge.x() + inEdge.y() * inEdge.y());
            double lenOut = std::sqrt(outEdge.x() * outEdge.x() + outEdge.y() * outEdge.y());

            if (lenIn > 0) inEdge /= lenIn;
            if (lenOut > 0) outEdge /= lenOut;

            // Поворачиваем на 90 градусов
            QPointF normalIn(-inEdge.y(), inEdge.x());
            QPointF normalOut(-outEdge.y(), outEdge.x());

            // Усредняем нормали
            QPointF normal((normalIn.x() + normalOut.x()) * 0.5,
                           (normalIn.y() + normalOut.y()) * 0.5);

            double len = std::sqrt(normal.x() * normal.x() + normal.y() * normal.y());
            if (len > 0) normal /= len;

            // Смещаем точку
            result.append(curr + normal * amount);
        }

        return result;
    }

// Извлечение полигона из пути с обратным масштабированием
    static QPolygonF extractPrecisePolygon(const QPainterPath &path) {
        const double precisionScale = 100000.0;
        const double epsilon = 1e-8;

        // Пробуем несколько методов извлечения
        QPolygonF result;

        // Метод 1: Проход по элементам
        QPolygonF fromElements;
        for (int i = 0; i < path.elementCount(); i++) {
            const QPainterPath::Element &elem = path.elementAt(i);
            fromElements.append(QPointF(elem.x / precisionScale, elem.y / precisionScale));
        }

        // Метод 2: Используем toFillPolygon
        QPolygonF fromFill = path.toFillPolygon();
        for (QPointF &p : fromFill) {
            p = QPointF(p.x() / precisionScale, p.y() / precisionScale);
        }

        // Выбираем лучший результат (с большей площадью)
        double area1 = polygonArea(fromElements);
        double area2 = polygonArea(fromFill);

        result = (area1 > area2) ? fromElements : fromFill;

        // Упрощаем результат
        result = simplifyPolygon(result, epsilon);

        return result;
    }

// Преобразование полигона в путь с повышенной точностью
    static QPainterPath polygonToHighPrecisionPath(const QPolygonF &poly) {
        QPainterPath path;
        if (poly.isEmpty()) return path;

        const double precisionScale = 100000.0; // Высокий масштаб для точности

        path.moveTo(poly[0].x() * precisionScale, poly[0].y() * precisionScale);
        for (int i = 1; i < poly.size(); i++) {
            path.lineTo(poly[i].x() * precisionScale, poly[i].y() * precisionScale);
        }
        path.closeSubpath();

        return path;
    }

// Проверка особых случаев (касания, вложенности)
    static QPolygonF checkSpecialCases(const QPolygonF &poly1, const QPolygonF &poly2) {
        const double epsilon = 1e-6;

        // Проверяем, находится ли один полигон внутри другого
        auto pointInPolygon = [](const QPointF &p, const QPolygonF &poly) -> bool {
            int windingNumber = 0;
            int n = poly.size();

            for (int i = 0; i < n; i++) {
                const QPointF &p1 = poly[i];
                const QPointF &p2 = poly[(i + 1) % n];

                if (p1.y() <= p.y()) {
                    if (p2.y() > p.y()) {
                        if ((p2.x() - p1.x()) * (p.y() - p1.y()) -
                            (p.x() - p1.x()) * (p2.y() - p1.y()) > 0) {
                            windingNumber++;
                        }
                    }
                } else {
                    if (p2.y() <= p.y()) {
                        if ((p2.x() - p1.x()) * (p.y() - p1.y()) -
                            (p.x() - p1.x()) * (p2.y() - p1.y()) < 0) {
                            windingNumber--;
                        }
                    }
                }
            }

            return windingNumber != 0;
        };

        // Проверяем центр полигона 1 в полигоне 2
        QPointF center1 = poly1.boundingRect().center();
        QPointF center2 = poly2.boundingRect().center();

        if (pointInPolygon(center1, poly2)) {
            // poly1 полностью внутри poly2
            return poly1;
        }

        if (pointInPolygon(center2, poly1)) {
            // poly2 полностью внутри poly1
            return poly2;
        }

        // Проверяем касание границ
        // Для этого создаем слегка "раздутые" полигоны
        const double inflateAmount = 0.001;

        QPolygonF inflated1 = inflatePolygon(poly1, inflateAmount);
        QPolygonF inflated2 = inflatePolygon(poly2, inflateAmount);

        QPainterPath path1, path2;
        path1.addPolygon(inflated1);
        path2.addPolygon(inflated2);

        QPainterPath intersected = path1.intersected(path2);

        if (!intersected.isEmpty()) {
            QPolygonF result = intersected.toFillPolygon();
            return simplifyPolygon(result, epsilon);
        }

        return QPolygonF();
    }

    static QPolygonF preciseIntersected(const QPolygonF &poly1, const QPolygonF &poly2) {
        // Быстрые проверки
        if (poly1.size() < 3 || poly2.size() < 3) return QPolygonF();

        // Проверка bounding boxes
        QRectF bbox1 = poly1.boundingRect();
        QRectF bbox2 = poly2.boundingRect();
        if (!bbox1.intersects(bbox2)) return QPolygonF();

        // Создаем пути с повышенной точностью
        QPainterPath path1 = polygonToHighPrecisionPath(poly1);
        QPainterPath path2 = polygonToHighPrecisionPath(poly2);

        // Вычисляем пересечение
        QPainterPath intersected = path1.intersected(path2);

        if (intersected.isEmpty()) {
            // Проверяем особые случаи
            return checkSpecialCases(poly1, poly2);
        }

        // Получаем полигон с обработкой ошибок
        QPolygonF result = extractPrecisePolygon(intersected);

        // Проверяем результат
        if (result.size() < 3) {
            return QPolygonF();
        }

        return result;
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
        for(int i = 0; i < check2.count()-1; ++i) {
            if(i == 0)
                whole = preciseIntersected(check2[i], check2[i+1]);
        }
        double squareWhole = polygonArea(whole);

        std::cout << "utilz [updateCellRule] squareWhole is " << squareWhole << " sumSq is " << sumSq << " min is " << min << std::endl;

        rule = (squareWhole == 0) ? 0 : (squareWhole < min) ? 1 : 2; // squareWhole == min входит в 2

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
            if (result.size() < minPoints && i % (points.size() / minPoints) == 0) {
                result.append(curr);
            }
        }

        result.append(points.last());
        return result;
    }

}
#endif //TRYCELLUARPATHCOMPOSE_UTILS_H
