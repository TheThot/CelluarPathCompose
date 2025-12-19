//
// Created by Admin on 08.12.2025.
//
#include "decomposer.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <ostream>
#include <vector>


Decomposer::Decomposer(QObject *parent)
    : QObject(parent)
    , m_sweepAngle(0.0)
    , m_showDecomposition(true)
    , m_showOrientedRect(true)
{
//    createDefaultPolygon();
    createPolygonWithHoles();
    updateDecomposition();
    transects = PathGenerator(m_originalPolygon, m_orientedRect, 10, m_sweepAngle);
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

QVariantList Decomposer::holes_2Darray() const{
    return configListVariantLists<QList<QPolygonF>>(m_holes);
}

QVariantList Decomposer::bpdDecompositionCells() const {
    QVariantList cellsList;
    for (const QPolygonF &cell : m_bpd_decompositionCells) {
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

QVariantList Decomposer::orientedHoleRects() const {
    return configListVariantLists<QList<QPolygonF>>(m_orientedHoleRects);
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
    m_orientedHoleRects.clear();
    m_mapOriendtedHoleRectLines.clear();
    m_bpd_decompositionCells.clear();

    if (m_originalPolygon.size() < 3) {
        emit decompositionCellsChanged();
        emit orientedRectChanged();
        return;
    }

    // Вычисляем ориентированный ограничивающий прямоугольник
    QMap<OrientedLine, QLineF> buff = {};
    m_orientedRect = getOrientedBoundingRect(m_originalPolygon, buff, m_sweepAngle);
    for(int i = 0; i < m_orientedRect.count(); ++i){
        for(int j = 0; j < m_orientedRect.count(); ++j) {
            uint64_t maxNumBuff = static_cast<uint64_t>(QLineF(m_orientedRect[i], m_orientedRect[j]).length());
            if (maxBoundSurvPolyRad < maxNumBuff)
                maxBoundSurvPolyRad = maxNumBuff * 2;
        }
    }
    m_orientedHoleRects = getOrientedBoundingHoleRects(m_originalPolygon, m_holes,m_sweepAngle);

    // Выполняем трапецоидальную декомпозицию
    std::vector<QPolygonF> cells = trapezoidalDecomposition(m_originalPolygon, m_sweepAngle);
    m_decompositionCells = QVector<QPolygonF>(cells.begin(), cells.end());

    // Выполняем бустрофедон декомпозицию
    m_bpd_decompositionCells = boustrophedonDecomposition_compact(m_originalPolygon, m_holes, m_mapOriendtedHoleRectLines, m_sweepAngle);

    emit decompositionCellsChanged();
    emit orientedRectChanged();
}

void Decomposer::resetPolygon() {
    createDefaultPolygon();
    emit originalPolygonChanged();
    updateDecomposition();
}

void Decomposer::resetToPolygonWithHoleState() {
    createPolygonWithHoles();
    emit originalPolygonChanged();
    updateDecomposition();
}

// Алгоритм трапецоидальной декомпозиции
std::vector<QPolygonF> Decomposer::trapezoidalDecomposition(const QPolygonF& polygon, double sweepAngle) {
    std::vector<QPolygonF> cells;

    if (polygon.size() < 3) {
        return cells;
    }

    // поворачиваем полигон
    auto rotatedPolygon = rotationStruct<QPolygonF>(polygon, sweepAngle);

    // упорядовачиваем X-координаты вершин, уникальные
    QList<qreal> xLevels;
    for (const QPointF& point : rotatedPolygon) {
        if (!xLevels.contains(point.x())) {
            xLevels.append(point.x());
        }
    }
    std::sort(xLevels.begin(), xLevels.end());

    // для каждого уровня X находим пересечения с ребрами
    QLineF level, edge;
    for (int i = 0; i < xLevels.size() - 1; ++i) {
        double x1 = xLevels[i];
        double x2 = xLevels[i + 1];
        // пропуск случая малого уровня
        if (std::abs(x2 - x1) < 1e-9) {
            continue;
        }

        // находим центр уровня и все пересечения
        double xMid = (x1 + x2) / 2;
        level = QLineF(QPointF(xMid, -10e5),
                       QPointF(xMid, 10e5));
        QList<QPointF> intersections;

        for (int j = 0; j < rotatedPolygon.size(); ++j) {
            int k = (j + 1) % rotatedPolygon.size(); // следующий и замыкание
            edge = QLineF(rotatedPolygon[j], rotatedPolygon[k]);

            // Проверяем пересечение
            intersectionListFormimgRoutine(level, edge, intersections, QLineF::BoundedIntersection);
        }

        // Сортируем пересечения
        std::sort(intersections.begin(), intersections.end(),
                  [](const QPointF& a, const QPointF& b) {
                      bool res = false;
                      if (a.y() != b.y())
                          res = a.y() < b.y();
                      return res;
                  });
        // Создаем трапеции между пересечениями
        for (int j = 0; j < intersections.size() - 1; j += 2) {
            double y1 = intersections[j].y();
            double y2 = intersections[j + 1].y();

            if (y2 - y1 < 1e-9) {
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
                rotatedTrapezoid.append(inverseRotatePoint(point, sweepAngle));
            }

            cells.push_back(rotatedTrapezoid);
        }
    }

    return cells;
}

void Decomposer::intersectionListFormimgRoutine(const QLineF& l1, const QLineF& l2, QList<QPointF>& intersections_list, QLineF::IntersectionType foundingType)
{
    // на случай QLineF::UnboundedIntersection ограничиваем возможные пересечения за пределами области определения линий самым большим размером = max(SurvPolyBoundRect)
    QPointF resIntersection{0,0};
    QLineF::IntersectionType flag = l1.intersects(l2, &resIntersection);

    if(flag != 0) // берём l2 в условие потому что это линия должна быть со структуры полигона with holes
        if(static_cast<uint64_t>(QLineF(l2.p1(), resIntersection).length()) > maxBoundSurvPolyRad)
            return;

    if (flag == foundingType)
        if (!intersections_list.contains(resIntersection))
            intersections_list.append(resIntersection);
}

QList<double> Decomposer::distanceToPointRoutine(const QPointF& point, const QList<QPointF>& intersections_list)
{
    QList<double> result;
    for(int i = 0; i < intersections_list.count(); ++i)
        result.append(QLineF(point, intersections_list[i]).length());
    return result;
}

QLineF Decomposer::findLineBetweenLines(const QLineF& parall1, const QLineF& parall2, const QPointF& coord)
{
    // Получаем нормальный единичный вектор
    QLineF normal = parall1.normalVector();
    QPointF unitNormal = normal.p2() - normal.p1();

    // Создаем линию через точку coord в перпендикулярном направлении
    qreal length = 1e3;

    auto approxToPrecise = QLineF{coord - unitNormal * length, coord + unitNormal * length};
    QPointF preciseP;
    parall1.intersects(approxToPrecise, &preciseP);
    approxToPrecise.setP1(preciseP);
    parall2.intersects(approxToPrecise, &preciseP);
    approxToPrecise.setP2(preciseP);
    return approxToPrecise;

}

void Decomposer::upDownBorderFormingRoutineNewMannerReadyPoly(const QMap<OrientedLine, QLineF>& inMap,
                                                              const QPolygonF& hole,
                                                              QPolygonF& returnUp,
                                                             QPolygonF& returnDown)
{
    QPointF centerHole(0, 0);
    for (const QPointF& p : hole) {
        centerHole += p;
    }
    centerHole /= hole.size();

    //линия делящая на U и D
    auto midLine = findLineBetweenLines(inMap[OrientedLine::ParallelSweepL], inMap[OrientedLine::ParallelSweepR], centerHole);

    // проверим какие из точек ParallelSweepL ParallelSweepR p1 или p2 ближе друг к другу?
    QList<QLineF> box;
    box       << QLineF(inMap[OrientedLine::ParallelSweepL].p2(), inMap[OrientedLine::ParallelSweepR].p2())
              << QLineF(inMap[OrientedLine::ParallelSweepL].p1(), inMap[OrientedLine::ParallelSweepR].p2())
              << QLineF(inMap[OrientedLine::ParallelSweepL].p2(), inMap[OrientedLine::ParallelSweepR].p1())
              << QLineF(inMap[OrientedLine::ParallelSweepL].p1(), inMap[OrientedLine::ParallelSweepR].p1());
    QList<double> distCheck;
    for(const auto& currL : box){
        distCheck.append(currL.length());
    }

    auto resIdx = sort_indexes<double>(distCheck);

    returnUp << box[resIdx[0]].p1()
             << box[resIdx[0]].p2()
             << midLine.p1()
             << midLine.p2();
    returnDown  << box[resIdx[1]].p1()
                << box[resIdx[1]].p2()
                << midLine.p1()
                << midLine.p2();
}

void Decomposer::lineHoleUpDownBorderFormingRoutineNewManner(const QMap<OrientedLine, QLineF>& inMap,
                                                             const QPolygonF& hole,
                                                             QPolygonF& returnUpPoly,
                                                             QPolygonF& returnDownPoly,
                                                             QList<QPointF>& returnUpL,
                                                             QList<QPointF>& returnDownL)
{
    upDownBorderFormingRoutineNewMannerReadyPoly(inMap, hole, returnUpPoly, returnDownPoly);
    returnUpPoly = sortPolygonClockwise(returnUpPoly);
    returnDownPoly = sortPolygonClockwise(returnDownPoly);
    //просмотрим все точки Inner Hole на вхождение в зону
    for(const auto& holeP: hole){
        if(returnUpPoly.containsPoint(holeP, Qt::OddEvenFill)){
            if (!returnUpL.contains(holeP))
                returnUpL.append(holeP);
        }
        if(returnDownPoly.containsPoint(holeP, Qt::OddEvenFill)){
            if (!returnDownL.contains(holeP))
                returnDownL.append(holeP);
        }
    }
}

/**
 * @brief Пролонгирует линию в обе стороны на заданное расстояние
 * @param line Исходная линия
 * @param delta Расстояние для удлинения в каждую сторону
 * @return Пролонгированная линия
 */
QLineF Decomposer::extendLineBothWays(const QLineF& line, qreal delta)
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


void Decomposer::newBorderFormingRoutine(const QMap<OrientedLine, QLineF>& inMap,
                                         const QPolygonF& hole,
                                         QLineF& returnUp,
                                         QLineF& returnDown)
{
    QList<QPointF> intersectionsR_list, intersectionsL_list;
    QList<double> disR_list, disL_list;
    // формируем прямоугольник описывающий hole
    QMap<OrientPointNames, QPointF> orientRectOverHole;
    int size = inMap.count() == 4 ? inMap.count() : 4;

    for(int i = 0; i < size; ++i){
        int k = (i + 1) % size;
        auto orntL_i = static_cast<OrientedLine>(i);
        auto orntL_k = static_cast<OrientedLine>(k);
        inMap[orntL_i].intersects(inMap[orntL_k],
                                  &orientRectOverHole[static_cast<OrientPointNames>(k)]);
    }

    //определяем пересечения holes c ParallelSweepL и ParallelSweepR
    const auto currParallelOrientLineL = inMap[OrientedLine::ParallelSweepL];
    const auto currParallelOrientLineR = inMap[OrientedLine::ParallelSweepR];

    for(int i = 0; i < hole.count(); ++i){
        int k = (i + 1) % hole.count();
        QLineF currHoleLine = QLineF(hole[i], hole[k]);
        currHoleLine = extendLineBothWays(currHoleLine, 10);
        // для надёжности оба типа внутреннее и внешнее пересечения
//        intersectionListFormimgRoutine(currParallelOrientLineL, currHoleLine, intersectionsL_list, QLineF::UnboundedIntersection);
        intersectionListFormimgRoutine(currParallelOrientLineL, currHoleLine, intersectionsL_list, QLineF::BoundedIntersection);
//        intersectionListFormimgRoutine(currParallelOrientLineR, currHoleLine, intersectionsR_list, QLineF::UnboundedIntersection);
        intersectionListFormimgRoutine(currParallelOrientLineR, currHoleLine, intersectionsR_list, QLineF::BoundedIntersection);
    }
    std::cout << "Count L intersections " << intersectionsL_list.count() << std::endl;
    std::cout << "Count R intersections " << intersectionsR_list.count() << std::endl;
    //пересечения определены, после вычисляем расстояния соотвественно до D и U заносим в массив и сортируем
    // финальные линии returnUp и returnDown формируем мин расстоянием соответсвенно
    disR_list = distanceToPointRoutine(orientRectOverHole[OrientPointNames::RightBottom], intersectionsR_list);
    disL_list = distanceToPointRoutine(orientRectOverHole[OrientPointNames::RightTop], intersectionsL_list);

    auto resIdxR = sort_indexes<double>(disR_list);
    auto resIdxL = sort_indexes<double>(disL_list);
    //поскольку проверяется оба типа пересечений то небходимо делать доп фильтрацию точек уходящих в результат

    // теперь мы знаем индекс по intersectionsR_list и intersectionsL_list для hole линии пересечение и формируем U и D
    returnUp    = QLineF(intersectionsL_list[resIdxL[0]], intersectionsR_list[resIdxR[0]]);
    returnDown  = QLineF(intersectionsL_list[resIdxL[resIdxL.size()-1]], intersectionsR_list[resIdxR[resIdxR.size()-1]]);
}

void Decomposer::newParallFormingRoutine(const QMap<OrientedLine, QLineF>& inMap,
                                         const QPolygonF& survPolyBound,
                                         QLineF& returnL,
                                         QLineF& returnR)
{
    const auto& parallelL = inMap[OrientedLine::ParallelSweepL];
    const auto& parallelR = inMap[OrientedLine::ParallelSweepR];
    QList<QPointF> resIntersectionsL, resIntersectionsR;
    for (int i = 0; i < survPolyBound.count(); i++)
    {
        int k = (i + 1) % survPolyBound.count();
        auto buffLine = QLineF(survPolyBound[i], survPolyBound[k]);
        intersectionListFormimgRoutine(parallelL, buffLine, resIntersectionsL, QLineF::UnboundedIntersection);
        intersectionListFormimgRoutine(parallelR, buffLine, resIntersectionsR, QLineF::UnboundedIntersection);
    }
    returnL = QLineF(resIntersectionsL[0], resIntersectionsL[1]);
    returnR = QLineF(resIntersectionsR[0], resIntersectionsR[1]);
}

template <typename T>
T Decomposer::rotationStruct(const T& v, double sweepAngle) {
    T res;
    for (const QPointF& point : v) {
        res.append(rotatePoint(point, sweepAngle));
    }
    return res;
}

enum class Decomposer::BCD_levels
{
    //TODO
};

// реализация деления полигона с отверстиями на зоны
// задел для path construcion алгоритма туда-обратно
QList<QPolygonF> Decomposer::boustrophedonDecomposition_compact(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                                const QList<QMap<OrientedLine, QLineF>>& mapOriendtedHoleRectLines,
                                                                double sweepAngle)
{
    QList<QPolygonF> resCells;

    if (polygon.size() < 3) {
        return resCells;
    }
    if (holes.count() < 1){
        return resCells;
    }

    const auto& copySurvPolyBound = m_orientedRect; // убедиться что m_orientedRect создан на момент вызова boustrophedonDecomposition
    QList<QMap<OrientedLine, QLineF>> copy;
    QMap<OrientedLine, QLineF> empty = {};
    copy.reserve(mapOriendtedHoleRectLines.count());
    for (int i = 0; i < mapOriendtedHoleRectLines.count(); ++i) {
        copy.append(empty);
    }

    // ------------вспомогательные функции--------------------------
    auto flambdaDirect = [](QLineF in) {
        QPolygonF buff;
        buff.reserve(2);
        buff << in.p1() << in.p2();
        return buff;
    };
    auto flambdaReverse = [](QPolygonF in) {
        return QLineF(in[0], in[1]);
    };
    // ------------!!!!!!!!!!!!!!!!!!!!!!!---------------------------
    QList<qreal> yLevelsOrientLines;
    QList<QList<qreal>> yLevelsHolesList;
    yLevelsHolesList.reserve(mapOriendtedHoleRectLines.count());
    for (int i = 0; i < mapOriendtedHoleRectLines.count(); ++i) {
        yLevelsHolesList.append(QList<qreal>{});
    }
    QList<QPolygonF> readyPolyUp, readyPolyDown;
    QList<QList<QPointF>> holeBorderUp, holeBorderDown;
    readyPolyUp.reserve(mapOriendtedHoleRectLines.count());
    readyPolyDown.reserve(mapOriendtedHoleRectLines.count());
    holeBorderUp.reserve(mapOriendtedHoleRectLines.count());
    holeBorderDown.reserve(mapOriendtedHoleRectLines.count());
    for (int i = 0; i < mapOriendtedHoleRectLines.count(); ++i) {
        readyPolyUp.append(QPolygonF{});
        readyPolyDown.append(QPolygonF{});
        holeBorderUp.append(QList<QPointF>{});
        holeBorderDown.append(QList<QPointF>{});
    }
    for(int i = 0; i < mapOriendtedHoleRectLines.count(); ++i) //распаковываем по одной штук на каждую hole
    {
        QLineF parallelSweepLRot, parallelSweepRRot;
        //auto orntLn = static_cast<OrientedLine>(numLine); // OrientedLine

        // Организация данных в новую структуру схожую с mapOriendtedHoleRectLines
        // 0. Preprocessing получаем линии Parallel L и К длинные до пересечений boundedRect SurvPoly
        newParallFormingRoutine(mapOriendtedHoleRectLines[i], copySurvPolyBound,
                                copy[i][OrientedLine::ParallelSweepL],
                                copy[i][OrientedLine::ParallelSweepR]);
        /*m_mapOriendtedHoleRectLines[i][OrientedLine::ParallelSweepL] = copy[i][OrientedLine::ParallelSweepL];
        m_mapOriendtedHoleRectLines[i][OrientedLine::ParallelSweepR] = copy[i][OrientedLine::ParallelSweepR];*/
        // ----------------------------------------------------
        newBorderFormingRoutine(mapOriendtedHoleRectLines[i], holes[i],
                                copy[i][OrientedLine::PerpendiclSweepU],
                                copy[i][OrientedLine::PerpendiclSweepD]);
        // 1. И формируется новые U и D границы
        lineHoleUpDownBorderFormingRoutineNewManner(copy[i], holes[i],
                                                    readyPolyUp[i],
                                                    readyPolyDown[i],
                                                    holeBorderUp[i],
                                                    holeBorderDown[i]);
        // cheat [готовый полигон]
        /*upDownBorderFormingRoutineNewMannerReadyPoly(copy[i], holes[i],
                                                    readyPolyUp[i],
                                                    readyPolyDown[i]);*/

        // 2. В compact реализации 2 и 3 пункт пропускаем
        // 4. Поворот Parallel L и R
        //  TODO ------------- можно в цикл организовать ------
        QPolygonF buff = flambdaDirect(copy[i][OrientedLine::ParallelSweepL]);
        parallelSweepLRot = flambdaReverse(rotationStruct<QPolygonF>(buff, sweepAngle));
        buff = flambdaDirect(copy[i][OrientedLine::ParallelSweepR]);
        parallelSweepRRot = flambdaReverse(rotationStruct<QPolygonF>(buff, sweepAngle));
        // ----------------------------------------------------
        // упорядовачиваем координаты вершин, уникальные, линий ограничивающих зоны
        // линии паралельны потому без разницы [0] p1 или [1] p2
        if (!yLevelsOrientLines.contains(parallelSweepLRot.p1().y())) {
            yLevelsOrientLines.append(parallelSweepLRot.p1().y());
            yLevelsHolesList[i].append(parallelSweepLRot.p1().y());
        }
        if (!yLevelsOrientLines.contains(parallelSweepRRot.p1().y())) {
            yLevelsOrientLines.append(parallelSweepRRot.p1().y());
            yLevelsHolesList[i].append(parallelSweepRRot.p1().y());
        }
    }
    // поворачиваем полигон SurvPoly
    auto rotatedPolygon = rotationStruct<QPolygonF>(polygon, sweepAngle);
    // после того как весь QList mapOriendtedHoleRectLines обработан и сориентрован выполняем деление на зоны
    // 4. формирование yLevels по аналогии с trapezoidalCellAlgorithm и зон
    // упорядовачиваем координаты вершин, уникальные
    QList<qreal> yLevels;
    for (const QPointF& point : rotatedPolygon) {
        if (!yLevels.contains(point.y())) {
            yLevels.append(point.y());
        }
    }
    std::sort(yLevels.begin(), yLevels.end());
    std::sort(yLevelsOrientLines.begin(), yLevelsOrientLines.end());
    //сформируем для проверки две крайних боковых зоны
    double yMinPoly = yLevels[0], yMinHole = yLevelsOrientLines[0];
    QPolygonF buff;
    QLineF level, edge;
    level = QLineF(QPointF(-10e5, yMinHole),
                        QPointF(10e5, yMinHole));
    QList<QPointF> intersections;
    for (int j = 0; j < rotatedPolygon.size(); ++j) {
        int k = (j + 1) % rotatedPolygon.size(); // следующий и замыкание
        if(rotatedPolygon[j].y() >= yMinPoly && rotatedPolygon[j].y() < yMinHole){
            buff.append(rotatedPolygon[j]);
        }
        edge = QLineF(rotatedPolygon[j], rotatedPolygon[k]);
        intersectionListFormimgRoutine(level, edge, intersections, QLineF::BoundedIntersection);
    }
    buff.append(intersections[0]); buff.append(intersections[1]);
    buff = rotationStruct<QPolygonF>(buff, -sweepAngle); // поворачиваем обратно точки декомпозиции
    buff = sortPolygonClockwise(buff);
    resCells.append(buff);
    // ------------ закончили один край ---------- приступили к другому ----------------
    buff.clear(); intersections.clear();
    yMinPoly = yLevels[yLevels.count()-1]; yMinHole = yLevelsOrientLines[yLevelsOrientLines.count()-1];
    level = QLineF(QPointF(-10e5, yMinHole),
                   QPointF(10e5, yMinHole));
    for (int j = 0; j < rotatedPolygon.size(); ++j) {
        int k = (j + 1) % rotatedPolygon.size(); // следующий и замыкание
        if(rotatedPolygon[j].y() <= yMinPoly && rotatedPolygon[j].y() > yMinHole){
            buff.append(rotatedPolygon[j]);
        }
        edge = QLineF(rotatedPolygon[j], rotatedPolygon[k]);
        intersectionListFormimgRoutine(level, edge, intersections, QLineF::BoundedIntersection);
    }
    buff.append(intersections[0]); buff.append(intersections[1]);
    buff = rotationStruct<QPolygonF>(buff, -sweepAngle); // поворачиваем обратно точки декомпозиции
    buff = sortPolygonClockwise(buff);
    resCells.append(buff);
    // ------------ в промежутках ----------------
    // на практике Mannner biggerThanPoly rotation is clockwise может являться индикатором актуальности этой cell 0 - добавляем, 1 - скипаем
    buff.clear(); intersections.clear();
    for(int i = 0; i < copy.count() - 1; ++i)
    {
        int k = i + 1;
        QLineF firstPar = copy[k][OrientedLine::ParallelSweepR];
        QLineF secPar = copy[i][OrientedLine::ParallelSweepL]; // странно почему у i OrientedLine::ParallelSweepL у k OrientedLine::ParallelSweepR, а не наоборот [на подумать?]
        QPolygonF biggerThanPoly, biggerThanPolyRev;
        biggerThanPoly  << copy[k][OrientedLine::ParallelSweepR].p1()
                        << copy[i][OrientedLine::ParallelSweepL].p1()
                        << copy[i][OrientedLine::ParallelSweepL].p2()
                        << copy[k][OrientedLine::ParallelSweepR].p2();
        //условие положительное ли направление движения между firstPar и secPar или отрицательное
        if(isPolyInClockwiseMannerRot(biggerThanPoly.data(), biggerThanPoly.count()))
            break;
        for (int j = 0; j < polygon.size(); ++j) {
            int w = (j + 1) % polygon.size(); // следующий и замыкание
            edge = QLineF(polygon[j], polygon[w]);
            if(biggerThanPoly.containsPoint(polygon[j], Qt::WindingFill)){ // containsPoint QPolygonF нужно усиливать для точек на краю
                buff.append(polygon[j]);
            }
            intersectionListFormimgRoutine(firstPar, edge, intersections, QLineF::BoundedIntersection);
            intersectionListFormimgRoutine(secPar, edge, intersections, QLineF::BoundedIntersection);
        }
        for (const auto& currP: intersections)
        {
            buff.append(currP);
        }
        buff = sortPolygonClockwise(buff);
        resCells.append(biggerThanPoly);
    }
    // решение для 180 градусов обратно все k -> i а i -> k
    buff.clear(); intersections.clear();
    for(int i = 0; i < copy.count() - 1; ++i)
    {
        int k = i + 1;
        QLineF firstParRevers = copy[i][OrientedLine::ParallelSweepR];
        QLineF secParRevers = copy[k][OrientedLine::ParallelSweepL];
        QPolygonF biggerThanPolyRev;
        biggerThanPolyRev   << copy[i][OrientedLine::ParallelSweepR].p1()
                            << copy[k][OrientedLine::ParallelSweepL].p1()
                            << copy[k][OrientedLine::ParallelSweepL].p2()
                            << copy[i][OrientedLine::ParallelSweepR].p2();
        //условие положительное ли направление движения между firstPar и secPar или отрицательное
        if(isPolyInClockwiseMannerRot(biggerThanPolyRev.data(), biggerThanPolyRev.count()))
            break;
        for (int j = 0; j < polygon.size(); ++j) {
            int w = (j + 1) % polygon.size(); // следующий и замыкание
            edge = QLineF(polygon[j], polygon[w]);
            if(biggerThanPolyRev.containsPoint(polygon[j], Qt::WindingFill)){ // containsPoint QPolygonF нужно усиливать для точек на краю
                buff.append(polygon[j]);
            }
            intersectionListFormimgRoutine(firstParRevers, edge, intersections, QLineF::BoundedIntersection);
            intersectionListFormimgRoutine(secParRevers, edge, intersections, QLineF::BoundedIntersection);
        }
        for (const auto& currP: intersections)
        {
            buff.append(currP);
        }
        buff = sortPolygonClockwise(buff);
        resCells.append(biggerThanPolyRev);
    }
    // ------------ решение для зон с U и D ----------------

    for(int i = 0; i < copy.count(); ++i)
    {
        buff.clear();
        buff    << copy[i][OrientedLine::PerpendiclSweepU].p1()
                << copy[i][OrientedLine::PerpendiclSweepU].p2()
                << copy[i][OrientedLine::ParallelSweepL].p2()
                << copy[i][OrientedLine::ParallelSweepR].p2();
        buff = sortPolygonClockwise(buff);
        resCells.append(buff);
        buff.clear();
        buff    << copy[i][OrientedLine::PerpendiclSweepD].p1()
                << copy[i][OrientedLine::PerpendiclSweepD].p2()
                << copy[i][OrientedLine::ParallelSweepL].p1()
                << copy[i][OrientedLine::ParallelSweepR].p1();
        buff = sortPolygonClockwise(buff);
        resCells.append(buff);
    }
    /*for(int i = 0; i < mapOriendtedHoleRectLines.count(); ++i)
    {
        readyPolyDown[i] = sortPolygonClockwise(readyPolyDown[i]);
        readyPolyUp[i] = sortPolygonClockwise(readyPolyUp[i]);
        resCells.append(readyPolyDown[i]);
        resCells.append(readyPolyUp[i]);
    }*/
    // ---------- more precise для зон с U и D ----------------
    /*for(int i = 0; i < mapOriendtedHoleRectLines.count(); ++i)
    {
        buff.clear();
        buff.append(readyPolyUp[i]);
        for(const auto& currP : holeBorderUp[i]){
            buff.append(currP);
        }
        resCells.append(buff);
        buff.clear();
        buff.append(readyPolyDown[i]);
        for(const auto& currP : readyPolyDown[i]){
            buff.append(currP);
        }
        resCells.append(buff);
    }*/

    return resCells;
}

// not finalized yet
QList<QPolygonF> Decomposer::boustrophedonDecomposition(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                        const QList<QMap<OrientedLine, QLineF>>& mapOriendtedHoleRectLines,
                                                        double sweepAngle) {
    QList<QPolygonF> resCells{};

    if (polygon.size() < 3) {
        return resCells;
    }
    if (holes.count() < 1){
        return resCells;
    }

    //все линии polygon и holes для удобного движения по списку в пункте 2
    /*QList<QLineF> wholeLinesList;
    for(int i = 0; i < polygon.count(); ++i){
        int k = (i+1) % polygon.count();
        wholeLinesList.append(QLineF(polygon[i], polygon[k]));
    }
    for(const auto& currHole : holes){
        for(int i = 0; i < currHole.count(); ++i){
            int k = (i+1) % currHole.count();
            wholeLinesList.append(QLineF(currHole[i], currHole[k]));
        }
    }

    int numLine = 0;
    QList<QMap<OrientedLine, QLineF>> copy;
    QMap<OrientedLine, QLineF> empty = {};
    copy.reserve(mapOriendtedHoleRectLines.count());
    for (int i = 0; i < mapOriendtedHoleRectLines.count(); ++i) {
        copy.append(empty);
    }
    for(int i = 0; i < mapOriendtedHoleRectLines.count(); ++i) //распаковываем по одной штук на каждую hole
    {
        //auto orntLn = static_cast<OrientedLine>(numLine); // OrientedLine
        // 1. Организация данных в новую структуру схожую с mapOriendtedHoleRectLines,
        // ParallelSweepL и ParallelSweepR в направлении галса берутся без изменения
        copy[i][OrientedLine::ParallelSweepL] = mapOriendtedHoleRectLines[i][OrientedLine::ParallelSweepL];
        copy[i][OrientedLine::ParallelSweepR] = mapOriendtedHoleRectLines[i][OrientedLine::ParallelSweepR];
        // и формируется новые U и D границы
        newBorderFormingRoutine(mapOriendtedHoleRectLines[i], holes[i],
                                copy[i][OrientedLine::PerpendiclSweepU],
                                copy[i][OrientedLine::PerpendiclSweepD]);

        // 2. Пересечения для линий ParallelSweepL и ParallelSweepR из mapOriendtedHoleRectLines до всех возможных границ Polygon и Polygon Holes
        // -> результат для каждой линии свой список

        ++numLine; //?? это нужно ??

    }*/

    return resCells;
}

QList<QPolygonF> Decomposer::getOrientedBoundingHoleRects(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                          double angleDegrees) {
//    m_mapOriendtedHoleRectLines.clear();
    QList<QPolygonF> orientedRects;

    if (polygon.size() < 3) {
        return QList<QPolygonF>{};
    }
    if (holes.count() < 1){
        return QList<QPolygonF>{};
    }

    QMap<OrientedLine, QLineF> buff;
    for(const auto& currHole: holes) {
        QPolygonF currOrieRect = getOrientedBoundingRect(currHole, buff, angleDegrees);
        m_mapOriendtedHoleRectLines.append(buff);
        orientedRects.append(currOrieRect);
    }

    return orientedRects;
}

// Ориентированный ограничивающий прямоугольник
QPolygonF Decomposer::getOrientedBoundingRect(const QPolygonF& polygon, QMap<OrientedLine, QLineF>& currOrient,
                                              double angleDegrees) {
    if (polygon.size() < 3) {
        return QPolygonF{};
    }

    currOrient.clear();

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
        QPointF(minX, minY), // [0] - лев ниж
        QPointF(maxX, minY), // [1] - прав ниж
        QPointF(maxX, maxY), // [2] - прав верх
        QPointF(minX, maxY)  // [3] - лев верх
    };

    // Поворачиваем углы обратно на angle
    QPolygonF orientedRect;
    for (auto currCorn : corners)
    {
        auto pR = inverseRotatePoint(QPointF(currCorn.x() - centroid.x(), currCorn.y() - centroid.y()),
                                        angleDegrees);

        orientedRect << QPointF(pR.x() + centroid.x(), pR.y() + centroid.y());
    }

    // Формируем currOrient учитываем что sweep line галс параллелен X без поворота
    // против часовой стрелки ориентация
    // финалим currOrient
    for (int pNum  = 0; pNum < orientedRect.count(); pNum++)
    { // [3 - PerpendiclSweepD] 3 и 0 точки, [0 - ParallelSweepR] 0 и 1, [1 - PerpendiclSweepU] 1 и 2 точки, [2 - ParallelSweepL] 2 и 3 точки
        QLineF buff = {};
        auto orntLn = static_cast<OrientedLine>(pNum); // OrientedLine - PerpendiclSweepD, ParallelSweepR, PerpendiclSweepU, ParallelSweepL
        int nextP = (pNum+1) % orientedRect.count();
        buff = QLineF(orientedRect[pNum], orientedRect[nextP]);
        currOrient[orntLn] = buff;
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

//TODO определить порядок задания данных что первичнее qml или ++ ?
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

void Decomposer::createPolygonWithHoles() {
    m_originalPolygon.clear();
    m_holes.clear();
    // Шестиугольник
    m_originalPolygon << QPointF(200, 100)
                      << QPointF(400, 100)
                      << QPointF(500, 250)
                      << QPointF(400, 400)
                      << QPointF(200, 400)
                      << QPointF(100, 250);
    QPolygonF oneHole;
    oneHole << QPointF(180, 180)
            << QPointF(270, 180)
            << QPointF(300, 225)
            << QPointF(270, 270)
            << QPointF(180, 270)
            << QPointF(150, 225);

    m_holes.append(oneHole);
    QPolygonF secHole;
    /*secHole << QPointF(300, 350)
            << QPointF(350, 350)
            << QPointF(350, 300)
            << QPointF(300, 300);*/
    secHole << QPointF(289.64, 325.00)
            << QPointF(325.00, 360.36)
            << QPointF(360.36, 325.00)
            << QPointF(325.00, 289.64);
    m_holes.append(secHole);
}

//проверка лежит ли точка на прямой
bool Decomposer::isPointOnLineF(const QPointF& p, const QLineF& line){
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

double Decomposer::lineEquationKoeff(const QPointF& p1, const QPointF& p2){
    double res;
    res = (p2.x() - p1.x()) / (p2.y() + p1.y());
    return res;
}

//проверка на направление вращения полигона true - clockwise; false - counter-clockwise
bool Decomposer::isPolyInClockwiseMannerRot(const QPointF* polygonPoints, uint size){
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

QPolygonF Decomposer::sortPolygonClockwise(QPolygonF polygon) {
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

