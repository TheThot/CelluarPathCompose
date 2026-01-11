//
// Created by Admin on 08.12.2025.
//
#include "decomposer.h"
#include <algorithm>
#include <iostream>
#include <ostream>
#include <vector>
#include <QDebug>

using namespace baseFunc;

Decomposer::Decomposer(QObject *parent)
    : QObject(parent)
    , m_sweepAngle(0.0)
    , m_showDecomposition(true)
    , m_showOrientedRect(true)
    , _trWidth(45)
    , _pb()
{
    //создание созависимых сперва
    _transects = new PathGenerator(_trWidth, m_sweepAngle, this);
//    connect(this, &Decomposer::sweepAngleChanged, _transects, &PathGenerator::pathUpdation);
    connect(this, &Decomposer::originalPolygonChanged, this, [this] (){
        _transects->setSurvPoly(m_originalPolygon);
    });
    connect(this, &Decomposer::holesPolygonsChanged, this, [this] (){
        _transects->setPolyHolesList(m_holes);
        _transects->changeHolesActiveState(); // TODO повесить на сигнал qml смены режима
    });
    
    //после определяем остальное
//    createDefaultPolygon();
    createPolygonWithHoles();
    updateDecomposition();

}

PathGenerator* Decomposer::transects() const{
    return _transects;
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
    transects()->setGridAngle(m_sweepAngle);
//    _transects->pathUpdation();
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
//    m_mapOriendtedHoleRectLines.clear();
    m_bpd_decompositionCells.clear();
    _holeData.holeBorderSegm.clear();
    _holeData.holeToBCD.clear();

    if (m_originalPolygon.size() < 3) {
        emit decompositionCellsChanged();
        emit orientedRectChanged();
        return;
    }

    // Вычисляем ориентированный ограничивающий прямоугольник
    QMap<OrientedLine, QLineF> buff = {};
    m_orientedRect = getOrientedBoundingRect(m_originalPolygon, buff, m_sweepAngle);
    m_orientedHoleRects = getOrientedBoundingHoleRects(m_originalPolygon, m_holes,m_sweepAngle);

    // Выполняем трапецоидальную декомпозицию
    std::vector<QPolygonF> cells = trapezoidalDecomposition(m_originalPolygon, m_sweepAngle);
    m_decompositionCells = QVector<QPolygonF>::fromStdVector(cells);

    // Выполняем бустрофедон декомпозицию
    m_bpd_decompositionCells = boustrophedonDecomposition(m_originalPolygon, m_holes, m_mapOriendtedHoleRectLines, m_sweepAngle);

    /*m_bpd_decompositionCells.append(m_orientedRect);
    _debugPolyListToConsole(m_bpd_decompositionCells);
    m_bpd_decompositionCells.pop_back();*/

    feedHolesInfoIn();

    _transects->setPolyBoundary(m_orientedRect);
    _transects->setDecomposeStruct(_holeData);
    _transects->setPathSegments(m_bpd_decompositionCells);
    _transects->pathUpdation();

    emit decompositionCellsChanged();
    emit orientedRectChanged();
}

void Decomposer::resetPolygon() {
    createPolygonWithHoles();
    emit originalPolygonChanged();
    updateDecomposition();
}

void Decomposer::resetToPolygonWithHoleState() {
    createPolygonWithHoles();
    emit originalPolygonChanged();
    emit holesPolygonsChanged();
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
        inMap[orntL_i].intersect(inMap[orntL_k],
                                  &orientRectOverHole[static_cast<OrientPointNames>(k)]);
    }

    //определяем пересечения holes c ParallelSweepL и ParallelSweepR
    const auto currParallelOrientLineL = inMap[OrientedLine::ParallelSweepL];
    const auto currParallelOrientLineR = inMap[OrientedLine::ParallelSweepR];

    for(int i = 0; i < hole.count(); ++i){
        int k = (i + 1) % hole.count();
        QLineF currHoleLine = QLineF(hole[i], hole[k]);
        currHoleLine = extendLineBothWays(currHoleLine, 0.5); // для надёжности удлиняем линии границ hole

        intersectionListFormimgRoutine(currParallelOrientLineL, currHoleLine, intersectionsL_list, QLineF::BoundedIntersection);
        intersectionListFormimgRoutine(currParallelOrientLineR, currHoleLine, intersectionsR_list, QLineF::BoundedIntersection);
    }
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
//    std::cout << "Count L intersections " << intersectionsL_list.count() << std::endl;
//    std::cout << "Count R intersections " << intersectionsR_list.count() << std::endl;
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
        if (intersectionListFormimgRoutine(parallelL, buffLine, resIntersectionsL, QLineF::UnboundedIntersection))
            if (!isPointOnLineF(resIntersectionsL.last(), buffLine))
                resIntersectionsL.pop_back();
        if (intersectionListFormimgRoutine(parallelR, buffLine, resIntersectionsR, QLineF::UnboundedIntersection))
            if (!isPointOnLineF(resIntersectionsR.last(), buffLine))
                resIntersectionsR.pop_back();
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

void Decomposer::iniAllAlliasBCD(int size, QList<QMap<OrientedLine, QLineF>>& copy,
                                QList<QPolygonF>& readyPolyUp, QList<QPolygonF>& readyPolyDown,
                                QList<QList<QPointF>>& holeBorderUp,
                                QList<QList<QPointF>>& holeBorderDown)
{
    copy.reserve(size);
    readyPolyUp.reserve(size);
    readyPolyDown.reserve(size);
    holeBorderUp.reserve(size);
    holeBorderDown.reserve(size);

    for (int i = 0; i < size; ++i) {
        copy.append(QMap<OrientedLine, QLineF>{});
        readyPolyUp.append(QPolygonF{});
        readyPolyDown.append(QPolygonF{});
        holeBorderUp.append(QList<QPointF>{});
        holeBorderDown.append(QList<QPointF>{});
    }
}

// реализация деления полигона с отверстиями на зоны
// задел для path construcion алгоритма туда-обратно
QList<QPolygonF> Decomposer::boustrophedonDecomposition_compact(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                                QList<QMap<OrientedLine, QLineF>>& mapOriendtedHoleRectLines,
                                                                double sweepAngle)
{
    QList<QPolygonF> resCells;

    if (polygon.size() < 3) {
        return resCells;
    }
    if (holes.count() < 1){
        return resCells;
    }

    QList<QMap<OrientedLine, QLineF>> copy;
    QList<QPolygonF> readyPolyUp, readyPolyDown;
    QList<QList<QPointF>> holeBorderUp, holeBorderDown;

    // ------------!!!!!!!!!!!!!!!!!!!!!!!---------------------------
    iniAllAlliasBCD(mapOriendtedHoleRectLines.count(), copy, readyPolyUp, readyPolyDown, holeBorderUp, holeBorderDown);
    const auto& copySurvPolyBound = m_orientedRect; // убедиться что m_orientedRect создан на момент вызова boustrophedonDecomposition

    for(int i = 0; i < mapOriendtedHoleRectLines.count(); ++i) //распаковываем по одной штук на каждую hole
    {
        QPair<QList<QPointF>,QList<QPointF>> buffP;
        // Организация данных в новую структуру схожую с mapOriendtedHoleRectLines
        // 0. Preprocessing получаем линии Parallel L и R длинные до пересечений boundedRect SurvPoly
        newParallFormingRoutine(mapOriendtedHoleRectLines[i], copySurvPolyBound,
                                copy[i][OrientedLine::ParallelSweepL],
                                copy[i][OrientedLine::ParallelSweepR]);
        // ----------------------- для верха и низа -----------------------------
        newBorderFormingRoutine(mapOriendtedHoleRectLines[i], holes[i],
                                copy[i][OrientedLine::PerpendiclSweepU],
                                copy[i][OrientedLine::PerpendiclSweepD]);
        // 1. И формируется новые U и D границы
        lineHoleUpDownBorderFormingRoutineNewManner(copy[i], holes[i],
                                                    readyPolyUp[i],
                                                    readyPolyDown[i],
                                                    holeBorderUp[i],
                                                    holeBorderDown[i]);
        buffP.first = holeBorderUp[i];
        buffP.second = holeBorderDown[i];
        _holeData.holeBorderSegm[&m_holes[i]] = buffP;
        // 2. В compact реализации 2 и 3 пункт пропускаем
        copy[i][OrientedLine::ParallelSweepL] = extendLineBothWays(copy[i][OrientedLine::ParallelSweepL], 10);
        copy[i][OrientedLine::ParallelSweepR] = extendLineBothWays(copy[i][OrientedLine::ParallelSweepR], 10);
    }
    // поворачиваем полигон SurvPoly
    auto rotatedPolygon = rotationStruct<QPolygonF>(m_orientedRect, sweepAngle);
    // после того как весь QList mapOriendtedHoleRectLines обработан и сориентрован выполняем деление на зоны

    // ------------ решение для зон с U и D ----------------
    // в resCells удлинённые ParallelSweepLиR до survPoly границ ячейки
    QPolygonF buff;
    for(int i = 0; i < copy.count(); ++i)
    {
        buff.clear();
        buff    << copy[i][OrientedLine::PerpendiclSweepU].p1()
                << copy[i][OrientedLine::PerpendiclSweepU].p2()
                << copy[i][OrientedLine::ParallelSweepL].p2()
                << copy[i][OrientedLine::ParallelSweepR].p2();
        buff = sortPolygonCounterClockwise(buff);
        resCells.append(buff);
        buff.clear();
        buff    << copy[i][OrientedLine::PerpendiclSweepD].p1()
                << copy[i][OrientedLine::PerpendiclSweepD].p2()
                << copy[i][OrientedLine::ParallelSweepL].p1()
                << copy[i][OrientedLine::ParallelSweepR].p1();
        buff = sortPolygonCounterClockwise(buff);
        resCells.append(buff);
    }

    // ------------------------ внесения крайних зон и в промежутках с использованием clipper -------------------------------
    auto listEdgesCells = _pb.subtractedListWrp(m_orientedRect, resCells);
    std::cout << "Size of listEdgesCells is " << listEdgesCells.count() << std::endl;
    for(const auto& curr: listEdgesCells){
        resCells.append(curr);
    }

    mapOriendtedHoleRectLines = copy;

//    std::cout << "Size of cell decmps is " << resCells.count() << std::endl;

    return resCells;
}

bool Decomposer::updateOrientedLine3(QList<QMap<OrientedLine, QLineF>>& inMap, int h1, int h2){
    QList<QPointF> intersections;
    int _h1 = h1, _h2 = h2;
    if(h2 > h1) {
        _h1 = h2;
        _h2 = h1;
    }
    auto currParr = inMap[_h1][OrientedLine::ParallelSweepL];
    // проверяем пересечения для линий ParallelSweepL и ParallelSweepR из mapOriendtedHoleRectLines до новых возможных границ c Holes
    for(int j = 0; j < inMap.count(); ++j){
        if(j != _h1) {
            intersectionListFormimgRoutine(currParr, inMap[j][OrientedLine::PerpendiclSweepD], intersections,
                                           QLineF::BoundedIntersection);
        }
    }
    if(intersections.count() == 0)
        return false;
    inMap[_h2][OrientedLine::PerpendiclSweepU].setP2(intersections[0]);
    intersections.clear();
    currParr = inMap[_h2][OrientedLine::ParallelSweepR];
    // проверяем пересечения для линий ParallelSweepL и ParallelSweepR из mapOriendtedHoleRectLines до новых возможных границ c Holes
    for(int j = 0; j < inMap.count(); ++j){
        if(j != _h2) {
            intersectionListFormimgRoutine(currParr, inMap[j][OrientedLine::PerpendiclSweepU], intersections,
                                           QLineF::BoundedIntersection);
        }
    }if(intersections.count() == 0)
        return false;
    inMap[_h1][OrientedLine::PerpendiclSweepD].setP1(intersections[0]);

    if(h2 > h1) {
        inMap[h1][OrientedLine::ParallelSweepR].setP2(inMap[h2][OrientedLine::ParallelSweepL].p2());
        inMap[h2][OrientedLine::ParallelSweepL].setP1(inMap[h1][OrientedLine::ParallelSweepR].p1());
    }else{
        inMap[h2][OrientedLine::ParallelSweepR].setP2(inMap[h1][OrientedLine::ParallelSweepL].p2());
        inMap[h1][OrientedLine::ParallelSweepL].setP1(inMap[h2][OrientedLine::ParallelSweepR].p1());
    }
    return true;
}

bool Decomposer::updateOrientedLine1(QList<QMap<OrientedLine, QLineF>>& inMap, int h1, int h2){
    QList<QPointF> intersections;
    int _h1 = h1, _h2 = h2;
    if(h2 > h1) {
        _h1 = h2;
        _h2 = h1;
    }
    auto currParr = inMap[_h2][OrientedLine::ParallelSweepL];
    // проверяем пересечения для линий ParallelSweepL и ParallelSweepR из mapOriendtedHoleRectLines до новых возможных границ c Holes
    for(int j = 0; j < inMap.count(); ++j){
        if(j != _h2) {
            intersectionListFormimgRoutine(currParr, inMap[j][OrientedLine::PerpendiclSweepD], intersections,
                                           QLineF::BoundedIntersection);
        }
    }
    if(intersections.count() == 0)
        return false;
    inMap[_h1][OrientedLine::PerpendiclSweepD].setP2(intersections[0]);
    intersections.clear();
    currParr = inMap[_h1][OrientedLine::ParallelSweepR];
    // проверяем пересечения для линий ParallelSweepL и ParallelSweepR из mapOriendtedHoleRectLines до новых возможных границ c Holes
    for(int j = 0; j < inMap.count(); ++j){
        if(j != _h1) {
            intersectionListFormimgRoutine(currParr, inMap[j][OrientedLine::PerpendiclSweepU], intersections,
                                           QLineF::BoundedIntersection);
        }
    }
    if(intersections.count() == 0)
        return false;
    inMap[_h2][OrientedLine::PerpendiclSweepU].setP1(intersections[0]);

    if(h2 > h1) {
        inMap[h2][OrientedLine::ParallelSweepR].setP1(inMap[h1][OrientedLine::ParallelSweepL].p1());
        inMap[h1][OrientedLine::ParallelSweepL].setP2(inMap[h2][OrientedLine::ParallelSweepR].p2());
    }else{
        inMap[h1][OrientedLine::ParallelSweepR].setP1(inMap[h2][OrientedLine::ParallelSweepL].p1());
        inMap[h2][OrientedLine::ParallelSweepL].setP2(inMap[h1][OrientedLine::ParallelSweepR].p2());
    }
    return true;
}

void Decomposer::updateOrientedLine2(QList<QMap<OrientedLine, QLineF>>& inMap, int h1, int h2){
    bool flag = false;
    if(h2 > h1)
        flag = true;
    for(int i = 0; i < inMap.count(); ++i){
        QList<QPointF> intersectionsR;
        QList<QPointF> intersectionsL;
        auto currParrR = inMap[i][OrientedLine::ParallelSweepR];
        auto currParrL = inMap[i][OrientedLine::ParallelSweepL];
        // проверяем пересечения для линий ParallelSweepL и ParallelSweepR из mapOriendtedHoleRectLines до новых возможных границ c Holes
        for(int j = 0; j < inMap.count(); ++j){
            if(i != j) {
                intersectionListFormimgRoutine(currParrR, inMap[j][OrientedLine::PerpendiclSweepU], intersectionsR,
                                               QLineF::BoundedIntersection);
                intersectionListFormimgRoutine(currParrL, inMap[j][OrientedLine::PerpendiclSweepU], intersectionsL,
                                               QLineF::BoundedIntersection);
            }
        }
        QList<double> disR_list, disL_list;
        if(flag) {
            disR_list = distanceToPointRoutine(inMap[i][OrientedLine::ParallelSweepR].p1(), intersectionsR);
            disL_list = distanceToPointRoutine(inMap[i][OrientedLine::ParallelSweepL].p1(), intersectionsL);
        }else{
            disR_list = distanceToPointRoutine(inMap[i][OrientedLine::ParallelSweepR].p2(), intersectionsR);
            disL_list = distanceToPointRoutine(inMap[i][OrientedLine::ParallelSweepL].p2(), intersectionsL);
        }

        auto resIdxR = sort_indexes<double>(disR_list);
        auto resIdxL = sort_indexes<double>(disL_list);
        if(flag) {
            if (intersectionsR.count() >= 1)
                inMap[i][OrientedLine::ParallelSweepR].setP1(intersectionsR[resIdxR[0]]);
            if (intersectionsL.count() >= 1)
                inMap[i][OrientedLine::ParallelSweepL].setP1(intersectionsL[resIdxL[0]]);
        }else{
            if (intersectionsR.count() >= 1)
                inMap[i][OrientedLine::ParallelSweepR].setP2(intersectionsR[resIdxR[0]]);
            if (intersectionsL.count() >= 1)
                inMap[i][OrientedLine::ParallelSweepL].setP2(intersectionsL[resIdxL[0]]);
        }
    }
}

// not finalized yet
QList<QPolygonF> Decomposer::boustrophedonDecomposition(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                        const QList<QMap<OrientedLine, QLineF>>& mapOriendtedHoleRectLines,
                                                        double sweepAngle) {
    QList<QPolygonF> resCells{};

    QList<QMap<OrientedLine, QLineF>> copy = mapOriendtedHoleRectLines;
    resCells = boustrophedonDecomposition_compact(polygon, holes, copy, sweepAngle);

    // modify first 2 * holes.count cells of compact func resCell
    // сделаем чтобы все зоны строго были поделены для дальнейшего path cover
    auto check = _updateCellRule(&holes, &resCells);
//    std::cout << "Rule is " << check << std::endl;

    int h1, h2;
    QList<double> square;
    for(int i = 0; i < holes.count(); ++i){
        square.append(polygonArea(holes[i]));
    }
    auto hs = sort_indexes<double>(square);
    h1 = hs[1];
    h2 = hs[0];

    bool flag1 = false, flag3 = false;
    if(check == 2)
        updateOrientedLine2(copy, h1, h2);
    else
        if(check == 1) {
            flag1 = updateOrientedLine1(copy, h1, h2);
            flag3 = updateOrientedLine3(copy, h1, h2);
        }
//    std::cout << "[boustrophedonDecomposition] here " <<  check << std::endl;

    QPolygonF buff;
    if(check > 0) {
        // ------------ решение для зон с U и D ----------------
        int j = 0;
//        resCells.clear();
        for (int i = 0; i < copy.count(); ++i) {
            buff.clear();
            buff << copy[i][OrientedLine::PerpendiclSweepU].p1()
                 << copy[i][OrientedLine::PerpendiclSweepU].p2()
                 << copy[i][OrientedLine::ParallelSweepL].p2()
                 << copy[i][OrientedLine::ParallelSweepR].p2();
            buff = sortPolygonClockwise(buff);
//            resCells.append(buff);
            resCells.replace(j, buff);
            buff.clear();
            buff << copy[i][OrientedLine::PerpendiclSweepD].p1()
                 << copy[i][OrientedLine::PerpendiclSweepD].p2()
                 << copy[i][OrientedLine::ParallelSweepL].p1()
                 << copy[i][OrientedLine::ParallelSweepR].p1();
            buff = sortPolygonClockwise(buff);
//            resCells.append(buff);
            resCells.replace(j+1, buff);
            j += 2;
        }
        if (check == 1)
        {
            // допом ещё cell
            if(flag1) {
                buff.clear();
                buff << copy[0][OrientedLine::PerpendiclSweepU].p1()
                     << copy[1][OrientedLine::PerpendiclSweepU].p2()
                     << copy[1][OrientedLine::PerpendiclSweepD].p2()
                     << copy[0][OrientedLine::PerpendiclSweepD].p1();
                //            buff = sortPolygonClockwise(buff);
                resCells.append(buff);
            }
            if(flag3){
                buff.clear();
                buff << copy[1][OrientedLine::PerpendiclSweepU].p1()
                     << copy[0][OrientedLine::PerpendiclSweepU].p2()
                     << copy[0][OrientedLine::PerpendiclSweepD].p2()
                     << copy[1][OrientedLine::PerpendiclSweepD].p1();
                //            buff = sortPolygonClockwise(buff);
                resCells.append(buff);
            }
        }else{
            //TODO определить что лучше polygon operation или construction
            if(check == 2){
                if(h2 > h1) {
                    resCells.removeAt(0);
                    buff.clear();
                    buff << copy[h1][OrientedLine::PerpendiclSweepU].p1()
                         << copy[h2][OrientedLine::ParallelSweepL].p1()
                         << copy[h2][OrientedLine::ParallelSweepL].p2()
                         << copy[h1][OrientedLine::ParallelSweepL].p2();
                    resCells.prepend(buff);
                    buff.clear();
                    buff << copy[h1][OrientedLine::PerpendiclSweepU].p2()
                         << copy[h2][OrientedLine::ParallelSweepR].p1()
                         << copy[h2][OrientedLine::ParallelSweepR].p2()
                         << copy[h1][OrientedLine::ParallelSweepR].p2();
                    resCells.prepend(buff);
                }else{
                    resCells.removeAt(3);
                    buff.clear();
                    buff << copy[h1][OrientedLine::PerpendiclSweepU].p2()
                         << copy[h2][OrientedLine::ParallelSweepR].p2()
                         << copy[h2][OrientedLine::ParallelSweepR].p1()
                         << copy[h1][OrientedLine::ParallelSweepR].p1();
                    buff = sortPolygonClockwise(buff);
                    resCells.prepend(buff);
                    buff.clear();
                    buff << copy[h1][OrientedLine::PerpendiclSweepU].p1()
                         << copy[h2][OrientedLine::ParallelSweepL].p2()
                         << copy[h2][OrientedLine::ParallelSweepL].p1()
                         << copy[h1][OrientedLine::ParallelSweepL].p1();
                    buff = sortPolygonClockwise(buff);
                    resCells.prepend(buff);
                }
            }
        }
    }

    return resCells;
}

void Decomposer::feedHolesInfoIn()
{
    int holeCount = 0;
    for(int i = 0; i < 2*m_holes.count(); i+=2){
        QPair<QPolygonF*,QPolygonF*> holeDataFeed1;
        holeDataFeed1.first     = &m_bpd_decompositionCells[i];
        holeDataFeed1.second    = &m_bpd_decompositionCells[i+1];
        _holeData.holeToBCD[&m_holes[holeCount]] = holeDataFeed1;
        ++holeCount;
    }
    //остальные зоны без QPolygonF holes
    //их просто можно взять из m_bpd_decompositionCells начиная с индекса 2*m_holes.count()

    //формируем holeBorderSegm
    /*QPair<borderLine,borderLine> buff;
    borderLine buffL;
    for(int i = 0; i < m_holes.count(); ++i) {

    }*/
}

QList<QPolygonF> Decomposer::getOrientedBoundingHoleRects(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                          double angleDegrees) {
    m_mapOriendtedHoleRectLines.clear();
    QList<QPolygonF> orientedRects;

    if (polygon.size() < 3) {
        return QList<QPolygonF>{};
    }
    if (holes.count() < 1){
        return QList<QPolygonF>{};
    }

    QMap<OrientedLine, QLineF> buff;
    for(const auto& currHole: holes) { // зон U и D в два раза больше holes
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
    /*QPolygonF oneHole;
    oneHole << QPointF(180, 180)
            << QPointF(270, 180)
            << QPointF(300, 225)
            << QPointF(270, 270)
            << QPointF(180, 270)
            << QPointF(150, 225);
    m_holes.append(oneHole);
    QPolygonF secHole;
    *//*secHole << QPointF(300, 350)
            << QPointF(350, 350)
            << QPointF(350, 300)
            << QPointF(300, 300);*//*
    secHole << QPointF(289.64, 325.00)
            << QPointF(325.00, 360.36)
            << QPointF(360.36, 325.00)
            << QPointF(325.00, 289.64);
    m_holes.append(secHole);*/
    QPolygonF secHole;
    secHole << QPointF(289.64 - 100, 325.00 - 100)
            << QPointF(325.00 - 100, 360.36 - 100)
            << QPointF(360.36 - 100, 325.00 - 100)
            << QPointF(325.00 - 100, 289.64 - 100);
    m_holes.append(secHole);
    QPolygonF oneHole;
    oneHole << QPointF(180 + 100, 180 + 100)
            << QPointF(270 + 100, 180 + 100)
            << QPointF(300 + 100, 225 + 100)
            << QPointF(270 + 100, 270 + 100)
            << QPointF(180 + 100, 270 + 100)
            << QPointF(150 + 100, 225 + 100);
    m_holes.append(oneHole);
}

bool Decomposer::showPathCoverage() const {
    return _isPathShow;
}

void Decomposer::setShowPathCoverage(bool in) {
    if(_isPathShow == in)
        return;

    _isPathShow = in;
    emit showPathCoverageChanged();
}

void Decomposer::setTransectWidth(double w) {
    if (qFuzzyCompare(_trWidth, w))
        return;

    _trWidth = w;
    transects()->setTransectWidth(_trWidth);
//    _transects->pathUpdation();
    emit trWdthChanged();
}

double Decomposer::transectWidth() const {
    return _trWidth;
}

QList<QPolygonF>
Decomposer::performDecomposition(const QPolygonF &polygon, const QList<QPolygonF> &holes, double sweepAngle) {
//    m_mapOriendtedHoleRectLines.clear();
    m_orientedRect.clear();
    m_bpd_decompositionCells.clear();
    QMap<OrientedLine, QLineF> buff = {};
    m_orientedRect = getOrientedBoundingRect(polygon, buff, sweepAngle);
    auto res = boustrophedonDecomposition(polygon,  holes, m_mapOriendtedHoleRectLines, sweepAngle);
    /*res.append(m_orientedRect);
    _debugPolyListToConsole(res);
    res.pop_back();*/
    return res;
}

void Decomposer::_debugPolyListToConsole(const QList<QPolygonF>& polygons) {
    QDebug dbg = qDebug().nospace();
    dbg << "=== Polygon List Debug ===\n";
    dbg << "Total polygons: " << polygons.size() << "\n";

    for (int i = 0; i < polygons.size(); ++i) {
        const QPolygonF& poly = polygons[i];
        dbg << "\nPolygon #" << i
            << " (points: " << poly.size() << "):\n";

        for (int j = 0; j < poly.size(); ++j) {
            dbg << "  " << j << ": ("
                << poly[j].x() << ", "
                << poly[j].y() << ")\n";
        }

        // Дополнительная информация
        if (!poly.isEmpty() && poly.isClosed()) {
            dbg << "  Closed polygon";
        }
    }
    dbg << "\n=========================\n";
}
