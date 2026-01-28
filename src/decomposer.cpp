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

    if (m_originalPolygon.size() < 3) {
        emit decompositionCellsChanged();
        emit orientedRectChanged();
        return;
    }

    //выполняем предобработку holes (граничные случаи)
//    preprocPolys(m_originalPolygon, m_holes);

    // Вычисляем ориентированный ограничивающий прямоугольник
    QMap<OrientedLine, QLineF> buff = {};
    m_orientedRect = getOrientedBoundingRect(m_originalPolygon, buff, m_sweepAngle);
    m_orientedHoleRects = getOrientedBoundingHoleRects(m_originalPolygon, m_holes, m_sweepAngle);

    // Выполняем трапецоидальную декомпозицию
    std::vector<QPolygonF> cells = trapezoidalDecomposition(m_originalPolygon, m_sweepAngle);
    m_decompositionCells = QVector<QPolygonF>::fromStdVector(cells);

    // Выполняем бустрофедон декомпозицию
    m_bpd_decompositionCells = boustrophedonDecomposition(m_originalPolygon, m_holes, m_mapOriendtedHoleRectLines, m_sweepAngle);

    /*m_bpd_decompositionCells.append(m_orientedRect);
    _debugPolyListToConsole(m_bpd_decompositionCells);
    m_bpd_decompositionCells.pop_back();*/

    _transects->setPolyBoundary(m_orientedRect);
    _transects->setPathSegments(m_bpd_decompositionCells);
    _transects->pathUpdation();

    emit decompositionCellsChanged();
    emit orientedRectChanged();
}

void Decomposer::preprocPolys(QPolygonF& survPoly, QList<QPolygonF>& holes){
    // проверяем не наползает ли holes один на другой
    PolyBuilder pb = PolyBuilder();
    QList<QPolygonF> resHoles;
    resHoles = holes;
    for(int i = 0; i < holes.count(); ++i) {
        for (int j = resHoles.count() - 1; j > i; --j) {
            auto resIntersect = pb.snglIntersctnWrp(resHoles[j], holes[i]);
            if(!resIntersect.isEmpty()) {
                auto temp = QList<QPolygonF>{};
                temp.append(resHoles[j]);
                temp.append(holes[i]);
                resHoles.pop_back(); // удаляем как отдельную сущ эту hole
                resHoles.replace(i, pb.unitedListWrp(temp).at(0));
            }
        }
    }
    holes = resHoles;
    resHoles.clear();
    // проверяем не наползает ли holes на survPoly и в границах ли он
    // todo обойти этот случай аккуратнее
    for(int i = 0; i < holes.count(); ++i){
        int containsEdgePointCount = 0;
        for(const auto& currHoleP: holes[i])
            if(survPoly.containsPoint(currHoleP, Qt::OddEvenFill))
                ++containsEdgePointCount;
        if(containsEdgePointCount >= holes[i].count() - 1){ // случай что почти весь hole внутри survPoly (корректируем hole)
            resHoles.append(pb.snglIntersctnWrp(holes[i], survPoly));
        }else{
            // корректируем survPoly и убираем тот hole
            if(containsEdgePointCount == 0)
                continue;
            else
                survPoly = pb.subtractedListWrp(survPoly, holes[i]).at(0);
        }
    }
    holes = resHoles;
    for(auto& currHole: holes){
        if(currHole.count() > 0) {
            currHole = pb.offsetWrp(currHole, pb.getScale() * 1.05);
            // открываем полигон (удаляем последнюю p которая = первой)
            currHole.pop_back();
        }
    }
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
    if(intersectionsL_list.count() >= 1 && intersectionsR_list.count() >= 1) {
        returnUp    = QLineF(intersectionsL_list[resIdxL[0]], intersectionsR_list[resIdxR[0]]);
        returnDown  = QLineF(intersectionsL_list[resIdxL[resIdxL.size() - 1]],
                            intersectionsR_list[resIdxR[resIdxR.size() - 1]]);
    }
    else{
        returnUp    = QLineF{0,0,0,0};
        returnDown  = QLineF{0,0,0,0};
    }
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
    if(resIntersectionsL.count() >= 2)
        returnL = QLineF(resIntersectionsL[0], resIntersectionsL[1]);
    else
        returnL = QLineF{0,0,0,0};
    if(resIntersectionsR.count() >= 2)
        returnR = QLineF(resIntersectionsR[0], resIntersectionsR[1]);
    else
        returnL = QLineF{0,0,0,0};
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
    QList<QPolygonF> resCells{};

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
        // Организация данных в новую структуру схожую с mapOriendtedHoleRectLines
        // 0. Preprocessing получаем линии Parallel L и R длинные до пересечений boundedRect SurvPoly
        newParallFormingRoutine(mapOriendtedHoleRectLines[i], copySurvPolyBound,
                                copy[i][OrientedLine::ParallelSweepL],
                                copy[i][OrientedLine::ParallelSweepR]);
        if(copy[i][OrientedLine::ParallelSweepL].isNull() || copy[i][OrientedLine::ParallelSweepR].isNull())
            return resCells;
        // ----------------------- для верха и низа -----------------------------
        newBorderFormingRoutine(mapOriendtedHoleRectLines[i], holes[i],
                                copy[i][OrientedLine::PerpendiclSweepU],
                                copy[i][OrientedLine::PerpendiclSweepD]);
        if(copy[i][OrientedLine::PerpendiclSweepU].isNull() || copy[i][OrientedLine::PerpendiclSweepD].isNull())
            return resCells;
        // 1. И формируется новые U и D границы
        lineHoleUpDownBorderFormingRoutineNewManner(copy[i], holes[i],
                                                    readyPolyUp[i],
                                                    readyPolyDown[i],
                                                    holeBorderUp[i],
                                                    holeBorderDown[i]);
        // 2. В compact реализации 2 и 3 пункт пропускаем
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
    QList<QPolygonF> whole;
    for(auto& curr: resCells){
        curr.append(curr[0]);
    }
    whole = _pb.unitedListWrp(resCells);
    auto listEdgesCells = _pb.subtractedListWrp(m_orientedRect, whole);
//    std::cout << "Size of listEdgesCells is " << listEdgesCells.count() << std::endl;
    for(const auto& curr: listEdgesCells){
        resCells.append(curr);
    }

    mapOriendtedHoleRectLines = copy;

//    std::cout << "Size of cell decmps is " << resCells.count() << std::endl;

    return resCells;
}

QList<QPolygonF> Decomposer::boustrophedonDecomposition(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                        const QList<QMap<OrientedLine, QLineF>>& mapOriendtedHoleRectLines,
                                                        double sweepAngle) {
    QList<QPolygonF> resCells{};
    QList<QMap<OrientedLine, QLineF>> copy = mapOriendtedHoleRectLines;
    resCells = boustrophedonDecomposition_compact(polygon, holes, copy, sweepAngle);

    // Автоматическое определение пересечений и обработка
    int idxCellForHole1, idxCellForHole2;
    for(int i = 0; i < holes.count(); ++i) // каждая holes анализир
        for (int j = 0; j < holes.count(); ++j) // с другой holes
            if(i != j)
                // у каждой holes 2 cells в resCells = boustrophedonDecomposition_compact были посл положены в начале массива
                for(int u = 0; u < 2; ++u){
                    idxCellForHole1 = i * 2 + u;
                    auto holecell = _pb.snglIntersctnWrp(resCells[idxCellForHole1], holes[j]);
                    // Проверяем все угловые точки отверстия как в cell от hole(i) входит hole(j)
                    for (int w = 0; w < 2; ++w) {
                        idxCellForHole2 = j * 2 + w;
                        if(polygonArea(holecell) != 0) {
                            // Вырезаем отверстие из ячейки
                            auto subtracted = _pb.subtractedListWrp(resCells[idxCellForHole1],
                                                                    resCells[idxCellForHole2]);
                            if (!subtracted.isEmpty()) {
                                // Заменяем исходную ячейку результатом вычитания
                                resCells[idxCellForHole1] = subtracted[0];

                                for (int k = 1; k < subtracted.count(); ++k)
                                    resCells.append(subtracted[k]);
                            }
                        }
                    }
                }

    // удалить накладывающиеся друг на друга зоны
    for(int i = 0; i < holes.count(); ++i) {
        for (int j = 0; j < holes.count(); ++j) {
            if (i == j) continue;  // Пропускаем сравнение с собой

            for (int u = 0; u < 2; ++u) {
                for (int w = 0; w < 2; ++w) {
                    idxCellForHole1 = i * 2 + u;
                    idxCellForHole2 = j * 2 + w;

                    // Получаем текущие значения
                    QPolygonF cell1 = resCells[idxCellForHole1];
                    QPolygonF cell2 = resCells[idxCellForHole2];

                    // Если cell2 внутри cell1, вычитаем cell2 из cell1
                    auto res = _pb.subtractedListWrp(cell1, cell2, false);
                    if (!res.isEmpty()) {
                        // Обновляем cell1 результатом вычитания
                        resCells[idxCellForHole1] = res[0];
                        for(const auto& currRes: res)
                            if(polygonArea(currRes) > polygonArea(resCells[idxCellForHole1]))
                                resCells[idxCellForHole1] = currRes;
                    }
                }
            }
        }
    }

    return resCells;
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
    QPolygonF oneHole;
    oneHole << QPointF(180, 180)
            << QPointF(270, 180)
            << QPointF(300, 225)
            << QPointF(270, 270)
            << QPointF(180, 270)
            << QPointF(150, 225);
    m_holes.append(oneHole);
    QPolygonF secHole;
    secHole << QPointF(389.64, 425.00-200)
            << QPointF(425.00, 460.36-200)
            << QPointF(460.36, 425.00-200)
            << QPointF(425.00, 389.64-200);
    QPolygonF thirdHole;
    thirdHole << QPointF(200, 400-50)
              << QPointF(250, 400-50)
              << QPointF(250, 350-50)
              << QPointF(200, 350-50);
    m_holes.append(secHole);
    m_holes.append(thirdHole);
    /*QPolygonF secHole;
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
    m_holes.append(oneHole);*/
    /*QPolygonF oneHole;
    oneHole << QPointF(389.64 - 200, 425.00-200)
            << QPointF(425.00 - 200, 460.36-200)
            << QPointF(460.36 - 200, 425.00-200)
            << QPointF(425.00 - 200, 389.64-200);
    QPolygonF secHole;
    secHole << QPointF(389.64, 425.00-200)
            << QPointF(425.00, 460.36-200)
            << QPointF(460.36, 425.00-200)
            << QPointF(425.00, 389.64-200);
    QPolygonF thirdHole;
    thirdHole << QPointF(389.64 - 100, 425.00-200)
            << QPointF(425.00- 100, 460.36-200)
            << QPointF(460.36- 100, 425.00-200)
            << QPointF(425.00- 100, 389.64-200);
    m_holes.append(oneHole);
    m_holes.append(secHole);
    m_holes.append(thirdHole);*/
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
Decomposer::performDecomposition(QPolygonF &polygon, QList<QPolygonF> &holes, double sweepAngle) {
//    m_mapOriendtedHoleRectLines.clear();
    m_orientedRect.clear();
    m_bpd_decompositionCells.clear();
    QMap<OrientedLine, QLineF> buff = {};
    //выполняем предобработку holes (граничные случаи)
    preprocPolys(polygon, holes);
    m_orientedRect = getOrientedBoundingRect(polygon, buff, sweepAngle);
    m_orientedHoleRects = getOrientedBoundingHoleRects(polygon, holes, sweepAngle);
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
