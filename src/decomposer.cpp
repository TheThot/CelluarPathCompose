//
// Created by Admin on 08.12.2025.
//
#include "decomposer.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <ostream>
#include <vector>

using namespace baseFunc;

Decomposer::Decomposer(QObject *parent)
    : QObject(parent)
    , m_sweepAngle(0.0)
    , m_showDecomposition(true)
    , m_showOrientedRect(true)
{
    //создание созависимых сперва
    _transects = new PathGenerator(45, m_sweepAngle, this);
    connect(this, &Decomposer::sweepAngleChanged, _transects, &PathGenerator::pathUpdation);
    connect(this, &Decomposer::originalPolygonChanged, this, [this] (){
        _transects->setSurvPoly(m_originalPolygon);
    });
    connect(this, &Decomposer::holesPolygonsChanged, this, [this] (){
        _transects->setPolyHolesList(m_holes);
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

    _transects->setPolyBoundary(m_orientedRect);
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
            intersectionListFormimgRoutine(level, edge, intersections, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
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
        inMap[orntL_i].intersects(inMap[orntL_k],
                                  &orientRectOverHole[static_cast<OrientPointNames>(k)]);
    }

    //определяем пересечения holes c ParallelSweepL и ParallelSweepR
    const auto currParallelOrientLineL = inMap[OrientedLine::ParallelSweepL];
    const auto currParallelOrientLineR = inMap[OrientedLine::ParallelSweepR];

    for(int i = 0; i < hole.count(); ++i){
        int k = (i + 1) % hole.count();
        QLineF currHoleLine = QLineF(hole[i], hole[k]);
        currHoleLine = extendLineBothWays(currHoleLine, 0.5);
        // для надёжности оба типа внутреннее и внешнее пересечения
//        intersectionListFormimgRoutine(currParallelOrientLineL, currHoleLine, intersectionsL_list, QLineF::UnboundedIntersection, maxBoundSurvPolyRad);
        intersectionListFormimgRoutine(currParallelOrientLineL, currHoleLine, intersectionsL_list, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
//        intersectionListFormimgRoutine(currParallelOrientLineR, currHoleLine, intersectionsR_list, QLineF::UnboundedIntersection, maxBoundSurvPolyRad);
        intersectionListFormimgRoutine(currParallelOrientLineR, currHoleLine, intersectionsR_list, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
    }
//    std::cout << "Count L intersections " << intersectionsL_list.count() << std::endl;
//    std::cout << "Count R intersections " << intersectionsR_list.count() << std::endl;
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
        intersectionListFormimgRoutine(parallelL, buffLine, resIntersectionsL, QLineF::UnboundedIntersection, maxBoundSurvPolyRad);
        intersectionListFormimgRoutine(parallelR, buffLine, resIntersectionsR, QLineF::UnboundedIntersection, maxBoundSurvPolyRad);
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
        intersectionListFormimgRoutine(level, edge, intersections, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
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
        intersectionListFormimgRoutine(level, edge, intersections, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
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
            intersectionListFormimgRoutine(firstPar, edge, intersections, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
            intersectionListFormimgRoutine(secPar, edge, intersections, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
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
            intersectionListFormimgRoutine(firstParRevers, edge, intersections, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
            intersectionListFormimgRoutine(secParRevers, edge, intersections, QLineF::BoundedIntersection, maxBoundSurvPolyRad);
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

bool Decomposer::showPathCoverage() const {
    return _isPathShow;
}

void Decomposer::setShowPathCoverage(bool in) {
    if(_isPathShow == in)
        return;

    _isPathShow = in;
    emit showPathCoverageChanged();
}