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
    , _pb()
    , _isPathShow(false)
    , _trWidth(45)
{
    _init();
    //создание созависимых сперва
    _transects = new PathGenerator(_trWidth, m_sweepAngle, this);
//    connect(this, &Decomposer::sweepAngleChanged, _transects, &PathGenerator::pathUpdation);
    connect(this, &Decomposer::originalPolygonChanged, this, [this] (){
        _transects->setSurvPoly(m_originalPolygon);
    });
    connect(this, &Decomposer::holesPolygonsChanged, this, [this] (){
        _transects->setPolyHolesList(m_holes);
        _transects->changeHolesActiveState();
    });
    
    //после определяем остальное
//    createDefaultPolygon();
    createPolygonWithHoles();

    //выполняем предобработку holes (граничные случаи)
    preprocPolys(m_originalPolygon, m_holes);

    updateDecomposition();
}

Decomposer::Decomposer(double pathSpace, double angle, QObject *parent)
        : QObject(parent)
        , m_sweepAngle(angle)
        , m_showDecomposition(false)
        , m_showOrientedRect(false)
        , _pb()
        , _isPathShow(false)
        , _trWidth(pathSpace)
{
    _init();
    //создание созависимых сперва
    _transects = new PathGenerator(_trWidth, m_sweepAngle, this);
//    connect(this, &Decomposer::sweepAngleChanged, _transects, &PathGenerator::pathUpdation);
    connect(this, &Decomposer::originalPolygonChanged, this, [this] (){
        _transects->setSurvPoly(m_originalPolygon);
    });
    connect(this, &Decomposer::holesPolygonsChanged, this, [this] (){
        _transects->setPolyHolesList(m_holes);
        _transects->changeHolesActiveState();
    });
}

void Decomposer::_init(){
    connect(this , &Decomposer::originalPolygonChanged,     this, &Decomposer::updateCanvas);
    connect(this , &Decomposer::decompositionCellsChanged,  this, &Decomposer::updateCanvas);
    connect(this , &Decomposer::orientedRectChanged,        this, &Decomposer::updateCanvas);
    connect(this , &Decomposer::showDecompositionChanged,   this, &Decomposer::updateCanvas);
    connect(this , &Decomposer::showOrientedRectChanged,    this, &Decomposer::updateCanvas);
    connect(this , &Decomposer::sweepAngleChanged,          this, &Decomposer::updateCanvas);
    connect(this , &Decomposer::trWdthChanged,              this, &Decomposer::updateCanvas);
    connect(this , &Decomposer::showPathCoverageChanged,    this, &Decomposer::updateCanvas);
}

QList<QList<QPointF>> Decomposer::config(const QPolygonF& survPoly, const QList<QPolygonF>& holes)
{
    m_originalPolygon = survPoly;
    if(!holes.isEmpty()) {
        m_holes = holes;
        preprocPolys(m_originalPolygon, m_holes);
        if(m_holes.count())
            emit holesPolygonsChanged();
    }

    emit originalPolygonChanged();
    updateDecomposition();
    return _transects->getPath();
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
    auto currKindCells = _decmpsKind ? m_bpd_decompositionCells : m_decompositionCells;
    for (const QPolygonF &cell : currKindCells) {
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
    m_bpd_decompositionCells.clear();

    if (m_originalPolygon.size() < 3) {
        emit decompositionCellsChanged();
        emit orientedRectChanged();
        return;
    }

    // Вычисляем ориентированный ограничивающий прямоугольник
    m_orientedRect = getOrientedBoundingRect(m_originalPolygon, m_sweepAngle);
    m_orientedHoleRects = getOrientedBoundingHoleRects(m_originalPolygon, m_holes, m_sweepAngle);

    // Выполняем трапецоидальную декомпозицию
    m_decompositionCells = trapezoidalDecomposition(m_originalPolygon, m_sweepAngle);

    // Выполняем бустрофедон декомпозицию
    m_bpd_decompositionCells = boustrophedonDecomposition(m_originalPolygon, m_holes, m_sweepAngle);

    _transects->setPolyBoundary(m_orientedRect);
    _transects->setPathSegments(m_bpd_decompositionCells);
    _transects->pathUpdation();

    emit decompositionCellsChanged();
    emit orientedRectChanged();
}

void Decomposer::preprocPolys(QPolygonF& survPoly, QList<QPolygonF>& holes){
    // проверяем не наползает ли holes один на другой
    PolyBuilder pb = PolyBuilder();
    QList<QPolygonF> resHoles = {};
    resHoles.append(holes[0]);
    for(int i = 1; i < holes.count(); ++i) {
        QList<QPolygonF> merged;
        QPolygonF currentHole = holes[i];

        for (int j = 0; j < resHoles.count(); ++j) {
            auto resIntersect = pb.snglIntersctnWrp(resHoles[j], currentHole);
            if (!resIntersect.isEmpty()) {
                // накопленный результатом в случае пересечений
                currentHole = pb.unitedListWrp(QList{resHoles[j], currentHole}).at(0);
            } else {
                merged.append(resHoles[j]);
            }
        }

        merged.append(currentHole);
        resHoles = merged;
    }
    holes = resHoles;
    resHoles.clear();
    // проверяем не наползает ли holes на survPoly и в границах ли он
    for(int i = 0; i < holes.count(); ++i){
        int containsEdgePointCount = 0;
        for(const auto& currHoleP: holes[i])
            if(survPoly.containsPoint(currHoleP, Qt::OddEvenFill))
                ++containsEdgePointCount;
        // корректируем survPoly и убираем тот hole
        if(containsEdgePointCount == holes[i].count())
            resHoles.append(holes[i]);
        else
            survPoly = pb.subtractedListWrp(survPoly, holes[i]).at(0);

    }
    for(auto& currHole: resHoles){
        if(currHole.count() > 0) {
            currHole = pb.offsetWrp(currHole, pb.getScale() * 1.05);
            // открываем полигон (удаляем последнюю p которая = первой)
            currHole.pop_back();
        }
    }
    holes = std::move(resHoles);
}

void Decomposer::resetPolygon(int idx) {
    createPolygonWithHoles(idx);
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
QList<QPolygonF> Decomposer::trapezoidalDecomposition(const QPolygonF& polygon, double sweepAngle) {
    QList<QPolygonF> cells;

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

template <typename T>
T Decomposer::rotationStruct(const T& v, double sweepAngle) {
    T res = v;
    for (QPointF& point : res) {
        point = rotatePoint(point, sweepAngle);
    }
    return res;
}

QPointF Decomposer::updateCenterBeforeRot(QPolygonF& poly)
{
    QPointF centroid(0, 0);

    for (const QPointF& p : poly)
        centroid += p;
    centroid /= poly.size();

    for (QPointF& p : poly)
        p -= centroid;

    return centroid;
}

// реализация деления полигона с отверстиями на зоны
// задел для path construcion алгоритма туда-обратно
QList<QPolygonF> Decomposer::boustrophedonDecomposition_compact(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                                double sweepAngle)
{
    QList<QPolygonF> resCells{};
    QList<QPolygonF> edgeOp{}; // нужен доп для внесения крайних зон (без этого пока никак)

    if (polygon.size() < 3) {
        return resCells;
    }
    if (holes.count() < 1){
        return resCells;
    }

    // ------------------------ внесение зон связанных с holes ------------------------
    // поворачиваем полигон SurvPoly
    auto rot = m_orientedRect;
    auto center = updateCenterBeforeRot(rot);
    auto rotatedSurvPolyPolygon = rotationStruct<QPolygonF>(rot, sweepAngle);
    for(auto& p : rotatedSurvPolyPolygon)
        p += center;

    auto rotationRoutine = [this] (QPolygonF& poly, QPointF& currCenter, double angle) {
        for (QPointF& p : poly)
            p -= currCenter;
        poly = rotationStruct<QPolygonF>(poly, angle);
        for(auto& p : poly)
            p += currCenter;
    };

    for(auto& currOrientHole: m_orientedHoleRects)
    {
        double xSurvLeft = rotatedSurvPolyPolygon.boundingRect().left();
        double xSurvRight = rotatedSurvPolyPolygon.boundingRect().right();
        auto currCenter = updateCenterBeforeRot(currOrientHole);
        currOrientHole = rotationStruct<QPolygonF>(currOrientHole, sweepAngle);
        for(auto& p : currOrientHole)
            p += currCenter;

        auto tempRotCenter = currCenter - center;
        tempRotCenter = rotatePoint(tempRotCenter,sweepAngle);
        tempRotCenter += center;
        xSurvLeft   -= tempRotCenter.x() - currCenter.x();
        xSurvRight  += currCenter.x() - tempRotCenter.x();

        // монолитный полигольник для hole по выходу из цикла вырежем hole
        QPolygonF single;
        single  << QPointF(xSurvRight, currOrientHole.boundingRect().bottom())
                << QPointF(xSurvRight, currOrientHole.boundingRect().top())
                << QPointF(xSurvLeft, currOrientHole.boundingRect().top())
                << QPointF(xSurvLeft, currOrientHole.boundingRect().bottom());

        rotationRoutine(single, currCenter, -sweepAngle);
        resCells.append(single);

        // заполняем допник
        QPolygonF up, down;
        up      << QPointF(xSurvRight, currOrientHole.boundingRect().bottom())
                << QPointF(xSurvRight, currOrientHole.boundingRect().top())
                << QPointF(currCenter.x(), currOrientHole.boundingRect().top())
                << QPointF(currCenter.x(), currOrientHole.boundingRect().bottom());
        down    << QPointF(currCenter.x(), currOrientHole.boundingRect().bottom())
                << QPointF(currCenter.x(), currOrientHole.boundingRect().top())
                << QPointF(xSurvLeft, currOrientHole.boundingRect().top())
                << QPointF(xSurvLeft, currOrientHole.boundingRect().bottom());

        rotationRoutine(up, currCenter, -sweepAngle);
        rotationRoutine(down, currCenter, -sweepAngle);
        edgeOp.append(up);
        edgeOp.append(down);
    }

    // ------------------------ вырезаем hole -------------------------------
    QList<QPolygonF> extra = {};
    for(int i = 0; i < m_holes.count(); ++i)
    {
        auto substract = _pb.subtractedListWrp(resCells[i], m_holes[i]);
        for(int k = 0; k < substract.count() && polygonsAreDifferent(substract[0], resCells[i]); ++k)
            if(!substract[k].empty())
                extra.append(substract[k]);
    }
    resCells = std::move(extra);

    // ------------------------ внесение крайних зон и в промежутках с использованием clipper EdgeOp -------------------------------
    QList<QPolygonF> whole;
    whole = _pb.unitedListWrp(edgeOp);
    auto listEdgesCells = _pb.subtractedListWrp(m_orientedRect, whole);
    for(const auto& curr: listEdgesCells){
        resCells.append(curr);
    }

    return resCells;
}

QList<QPolygonF> Decomposer::boustrophedonDecomposition(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                        double sweepAngle) {
    QList<QPolygonF> resCells{};
    resCells = boustrophedonDecomposition_compact(polygon, holes, sweepAngle);

    if(resCells.isEmpty())
        return resCells;

    // Автоматическое определение пересечений и обработка
    int idxCellForHole1, idxCellForHole2;
    for(int i = 0; i < holes.count(); ++i) // каждая holes анализир
        for (int j = 0; j < holes.count(); ++j) // с другой holes
            if(i != j)
                // у каждой holes 2 cells в resCells = boustrophedonDecomposition_compact были посл положены в начале массива
                for(int u = 0; u < 2; ++u){
                    idxCellForHole1 = i * 2 + u;
                    // Проверяем все угловые точки отверстия как в cell от hole(i) входит hole(j)
                            // Вырезаем отверстие из ячейки
                            QList<QPolygonF> subtracted;
                            bool rule = polygonArea(_pb.snglIntersctnWrp(resCells[j * 2], holes[i], true)) != 0;
                            subtracted = _pb.subtractedListWrp(resCells[idxCellForHole1],
                                                               rule ? resCells[j * 2 + 1] : resCells[j * 2]);
                            if (!subtracted.isEmpty()) {
                                // Заменяем исходную ячейку результатом вычитания
                                resCells[idxCellForHole1] = subtracted.first();

                                for (int k = 1; k < subtracted.count(); ++k)
                                    resCells.append(subtracted[k]);
                            }
                }
    //удаляем лишние наложения на holes в рез прошлых операций и вписываем в границы survPoly
    QList<QPolygonF> extra = {};
    for(auto & currCell: resCells)
    {
        for (const auto& currHole:m_holes)
        {
            auto res = _pb.subtractedListWrp(currCell, currHole);
            if(!res.isEmpty()) {
                currCell = res.first();
                for (int k = 1; k < res.count(); ++k)
                    extra.append(res[k]);
            }
        }
    }
    for(const auto& curr:extra)
        if(!resCells.contains(curr))
            resCells.append(curr);
    for(auto & currCell: resCells)
        currCell = _pb.snglIntersctnWrp(currCell, m_originalPolygon);

    // удалить накладывающиеся друг на друга зоны
    for(int i = 0; i < resCells.count(); ++i)
        for (int j = 0; j < resCells.count(); ++j) {
                    // Получаем текущие значения
                    QPolygonF cell1 = resCells[i];
                    QPolygonF cell2 = resCells[j];

                    // Если cell2 внутри cell1, вычитаем cell2 из cell1
                    auto subtracted = _pb.subtractedListWrp(cell1, cell2);
                    if (!subtracted.isEmpty()) {
                        // Обновляем cell1 результатом вычитания
                        resCells[i] = subtracted.first();
                        for(int k = 1; k < subtracted.count(); ++k)
                            resCells.append(subtracted);
                    }
        }

    // hit me one more time:) иногда проскакивают дубли, убираем
    QList<QPolygonF> resCells2{};
    for(const auto& cell : resCells) {
        auto it = std::find_if(resCells2.begin(), resCells2.end(),
                               [&cell](const QPolygonF& existing) {
                                   return !polygonsAreDifferentV2(cell, existing);
                               });

        if(it == resCells2.end())
            resCells2.append(cell);
    }

    return resCells2;
}

QList<QPolygonF> Decomposer::getOrientedBoundingHoleRects(const QPolygonF& polygon, const QList<QPolygonF>& holes,
                                                          double angleDegrees) {
    QList<QPolygonF> orientedRects;

    if (polygon.size() < 3) {
        return QList<QPolygonF>{};
    }
    if (holes.count() < 1){
        return QList<QPolygonF>{};
    }

    for(const auto& currHole: holes) { // зон U и D в два раза больше holes
        QPolygonF currOrieRect = getOrientedBoundingRect(currHole, angleDegrees);
        orientedRects.append(currOrieRect);
    }

    return orientedRects;
}

// Ориентированный ограничивающий прямоугольник
QPolygonF Decomposer::getOrientedBoundingRect(const QPolygonF& polygon, double angleDegrees) {
    if (polygon.size() < 3) {
        return QPolygonF{};
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
    emit originalPolygonChanged();
}

void Decomposer::createPolygonWithHoles(int idx) {
    m_originalPolygon.clear();
    m_holes.clear();
    // Шестиугольник
    m_originalPolygon << QPointF(200, 100)
                      << QPointF(400, 100)
                      << QPointF(500, 250)
                      << QPointF(400, 400)
                      << QPointF(200, 400)
                      << QPointF(100, 250);
    if(idx == 0) {
        QPolygonF oneHole;
        oneHole << QPointF(180, 180)
                << QPointF(270, 180)
                << QPointF(300, 225)
                << QPointF(270, 270)
                << QPointF(180, 270)
                << QPointF(150, 225);
        m_holes.append(oneHole);
        QPolygonF secHole;
        secHole << QPointF(389.64, 425.00 - 200)
                << QPointF(425.00, 460.36 - 200)
                << QPointF(460.36, 425.00 - 200)
                << QPointF(425.00, 389.64 - 200);
        QPolygonF thirdHole;
        thirdHole << QPointF(200, 400 - 50)
                  << QPointF(250, 400 - 50)
                  << QPointF(250, 350 - 50)
                  << QPointF(200, 350 - 50);
        m_holes.append(secHole);
        m_holes.append(thirdHole);
    }
    else if(idx == 1) {
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
    else if(idx == 2) {
        QPolygonF oneHole;
        oneHole << QPointF(389.64 - 200, 425.00 - 200)
                << QPointF(425.00 - 200, 460.36 - 200)
                << QPointF(460.36 - 200, 425.00 - 200)
                << QPointF(425.00 - 200, 389.64 - 200);
        QPolygonF secHole;
        secHole << QPointF(389.64, 425.00 - 200)
                << QPointF(425.00, 460.36 - 200)
                << QPointF(460.36, 425.00 - 200)
                << QPointF(425.00, 389.64 - 200);
        QPolygonF thirdHole;
        thirdHole << QPointF(389.64 - 100, 425.00 - 200)
                  << QPointF(425.00 - 100, 460.36 - 200)
                  << QPointF(460.36 - 100, 425.00 - 200)
                  << QPointF(425.00 - 100, 389.64 - 200);
        m_holes.append(oneHole);
        m_holes.append(secHole);
        m_holes.append(thirdHole);
    }
    emit originalPolygonChanged();
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

bool Decomposer::decmpsKind() const {
    return _decmpsKind;
}

void Decomposer::setDecmpsKind(bool in) {
    _decmpsKind = in;
}
