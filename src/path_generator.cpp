//
// Created by Admin on 19.12.2025.
//
#include <iostream>
#include <QDebug>
#include "path_generator.h"

using namespace baseFunc;

PathGenerator::PathGenerator(QObject* parent) :
                            QObject(parent)
{

}

PathGenerator::PathGenerator(double inStep, double inAngle, QObject *parent) :
                            QObject(parent),
                            _gridSpace(inStep),
                            _gridAngle(inAngle)
{
    pfc = new PathFinderCalculator();
//    _initNonRespectInnerHoles();
}

QList<QLineF> PathGenerator::_initNonRespectInnerHoles(const QPolygonF* inPoly)
{
    QList<QLineF> res = {};
    if(inPoly == nullptr) {
        std::cout << "[_initNonRespectInnerHoles] warning\n";
        return res;
    }

    // формируем полное покрытие полигона
    QPointF center;
    for(const auto& currP: *inPoly){
        center += currP;
    }
    center /= inPoly->size();
    QRectF bR = inPoly->boundingRect();

    QList<QLineF> lineList;
    double maxWidth = qMax(bR.width(), bR.height()) + 1e5;
    double halfWidth = maxWidth / 2.0;
    double transectX = center.x() - halfWidth;
    double transectXMax = transectX + maxWidth;
    while (transectX < transectXMax) {
        double transectYTop = center.y() - halfWidth;
        double transectYBottom = center.y() + halfWidth;

        lineList += QLineF(rotatePoint(QPointF(transectX, transectYTop), center, _gridAngle + 90),
                           rotatePoint(QPointF(transectX, transectYBottom), center, _gridAngle + 90));
        transectX += _gridSpace;
    }

    // Now intersect the lines with the polygon
    QList<QLineF> intersectLines;
    if(!_isHolesActive)
        intersectLinesWithPolygon(lineList, *inPoly, intersectLines);
    else
        intersectLinesWithPolygon(lineList, *_survPolygon, intersectLines, *inPoly, _holes);
    res = intersectLines;

    // убеждаемся что все линии в одном направлении
    QList<QLineF> res2 = {};
    _orientLineOneDirection(res, res2);

    return res2;
}

PathGenerator::~PathGenerator()
{
    _path.clear();
}

QVariantList PathGenerator::_oneLoopTraj(const QList<QList<QPointF>>& in) const
{
    QVariantList pathTraj;

    QVariantList row;
    for (const auto &lineS: in) {
        for (const auto &p: lineS) {
            QVariantMap pointMap;
            pointMap["x"] = p.x();
            pointMap["y"] = p.y();
            row.append(pointMap);
        }
        pathTraj.append(QVariant::fromValue(row));
        row.clear();
    }

    return pathTraj;
}

QVariantList PathGenerator::pathTraj() const
{
    QVariantList pathTraj;

    // определяем какой у нас активен режим ? от этого будет зависеть возвращаемое в qml значение
    if (!_isHolesActive) {
        pathTraj = _oneLoopTraj(_orientedPathSimpl);
    }
    else {
        auto iter = _pathIntoCell.begin();
        for(;iter != _pathIntoCell.end(); ++iter) {
            QVariantList temp = _oneLoopTraj(iter.value());
            pathTraj.append(QVariant::fromValue(temp));
        }
    }

    return pathTraj;
}

void PathGenerator::setGridAngle(double in)
{
    _gridAngle = in;
}

void PathGenerator::pathUpdation()
{
    _path.clear();
    _pathRespectHoles.clear();
    _pathIntoCell.clear();
    QVector<const QPolygonF*> order;
    _startEndPointsIntoCell.clear();
    _pathConnectionLines.clear();

    _path = _initNonRespectInnerHoles(_survPolygon);
    _orientedPathSimpl = _orientNonRespectPath(_path);

    if(_holes != nullptr) {
        std::cout << "[PathGenerator] _bpd_decompositionCells count is " << _bpd_decompositionCells->count() << std::endl;
        //pfc->init(*_holes);
        pfc->init(*_holes);
        for (int i = 0; i < _bpd_decompositionCells->count(); ++i) {
            auto res = _pathSegmRelationToCell(_bpd_decompositionCells->at(i));
            QList<QList<QPointF>> resPointList = _orientNonRespectPath(res);
            if (!resPointList.isEmpty())
                _pathIntoCell[&_bpd_decompositionCells->at(i)] = resPointList;

            /*if(resPointList.count() != 0)
                _pathRespectHoles += resPointList;*/
        }
        _configurePathIntoCell(order, _startEndPointsIntoCell);
        /*auto iter = _pathIntoCell.begin();
        while(iter != _pathIntoCell.end()) {
            _qDebugPrintPath(iter.value());
            ++iter;
        }*/
        //применяем astar для соединения между cell
        _pathIntoCell = _pathRouteCells(_pathIntoCell);
        _pathConnectionLines = _pathRouteConnections(_startEndPointsIntoCell);
        std::cout << "Connection count is " << _pathRespectHoles.size() << std::endl;
    }
}

QList<QList<QPointF>> PathGenerator::_pathRouteConnections(const QVector<QPair<QPointF, QPointF>>& inConnections){
    QList<QList<QPointF>> result;

    if (inConnections.isEmpty()) {
        return result;
    }

    // Обрабатываем переходы между зонами соединяем элементы старт и финиш двух зон
    for (int i = 0; i < inConnections.count() - 1; ++i) {

        QLineF temp = QLineF(inConnections[i].second, inConnections[i+1].first);

        pfc->perform(temp.p1(), temp.p2());
        auto connectionPath = pfc->getPath2d();

        if (!connectionPath.isEmpty()) {
            connectionPath = adaptiveSample(connectionPath, 40, 4);
            result.append(connectionPath);
        }

    }

    return result;
}

QHash< const QPolygonF*, QList<QList<QPointF>> > PathGenerator::_pathRouteCells(const QHash< const QPolygonF*, QList<QList<QPointF>> >& inPath){

    QHash< const QPolygonF*, QList<QList<QPointF>> > result;

    if (inPath.isEmpty()) {
        return result;
    }
    QHash< const QPolygonF*, QList<QLineF>> holesLines;

    for(const auto& currHole: *_holes){
        QList<QLineF> temp;
        for(int i = 0; i < currHole.count(); ++i){
            temp.append(QLineF(currHole[i], currHole[(i+1)%currHole.count()]));
        }
        holesLines[&currHole] = temp;
    }

    auto it = inPath.constBegin();

    // Обрабатываем остальные элементы
    for (; it != inPath.constEnd(); ++it) {
        QList<QList<QPointF>> configPathRes;
        const auto& currentPaths = it.value();

        if (currentPaths.isEmpty()) {
            std::cout << "Bad section path is empty\n";
            continue;
        }

        QList<QPointF> orderExtraPolyline;
        for(int i = 0; i < currentPaths.count()-1; ++i){
            QList<QPointF> buff;
            buff.append(currentPaths[i].first());
            QLineF betweenLines = QLineF(currentPaths[i].last(), currentPaths[i+1].first());
            for(const auto& currHole : holesLines)
                if(extraDangerPointsRoutine(currHole, betweenLines, orderExtraPolyline)) // добавляем точки полигона если они есть в местах пересечений чтобы траектория не проходила внутри holes
                    break;
            buff.append(orderExtraPolyline);
            configPathRes.append(buff);
        }
        configPathRes.append(QList<QPointF>{currentPaths.last().first(), currentPaths.last().last()});

        // Добавляем пути текущего элемента
        result[it.key()] = configPathRes;

    }

    return result;

}

QHash<const QPolygonF*, QList<QList<QPointF>>> PathGenerator::_configurePathIntoCell(
        QVector<const QPolygonF*>& order, QVector< QPair<QPointF, QPointF> >& flP)
{
    order.clear();
    flP.clear();
    QHash<const QPolygonF*, QList<QList<QPointF>>> res;
    if (_pathIntoCell.count() == 0)
    {
        std::cout << "Warning path hash is null\n";
        return res;
    }
    // формируем массив с расстояниями от начала одной до конца другой из qhash
    QVector<const QPolygonF*> allIn;
    for (auto itOne = _pathIntoCell.constBegin(); itOne != _pathIntoCell.constEnd(); ++itOne)
    {
        allIn.push_back(itOne.key());
    }
    auto stackIter = allIn.constBegin();
    auto currCell = allIn.last();
    allIn.pop_back();
    auto prevPath = _pathIntoCell[currCell];
    double distance, distanceMin;
    int counter = 0;
    order.push_back(currCell);
    const QPolygonF* currP;
    while (!allIn.empty())
    {
        ++counter;
        distanceMin = 1e9;
        while (stackIter != allIn.constEnd())
        {
            auto currPath = _pathIntoCell[*stackIter];
            distance = QLineF(prevPath.first().first(), currPath.last().last()).length();
            if (distance < distanceMin)
            {
                distanceMin = distance;
                currP = *stackIter;
            }
            ++stackIter;
        }
        order.push_back(currP);
        allIn.removeOne(currP);
        stackIter = allIn.constBegin();
    }
    /*for (const auto& curr: order) {
        std::cout << "Curr order value is " << curr << std::endl;
    }*/
    for (const auto& curr: order) {
        QPair<QPointF, QPointF> temp;
        auto currPath = _pathIntoCell[curr];
        QLineF first = QLineF(currPath.first().first(), currPath.first().last());
        temp.first = first.pointAt(0.5);
        QLineF second = QLineF(currPath.last().first(), currPath.last().last());
        temp.second = second.pointAt(0.5);
        flP.push_back(temp);
    }
    return res;
}

QList<QLineF> PathGenerator::_pathSegmRelationToCell(const QPolygonF& inPoly){
    QList<QLineF> res = {};

    res = _initNonRespectInnerHoles(&inPoly);

    return res;
}

void PathGenerator::setPolyHolesList(const QList<QPolygonF>& in)
{
    _holes = &in;
    _isHolesActive = !_isHolesActive;
    //pfc->init(*_holes);
}

void PathGenerator::setSurvPoly(const QPolygonF& in)
{
    _survPolygon = &in;
}

void PathGenerator::setPolyBoundary(const QPolygonF& in)
{
    _polyBoundary = &in;
}

void PathGenerator::_qDebugPrintPath(const QList<QList<QPointF>>& pathData){
    qDebug() << "=== Debug _pathRespectHolesWithNum ===";
    qDebug() << "Total paths:" << pathData.size();

    for (int pathIndex = 0; pathIndex < pathData.size(); ++pathIndex) {
        const QList<QPointF>& currentPath = pathData[pathIndex];

        qDebug() << "\nPath #" << pathIndex + 1 << "(size:" << currentPath.size() << "):";

        if (currentPath.isEmpty()) {
            qDebug() << "  [Empty path]";
            continue;
        }

        // Вывод всех точек с индексами
        for (int pointIndex = 0; pointIndex < currentPath.size(); ++pointIndex) {
            const QPointF& point = currentPath[pointIndex];

            qDebug() << QString("  %1: (%2, %3)")
                    .arg(pointIndex, 3)
                    .arg(point.x(), 0, 'f', 2)
                    .arg(point.y(), 0, 'f', 2);
        }

    }

    // Общая статистика
    int totalPoints = 0;
    for (const auto& path : pathData) {
        totalPoints += path.size();
    }

    qDebug() << "\n=== Summary ===";
    qDebug() << "Total paths:" << pathData.size();
    qDebug() << "Total points:" << totalPoints;

    if (!pathData.empty()) {
        qDebug() << QString("Average points per path: %1")
                .arg(static_cast<double>(totalPoints) / pathData.size(), 0, 'f', 2);
    }

    qDebug() << "=== End debug ===";
}

void PathGenerator::_qDebugPrintPathRespectHoles(const QList<QList<QPair<QPointF, int>>>& pathData) {
    qDebug() << "=== Debug _pathRespectHolesWithNum ===";
    qDebug() << "Total paths:" << pathData.size();

    for (int pathIndex = 0; pathIndex < pathData.size(); ++pathIndex) {
        const QList<QPair<QPointF, int>>& currentPath = pathData[pathIndex];

        qDebug() << "\nPath #" << pathIndex + 1 << "(size:" << currentPath.size() << "):";

        if (currentPath.isEmpty()) {
            qDebug() << "  [Empty path]";
            continue;
        }

        // Вывод всех точек с индексами
        for (int pointIndex = 0; pointIndex < currentPath.size(); ++pointIndex) {
            const QPair<QPointF, int> pointPair = currentPath[pointIndex];
            const QPointF& point = pointPair.first;
            int value = pointPair.second;

            qDebug() << QString("  %1: (%2, %3) → %4")
                    .arg(pointIndex, 3)
                    .arg(point.x(), 0, 'f', 2)
                    .arg(point.y(), 0, 'f', 2)
                    .arg(value);
        }

        // Статистика по значениям
        QMap<int, int> valueCount;
        for (const auto& pair : currentPath) {
            valueCount[pair.second]++;
        }

        if (valueCount.size() > 1) {
            qDebug() << "  Value statistics:";
            for (auto it = valueCount.constBegin(); it != valueCount.constEnd(); ++it) {
                qDebug() << QString("    Value %1: %2 points").arg(it.key()).arg(it.value());
            }
        }
    }

    // Общая статистика
    int totalPoints = 0;
    for (const auto& path : pathData) {
        totalPoints += path.size();
    }

    qDebug() << "\n=== Summary ===";
    qDebug() << "Total paths:" << pathData.size();
    qDebug() << "Total points:" << totalPoints;

    if (!pathData.empty()) {
        qDebug() << QString("Average points per path: %1")
                .arg(static_cast<double>(totalPoints) / pathData.size(), 0, 'f', 2);
    }

    qDebug() << "=== End debug ===";
}

QVariantList PathGenerator::connPList() const{
    QVariantList list;
    for(const auto& currL : _startEndPointsIntoCell){
        QVariantMap pointMap;
        pointMap["startP"]  = currL.first;
        pointMap["endP"]    = currL.second;
        list.append(pointMap);
    }
    return list;
}

void PathGenerator::_orientLineOneDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines) {
    qreal firstAngle = 0;
    for (const auto& currLine : lineList) {
        QLineF adjustedLine;

        if (lineList.first() == currLine) {
            firstAngle = currLine.angle();
        }

        if (qAbs(currLine.angle() - firstAngle) > 1.0) {
            adjustedLine.setP1(currLine.p2());
            adjustedLine.setP2(currLine.p1());
        } else {
            adjustedLine = currLine;
        }

        resultLines.append(adjustedLine);
    }
}

template<typename Type>
void PathGenerator::_orientLineOneDirection(const QList<Type>& lineList, QList<Type>& resultLines) {
    qreal firstAngle = 0;
    for (const Type & currLine : lineList) {
        Type res_line;
        for(int k = 0; k < currLine.count()-1; ++(++k)) {
            const QLineF &line = QLineF(currLine[k], currLine[k+1]);
            QLineF adjustedLine;

            if (lineList.first() == currLine) {
                firstAngle = line.angle();
            }

            if (qAbs(line.angle() - firstAngle) > 1.0) {
                adjustedLine.setP1(line.p2());
                adjustedLine.setP2(line.p1());
            } else {
                adjustedLine = line;
            }

            res_line << adjustedLine.p1();
            res_line << adjustedLine.p2();
        }
        resultLines.append(res_line);
    }
}

// обратный ход для вектора
template<typename Type>
void PathGenerator::_adjustToLawnower_oneVectorCase(const Type &lineList, Type &resultLines, bool &reverseVertices) {

    Type transectVertices = lineList;
    if (reverseVertices) {
        reverseVertices = false;
        Type reversedVertices;
        for (int j = transectVertices.count() - 1; j >= 0; j--) {
            reversedVertices += transectVertices[j];
        }
        transectVertices = reversedVertices;
    } else {
        reverseVertices = true;
    }
    resultLines = transectVertices;

}

QList<QList<QPointF>> PathGenerator::_orientNonRespectPath(const QList<QLineF>& inPath){
    QList<QList<QPointF>> res1, res2;

    /*if(inPath.count() < 1){
        _startEndP.first = QPointF{-1e4, -1e4};
        _startEndP.second = _startEndP.first;
        return res1;
    }*/

    for(const auto& currL:inPath){
        QList<QPointF> buffL;
        buffL   << currL.p1()
                << currL.p2();
        res1.append(buffL);
    }

    // чередование направления линий змейкой
    bool reverse = false;
    //добавление вектора в прямом или обратном направлении
    for(const auto& row: res1) {
        QList<QPointF> buffL;
        if (!reverse) {
            auto rowI = row.begin();
            while(rowI != row.end()) {
                buffL.append(*rowI);
                ++rowI;
            }
        }
        else {
            auto rowIr = row.rbegin();
            while(rowIr != row.rend()) {
                buffL.append(*rowIr);
                ++rowIr;
            }
        }
        //_adjustToLawnower_oneVectorCase(row, buffL, reverse);
        res2.append(buffL);
        reverse = !reverse;
    }

    // формируем точку входа выхода
    /*_startEndP.first    = res2[0][0];
    const auto& lastRow =  res2[res2.count()-1];
    _startEndP.second   = lastRow[lastRow.count()-1];*/

    return res2;
}

void PathGenerator::setTransectWidth(double in) {
    _gridSpace = in;
}

void PathGenerator::setPathSegments(const QList<QPolygonF> &in) {
    _bpd_decompositionCells = &in;
}

void PathGenerator::setDecomposeStruct(const holesInfoIn &in) {
    _decompose = &in;
//    _debugPrintHolesInfo(in);
}

void PathGenerator::_debugPrintHolesInfo(const holesInfoIn& info) {
    qDebug() << "=== holesInfoIn ===";

    qDebug() << "holeBorderSegm:" << info.holeBorderSegm.size() << "entries";
    int i = 1;
    for (auto it = info.holeBorderSegm.constBegin(); it != info.holeBorderSegm.constEnd(); ++it, ++i) {
        qDebug() << "  Hole" << i << "key:" << it.key();
        qDebug() << "    Segment 1:" << it.value().first;
        qDebug() << "    Segment 2:" << it.value().second;
    }

    qDebug() << "\nholeToBCD:" << info.holeToBCD.size() << "entries";
    i = 1;
    for (auto it = info.holeToBCD.constBegin(); it != info.holeToBCD.constEnd(); ++it, ++i) {
        qDebug() << "  Hole" << i << "key:" << it.key();
        qDebug() << "    BCD1:" << it.value().first;
        qDebug() << "    BCD2:" << it.value().second;
    }
}

QVariantList PathGenerator::pathConnection() const {
    QVariantList list;

    QVariantList row;
    for (const auto &lineS: _pathConnectionLines) {
        for (const auto &p: lineS) {
            QVariantMap pointMap;
            pointMap["x"] = p.x();
            pointMap["y"] = p.y();
            row.append(pointMap);
        }
        list.append(QVariant::fromValue(row));
        row.clear();
    }

    return list;
}

