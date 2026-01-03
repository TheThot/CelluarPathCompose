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
        pathTraj = _oneLoopTraj(_pathRespectHoles);
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
        /*auto iter = _pathIntoCell.begin();
        while(iter != _pathIntoCell.end()) {
            _qDebugPrintPath(iter.value());
            ++iter;
        }*/
        //применяем astar для соединения между cell
        _pathRespectHoles = _pathRouteBetweenCells(_pathIntoCell);
    }
}

QList<QList<QPointF>> PathGenerator::_pathRouteBetweenCells(const QHash< const QPolygonF*, QList<QList<QPointF>> >& inPath){

    QList<QList<QPointF>> result;

    if (inPath.isEmpty()) {
        return result;
    }

    auto it = inPath.constBegin();
    const auto firstCellPaths = it.value();

    // Если первый элемент пуст, возвращаем пустой результат
    if (firstCellPaths.isEmpty()) {
        std::cout << "Bad section 3\n";
        return result;
    }

    // Получаем первую точку из последнего сегмента первого элемента
    const auto& lastSegment = firstCellPaths.last();
    if (lastSegment.size() < 2) {
        std::cout << "Bad section 4\n";
        return result;
    }

    QPointF prevConnectionPoint = lastSegment.last();

    // Обрабатываем первый элемент
    result.append(firstCellPaths);

    int counter = 0;
    // Обрабатываем остальные элементы
    for (++it; it != inPath.constEnd(); ++it) {
        const auto& currentPaths = it.value();

        if (currentPaths.isEmpty()) {
            std::cout << "Bad section 1\n";
            continue;
        }

        // Добавляем соединение между предыдущим и текущим элементом
        const auto& firstSegment = currentPaths.first();
        if (firstSegment.size() >= 2) {
            QPointF currentConnectionPoint = firstSegment.first();

            pfc->perform(prevConnectionPoint, currentConnectionPoint);
            auto connectionPath = pfc->getPath2d();

            if (!connectionPath.isEmpty()) {
                connectionPath = adaptiveSample(connectionPath);
                result.append(connectionPath);
            }
        }
        else
        {
            std::cout <<  "Bad section" << std::endl;
        }

        // Добавляем пути текущего элемента
        result.append(currentPaths);

        // Обновляем точку соединения для следующей итерации
        const auto& lastSegment = currentPaths.last();
        if (lastSegment.size() >= 2) {
            prevConnectionPoint = lastSegment.last();
        }

    }

    return result;

}

QHash<const QPolygonF*, QList<QList<QPointF>>> PathGenerator::_configurePathIntoCell(
    QHash<const QPolygonF*, int>& order, QHash<const QPolygonF*, QPair<QPointF, QPointF>>& flP)
{
    QHash<const QPolygonF*, QList<QList<QPointF>>> res;
    if (_pathIntoCell.count() == 0)
    {
        std::cout << "Warning path hash is null\n";
        return res;
    }
    // формируем массив с расстояниями от начала одной до конца другой из qhash
    QVector<const QPolygonF*> allIn;
    for (auto itOne = _pathIntoCell.constBegin(); itOne != _pathIntoCell.constBegin(); ++itOne)
    {
        allIn.push_back(itOne.key());
    }
    auto stackIter = allIn.constBegin();
    auto currCell = allIn.last();
    allIn.pop_back();
    auto prevPath = _pathIntoCell[currCell];
    double distance, distanceMin;
    int counter = 0;
    order[currCell] = counter;
    const QPolygonF* currP;
    while (allIn.empty() == false)
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
                order[currP] = counter;
            }
            ++stackIter;
        }
        allIn.removeOne(currP);
        stackIter = allIn.constBegin();
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

QPointF PathGenerator::startP() const{
    return _startEndP.first;
}

QPointF PathGenerator::endP() const{
    return _startEndP.second;
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

