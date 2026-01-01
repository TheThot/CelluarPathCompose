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
    if(inPoly == nullptr)
        return res;

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
    _pathRespectHolesWithNum.clear();
    _holeMannerPathSegm.clear();
    _pathIntoCell.clear();

    _path = _initNonRespectInnerHoles(_survPolygon);
    _orientedPathSimpl = _orientNonRespectPath(_path);

    if(_holes != nullptr) {
        pfc->init(*_holes);
        for (int i = 0; i < _bpd_decompositionCells->count(); ++i) {
            auto res = _pathSegmRelationToCell(_bpd_decompositionCells->at(i));
            QList<QList<QPointF>> resPointList = _orientNonRespectPath(res);
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
    QList<QList<QPointF>> res = {};

    auto iter = inPath.begin();
    QPointF first, sec;
    if(iter.value().count() != 0)
        first = iter.value().last().last();
    else
        return res;
    res.append(iter.value());
    ++iter;
    if(iter.value().count() != 0)
        sec = iter.value().first().first();
    else
        return res;
    pfc->perform(first, sec);
    auto pathBuff = pfc->getPath2d();
    std::cout << "[_pathRouteBetweenCells] size path before uni " << pathBuff.count() << std::endl;
    pathBuff = uniformSample(pathBuff, 4);
    std::cout << "[_pathRouteBetweenCells] size path after uni " << pathBuff.count() << std::endl;
    if (pathBuff.count() != 0)
        res += pathBuff;
    res.append(iter.value());

    /*auto iter = inPath.begin();
    QPointF first, sec;
    if(iter.value().count() != 0)
        first = iter.value().last().last();
    while(iter != inPath.end()){
        auto currPath = iter.value();
        if(currPath.count() != 0) {
            res += currPath;
            if(iter != inPath.begin()) {
                sec = currPath[0][0];
                pfc->perform(first, sec, _gridSpace);
                auto pathBuff = pfc->getPath2d();
                pathBuff = uniformSample(pathBuff, 4);
                first = currPath.last().last();
                if (pathBuff.count() != 0)
                    res += pathBuff;
                else
                    std::cout << "[_pathRouteBetweenCells] pathBuff is null " << std::endl;
            }
        }
        ++iter;
    }*/

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
}

void PathGenerator::setSurvPoly(const QPolygonF& in)
{
    _survPolygon = &in;
}

void PathGenerator::setPolyBoundary(const QPolygonF& in)
{
    _polyBoundary = &in;
}

QList<QList<QPointF>> PathGenerator::_initLinesRespectHoles(QList<QList<QPair<QPointF,int>>>& pathRespectHolesWithNum)
{
    QList<QList<QPointF>> res = {};
    if(_holes == nullptr)
        return res;

    int counter = 0;
    QLineF holeL;
    QPair<QPointF,int> buffPr = {};
//    pathRespectHolesWithNum.reserve(_path.count());
    for(int i = 0; i < _path.count(); ++i)
        pathRespectHolesWithNum.append(QList<QPair<QPointF,int>>{});
    for (const auto& currTransect : _path)
    {
        int j = 1;
        QList<int> resHoleNum;
        QList<QPointF> resI;
        QList<QPointF> buff;
        QList<double> dist;
        buff.append(currTransect.p1());
        buffPr.first = currTransect.p1(); buffPr.second = 0;
        pathRespectHolesWithNum[counter].append(buffPr);
        for (const auto& currHolePoly: *_holes)
        {
            for (int i = 0; i < currHolePoly.count(); ++i)
            {
                int k = (i + 1) % currHolePoly.count();
                holeL = QLineF(currHolePoly[i], currHolePoly[k]);
                //res - все пересечения линии с любыми _holes
                if(intersectionListFormimgRoutine(currTransect, holeL, resI, QLineF::BoundedIntersection))
                    resHoleNum.append(j);
            }
            ++j;
        }
        // сразу buff пересечений класть в _pathRespectHoles нельзя
        // сперва отсортировать по расстояниям
        for(const auto& oneIntersection:resI){
            holeL = QLineF(currTransect.p1(), oneIntersection);
            dist.append(holeL.length());
        }
        auto idx = sort_indexes<double>(dist); // отсортированы по индесам однозначно resI и resHoleNum
        j = -1;
        for(const auto& currI:idx){
            buff.append(resI[currI]);
            buffPr.first = resI[currI]; buffPr.second = j*resHoleNum[currI];
            pathRespectHolesWithNum[counter].append(buffPr);
            j *= -1;
        }
        buff.append(currTransect.p2());
        buffPr.first = currTransect.p2(); buffPr.second = 0;
        pathRespectHolesWithNum[counter].append(buffPr);
//        std::cout << "[_initLinesRespectHoles] buff length is " << buff.count() << std::endl;
        res.append(buff);
        ++counter;
    }

    return res;
}

//функция по сути преобразует сложные комбинации типов хранящие информацию о структуре SurvPoly в _pathRespectHoles которое выдаются qml интерфейсу
QList<QList<QPointF>> PathGenerator::_drawComplexCoverPathSequence(){
    // preprocess _pathRespectHolesWithNum and construct info var pathCoverProgress
    _pcp.totalTransects = 0;
    _pcp.numRows = _pathRespectHolesWithNum.count();
    for(const auto & tr: _pathRespectHolesWithNum){
        _pcp.totalTransects += tr.count();
    }
    int buffCount = 0;
    for(int holeCount = 0; holeCount < _holes->count(); ++holeCount){
        QPolygonF* ptr = &const_cast<QPolygonF&>(_holes->at(holeCount));
        auto currCells = _decompose->holeToBCD[ptr];
        for(const auto & tr: _pathRespectHolesWithNum){
            for(const auto& p:tr) {
                if (p.second == holeCount) // нашли сегмент пересек интересующую зону
                    ++buffCount;
            }
        }
        _pcp.zoneLoadRate.insert(currCells.first, QPair<int,double>{1,0.5});
        _pcp.zoneLoadRate.insert(currCells.second, QPair<int,double>{2,0.1});
        buffCount = 0;
    }
    // чередование направления линий змейкой
    bool reverse = false;
    QList<QList<QPointF>> res;
    for(const auto& row: _pathRespectHolesWithNum){ // row - список пар (точка, hole intersect) hole intersect = - 1 на краях SurvPoly
        QList<QPointF> buffL, adjL;
        for(const auto& p:row){
            if(p.second == -1){
                buffL.append(p.first);
            }else{
                buffL.clear();
            }
        }
        _adjustToLawnower_oneVectorCase(buffL, adjL, reverse);
        res.append(adjL);
    }

    return res;
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

QMap<int,QList<QList<QPair<QPointF,int>>>> PathGenerator::_preProcRespectInnerHoles()
{
    //как показывает практика ориентация direct reverse пути змейки не зависит от конфигурации survPoly и holes
    //и формирование направления у _pathRespectHolesWithNum может быть выполнено предварительно
    bool reverse = false;
    for(auto& row: _pathRespectHolesWithNum) {
        QList<QPair<QPointF, int>> buffL;
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
        row = buffL;
        reverse = !reverse;
    }

    // формируем новый массив на базе _pathRespectHolesWithNum суть та же но теперь по key {номер бесполётной и сторона}
    // можно получить доступ к списку segm вокруг бесполётной
    // поскольку все развернули то ветви if(row[i].second < 0) append так if(row[i].second > 0) сяк
    QMap<int,QList<QList<QPair<QPointF,int>>>> res = {};
    int reverseI = (-1);
    for(auto& lineSegments: _pathRespectHolesWithNum){
        reverseI *= (-1);
        for(int i = 0; i < lineSegments.count(); ++i){
            if(lineSegments[i].second != 0){
                auto buff = res[lineSegments[i].second];

                auto line = QList<QPair<QPointF,int>>{};

                if(reverseI*lineSegments[i].second < 0)
                    line += lineSegments[i-1];

                line += lineSegments[i];

                if(reverseI*lineSegments[i].second > 0)
                    line += lineSegments[i+1];

                buff.append(line);

                res.insert(lineSegments[i].second, buff);
            }
        }
    }

    return res;
}

bool PathGenerator::_updateCountRule(){
    bool rule;

    if(_holes == nullptr){
        return false;
    }

    // первые holes.count * 2 _bpd_decompositionCells это U и D каждой Cell
    QList<QRectF> check;
    for(int i = 0; i < _holes->count() * 2; ++i){
        check.append(_bpd_decompositionCells->at(i).boundingRect());
    }
    QList<QRectF> check2;
    for(int i = 0; i < _holes->count() * 2; i+=2){
        check2.append(check[i].united(check[i+1]));
    }
    QList<double> squareHolesSection;
    for(int i = 0; i < check2.count(); ++i) {
        squareHolesSection.append(check2[i].height()*check2[i].width());
    }
    double sumSq = 0;
    double max = 0;
    for(int i = 0; i < check2.count(); ++i) {
        if(max < squareHolesSection[i])
            max = squareHolesSection[i];
        sumSq += squareHolesSection[i];
    }
    QRectF whole;
    for(int i = 0; i < check2.count()-1; ++i) {
        if(i == 0)
            whole = check2[i].united(check2[i+1]);
        else
            whole = whole.united(check2[i+1]);
    }
    double squareWhole = whole.width()*whole.height();
    _ruleRate =  squareWhole / sumSq;
    _ruleRate = std::clamp<double>(_ruleRate, 0, 1);
    // false - max, true - sum
    if(squareWhole > max)
        rule = true;
    else
        rule = false;

    return rule;
}

int PathGenerator::goTroughtHoleDirectProc(QVector<int>& holesNumStack, QList<QList<QPointF>>& res) // -> return numrows
{
    // рекурсивный алгоритм добавления сегментов вокруг бесполётной зоны
    // рекурсия смотрит закончена ли сторона ? определяемая QPair<QPointF, int> + и - разные стороны у номера hole
    // рекурсия смотрит есть ли у текущих сегментов смежные с другими hole области все они добавляются в стек holesNumStack
    if (holesNumStack.count() < 1)
        return 0;
    int currSideNum = holesNumStack.last();
    /*std::cout << " ======== start ========" << std::endl;
    for(const auto& hs: holesNumStack){
        std::cout << " holesNumStack is " << hs;
    }
    std::cout << "\n ======== end ========" << std::endl;*/
    bool triggerRule = true;
    int revers = -1;
    int skeepRowNums = 0;
QLineF testL;
    int sum = 0;
    auto lastAdd = res.last()[1];
    for(int k = 0; k < 2; ++k) {
        revers *= -1;
        auto currList = _holeMannerPathSegm[revers*currSideNum];
        for(int i = k; i < currList.count(); ++i){
            QList<QPointF> buffL;
            for (const auto &curr: currList[i]) {
                testL = QLineF{lastAdd, curr.first};
                testL = extendLineBothWays(testL, 10);
                pfc->perform(testL.p1(), testL.p2());
                auto pathBuff = pfc->getPath2d();
                pathBuff = uniformSample(pathBuff,4);
                buffL.append(pathBuff);
                res.append(buffL);
                return 0;
                if (curr.second != 0) {  // другая точка сообщает есть ли рядом другие зоны
                    if(!holesNumStack.contains((-1) * curr.second) || !holesNumStack.contains(curr.second)) {
                        holesNumStack.push_back(curr.second);
                        holesNumStack.push_back((-1) * curr.second);
                        res.append(buffL);
                        triggerRule = false;
                        skeepRowNums = goTroughtHoleDirectProc(holesNumStack, res);
                        i += skeepRowNums;
                    }
                }
            }
            if (triggerRule) {
                res.append(buffL);
            } else {
                triggerRule = true;
            }
        }
    }
    if(_currRule)
        skeepRowNums += _holeMannerPathSegm[revers * currSideNum].count() - 1;
    else
        skeepRowNums = std::max(skeepRowNums, _holeMannerPathSegm[revers * currSideNum].count() - 1);

    //holesNumStack.pop_back();
    //holesNumStack.pop_back(); // удаление из стека для остановки рекурсии
    return skeepRowNums;
}

QList<QList<QPointF>> PathGenerator::_pathProcRespectInnerHoles()
{
    _holesNumStack.clear(); // храним текущую сторону - или + и номер бесполётной
    QList<QList<QPointF>> res; // список линий QList<QPointF>
    bool flag = false;
    for(int i = 0; i < _pathRespectHolesWithNum.count(); ++i){ // row - список пар (точка, hole intersect) hole intersect = - 1 на краях SurvPoly
        auto currRow = _pathRespectHolesWithNum[i];
        auto rowI = currRow.begin();
        QList<QPointF> buffL;
        while(rowI != currRow.end()) {
            buffL += rowI->first;
            if(rowI->second != 0){ // наткнулись на hole
                if(!_holesNumStack.contains((-1) * rowI->second) || !_holesNumStack.contains(rowI->second)) {
                    flag = true;
                    auto num = rowI->second;
                    _holesNumStack.push_back(num);
                    _holesNumStack.push_back((-1) * num);
                    res.append(buffL);
                    //по выходу с рекурсии корректируем счётчик внешнего цикла i (так как много строк связанных с holes добавлено функцией)
                    i += goTroughtHoleDirectProc(_holesNumStack, res); // уходим в рекурсию
                    //break;
                    return res;
                }
            }
            ++rowI;
        }
        if(!flag) {
            res.append(buffL);
        }else{
            flag = false;
        }
    }
    return res;
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

