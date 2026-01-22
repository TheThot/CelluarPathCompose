//
// Created by Admin on 11.01.2026.
//

#include "PolyBuilder.h"

void PolyBuilder::performClipperOperations(Path64& inProc){

}

PathD PolyBuilder::q_convert(const QPolygonF& in){
    if (in.isEmpty()) {
        std::cout << "Warning [poly_build] in poly is empty\n";
        return PathD{};
    }

    PathD currPath;
    for(const auto& curr: in){
        currPath.push_back(PointD(curr.x(), curr.y()));
    }

    storage_precision = currPath;

    setGeometry(storage_precision);

    return storage_precision;
}

QPolygonF PolyBuilder::q_getResult(){

    if(storage_precision.empty() && working_precision.empty())
        return QPolygonF{};
    auto polyRes = !storage_precision.empty() ? storage_precision :
                                        ScalePath<double, int64_t>(working_precision, scale_factor,_error_code);
    QPolygonF result;
    for (const auto &point : polyRes) {
        result << QPointF(point.x, point.y);
    }
    return result;
}

bool PolyBuilder::_hasIntersectionBounds(const Path64& path1, const Path64& path2) {
    if (path1.empty() || path2.empty()) {
        return false;
    }

    // Находим bounding box для первого полигона
    int64_t minX1 = std::numeric_limits<int64_t>::max();
    int64_t maxX1 = std::numeric_limits<int64_t>::min();
    int64_t minY1 = std::numeric_limits<int64_t>::max();
    int64_t maxY1 = std::numeric_limits<int64_t>::min();

    for (const auto& point : path1) {
        minX1 = std::min(minX1, point.x);
        maxX1 = std::max(maxX1, point.x);
        minY1 = std::min(minY1, point.y);
        maxY1 = std::max(maxY1, point.y);
    }

    // Находим bounding box для второго полигона
    int64_t minX2 = std::numeric_limits<int64_t>::max();
    int64_t maxX2 = std::numeric_limits<int64_t>::min();
    int64_t minY2 = std::numeric_limits<int64_t>::max();
    int64_t maxY2 = std::numeric_limits<int64_t>::min();

    for (const auto& point : path2) {
        minX2 = std::min(minX2, point.x);
        maxX2 = std::max(maxX2, point.x);
        minY2 = std::min(minY2, point.y);
        maxY2 = std::max(maxY2, point.y);
    }

    // Проверяем пересечение bounding box'ов
    return !(maxX1 < minX2 || minX1 > maxX2 || maxY1 < minY2 || minY1 > maxY2);
}

PathsD PolyBuilder::_intersect(const Path64& workingClip){

    if (!_hasIntersectionBounds(working_precision, workingClip)) {
        return PathsD{};
    }

    // Выполняем операцию пересечения
    Clipper64 clipper;
    clipper.AddSubject({working_precision});
    clipper.AddClip({workingClip});

    Paths64 solution;
    clipper.Execute(ClipType::Intersection,
                    FillRule::NonZero,
                    solution);

    return ScalePaths<double, int64_t>(solution, 1.0 / scale_factor, _error_code);
}

PathsD PolyBuilder::_substractS(const PathsD& workingClips){
    // Преобразуем PathsD в Paths64
    Paths64 clips64 = ScalePaths<int64_t, double>(workingClips, scale_factor, _error_code);

    // сначала добавляем + смещение чтобы секции не совпадали
    auto offsetPolygons = _offsetPolygons(clips64, 1);

    // Выполняем операцию вычитания
    Clipper64 clipper;

    clipper.AddSubject({working_precision});
    clipper.AddClip(offsetPolygons);  // Добавляем все clip полигоны

    Paths64 solution;
    clipper.Execute(ClipType::Difference,
                    FillRule::EvenOdd,
                    solution);

    return ScalePaths<double, int64_t>(solution, 1.0 / scale_factor, _error_code);
}

PathsD PolyBuilder::_substract(const Path64& workingClip, bool doOffset){

    // Выполняем операцию вычитания
    Clipper64 clipper;
    clipper.AddSubject({working_precision});
    if(doOffset) {
        // сначала добавляем + смещение чтобы секции не совпадали
        auto offsetPolygons = _offsetPolygon(workingClip, -5);
        clipper.AddClip({offsetPolygons});
    }
    else
        clipper.AddClip({workingClip});

    Paths64 solution;
    clipper.Execute(ClipType::Difference,
                    FillRule::EvenOdd,
                    solution);

    return ScalePaths<double, int64_t>(solution, 1.0 / scale_factor, _error_code);
}

QList<QPolygonF> PolyBuilder::subtractedListWrp(const QPolygonF &poly1, const QList<QPolygonF> &poly2){
    QList<QPolygonF> res = {};
    if (poly1.isEmpty() || poly2.isEmpty()) {
        return res;
    }

    // Устанавливаем основной полигон
    q_convert(poly1);

    // Подготавливаем все clip полигоны
    PathsD clip;
    for (const auto &currPoly : poly2) {
        if (currPoly.isEmpty()) continue;

        PathD oneClip;
        for (const auto &p : currPoly) {
            oneClip.push_back(PointD(p.x(), p.y()));
        }
        clip.push_back(oneClip);  // ВАЖНО: добавляем в вектор!
    }

    // Используем новую функцию для вычитания многих полигонов
    auto resClipper = _substractS(clip);

    for (const auto &onePoly : resClipper) {
        QPolygonF temPoly;
        for (const auto &p: onePoly) {
            temPoly << QPointF(p.x, p.y);
        }
        if (!temPoly.isEmpty()) {
            res.append(temPoly);
        }
    }

    if (res.isEmpty()) {
        std::cout << "Warning [poly_build] substr result is zero poly\n";
    }

    return res;
}

QList<QPolygonF> PolyBuilder::subtractedListWrp(const QPolygonF &poly1, const QPolygonF &poly2, bool doOffset){
    QList<QPolygonF> res = {};
    if (poly1.isEmpty() || poly2.isEmpty()) {
        return res;
    }

    q_convert(poly1);
    PolyBuilder temp;
    PathD clip;
    for (const QPointF &point : poly2)
        clip.push_back(PointD(point.x(), point.y()));

    temp.setGeometry(clip);

    auto resClipper = _substract(temp.getPrecise(), doOffset);

    for (const auto &onePoly : resClipper) {
        QPolygonF temPoly;
        for (const auto &p: onePoly)
            temPoly << QPointF(p.x, p.y);
        res.append(temPoly);
    }

    if (res.isEmpty())
        std::cout << "Warning [poly_build] substr result is zero poly\n";

    return res;
}

QPolygonF PolyBuilder::snglIntersctnWrp(const QPolygonF &poly1, const QPolygonF &poly2){
    QPolygonF res = {};
    if (poly1.isEmpty() || poly2.isEmpty()) {
        return res;
    }
    // close polies
    QPolygonF polyLoc1 = poly1;
    QPolygonF polyLoc2 = poly2;
    polyLoc1.append(poly1[0]);
    polyLoc2.append(poly2[0]);

    q_convert(polyLoc1);
    PolyBuilder temp;
    PathD clip;
    for (const QPointF &point : polyLoc2)
        clip.push_back(PointD(point.x(), point.y()));

    temp.setGeometry(clip);

    auto resClipper = _intersect(temp.getPrecise());

    // по результату данной функции нужен только один полигон
    // Находим самый большой полигон по площади
    double maxArea = -1.0;
    PathD largestPath;
    bool hasValidPolygon = false;
    const double MIN_VALID_AREA = 0.0001;

    if (resClipper.empty()) {
        return res;
    }

    for (const auto &path : resClipper) {
        double area = std::abs(Clipper2Lib::Area(path));
        if (area > MIN_VALID_AREA && area > maxArea) {
            maxArea = area;
            largestPath = path;
            hasValidPolygon = true;
        }
    }

    if (!hasValidPolygon) {
        return res;
    }

    for (const auto &point : largestPath)
        res << QPointF(point.x,point.y);

    if (!res.isEmpty() && res.first() != res.last()) {
        res.append(res.first());
    }

    return res;
}
// Добавляем небольшое смещение к clip полигонам
Paths64 PolyBuilder::_offsetPolygon(const Path64& polygons, double delta) {
    Clipper2Lib::ClipperOffset offsetter;

    // Настройки offset
    Clipper2Lib::JoinType joinType = Clipper2Lib::JoinType::Square;
    Clipper2Lib::EndType endType = Clipper2Lib::EndType::Polygon;

    offsetter.AddPath(polygons, joinType, endType);

    Paths64 solution;
    offsetter.Execute(delta, solution);

    return solution;
}

// Добавляем небольшое смещение к clip полигонам
Paths64 PolyBuilder::_offsetPolygons(const Paths64& polygons, double delta) {
    Clipper2Lib::ClipperOffset offsetter;

    // Настройки offset
    Clipper2Lib::JoinType joinType = Clipper2Lib::JoinType::Square;
    Clipper2Lib::EndType endType = Clipper2Lib::EndType::Polygon;

    offsetter.AddPaths(polygons, joinType, endType);

    Paths64 solution;
    offsetter.Execute(delta, solution);

    return solution;
}

PathsD PolyBuilder::_union(const PathsD& workingClip, int offset){
    // Преобразуем PathsD в Paths64
    Paths64 clips64 = ScalePaths<int64_t, double>(workingClip, scale_factor, _error_code);
    // сначала добавляем + смещение чтобы секции не совпадали
    auto offsetPolygons = _offsetPolygons(clips64, offset);
    // Выполняем операцию объединения
    // Выполняем операцию объединения ВСЕХ полигонов как Subject
    Clipper64 clipper;
    clipper.AddSubject(offsetPolygons);  // ВСЕ полигоны добавляем как subject

    Paths64 solution;
    clipper.Execute(ClipType::Union,
                    FillRule::NonZero,
                    solution);

    return ScalePaths<double, int64_t>(solution, 1.0 / scale_factor, _error_code);
}

QList<QPolygonF> PolyBuilder::unitedListWrp(const QList<QPolygonF> &poly2, int offset) {
    auto res = QList<QPolygonF>();

    if (poly2.isEmpty()) {
        return res;
    }

    // Подготавливаем все clip полигоны
    PathsD clip;
    for (const auto &currPoly : poly2) {
        if (currPoly.isEmpty()) continue;

        PathD oneClip;
        for (const auto &p : currPoly) {
            oneClip.push_back(PointD(p.x(), p.y()));
        }
        clip.push_back(oneClip);  // ВАЖНО: добавляем в вектор!
    }

    // Используем новую функцию для вычитания многих полигонов
    auto resClipper = _union(clip, offset);

//    std::cout << "[poly_build] resClipper size " << resClipper.size() << std::endl;

    for (const auto &onePoly : resClipper) {
        QPolygonF temPoly;
        for (const auto &p: onePoly)
            temPoly << QPointF(p.x, p.y);
        res.append(temPoly);
    }

    if(res.isEmpty()){
        std::cout << "Warning [poly_build] union result is empty\n";
        return res;
    }

    return res;
}

QPolygonF PolyBuilder::offsetWrp(const QPolygonF &poly, int offset){
    auto res = QPolygonF();

    if (poly.isEmpty()) {
        return res;
    }

    // close polies
    QPolygonF polyLoc = poly;
    polyLoc.append(poly[0]);

    q_convert(polyLoc);
    PathsD workingClip;
    workingClip.push_back(storage_precision);

    Paths64 clips64 = ScalePaths<int64_t, double>(workingClip, scale_factor, _error_code);

    auto resClipper = _offsetPolygons(clips64, offset);

    PathsD resClipperD = ScalePaths<double, int64_t>(resClipper, 1.0 / scale_factor, _error_code);

    for (const auto &onePoly : resClipperD) {
        QPolygonF temPoly;
        for (const auto &p: onePoly)
            temPoly << QPointF(p.x, p.y);
        res.append(temPoly);
    }

    if(res.isEmpty()){
        std::cout << "Warning [poly_build] offset result is empty\n";
        return res;
    }

    return res;
}
