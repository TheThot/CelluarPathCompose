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

PathsD PolyBuilder::_intersect(const Path64& workingClip){

    // Выполняем операцию пересечения
    Clipper64 clipper;
    clipper.AddSubject({working_precision});
    clipper.AddClip({workingClip});

    Paths64 solution;
    clipper.Execute(ClipType::Intersection,
                    FillRule::EvenOdd,
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

PathsD PolyBuilder::_substract(const Path64& workingClip){
    // Выполняем операцию вычитания
    Clipper64 clipper;
    clipper.AddSubject({working_precision});
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

    if(res.count() == 1) {
        std::cout << "Is it ok? [poly_build] substr one poly result\n";
    }

    if (res.isEmpty()) {
        std::cout << "Warning [poly_build] substr result is zero poly\n";
    }

    return res;
}

QList<QPolygonF> PolyBuilder::subtractedListWrp(const QPolygonF &poly1, const QPolygonF &poly2){
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

    auto resClipper = _substract(temp.getPrecise());

    for (const auto &onePoly : resClipper) {
        QPolygonF temPoly;
        for (const auto &p: onePoly)
            temPoly << QPointF(p.x, p.y);
        res.append(temPoly);
    }

    if(res.count() == 1)
        std::cout << "Is it ok? [poly_build] substr one poly result\n";

    if (res.isEmpty())
        std::cout << "Warning [poly_build] substr result is zero poly\n";

    return res;
}

QPolygonF PolyBuilder::snglIntersctnWrp(const QPolygonF &poly1, const QPolygonF &poly2){
    QPolygonF res = {};
    if (poly1.isEmpty() || poly2.isEmpty()) {
        return res;
    }

    q_convert(poly1);
    PolyBuilder temp;
    PathD clip;
    for (const QPointF &point : poly2)
        clip.push_back(PointD(point.x(), point.y()));

    temp.setGeometry(clip);

    auto resClipper = _intersect(temp.getPrecise());

    // по результату данной функции нужен только один полигон
    // Находим самый большой полигон по площади
    double maxArea = -1.0;
    PathD largestPath;

    for (const auto &path : resClipper) {
        double area = std::abs(Clipper2Lib::Area(path));
        if (area > maxArea) {
            maxArea = area;
            largestPath = path;
        }
    }

    for (const auto &point : largestPath)
        res << QPointF(point.x,point.y);

    if (!res.isEmpty() && res.first() != res.last()) {
        res.append(res.first());
    }

    return res;
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

PathsD PolyBuilder::_union(const PathsD& workingClip){
    // Преобразуем PathsD в Paths64
    Paths64 clips64 = ScalePaths<int64_t, double>(workingClip, scale_factor, _error_code);
    // сначала добавляем + смещение чтобы секции не совпадали
    auto offsetPolygons = _offsetPolygons(clips64, 10);
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

QList<QPolygonF> PolyBuilder::unitedListWrp(const QList<QPolygonF> &poly2) {
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
    auto resClipper = _union(clip);

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
