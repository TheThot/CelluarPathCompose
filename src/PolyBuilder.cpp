//
// Created by Admin on 11.01.2026.
//

#include "PolyBuilder.h"

void PolyBuilder::performClipperOperations(Path64& inProc){

}

PathD PolyBuilder::q_convert(const QPolygonF& in) {
    if (in.isEmpty()) {
        std::cout << "Warning [poly_build] in poly is empty\n";
        return PathD{};
    }

    // Автоматически закрываем полигон если нужно
    QPolygonF closedPolygon = closePolygon(in);

    PathD currPath;
    for(const auto& curr: closedPolygon) {
        currPath.push_back(PointD(curr.x(), curr.y()));
    }

    storage_precision = currPath;
    setGeometry(storage_precision);
    return storage_precision;
}

QList<QPolygonF> PolyBuilder::subtractedListWrp(const QPolygonF &poly1, const QList<QPolygonF> &poly2) {
    QList<QPolygonF> result;

    // Проверка входных данных
    if (poly1.isEmpty()) {
        std::cout << "Error: poly1 is empty\n";
        return result;
    }

    if (poly2.isEmpty()) {
        std::cout << "Error: poly2 list is empty\n";
        return result;
    }

    // 1. Закрываем и проверяем poly1
    QPolygonF closedPoly1 = poly1;
    if (closedPoly1.first() != closedPoly1.last()) {
        closedPoly1.append(closedPoly1.first());
    }

    if (closedPoly1.size() < 3) {
        std::cout << "Error: poly1 has less than 3 points after closing\n";
        return result;
    }

    std::cout << "Debug: poly1 points: " << closedPoly1.size() << "\n";

    // 2. Устанавливаем основной полигон
    q_convert(closedPoly1);

    // 3. Подготавливаем clip полигоны
    PathsD clipPaths;
    int validClipCount = 0;

    for (int i = 0; i < poly2.size(); ++i) {
        const QPolygonF& currPoly = poly2[i];

        if (currPoly.isEmpty()) {
            std::cout << "Debug: poly2[" << i << "] is empty, skipping\n";
            continue;
        }

        // Закрываем полигон
        QPolygonF closedPoly = currPoly;
        if (closedPoly.first() != closedPoly.last()) {
            closedPoly.append(closedPoly.first());
        }

        if (closedPoly.size() < 3) {
            std::cout << "Debug: poly2[" << i << "] has less than 3 points, skipping\n";
            continue;
        }

        // Преобразуем в PathD
        PathD path;
        for (const QPointF& point : closedPoly) {
            path.push_back(PointD(point.x(), point.y()));
        }

        clipPaths.push_back(path);
        validClipCount++;

        std::cout << "Debug: poly2[" << i << "] points: " << closedPoly.size() << "\n";
    }

    if (clipPaths.empty()) {
        std::cout << "Error: no valid clip polygons\n";
        return result;
    }

    std::cout << "Debug: valid clip polygons: " << validClipCount << "\n";

    // 4. Выполняем вычитание через Clipper2 напрямую (обход _substractS)
    // Преобразуем основной полигон в Path64
    Path64 subjectPath = working_precision; // уже есть после q_convert

    // Преобразуем clip полигоны в Paths64
    Paths64 clipPaths64;
    for (const auto& path : clipPaths) {
        Path64 scaledPath = ScalePath<int64_t, double>(path, scale_factor, _error_code);
        if (_error_code == 0 && scaledPath.size() >= 3) {
            clipPaths64.push_back(scaledPath);
        }
    }

    // Выполняем операцию
    Clipper2Lib::Clipper64 clipper;
    clipper.AddSubject({subjectPath});
    clipper.AddClip(clipPaths64);

    Paths64 solution;
    bool success = clipper.Execute(Clipper2Lib::ClipType::Difference,
                                   Clipper2Lib::FillRule::EvenOdd,
                                   solution);

    if (!success) {
        std::cout << "Error: clipper operation failed\n";
        return result;
    }

    std::cout << "Debug: solution polygons count: " << solution.size() << "\n";

    // 5. Преобразуем результат
    for (const auto& path64 : solution) {
        if (path64.size() < 3) continue;

        // Масштабируем обратно
        PathD pathD = ScalePath<double, int64_t>(path64, 1.0 / scale_factor, _error_code);

        if (_error_code != 0 || pathD.size() < 3) continue;

        // Преобразуем в QPolygonF
        QPolygonF polygon;
        for (const auto& point : pathD) {
            polygon.append(QPointF(point.x, point.y));
        }

        // Закрываем полигон
        if (polygon.size() >= 2 && polygon.first() != polygon.last()) {
            polygon.append(polygon.first());
        }

        if (polygon.size() >= 3) {
            result.append(polygon);
        }
    }

    std::cout << "Debug: result polygons count: " << result.size() << "\n";

    // Сообщения как в оригинале
    if (result.count() == 1) {
        std::cout << "Is it ok? [poly_build] substr one poly result\n";
    }

    if (result.isEmpty()) {
        std::cout << "Warning [poly_build] substr result is zero poly\n";
    }

    return result;
}

QPolygonF PolyBuilder::snglIntersctnWrp(const QPolygonF &poly1, const QPolygonF &poly2) {
    QPolygonF res = {};

    if (poly1.isEmpty() || poly2.isEmpty()) {
        return res;
    }

    // Закрываем оба полигона
    QPolygonF closedPoly1 = closePolygon(poly1);
    QPolygonF closedPoly2 = closePolygon(poly2);

    if (closedPoly1.size() < 3 || closedPoly2.size() < 3) {
        std::cout << "Warning [poly_build] polygon has less than 3 points\n";
        return res;
    }

    q_convert(closedPoly1);

    // Создаем clip полигон
    PathD clip;
    for (const QPointF &point : closedPoly2) {
        clip.push_back(PointD(point.x(), point.y()));
    }

    // Закрываем clip если нужно
    clip = closePathD(clip);

    PolyBuilder temp;
    temp.setGeometry(clip);

    auto resClipper = _intersect(temp.getPrecise());

    // Находим самый большой полигон
    double maxArea = -1.0;
    PathD largestPath;

    for (const auto &path : resClipper) {
        if (path.empty()) continue;

        PathD closedPath = closePathD(path);
        double area = std::abs(Clipper2Lib::Area(closedPath));
        if (area > maxArea) {
            maxArea = area;
            largestPath = closedPath;
        }
    }

    if (largestPath.empty()) {
        return res;
    }

    for (const auto &point : largestPath) {
        res << QPointF(point.x, point.y);
    }

    return res;
}

PathsD PolyBuilder::_substract(const Path64& workingClip) {
    if (working_precision.empty() || workingClip.empty()) {
        return PathsD{};
    }

    Clipper64 clipper;
    clipper.AddSubject({working_precision});
    clipper.AddClip({workingClip});

    Paths64 solution;
    clipper.Execute(ClipType::Difference,
                    FillRule::EvenOdd,
                    solution);

    if (solution.empty()) {
        return PathsD{};
    }

    return ScalePaths<double, int64_t>(solution, 1.0 / scale_factor, _error_code);
}

// Вспомогательная функция для проверки закрытости PathD
bool isPathClosed(const PathD& path) {
    if (path.size() < 2) return false;

    const PointD& first = path[0];
    const PointD& last = path[path.size() - 1];

    const double epsilon = 1e-6;
    return std::abs(first.x - last.x) < epsilon &&
           std::abs(first.y - last.y) < epsilon;
}

PathsD PolyBuilder::_substractS(const PathsD& workingClip) {
    if (working_precision.empty() || workingClip.empty()) {
        std::cout << "Debug: working_precision or workingClip is empty\n";
        return PathsD{};
    }

    // 1. Преобразуем наш основной полигон в Path64 (уже есть в working_precision)
    // 2. Преобразуем ВСЕ clip полигоны в Paths64
    Paths64 clipPaths64;
    clipPaths64.reserve(workingClip.size());

    for (const auto& clipPath : workingClip) {
        if (clipPath.size() < 3) {
            std::cout << "Debug: skipping clip path with less than 3 points\n";
            continue;
        }

        // Проверяем и закрываем полигон
        PathD closedClipPath = clipPath;
        if (!isPathClosed(clipPath)) {
            std::cout << "Debug: closing clip path\n";
            closedClipPath.push_back(clipPath[0]);
        }

        // Масштабируем
        Path64 scaledPath = ScalePath<int64_t, double>(closedClipPath, scale_factor, _error_code);
        if (_error_code != 0) {
            std::cout << "Debug: error scaling path: " << _error_code << "\n";
            continue;
        }

        clipPaths64.push_back(scaledPath);
    }

    if (clipPaths64.empty()) {
        std::cout << "Debug: no valid clip paths after processing\n";
        return PathsD{};
    }

    std::cout << "Debug: working_precision size: " << working_precision.size()
              << ", clip paths count: " << clipPaths64.size() << "\n";

    // 3. Выполняем операцию вычитания
    Clipper2Lib::Clipper64 clipper;

    // Добавляем основной полигон (subject)
    clipper.AddSubject({working_precision});

    // Добавляем ВСЕ clip полигоны
    clipper.AddClip(clipPaths64);

    Clipper2Lib::Paths64 solution;
    clipper.Execute(Clipper2Lib::ClipType::Difference,
                    Clipper2Lib::FillRule::EvenOdd,
                    solution);

    std::cout << "Debug: clipper solution count: " << solution.size() << "\n";

    if (solution.empty()) {
        return PathsD{};
    }

    // 4. Преобразуем результат обратно
    return ScalePaths<double, int64_t>(solution, 1.0 / scale_factor, _error_code);
}

QList<QPolygonF> PolyBuilder::subtractedListWrp(const QPolygonF &poly1, const QPolygonF &poly2) {
    QList<QPolygonF> res = {};

    if (poly1.isEmpty() || poly2.isEmpty()) {
        return res;
    }

    // Проверяем, что полигоны замкнуты
    if (poly1.first() != poly1.last()) {
        std::cout << "Warning [poly_build] poly1 is not closed\n";
        return res;
    }

    if (poly2.first() != poly2.last()) {
        std::cout << "Warning [poly_build] poly2 is not closed\n";
        return res;
    }

    q_convert(poly1);

    // Создаем clip полигон
    PathD clip;
    for (const QPointF &point : poly2) {
        clip.push_back(PointD(point.x(), point.y()));
    }

    // Создаем временный builder и устанавливаем геометрию
    PolyBuilder temp;
    temp.setGeometry(clip);

    auto resClipper = _substract(temp.getPrecise());

    for (const auto &onePoly : resClipper) {
        if (onePoly.empty()) continue;

        QPolygonF temPoly;
        for (const auto &p: onePoly) {
            temPoly << QPointF(p.x, p.y);
        }

        // Убедимся, что полигон замкнут в результате
        if (!temPoly.isEmpty() && temPoly.first() != temPoly.last()) {
            temPoly.append(temPoly.first());
        }

        if (temPoly.size() > 2) {  // Минимум 3 точки для полигона
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

/*QPolygonF PolyBuilder::snglIntersctnWrp(const QPolygonF &poly1, const QPolygonF &poly2){
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
}*/

bool PolyBuilder::isPolygonClosed(const QPolygonF& polygon) {
    if (polygon.size() < 2) {
        return false; // Полигон с 0 или 1 точкой не может быть закрыт
    }

    const QPointF& first = polygon.first();
    const QPointF& last = polygon.last();

    // Сравниваем с учетом погрешности для double
    const double epsilon = 1e-6;
    return std::abs(first.x() - last.x()) < epsilon &&
           std::abs(first.y() - last.y()) < epsilon;
}

QPolygonF PolyBuilder::closePolygon(const QPolygonF& polygon) {
    if (polygon.isEmpty()) {
        return QPolygonF();
    }

    if (isPolygonClosed(polygon)) {
        return polygon; // Уже закрыт
    }

    QPolygonF closedPolygon = polygon;
    closedPolygon.append(polygon.first());
    return closedPolygon;
}

bool PolyBuilder::isPathDClosed(const PathD& path) {
    if (path.size() < 2) {
        return false;
    }

    const PointD& first = path[0];
    const PointD& last = path[path.size() - 1];

    const double epsilon = 1e-9;
    return std::abs(first.x - last.x) < epsilon &&
           std::abs(first.y - last.y) < epsilon;
}

PathD PolyBuilder::closePathD(const PathD& path) {
    if (path.empty()) {
        return PathD();
    }

    if (isPathDClosed(path)) {
        return path;
    }

    PathD closedPath = path;
    closedPath.push_back(path[0]);
    return closedPath;
}
