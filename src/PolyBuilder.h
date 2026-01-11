//
// Created by Admin on 11.01.2026.
//

#ifndef DECOMPOSERLIB_POLYBUILDER_H
#define DECOMPOSERLIB_POLYBUILDER_H

#include <QPolygonF>
#include "clipper2/clipper.h"

using namespace Clipper2Lib;

namespace PolyOp{

    static QPolygonF highPrecisionIntersected(const QPolygonF &poly1, const QPolygonF &poly2) {
        if (poly1.isEmpty() || poly2.isEmpty()) {
            return QPolygonF();
        }

        // Конвертируем QPolygonF в Clipper2 Path64 с высокой точностью
        Clipper2Lib::Path64 subject;
        for (const QPointF &point : poly1) {
            subject.push_back(Clipper2Lib::Point64(
                    static_cast<int64_t>(std::round(point.x() * 10000.0)),  // Масштабируем x1000
                    static_cast<int64_t>(std::round(point.y() * 10000.0))
            ));
        }

        Clipper2Lib::Path64 clip;
        for (const QPointF &point : poly2) {
            clip.push_back(Clipper2Lib::Point64(
                    static_cast<int64_t>(std::round(point.x() * 10000.0)),
                    static_cast<int64_t>(std::round(point.y() * 10000.0))
            ));
        }

        // Выполняем операцию пересечения
        Clipper2Lib::Clipper64 clipper;
        clipper.AddSubject({subject});
        clipper.AddClip({clip});

        Clipper2Lib::Paths64 solution;
        clipper.Execute(Clipper2Lib::ClipType::Intersection,
                        Clipper2Lib::FillRule::EvenOdd,
                        solution);

        if (solution.empty()) {
            return QPolygonF();
        }

        // Находим самый большой полигон по площади
        double maxArea = -1.0;
        Clipper2Lib::Path64 largestPath;

        for (const auto &path : solution) {
            double area = std::abs(Clipper2Lib::Area(path));
            if (area > maxArea) {
                maxArea = area;
                largestPath = path;
            }
        }

        // Конвертируем обратно в QPolygonF
        QPolygonF result;
        for (const auto &point : largestPath) {
            result << QPointF(
                    static_cast<double>(point.x) / 10000.0,
                    static_cast<double>(point.y) / 10000.0
            );
        }

        if (!result.isEmpty() && result.first() != result.last()) {
            result.append(result.first());
        }

        return result;
    }

}

class PolyBuilder{

    PolyBuilder() = default;
    ~PolyBuilder() = default;

public:
    void setGeometry(const PathsD& geometry) {
        // Сохраняем оригинал
        storage_precision = geometry;

        // Конвертируем для вычислений
        int error_code = 0;
        working_precision = ScalePaths<int64_t, double>(geometry, scale_factor, error_code);
    }

    PathsD process() {
        // Все вычисления на int64_t
        performClipperOperations(working_precision);

        // Возвращаем в double
        int error_code = 0;
        return ScalePaths<double, int64_t>(working_precision, 1.0 / scale_factor, error_code);
    }

    void q_convert(const QPolygonF& in);

    void performClipperOperations(Paths64& inProc);

private:

    // Для хранения - double (максимальная точность)
    PathsD storage_precision;

    // Для вычислений - int64_t (максимальная производительность)
    Paths64 working_precision;

    double scale_factor = 1000.0;  // сохраняем 3 знака после запятой

};

#endif //DECOMPOSERLIB_POLYBUILDER_H
