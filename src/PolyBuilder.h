//
// Created by Admin on 11.01.2026.
//

#ifndef DECOMPOSERLIB_POLYBUILDER_H
#define DECOMPOSERLIB_POLYBUILDER_H

#include <QPolygonF>
#include <QList>
#include "clipper2/clipper.h"

using namespace Clipper2Lib;

class PolyBuilder{

public:

    PolyBuilder() = default;
    ~PolyBuilder() = default;

    void setGeometry(const PathD& geometry) {
        // Сохраняем оригинал
        storage_precision = geometry;

        // Конвертируем для вычислений
        working_precision = ScalePath<int64_t, double>(geometry, scale_factor, _error_code);
    }

    Path64 getPrecise(){
        return working_precision;
    }

    double getScale() const { return scale_factor; }

    PathD process() {
        // Все вычисления на int64_t
        performClipperOperations(working_precision);

        // Возвращаем в double
        return ScalePath<double, int64_t>(working_precision, 1.0 / scale_factor, _error_code);
    }

    PathD q_convert(const QPolygonF& in);

    QPolygonF q_getResult();

    QPolygonF offsetWrp(const QPolygonF &poly, int offset);

    QList<QPolygonF> unitedListWrp(const QList<QPolygonF> &poly2, int offset = 10);

    QList<QPolygonF> subtractedListWrp(const QPolygonF &poly1, const QList<QPolygonF> &poly2);

    QList<QPolygonF> subtractedListWrp(const QPolygonF &poly1, const QPolygonF &poly2, bool doOffset = true);

    QPolygonF snglIntersctnWrp(const QPolygonF &poly1, const QPolygonF &poly2);

    void performClipperOperations(Path64& inProc);

private:

    PathsD _intersect(const Path64& workingClip);
    bool _hasIntersectionBounds(const Path64& path1, const Path64& path2);

    PathsD _substract(const Path64& workingClip, bool doOffset);
    PathsD _substractS(const PathsD& workingClip);

    Paths64 _offsetPolygon(const Path64& polygons, double delta);
    Paths64 _offsetPolygons(const Paths64& polygons, double delta);
    PathsD _union(const PathsD& workingClip, int offset);

    // Для хранения - double (максимальная точность)
    PathD storage_precision;

    // Для вычислений - int64_t (максимальная производительность)
    Path64 working_precision;

    double scale_factor = 10000.0;  // сохраняем 4 знака после запятой

    int _error_code = 0;

};

#endif //DECOMPOSERLIB_POLYBUILDER_H
