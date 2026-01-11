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

    PathD process() {
        // Все вычисления на int64_t
        performClipperOperations(working_precision);

        // Возвращаем в double
        return ScalePath<double, int64_t>(working_precision, 1.0 / scale_factor, _error_code);
    }

    PathD q_convert(const QPolygonF& in);

    QPolygonF q_getResult();

    QList<QPolygonF> subtractedListWrp(const QPolygonF &poly1, const QList<QPolygonF> &poly2);

    QList<QPolygonF> subtractedListWrp(const QPolygonF &poly1, const QPolygonF &poly2);

    QPolygonF snglIntersctnWrp(const QPolygonF &poly1, const QPolygonF &poly2);

    void performClipperOperations(Path64& inProc);

private:

    PathsD _intersect(const Path64& workingClip);

    PathsD _substract(const Path64& workingClip);
    PathsD _substractS(const PathsD& workingClip);

    // Закрыть QPolygonF (добавить первую точку в конец если не совпадает)
    static QPolygonF closePolygon(const QPolygonF& polygon);

    // Проверить, закрыт ли полигон
    static bool isPolygonClosed(const QPolygonF& polygon);

    // Закрыть PathD (для внутреннего использования)
    static PathD closePathD(const PathD& path);

    // Проверить, закрыт ли PathD
    static bool isPathDClosed(const PathD& path);

    // Для хранения - double (максимальная точность)
    PathD storage_precision;

    // Для вычислений - int64_t (максимальная производительность)
    Path64 working_precision;

    double scale_factor = 10000.0;  // сохраняем 4 знака после запятой

    int _error_code = 0;

};

#endif //DECOMPOSERLIB_POLYBUILDER_H
