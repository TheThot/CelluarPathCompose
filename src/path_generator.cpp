//
// Created by Admin on 19.12.2025.
//
#include "path_generator.h"
#define _USE_MATH_DEFINES
#include <cmath>

PathGenerator::PathGenerator(QPolygonF& inPolygon) :
                            _survPolygon(inPolygon)
{

}

PathGenerator::~PathGenerator()
{
    _path.clear();
}

QVariantList PathGenerator::pathTraj() const
{
    QVariantList pathTraj;

    for (const auto& path : _path)
    {
        pathTraj.append(QVariant::fromValue(path));
    }

    return pathTraj;
}