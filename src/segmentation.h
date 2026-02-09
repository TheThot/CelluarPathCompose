//
// Created by Admin on 09.02.2026.
//

#include <vector>

#ifndef DECOMPOSERLIB_SEGMENTATION_H
#define DECOMPOSERLIB_SEGMENTATION_H

using namespace std;

namespace segmentation {
    template<typename Path>
    class SegmentationBase {

        explicit SegmentationBase() = default;

        ~SegmentationBase() = default;

        virtual void setSurvPolyPath(const Path &inPath) = 0;

        int getDronesNum() const;

        Path getDronePath(int pos);

    private:

        virtual void _generatePathRespectEachDrone() = 0;

        struct _droneInfo {
            double _chargePercent;
            double _current; // Ампер
            double _cap;     // Ампер часы
            double _velocity;
            double _altitude;  // Высота над уровнем моря
            double _height;    // Высота над поверхностью
            double _windSpeed;
            double _payload;   // В случае агродрона полезная нагрузка - количество литров в баке
        };

        vector<_droneInfo> _listDrones;
        Path _wholePath;     // весь путь поля интереса
        vector<_droneInfo> _pathParts;     // на каждый дрон свой путь

    };

    //Реализация
    template<typename Path>
    class Segmentation : public SegmentationBase<Path> {
        explicit Segmentation() = default;
        ~Segmentation() = default;

        void setSurvPolyPath(const Path &inPath);
    private:
        void _generatePathRespectEachDrone();
    };
}

#endif //DECOMPOSERLIB_SEGMENTATION_H
