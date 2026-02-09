//
// Created by Admin on 09.02.2026.
//
#include "segmentation.h"

#include <iostream>

using namespace segmentation;

template<typename PathType> int SegmentationBase<PathType>::getDronesNum() const{
    return _listDrones.size();
}

template<typename PathType> PathType SegmentationBase<PathType>::getDronePath(int pos) {
    return _pathParts[pos];
}

template<typename PathType> void Segmentation<PathType>::setSurvPolyPath(const PathType& inPath){
    std::cout << "Setting original polygon path" << std::endl;
    /*устанавливаем путь*/
    _wholePath = inPath;
}

template<typename PathType> void Segmentation<PathType>::_generatePathRespectEachDrone(){
    /*выполняем генерацию каждого пути отдельно*/
    std::cout << "Generation process..." << std::endl;
}