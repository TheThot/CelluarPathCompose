/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/

#include <vector>
#include <functional>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <memory>

namespace AStar {
    struct Vec2i {
        int x, y;
        bool operator == (const Vec2i& coordinates_) const;
        friend Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
            return{ left_.x + right_.x, left_.y + right_.y };
        }
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    class Node {
    public:
        Vec2i coordinates;
        Node* parent;
        uint G, H;

        Node(Vec2i coordinates_, Node* parent_ = nullptr);
        uint getScore() const;
    };

    class Generator {
        void removeCollision(Vec2i coordinates_);
        bool isOutOfBounds(Vec2i coordinates_) const;
        bool detectCollision(const std::map<long, long>& wallsMap, Vec2i coordinates_, int directionIndex = -1);
        Node* findNodeOnList(std::vector<Node*>& nodes_, Vec2i coordinates_);
        void releaseNodes(std::vector<Node*>& nodes_);
        void buildWallsMap(std::map<long, long>& wallsMap);
        long coordinateToMapIndex(Vec2i coordinates_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        void addCollision(Vec2i coordinates_);
        void clearCollisions();
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void allowMovementAlongBorders(bool allow);

        // Новая функция для визуализации
        void visualizePath(const CoordinateList& path, Vec2i source_, Vec2i target_) const;
        void visualizePathColor(const CoordinateList& path, Vec2i source_, Vec2i target_) const;

    private:
        HeuristicFunction heuristic;
        CoordinateList direction;
        CoordinateList walls;
        Vec2i worldSize;
        uint directions;
        bool allowDiagonal;
        bool allowBorderMovement;
    };

    class Heuristic {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);
    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}
