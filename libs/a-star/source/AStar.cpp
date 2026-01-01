#include <iostream>
#include "AStar.hpp"

using namespace AStar;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_) const {
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Node::Node(Vec2i coordinates_, Node* parent_) {
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

uint AStar::Node::getScore() const {
    return G + H;
}

AStar::Generator::Generator() {
    allowDiagonal = false;
    allowBorderMovement = false;  // По умолчанию запрещено
    setHeuristic(&Heuristic::manhattan);
    direction = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0},
            {-1, -1}, {1, 1}, {-1, 1}, {1, -1}
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_) {
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_) {
    allowDiagonal = enable_;
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_) {
    heuristic = heuristic_;
}

void AStar::Generator::addCollision(Vec2i coordinates_) {
    // Проверяем, что препятствие внутри границ (если движение вдоль границ разрешено)
    if (!allowBorderMovement) {
        if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
            coordinates_.y < 0 || coordinates_.y >= worldSize.y) {
            return;  // Игнорируем препятствия вне границ
        }
    }
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_) {
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions() {
    walls.clear();
}


void AStar::Generator::allowMovementAlongBorders(bool allow) {
    allowBorderMovement = allow;
}

// проверка выхода за границы
bool AStar::Generator::isOutOfBounds(Vec2i coordinates_) const {
    if (allowBorderMovement) {
        // Разрешаем движение ПО границе, но не ЗА границу
        // Граничные клетки: x=0, x=worldSize.x-1, y=0, y=worldSize.y-1
        return (coordinates_.x < 0 || coordinates_.x > worldSize.x - 1 ||
                coordinates_.y < 0 || coordinates_.y > worldSize.y - 1);
    } else {
        // Стандартная проверка (нельзя быть на границе)
        return (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
                coordinates_.y < 0 || coordinates_.y >= worldSize.y);
    }
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_) {
    // Проверяем, находятся ли точки в допустимых пределах
    if (isOutOfBounds(source_) || isOutOfBounds(target_)) {
        std::cout << "Warning strat/finish point in out of bounds\n";
        return CoordinateList();
    }

    Node* current = nullptr;
    std::vector<Node*> openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    std::map<long, long> wallsMap;
    buildWallsMap(wallsMap);

    // ВАЖНО: проверяем, не являются ли начальная/конечная точки препятствиями
    // но учитываем, что на границах могут быть допустимые стартовые точки
    long sourceIndex = coordinateToMapIndex(source_);
    long targetIndex = coordinateToMapIndex(target_);

    if (wallsMap.find(sourceIndex) != wallsMap.end()) {
        std::cout << "Warning strat point on obstacle\n";
        releaseNodes(openSet);
        return CoordinateList();
    }

    if (wallsMap.find(targetIndex) != wallsMap.end()) {
        std::cout << "Warning finish point on obstacle\n";
        releaseNodes(openSet);
        return CoordinateList();
    }

    CoordinateList path;
    bool pathFound = false;

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            auto node = *it;
            if (node->getScore() < current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) {
            pathFound = true;
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates = current->coordinates + direction[i];

            if (detectCollision(wallsMap, newCoordinates, i) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);
            Node* successor = findNodeOnList(openSet, newCoordinates);

            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            } else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    // Собираем путь если он найден
    if (pathFound) {
        while (current != nullptr) {
            path.push_back(current->coordinates);
            current = current->parent;
        }
        std::reverse(path.begin(), path.end());
    }

    // Освобождаем всю память
    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(std::vector<Node*>& nodes_, Vec2i coordinates_) {
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(std::vector<Node*>& nodes_) {
    for (auto node : nodes_) {
        delete node;
    }
    nodes_.clear();
}

void AStar::Generator::buildWallsMap(std::map<long, long>& wallsMap) {
    wallsMap.clear();
    for (auto coordinate : walls) {
        // Если разрешено движение вдоль границ, добавляем все препятствия
        // Иначе фильтруем те, что вне границ
        if (!allowBorderMovement) {
            if (coordinate.x < 0 || coordinate.x >= worldSize.x ||
                coordinate.y < 0 || coordinate.y >= worldSize.y) {
                continue;
            }
        }
        long mapIndex = coordinateToMapIndex(coordinate);
        wallsMap[mapIndex] = mapIndex;
    }
}

long AStar::Generator::coordinateToMapIndex(Vec2i coordinates_) {
    long a = coordinates_.x;
    long b = coordinates_.y;
    long hash = (a >= b) ? (a * a + a + b) : (a + b * b);
    return hash;
}

void AStar::Generator::visualizePathColor(const CoordinateList& path, Vec2i source_, Vec2i target_) const {
    // ANSI коды цветов
    const std::string COLOR_RESET = "\033[0m";
    const std::string COLOR_RED = "\033[31m";
    const std::string COLOR_GREEN = "\033[32m";
    const std::string COLOR_YELLOW = "\033[33m";
    const std::string COLOR_BLUE = "\033[34m";
    const std::string COLOR_MAGENTA = "\033[35m";
    const std::string COLOR_CYAN = "\033[36m";

    std::vector<std::vector<char>> grid(worldSize.y, std::vector<char>(worldSize.x, '.'));
    std::vector<std::vector<std::string>> colors(worldSize.y, std::vector<std::string>(worldSize.x, COLOR_RESET));

    // Препятствия - красные
    for (const auto& wall : walls) {
        if (wall.x >= 0 && wall.x < worldSize.x && wall.y >= 0 && wall.y < worldSize.y) {
            grid[wall.y][wall.x] = '#';
            colors[wall.y][wall.x] = COLOR_RED;
        }
    }

    // Путь - синий
    for (size_t i = 0; i < path.size(); ++i) {
        const auto& coord = path[i];
        if (coord.x >= 0 && coord.x < worldSize.x && coord.y >= 0 && coord.y < worldSize.y) {
            if (i == 0) {
                grid[coord.y][coord.x] = 'S';
                colors[coord.y][coord.x] = COLOR_GREEN;
            } else if (i == path.size() - 1) {
                grid[coord.y][coord.x] = 'E';
                colors[coord.y][coord.x] = COLOR_GREEN;
            } else {
                grid[coord.y][coord.x] = '*';
                colors[coord.y][coord.x] = COLOR_BLUE;
            }
        }
    }

    std::cout << "\n   ";
    for (int x = 0; x < worldSize.x; ++x) {
        std::cout << (x % 10);
    }
    std::cout << "\n   ";
    for (int x = 0; x < worldSize.x; ++x) {
        std::cout << "-";
    }
    std::cout << "\n";

    for (int y = 0; y < worldSize.y; ++y) {
        std::cout << (y % 10) << "| ";
        for (int x = 0; x < worldSize.x; ++x) {
            std::cout << colors[y][x] << grid[y][x] << COLOR_RESET;
        }
        std::cout << "\n";
    }
}

// НОВАЯ ФУНКЦИЯ: Визуализация пути
void AStar::Generator::visualizePath(const CoordinateList& path, Vec2i source_, Vec2i target_) const {
    // Создаем карту мира
    std::vector<std::vector<std::string>> grid(worldSize.y, std::vector<std::string>(worldSize.x, "."));

    // Отмечаем препятствия
    for (const auto& wall : walls) {
        if (wall.x >= 0 && wall.x < worldSize.x && wall.y >= 0 && wall.y < worldSize.y) {
            grid[wall.y][wall.x] = '#';  // Препятствие
        }
    }

    // Отмечаем путь (кроме начальной и конечной точек)
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const auto& coord = path[i];
        if (coord.x >= 0 && coord.x < worldSize.x && coord.y >= 0 && coord.y < worldSize.y) {
            // Определяем направление движения для красивых стрелок
            if (i > 0) {
                const auto& prev = path[i-1];
                const auto& next = path[i+1];

                int dx1 = coord.x - prev.x;
                int dy1 = coord.y - prev.y;
                int dx2 = next.x - coord.x;
                int dy2 = next.y - coord.y;

                std::string symbol = "·";  // точка по умолчанию

                // Горизонтальное движение
                if ((dx1 == 1 && dx2 == 1) || (dx1 == -1 && dx2 == -1)) symbol = "─";
                    // Вертикальное движение
                else if ((dy1 == 1 && dy2 == 1) || (dy1 == -1 && dy2 == -1)) symbol = "│";
                    // Углы
                else if ((dx1 == 1 && dy2 == 1) || (dy1 == 1 && dx2 == 1)) symbol = "┌";
                else if ((dx1 == -1 && dy2 == 1) || (dy1 == 1 && dx2 == -1)) symbol = "┐";
                else if ((dx1 == 1 && dy2 == -1) || (dy1 == -1 && dx2 == 1)) symbol = "└";
                else if ((dx1 == -1 && dy2 == -1) || (dy1 == -1 && dx2 == -1)) symbol = "┘";
                    // Перекрестки
                else if (abs(dx1) + abs(dx2) + abs(dy1) + abs(dy2) > 2) symbol = "┼";

                grid[coord.y][coord.x] = symbol;
            }
        }
    }

    // Отмечаем начальную и конечную точки
    if (source_.x >= 0 && source_.x < worldSize.x && source_.y >= 0 && source_.y < worldSize.y) {
        grid[source_.y][source_.x] = 'S';  // Start
    }
    if (target_.x >= 0 && target_.x < worldSize.x && target_.y >= 0 && target_.y < worldSize.y) {
        grid[target_.y][target_.x] = 'E';  // End
    }

    // Выводим координатную сетку
    std::cout << "\n   ";
    for (int x = 0; x < worldSize.x; ++x) {
        std::cout << (x % 10);
    }
    std::cout << "\n   ";
    for (int x = 0; x < worldSize.x; ++x) {
        std::cout << "-";
    }
    std::cout << "\n";

    // Выводим карту
    for (int y = 0; y < worldSize.y; ++y) {
        std::cout << (y % 10) << "| ";
        for (int x = 0; x < worldSize.x; ++x) {
            std::cout << grid[y][x];
        }
        std::cout << "\n";
    }

    // Легенда
    std::cout << "\nЛегенда:\n";
    std::cout << "  S - Начальная точка\n";
    std::cout << "  E - Конечная точка\n";
    std::cout << "  # - Препятствие\n";
    std::cout << "  · - Путь\n";
    std::cout << "  ─│┌┐└┘┼ - Направление пути\n";

    // Статистика
    std::cout << "\nСтатистика:\n";
    std::cout << "  Длина пути: " << path.size() << " шагов\n";
    std::cout << "  Препятствий: " << walls.size() << "\n";

    // Координаты пути
    if (!path.empty()) {
        std::cout << "\nКоординаты пути:\n";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << "  " << i << ": (" << path[i].x << ", " << path[i].y << ")";
            if (path[i].x == source_.x && path[i].y == source_.y) std::cout << " [START]";
            if (path[i].x == target_.x && path[i].y == target_.y) std::cout << " [END]";
            std::cout << "\n";
        }
    } else {
        std::cout << "\nПуть не найден!\n";
    }
}

// Методы Heuristic
AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_) {
    return{ abs(source_.x - target_.x), abs(source_.y - target_.y) };
}

uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_) {
    auto delta = getDelta(source_, target_);
    return static_cast<uint>(10 * (delta.x + delta.y));
}

uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_) {
    auto delta = getDelta(source_, target_);
    return static_cast<uint>(10 * std::sqrt(delta.x * delta.x + delta.y * delta.y));
}

uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_) {
    auto delta = getDelta(source_, target_);
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}


bool AStar::Generator::detectCollision(const std::map<long, long>& wallsMap, Vec2i coordinates_, int directionIndex) {
    // Проверка границ
    if (isOutOfBounds(coordinates_)) {
        return true;
    }

    long mapIndex = coordinateToMapIndex(coordinates_);
    auto it = wallsMap.find(mapIndex);
    if (it != wallsMap.end()) {
        return true;
    }

    // Дополнительная проверка для диагональных ходов
    if (allowDiagonal && directionIndex >= 4) {
        // Для диагональных направлений (индексы 4-7)
        // Проверяем, что обе соседние клетки тоже свободны

        // Направления диагоналей:
        // 4: {-1, -1} ↖  // Нужно проверить {-1, 0} ← и {0, -1} ↑
        // 5: {1, 1}   ↘  // Нужно проверить {1, 0} → и {0, 1} ↓
        // 6: {-1, 1}  ↙  // Нужно проверить {-1, 0} ← и {0, 1} ↓
        // 7: {1, -1}  ↗  // Нужно проверить {1, 0} → и {0, -1} ↑

        Vec2i neighbor1, neighbor2;

        switch (directionIndex) {
            case 4: // ↖
                neighbor1 = {coordinates_.x + 1, coordinates_.y};  // →
                neighbor2 = {coordinates_.x, coordinates_.y + 1};  // ↓
                break;
            case 5: // ↘
                neighbor1 = {coordinates_.x - 1, coordinates_.y};  // ←
                neighbor2 = {coordinates_.x, coordinates_.y - 1};  // ↑
                break;
            case 6: // ↙
                neighbor1 = {coordinates_.x + 1, coordinates_.y};  // →
                neighbor2 = {coordinates_.x, coordinates_.y - 1};  // ↑
                break;
            case 7: // ↗
                neighbor1 = {coordinates_.x - 1, coordinates_.y};  // ←
                neighbor2 = {coordinates_.x, coordinates_.y + 1};  // ↓
                break;
        }

        // Проверяем соседние клетки на препятствия
        // Но НЕ проверяем на выход за границы для соседних клеток
        // при проверке диагональных углов
        long mapIndex1 = coordinateToMapIndex(neighbor1);
        long mapIndex2 = coordinateToMapIndex(neighbor2);

        if (wallsMap.find(mapIndex1) != wallsMap.end() ||
            wallsMap.find(mapIndex2) != wallsMap.end()) {
            return true;
        }
    }

    return false;
}
