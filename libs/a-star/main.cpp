#include <iostream>
#include "AStar.hpp"

int main()
{
    AStar::Generator generator;
    generator.setWorldSize({25, 25});
    generator.setHeuristic(AStar::Heuristic::manhattan);
    generator.setDiagonalMovement(true);

    // Добавляем различные препятствия
    std::cout << "=== Тест 1: Диагональная стена ===\n";
    int j = 20;
    for(int i = 0; i < j; ++i) {
        if(i != j-5)
            generator.addCollision({i, j - 1 - i});
    }

    // Добавляем горизонтальную стену
    /*for(int i = 5; i < 20; ++i) {
        generator.addCollision({i, 15});
    }

    // Добавляем вертикальную стену
    for(int i = 5; i < 18; ++i) {
        generator.addCollision({12, i});
    }

    // Добавляем несколько случайных препятствий
    generator.addCollision({3, 3});
    generator.addCollision({4, 3});
    generator.addCollision({5, 3});
    generator.addCollision({7, 7});
    generator.addCollision({8, 8});
    generator.addCollision({9, 9});*/

    // Ищем путь
    std::cout << "Поиск пути из (0,0) в (20,20)...\n";
    auto path = generator.findPath({0, 0}, {20, 20});

    // Визуализируем результат
    //generator.visualizePath(path, {0, 0}, {20, 20});
    generator.visualizePathColor(path, {0, 0}, {20, 20});

    // Тест 2: Непроходимый путь
    std::cout << "\n\n=== Тест 2: Полностью заблокированный путь ===\n";
    generator.clearCollisions();

    // Создаем непроходимую стену
    for(int i = 0; i < 25; ++i) {
        generator.addCollision({10, i});
    }

    auto path2 = generator.findPath({0, 0}, {20, 0});
    //generator.visualizePath(path2, {0, 0}, {20, 0});
    generator.visualizePathColor(path2, {0, 0}, {20, 0});

    // Тест 3: Простой путь без препятствий
    std::cout << "\n\n=== Тест 3: Путь без препятствий ===\n";
    generator.clearCollisions();

    auto path3 = generator.findPath({2, 2}, {22, 22});
    //generator.visualizePath(path3, {2, 2}, {22, 22});
    generator.visualizePathColor(path3, {2, 2}, {22, 22});

    return 0;
}