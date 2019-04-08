//  Copyright 2019 Tsung-Han lee
#include <iomanip>
#include <fstream>
#include "../include/a_star.h"
#include "readcsv.h"

#define MAP_PATH "../assets/map.csv"

int main(int argc, char* argv[]) {
    int map_size = 10;
    std::vector<int8_t> rawMap;
    for (int i = 0; i < map_size; ++i) {
        for (int j = 0; j < map_size; ++j) {
            if ((j == 4) && (i > 2 && i < 8)) {
                rawMap.push_back(100);
            } else if ((i <= 3) && (j == 5)) {
                rawMap.push_back(100);
            } else {
                rawMap.push_back(0);
            }
        }
    }
    Map myMap(rawMap, map_size, map_size);
    std::cout << "rawMap size: " << rawMap.size() << std::endl;
    std::cout << myMap << std::endl;

    /**
     **  ---------- CASE 1 -----------
     **/
    std::cout << "\n---------- CASE 1 ----------\n" << std::endl;

    auto path = myMap.moveRobotTo(1, 1).aStar(MapNode(6, 5));
    for (auto&& node : path) {
        std::cout << static_cast<Node>(node) << " -> ";
    }
    std::cout << std::endl;

    std::cout << myMap << std::endl;
    auto closeList = myMap.getCloseList();
    for (int i = map_size - 1; i >= 0; --i) {
        for (int j = 0; j < map_size; ++j) {
            std::cout << closeList[i*map_size + j] << " | ";
        }
        std::cout << std::endl;
    }

    /**
     **  ---------- CASE 2 -----------
     **/
    std::cout << "\n---------- CASE 2 ----------\n" << std::endl;

    myMap.setResolution(0.1);
    myMap.setOrigin(-0.5, -0.5);
    path = myMap.moveRobotTo(-0.1, -0.3).aStar(Node(0.1, 0));
    // path = myMap.moveRobotTo(4, 2).aStar(MapNode(6, 5));
    for (auto&& node : path) {
        std::cout << static_cast<Node>(node) << " -> ";
    }
    std::cout << std::endl;

    std::cout << myMap << std::endl;
    closeList = myMap.getCloseList();
    for (int i = map_size - 1; i >= 0; --i) {
        for (int j = 0; j < map_size; ++j) {
            std::cout << closeList[i*map_size + j] << " | ";
        }
        std::cout << std::endl;
    }


    /**
     **  ---------- CASE 3 -----------
     **/
    std::cout << "\n---------- CASE 3 ----------\n" << std::endl;
    std::vector<int8_t> rawLabMap = readMap(MAP_PATH);
    Map labMap(rawLabMap, 100, 100);

    path = labMap.moveRobotTo(3, 2).aStar(Node(3, 4));
    for (auto&& node : path) {
        std::cout << static_cast<Node>(node) << " -> ";
    }
    std::cout << std::endl;

    path = labMap.moveRobotTo(3, 4).aStar(Node(3, 10));
    for (auto&& node : path) {
        std::cout << static_cast<Node>(node) << " -> ";
    }
    std::cout << std::endl;

    path = labMap.moveRobotTo(3, 10).aStar(Node(9, 4));
    for (auto&& node : path) {
        std::cout << static_cast<Node>(node) << " -> ";
    }
    std::cout << std::endl;
    return 0;
}
