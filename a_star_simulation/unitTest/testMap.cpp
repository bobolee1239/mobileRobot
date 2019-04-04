//  Copyright 2019 Tsung-Han lee
#include <iomanip>
#include <fstream>
#include "../include/a_star.h"
#include "readcsv.h"

int main(int argc, char* argv[]) {
    int map_size = 100;
    std::vector<int8_t> rawMap;
    for (int i=0; i<map_size; ++i) {
        for (int j=0; j<map_size; ++j) {
            if ((i == 7) && (j == 2)) {
                rawMap.push_back(100);
            } else if ((i == 2) && (j == 5)) {
                rawMap.push_back(100);
            } else {
                rawMap.push_back(0);
            }
        }
    }
    std::cout << "rawMap size: " << rawMap.size() << std::endl;
    Map myMap(rawMap, map_size, map_size);


    /**
     **  ---------- CASE 1 -----------
     **/
    std::cout << "\n---------- CASE 1 ----------\n" << std::endl;

    auto path = myMap.moveRobotTo(2, 2).aStar(MapNode(3, 3));
    for (auto&& node : path) {
        std::cout << static_cast<Node>(node) << " -> ";
    }
    std::cout << std::endl;


    // std::cout << myMap.getRobot() << std::endl;
    // std::cout << myMap << std::endl;
    auto closeList = myMap.getCloseList();
    std::cout << closeList.size() << std::endl;
    /*
    for (int i = map_size; i >= 0; --i) {
        for (int j = 0; j < map_size; ++j) {
            std::cout << closeList[i*map_size + j] << " | ";
        }
        std::cout << std::endl;
    }
    */

    /**
     **  ---------- CASE 2 -----------
     **/
    std::cout << "\n---------- CASE 2 ----------\n" << std::endl;

    path = myMap.moveRobotTo(2, 2).aStar(MapNode(90, 90));
    for (auto&& node : path) {
        std::cout << static_cast<Node>(node) << " -> ";
    }
    std::cout << std::endl;
    // std::cout << myMap << std::endl;
    closeList = myMap.getCloseList();
    std::cout << closeList.size() << std::endl;
    /*
    for (int i = map_size; i >= 0; --i) {
        for (int j = 0; j < map_size; ++j) {
            std::cout << closeList[i*map_size + j] << " | ";
        }
        std::cout << std::endl;
    }
    */

    /**
     **  ---------- CASE 3 -----------
     **/
    std::cout << "\n---------- CASE 3 ----------\n" << std::endl;
    std::vector<int8_t> rawLabMap = readMap("/home/tea_lab/catkin_ws/src/a_star_simulation/assets/map.csv");
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
