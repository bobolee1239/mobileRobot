//  Copyright 2019 Tsung-Han lee
#include <iomanip>
#include "../include/a_star.h"


int main(int argc, char* argv[]) {
/*
    std::vector<int8_t> rawMap({100, 50, 0, -1, 20});

    auto print_int8 = [](int8_t& e){std::cout << (short)e << std::endl;};
    auto print_double = [](double& e){std::cout << e << std::endl;};

    std::for_each(rawMap.begin(), rawMap.end(), print_int8);

    std::vector<double> map;
    std::for_each(rawMap.begin(), rawMap.end(), [&map](int8_t& e){
        if (e > 0) {
            map.push_back((double)e / MAP_OCCUPIED);
        } else {
            map.push_back(e);
        }
    });
    std::for_each(map.begin(), map.end(), print_double);
*/
    /* Testing Map */
    /* case 1 easy map */
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

    return 0;
}
