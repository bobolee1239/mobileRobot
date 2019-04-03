//  Copyright 2019 Tsung-Han lee
#include <iostream>
#include <vector>
#include <algorithm>
#include "../include/a_star.h"

int main() {
    MapNode tmp[6];
    tmp[0] = MapNode(0, 1);
    tmp[1] = MapNode(1, 1, &tmp[0]);
    tmp[2] = MapNode(0, 2, &tmp[0]);
    tmp[3] = MapNode(2, 2, &tmp[0]);
    tmp[4] = MapNode(0, 3, &tmp[0]);
    tmp[5] = MapNode(3, 3, &tmp[0]);

    tmp[0].setFCost(6);
    tmp[1].setFCost(1);
    tmp[2].setFCost(2);
    tmp[3].setFCost(5);
    tmp[4].setFCost(3);
    tmp[5].setFCost(4);

    std::vector<MapNode> map(tmp, tmp+6);
    auto comparision = std::greater<MapNode>();

    std::make_heap(map.begin(), map.end(), comparision);

    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }

    MapNode n1(5, 5, &tmp[1]);
    n1.setFCost(0);

    map.push_back(n1);
    std::cout << "-----------------------------------------" << std::endl;
    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }

    push_heap(map.begin(), map.end(), comparision);
    std::cout << "-----------------------------------------" << std::endl;
    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }

    /* modify a node and reheap */
    map[2].setFCost(100);
    std::make_heap(map.begin(), map.end(), comparision);
    std::cout << "-----------------------------------------" << std::endl;
    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }

    /* pop from heap */
    std::pop_heap(map.begin(), map.end(), comparision);
    std::cout << "-----------------------------------------" << std::endl;
    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }

    auto min = map.back();
    map.pop_back();
    std::cout << "Min: " << min << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }


    for (auto&& n : map) {
        if (n.getX() == 1 && n.getY() == 1) {
            n.setFCost(1);
        }
    }
    std::cout << "-----------------------------------------" << std::endl;
    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }
    std::make_heap(map.begin(), map.end(), comparision);
    std::cout << "-----------------------------------------" << std::endl;
    for (auto&& n : map) {
        std::cout << n << " -> \n";
    }

    return 0;
}
