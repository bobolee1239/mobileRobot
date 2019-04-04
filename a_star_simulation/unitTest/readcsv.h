//  Copyright 2019 Brian
#ifndef _READ_MAP_
#define _READ_MAP_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

std::vector<int8_t> readMap(const char* filename) {
    std::vector<int8_t> map;
    std::fstream file;
    file.open(filename);
    std::string line;
    while (getline(file, line, '\n'))  {
        std::istringstream templine(line);  // string 轉換成 stream
        std::string data;
        while (getline(templine, data, ',')) {
          map.push_back(static_cast<int8_t>(atoi(data.c_str())));  // string 轉換成數字
        }
    }
    file.close();

    return map;
}

#endif  // _READ_MAP_
