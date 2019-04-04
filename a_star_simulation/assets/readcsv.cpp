//  Copyright 2019 Brian
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        cout << "Usage: ./readcsv [filename]" << endl;
        return -1;
    }

    vector<int8_t> matrix;
    fstream file;
    file.open(argv[1]);
    string line;
    while (getline(file, line, '\n'))  {
        istringstream templine(line);  // string 轉換成 stream
        string data;
        while (getline(templine, data, ',')) {
          matrix.push_back(static_cast<int8_t>(atoi(data.c_str())));  // string 轉換成數字
        }
    }
    file.close();

    for (auto&& e : matrix) {
        cout << (int)e << ", ";
    }


    return 0;
}
