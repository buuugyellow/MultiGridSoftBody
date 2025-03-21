#include <fstream>
#include <iostream>
#include <vector>

#include "global.h"

using namespace std;

void OutputPosNormIndex(string filepath, std::vector<float> pos = std::vector<float>(), std::vector<float> norm = std::vector<float>(),
                        std::vector<unsigned int> index = std::vector<unsigned int>()) {
    ofstream out(filepath);
    if (!out) {
        cout << "OutputPosNormIndex : open file " << filepath << " failed " << endl;
        return;
    }
    for (auto iter = pos.begin(); iter != pos.end();) {
        out << "v " << *(iter) << " ";
        iter++;
        out << *(iter) << " ";
        iter++;
        out << *(iter) << endl;
        iter++;
    }
    for (auto iter = norm.begin(); iter != norm.end();) {
        out << "vn " << *(iter) << " ";
        iter++;
        out << *(iter) << " ";
        iter++;
        out << *(iter) << endl;
        iter++;
    }
    for (auto iter = index.begin(); iter != index.end();) {
        out << "f " << *(iter) + 1 << " ";
        iter++;
        out << *(iter) + 1 << " ";
        iter++;
        out << *(iter) + 1 << endl;
        iter++;
    }
}