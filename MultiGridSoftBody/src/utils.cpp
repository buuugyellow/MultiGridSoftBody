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

float Matrix_Inverse_3(float* A, float* R)  
{
    R[0] = A[4] * A[8] - A[7] * A[5];
    R[1] = A[7] * A[2] - A[1] * A[8];
    R[2] = A[1] * A[5] - A[4] * A[2];
    R[3] = A[5] * A[6] - A[3] * A[8];
    R[4] = A[0] * A[8] - A[2] * A[6];
    R[5] = A[2] * A[3] - A[0] * A[5];
    R[6] = A[3] * A[7] - A[4] * A[6];
    R[7] = A[1] * A[6] - A[0] * A[7];
    R[8] = A[0] * A[4] - A[1] * A[3];
    float det = A[0] * R[0] + A[3] * R[1] + A[6] * R[2];
    if (fabs(det) < 1e-7) {
        printf("det is %f\n", det);
        det = 1e-5;
    }
    float inv_det = 1 / det;
    for (int i = 0; i < 9; i++) R[i] *= inv_det;
    return det;
}