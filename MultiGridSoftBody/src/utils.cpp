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

float Matrix_Inverse_3(float* A, float* R) {
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

Point3D operator-(const Point3D& a, const Point3D& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

Point3D crossProduct(const Point3D& a, const Point3D& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

Point3D operator/(const Point3D& a, float b) { return {a.x / b, a.y / b, a.z / b}; }

float dotProduct(const Point3D& a, const Point3D& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

float vectorLength(const Point3D& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point) {
    const Point3D P = {point[0], point[1], point[2]};
    for (int k = 0; k < 4; k++) {
        int id_A = (k + 1) % 4;  // 任意选取另一个点
        const Point3D A = {tetVertPos[id_A * 3 + 0], tetVertPos[id_A * 3 + 1], tetVertPos[id_A * 3 + 2]};
        const Point3D AP = P - A;
        const Point3D normal = {tetFaceNormal[k * 3 + 0], tetFaceNormal[k * 3 + 1], tetFaceNormal[k * 3 + 2]};
        float dot = dotProduct(AP, normal);
        if (dot > 0) {
            return false;
        }
    }
    return true;
}

vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal, float V) {
    Point3D r = {point[0], point[1], point[2]};
    Point3D rB = {tetCenter[0], tetCenter[1], tetCenter[2]};
    float F0 = tetFaceArea[0];
    float F1 = tetFaceArea[1];
    float F2 = tetFaceArea[2];
    float F3 = tetFaceArea[3];
    Point3D n0 = {tetFaceNormal[3 * 0 + 0], tetFaceNormal[3 * 0 + 1], tetFaceNormal[3 * 0 + 2]};
    Point3D n1 = {tetFaceNormal[3 * 1 + 0], tetFaceNormal[3 * 1 + 1], tetFaceNormal[3 * 1 + 2]};
    Point3D n2 = {tetFaceNormal[3 * 2 + 0], tetFaceNormal[3 * 2 + 1], tetFaceNormal[3 * 2 + 2]};
    Point3D n3 = {tetFaceNormal[3 * 3 + 0], tetFaceNormal[3 * 3 + 1], tetFaceNormal[3 * 3 + 2]};

    float lambda0 = 0.25f - dotProduct(r - rB, n0) * F0 / (3 * V);
    float lambda1 = 0.25f - dotProduct(r - rB, n1) * F1 / (3 * V);
    float lambda2 = 0.25f - dotProduct(r - rB, n2) * F2 / (3 * V);
    float lambda3 = 0.25f - dotProduct(r - rB, n3) * F3 / (3 * V);

    vector<float> ans = {lambda0, lambda1, lambda2, lambda3};
    return ans;
}