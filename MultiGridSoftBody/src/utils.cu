#include "global.h"

#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

using namespace std;

__host__ void OutputPosNormIndex(string filepath, std::vector<float> pos = std::vector<float>(), std::vector<float> norm = std::vector<float>(),
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

__host__ void printCudaError(const char* funcName) {
    cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "%s error: %s\n", funcName, cudaGetErrorString(cudaStatus));
    }
}

__device__ __host__ void MatrixProduct_3_D(const float* A, const float* B, float* R) {
    R[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
    R[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
    R[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
    R[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
    R[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
    R[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
    R[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
    R[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
    R[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

__device__ __host__ void MatrixSubstract_3_D(float* A, float* B, float* R) {
    for (int i = 0; i < 9; i++) R[i] = A[i] - B[i];
}

__device__ __host__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz) {
    memset(R, 0, sizeof(float) * nx * nz);
    for (int i = 0; i < nx; i++)
        for (int j = 0; j < nz; j++)
            for (int k = 0; k < ny; k++) R[i * nz + j] += A[i * ny + k] * B[k * nz + j];
}

__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3], float& deltaF) {
    float C[3][3];
    memset(&C[0][0], 0, sizeof(float) * 9);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++) C[i][j] += F[k][i] * F[k][j];

    float C2[3][3];
    memset(&C2[0][0], 0, sizeof(float) * 9);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++) C2[i][j] += C[i][k] * C[j][k];

    float det = F[0][0] * F[1][1] * F[2][2] + F[0][1] * F[1][2] * F[2][0] + F[1][0] * F[2][1] * F[0][2] - F[0][2] * F[1][1] * F[2][0] -
                F[0][1] * F[1][0] * F[2][2] - F[0][0] * F[1][2] * F[2][1];
    deltaF = det;

    float I_c = C[0][0] + C[1][1] + C[2][2];
    float I_c2 = I_c * I_c;
    float II_c = 0.5 * (I_c2 - C2[0][0] - C2[1][1] - C2[2][2]);
    float III_c = det * det;
    float k = I_c2 - 3 * II_c;  // k 是一个平方和，大于等于 0

    float inv_U[3][3];
    if (k < 1e-6f) {                                                       // k == 0
        if (I_c < 1e-6) printf("[ERROR]I_c = %f, 退化成一个点？\n", I_c);  // I_c == 0 <=> F = {0}
        float inv_lambda = 1 / sqrt(I_c / 3);
        memset(inv_U, 0, sizeof(float) * 9);
        inv_U[0][0] = inv_lambda;
        inv_U[1][1] = inv_lambda;
        inv_U[2][2] = inv_lambda;
    } else {  // k > 0
        float l = I_c * (I_c * I_c - 4.5 * II_c) + 13.5 * III_c;
        float k_root = sqrt(k);
        float value = l / (k * k_root);
        if (value < -1.0) value = -1.0;
        if (value > 1.0) value = 1.0;
        float phi = acos(value);
        float lambda2 = (I_c + 2 * k_root * cos(phi / 3)) / 3.0;  // phi in [0, pi], phi/3 in [0, pi/3], cos > 0
        float lambda = sqrt(lambda2);

        float III_u = sqrt(III_c);
        if (det < 0) III_u = -III_u;  // ??? 迷惑行为 III_u == det

        if (lambda < 1e-6) printf("[ERROR]lambada = %f, 应该是大于 0 的？\n", lambda);
        if (-lambda2 + I_c + 2 * III_u / lambda < 1e-6)
            printf("[ERROR] -lambda2 + I_c + 2 * III_u / lambda = %f (det = %f)\n", -lambda2 + I_c + 2 * III_u / lambda, det);
        float I_u = lambda + sqrt(-lambda2 + I_c + 2 * III_u / lambda);
        float II_u = (I_u * I_u - I_c) * 0.5;

        float U[3][3];
        float inv_rate, factor;

        if (I_u * II_u - III_u < 1e-6) printf("[ERROR]I_u * II_u - III_u = %f\n", I_u * II_u - III_u);
        inv_rate = 1 / (I_u * II_u - III_u);

        factor = I_u * III_u * inv_rate;

        memset(U, 0, sizeof(float) * 9);
        U[0][0] = factor;
        U[1][1] = factor;
        U[2][2] = factor;

        factor = (I_u * I_u - II_u) * inv_rate;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) U[i][j] += factor * C[i][j] - inv_rate * C2[i][j];

        if (fabs(III_u) < 1e-6) printf("[ERROR]III_u = %f, det = %f\n", III_u, det);  // 这里是因为四面体退化成一个平面了
        inv_rate = 1 / III_u;

        factor = II_u * inv_rate;
        memset(inv_U, 0, sizeof(float) * 9);
        inv_U[0][0] = factor;
        inv_U[1][1] = factor;
        inv_U[2][2] = factor;

        factor = -I_u * inv_rate;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) inv_U[i][j] += factor * U[i][j] + inv_rate * C[i][j];
    }

    memset(&R[0][0], 0, sizeof(float) * 9);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++) R[i][j] += F[i][k] * inv_U[k][j];
}

__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3]) {
    float det;
    GetRotation_D(F, R, det);
}

__device__ __host__ float Matrix_Inverse_3(float* A, float* R) {
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
    if (fabs(det) < 1e-6) {
        printf("det is %f, 四面体退化\n", det);
        // det = 1e-5;
        exit(0);
    }
    float inv_det = 1 / det;
    for (int i = 0; i < 9; i++) R[i] *= inv_det;
    return det;
}

__device__ __host__ float GetVolumn(const Point3D& A, const Point3D& B, const Point3D& C, const Point3D& D) {
    Point3D AB = B - A;
    Point3D AC = C - A;
    Point3D AD = D - A;
    float Deformation[9] = {AB.x, AC.x, AD.x, AB.y, AC.y, AD.y, AB.z, AC.z, AD.z};
    float R[9];
    R[0] = Deformation[4] * Deformation[8] - Deformation[7] * Deformation[5];
    R[1] = Deformation[7] * Deformation[2] - Deformation[1] * Deformation[8];
    R[2] = Deformation[1] * Deformation[5] - Deformation[4] * Deformation[2];
    R[3] = Deformation[5] * Deformation[6] - Deformation[3] * Deformation[8];
    R[4] = Deformation[0] * Deformation[8] - Deformation[2] * Deformation[6];
    R[5] = Deformation[2] * Deformation[3] - Deformation[0] * Deformation[5];
    R[6] = Deformation[3] * Deformation[7] - Deformation[4] * Deformation[6];
    R[7] = Deformation[1] * Deformation[6] - Deformation[0] * Deformation[7];
    R[8] = Deformation[0] * Deformation[4] - Deformation[1] * Deformation[3];
    float det = Deformation[0] * R[0] + Deformation[3] * R[1] + Deformation[6] * R[2];
    if (fabs(det) < 1e-6) {
        printf("det is %f, 四面体退化\n", det);
        exit(0);
    }
    return det / 6.0f;
}

__device__ __host__ Point3D operator-(const Point3D& a, const Point3D& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

__device__ __host__ Point3D operator+(const Point3D& a, const Point3D& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }

__device__ __host__ Point3D crossProduct(const Point3D& a, const Point3D& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

__device__ __host__ Point3D operator/(const Point3D& a, float b) { return {a.x / b, a.y / b, a.z / b}; }

__device__ __host__ Point3D operator*(const Point3D& a, float b) { return {a.x * b, a.y * b, a.z * b}; }

__device__ __host__ float dotProduct(const Point3D& a, const Point3D& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

__device__ __host__ float vectorLength(const Point3D& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

__device__ __host__ bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point) {
    const Point3D P = {point[0], point[1], point[2]};
    for (int k = 0; k < 4; k++) {
        int id_A = (k + 1) % 4;  // 任意选取另一个点
        const Point3D A = {tetVertPos[id_A * 3 + 0], tetVertPos[id_A * 3 + 1], tetVertPos[id_A * 3 + 2]};
        const Point3D AP = P - A;
        const Point3D normal = {tetFaceNormal[k * 3 + 0], tetFaceNormal[k * 3 + 1], tetFaceNormal[k * 3 + 2]};
        float dot = dotProduct(AP, normal);
        if (dot > 1e-6) {
            return false;
        }
    }
    return true;
}

__device__ __host__ vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal,
                                                        float V) {
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

__device__ __host__ void barycentricCoordinate(const Point3D& point, const Point3D& tetVertA, const Point3D& tetVertB, const Point3D& tetVertC,
                                               const Point3D& tetVertD, float* weights) {
    float V = GetVolumn(tetVertA, tetVertB, tetVertC, tetVertD);
    Point3D center = (tetVertA + tetVertB + tetVertC + tetVertD) * 0.25f;
    Point3D tetVerts[4] = {tetVertA, tetVertB, tetVertC, tetVertD};
    int pointFacePair[4][4] = {{0, 1, 2, 3}, {1, 2, 3, 0}, {2, 3, 0, 1}, {3, 0, 1, 2}};
    for (int i = 0; i < 4; i++) {
        Point3D p = tetVerts[pointFacePair[i][0]];
        Point3D facePoint0 = tetVerts[pointFacePair[i][1]];
        Point3D facePoint1 = tetVerts[pointFacePair[i][2]];
        Point3D facePoint2 = tetVerts[pointFacePair[i][3]];

        Point3D edge01 = facePoint1 - facePoint0;
        Point3D edge02 = facePoint2 - facePoint0;

        Point3D normal = crossProduct(edge01, edge02);
        Point3D edge0p = p - facePoint0;
        if (dotProduct(normal, edge0p) > 0){ // 前面有先判断四面体体积是否为 0，此处应该不会有较小值
            normal.x = -normal.x;
            normal.y = -normal.y;
            normal.z = -normal.z;
        }

        float len = vectorLength(normal);
        float area = len * 0.5f;
        normal = normal / len;

        weights[i] = 0.25f - dotProduct(point - center, normal) * area / (3 * V);
    }
}