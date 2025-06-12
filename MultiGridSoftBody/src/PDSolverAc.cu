#include <math.h>
#include <stdio.h>

#include <memory>

#include "Simulator.h"
#include "global.h"

__global__ void calculateIFBase(float* positions, int* m_tetIndex, float* m_tetInvD3x3, float* m_tetInvD3x4, float* force, float* tetVolumn, int tetNum,
                                float m_volumnStiffness) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum) return;

    // 计算每个四面体初始化的shape矩阵的逆
    int vIndex0 = m_tetIndex[threadid * 4 + 0];
    int vIndex1 = m_tetIndex[threadid * 4 + 1];
    int vIndex2 = m_tetIndex[threadid * 4 + 2];
    int vIndex3 = m_tetIndex[threadid * 4 + 3];

    // 先计算shape矩阵
    float D[9];
    D[0] = positions[vIndex1 * 3 + 0] - positions[vIndex0 * 3 + 0];
    D[1] = positions[vIndex2 * 3 + 0] - positions[vIndex0 * 3 + 0];
    D[2] = positions[vIndex3 * 3 + 0] - positions[vIndex0 * 3 + 0];
    D[3] = positions[vIndex1 * 3 + 1] - positions[vIndex0 * 3 + 1];
    D[4] = positions[vIndex2 * 3 + 1] - positions[vIndex0 * 3 + 1];
    D[5] = positions[vIndex3 * 3 + 1] - positions[vIndex0 * 3 + 1];
    D[6] = positions[vIndex1 * 3 + 2] - positions[vIndex0 * 3 + 2];
    D[7] = positions[vIndex2 * 3 + 2] - positions[vIndex0 * 3 + 2];
    D[8] = positions[vIndex3 * 3 + 2] - positions[vIndex0 * 3 + 2];

    // 计算形变梯度F
    float F[9];
    MatrixProduct_3_D(D, &m_tetInvD3x3[threadid * 9], F);

    // 从F中分解出R（直接搬运，这个算法太复杂了）
    float R[9];
    GetRotation_D((float(*)[3])F, (float(*)[3])R);  // 转化为数组指针，即对应二维数组的形参要求

    MatrixSubstract_3_D(R, F, R);

    float temp[12] = {0};
    MatrixProduct_D(R, &m_tetInvD3x4[threadid * 12], temp, 3, 3, 4);
    for (int i = 0; i < 12; i++) {
        if (isnan(temp[i])) temp[i] = 0;
        temp[i] = temp[i] > 10 ? 10 : temp[i];
        temp[i] = temp[i] < -10 ? -10 : temp[i];
    }

    // 对应的四个点的xyz分量
    // 这里应该需要原子操作
    atomicAdd(force + vIndex0 * 3 + 0, temp[0] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex0 * 3 + 1, temp[4] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex0 * 3 + 2, temp[8] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 0, temp[1] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 1, temp[5] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 2, temp[9] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 0, temp[2] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 1, temp[6] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 2, temp[10] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 0, temp[3] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 1, temp[7] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 2, temp[11] * tetVolumn[threadid] * m_volumnStiffness);
}

__global__ void calculateTetDG(float* positions, int* m_tetIndex, float* m_tetInvD3x3, float* tetDG, int tetNum) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum) return;

    // 计算每个四面体初始化的shape矩阵的逆
    int vIndex0 = m_tetIndex[threadid * 4 + 0];
    int vIndex1 = m_tetIndex[threadid * 4 + 1];
    int vIndex2 = m_tetIndex[threadid * 4 + 2];
    int vIndex3 = m_tetIndex[threadid * 4 + 3];

    // 先计算shape矩阵
    float D[9];
    D[0] = positions[vIndex1 * 3 + 0] - positions[vIndex0 * 3 + 0];
    D[1] = positions[vIndex2 * 3 + 0] - positions[vIndex0 * 3 + 0];
    D[2] = positions[vIndex3 * 3 + 0] - positions[vIndex0 * 3 + 0];
    D[3] = positions[vIndex1 * 3 + 1] - positions[vIndex0 * 3 + 1];
    D[4] = positions[vIndex2 * 3 + 1] - positions[vIndex0 * 3 + 1];
    D[5] = positions[vIndex3 * 3 + 1] - positions[vIndex0 * 3 + 1];
    D[6] = positions[vIndex1 * 3 + 2] - positions[vIndex0 * 3 + 2];
    D[7] = positions[vIndex2 * 3 + 2] - positions[vIndex0 * 3 + 2];
    D[8] = positions[vIndex3 * 3 + 2] - positions[vIndex0 * 3 + 2];

    // 计算形变梯度F
    float* B = &m_tetInvD3x3[threadid * 9];
    float* DG = &tetDG[threadid * 9];
    // DG[0] = D[0] * B[0] + D[1] * B[3] + D[2] * B[6];
    // DG[1] = D[0] * B[1] + D[1] * B[4] + D[2] * B[7];
    // DG[2] = D[0] * B[2] + D[1] * B[5] + D[2] * B[8];
    // DG[3] = D[3] * B[0] + D[4] * B[3] + D[5] * B[6];
    // DG[4] = D[3] * B[1] + D[4] * B[4] + D[5] * B[7];
    // DG[5] = D[3] * B[2] + D[4] * B[5] + D[5] * B[8];
    // DG[6] = D[6] * B[0] + D[7] * B[3] + D[8] * B[6];
    // DG[7] = D[6] * B[1] + D[7] * B[4] + D[8] * B[7];
    // DG[8] = D[6] * B[2] + D[7] * B[5] + D[8] * B[8];

    float dg0 = D[0] * B[0] + D[1] * B[3] + D[2] * B[6];
    float dg1 = D[0] * B[1] + D[1] * B[4] + D[2] * B[7];
    float dg2 = D[0] * B[2] + D[1] * B[5] + D[2] * B[8];
    float dg3 = D[3] * B[0] + D[4] * B[3] + D[5] * B[6];
    float dg4 = D[3] * B[1] + D[4] * B[4] + D[5] * B[7];
    float dg5 = D[3] * B[2] + D[4] * B[5] + D[5] * B[8];
    float dg6 = D[6] * B[0] + D[7] * B[3] + D[8] * B[6];
    float dg7 = D[6] * B[1] + D[7] * B[4] + D[8] * B[7];
    float dg8 = D[6] * B[2] + D[7] * B[5] + D[8] * B[8];
    DG[0] = dg0;
    DG[1] = dg1;
    DG[2] = dg2;
    DG[3] = dg3;
    DG[4] = dg4;
    DG[5] = dg5;
    DG[6] = dg6;
    DG[7] = dg7;
    DG[8] = dg8;
}

__global__ void calculateTetDG_Test(float* positions, int* m_tetIndex, float* m_tetInvD3x3, float* tetDG, int tetNum) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum * 9) return;

    // 0: 0 3 6
    // 1: 1 4 7
    // 2: 2 5 8
    // 3: 0 3 6
    // 4: 1 4 7
    // 5: 2 5 8
    // 6: 0 3 6
    // 7: 1 4 7
    // 8: 2 5 8
    int tetId = threadid / 9;
    float inv0 = m_tetInvD3x3[tetId * 9 + threadid % 3 + 0];
    float inv1 = m_tetInvD3x3[tetId * 9 + threadid % 3 + 3];
    float inv2 = m_tetInvD3x3[tetId * 9 + threadid % 3 + 6];

    // 0: 0 3 6 9
    // 1: 0 3 6 9
    // 2: 0 3 6 9
    // 3: 1 4 7 10
    // 4: 1 4 7 10
    // 5: 1 4 7 10
    // 6: 2 5 8 11
    // 7: 2 5 8 11
    // 8: 2 5 8 11
    int vIndex0 = m_tetIndex[tetId * 4 + 0];
    int vIndex1 = m_tetIndex[tetId * 4 + 1];
    int vIndex2 = m_tetIndex[tetId * 4 + 2];
    int vIndex3 = m_tetIndex[tetId * 4 + 3];

    float p0 = positions[vIndex0 * 3 + (threadid % 9) / 3];
    float p1 = positions[vIndex1 * 3 + (threadid % 9) / 3];
    float p2 = positions[vIndex2 * 3 + (threadid % 9) / 3];
    float p3 = positions[vIndex3 * 3 + (threadid % 9) / 3];

    float d0 = p1 - p0;
    float d1 = p2 - p0;
    float d2 = p3 - p0;

    tetDG[threadid] = d0 * inv0 + d1 * inv1 + d2 * inv2;
}

__device__ __host__ void GetRotation(float F[3][3], float R[3][3]) {
    memset(&R[0][0], 0, sizeof(float) * 9);
    R[0][0] = R[1][1] = R[2][2] = 1;

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

    float I_c = C[0][0] + C[1][1] + C[2][2];
    float I_c2 = I_c * I_c;
    float II_c = 0.5 * (I_c2 - C2[0][0] - C2[1][1] - C2[2][2]);
    float III_c = det * det;
    float k = I_c2 - 3 * II_c;  // k 是一个平方和，大于等于 0

    float inv_U[3][3];
    if (k < 1e-6f) {  // k == 0
        if (I_c < 1e-6) {
            printf("[ERROR]I_c = %f, 四面体退化成一个点\n", I_c);  // I_c == 0 <=> F = {0}
            return;
        }
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

        if (fabs(lambda) < 1e-6) {
            printf("[ERROR]lambada = %f, 应该是大于 0 的？\n", lambda);
            return;
        }
        if (-lambda2 + I_c + 2 * III_u / lambda < 1e-6) {
            printf("[ERROR] -lambda2 + I_c + 2 * III_u / lambda = %f (det = %f)\n", -lambda2 + I_c + 2 * III_u / lambda, det);
            return;
        }

        float I_u = lambda + sqrt(-lambda2 + I_c + 2 * III_u / lambda);
        float II_u = (I_u * I_u - I_c) * 0.5;

        float U[3][3];
        float inv_rate, factor;

        if (fabs(I_u * II_u - III_u) < 1e-6) {
            printf("[ERROR]I_u * II_u - III_u = %f\n", I_u * II_u - III_u);
            return;
        }
        inv_rate = 1 / (I_u * II_u - III_u);

        factor = I_u * III_u * inv_rate;

        memset(U, 0, sizeof(float) * 9);
        U[0][0] = factor;
        U[1][1] = factor;
        U[2][2] = factor;

        factor = (I_u * I_u - II_u) * inv_rate;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) U[i][j] += factor * C[i][j] - inv_rate * C2[i][j];

        if (fabs(III_u) < 1e-6) {
            printf("[ERROR]III_u = %f, det = %f\n", III_u, det);  // 这里是因为四面体退化成一个平面了
            return;
        }
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

__global__ void calculateTetIF(int tetNum, int* m_tetIndex, float* tetDG, float* m_tetInvD3x4, float* force, float* tetVolumn, float m_volumnStiffness) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum) return;

    float R[9];
    GetRotation((float(*)[3]) & tetDG[threadid * 9], (float(*)[3])R);  // 转化为数组指针，即对应二维数组的形参要求

    MatrixSubstract_3_D(R, &tetDG[threadid * 9], R);

    float temp[12] = {0};
    MatrixProduct_D(R, &m_tetInvD3x4[threadid * 12], temp, 3, 3, 4);
    for (int i = 0; i < 12; i++) {
        if (isnan(temp[i])) temp[i] = 0;
        temp[i] = temp[i] > 10 ? 10 : temp[i];
        temp[i] = temp[i] < -10 ? -10 : temp[i];
    }

    // 对应的四个点的xyz分量
    // 这里应该需要原子操作
    int vIndex0 = m_tetIndex[threadid * 4 + 0];
    int vIndex1 = m_tetIndex[threadid * 4 + 1];
    int vIndex2 = m_tetIndex[threadid * 4 + 2];
    int vIndex3 = m_tetIndex[threadid * 4 + 3];
    atomicAdd(force + vIndex0 * 3 + 0, temp[0] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex0 * 3 + 1, temp[4] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex0 * 3 + 2, temp[8] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 0, temp[1] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 1, temp[5] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 2, temp[9] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 0, temp[2] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 1, temp[6] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 2, temp[10] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 0, temp[3] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 1, temp[7] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 2, temp[11] * tetVolumn[threadid] * m_volumnStiffness);
}

// 基准版本，使用本地内存
__global__ void getTetFRBase(int tetNum, float* tetDG, float* tetFR) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum) return;

    float R[9];
    GetRotation((float(*)[3]) & tetDG[threadid * 9], (float(*)[3])R);  // 转化为数组指针，即对应二维数组的形参要求
    MatrixSubstract_3_D(R, &tetDG[threadid * 9], R);
    memcpy(tetFR + threadid * 9, R, 9 * sizeof(float));
}

// 使用寄存器的版本
__global__ void getTetFRV1(int tetNum, float* tetDG, float* tetFR) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum * 9) return;

    int tetId = threadid / 9;
    int i = (threadid % 9) / 3;
    int j = (threadid % 9) % 3;
    float F00 = tetDG[tetId * 9 + 0];
    float F01 = tetDG[tetId * 9 + 1];
    float F02 = tetDG[tetId * 9 + 2];
    float F10 = tetDG[tetId * 9 + 3];
    float F11 = tetDG[tetId * 9 + 4];
    float F12 = tetDG[tetId * 9 + 5];
    float F20 = tetDG[tetId * 9 + 6];
    float F21 = tetDG[tetId * 9 + 7];
    float F22 = tetDG[tetId * 9 + 8];
    float F0j = tetDG[tetId * 9 + 0 + j];  // 想要按照索引去取值，但是不能用数组，用这个中间寄存器保存
    float F1j = tetDG[tetId * 9 + 3 + j];
    float F2j = tetDG[tetId * 9 + 6 + j];
    float Fi0 = tetDG[tetId * 9 + i * 3 + 0];
    float Fi1 = tetDG[tetId * 9 + i * 3 + 1];
    float Fi2 = tetDG[tetId * 9 + i * 3 + 2];
    float Fij = tetDG[tetId * 9 + i * 3 + j];
    float det = F00 * F11 * F22 + F01 * F12 * F20 + F10 * F21 * F02 - F02 * F11 * F20 - F01 * F10 * F22 - F00 * F12 * F21;
    float III_c = det * det;

    float C00 = F00 * F00 + F10 * F10 + F20 * F20;
    float C01 = F00 * F01 + F10 * F11 + F20 * F21;
    float C02 = F00 * F02 + F10 * F12 + F20 * F22;
    float C10 = F01 * F00 + F11 * F10 + F21 * F20;
    float C11 = F01 * F01 + F11 * F11 + F21 * F21;
    float C12 = F01 * F02 + F11 * F12 + F21 * F22;
    float C20 = F02 * F00 + F12 * F10 + F22 * F20;
    float C21 = F02 * F01 + F12 * F11 + F22 * F21;
    float C22 = F02 * F02 + F12 * F12 + F22 * F22;
    float I_c = C00 + C11 + C22;
    float I_c2 = I_c * I_c;
    float C0j = F00 * F0j + F10 * F1j + F20 * F2j;
    float C1j = F01 * F0j + F11 * F1j + F21 * F2j;
    float C2j = F02 * F0j + F12 * F1j + F22 * F2j;

    float CC00 = C00 * C00 + C01 * C01 + C02 * C02;
    float CC01 = C00 * C10 + C01 * C11 + C02 * C12;
    float CC02 = C00 * C20 + C01 * C21 + C02 * C22;
    float CC10 = C10 * C00 + C11 * C01 + C12 * C02;
    float CC11 = C10 * C10 + C11 * C11 + C12 * C12;
    float CC12 = C10 * C20 + C11 * C21 + C12 * C22;
    float CC20 = C20 * C00 + C21 * C01 + C22 * C02;
    float CC21 = C20 * C10 + C21 * C11 + C22 * C12;
    float CC22 = C20 * C20 + C21 * C21 + C22 * C22;
    float II_c = 0.5 * (I_c2 - CC00 - CC11 - CC22);
    float k = I_c2 - 3 * II_c;  // k 是一个平方和，大于等于 0

    // float CC0j = C00 * Cj0 + C01 * Cj1 + C02 * Cj2;
    // float CC1j = C10 * Cj0 + C11 * Cj1 + C12 * Cj2;
    // float CC1j = C20 * Cj0 + C21 * Cj1 + C22 * Cj2;
    // 因为 C = (F^T)F 是对称阵，因此 Cj0 = C0j
    float CC0j = C00 * C0j + C01 * C1j + C02 * C2j;
    float CC1j = C10 * C0j + C11 * C1j + C12 * C2j;
    float CC2j = C20 * C0j + C21 * C1j + C22 * C2j;

    if (k < 1e-6f) {
        if (I_c < 1e-6) printf("[ERROR]I_c = %f, 四面体退化成一个点\n", I_c);  // I_c == 0 <=> F = {0}
        float temp = 1 / sqrt(I_c / 3);
        tetFR[threadid] = Fij * temp - Fij;
        return;
    }

    float l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
    float k_root = sqrt(k);
    float value = l / (k * k_root);
    value = max(1.0f, min(-1.0f, value));
    float phi = acos(value);
    float lambda2 = (I_c + 2 * k_root * cos(phi / 3)) / 3.0;  // phi in [0, pi], phi/3 in [0, pi/3], cos > 0
    float lambda = sqrt(lambda2);
    float III_u = det;
    if (fabs(lambda) < 1e-6) printf("[ERROR]lambada = %f, 应该是大于 0 的？\n", lambda);
    if (-lambda2 + I_c + 2 * III_u / lambda < 1e-6)
        printf("[ERROR] -lambda2 + I_c + 2 * III_u / lambda = %f (det = %f)\n", -lambda2 + I_c + 2 * III_u / lambda, det);
    float I_u = lambda + sqrt(-lambda2 + I_c + 2 * III_u / lambda);
    float II_u = (I_u * I_u - I_c) * 0.5;
    if (fabs(I_u * II_u - III_u) < 1e-6) printf("[ERROR]I_u * II_u - III_u = %f\n", I_u * II_u - III_u);
    float inv_rate = 1 / (I_u * II_u - III_u);
    float factor = I_u * III_u * inv_rate;

    float temp = factor;
    factor = (I_u * I_u - II_u) * inv_rate;
    float U0j = factor * C0j - inv_rate * CC0j;
    float U1j = factor * C1j - inv_rate * CC1j;
    float U2j = factor * C2j - inv_rate * CC2j;
    if (j == 0) U0j += temp;  // 这三个分支需要合并
    if (j == 1) U1j += temp;
    if (j == 2) U2j += temp;

    if (fabs(III_u) < 1e-6) printf("[ERROR]III_u = %f, det = %f\n", III_u, det);  // 这里是因为四面体退化成一个平面了
    inv_rate = 1 / III_u;
    temp = II_u * inv_rate;
    factor = -I_u * inv_rate;
    float inv_U0j = factor * U0j + inv_rate * C0j;
    float inv_U1j = factor * U1j + inv_rate * C1j;
    float inv_U2j = factor * U2j + inv_rate * C2j;
    if (j == 0) inv_U0j += temp;  // 这三个分支需要合并
    if (j == 1) inv_U1j += temp;
    if (j == 2) inv_U2j += temp;

    tetFR[threadid] = Fi0 * inv_U0j + Fi1 * inv_U1j + Fi2 * inv_U2j - Fij;
}

__global__ void calTetIFBase(int tetNum, float* tetFR, float* m_tetInvD3x4, int* m_tetIndex, float* force, float* tetVolumn, float m_volumnStiffness) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum) return;

    float temp[12] = {0};
    MatrixProduct_D(tetFR + threadid * 9, &m_tetInvD3x4[threadid * 12], temp, 3, 3, 4);
    for (int i = 0; i < 12; i++) {
        if (isnan(temp[i])) temp[i] = 0;
        temp[i] = temp[i] > 10 ? 10 : temp[i];
        temp[i] = temp[i] < -10 ? -10 : temp[i];
    }

    // 对应的四个点的xyz分量
    // 这里应该需要原子操作
    int vIndex0 = m_tetIndex[threadid * 4 + 0];
    int vIndex1 = m_tetIndex[threadid * 4 + 1];
    int vIndex2 = m_tetIndex[threadid * 4 + 2];
    int vIndex3 = m_tetIndex[threadid * 4 + 3];
    atomicAdd(force + vIndex0 * 3 + 0, temp[0] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex0 * 3 + 1, temp[4] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex0 * 3 + 2, temp[8] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 0, temp[1] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 1, temp[5] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex1 * 3 + 2, temp[9] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 0, temp[2] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 1, temp[6] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex2 * 3 + 2, temp[10] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 0, temp[3] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 1, temp[7] * tetVolumn[threadid] * m_volumnStiffness);
    atomicAdd(force + vIndex3 * 3 + 2, temp[11] * tetVolumn[threadid] * m_volumnStiffness);
}

__global__ void calTetIFV1(int tetNum, int* tetIndex, float* tetFR, float* tetInvD3x4, float* tetVolumn, float volumnStiffness, float* force) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetNum * 12) return;

    int tetId = threadid / 12;
    int i = (threadid % 12) / 4;
    int j = (threadid % 12) % 4;

    int vertId = tetIndex[tetId * 4 + j];
    int forceId = vertId * 3 + i;

    float volumn = tetVolumn[tetId];

    float FRi0 = tetFR[tetId * 9 + i * 3 + 0];
    float FRi1 = tetFR[tetId * 9 + i * 3 + 1];
    float FRi2 = tetFR[tetId * 9 + i * 3 + 2];

    float inv0j = tetInvD3x4[tetId * 12 + 0 * 4 + j];
    float inv1j = tetInvD3x4[tetId * 12 + 1 * 4 + j];
    float inv2j = tetInvD3x4[tetId * 12 + 2 * 4 + j];

    float temp = FRi0 * inv0j + FRi1 * inv1j + FRi2 * inv2j;

    // if (isnan(temp)) temp[i] = 0;
    temp = min(10.0f, max(-10.0f, temp));
    float result = temp * volumn * volumnStiffness;

    atomicAdd(force + forceId, result);
}

void PDSolverData::runCalculateIFAc(float m_volumnStiffness) {
    // calculateTetDG<<<BLOCK_SIZE(tetNum, 512), 512>>>(tetVertPos_d, tetIndex_d, tetInvD3x3_d, tetDG_d, tetNum);
    calculateTetDG_Test<<<BLOCK_SIZE(tetNum * 9, 512), 512>>>(tetVertPos_d, tetIndex_d, tetInvD3x3_d, tetDG_d, tetNum);

    // calculateTetIF<<<BLOCK_SIZE(tetNum, 512), 512>>>(tetNum, tetIndex_d, tetDG_d, tetInvD3x4_d, tetVertForce_d, tetVolume_d, m_volumnStiffness);
    // getTetFRBase<<<BLOCK_SIZE(tetNum, 512), 512>>>(tetNum, tetDG_d, tetFR_d);
    getTetFRV1<<<BLOCK_SIZE(tetNum * 9, 512), 512>>>(tetNum, tetDG_d, tetFR_d);
    // calTetIFBase<<<BLOCK_SIZE(tetNum, 512), 512>>>(tetNum, tetFR_d, tetInvD3x4_d, tetIndex_d, tetVertForce_d, tetVolume_d, m_volumnStiffness);
    calTetIFV1<<<BLOCK_SIZE(tetNum * 12, 512), 512>>>(tetNum, tetIndex_d, tetFR_d, tetInvD3x4_d, tetVolume_d, m_volumnStiffness, tetVertForce_d);
    PRINT_CUDA_ERROR_AFTER("runCalculateIFAc");
}

__global__ void calculatePOSV1(float* positions, float* fixed, float* mass, float* next_positions, float* prev_positions, float* old_positions,
                               float* volumnDiag, float* force, float* collisionDiag, float* collisionForce, int vertexNum, float m_dt, float omega) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertexNum * 3) return;

    int vertId = threadid / 3;
    int index = vertId * 3 + threadid % 3;

    float fixflag = fixed[vertId];
    float m = mass[vertId];
    float f = force[index];
    float colF = collisionForce[index];
    float oldPos = old_positions[index];
    float prevPos = prev_positions[index];
    float pos = positions[index];
    float vDiag = volumnDiag[vertId];
    float colDiag = collisionDiag[index];

    float diagConstant = m / (m_dt * m_dt);
    float element = f + colF;

    float next = (diagConstant * (oldPos - pos) + element) / (vDiag + colDiag + diagConstant) * fixflag + pos;

    // under-relaxation 和 切比雪夫迭代
    next = (next - pos) * 0.6 + pos;

    // omega定义：omega = 4 / (4 - rho*rho*omega);
    next = omega * (next - prevPos) + prevPos;

    next_positions[index] = next;
    prev_positions[index] = pos;
    positions[index] = next;
}

void PDSolverData::runcalculatePOSAc(float omega, float m_dt) {
    calculatePOSV1<<<BLOCK_SIZE(tetVertNum * 3, 512), 512>>>(tetVertPos_d, tetVertFixed_d, tetVertMass_d, tetVertPos_next_d, tetVertPos_prev_d,
                                                             tetVertPos_old_d, tetVolumeDiag_d, tetVertForce_d, tetVertCollisionDiag_d, tetVertCollisionForce_d,
                                                             tetVertNum, m_dt, omega);
    PRINT_CUDA_ERROR_AFTER("runCalculatePOS");
}