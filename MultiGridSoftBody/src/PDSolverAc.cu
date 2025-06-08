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

void PDSolverData::runCalculateIFAc(float m_volumnStiffness) {
    int threadNum = 512;
    int blockNum = (tetNum + threadNum - 1) / threadNum;
    calculateTetDG_Test<<<(tetNum * 9 + threadNum - 1) / threadNum, threadNum>>>(tetVertPos_d, tetIndex_d, tetInvD3x3_d, tetDG_d, tetNum);
    //calculateTetDG<<<blockNum, threadNum>>>(tetVertPos_d, tetIndex_d, tetInvD3x3_d, tetDG_d, tetNum);
    calculateTetIF<<<blockNum, threadNum>>>(tetNum, tetIndex_d, tetDG_d, tetInvD3x4_d, tetVertForce_d, tetVolume_d, m_volumnStiffness);
    PRINT_CUDA_ERROR_AFTER("runCalculateIFAc");
}