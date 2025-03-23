#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>

#include <memory>

void printCudaError(const char* funcName) {
    cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "%s error: %s\n", funcName, cudaGetErrorString(cudaStatus));
    }
}

// R=A*B
__device__ void MatrixProduct_3_D(const float* A, const float* B, float* R) {
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

// R=A-B
__device__ void MatrixSubstract_3_D(float* A, float* B, float* R) {
    for (int i = 0; i < 9; i++) R[i] = A[i] - B[i];
}

// R=A*B
__device__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz) {
    memset(R, 0, sizeof(float) * nx * nz);
    for (int i = 0; i < nx; i++)
        for (int j = 0; j < nz; j++)
            for (int k = 0; k < ny; k++) R[i * nz + j] += A[i * ny + k] * B[k * nz + j];
}

__device__ void GetRotation_D(float F[3][3], float R[3][3]) {
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

    R[0][0] = 1;
    R[0][1] = 0;
    R[0][2] = 0;
    R[1][0] = 0;
    R[1][1] = 1;
    R[1][2] = 0;
    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 1;
    // ¼ì²é£¬±ÜÃâinvert
    if (det <= 0) return;

    float I_c = C[0][0] + C[1][1] + C[2][2];
    float I_c2 = I_c * I_c;
    float II_c = 0.5 * (I_c2 - C2[0][0] - C2[1][1] - C2[2][2]);
    float III_c = det * det;
    float k = I_c2 - 3 * II_c;

    float inv_U[3][3];
    if (k < 1e-5f) {
        float inv_lambda = 1 / sqrt(I_c / 3 + 0.000001f);
        memset(inv_U, 0, sizeof(float) * 9);
        inv_U[0][0] = inv_lambda;
        inv_U[1][1] = inv_lambda;
        inv_U[2][2] = inv_lambda;
    } else {
        float l = I_c * (I_c * I_c - 4.5 * II_c) + 13.5 * III_c;
        float k_root = sqrt(k);
        float value = l / (k * k_root + 0.0001f);
        if (value < -1.0) value = -1.0;
        if (value > 1.0) value = 1.0;
        float phi = acos(value);
        float lambda2 = (I_c + 2 * k_root * cos(phi / 3)) / 3.0;
        float lambda = sqrt(lambda2);

        float III_u = sqrt(III_c);

        if (det < 0) III_u = -III_u;

        if (isnan(III_u)) III_u = 1.f;

        float I_u = lambda + sqrt(-lambda2 + I_c + 2 * III_u / (lambda + 0.0001f));
        float II_u = (I_u * I_u - I_c) * 0.5;

        float U[3][3];
        float inv_rate, factor;

        inv_rate = 1 / (I_u * II_u - III_u);

        if (isnan(inv_rate)) inv_rate = 1.f;

        factor = I_u * III_u * inv_rate;

        memset(U, 0, sizeof(float) * 9);
        U[0][0] = factor;
        U[1][1] = factor;
        U[2][2] = factor;

        factor = (I_u * I_u - II_u) * inv_rate;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) U[i][j] += factor * C[i][j] - inv_rate * C2[i][j];

        inv_rate = 1 / III_u;
        if (isnan(inv_rate)) inv_rate = 1.f;
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