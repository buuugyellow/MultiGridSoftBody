#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

void printCudaError(const char* funcName);

// R=A*B
__device__ __host__ void MatrixProduct_3_D(const float* A, const float* B, float* R);

// R=A-B
__device__ __host__ void MatrixSubstract_3_D(float* A, float* B, float* R);

// R=A*B
__device__ __host__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz);

__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3]);