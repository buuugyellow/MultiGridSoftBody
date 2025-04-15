#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>

#include <memory>

#include "cudaUtils.h"
#include "global.h"

int tetNum;              // 四面体数量
int tetVertNum;          // 四面体顶点数量
float* tetVertPos_d;     // 当前位置，tetVertNum*3
int* tetIndex_d;         // 四面体索引
float* tetInvD3x3_d;     // 逆矩阵, tetNum*9
float* tetInvD3x4_d;     // Ac阵， tetNum*12
float* tetVolume_d;      // 四面体体积，tetNum
float* tetVolumeDiag_d;  // 四面体顶点形变梯度，tetVertNum_d
float* tetVertMass_d;    // 质量，tetVertNum_d*3
float* tetVertFixed_d;   // 四面体顶点是否固定，0.0f表示没有固定，tetVertNum

float* tetVertPos_last_d;     // 上一时刻位置，tetVertNum*3
float* tetVertPos_old_d;      // st，tetVertNum*3
float* tetVertPos_prev_d;     // 上一次迭代，tetVertNum*3
float* tetVertPos_next_d;     // 下一步位置，tetVertNum*3
float* tetVertVelocity_d;     // 速度，tetVertNum*3
float* tetVertExternForce_d;  // 外力，tetVertNum*3
float* tetVertForce_d;        // 顶点受力, tetVertNum*3

void runInitialize(int tetNum_h, int tetVertNum_h, int* tetIndex_h, float* tetInvD3x3_h, float* tetInvD3x4_h, float* tetVolume_h, float* tetVolumeDiag_h,
                   float* tetVertMass_h, float* tetVertFixed_h, float* tetVertPos_h) {
    tetNum = tetNum_h;
    tetVertNum = tetVertNum_h;
    cudaMalloc((void**)&tetVertPos_d, tetVertNum * 3 * sizeof(float));
    cudaMemcpy(tetVertPos_d, tetVertPos_h, tetVertNum * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetIndex_d, tetNum * 4 * sizeof(int));
    cudaMemcpy(tetIndex_d, tetIndex_h, tetNum * 4 * sizeof(int), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetInvD3x3_d, tetNum * 9 * sizeof(float));
    cudaMemcpy(tetInvD3x3_d, tetInvD3x3_h, tetNum * 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetInvD3x4_d, tetNum * 12 * sizeof(float));
    cudaMemcpy(tetInvD3x4_d, tetInvD3x4_h, tetNum * 12 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetVolume_d, tetNum * sizeof(float));
    cudaMemcpy(tetVolume_d, tetVolume_h, tetNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetVolumeDiag_d, tetVertNum * sizeof(float));
    cudaMemcpy(tetVolumeDiag_d, tetVolumeDiag_h, tetVertNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetVertMass_d, tetVertNum * sizeof(float));
    cudaMemcpy(tetVertMass_d, tetVertMass_h, tetVertNum * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetVertFixed_d, tetVertNum * sizeof(float));
    cudaMemcpy(tetVertFixed_d, tetVertFixed_h, tetVertNum * sizeof(float), cudaMemcpyHostToDevice);

    cudaMalloc((void**)&tetVertPos_last_d, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertPos_old_d, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertPos_prev_d, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertPos_next_d, tetVertNum * 3 * sizeof(float));

    cudaMalloc((void**)&tetVertVelocity_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertVelocity_d, 0.0f, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertExternForce_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertExternForce_d, 0.0f, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertForce_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertForce_d, 0.0f, tetVertNum * 3 * sizeof(float));
}

__global__ void calculateST(float* positions, float* velocity, float* externForce, float* old_positions, float* prev_positions, float* last_Positions,
                            float* fixed, float m_gravityX, float m_gravityY, float m_gravityZ, int vertexNum, float m_damping, float m_dt) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertexNum) return;

    int indexX = threadid * 3 + 0;
    int indexY = threadid * 3 + 1;
    int indexZ = threadid * 3 + 2;

    last_Positions[indexX] = positions[indexX];
    last_Positions[indexY] = positions[indexY];
    last_Positions[indexZ] = positions[indexZ];

    float fixflag = fixed[threadid];
    velocity[indexX] *= m_damping * fixflag;
    velocity[indexY] *= m_damping * fixflag;
    velocity[indexZ] *= m_damping * fixflag;

    // 施加重力
    velocity[indexX] += m_gravityX * m_dt * fixflag;
    velocity[indexY] += m_gravityY * m_dt * fixflag;
    velocity[indexZ] += m_gravityZ * m_dt * fixflag;

    // 施加其他外力
    velocity[indexX] += externForce[indexX] * m_dt * fixflag;
    velocity[indexY] += externForce[indexY] * m_dt * fixflag;
    velocity[indexZ] += externForce[indexZ] * m_dt * fixflag;

    positions[indexX] += velocity[indexX] * m_dt * fixflag;
    positions[indexY] += velocity[indexY] * m_dt * fixflag;
    positions[indexZ] += velocity[indexZ] * m_dt * fixflag;

    // st
    old_positions[indexX] = positions[indexX];
    old_positions[indexY] = positions[indexY];
    old_positions[indexZ] = positions[indexZ];
    prev_positions[indexX] = positions[indexX];
    prev_positions[indexY] = positions[indexY];
    prev_positions[indexZ] = positions[indexZ];

    // 外力清零
    externForce[indexX] = 0.0;
    externForce[indexY] = 0.0;
    externForce[indexZ] = 0.0;
}

void runCalculateST(float m_damping, float m_dt, float m_gravityX, float m_gravityY, float m_gravityZ) {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    calculateST<<<blockNum, threadNum>>>(tetVertPos_d, tetVertVelocity_d, tetVertExternForce_d, tetVertPos_old_d, tetVertPos_prev_d, tetVertPos_last_d,
                                         tetVertFixed_d, m_gravityX, m_gravityY, m_gravityZ, tetVertNum, m_damping, m_dt);
#ifdef PRINT_CUDA_ERROR
    cudaDeviceSynchronize();
    printCudaError("runcalculateST");
#endif  // PRINT_CUDA_ERROR
}

void runClearTemp() { cudaMemset(tetVertForce_d, 0.0f, tetVertNum * 3 * sizeof(float)); }

__global__ void calculateIF(float* positions, int* m_tetIndex, float* m_tetInvD3x3, float* m_tetInvD3x4, float* force, float* tetVolumn, int tetNum,
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

void runCalculateIF(float m_volumnStiffness) {
    int threadNum = 512;
    int blockNum = (tetNum + threadNum - 1) / threadNum;
    calculateIF<<<blockNum, threadNum>>>(tetVertPos_d, tetIndex_d, tetInvD3x3_d, tetInvD3x4_d, tetVertForce_d, tetVolume_d, tetNum, m_volumnStiffness);
#ifdef PRINT_CUDA_ERROR
    cudaDeviceSynchronize();
    printCudaError("runCalculateIF");
#endif  //  PRINT_CUDA_ERROR
}

__global__ void calculatePOS(float* positions, float* force, float* fixed, float* mass, float* next_positions, float* prev_positions, float* old_positions,
                             float* volumnDiag, int vertexNum, float m_dt, float omega) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertexNum) return;

    int indexX = threadid * 3 + 0;
    int indexY = threadid * 3 + 1;
    int indexZ = threadid * 3 + 2;
    float fixflag = fixed[threadid];
    float diagConstant = (mass[threadid]) / (m_dt * m_dt);
    float forceLen = sqrt(force[indexX] * force[indexX] + force[indexY] * force[indexY] + force[indexZ] * force[indexZ]);

    // 计算每个点的shape match产生的约束部分，因为之前是按照每个四面体计算的，现在要摊到每个顶点上
    float elementX = force[indexX];
    float elementY = force[indexY];
    float elementZ = force[indexZ];

    next_positions[indexX] =
        (diagConstant * (old_positions[indexX] - positions[indexX]) + elementX) / (volumnDiag[threadid] + diagConstant) * fixflag + positions[indexX];
    next_positions[indexY] =
        (diagConstant * (old_positions[indexY] - positions[indexY]) + elementY) / (volumnDiag[threadid] + diagConstant) * fixflag + positions[indexY];
    next_positions[indexZ] =
        (diagConstant * (old_positions[indexZ] - positions[indexZ]) + elementZ) / (volumnDiag[threadid] + diagConstant) * fixflag + positions[indexZ];

    // under-relaxation 和 切比雪夫迭代
    next_positions[indexX] = (next_positions[indexX] - positions[indexX]) * 0.6 + positions[indexX];
    next_positions[indexY] = (next_positions[indexY] - positions[indexY]) * 0.6 + positions[indexY];
    next_positions[indexZ] = (next_positions[indexZ] - positions[indexZ]) * 0.6 + positions[indexZ];

    // omega定义：omega = 4 / (4 - rho*rho*omega);
    next_positions[indexX] = omega * (next_positions[indexX] - prev_positions[indexX]) + prev_positions[indexX];
    next_positions[indexY] = omega * (next_positions[indexY] - prev_positions[indexY]) + prev_positions[indexY];
    next_positions[indexZ] = omega * (next_positions[indexZ] - prev_positions[indexZ]) + prev_positions[indexZ];

    prev_positions[indexX] = positions[indexX];
    prev_positions[indexY] = positions[indexY];
    prev_positions[indexZ] = positions[indexZ];

    positions[indexX] = next_positions[indexX];
    positions[indexY] = next_positions[indexY];
    positions[indexZ] = next_positions[indexZ];
}

void runcalculatePOS(float omega, float m_dt) {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    calculatePOS<<<blockNum, threadNum>>>(tetVertPos_d, tetVertForce_d, tetVertFixed_d, tetVertMass_d, tetVertPos_next_d, tetVertPos_prev_d, tetVertPos_old_d,
                                          tetVolumeDiag_d, tetVertNum, m_dt, omega);

#ifdef PRINT_CUDA_ERROR
    cudaDeviceSynchronize();
    printCudaError("runCalculatePOS");
#endif  //  PRINT_CUDA_ERROR
}

__global__ void calculateV(float* positions, float* velocity, float* last_positions, int vertexNum, float m_dt) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertexNum) return;

    velocity[threadid * 3 + 0] = (positions[threadid * 3 + 0] - last_positions[threadid * 3 + 0]) / m_dt;
    velocity[threadid * 3 + 1] = (positions[threadid * 3 + 1] - last_positions[threadid * 3 + 1]) / m_dt;
    velocity[threadid * 3 + 2] = (positions[threadid * 3 + 2] - last_positions[threadid * 3 + 2]) / m_dt;
}

void runCalculateV(float m_dt) {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    calculateV<<<blockNum, threadNum>>>(tetVertPos_d, tetVertVelocity_d, tetVertPos_last_d, tetVertNum, m_dt);
#ifdef PRINT_CUDA_ERROR
    cudaDeviceSynchronize();
    printCudaError("runCalculateV");
#endif  //  PRINT_CUDA_ERROR
}

void runCpyTetVertForRender() { cudaMemcpy(g_simulator->m_tetVertPos.data(), tetVertPos_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost); }

//////////////////////////////////////////// test code ////////////////////////////////////////////

double deltaXFirst = 0;
void runTestConvergence(int iter) {
    vector<float> tetVertPos_prev_h(tetVertNum * 3);
    vector<float> tetVertPos_h(tetVertNum * 3);
    cudaMemcpy(tetVertPos_h.data(), tetVertPos_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(tetVertPos_prev_h.data(), tetVertPos_prev_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    // 计算 deltaX
    double deltaX = 0;
    double deltaXRate = 1;
    for (int i = 0; i < tetVertNum * 3; i++) {
        deltaX += (tetVertPos_h[i] - tetVertPos_prev_h[i]) * (tetVertPos_h[i] - tetVertPos_prev_h[i]);
    }
    deltaX = sqrt(deltaX);
    if (iter == 0) deltaXFirst = deltaX;
    deltaXRate = (iter == 0) ? 1 : (deltaX / deltaXFirst);
    printf("iter: %d, deltaX: %f, rate: %f\n", iter, deltaX, deltaXRate);
}

void runCalEnergy(int iter, float m_dt, const vector<float>& m_tetVertMass, const vector<int>& m_tetIndex, const vector<float>& m_tetInvD3x3,
                  const vector<float>& m_tetVolume, float m_volumnStiffness) {
    vector<float> tetVertPos_h(tetVertNum * 3);
    vector<float> tetVertPos_old_h(tetVertNum * 3);
    vector<float> tetVertPos_prev_h(tetVertNum * 3);
    cudaMemcpy(tetVertPos_h.data(), tetVertPos_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(tetVertPos_old_h.data(), tetVertPos_old_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(tetVertPos_prev_h.data(), tetVertPos_prev_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    // 计算 deltaX, Ek
    double deltaX = 0;
    double Ek = 0;
    for (int i = 0; i < tetVertNum; i++) {
        float vx = tetVertPos_h[i * 3 + 0];
        float vy = tetVertPos_h[i * 3 + 1];
        float vz = tetVertPos_h[i * 3 + 2];
        float vx_pre = tetVertPos_prev_h[i * 3 + 0];
        float vy_pre = tetVertPos_prev_h[i * 3 + 1];
        float vz_pre = tetVertPos_prev_h[i * 3 + 2];
        float vx_st = tetVertPos_old_h[i * 3 + 0];
        float vy_st = tetVertPos_old_h[i * 3 + 1];
        float vz_st = tetVertPos_old_h[i * 3 + 2];

        deltaX += (pow((vx - vx_pre), 2) + pow((vy - vy_pre), 2) + pow((vz - vz_pre), 2));
        Ek += m_tetVertMass[i] * (pow((vx - vx_st), 2) + pow((vy - vy_st), 2) + pow((vz - vz_st), 2));
    }
    deltaX = sqrt(deltaX);
    Ek = Ek / (2 * m_dt * m_dt);

    // 计算 Ep
    double Ep = 0;
    for (int i = 0; i < tetNum; i++) {
        int vIndex0 = m_tetIndex[i * 4 + 0];
        int vIndex1 = m_tetIndex[i * 4 + 1];
        int vIndex2 = m_tetIndex[i * 4 + 2];
        int vIndex3 = m_tetIndex[i * 4 + 3];

        // 先计算shape矩阵
        float D[9];
        D[0] = tetVertPos_h[vIndex1 * 3 + 0] - tetVertPos_h[vIndex0 * 3 + 0];
        D[1] = tetVertPos_h[vIndex2 * 3 + 0] - tetVertPos_h[vIndex0 * 3 + 0];
        D[2] = tetVertPos_h[vIndex3 * 3 + 0] - tetVertPos_h[vIndex0 * 3 + 0];
        D[3] = tetVertPos_h[vIndex1 * 3 + 1] - tetVertPos_h[vIndex0 * 3 + 1];
        D[4] = tetVertPos_h[vIndex2 * 3 + 1] - tetVertPos_h[vIndex0 * 3 + 1];
        D[5] = tetVertPos_h[vIndex3 * 3 + 1] - tetVertPos_h[vIndex0 * 3 + 1];
        D[6] = tetVertPos_h[vIndex1 * 3 + 2] - tetVertPos_h[vIndex0 * 3 + 2];
        D[7] = tetVertPos_h[vIndex2 * 3 + 2] - tetVertPos_h[vIndex0 * 3 + 2];
        D[8] = tetVertPos_h[vIndex3 * 3 + 2] - tetVertPos_h[vIndex0 * 3 + 2];

        // 计算形变梯度F
        float F[9];
        MatrixProduct_3_D(D, &m_tetInvD3x3[i * 9], F);

        // 从F中分解出R（直接搬运，这个算法太复杂了）
        float R[9];
        GetRotation_D((float(*)[3])F, (float(*)[3])R);  // 转化为数组指针，即对应二维数组的形参要求

        double e = 0;
        for (int i = 0; i < 9; i++) {
            e += pow((F[i] - R[i]), 2);
        }
        e = e * m_tetVolume[i] * m_volumnStiffness / 2;

        Ep += e;
    }

    printf("iter = %d, Energy = %f, Ek = %f, Ep = %f, deltaX = %f\n", iter, Ek + Ep, Ek, Ep, deltaX);
}