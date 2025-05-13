#include <math.h>
#include <stdio.h>

#include <memory>

#include "Simulator.h"
#include "global.h"

#ifdef PRINT_CUDA_ERROR
#define PRINT_CUDA_ERROR_AFTER(func) \
    do {                             \
        cudaDeviceSynchronize();     \
        printCudaError(func);        \
    } while (0)
#else
#define PRINT_CUDA_ERROR_AFTER(func)
#endif  // PRINT_CUDA_ERROR

void PDSolverData::Init(int tetNum_h, int tetVertNum_h, int* tetIndex_h, float* tetInvD3x3_h, float* tetInvD3x4_h, float* tetVolume_h, float* tetVolumeDiag_h,
                        float* tetVertMass_h, float* tetVertFixed_h, float* tetVertPos_h, int outsideTriNum_h, unsigned int* outsideTriIndex_h) {
    tetNum = tetNum_h;
    tetVertNum = tetVertNum_h;
    outsideTriNum = outsideTriNum_h;
    cudaMalloc((void**)&tetVertPos_d, tetVertNum * 3 * sizeof(float));
    cudaMemcpy(tetVertPos_d, tetVertPos_h, tetVertNum * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&tetIndex_d, tetNum * 4 * sizeof(int));
    cudaMemcpy(tetIndex_d, tetIndex_h, tetNum * 4 * sizeof(int), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&outsideTriIndex_d, outsideTriNum * 3 * sizeof(unsigned int));
    cudaMemcpy(outsideTriIndex_d, outsideTriIndex_h, outsideTriNum * 3 * sizeof(unsigned int), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&outsideTriNormal_d, outsideTriNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertNormal_d, tetVertNum * 3 * sizeof(float));
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
    cudaMemset(tetVertVelocity_d, 0, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertVelocityBak_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertVelocityBak_d, 0, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertExternForce_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertExternForce_d, 0, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertForce_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertForce_d, 0, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertIsCollied_d, tetVertNum * sizeof(char));
    cudaMemset(tetVertIsCollied_d, 0, tetVertNum * sizeof(char));
    cudaMalloc((void**)&tetVertCollisionDiag_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertCollisionDiag_d, 0, tetVertNum * 3 * sizeof(float));
    cudaMalloc((void**)&tetVertCollisionForce_d, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertCollisionForce_d, 0, tetVertNum * 3 * sizeof(float));
    PRINT_CUDA_ERROR_AFTER("Init");
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

void PDSolverData::runCalculateST(float m_damping, float m_dt, float m_gravityX, float m_gravityY, float m_gravityZ) {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    calculateST<<<blockNum, threadNum>>>(tetVertPos_d, tetVertVelocity_d, tetVertExternForce_d, tetVertPos_old_d, tetVertPos_prev_d, tetVertPos_last_d,
                                         tetVertFixed_d, m_gravityX, m_gravityY, m_gravityZ, tetVertNum, m_damping, m_dt);
    PRINT_CUDA_ERROR_AFTER("runcalculateST");
}

void PDSolverData::runClearTemp() { cudaMemset(tetVertForce_d, 0.0f, tetVertNum * 3 * sizeof(float)); }

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

void PDSolverData::runCalculateIF(float m_volumnStiffness) {
    int threadNum = 512;
    int blockNum = (tetNum + threadNum - 1) / threadNum;
    calculateIF<<<blockNum, threadNum>>>(tetVertPos_d, tetIndex_d, tetInvD3x3_d, tetInvD3x4_d, tetVertForce_d, tetVolume_d, tetNum, m_volumnStiffness);
    PRINT_CUDA_ERROR_AFTER("runCalculateIF");
}

__global__ void calculatePOS(float* positions, float* fixed, float* mass, float* next_positions, float* prev_positions, float* old_positions, float* volumnDiag,
                             float* force, float* collisionDiag, float* collisionForce, int vertexNum, float m_dt, float omega) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertexNum) return;

    int indexX = threadid * 3 + 0;
    int indexY = threadid * 3 + 1;
    int indexZ = threadid * 3 + 2;
    float fixflag = fixed[threadid];
    float diagConstant = (mass[threadid]) / (m_dt * m_dt);

    // 计算每个点的shape match产生的约束部分，因为之前是按照每个四面体计算的，现在要摊到每个顶点上
    float elementX = force[indexX] + collisionForce[indexX];
    float elementY = force[indexY] + collisionForce[indexY];
    float elementZ = force[indexZ] + collisionForce[indexZ];

    next_positions[indexX] =
        (diagConstant * (old_positions[indexX] - positions[indexX]) + elementX) / (volumnDiag[threadid] + collisionDiag[indexX] + diagConstant) * fixflag +
        positions[indexX];
    next_positions[indexY] =
        (diagConstant * (old_positions[indexY] - positions[indexY]) + elementY) / (volumnDiag[threadid] + collisionDiag[indexY] + diagConstant) * fixflag +
        positions[indexY];
    next_positions[indexZ] =
        (diagConstant * (old_positions[indexZ] - positions[indexZ]) + elementZ) / (volumnDiag[threadid] + collisionDiag[indexZ] + diagConstant) * fixflag +
        positions[indexZ];

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

void PDSolverData::runcalculatePOS(float omega, float m_dt) {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    calculatePOS<<<blockNum, threadNum>>>(tetVertPos_d, tetVertFixed_d, tetVertMass_d, tetVertPos_next_d, tetVertPos_prev_d, tetVertPos_old_d, tetVolumeDiag_d,
                                          tetVertForce_d, tetVertCollisionDiag_d, tetVertCollisionForce_d, tetVertNum, m_dt, omega);
    PRINT_CUDA_ERROR_AFTER("runCalculatePOS");
}

__global__ void calculateV(float* positions, float* velocity, float* last_positions, int vertexNum, float m_dt) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertexNum) return;

    velocity[threadid * 3 + 0] = (positions[threadid * 3 + 0] - last_positions[threadid * 3 + 0]) / m_dt;
    velocity[threadid * 3 + 1] = (positions[threadid * 3 + 1] - last_positions[threadid * 3 + 1]) / m_dt;
    velocity[threadid * 3 + 2] = (positions[threadid * 3 + 2] - last_positions[threadid * 3 + 2]) / m_dt;
}

void PDSolverData::runCalculateV(float m_dt) {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    calculateV<<<blockNum, threadNum>>>(tetVertPos_d, tetVertVelocity_d, tetVertPos_last_d, tetVertNum, m_dt);
    PRINT_CUDA_ERROR_AFTER("runCalculateV");
}

void PDSolverData::runCpyTetVertForRender() {
    cudaMemcpy(g_simulator->m_tetVertPos.data(), tetVertPos_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(g_simulator->m_normal.data(), tetVertNormal_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
}

void PDSolverData::runResetPosVel() {
    cudaMemcpy(tetVertPos_d, tetVertPos_last_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(tetVertVelocity_d, tetVertVelocityBak_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToDevice);
}

void PDSolverData::runSaveVel() { cudaMemcpy(tetVertVelocityBak_d, tetVertVelocity_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToDevice); }

__global__ void DCDByPoint_sphere(Point3D center, float radius, float collisionStiffness, int vertexNum, float* positions, float* directDir, char* isCollied,
                                  float* collisionDiag, float* collisionForce) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertexNum) return;

    int xId = threadid * 3 + 0;
    int yId = threadid * 3 + 1;
    int zId = threadid * 3 + 2;
    Point3D p = {positions[xId], positions[yId], positions[zId]};
    Point3D cp = p - center;

    Point3D dir = cp / length(cp);
    if (directDir != nullptr) dir = {directDir[xId], directDir[yId], directDir[zId]};

    float distance = length(cp);
    if (distance < radius) {
        // (cp + t * dir)^2 = r^2
        float a = 1;
        float b = 2 * dotProduct(cp, dir);
        float c = dotProduct(cp, cp) - radius * radius;
        float t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
        Point3D colP = p + dir * t;
        Point3D centerColP = colP - center;
        Point3D colN = centerColP / length(centerColP);
        float colNDotDir = dotProduct(colN, dir);

        isCollied[threadid] = 1;
        collisionDiag[xId] += collisionStiffness * colN.x * colN.x;
        collisionDiag[yId] += collisionStiffness * colN.y * colN.y;
        collisionDiag[zId] += collisionStiffness * colN.z * colN.z;
        collisionForce[xId] += collisionStiffness * t * colNDotDir * colN.x;
        collisionForce[yId] += collisionStiffness * t * colNDotDir * colN.y;
        collisionForce[zId] += collisionStiffness * t * colNDotDir * colN.z;
    }
}

void PDSolverData::runDCDByPoint_sphere(Point3D center, float radius, float collisionStiffness) {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    DCDByPoint_sphere<<<blockNum, threadNum>>>(center, radius, collisionStiffness, tetVertNum, tetVertPos_d, nullptr, tetVertIsCollied_d,
                                               tetVertCollisionDiag_d, tetVertCollisionForce_d);
    PRINT_CUDA_ERROR_AFTER("runDCDByPoint_sphere");
}

__global__ void UpdateTriNormal(int outsideTriNum, float* positions, unsigned int* outsideTriIndex, float* outsideTriNormal, float* tetVertNormal) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= outsideTriNum) return;

    unsigned int idA = outsideTriIndex[threadid * 3 + 0];
    unsigned int idB = outsideTriIndex[threadid * 3 + 1];
    unsigned int idC = outsideTriIndex[threadid * 3 + 2];
    Point3D A = {positions[idA * 3 + 0], positions[idA * 3 + 1], positions[idA * 3 + 2]};
    Point3D B = {positions[idB * 3 + 0], positions[idB * 3 + 1], positions[idB * 3 + 2]};
    Point3D C = {positions[idC * 3 + 0], positions[idC * 3 + 1], positions[idC * 3 + 2]};
    Point3D AB = B - A;
    Point3D AC = C - A;
    Point3D ABCrossAC = crossProduct(AB, AC);
    normalize(ABCrossAC);
    float x = ABCrossAC.x;
    float y = ABCrossAC.y;
    float z = ABCrossAC.z;
    outsideTriNormal[threadid * 3 + 0] = x;
    outsideTriNormal[threadid * 3 + 1] = y;
    outsideTriNormal[threadid * 3 + 2] = z;
    
    // 直接平均所在的三角形法向量，加权可以考虑：1. 面积 2. 角度
    atomicAdd(tetVertNormal + idA * 3 + 0, x);
    atomicAdd(tetVertNormal + idA * 3 + 1, y);
    atomicAdd(tetVertNormal + idA * 3 + 2, z);
    atomicAdd(tetVertNormal + idB * 3 + 0, x);
    atomicAdd(tetVertNormal + idB * 3 + 1, y);
    atomicAdd(tetVertNormal + idB * 3 + 2, z);
    atomicAdd(tetVertNormal + idC * 3 + 0, x);
    atomicAdd(tetVertNormal + idC * 3 + 1, y);
    atomicAdd(tetVertNormal + idC * 3 + 2, z);
}

void PDSolverData::runUpdateTriNormal() {
    int threadNum = 512;
    int blockNum = (outsideTriNum + threadNum - 1) / threadNum;
    UpdateTriNormal<<<blockNum, threadNum>>>(outsideTriNum, tetVertPos_d, outsideTriIndex_d, outsideTriNormal_d, tetVertNormal_d);
    PRINT_CUDA_ERROR_AFTER("runUpdateTriNormal");
}

void PDSolverData::runClearTetVertNormal() { cudaMemset(tetVertNormal_d, 0, tetVertNum * 3 * sizeof(float)); }

__global__ void AvgOutsideTetVertNormal(int tetVertNum, float* tetVertNormal) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetVertNum) return;

    Point3D dir = {tetVertNormal[threadid * 3 + 0], tetVertNormal[threadid * 3 + 1], tetVertNormal[threadid * 3 + 2]};
    float len = length(dir);
    if (len > 1.0f - 1e-5f) {
        tetVertNormal[threadid * 3 + 0] /= len;
        tetVertNormal[threadid * 3 + 1] /= len;
        tetVertNormal[threadid * 3 + 2] /= len;
    } else if (len > 1e-5f) {
        printf("[ERROR]顶点法向量的模长应为 0 或者大于等于 1\n");
    }
}

void PDSolverData::runAvgOutsideTetVertNormal() {
    int threadNum = 512;
    int blockNum = (tetVertNum + threadNum - 1) / threadNum;
    AvgOutsideTetVertNormal<<<blockNum, threadNum>>>(tetVertNum, tetVertNormal_d);
    PRINT_CUDA_ERROR_AFTER("runAvgOutsideTetVertNormal");
}

void PDSolverData::runUpdateOutsideTetVertNormal() { 
    runClearTetVertNormal();
    runUpdateTriNormal();
    runAvgOutsideTetVertNormal();
}

__global__ void DCDByTriangle_sphere(Point3D center, float radius, float collisionStiffness, int outsideTriNum, float* positions, unsigned int* outsideTriIndex,
                                     float* outsideTriNormal) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= outsideTriNum) return;

    unsigned int idA = outsideTriIndex[threadid * 3 + 0];
    unsigned int idB = outsideTriIndex[threadid * 3 + 1];
    unsigned int idC = outsideTriIndex[threadid * 3 + 2];
    Point3D A = {positions[idA * 3 + 0], positions[idA * 3 + 1], positions[idA * 3 + 2]};
    Point3D B = {positions[idB * 3 + 0], positions[idB * 3 + 1], positions[idB * 3 + 2]};
    Point3D C = {positions[idC * 3 + 0], positions[idC * 3 + 1], positions[idC * 3 + 2]};
    Point3D triNormal = {outsideTriNormal[threadid * 3 + 0], outsideTriNormal[threadid * 3 + 1], outsideTriNormal[threadid * 3 + 2]};
    Point3D sphereDir = {-triNormal.x, -triNormal.y, -triNormal.z};

    // 1. 判断球是否与三角形相交，如果不相交则退出
    // 2. 判断球射线是否与三角形对应平面相交
    // 3. 判断球射线是否与三条边所对应的圆柱体相交（如果正向交点是外面的半圆柱则找到）
    // 4. 判断球射线是否与三个顶点所对应的球面相交（如果正向交点是外面的球面则找到）

    // ---- Step 1: 计算球是否与三角形相交 ----
    if (!sphereIntersectTri(center, radius, A, B, C)) return;
    // 在这里测试碰撞检测的正确性 TODO

    // ---- Step 2: 计算球心射线与两个三角面是否相交 ----
    Point3D planePos = A + triNormal * radius;                                                  // 正向扩展平面
    Point3D planeNeg = A - triNormal * radius;                                                  // 负向扩展平面
    float t_pos = dotProduct(planePos - center, triNormal) / dotProduct(sphereDir, triNormal);  // 计算射线与两个平面的交点
    float t_neg = dotProduct(planeNeg - center, triNormal) / dotProduct(sphereDir, triNormal);  // 计算射线与两个平面的交点
    float t;
    Point3D AA, BB, CC;
    if (t_pos > 0) { // 因为前面已经判定过相交，因此这里肯定一正一负
        t = t_pos;
        AA = A + triNormal * radius;
        BB = B + triNormal * radius;
        CC = C + triNormal * radius;
    } else {
        t = t_neg;
        AA = A - triNormal * radius;
        BB = B - triNormal * radius;
        CC = C - triNormal * radius;
    }    
    Point3D hit = center + sphereDir * t;  // 筛选有效交点（t>0 且在三角形内）
    bool valid = pointInTriangle(hit, AA, BB, CC);
    if (valid) {
        // TODO

        return;
    }

    Point3D triVerts[3] = {A, B, C};
    // ---- Step 3: 计算球心射线与三个圆柱面是否相交 ----
    for (int i = 0; i < 3; i++) {
        Point3D& Begin = triVerts[i];
        Point3D& End = triVerts[(i + 1) % 3];
        Point3D edgeDir = End - Begin;
        normalize(edgeDir);
        Point3D localCenter = center - Begin;
        float a = dotProduct(sphereDir, sphereDir) - pow(dotProduct(sphereDir, edgeDir), 2);
        float b = 2 * (dotProduct(localCenter, sphereDir) - dotProduct(localCenter, edgeDir) * dotProduct(sphereDir, edgeDir));
        float c = dotProduct(localCenter, localCenter) - pow(dotProduct(localCenter, edgeDir), 2) - radius * radius;
        float discriminant = b * b - 4 * a * c;
        if (discriminant > -FLT_EPSILON) { // 射线与无限长圆柱相交
            float t_cyl = (-b + sqrt(discriminant)) / (2 * a);  // 取较大解（靠近外侧）
            Point3D hit = center + sphereDir * t_cyl; // 射线与圆柱面的正向交点，判断：交点是否在线段范围内且交点是否在外表面
            if (t_cyl > 0 && isOnCylinderSegment(hit, Begin, End)) {
                Point3D projectedHit = hit - triNormal * dotProduct((hit - A), triNormal); // 投影到三角形平面
                Point3D Other = triVerts[(i + 2) % 3];
                Point3D BE = End - Begin;
                Point3D BO = Other - Begin;
                Point3D BH = projectedHit - Begin;
                Point3D BECrossBO = crossProduct(BE, BO);
                Point3D BECrossBH = crossProduct(BE, BH);
                bool valid = (dotProduct(BECrossBO, BECrossBH) < 0);
                if (valid) {
                    // TODO

                    return;
                }
            }
        }

    }

    // ---- Step 4: 计算球心射线与三个球面是否相交 ----
    for (int i = 0; i < 3; i++) {
        Point3D& V = triVerts[i];
        Point3D VO = center - V;
        float a = dotProduct(sphereDir, sphereDir);
        float b = 2 * dotProduct(sphereDir, VO);
        float c = dotProduct(VO, VO) - radius * radius;
        float discriminant = b * b - 4 * a * c;
        if (discriminant > -FLT_EPSILON) { // 射线与球相交
            float t = (-b + sqrtf(discriminant)) / (2 * a); // 取大的解
            if (t > 0) { 
                Point3D hit = center + sphereDir * t;
                Point3D& A = triVerts[(i + 1) % 3];
                Point3D& B = triVerts[(i + 2) % 3];
                bool valid = !isOnCylinderSegment(hit, V, A) && !isOnCylinderSegment(hit, V, B);
                if (valid) {
                    // TODO

                    return;
                }
            }
        }
    }
}

void PDSolverData::runDCDByTriangle_sphere(Point3D center, float radius, float collisionStiffness) {
    int threadNum = 512;
    int blockNum = (outsideTriNum + threadNum - 1) / threadNum;
    //DCDByTriangle_sphere<<<blockNum, threadNum>>>();
    PRINT_CUDA_ERROR_AFTER("runDCDByTriangle_sphere");
}

void PDSolverData::runClearCollision() {
    cudaMemset(tetVertIsCollied_d, 0, tetVertNum * sizeof(char));
    cudaMemset(tetVertCollisionDiag_d, 0, tetVertNum * 3 * sizeof(float));
    cudaMemset(tetVertCollisionForce_d, 0, tetVertNum * 3 * sizeof(float));
}

__global__ void interpolate(int tetVertNumFine, float* tetVertPosFine, float* tetVertPosPrevFine, float* tetVertPosCoarse, int* interpolationIds,
                            float* interpolationWights) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetVertNumFine) return;

    int id0 = interpolationIds[threadid * 4 + 0];
    int id1 = interpolationIds[threadid * 4 + 1];
    int id2 = interpolationIds[threadid * 4 + 2];
    int id3 = interpolationIds[threadid * 4 + 3];
    float w0 = interpolationWights[threadid * 4 + 0];
    float w1 = interpolationWights[threadid * 4 + 1];
    float w2 = interpolationWights[threadid * 4 + 2];
    float w3 = interpolationWights[threadid * 4 + 3];
    float tVPosCoarse0[3] = {tetVertPosCoarse[id0 * 3 + 0], tetVertPosCoarse[id0 * 3 + 1], tetVertPosCoarse[id0 * 3 + 2]};
    float tVPosCoarse1[3] = {tetVertPosCoarse[id1 * 3 + 0], tetVertPosCoarse[id1 * 3 + 1], tetVertPosCoarse[id1 * 3 + 2]};
    float tVPosCoarse2[3] = {tetVertPosCoarse[id2 * 3 + 0], tetVertPosCoarse[id2 * 3 + 1], tetVertPosCoarse[id2 * 3 + 2]};
    float tVPosCoarse3[3] = {tetVertPosCoarse[id3 * 3 + 0], tetVertPosCoarse[id3 * 3 + 1], tetVertPosCoarse[id3 * 3 + 2]};

    for (int i = 0; i < 3; i++) {
        tetVertPosPrevFine[threadid * 3 + i] = tetVertPosFine[threadid * 3 + i] =
            w0 * tVPosCoarse0[i] + w1 * tVPosCoarse1[i] + w2 * tVPosCoarse2[i] + w3 * tVPosCoarse3[i];
    }
}

__global__ void updatePointInTet(int vertNum, float* vertPos, float* vertPosPrev, float* tetVertPos, int* mapIds, float* mapWights) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= vertNum) return;

    int id0 = mapIds[threadid * 4 + 0];
    int id1 = mapIds[threadid * 4 + 1];
    int id2 = mapIds[threadid * 4 + 2];
    int id3 = mapIds[threadid * 4 + 3];
    float w0 = mapWights[threadid * 4 + 0];
    float w1 = mapWights[threadid * 4 + 1];
    float w2 = mapWights[threadid * 4 + 2];
    float w3 = mapWights[threadid * 4 + 3];
    float tVPos0[3] = {tetVertPos[id0 * 3 + 0], tetVertPos[id0 * 3 + 1], tetVertPos[id0 * 3 + 2]};
    float tVPos1[3] = {tetVertPos[id1 * 3 + 0], tetVertPos[id1 * 3 + 1], tetVertPos[id1 * 3 + 2]};
    float tVPos2[3] = {tetVertPos[id2 * 3 + 0], tetVertPos[id2 * 3 + 1], tetVertPos[id2 * 3 + 2]};
    float tVPos3[3] = {tetVertPos[id3 * 3 + 0], tetVertPos[id3 * 3 + 1], tetVertPos[id3 * 3 + 2]};

    for (int i = 0; i < 3; i++) {
        vertPosPrev[threadid * 3 + i] = vertPos[threadid * 3 + i] = w0 * tVPos0[i] + w1 * tVPos1[i] + w2 * tVPos2[i] + w3 * tVPos3[i];
    }
}

void PDSolver_MG::runInterpolate() {
    int threadNum = 512;
    int blockNum = (m_pdSolverFine->m_tetVertNum + threadNum - 1) / threadNum;
    updatePointInTet<<<blockNum, threadNum>>>(m_pdSolverFine->m_tetVertNum, m_pdSolverFine->pdSolverData->tetVertPos_d,
                                              m_pdSolverFine->pdSolverData->tetVertPos_prev_d, m_pdSolverCoarse->pdSolverData->tetVertPos_d, interpolationIds_d,
                                              interpolationWights_d);
    PRINT_CUDA_ERROR_AFTER("runInterpolate");
}

void PDSolver_MG::runAverage() {
    int threadNum = 512;
    int blockNum = (m_pdSolverCoarse->m_tetVertNum + threadNum - 1) / threadNum;
    updatePointInTet<<<blockNum, threadNum>>>(m_pdSolverCoarse->m_tetVertNum, m_pdSolverCoarse->pdSolverData->tetVertPos_d,
                                              m_pdSolverCoarse->pdSolverData->tetVertPos_prev_d, m_pdSolverFine->pdSolverData->tetVertPos_d, averageIds_d,
                                              averageWeights_d);
    PRINT_CUDA_ERROR_AFTER("runAverage");
}

__global__ void updateMapping(int tetVertNumFine, float* tetVertPosFine, int* interpolationIds, float* interpolationWights, float* tetVertPosCoarse) {
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid >= tetVertNumFine) return;

    float tVPosFine[3] = {tetVertPosFine[threadid * 3 + 0], tetVertPosFine[threadid * 3 + 1], tetVertPosFine[threadid * 3 + 2]};

    int id0 = interpolationIds[threadid * 4 + 0];
    int id1 = interpolationIds[threadid * 4 + 1];
    int id2 = interpolationIds[threadid * 4 + 2];
    int id3 = interpolationIds[threadid * 4 + 3];
    Point3D tVPosCoarse0 = {tetVertPosCoarse[id0 * 3 + 0], tetVertPosCoarse[id0 * 3 + 1], tetVertPosCoarse[id0 * 3 + 2]};
    Point3D tVPosCoarse1 = {tetVertPosCoarse[id1 * 3 + 0], tetVertPosCoarse[id1 * 3 + 1], tetVertPosCoarse[id1 * 3 + 2]};
    Point3D tVPosCoarse2 = {tetVertPosCoarse[id2 * 3 + 0], tetVertPosCoarse[id2 * 3 + 1], tetVertPosCoarse[id2 * 3 + 2]};
    Point3D tVPosCoarse3 = {tetVertPosCoarse[id3 * 3 + 0], tetVertPosCoarse[id3 * 3 + 1], tetVertPosCoarse[id3 * 3 + 2]};

    float weights[4] = {0.25f};
    barycentricCoordinate(tVPosFine, tVPosCoarse0, tVPosCoarse1, tVPosCoarse2, tVPosCoarse3, weights);
    interpolationWights[threadid * 4 + 0] = weights[0];
    interpolationWights[threadid * 4 + 1] = weights[1];
    interpolationWights[threadid * 4 + 2] = weights[2];
    interpolationWights[threadid * 4 + 3] = weights[3];
}

void PDSolver_MG::runUpdateMapping() {
    int threadNum = 512;
    int blockNum = (m_pdSolverFine->m_tetVertNum + threadNum - 1) / threadNum;
    updateMapping<<<blockNum, threadNum>>>(m_pdSolverFine->m_tetVertNum, m_pdSolverFine->pdSolverData->tetVertPos_d, interpolationIds_d, interpolationWights_d,
                                           m_pdSolverCoarse->pdSolverData->tetVertPos_d);
    PRINT_CUDA_ERROR_AFTER("runUpdateMapping");
}

//////////////////////////////////////////// test code ////////////////////////////////////////////

double deltaXFirst = 0;
void PDSolverData::runTestConvergence(int iter) {
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

void PDSolverData::runCalEnergy(float m_dt, const vector<float>& m_tetVertMass, const vector<int>& m_tetIndex, const vector<float>& m_tetInvD3x3,
                                const vector<float>& m_tetVolume, float m_volumnStiffness, float& Ek, float& Ep, float& dX, bool calEveryVertEp) {
    vector<float> tetVertPos_h(tetVertNum * 3);
    vector<float> tetVertPos_old_h(tetVertNum * 3);
    vector<float> tetVertPos_prev_h(tetVertNum * 3);
    cudaMemcpy(tetVertPos_h.data(), tetVertPos_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(tetVertPos_old_h.data(), tetVertPos_old_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(tetVertPos_prev_h.data(), tetVertPos_prev_d, tetVertNum * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    if (calEveryVertEp) {
        assert(g_simulator->m_tetVertEpDensity.size() == tetVertNum);
        fill(g_simulator->m_tetVertEpDensity.begin(), g_simulator->m_tetVertEpDensity.end(), 0);
        fill(g_simulator->m_tetVertEpSum.begin(), g_simulator->m_tetVertEpSum.end(), 0);
        fill(g_simulator->m_tetVertVSum.begin(), g_simulator->m_tetVertVSum.end(), 0);
    }
    Ek = Ep = dX = 0;

    // 计算 deltaX, Ek
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

        dX += (pow((vx - vx_pre), 2) + pow((vy - vy_pre), 2) + pow((vz - vz_pre), 2));
        Ek += m_tetVertMass[i] * (pow((vx - vx_st), 2) + pow((vy - vy_st), 2) + pow((vz - vz_st), 2));
    }
    dX = sqrt(dX);
    Ek = Ek / (2 * m_dt * m_dt);

    // 计算 Ep
    for (int i = 0; i < tetNum; i++) {
        int vIndex0 = m_tetIndex[i * 4 + 0];
        int vIndex1 = m_tetIndex[i * 4 + 1];
        int vIndex2 = m_tetIndex[i * 4 + 2];
        int vIndex3 = m_tetIndex[i * 4 + 3];

        Point3D vert0 = {tetVertPos_h[vIndex0 * 3 + 0], tetVertPos_h[vIndex0 * 3 + 1], tetVertPos_h[vIndex0 * 3 + 2]};
        Point3D vert1 = {tetVertPos_h[vIndex1 * 3 + 0], tetVertPos_h[vIndex1 * 3 + 1], tetVertPos_h[vIndex1 * 3 + 2]};
        Point3D vert2 = {tetVertPos_h[vIndex2 * 3 + 0], tetVertPos_h[vIndex2 * 3 + 1], tetVertPos_h[vIndex2 * 3 + 2]};
        Point3D vert3 = {tetVertPos_h[vIndex3 * 3 + 0], tetVertPos_h[vIndex3 * 3 + 1], tetVertPos_h[vIndex3 * 3 + 2]};
        float volumn = GetVolumn(vert0, vert1, vert2, vert3);

        // 先计算shape矩阵
        float D[9];
        D[0] = vert1.x - vert0.x;
        D[1] = vert2.x - vert0.x;
        D[2] = vert3.x - vert0.x;
        D[3] = vert1.y - vert0.y;
        D[4] = vert2.y - vert0.y;
        D[5] = vert3.y - vert0.y;
        D[6] = vert1.z - vert0.z;
        D[7] = vert2.z - vert0.z;
        D[8] = vert3.z - vert0.z;

        // 计算形变梯度F
        float F[9];
        MatrixProduct_3_D(D, &m_tetInvD3x3[i * 9], F);

        // 从F中分解出R（直接搬运，这个算法太复杂了）
        float R[9];
        float deltaF;
        GetRotation_D((float(*)[3])F, (float(*)[3])R, deltaF);  // 转化为数组指针，即对应二维数组的形参要求

        double e = 0;
        for (int i = 0; i < 9; i++) {
            e += pow((F[i] - R[i]), 2);
        }
        e = e * m_tetVolume[i] * m_volumnStiffness / 2;

        Ep += e;
        if (calEveryVertEp) {
            if (deltaF < 1.0f) e = -e;
            g_simulator->m_tetVertEpSum[vIndex0] += e;
            g_simulator->m_tetVertEpSum[vIndex1] += e;
            g_simulator->m_tetVertEpSum[vIndex2] += e;
            g_simulator->m_tetVertEpSum[vIndex3] += e;
            g_simulator->m_tetVertVSum[vIndex0] += volumn;
            g_simulator->m_tetVertVSum[vIndex1] += volumn;
            g_simulator->m_tetVertVSum[vIndex2] += volumn;
            g_simulator->m_tetVertVSum[vIndex3] += volumn;
        }
    }

    if (calEveryVertEp) {
        float maxEpDensity = -FLT_MAX;
        float minEpDensity = FLT_MAX;
        for (int i = 0; i < tetVertNum; i++) {
            g_simulator->m_tetVertEpDensity[i] = g_simulator->m_tetVertEpSum[i] / g_simulator->m_tetVertVSum[i];
            maxEpDensity = max(maxEpDensity, g_simulator->m_tetVertEpDensity[i]);
            minEpDensity = min(minEpDensity, g_simulator->m_tetVertEpDensity[i]);
        }
        // printf("maxEpDensity = %f, minEpDensity = %f\n", maxEpDensity, minEpDensity);
    }
}