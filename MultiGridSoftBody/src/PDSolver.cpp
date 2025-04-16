#include "PDSolver.h"

#include <chrono>
#include <iostream>

#include "global.h"
using namespace std;
double duration_physical = 0;

void runInitialize(int tetNum_h, int tetVertNum_h, int* tetIndex_h, float* tetInvD3x3_h, float* tetInvD3x4_h, float* tetVolume_h, float* tetVolumeDiag_h,
                   float* tetVertMass_h, float* tetVertFixed_h, float* tetVertPos_h);
void runCalculateST(float m_damping, float m_dt, float m_gravityX, float m_gravityY, float m_gravityZ);
void runClearTemp();
void runCalculateIF(float m_volumnStiffness);
void runcalculatePOS(float omega, float m_dt);
void runCalculateV(float m_dt);
void runCpyTetVertForRender();
void runTestConvergence(int iter);
void runCalEnergy(int iter, float m_dt, const vector<float>& m_tetVertMass, const vector<int>& m_tetIndex, const vector<float>& m_tetInvD3x3,
                  const vector<float>& m_tetVolume, float m_volumnStiffness);

void PDSolver::InitVolumeConstraint() {
    m_tetVertMass.resize(m_tetVertNum);
    m_tetVolumeDiag.resize(m_tetVertNum);
    m_tetInvD3x3.resize(m_tetNum * 9);
    m_tetInvD3x4.resize(m_tetNum * 12);

    float volumnSum = 0;
    for (int i = 0; i < m_tetNum; i++) {
        // 计算每个四面体初始化的shape矩阵的逆
        int vIndex0 = m_tetIndex[i * 4 + 0];
        int vIndex1 = m_tetIndex[i * 4 + 1];
        int vIndex2 = m_tetIndex[i * 4 + 2];
        int vIndex3 = m_tetIndex[i * 4 + 3];

        // 先计算shape矩阵
        float D[9];
        D[0] = m_tetVertPos[vIndex1 * 3 + 0] - m_tetVertPos[vIndex0 * 3 + 0];
        D[1] = m_tetVertPos[vIndex2 * 3 + 0] - m_tetVertPos[vIndex0 * 3 + 0];
        D[2] = m_tetVertPos[vIndex3 * 3 + 0] - m_tetVertPos[vIndex0 * 3 + 0];
        D[3] = m_tetVertPos[vIndex1 * 3 + 1] - m_tetVertPos[vIndex0 * 3 + 1];
        D[4] = m_tetVertPos[vIndex2 * 3 + 1] - m_tetVertPos[vIndex0 * 3 + 1];
        D[5] = m_tetVertPos[vIndex3 * 3 + 1] - m_tetVertPos[vIndex0 * 3 + 1];
        D[6] = m_tetVertPos[vIndex1 * 3 + 2] - m_tetVertPos[vIndex0 * 3 + 2];
        D[7] = m_tetVertPos[vIndex2 * 3 + 2] - m_tetVertPos[vIndex0 * 3 + 2];
        D[8] = m_tetVertPos[vIndex3 * 3 + 2] - m_tetVertPos[vIndex0 * 3 + 2];

        // 计算D的逆,顺便记录体积
        m_tetVolume.push_back(fabs(Matrix_Inverse_3(D, &m_tetInvD3x3[i * 9])) / 6.0);
        volumnSum += m_tetVolume[i];

        // 计算质量
        m_tetVertMass[vIndex0] += m_tetVolume[i] / 4;
        m_tetVertMass[vIndex1] += m_tetVolume[i] / 4;
        m_tetVertMass[vIndex2] += m_tetVolume[i] / 4;
        m_tetVertMass[vIndex3] += m_tetVolume[i] / 4;

        float* inv_D = &m_tetInvD3x3[i * 9];
        // 论文中的AC矩阵
        m_tetInvD3x4[i * 12 + 0] = -inv_D[0] - inv_D[3] - inv_D[6];
        m_tetInvD3x4[i * 12 + 1] = inv_D[0];
        m_tetInvD3x4[i * 12 + 2] = inv_D[3];
        m_tetInvD3x4[i * 12 + 3] = inv_D[6];
        m_tetInvD3x4[i * 12 + 4] = -inv_D[1] - inv_D[4] - inv_D[7];
        m_tetInvD3x4[i * 12 + 5] = inv_D[1];
        m_tetInvD3x4[i * 12 + 6] = inv_D[4];
        m_tetInvD3x4[i * 12 + 7] = inv_D[7];
        m_tetInvD3x4[i * 12 + 8] = -inv_D[2] - inv_D[5] - inv_D[8];
        m_tetInvD3x4[i * 12 + 9] = inv_D[2];
        m_tetInvD3x4[i * 12 + 10] = inv_D[5];
        m_tetInvD3x4[i * 12 + 11] = inv_D[8];

        for (int j = 0; j < 12; j++) {
            if (m_tetInvD3x4[i * 12 + j] != m_tetInvD3x4[i * 12 + j]) {
                printf("m_tetInvD3x4[%d] is nan\n", i * 12 + j);
            }
        }

        // 记录该点的对应的对角矩阵分量（用于雅各比迭代，因为只需要对角阵就可以实现）
        m_tetVolumeDiag[vIndex0] += m_tetInvD3x4[i * 12 + 0] * m_tetInvD3x4[i * 12 + 0] * m_tetVolume[i] * m_volumnStiffness;  // 第i个四面体中的第一个点
        m_tetVolumeDiag[vIndex0] += m_tetInvD3x4[i * 12 + 4] * m_tetInvD3x4[i * 12 + 4] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex0] += m_tetInvD3x4[i * 12 + 8] * m_tetInvD3x4[i * 12 + 8] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex1] += m_tetInvD3x4[i * 12 + 1] * m_tetInvD3x4[i * 12 + 1] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex1] += m_tetInvD3x4[i * 12 + 5] * m_tetInvD3x4[i * 12 + 5] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex1] += m_tetInvD3x4[i * 12 + 9] * m_tetInvD3x4[i * 12 + 9] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex2] += m_tetInvD3x4[i * 12 + 2] * m_tetInvD3x4[i * 12 + 2] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex2] += m_tetInvD3x4[i * 12 + 6] * m_tetInvD3x4[i * 12 + 6] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex2] += m_tetInvD3x4[i * 12 + 10] * m_tetInvD3x4[i * 12 + 10] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex3] += m_tetInvD3x4[i * 12 + 3] * m_tetInvD3x4[i * 12 + 3] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex3] += m_tetInvD3x4[i * 12 + 7] * m_tetInvD3x4[i * 12 + 7] * m_tetVolume[i] * m_volumnStiffness;
        m_tetVolumeDiag[vIndex3] += m_tetInvD3x4[i * 12 + 11] * m_tetInvD3x4[i * 12 + 11] * m_tetVolume[i] * m_volumnStiffness;
    }
}

void PDSolver::SetFixedVert() {
    m_tetVertFixed.resize(m_tetVertNum, 1.0f);
    for (int i = 0; i < m_tetVertNum; i++) {
        float z = m_tetVertPos[i * 3 + 2];
        if (fabs(z - 10) < 1e-5) m_tetVertFixed[i] = 0.0f;
    }
}

void PDSolver::Init() {
    m_iterNum = 16;
    m_dt = 1.0f / 30.0f;
    m_damping = 0.5f;
    m_volumnStiffness = 1000.0f;
    m_rho = 0.9992f;
    m_gravityX = 0.0f;
    m_gravityY = -9.8f;
    m_gravityZ = 0.0f;

    m_tetNum = g_simulator->m_tetIdx.size() / 4;
    m_tetVertNum = g_simulator->m_tetVertPos.size() / 3;
    m_tetIndex = g_simulator->m_tetIdx;
    m_tetVertPos = g_simulator->m_tetVertPos;
    InitVolumeConstraint();
    LOG(INFO) << "InitVolumeConstraint 结束";
    SetFixedVert();
    runInitialize(m_tetNum, m_tetVertNum, m_tetIndex.data(), m_tetInvD3x3.data(), m_tetInvD3x4.data(), m_tetVolume.data(), m_tetVolumeDiag.data(),
                  m_tetVertMass.data(), m_tetVertFixed.data(), m_tetVertPos.data());
}

void PDSolver::Step() {
    auto start = std::chrono::high_resolution_clock::now();
    runCalculateST(m_damping, m_dt, m_gravityX, m_gravityY, m_gravityZ);
    float omega = 1.0f;
    for (int i = 0; i < m_iterNum; i++) {
        runCalEnergy(i, m_dt, m_tetVertMass, m_tetIndex, m_tetInvD3x3, m_tetVolume, m_volumnStiffness);  // 计算能量，测 fps 时需要注释

        runClearTemp();
        runCalculateIF(m_volumnStiffness);
        omega = 4 / (4 - m_rho * m_rho * omega);
        runcalculatePOS(omega, m_dt);
    }
    runCalculateV(m_dt);
    runCpyTetVertForRender();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    duration_physical = duration.count();
}