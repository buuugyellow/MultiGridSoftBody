#include "PDSolver.h"

#include <chrono>
#include <iostream>

#include "global.h"

using namespace std;
double duration_physical = 0;

void PDSolver::InitVolumeConstraint() {
    m_tetVertMass.resize(m_tetVertNum);
    m_tetVolumeDiag.resize(m_tetVertNum);
    m_tetInvD3x3.resize(m_tetNum * 9);
    m_tetInvD3x4.resize(m_tetNum * 12);
    m_tetCenter.resize(m_tetNum * 3, 0);
    m_tetFaceArea.resize(m_tetNum * 4, 0);
    m_tetFaceNormal.resize(m_tetNum * 12, 0);

    float volumnSum = 0;
    for (int i = 0; i < m_tetNum; i++) {
        // ����ÿ���������ʼ����shape�������
        int vIndex0 = m_tetIndex[i * 4 + 0];
        int vIndex1 = m_tetIndex[i * 4 + 1];
        int vIndex2 = m_tetIndex[i * 4 + 2];
        int vIndex3 = m_tetIndex[i * 4 + 3];

        // �ȼ���shape����
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

        // ����D����,˳���¼���
        m_tetVolume.push_back(fabs(Matrix_Inverse_3(D, &m_tetInvD3x3[i * 9])) / 6.0);
        volumnSum += m_tetVolume[i];

        // ��������
        m_tetVertMass[vIndex0] += m_tetVolume[i] / 4;
        m_tetVertMass[vIndex1] += m_tetVolume[i] / 4;
        m_tetVertMass[vIndex2] += m_tetVolume[i] / 4;
        m_tetVertMass[vIndex3] += m_tetVolume[i] / 4;

        float* inv_D = &m_tetInvD3x3[i * 9];
        // �����е�AC����
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

        // ��¼�õ�Ķ�Ӧ�ĶԽǾ�������������Ÿ��ȵ�������Ϊֻ��Ҫ�Խ���Ϳ���ʵ�֣�
        m_tetVolumeDiag[vIndex0] += m_tetInvD3x4[i * 12 + 0] * m_tetInvD3x4[i * 12 + 0] * m_tetVolume[i] * m_volumnStiffness;  // ��i���������еĵ�һ����
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

        // ����ÿ�����������������
        vector<int> vertIds = {vIndex0, vIndex1, vIndex2, vIndex3};
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 4; k++) {
                m_tetCenter[i * 3 + j] += m_tetVertPos[vertIds[k] * 3 + j];
            }
            m_tetCenter[i * 3 + j] /= 4;
        }

        // ����ÿ���������ĸ��������
        vector<vector<int>> faces = {{vIndex1, vIndex2, vIndex3}, {vIndex0, vIndex2, vIndex3}, {vIndex0, vIndex1, vIndex3}, {vIndex0, vIndex1, vIndex2}};
        for (int k = 0; k < 4; k++) {
            int D_id = m_tetIndex[i * 4 + k];
            int A_id = faces[k][0];
            int B_id = faces[k][1];
            int C_id = faces[k][2];
            Point3D A(m_tetVertPos.data() + A_id * 3);
            Point3D B(m_tetVertPos.data() + B_id * 3);
            Point3D C(m_tetVertPos.data() + C_id * 3);
            Point3D D(m_tetVertPos.data() + D_id * 3);

            // ���������
            const Point3D AB = B - A;
            const Point3D AC = C - A;

            // ���㷨����
            Point3D normal = crossProduct(AB, AC);
            const Point3D AD = D - A;
            if (dotProduct(normal, AD) > 0) {
                normal.x = -normal.x;
                normal.y = -normal.y;
                normal.z = -normal.z;
            }

            // �������
            double len = vectorLength(normal);
            m_tetFaceArea[i * 4 + k] = 0.5 * len;
            assert(fabs(len) > 1e-5);

            // �����һ���ķ�����
            normal = normal / len;
            m_tetFaceNormal[i * 12 + k * 3 + 0] = normal.x;
            m_tetFaceNormal[i * 12 + k * 3 + 1] = normal.y;
            m_tetFaceNormal[i * 12 + k * 3 + 2] = normal.z;
        }
    }
}

void PDSolver::SetFixedVert() {
    m_tetVertFixed.resize(m_tetVertNum, 1.0f);
    for (int i = 0; i < m_tetVertNum; i++) {
        float z = m_tetVertPos[i * 3 + 2];
        if (fabs(z - 10) < 1e-5) m_tetVertFixed[i] = 0.0f;
    }
}

void PDSolver::Init(const vector<int>& tetIdx, const vector<float> tetVertPos) {
    m_iterNum = 80;
    m_iterNumCvg = 128;
    m_dt = 1.0f / 30.0f;
    m_damping = 0.5f;
    m_volumnStiffness = 1000.0f;
    m_rho = 0.9992f;
    m_gravityX = 0.0f;
    m_gravityY = -9.8f;
    m_gravityZ = 0.0f;

    m_tetIndex = tetIdx;
    m_tetVertPos = tetVertPos;
    m_tetNum = m_tetIndex.size() / 4;
    m_tetVertNum = m_tetVertPos.size() / 3;

    InitVolumeConstraint();
    LOG(INFO) << "InitVolumeConstraint ����";

    SetFixedVert();
    pdSolverData = new PDSolverData();
    pdSolverData->Init(m_tetNum, m_tetVertNum, m_tetIndex.data(), m_tetInvD3x3.data(), m_tetInvD3x4.data(), m_tetVolume.data(), m_tetVolumeDiag.data(),
                       m_tetVertMass.data(), m_tetVertFixed.data(), m_tetVertPos.data());
    LOG(INFO) << "pdSolverData Init ����";
}

void PDSolver::StepForConvergence() {
    float Ek, Ep, dX;
    pdSolverData->runSaveVel();
    pdSolverData->runCalculateST(m_damping, m_dt, m_gravityX, m_gravityY, m_gravityZ);
    pdSolverData->runCalEnergy(m_dt, m_tetVertMass, m_tetIndex, m_tetInvD3x3, m_tetVolume, m_volumnStiffness, Ek, Ep, dX);
    fprintf(energyOutputFile, "%d,%f,%f,%f,%f\n", 0, Ek + Ep, Ek, Ep, dX);
    float omega = 1.0f;
    for (int i = 0; i < m_iterNumCvg; i++) {
        pdSolverData->runClearTemp();
        pdSolverData->runCalculateIF(m_volumnStiffness);
        omega = 4 / (4 - m_rho * m_rho * omega);
        pdSolverData->runcalculatePOS(omega, m_dt);

        pdSolverData->runCalEnergy(m_dt, m_tetVertMass, m_tetIndex, m_tetInvD3x3, m_tetVolume, m_volumnStiffness, Ek, Ep, dX);
        fprintf(energyOutputFile, "%d,%f,%f,%f,%f\n", i + 1, Ek + Ep, Ek, Ep, dX);
    }
    pdSolverData->runResetPosVel();
    g_conEnergy_V2 = Ek + Ep;
}

void PDSolver::RenderOnce() { 
    pdSolverData->runCpyTetVertForRender();
    renderOnce();
}

void PDSolver::Step() {
    static float E0 = 0;
    float Ek, Ep, dX, error;
    auto start = std::chrono::high_resolution_clock::now();

    StepForConvergence();

    pdSolverData->runCalculateST(m_damping, m_dt, m_gravityX, m_gravityY, m_gravityZ);
    pdSolverData->runCalEnergy(m_dt, m_tetVertMass, m_tetIndex, m_tetInvD3x3, m_tetVolume, m_volumnStiffness, Ek, Ep, dX, true);
    E0 = Ep + Ek;
    if (g_stepCnt < 200) error = (Ek + Ep - g_conEnergy_V2) / (E0 - g_conEnergy_V2);
    fprintf(energyOutputFile, "%d,%f,%f,%f,%f,%f\n", 0, Ek + Ep, Ek, Ep, dX, error);
    //RenderOnce();

    float omega = 1.0f;
    for (int i = 0; i < m_iterNum; i++) {
        pdSolverData->runClearTemp();
        pdSolverData->runCalculateIF(m_volumnStiffness);
        omega = 4 / (4 - m_rho * m_rho * omega);
        pdSolverData->runcalculatePOS(omega, m_dt);
        
        pdSolverData->runCalEnergy(m_dt, m_tetVertMass, m_tetIndex, m_tetInvD3x3, m_tetVolume, m_volumnStiffness, Ek, Ep, dX, true);
        if (g_stepCnt < 200) error = (Ek + Ep - g_conEnergy_V2) / (E0 - g_conEnergy_V2);
        fprintf(energyOutputFile, "%d,%f,%f,%f,%f,%f\n", i + 1, Ek + Ep, Ek, Ep, dX, error);
        //RenderOnce();
    }
    pdSolverData->runCalculateV(m_dt);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    duration_physical = duration.count();
    fprintf(timeOutputFile, "%d,%f\n", g_stepCnt, duration_physical);

    pdSolverData->runCalEnergy(m_dt, m_tetVertMass, m_tetIndex, m_tetInvD3x3, m_tetVolume, m_volumnStiffness, Ek, Ep, dX, true);
    RenderOnce();
}