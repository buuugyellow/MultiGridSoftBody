#pragma once
#include "PDSolver.h"

class PDSolver_MG {
public:
    PDSolver* m_pdSolverCoarse; // �����������
    PDSolver* m_pdSolverFine; // ϸ���������
    vector<int> m_interpolationIds; // ϸ�����嶥��󶨵Ĵ������壬4 ��������������
    vector<float> m_interpolationWights; // Ȩ��
    vector<int> m_averageIds; // �������嶥��󶨵�ϸ�����壬4 ��ϸ����������
    vector<float> m_averageWeights; // Ȩ��
    int* interpolationIds_d;
    float* interpolationWights_d;
    int* averageIds_d;
    float* averageWeights_d;

    void Init(const vector<int>& tetIdxCoarse, const vector<float> tetVertPosCoarse, const vector<int>& tetIdxFine, const vector<float> tetVertPosFine);
    void Step();
    void runInterpolate();
    void runAverage();
    void runUpdateMapping();
};