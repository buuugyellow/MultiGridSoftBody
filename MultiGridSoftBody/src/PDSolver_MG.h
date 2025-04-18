#pragma once
#include "PDSolver.h"

class PDSolver_MG {
public:
    PDSolver* m_pdSolverCoarse; // �����������
    PDSolver* m_pdSolverFine; // ϸ���������
    vector<int> m_interpolationIds; // ϸ�����嶥��󶨵Ĵ������壬4 ��������������
    vector<float> m_interpolationWights; // Ȩ��

    void Init(const vector<int>& tetIdxCoarse, const vector<float> tetVertPosCoarse, const vector<int>& tetIdxFine, const vector<float> tetVertPosFine);
    void Step();
};