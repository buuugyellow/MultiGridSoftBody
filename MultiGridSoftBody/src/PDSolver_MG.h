#pragma once
#include "PDSolver.h"

class PDSolver_MG {
public:
    PDSolver* m_pdSolverCoarse; // �����������
    PDSolver* m_pdSolverFine; // ϸ���������

    void Init(vector<int>& tetIdxCoarse, vector<float> tetVertPosCoarse, vector<int>& tetIdxFine, vector<float> tetVertPosFine);
    void Step();
};