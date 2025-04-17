#pragma once
#include "PDSolver.h"

class PDSolver_MG {
public:
    PDSolver* m_pdSolverCoarse; // 粗网格解算器
    PDSolver* m_pdSolverFine; // 细网格解算器

    void Init(vector<int>& tetIdxCoarse, vector<float> tetVertPosCoarse, vector<int>& tetIdxFine, vector<float> tetVertPosFine);
    void Step();
};