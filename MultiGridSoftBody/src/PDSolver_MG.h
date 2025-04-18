#pragma once
#include "PDSolver.h"

class PDSolver_MG {
public:
    PDSolver* m_pdSolverCoarse; // 粗网格解算器
    PDSolver* m_pdSolverFine; // 细网格解算器
    vector<int> m_interpolationIds; // 细四面体顶点绑定的粗四面体，4 个粗四面体索引
    vector<float> m_interpolationWights; // 权重

    void Init(const vector<int>& tetIdxCoarse, const vector<float> tetVertPosCoarse, const vector<int>& tetIdxFine, const vector<float> tetVertPosFine);
    void Step();
};