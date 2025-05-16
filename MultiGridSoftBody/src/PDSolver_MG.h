#pragma once
#include "PDSolver.h"

class PDSolver_MG {
public:
    PDSolver* m_pdSolverCoarse; // 粗网格解算器
    PDSolver* m_pdSolverFine; // 细网格解算器
    vector<int> m_interpolationIds; // 细四面体顶点绑定的粗四面体，4 个粗四面体索引
    vector<float> m_interpolationWights; // 权重
    vector<int> m_averageIds; // 粗四面体顶点绑定的细四面体，4 个细四面体索引
    vector<float> m_averageWeights; // 权重
    int* interpolationIds_d;
    float* interpolationWights_d;
    int* averageIds_d;
    float* averageWeights_d;

    void interpolate(); // 细网格顶点寻找所在的粗四面体并计算重心坐标
    void Init(const vector<float>& tetVertPosCoarse, const vector<int>& tetIdxCoarse, const vector<unsigned int>& tetFaceIdxCoarse,
              const vector<unsigned int>& tetFaceOppositeTetVertIdxCoarse, const vector<float>& tetVertPosFine, const vector<int>& tetIdxFine,
              const vector<unsigned int>& tetFaceIdxFine, const vector<unsigned int>& tetFaceOppositeTetVertIdxFine);
    void Step();
    void runInterpolate();
    void runAverage();
    void runUpdateMapping();
};