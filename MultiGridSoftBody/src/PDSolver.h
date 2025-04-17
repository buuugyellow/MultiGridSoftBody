#pragma once
#include <vector>

using namespace std;
struct PDSolverData {
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

    void Init(int tetNum_h, int tetVertNum_h, int* tetIndex_h, float* tetInvD3x3_h, float* tetInvD3x4_h, float* tetVolume_h, float* tetVolumeDiag_h,
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
};

class PDSolver {
public:
    int m_iterNum;
    float m_dt;
    float m_damping;
    float m_volumnStiffness;
    float m_rho;
    float m_gravityX;
    float m_gravityY;
    float m_gravityZ;

    int m_tetNum;
    int m_tetVertNum;
    vector<int> m_tetIndex;
    vector<float> m_tetInvD3x3;
    vector<float> m_tetInvD3x4;
    vector<float> m_tetVolume;
    vector<float> m_tetVolumeDiag;
    vector<float> m_tetVertMass;
    vector<float> m_tetVertFixed;
    vector<float> m_tetVertPos;

    PDSolverData* pdSolverData;

    void Init(vector<int>& tetIdx, vector<float> tetVertPos);
    void Step();
    void InitVolumeConstraint();
    void SetFixedVert();
};