#pragma once
#include <vector>
#include "simpleMath.h"

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
    float* tetVertVelocityBak_d;  // 速度的备份，用于收敛 step 中的计算
    float* tetVertExternForce_d;  // 外力，tetVertNum*3
    float* tetVertForce_d;        // 顶点受力, tetVertNum*3

    char* tetVertIsCollied_d;       // 发生碰撞为 1，否则为 0
    float* tetVertCollisionDiag_d;  // 碰撞能量的 Hessian 阵的对角阵（向量存储）
    float* tetVertCollisionForce_d; // 碰撞能量的一阶导向量

    void Init(int tetNum_h, int tetVertNum_h, int* tetIndex_h, float* tetInvD3x3_h, float* tetInvD3x4_h, float* tetVolume_h, float* tetVolumeDiag_h,
              float* tetVertMass_h, float* tetVertFixed_h, float* tetVertPos_h);
    void runCalculateST(float m_damping, float m_dt, float m_gravityX, float m_gravityY, float m_gravityZ);
    void runClearTemp();
    void runCalculateIF(float m_volumnStiffness);
    void runcalculatePOS(float omega, float m_dt);
    void runCalculateV(float m_dt);
    void runCpyTetVertForRender();
    void runResetPosVel();
    void runSaveVel();
    void runTestConvergence(int iter);
    void runCalEnergy(float m_dt, const vector<float>& m_tetVertMass, const vector<int>& m_tetIndex, const vector<float>& m_tetInvD3x3,
                      const vector<float>& m_tetVolume, float m_volumnStiffness, float& Ek, float& Ep, float& dX, bool calEveryVertEp = false);
    void runClearCollision();
    void runDCDByPoint_sphere(Point3D center, float radius, float collisionStiffness);
    void runDCDByTriangle_sphere(Point3D center, float radius, float collisionStiffness);
};

class PDSolver {
public:
    int m_iterNum;
    int m_iterNumCvg;  // 收敛所需的迭代次数
    float m_dt;
    float m_damping;
    float m_volumnStiffness;
    float m_collisionStiffness;
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
    vector<float> m_tetFaceNormal;  // 这个是每个四面体的四个面的法向量，面的顺序和 m_tetIndex 中对应点的顺序一致
    vector<float> m_tetFaceArea;    // 每个四面体的四个面的面积，顺序同上
    vector<float> m_tetCenter;      // 四面体重心坐标
    PDSolverData* pdSolverData;

    void Init(const vector<int>& tetIdx, const vector<float> tetVertPos);
    void Step();
    void StepForConvergence();
    void InitVolumeConstraint();
    void SetFixedVert();
    void RenderOnce();
    void DCDByPoint();
    void DCDByTriangle();
};