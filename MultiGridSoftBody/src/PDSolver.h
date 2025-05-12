#pragma once
#include <vector>
#include "simpleMath.h"

using namespace std;
struct PDSolverData {
    int tetNum;              // ����������
    int tetVertNum;          // �����嶥������
    float* tetVertPos_d;     // ��ǰλ�ã�tetVertNum*3
    int* tetIndex_d;         // ����������
    float* tetInvD3x3_d;     // �����, tetNum*9
    float* tetInvD3x4_d;     // Ac�� tetNum*12
    float* tetVolume_d;      // �����������tetNum
    float* tetVolumeDiag_d;  // �����嶥���α��ݶȣ�tetVertNum_d
    float* tetVertMass_d;    // ������tetVertNum_d*3
    float* tetVertFixed_d;   // �����嶥���Ƿ�̶���0.0f��ʾû�й̶���tetVertNum

    float* tetVertPos_last_d;     // ��һʱ��λ�ã�tetVertNum*3
    float* tetVertPos_old_d;      // st��tetVertNum*3
    float* tetVertPos_prev_d;     // ��һ�ε�����tetVertNum*3
    float* tetVertPos_next_d;     // ��һ��λ�ã�tetVertNum*3
    float* tetVertVelocity_d;     // �ٶȣ�tetVertNum*3
    float* tetVertVelocityBak_d;  // �ٶȵı��ݣ��������� step �еļ���
    float* tetVertExternForce_d;  // ������tetVertNum*3
    float* tetVertForce_d;        // ��������, tetVertNum*3

    char* tetVertIsCollied_d;       // ������ײΪ 1������Ϊ 0
    float* tetVertCollisionDiag_d;  // ��ײ������ Hessian ��ĶԽ��������洢��
    float* tetVertCollisionForce_d; // ��ײ������һ�׵�����

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
    int m_iterNumCvg;  // ��������ĵ�������
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
    vector<float> m_tetFaceNormal;  // �����ÿ����������ĸ���ķ����������˳��� m_tetIndex �ж�Ӧ���˳��һ��
    vector<float> m_tetFaceArea;    // ÿ����������ĸ���������˳��ͬ��
    vector<float> m_tetCenter;      // ��������������
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