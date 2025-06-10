#pragma once
#include <set>
#include <vector>

#include "simpleMath.h"

using namespace std;
struct PDSolverData {
    int tetNum;                                 // ����������
    int tetVertNum;                             // �����嶥������
    int outsideTriNum;                          // ���������������
    int outsideTetVertNum;                      // ����������嶥������
    float* tetVertPos_d;                        // ��ǰλ�ã�tetVertNum*3
    int* tetIndex_d;                            // ����������
    unsigned int* outsideTriIndex_d;            // ����������������
    unsigned int* outsideTriOppositeVertIds_d;  // ����������ζ�Ӧ�������嶥������
    float* outsideTriNormal_d;                  // ����������η�����
    unsigned int* outsideTetVertIds_d;          // ����������嶥����������
    float* tetVertNormal_d;                     // �����嶥��ķ���������������涥�㣬ƽ�����ڵ������εķ�����
    float* tetInvD3x3_d;                        // �����, tetNum*9
    float* tetInvD3x4_d;                        // Ac�� tetNum*12
    float* tetVolume_d;                         // �����������tetNum
    float* tetVolumeDiag_d;                     // �����嶥���α��ݶȣ�tetVertNum_d
    float* tetVertMass_d;                       // ������tetVertNum_d*3
    float* tetVertFixed_d;                      // �����嶥���Ƿ�̶���0.0f��ʾû�й̶���tetVertNum
    float* tetVertPos_last_d;                   // ��һʱ��λ�ã�tetVertNum*3
    float* tetVertPos_old_d;                    // st��tetVertNum*3
    float* tetVertPos_prev_d;                   // ��һ�ε�����tetVertNum*3
    float* tetVertPos_next_d;                   // ��һ��λ�ã�tetVertNum*3
    float* tetVertVelocity_d;                   // �ٶȣ�tetVertNum*3
    float* tetVertVelocityBak_d;                // �ٶȵı��ݣ��������� step �еļ���
    float* tetVertExternForce_d;                // ������tetVertNum*3
    float* tetVertForce_d;                      // ��������, tetVertNum*3
    int* tetVertIsCollided_d;                   // ������ײ���� 0������Ϊ 0
    float* tetVertCollisionDepth_d;               // ������ײ��ȣ�Ŀǰ���ڿ��ӻ�
    float* tetVertCollisionEnergy_d;            // ������ײ�������������ݷ���
    int* triIsCollided_d;                       // �������Ƿ�����ײ
    float* triColProjectVector_d;               // ��������ײ�ų�������
    float* tetVertCollisionDiag_d;              // ��ײ������ Hessian ��ĶԽ��������洢��
    float* tetVertCollisionForce_d;             // ��ײ������һ�׵�����
    float* tetDG_d;                              // �������α��ݶȣ�3*3��F = [x01, x02, x03][X01, X02, X03]^-1
    float* tetFR_d;                              // ������ F-R

    void Init(int tetNum_h, int tetVertNum_h, int* tetIndex_h, float* tetInvD3x3_h, float* tetInvD3x4_h, float* tetVolume_h, float* tetVolumeDiag_h,
              float* tetVertMass_h, float* tetVertFixed_h, float* tetVertPos_h, int outsideTriNum_h, unsigned int* outsideTriIndex_h, int outsideTetVertNum_h,
              unsigned int* outsideTetVertIds_h, unsigned int* outsideTriOppositeVertIds_h);
    void runCalculateST(float m_damping, float m_dt, float m_gravityX, float m_gravityY, float m_gravityZ);
    void runClearTemp();
    void runCalculateIF(float m_volumnStiffness);
    void runCalculateIFAc(float m_volumnStiffness);
    void runcalculatePOS(float omega, float m_dt);
    void runCalculateV(float m_dt);
    void runCpyTetVertForRender();
    void runResetPosVel();
    void runSaveVel();
    void runTestConvergence(int iter);
    void runCalEnergy(float m_dt, const vector<float>& m_tetVertMass, const vector<int>& m_tetIndex, const vector<float>& m_tetInvD3x3,
                      const vector<float>& m_tetVolume, float m_volumnStiffness, float& Ek, float& Ep, float& Ec, float& Nc, float& dX, bool calEveryVertEp = false);
    void runClearCollision();
    void runDCDByPoint_sphere(Point3D center, float radius, float collisionStiffness);
    void runUpdateTriNormal();
    void runClearTetVertNormal();
    void runAvgOutsideTetVertNormal();
    void runUpdateOutsideTetVertNormal();
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
    int m_outsideTriNum;
    int m_outsideTetVertNum;
    vector<int> m_tetIndex;
    vector<unsigned int> m_outsideTriIndex;           // ���������� [(v0,v2,v4), (v4,v6,v8)...]
    vector<unsigned int> m_outsideTriOppositeVertId;  // ���������ζ�Ӧ�������嶥������
    vector<unsigned int> m_outsideTetVertIds;         // �ڱ���Ķ����������� [v0, v2, v4, v6, v8...]
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

    void Init(const vector<float> tetVertPos, const vector<int>& tetIdx, const vector<unsigned int>& tetFaceIdx, vector<unsigned int> tetFaceOppositeTetVertIdx);
    void Step();
    void StepForConvergence();
    void InitVolumeConstraint();
    void SetFixedVert();
    void RenderOnce();
    void DCDByPoint();
    void DCDByTriangle();
};