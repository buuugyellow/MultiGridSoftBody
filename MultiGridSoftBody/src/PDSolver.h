#pragma once
#include "Solver.h"
#include <vector>
using namespace std;
class PDSolver : public Solver {
public:
    int m_iterNum = 16;
    float m_dt = 1.0f / 30.0f;
    float m_damping = 0.5f;
    float m_volumnStiffness = 1000.0f;
    float m_rho = 0.9992f;
    float m_gravityX = 0.0f;
    float m_gravityY = -9.8f;
    float m_gravityZ = 0.0f;

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

    void Init();
    void Step();
    void InitVolumeConstraint();
    void SetFixedVert();
};