#pragma once
#include <vector>

#include "Solver.h"
using namespace std;
class PDSolver : public Solver {
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

    void Init();
    void Step();
    void InitVolumeConstraint();
    void SetFixedVert();
};