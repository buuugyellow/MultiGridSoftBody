#include "PDSolver_MG.h"

void PDSolver_MG::Init(vector<int>& tetIdxCoarse, vector<float> tetVertPosCoarse, vector<int>& tetIdxFine, vector<float> tetVertPosFine) {
    m_pdSolverCoarse->Init(tetIdxCoarse, tetVertPosCoarse);
    m_pdSolverFine->Init(tetIdxFine, tetVertPosFine);
    // ������ϸ����֮���ӳ���ϵ
}

void PDSolver_MG::Step() {

}