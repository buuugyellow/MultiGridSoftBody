#include "PDSolver_MG.h"
#include "global.h"

void PDSolver_MG::Init(const vector<int>& tetIdxCoarse, const vector<float> tetVertPosCoarse, const vector<int>& tetIdxFine,
                       const vector<float> tetVertPosFine) {
    m_pdSolverCoarse = new PDSolver();
    m_pdSolverFine = new PDSolver();
    m_pdSolverCoarse->Init(tetIdxCoarse, tetVertPosCoarse);
    m_pdSolverFine->Init(tetIdxFine, tetVertPosFine);
    // ������ϸ����֮���ӳ���ϵ
    m_interpolationIds.resize(m_pdSolverFine->m_tetVertNum * 4);
    m_interpolationWights.resize(m_pdSolverFine->m_tetVertNum * 4);
    // ����һ������
    // �������������ꡢ������������ⷨ�ߡ����
    int tetNumCoarse = m_pdSolverCoarse->m_tetNum;
    const vector<float>& tetVolumeCoarse = m_pdSolverCoarse->m_tetVolume;
    const vector<float>& tetCenterCoarse = m_pdSolverCoarse->m_tetCenter;
    const vector<float>& tetFaceNormalCoarse = m_pdSolverCoarse->m_tetFaceNormal;
    const vector<float>& tetFaceAreaCoarse = m_pdSolverCoarse->m_tetFaceArea;

    int tetVertNumFine = m_pdSolverFine->m_tetVertNum;
    
    float maxDistance = 0; // ϸ�����嶥��ʹ�����������֮��ľ������ֵ��������֤
    for (int vIdFine = 0; vIdFine < tetVertNumFine; ++vIdFine) {
        for (int tIdCoarse = 0; tIdCoarse < tetNumCoarse; ++tIdCoarse) {
            vector<float> tetVertPosInATet;
            vector<int> tetVertIdInATet = {tetIdxCoarse[tIdCoarse * 4 + 0], tetIdxCoarse[tIdCoarse * 4 + 1], tetIdxCoarse[tIdCoarse * 4 + 2],
                                           tetIdxCoarse[tIdCoarse * 4 + 3]};
            for (int ii = 0; ii < 4; ii++) {
                int id = tetVertIdInATet[ii];
                tetVertPosInATet.push_back(tetVertPosCoarse[id * 3 + 0]);
                tetVertPosInATet.push_back(tetVertPosCoarse[id * 3 + 1]);
                tetVertPosInATet.push_back(tetVertPosCoarse[id * 3 + 2]);
            }
            // https://www.iue.tuwien.ac.at/phd/nentchev/node31.html
            if (pointInTet(tetVertPosInATet.data(), tetFaceNormalCoarse.data() + tIdCoarse * 12, tetVertPosFine.data() + vIdFine * 3)) {
                vector<float> lambdas =
                    barycentricCoordinate(tetVertPosFine.data() + vIdFine * 3, tetCenterCoarse.data() + tIdCoarse * 3, tetFaceAreaCoarse.data() + tIdCoarse * 4,
                                          tetFaceNormalCoarse.data() + tIdCoarse * 12, tetVolumeCoarse[tIdCoarse]);
                for (int ii = 0; ii < 4; ++ii) {
                    m_interpolationIds[vIdFine * 4 + ii] = tetVertIdInATet[ii];
                    m_interpolationWights[vIdFine * 4 + ii] = lambdas[ii];
                }

                // test
                for (int ii = 0; ii < 4; ii++) {
                    float l = lambdas[ii];
                    assert(l>-1e-5 && l < 1 + 1e-5);
                }
                Point3D p = {tetVertPosFine[vIdFine * 3 + 0], tetVertPosFine[vIdFine * 3 + 1], tetVertPosFine[vIdFine * 3 + 2]};
                Point3D center = {tetCenterCoarse[tIdCoarse * 3 + 0], tetCenterCoarse[tIdCoarse * 3 + 1], tetCenterCoarse[tIdCoarse * 3 + 2]};
                Point3D centerP = p - center;
                float dis = vectorLength(centerP);
                maxDistance = max(maxDistance, dis);
                break; // ���ü�������������������
            }
        }
    }
    LOG(INFO) << "�󶨵�ϸ�����嶥���������������֮��������� = " << maxDistance;
}

void PDSolver_MG::Step() {

}