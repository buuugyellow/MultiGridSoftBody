#include "PDSolver_MG.h"

#include "global.h"

void PDSolver_MG::interpolate() {
    static int cnt = 0;
    LOG(INFO) << "interpolate cnt:" << ++cnt;
    // https://www.iue.tuwien.ac.at/phd/nentchev/node31.html
    m_interpolationIds.resize(m_pdSolverFine->m_tetVertNum * 4, -1);
    m_interpolationWights.resize(m_pdSolverFine->m_tetVertNum * 4, 0);
    int tetVertNumFine = m_pdSolverFine->m_tetVertNum;
    int tetVertNumCoarse = m_pdSolverCoarse->m_tetVertNum;
    int tetNumCoarse = m_pdSolverCoarse->m_tetNum;
    const vector<int>& tetIdxCoarse = m_pdSolverCoarse->m_tetIndex;
    vector<float> tetVertPosCoarse(tetVertNumCoarse * 3);
    vector<float> tetVertPosFine(tetVertNumFine * 3);
    cudaMemcpy(tetVertPosCoarse.data(), m_pdSolverCoarse->pdSolverData->tetVertPos_d, tetVertNumCoarse * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(tetVertPosFine.data(), m_pdSolverFine->pdSolverData->tetVertPos_d, tetVertNumFine * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    float maxDistance = 0;  // 细四面体顶点和粗四面体重心之间的距离最大值，用于验证
    for (int vIdFine = 0; vIdFine < tetVertNumFine; ++vIdFine) {
        bool found = false;
        Point3D P = {tetVertPosFine[vIdFine * 3 + 0], tetVertPosFine[vIdFine * 3 + 1], tetVertPosFine[vIdFine * 3 + 2]};
        for (int tIdCoarse = 0; tIdCoarse < tetNumCoarse; ++tIdCoarse) {
            int tetVertId0 = tetIdxCoarse[tIdCoarse * 4 + 0];
            int tetVertId1 = tetIdxCoarse[tIdCoarse * 4 + 1];
            int tetVertId2 = tetIdxCoarse[tIdCoarse * 4 + 2];
            int tetVertId3 = tetIdxCoarse[tIdCoarse * 4 + 3];
            Point3D A = {tetVertPosCoarse[tetVertId0 * 3 + 0], tetVertPosCoarse[tetVertId0 * 3 + 1], tetVertPosCoarse[tetVertId0 * 3 + 2]};
            Point3D B = {tetVertPosCoarse[tetVertId1 * 3 + 0], tetVertPosCoarse[tetVertId1 * 3 + 1], tetVertPosCoarse[tetVertId1 * 3 + 2]};
            Point3D C = {tetVertPosCoarse[tetVertId2 * 3 + 0], tetVertPosCoarse[tetVertId2 * 3 + 1], tetVertPosCoarse[tetVertId2 * 3 + 2]};
            Point3D D = {tetVertPosCoarse[tetVertId3 * 3 + 0], tetVertPosCoarse[tetVertId3 * 3 + 1], tetVertPosCoarse[tetVertId3 * 3 + 2]};
            float weights[4] = {0.25f};
            barycentricCoordinate(P, A, B, C, D, weights);

            bool PinABCD = true;
            for (int i = 0; i < 4; i++) {
                if (weights[i] < -1e-5 || weights[i] > 1 + 1e-5) {
                    PinABCD = false;
                }
            }
            if (PinABCD) {
                for (int i = 0; i < 4; ++i) {
                    m_interpolationIds[vIdFine * 4 + i] = tetIdxCoarse[tIdCoarse * 4 + i];
                    m_interpolationWights[vIdFine * 4 + i] = weights[i];
                }
                Point3D center = (A + B + C + D) / 4.0f;
                Point3D centerP = P - center;
                float dis = vectorLength(centerP);
                maxDistance = max(maxDistance, dis);
                found = true;
                break;
            }
        }
        if (!found) LOG(ERROR) << "vIdFine:" << vIdFine << " 没有匹配到粗四面体\n";
    }
    LOG(INFO) << "绑定的细四面体顶点与粗四面体重心之间的最大距离 = " << maxDistance;
}

void PDSolver_MG::Init(const vector<int>& tetIdxCoarse, const vector<float> tetVertPosCoarse, const vector<int>& tetIdxFine,
                       const vector<float> tetVertPosFine) {
    m_pdSolverCoarse = new PDSolver();
    m_pdSolverFine = new PDSolver();
    m_pdSolverCoarse->Init(tetIdxCoarse, tetVertPosCoarse);
    m_pdSolverFine->Init(tetIdxFine, tetVertPosFine);
    // 建立粗细网格之间的映射关系
    m_interpolationIds.resize(m_pdSolverFine->m_tetVertNum * 4);
    m_interpolationWights.resize(m_pdSolverFine->m_tetVertNum * 4);
    m_averageIds.resize(m_pdSolverCoarse->m_tetVertNum * 4);
    m_averageWeights.resize(m_pdSolverCoarse->m_tetVertNum * 4);
    // 方法一：暴力
    // 四面体重心坐标、体积、三角形外法线、面积
    int tetNumCoarse = m_pdSolverCoarse->m_tetNum;
    const vector<float>& tetVolumeCoarse = m_pdSolverCoarse->m_tetVolume;
    const vector<float>& tetCenterCoarse = m_pdSolverCoarse->m_tetCenter;
    const vector<float>& tetFaceNormalCoarse = m_pdSolverCoarse->m_tetFaceNormal;
    const vector<float>& tetFaceAreaCoarse = m_pdSolverCoarse->m_tetFaceArea;

    int tetVertNumFine = m_pdSolverFine->m_tetVertNum;

    float maxDistance = 0;  // 细四面体顶点和粗四面体重心之间的距离最大值，用于验证

    interpolate();

    //for (int vIdFine = 0; vIdFine < tetVertNumFine; ++vIdFine) {
    //    bool found = false;
    //    for (int tIdCoarse = 0; tIdCoarse < tetNumCoarse; ++tIdCoarse) {
    //        vector<float> tetVertPosInATet;
    //        vector<int> tetVertIdInATet = {tetIdxCoarse[tIdCoarse * 4 + 0], tetIdxCoarse[tIdCoarse * 4 + 1], tetIdxCoarse[tIdCoarse * 4 + 2],
    //                                       tetIdxCoarse[tIdCoarse * 4 + 3]};
    //        for (int ii = 0; ii < 4; ii++) {
    //            int id = tetVertIdInATet[ii];
    //            tetVertPosInATet.push_back(tetVertPosCoarse[id * 3 + 0]);
    //            tetVertPosInATet.push_back(tetVertPosCoarse[id * 3 + 1]);
    //            tetVertPosInATet.push_back(tetVertPosCoarse[id * 3 + 2]);
    //        }
    //        // https://www.iue.tuwien.ac.at/phd/nentchev/node31.html
    //        if (pointInTet(tetVertPosInATet.data(), tetFaceNormalCoarse.data() + tIdCoarse * 12, tetVertPosFine.data() + vIdFine * 3)) {
    //            vector<float> lambdas =
    //                barycentricCoordinate(tetVertPosFine.data() + vIdFine * 3, tetCenterCoarse.data() + tIdCoarse * 3, tetFaceAreaCoarse.data() + tIdCoarse * 4,
    //                                      tetFaceNormalCoarse.data() + tIdCoarse * 12, tetVolumeCoarse[tIdCoarse]);
    //            for (int ii = 0; ii < 4; ++ii) {
    //                m_interpolationIds[vIdFine * 4 + ii] = tetVertIdInATet[ii];
    //                m_interpolationWights[vIdFine * 4 + ii] = lambdas[ii];
    //            }

    //            // test
    //            for (int ii = 0; ii < 4; ii++) {
    //                float l = lambdas[ii];
    //                assert(l > -1e-5 && l < 1 + 1e-5);
    //            }
    //            Point3D p = {tetVertPosFine[vIdFine * 3 + 0], tetVertPosFine[vIdFine * 3 + 1], tetVertPosFine[vIdFine * 3 + 2]};
    //            Point3D center = {tetCenterCoarse[tIdCoarse * 3 + 0], tetCenterCoarse[tIdCoarse * 3 + 1], tetCenterCoarse[tIdCoarse * 3 + 2]};
    //            Point3D centerP = p - center;
    //            float dis = vectorLength(centerP);
    //            maxDistance = max(maxDistance, dis);
    //            found = true;
    //            break;  // 不用继续遍历其他的四面体
    //        }
    //    }
    //    assert(found == true);
    //}
    //LOG(INFO) << "绑定的细四面体顶点与粗四面体重心之间的最大距离 = " << maxDistance;

    maxDistance = 0;
    int tetVertNumCoarse = m_pdSolverCoarse->m_tetVertNum;
    int tetNumFine = m_pdSolverFine->m_tetNum;
    const vector<float>& tetVolumeFine = m_pdSolverFine->m_tetVolume;
    const vector<float>& tetCenterFine = m_pdSolverFine->m_tetCenter;
    const vector<float>& tetFaceNormalFine = m_pdSolverFine->m_tetFaceNormal;
    const vector<float>& tetFaceAreaFine = m_pdSolverFine->m_tetFaceArea;
    for (int vIdCoarse = 0; vIdCoarse < tetVertNumCoarse; ++vIdCoarse) {
        bool found = false;
        for (int tIdFine = 0; tIdFine < tetNumFine; ++tIdFine) {
            vector<float> tetVertPosInATet;
            vector<int> tetVertIdInATet = {tetIdxFine[tIdFine * 4 + 0], tetIdxFine[tIdFine * 4 + 1], tetIdxFine[tIdFine * 4 + 2], tetIdxFine[tIdFine * 4 + 3]};
            for (int ii = 0; ii < 4; ii++) {
                int id = tetVertIdInATet[ii];
                tetVertPosInATet.push_back(tetVertPosFine[id * 3 + 0]);
                tetVertPosInATet.push_back(tetVertPosFine[id * 3 + 1]);
                tetVertPosInATet.push_back(tetVertPosFine[id * 3 + 2]);
            }
            if (pointInTet(tetVertPosInATet.data(), tetFaceNormalFine.data() + tIdFine * 12, tetVertPosCoarse.data() + vIdCoarse * 3)) {
                vector<float> lambdas =
                    barycentricCoordinate(tetVertPosCoarse.data() + vIdCoarse * 3, tetCenterFine.data() + tIdFine * 3, tetFaceAreaFine.data() + tIdFine * 4,
                                          tetFaceNormalFine.data() + tIdFine * 12, tetVolumeFine[tIdFine]);
                for (int ii = 0; ii < 4; ++ii) {
                    m_averageIds[vIdCoarse * 4 + ii] = tetVertIdInATet[ii];
                    m_averageWeights[vIdCoarse * 4 + ii] = lambdas[ii];
                }

                // test
                for (int ii = 0; ii < 4; ii++) {
                    float l = lambdas[ii];
                    assert(l > -1e-5 && l < 1 + 1e-5);
                }
                Point3D p = {tetVertPosCoarse[vIdCoarse * 3 + 0], tetVertPosCoarse[vIdCoarse * 3 + 1], tetVertPosCoarse[vIdCoarse * 3 + 2]};
                Point3D center = {tetCenterFine[tIdFine * 3 + 0], tetCenterFine[tIdFine * 3 + 1], tetCenterFine[tIdFine * 3 + 2]};
                Point3D centerP = p - center;
                float dis = vectorLength(centerP);
                maxDistance = max(maxDistance, dis);
                found = true;
                break;  // 不用继续遍历其他的四面体
            }
        }
        assert(found == true);
    }
    LOG(INFO) << "绑定的粗四面体顶点与细四面体重心之间的最大距离 = " << maxDistance;

    // 分配 cuda 数组
    cudaMalloc((void**)&interpolationIds_d, tetVertNumFine * 4 * sizeof(int));
    cudaMemcpy(interpolationIds_d, m_interpolationIds.data(), tetVertNumFine * 4 * sizeof(int), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&interpolationWights_d, tetVertNumFine * 4 * sizeof(float));
    cudaMemcpy(interpolationWights_d, m_interpolationWights.data(), tetVertNumFine * 4 * sizeof(float), cudaMemcpyHostToDevice);

    cudaMalloc((void**)&averageIds_d, tetVertNumCoarse * 4 * sizeof(int));
    cudaMemcpy(averageIds_d, m_averageIds.data(), tetVertNumCoarse * 4 * sizeof(int), cudaMemcpyHostToDevice);
    cudaMalloc((void**)&averageWeights_d, tetVertNumCoarse * 4 * sizeof(float));
    cudaMemcpy(averageWeights_d, m_averageWeights.data(), tetVertNumCoarse * 4 * sizeof(float), cudaMemcpyHostToDevice);
}

void PDSolver_MG::Step() {
    static float E0 = 0;
    float Ek, Ep, dX, error;
    auto start = std::chrono::high_resolution_clock::now();
    m_pdSolverFine->StepForConvergence();

    m_pdSolverCoarse->pdSolverData->runCalculateST(m_pdSolverCoarse->m_damping, m_pdSolverCoarse->m_dt, m_pdSolverCoarse->m_gravityX,
                                                   m_pdSolverCoarse->m_gravityY, m_pdSolverCoarse->m_gravityZ);
    m_pdSolverFine->pdSolverData->runCalculateST(m_pdSolverFine->m_damping, m_pdSolverFine->m_dt, m_pdSolverFine->m_gravityX, m_pdSolverFine->m_gravityY,
                                                 m_pdSolverFine->m_gravityZ);
    m_pdSolverFine->pdSolverData->runCalEnergy(m_pdSolverFine->m_dt, m_pdSolverFine->m_tetVertMass, m_pdSolverFine->m_tetIndex, m_pdSolverFine->m_tetInvD3x3,
                                               m_pdSolverFine->m_tetVolume, m_pdSolverFine->m_volumnStiffness, Ek, Ep, dX);
    E0 = Ep + Ek;
    if (g_stepCnt < 200) error = (Ek + Ep - g_conEnergy_V2) / (E0 - g_conEnergy_V2);
    fprintf(energyOutputFile, "%d,%f,%f,%f,%f,%f\n", 0, Ek + Ep, Ek, Ep, dX, error);

    // 粗网格迭代到收敛
    float omega = 1.0f;
    for (int i = 0; i < 32; i++) {
        m_pdSolverCoarse->pdSolverData->runClearTemp();
        m_pdSolverCoarse->pdSolverData->runCalculateIF(m_pdSolverCoarse->m_volumnStiffness);
        omega = 4 / (4 - m_pdSolverCoarse->m_rho * m_pdSolverCoarse->m_rho * omega);
        m_pdSolverCoarse->pdSolverData->runcalculatePOS(omega, m_pdSolverCoarse->m_dt);
    }

    // 插值到细网格
    runInterpolate();
    m_pdSolverFine->pdSolverData->runCalEnergy(m_pdSolverFine->m_dt, m_pdSolverFine->m_tetVertMass, m_pdSolverFine->m_tetIndex, m_pdSolverFine->m_tetInvD3x3,
                                               m_pdSolverFine->m_tetVolume, m_pdSolverFine->m_volumnStiffness, Ek, Ep, dX);
    if (g_stepCnt < 200) error = (Ek + Ep - g_conEnergy_V2) / (E0 - g_conEnergy_V2);
    fprintf(energyOutputFile, "%d,%f,%f,%f,%f,%f\n", 1, Ek + Ep, Ek, Ep, dX, error);

    // 细网格迭代到收敛
    omega = 1.0f;
    for (int i = 0; i < 32; i++) {
        m_pdSolverFine->pdSolverData->runClearTemp();
        m_pdSolverFine->pdSolverData->runCalculateIF(m_pdSolverFine->m_volumnStiffness);
        omega = 4 / (4 - m_pdSolverFine->m_rho * m_pdSolverFine->m_rho * omega);
        m_pdSolverFine->pdSolverData->runcalculatePOS(omega, m_pdSolverFine->m_dt);

        m_pdSolverFine->pdSolverData->runCalEnergy(m_pdSolverFine->m_dt, m_pdSolverFine->m_tetVertMass, m_pdSolverFine->m_tetIndex,
                                                   m_pdSolverFine->m_tetInvD3x3, m_pdSolverFine->m_tetVolume, m_pdSolverFine->m_volumnStiffness, Ek, Ep, dX);
        if (g_stepCnt < 200) error = (Ek + Ep - g_conEnergy_V2) / (E0 - g_conEnergy_V2);
        fprintf(energyOutputFile, "%d,%f,%f,%f,%f,%f\n", i + 2, Ek + Ep, Ek, Ep, dX, error);
    }

    // 平均到粗网格
    runAverage();

    // 更新绑定的权重
    //runUpdateMapping(); // 会导致四面体反转
    //interpolate();
    //cudaMemcpy(interpolationIds_d, m_interpolationIds.data(), m_pdSolverFine->m_tetVertNum * 4 * sizeof(int), cudaMemcpyHostToDevice);
    //cudaMemcpy(interpolationWights_d, m_interpolationWights.data(), m_pdSolverFine->m_tetVertNum * 4 * sizeof(float), cudaMemcpyHostToDevice);

    m_pdSolverCoarse->pdSolverData->runCalculateV(m_pdSolverCoarse->m_dt);
    m_pdSolverFine->pdSolverData->runCalculateV(m_pdSolverFine->m_dt);

    m_pdSolverFine->pdSolverData->runCpyTetVertForRender();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    duration_physical = duration.count();
}