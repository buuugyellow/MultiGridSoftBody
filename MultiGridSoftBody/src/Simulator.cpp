#pragma once
#include "PDSolver.h"
#include "global.h"

#include <iostream>
#include <set>
#include <string>

using namespace std;

int g_stepCnt = 0;

Simulator& Simulator::GetInstance() {
    static Simulator instance;
    return instance;
}

void Simulator::Init() {
    // 模型初始化
    string name = config_objName;
    string objFile = config_dataDir + name + ".obj";
    string tetFile = config_dataDir + name;
    m_softObject = new SoftObject(name, objFile, tetFile);
    m_softObject->ReadFromFile();
    LOG(INFO) << "ReadFromFile 结束";
    m_softObject->TetFaceExtraction();
    LOG(INFO) << "TetFaceExtraction 结束";

    // 粗网格初始化
    string name_coarse = config_objName_coarse;
    string objFile_coarse = config_dataDir + name_coarse + ".obj";
    string tetFile_coarse = config_dataDir + name_coarse;
    m_softObject_coarse = new SoftObject(name_coarse, objFile_coarse, tetFile_coarse);
    m_softObject_coarse->ReadFromFile();
    LOG(INFO) << "coarse ReadFromFile 结束";
    m_softObject_coarse->TetFaceExtraction();
    LOG(INFO) << "coarse TetFaceExtraction 结束";

    // 中间变量初始化
    m_tetVertPos = m_softObject->m_tetVertPosORIG;
    m_tetIdx = m_softObject->m_tetIdxORIG;
    m_tetFaceIdx = m_softObject->m_tetFaceIdx;
    for (int i = 0; i < m_tetVertPos.size() / 3; i++) {
        m_normal.push_back(1.0f);
        m_normal.push_back(0.0f);
        m_normal.push_back(0.0f);
    }
    m_tetVertEp.resize(m_tetVertPos.size() / 3);

    // 解算器初始化
    switch (g_solverType) {
        case PD:
            m_solver = new PDSolver();
            m_solver->Init(m_tetIdx, m_tetVertPos);
            break;
        case PD_MG:
            m_solver_mg = new PDSolver_MG();
            m_solver_mg->Init(m_softObject_coarse->m_tetIdxORIG, m_softObject_coarse->m_tetVertPosORIG, m_softObject->m_tetIdxORIG,
                              m_softObject->m_tetVertPosORIG);
            break;
    }
    LOG(INFO) << "solver: " << g_solverType << " Init 结束";
}

void Simulator::Update() {
    if (g_stepCnt > 240) {
        if (timeOutputFile) {
            fclose(timeOutputFile);
            timeOutputFile = nullptr;
        }
        if (energyOutputFile) {
            fclose(energyOutputFile);
            energyOutputFile = nullptr;
        }
        renderOnce();
        return;
    }
    g_stepCnt++;
    cout << "step frame " << g_stepCnt << endl;

    switch (g_solverType) {
        case PD:
            m_solver->Step();
            break;
        case PD_MG:
            m_solver_mg->Step();
            break;
    }
}