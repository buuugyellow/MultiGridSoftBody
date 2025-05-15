#pragma once
#include "Simulator.h"
#include "PDSolver.h"
#include "global.h"
#include "glfw/include/GLFW/glfw3.h"

#include <iostream>
#include <set>
#include <string>
#include <thread>

using namespace std;

int g_stepCnt = 0;
double g_realDuration;
double g_totalDuration;
int g_collidedVertCnt;

Simulator& Simulator::GetInstance() {
    static Simulator instance;
    return instance;
}

void Simulator::Init() {
    // 模型初始化
    string name = config_objName;
    string objFile = config_dataDir + name + ".obj";
    string tetFile = config_dataDir + name;
    string tetFaceFile = config_tempDir + name + ".obj";
    m_softObject = new SoftObject(name, objFile, tetFile, tetFaceFile);
    m_softObject->ReadFromFile();
    LOG(INFO) << "ReadFromFile 结束";
    m_softObject->TetFaceExtraction();
    LOG(INFO) << "TetFaceExtraction 结束";

    // 粗网格初始化
    string name_coarse = config_objName_coarse;
    string objFile_coarse = config_dataDir + name_coarse + ".obj";
    string tetFile_coarse = config_dataDir + name_coarse;
    string tetFaceFile_coarse = config_tempDir + name_coarse + ".obj";
    m_softObject_coarse = new SoftObject(name_coarse, objFile_coarse, tetFile_coarse, tetFaceFile_coarse);
    m_softObject_coarse->ReadFromFile();
    LOG(INFO) << "coarse ReadFromFile 结束";
    m_softObject_coarse->TetFaceExtraction();
    LOG(INFO) << "coarse TetFaceExtraction 结束";

    // 碰撞体初始化
    m_sphereColliders.push_back(shared_ptr<SphereCollider>(new SphereCollider({-5, 0, 0}, 1)));

    // 中间变量初始化
    m_tetVertPos = m_softObject->m_tetVertPosORIG;
    m_tetFaceIdx = m_softObject->m_tetFaceIdx;
    for (int i = 0; i < m_tetVertPos.size() / 3; i++) {
        m_normal.push_back(1.0f);
        m_normal.push_back(0.0f);
        m_normal.push_back(0.0f);
    }
    m_tetVertEpDensity.resize(m_tetVertPos.size() / 3);
    m_tetVertEpSum.resize(m_tetVertPos.size() / 3);
    m_tetVertVSum.resize(m_tetVertPos.size() / 3);
    m_tetVertIsCollide.resize(m_tetVertPos.size() / 3);
    m_triIsCollide.resize(m_tetFaceIdx.size() / 3);
    m_triMoveVec.resize(m_tetFaceIdx.size());

    // 解算器初始化
    switch (g_solverType) {
        case PD:
            m_solver = new PDSolver();
            m_solver->Init(m_softObject->m_tetVertPosORIG, m_softObject->m_tetIdxORIG, m_softObject->m_tetFaceIdx);
            break;
        case PD_MG:
            m_solver_mg = new PDSolver_MG();
            m_solver_mg->Init(m_softObject_coarse->m_tetVertPosORIG, m_softObject_coarse->m_tetIdxORIG, m_softObject_coarse->m_tetFaceIdx,
                              m_softObject->m_tetVertPosORIG, m_softObject->m_tetIdxORIG, m_softObject->m_tetFaceIdx);
            break;
    }
    LOG(INFO) << "solver: " << g_solverType << " Init 结束";
}

void Simulator::UpdateCollider() {
    for (auto sphere : m_sphereColliders) {
        sphere->Update(Point3D(0.1f, 0, 0));
    }
    return;

    float delta = 0.05f;
    if (!m_sphereColliders.empty()) {
        auto sphere = m_sphereColliders[0];
        switch (g_key) {
            case GLFW_KEY_W:
                sphere->Update(Point3D(0, 0, -delta));
                break;
            case GLFW_KEY_S:
                sphere->Update(Point3D(0, 0, delta));
                break;
            case GLFW_KEY_A:
                sphere->Update(Point3D(-delta, 0, 0));
                break;
            case GLFW_KEY_D:
                sphere->Update(Point3D(delta, 0, 0));
                break;
            case GLFW_KEY_Q:
                sphere->Update(Point3D(0, delta, 0));
                break;
            case GLFW_KEY_Z:
                sphere->Update(Point3D(0, -delta, 0));
                break;
        }
    }
    g_key = 0;
}

void Simulator::Update() {
    static auto last_time = chrono::high_resolution_clock::now();
    auto begin_time = chrono::high_resolution_clock::now();
    g_totalDuration = (chrono::duration_cast<chrono::microseconds>(begin_time - last_time)).count();
    last_time = begin_time;

    if (g_stepCnt > 80) {
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

    UpdateCollider();
    switch (g_solverType) {
        case PD:
            m_solver->Step();
            break;
        case PD_MG:
            m_solver_mg->Step();
            break;
    }

    auto end_time = chrono::high_resolution_clock::now();
    g_realDuration = (chrono::duration_cast<chrono::microseconds>(end_time - begin_time)).count();
    auto target_time = begin_time + chrono::microseconds(33333);
    while (chrono::high_resolution_clock::now() < target_time) {}
}