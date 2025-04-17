#pragma once
#include "global.h"
#include "PDSolver.h"

#include <iostream>
#include <set>
#include <string>

using namespace std;

Simulator& Simulator::GetInstance() {
    static Simulator instance;
    return instance;
}

Simulator::Simulator() {}

Simulator::~Simulator() {}

void Simulator::Init() {
    // ģ�ͳ�ʼ��
    string name = config_objName;
    string objFile = config_dataDir + name + ".obj";
    string tetFile = config_dataDir + name;
    m_softObject = new SoftObject(name, objFile, tetFile);

    m_softObject->ReadFromFile();
    LOG(INFO) << "ReadFromFile ����";

    m_softObject->TetFaceExtraction();
    LOG(INFO) << "TetFaceExtraction ����";

    LOG(INFO) << "objects Init ����";

    // ��������ʼ��
    m_solver = new PDSolver();
    m_solver->Init();
    LOG(INFO) << "solver Init ����";

    m_tetVertPos = m_softObject->m_tetVertPosORIG;
    m_tetFaceIdx = m_softObject->m_tetFaceIdx;
    for (int i = 0; i < m_tetVertPos.size() / 3; i++) {
        m_normal.push_back(1.0f);
        m_normal.push_back(0.0f);
        m_normal.push_back(0.0f);
    }
}

void Simulator::Update() { 
    static int cnt = 0;
    if (++cnt > 301) return;
    cout << "step frame " << cnt << endl;
    m_solver->Step();
}