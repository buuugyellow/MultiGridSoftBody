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

void Simulator::SetObjConfig() {
    string name = config_objName;
    string objFile = config_dataDir + name + ".obj";
    string tetFile = config_dataDir + name;
    m_softObject = new SoftObject(name, objFile, tetFile);
}

void Simulator::ReadObjMshFiles() { m_softObject->ReadFromFile(); }

void Simulator::CopyObjToSimulator() {
    m_tetVertPos = m_softObject->m_tetVertPosORIG;
    m_tetIdx = m_softObject->m_tetIdxORIG;

    // test
    for (int i = 0; i < m_tetVertPos.size() / 3; i++) {
        m_normal.push_back(1.0f);
        m_normal.push_back(0.0f);
        m_normal.push_back(0.0f);
    }
}

void Simulator::TetFaceExtraction() {
    struct FaceKey {
        FaceKey(int a, int b, int c) {
            k[0] = a;
            k[1] = b;
            k[2] = c;
            f[0] = a;
            f[1] = b;
            f[2] = c;
            Sort();
        }
        int k[3];
        int f[3];
        bool operator==(const FaceKey& rhs) const { return k[0] == rhs.k[0] && k[1] == rhs.k[1] && k[2] == rhs.k[2]; }

        bool operator<(const FaceKey& rhs) const {
            if (rhs.k[0] != k[0])
                return rhs.k[0] < k[0];
            else if (rhs.k[1] != k[1])
                return rhs.k[1] < k[1];
            else
                return rhs.k[2] < k[2];
        }
        void Sort() {
            if (k[0] > k[1]) {
                int temp = k[0];
                k[0] = k[1];
                k[1] = temp;
            }
            if (k[1] > k[2]) {
                int temp = k[1];
                k[1] = k[2];
                k[2] = temp;
            }
            if (k[0] > k[1]) {
                int temp = k[0];
                k[0] = k[1];
                k[1] = temp;
            }
        }
    };

    set<FaceKey> outsideFace;
    set<FaceKey> insideFace;

    // todo: fill m_tetFaceIdx
    for (int i = 0; i < m_tetIdx.size() / 4; i++) {
        int vId0 = m_tetIdx[i * 4 + 0];
        int vId1 = m_tetIdx[i * 4 + 1];
        int vId2 = m_tetIdx[i * 4 + 2];
        int vId3 = m_tetIdx[i * 4 + 3];

        // 右手系
        FaceKey f0 = FaceKey(vId0, vId2, vId1);
        FaceKey f1 = FaceKey(vId0, vId1, vId3);
        FaceKey f2 = FaceKey(vId0, vId3, vId2);
        FaceKey f3 = FaceKey(vId1, vId2, vId3);

        FaceKey fs[4] = {f0, f1, f2, f3};

        // 1. 按索引从小到大生成4个面
        // 2. 如果表面集合中有这个面就移除，放进内部集合
        // 3. 如果表面集合中没有这个面，再看内部集合有没有这个面，如果内部没有，放进表面集合，内部有 skip
        for (int j = 0; j < 4; j++) {
            std::set<FaceKey>::iterator it;
            auto f = fs[j];
            it = outsideFace.find(f);
            if (it != outsideFace.end()) {
                outsideFace.erase(f);
                insideFace.insert(f);
            } else {
                it = insideFace.find(f);
                if (it == insideFace.end()) {
                    outsideFace.insert(f);
                }
            }
        }

        for (auto& f : outsideFace) {
            // 这里是用 MyFace 的原始顶点索引，而不是排序后的顶点索引
            int verIdx0 = f.f[0];
            int verIdx1 = f.f[1];
            int verIdx2 = f.f[2];
            m_tetFaceIdx.push_back(verIdx0);
            m_tetFaceIdx.push_back(verIdx1);
            m_tetFaceIdx.push_back(verIdx2);
        }
    }

    // 输出obj查看
    std::vector<unsigned int> idx;
    idx.assign(m_tetFaceIdx.begin(), m_tetFaceIdx.end());
    OutputPosNormIndex("../temp/outsideFace.obj", m_tetVertPos, std::vector<float>(), idx);
}

void Simulator::Init() {
    // 模型初始化
    SetObjConfig();
    ReadObjMshFiles();
    CopyObjToSimulator();
    TetFaceExtraction();

    // 解算器初始化
    m_solver = new PDSolver();
    m_solver->Init();
}

void Simulator::Update() { 
    //static int cnt = 0;
    //if (++cnt > 301) return;
    //cout << "step frame " << cnt << endl;
    m_solver->Step();
}