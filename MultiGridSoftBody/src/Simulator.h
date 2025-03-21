#pragma once
#include <vector>

#include "SoftObject.h"
#include "Solver.h"

// 懒汉式全局单例，包括模型和解算器
using namespace std;
class Simulator {
public:
    SoftObject* m_softObject;
    vector<float> m_tetVertPos;
    vector<int> m_tetIdx;
    vector<int> m_fixed; 

    vector<unsigned int> m_tetFaceIdx;
    vector<float> m_normal; // 所有四面体顶点都分配法向量，如果有需要可以用于碰撞检测

    Solver* m_solver;

    static Simulator& GetInstance();  // 获取单例对象
    void Init();                      // 初始化
    void Update();                    // 更新
    void SetObjConfig();
    void ReadObjMshFiles();
    void CopyObjToSimulator();  // 将 obj 文件数据复制到仿真器中
    void TetFaceExtraction();   // 提取四面体外表面

private:
    Simulator();
    ~Simulator();
    Simulator(const Simulator&) = delete;             // 禁止拷贝构造
    Simulator& operator=(const Simulator&) = delete;  // 禁止赋值操作
};