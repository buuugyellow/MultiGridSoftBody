#pragma once
#include <vector>

#include "SoftObject.h"
#include "Solver.h"

// 懒汉式全局单例，包括模型和解算器
using namespace std;
class Simulator {
public:
    SoftObject* m_softObject;
    Solver* m_solver;

    vector<float> m_tetVertPos;         // 这里需要复制一份顶点数据是用于与解算器解耦
    vector<unsigned int> m_tetFaceIdx;  // 用于控制渲染的三角形索引
    vector<float> m_normal;             // 所有四面体顶点都分配法向量，如果有需要可以用于碰撞检测

    static Simulator& GetInstance();  // 获取单例对象
    void Init();                      // 初始化
    void Update();                    // 更新

private:
    Simulator();
    ~Simulator();
    Simulator(const Simulator&) = delete;             // 禁止拷贝构造
    Simulator& operator=(const Simulator&) = delete;  // 禁止赋值操作
};