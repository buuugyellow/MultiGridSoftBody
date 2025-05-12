#pragma once
#include <vector>
#include <memory>

#include "Collider.h"
#include "PDSolver.h"
#include "PDSolver_MG.h"
#include "SoftObject.h"

// 懒汉式全局单例，包括模型和解算器
using namespace std;
class Simulator {
public:
    SoftObject* m_softObject;
    SoftObject* m_softObject_coarse;
    PDSolver* m_solver;
    PDSolver_MG* m_solver_mg;

    vector<float> m_tetVertPos;         // 这里需要复制一份顶点数据是用于与解算器解耦
    vector<unsigned int> m_tetFaceIdx;  // 用于控制渲染的三角形索引
    vector<float> m_normal;             // 所有四面体顶点都分配法向量，如果有需要可以用于碰撞检测

    // 碰撞检测
    vector<shared_ptr<SphereCollider>> m_sphereColliders;
    vector<shared_ptr<CapsuleCollider>> m_capsuleColliders;

    // 可视化
    vector<float> m_tetVertEpDensity;  // 四面体顶点的能量密度
    vector<float> m_tetVertEpSum;      // 所有顶点相关联的四面体的能量的总和
    vector<float> m_tetVertVSum;       // 所有顶点相关联的四面体的体积的总和

    static Simulator& GetInstance();  // 获取单例对象
    void Init();                      // 初始化
    void Update();                    // 更新
    void UpdateCollider();            // 更新碰撞体

private:
    Simulator() = default;
    ~Simulator() = default;
    Simulator(const Simulator&) = delete;             // 禁止拷贝构造
    Simulator& operator=(const Simulator&) = delete;  // 禁止赋值操作
};