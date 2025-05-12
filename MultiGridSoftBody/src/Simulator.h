#pragma once
#include <vector>
#include <memory>

#include "Collider.h"
#include "PDSolver.h"
#include "PDSolver_MG.h"
#include "SoftObject.h"

// ����ʽȫ�ֵ���������ģ�ͺͽ�����
using namespace std;
class Simulator {
public:
    SoftObject* m_softObject;
    SoftObject* m_softObject_coarse;
    PDSolver* m_solver;
    PDSolver_MG* m_solver_mg;

    vector<float> m_tetVertPos;         // ������Ҫ����һ�ݶ������������������������
    vector<unsigned int> m_tetFaceIdx;  // ���ڿ�����Ⱦ������������
    vector<float> m_normal;             // ���������嶥�㶼���䷨�������������Ҫ����������ײ���

    // ��ײ���
    vector<shared_ptr<SphereCollider>> m_sphereColliders;
    vector<shared_ptr<CapsuleCollider>> m_capsuleColliders;

    // ���ӻ�
    vector<float> m_tetVertEpDensity;  // �����嶥��������ܶ�
    vector<float> m_tetVertEpSum;      // ���ж������������������������ܺ�
    vector<float> m_tetVertVSum;       // ���ж�����������������������ܺ�

    static Simulator& GetInstance();  // ��ȡ��������
    void Init();                      // ��ʼ��
    void Update();                    // ����
    void UpdateCollider();            // ������ײ��

private:
    Simulator() = default;
    ~Simulator() = default;
    Simulator(const Simulator&) = delete;             // ��ֹ��������
    Simulator& operator=(const Simulator&) = delete;  // ��ֹ��ֵ����
};