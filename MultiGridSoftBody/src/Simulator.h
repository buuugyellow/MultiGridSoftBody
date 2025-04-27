#pragma once
#include "PDSolver.h"
#include "PDSolver_MG.h"
#include "SoftObject.h"

#include <vector>

// ����ʽȫ�ֵ���������ģ�ͺͽ�����
using namespace std;
class Simulator {
public:
    

    SoftObject* m_softObject;
    SoftObject* m_softObject_coarse;
    PDSolver* m_solver;
    PDSolver_MG* m_solver_mg;

    vector<float> m_tetVertPos;         // ������Ҫ����һ�ݶ������������������������
    vector<int> m_tetIdx;               // ͬ��
    vector<unsigned int> m_tetFaceIdx;  // ���ڿ�����Ⱦ������������
    vector<float> m_normal;             // ���������嶥�㶼���䷨�������������Ҫ����������ײ���

    static Simulator& GetInstance();  // ��ȡ��������
    void Init();                      // ��ʼ��
    void Update();                    // ����

private:
    Simulator() = default;
    ~Simulator() = default;
    Simulator(const Simulator&) = delete;             // ��ֹ��������
    Simulator& operator=(const Simulator&) = delete;  // ��ֹ��ֵ����
};