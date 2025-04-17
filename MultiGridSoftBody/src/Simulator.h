#pragma once
#include <vector>

#include "SoftObject.h"
#include "Solver.h"

// ����ʽȫ�ֵ���������ģ�ͺͽ�����
using namespace std;
class Simulator {
public:
    SoftObject* m_softObject;
    Solver* m_solver;

    vector<float> m_tetVertPos;         // ������Ҫ����һ�ݶ������������������������
    vector<unsigned int> m_tetFaceIdx;  // ���ڿ�����Ⱦ������������
    vector<float> m_normal;             // ���������嶥�㶼���䷨�������������Ҫ����������ײ���

    static Simulator& GetInstance();  // ��ȡ��������
    void Init();                      // ��ʼ��
    void Update();                    // ����

private:
    Simulator();
    ~Simulator();
    Simulator(const Simulator&) = delete;             // ��ֹ��������
    Simulator& operator=(const Simulator&) = delete;  // ��ֹ��ֵ����
};