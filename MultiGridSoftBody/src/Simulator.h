#pragma once
#include <vector>

#include "SoftObject.h"
#include "Solver.h"

// ����ʽȫ�ֵ���������ģ�ͺͽ�����
using namespace std;
class Simulator {
public:
    SoftObject* m_softObject;
    vector<float> m_tetVertPos;
    vector<int> m_tetIdx;
    vector<int> m_fixed; 

    vector<unsigned int> m_tetFaceIdx;
    vector<float> m_normal; // ���������嶥�㶼���䷨�������������Ҫ����������ײ���

    Solver* m_solver;

    static Simulator& GetInstance();  // ��ȡ��������
    void Init();                      // ��ʼ��
    void Update();                    // ����
    void SetObjConfig();
    void ReadObjMshFiles();
    void CopyObjToSimulator();  // �� obj �ļ����ݸ��Ƶ���������
    void TetFaceExtraction();   // ��ȡ�����������

private:
    Simulator();
    ~Simulator();
    Simulator(const Simulator&) = delete;             // ��ֹ��������
    Simulator& operator=(const Simulator&) = delete;  // ��ֹ��ֵ����
};