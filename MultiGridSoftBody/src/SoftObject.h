#pragma once
#include <string>
#include <vector>
using namespace std;

class SoftObject {
public:
    string m_name;
    string m_objFile;
    string m_tetFile;
    string m_fixFile;
    string m_tetFaceFile;
    int m_renderObjId;

    // ԭʼ obj �ļ����������
    vector<float> m_triVertPos;  // ��������
    vector<float> m_triUV;       // ��������
    vector<int> m_triIdx;        // �����ζ�������
    vector<int> m_triUVIdx;      // ������ UV ����

    // ԭʼ msh �ļ����������
    vector<float> m_tetVertPosORIG;  // �����嶥������
    vector<int> m_tetIdxORIG;        // �����嶥������

    // ����ԭʼ���ݵõ�������
    vector<unsigned int> m_tetFaceIdx;            // ���������������
    vector<unsigned int> m_tetFaceOppositeTetVertIdx;  // ���������ζ�Ӧ����һ�������嶥��������������������ײ��Ӧ

    SoftObject(string name, string objFile, string tetFile, string tetFaceFile)
        : m_name(name), m_objFile(objFile), m_tetFile(tetFile), m_tetFaceFile(tetFaceFile), m_renderObjId(-1) {};

    void ReadFromFile();
    void TetFaceExtraction();
};