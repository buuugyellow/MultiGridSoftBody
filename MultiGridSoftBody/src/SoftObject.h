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
    int m_renderObjId;

    // ԭʼ obj �ļ����������
    vector<float> m_triVertPos; // ��������
    vector<float> m_triUV; // ��������
    vector<int> m_triIdx; // �����ζ�������
    vector<int> m_triUVIdx; // ������ UV ����

    // ԭʼ msh �ļ����������
    vector<float> m_tetVertPosORIG; // �����嶥������
    vector<int> m_tetIdxORIG; // �����嶥������

    SoftObject(string name, string objFile, string tetFile) : m_name(name), m_objFile(objFile), m_tetFile(tetFile), m_renderObjId(-1){};

    void ReadFromFile();
};