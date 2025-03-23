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

    // 原始 obj 文件读入的数据
    vector<float> m_triVertPos; // 顶点坐标
    vector<float> m_triUV; // 纹理坐标
    vector<int> m_triIdx; // 三角形顶点索引
    vector<int> m_triUVIdx; // 三角形 UV 索引

    // 原始 msh 文件读入的数据
    vector<float> m_tetVertPosORIG; // 四面体顶点坐标
    vector<int> m_tetIdxORIG; // 四面体顶点索引

    SoftObject(string name, string objFile, string tetFile) : m_name(name), m_objFile(objFile), m_tetFile(tetFile), m_renderObjId(-1){};

    void ReadFromFile();
};