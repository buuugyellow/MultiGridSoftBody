#pragma once
#define PRINT_CUDA_ERROR

#include "Simulator.h"
#include "application.hpp"
#include "glog/logging.h"
#include "cudaUtils.h"

#include <string>
using namespace std;
extern string config_objName;
extern string config_objName_coarse;
extern string config_dataDir;
extern FILE* energyOutputFile;
extern FILE* energyStepFile;
extern vector<float> g_pointsForRender;  // �����߳���Ⱦ�̹߳���Ķ������ݣ���Ҫ���ݵİ������������� + ������ + UV����
extern vector<float> g_normalsForRender;
extern vector<float> g_uvForRender;

extern Application* g_render;
extern Simulator* g_simulator;

extern double duration_physical;

void OutputPosNormIndex(string filepath, std::vector<float> pos, std::vector<float> norm, std::vector<unsigned int> index);
float Matrix_Inverse_3(float* A, float* R);  // �������� R=inv(A)

struct Point3D {
    float x, y, z;
    Point3D(float* point) : x(point[0]), y(point[1]), z(point[2]){}
    Point3D(float xx, float yy, float zz) : x(xx), y(yy), z(zz){}
};

Point3D operator-(const Point3D& a, const Point3D& b); // ��������
Point3D operator/(const Point3D& a, float b);              // ��������
Point3D crossProduct(const Point3D& a, const Point3D& b);  // ��˼���
float dotProduct(const Point3D& a, const Point3D& b);      // �������
float vectorLength(const Point3D& v);                      // ����ģ��
bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point); // �жϵ��Ƿ�����������
vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal, float V);