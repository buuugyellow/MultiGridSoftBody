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
extern vector<float> g_pointsForRender;  // 仿真线程渲染线程共享的顶点数据，需要传递的包括：顶点坐标 + 法向量 + UV坐标
extern vector<float> g_normalsForRender;
extern vector<float> g_uvForRender;

extern Application* g_render;
extern Simulator* g_simulator;

extern double duration_physical;

void OutputPosNormIndex(string filepath, std::vector<float> pos, std::vector<float> norm, std::vector<unsigned int> index);
float Matrix_Inverse_3(float* A, float* R);  // 矩阵求逆 R=inv(A)

struct Point3D {
    float x, y, z;
    Point3D(float* point) : x(point[0]), y(point[1]), z(point[2]){}
    Point3D(float xx, float yy, float zz) : x(xx), y(yy), z(zz){}
};

Point3D operator-(const Point3D& a, const Point3D& b); // 向量减法
Point3D operator/(const Point3D& a, float b);              // 标量除法
Point3D crossProduct(const Point3D& a, const Point3D& b);  // 叉乘计算
float dotProduct(const Point3D& a, const Point3D& b);      // 点积计算
float vectorLength(const Point3D& v);                      // 向量模长
bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point); // 判断点是否在四面体内
vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal, float V);