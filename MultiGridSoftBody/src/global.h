#pragma once
//#define PRINT_CUDA_ERROR

#include <cuda.h>
#include <cuda_runtime.h>

#include <string>

#include "Simulator.h"
#include "application.hpp"
#include "glog/logging.h"
using namespace std;
extern string config_objName;
extern string config_objName_coarse;
extern string config_dataDir;
extern FILE* timeOutputFile;
extern FILE* energyOutputFile;
extern FILE* energyStepFile;
extern bool config_writeOrReadEnergy;
extern vector<float> g_pointsForRender;  // 仿真线程渲染线程共享的顶点数据，需要传递的包括：顶点坐标 + 法向量 + UV坐标
extern vector<float> g_normalsForRender;
extern vector<float> g_uvForRender;

extern Application* g_render;
extern Simulator* g_simulator;
extern int g_stepCnt;
extern vector<float> g_conEnergy;  // 保留读取的收敛能量
extern vector<float> g_conEk;      // 保留读取的收敛能量
extern vector<float> g_conEp;      // 保留读取的收敛能量
extern float g_conEnergy_V2;       // 每次迭代都先跑一次收敛 step，记录能量
extern float g_conEk_V2;
extern float g_conEp_V2;
enum SolverType { PD, PD_MG };
extern SolverType g_solverType;
extern double duration_physical;

__device__ __host__ struct Point3D {
    float x, y, z;
    __device__ __host__ Point3D(float* point) : x(point[0]), y(point[1]), z(point[2]) {}
    __device__ __host__ Point3D(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
};

__host__ void renderOnce();
__host__ void printCudaError(const char* funcName);
__host__ void OutputPosNormIndex(string filepath, std::vector<float> pos, std::vector<float> norm, std::vector<unsigned int> index);
__device__ __host__ float Matrix_Inverse_3(float* A, float* R);                                  // 矩阵求逆 R=inv(A)
__device__ __host__ void MatrixProduct_3_D(const float* A, const float* B, float* R);            // R=A*B
__device__ __host__ void MatrixSubstract_3_D(float* A, float* B, float* R);                      // R=A-B
__device__ __host__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz);  // R=A*B
__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3]);
__device__ __host__ Point3D operator-(const Point3D& a, const Point3D& b);                                     // 向量减法
__device__ __host__ Point3D operator+(const Point3D& a, const Point3D& b);                                     // 向量加法
__device__ __host__ Point3D operator/(const Point3D& a, float b);                                              // 标量除法
__device__ __host__ Point3D operator*(const Point3D& a, float b);                                              // 标量乘法
__device__ __host__ Point3D crossProduct(const Point3D& a, const Point3D& b);                                  // 叉乘计算
__device__ __host__ float dotProduct(const Point3D& a, const Point3D& b);                                      // 点积计算
__device__ __host__ float vectorLength(const Point3D& v);                                                      // 向量模长
__device__ __host__ bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point);  // 判断点是否在四面体内
__device__ __host__ vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal,
                                                        float V);  // 有预计算数据的四面体重心坐标计算，如四面体体积、三角形面积等
__device__ __host__ void barycentricCoordinate(const Point3D& point, const Point3D& tetVertA, const Point3D& tetVertB, const Point3D& tetVertC,
                                               const Point3D& tetVertD, float* weights);
__device__ __host__ float GetVolumn(const Point3D& A, const Point3D& B, const Point3D& C, const Point3D& D);