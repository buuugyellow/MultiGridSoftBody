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
extern vector<float> g_pointsForRender;  // �����߳���Ⱦ�̹߳���Ķ������ݣ���Ҫ���ݵİ������������� + ������ + UV����
extern vector<float> g_normalsForRender;
extern vector<float> g_uvForRender;

extern Application* g_render;
extern Simulator* g_simulator;
extern int g_stepCnt;
extern vector<float> g_conEnergy;  // ������ȡ����������
extern vector<float> g_conEk;      // ������ȡ����������
extern vector<float> g_conEp;      // ������ȡ����������
extern float g_conEnergy_V2;       // ÿ�ε���������һ������ step����¼����
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
__device__ __host__ float Matrix_Inverse_3(float* A, float* R);                                  // �������� R=inv(A)
__device__ __host__ void MatrixProduct_3_D(const float* A, const float* B, float* R);            // R=A*B
__device__ __host__ void MatrixSubstract_3_D(float* A, float* B, float* R);                      // R=A-B
__device__ __host__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz);  // R=A*B
__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3]);
__device__ __host__ Point3D operator-(const Point3D& a, const Point3D& b);                                     // ��������
__device__ __host__ Point3D operator+(const Point3D& a, const Point3D& b);                                     // �����ӷ�
__device__ __host__ Point3D operator/(const Point3D& a, float b);                                              // ��������
__device__ __host__ Point3D operator*(const Point3D& a, float b);                                              // �����˷�
__device__ __host__ Point3D crossProduct(const Point3D& a, const Point3D& b);                                  // ��˼���
__device__ __host__ float dotProduct(const Point3D& a, const Point3D& b);                                      // �������
__device__ __host__ float vectorLength(const Point3D& v);                                                      // ����ģ��
__device__ __host__ bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point);  // �жϵ��Ƿ�����������
__device__ __host__ vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal,
                                                        float V);  // ��Ԥ�������ݵ�����������������㣬������������������������
__device__ __host__ void barycentricCoordinate(const Point3D& point, const Point3D& tetVertA, const Point3D& tetVertB, const Point3D& tetVertC,
                                               const Point3D& tetVertD, float* weights);
__device__ __host__ float GetVolumn(const Point3D& A, const Point3D& B, const Point3D& C, const Point3D& D);