#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <vector>
using namespace std;
__device__ __host__ struct Point3D {
    float x, y, z;
    __device__ __host__ Point3D() : x(0), y(0), z(0) {}
    __device__ __host__ Point3D(float* point) : x(point[0]), y(point[1]), z(point[2]) {}
    __device__ __host__ Point3D(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
};

// 向量减法
__device__ __host__ Point3D operator-(const Point3D& a, const Point3D& b);     
// 向量加法
__device__ __host__ Point3D operator+(const Point3D& a, const Point3D& b);     
// 标量除法
__device__ __host__ Point3D operator/(const Point3D& a, float b);              
// 标量乘法
__device__ __host__ Point3D operator*(const Point3D& a, float b);              
// 叉乘计算
__device__ __host__ Point3D crossProduct(const Point3D& a, const Point3D& b);  
// 点积计算
__device__ __host__ float dotProduct(const Point3D& a, const Point3D& b);      
// 向量模长
__device__ __host__ float length(const Point3D& v);                            
// 向量模长平方
__device__ __host__ float lengthSq(const Point3D& v);                          
// 向量归一化
__device__ __host__ void normalize(Point3D& a);                                
// 点在四面体中的重心坐标
__device__ __host__ void barycentricCoordinate(const Point3D& point, const Point3D& tetVertA, const Point3D& tetVertB, const Point3D& tetVertC,
                                               const Point3D& tetVertD, float* weights);
// 四面体体积
__device__ __host__ float GetVolumn(const Point3D& A, const Point3D& B, const Point3D& C, const Point3D& D);
// 矩阵求逆 R=inv(A)
__device__ __host__ float Matrix_Inverse_3(float* A, float* R);                                  
// R=A*B
__device__ __host__ void MatrixProduct_3_D(const float* A, const float* B, float* R);            
// R=A-B
__device__ __host__ void MatrixSubstract_3_D(float* A, float* B, float* R);                      
// R=A*B
__device__ __host__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz);  
// 提取旋转矩阵
__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3]);
// 提取旋转矩阵以及形变矩阵的行列式
__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3], float& deltaF);
// 判断点是否在四面体内
__device__ __host__ bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point);
// 有预计算数据的四面体重心坐标计算，如四面体体积、三角形面积等
__device__ __host__ vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal,
                                                        float V);
// 判断点P是否在三角形ABC内部（使用重心坐标法）
__device__ __host__ bool pointInTriangle(const Point3D& P, const Point3D& A, const Point3D& B, const Point3D& C);
// 计算点P到线段AB的最短距离平方，并返回最近点
__device__ __host__ float closestPointOnSegment(const Point3D& P, const Point3D& A, const Point3D& B, Point3D& closest);
// 球与三角形是否相交
__device__ __host__ bool sphereIntersectTri(const Point3D& center, float radius, const Point3D& A, const Point3D& B, const Point3D& C);
// 点是否在线段的垂直范围内，如果提供了 radius，则返回点是否在圆柱体内
__device__ __host__ bool isOnCylinderSegment(const Point3D& P, const Point3D& A, const Point3D& B, float radius = -1);

void CreateSphere(float radius, int slices, int segments, vector<float>& verts, vector<unsigned int>& indices, const vector<float>& center);