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

// ��������
__device__ __host__ Point3D operator-(const Point3D& a, const Point3D& b);     
// �����ӷ�
__device__ __host__ Point3D operator+(const Point3D& a, const Point3D& b);     
// ��������
__device__ __host__ Point3D operator/(const Point3D& a, float b);              
// �����˷�
__device__ __host__ Point3D operator*(const Point3D& a, float b);              
// ��˼���
__device__ __host__ Point3D crossProduct(const Point3D& a, const Point3D& b);  
// �������
__device__ __host__ float dotProduct(const Point3D& a, const Point3D& b);      
// ����ģ��
__device__ __host__ float length(const Point3D& v);                            
// ����ģ��ƽ��
__device__ __host__ float lengthSq(const Point3D& v);                          
// ������һ��
__device__ __host__ void normalize(Point3D& a);                                
// �����������е���������
__device__ __host__ void barycentricCoordinate(const Point3D& point, const Point3D& tetVertA, const Point3D& tetVertB, const Point3D& tetVertC,
                                               const Point3D& tetVertD, float* weights);
// ���������
__device__ __host__ float GetVolumn(const Point3D& A, const Point3D& B, const Point3D& C, const Point3D& D);
// �������� R=inv(A)
__device__ __host__ float Matrix_Inverse_3(float* A, float* R);                                  
// R=A*B
__device__ __host__ void MatrixProduct_3_D(const float* A, const float* B, float* R);            
// R=A-B
__device__ __host__ void MatrixSubstract_3_D(float* A, float* B, float* R);                      
// R=A*B
__device__ __host__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz);  
// ��ȡ��ת����
__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3]);
// ��ȡ��ת�����Լ��α���������ʽ
__device__ __host__ void GetRotation_D(float F[3][3], float R[3][3], float& deltaF);
// �жϵ��Ƿ�����������
__device__ __host__ bool pointInTet(const float* tetVertPos, const float* tetFaceNormal, const float* point);
// ��Ԥ�������ݵ�����������������㣬������������������������
__device__ __host__ vector<float> barycentricCoordinate(const float* point, const float* tetCenter, const float* tetFaceArea, const float* tetFaceNormal,
                                                        float V);
// �жϵ�P�Ƿ���������ABC�ڲ���ʹ���������귨��
__device__ __host__ bool pointInTriangle(const Point3D& P, const Point3D& A, const Point3D& B, const Point3D& C);
// �����P���߶�AB����̾���ƽ���������������
__device__ __host__ float closestPointOnSegment(const Point3D& P, const Point3D& A, const Point3D& B, Point3D& closest);
// �����������Ƿ��ཻ
__device__ __host__ bool sphereIntersectTri(const Point3D& center, float radius, const Point3D& A, const Point3D& B, const Point3D& C);
// ���Ƿ����߶εĴ�ֱ��Χ�ڣ�����ṩ�� radius���򷵻ص��Ƿ���Բ������
__device__ __host__ bool isOnCylinderSegment(const Point3D& P, const Point3D& A, const Point3D& B, float radius = -1);

void CreateSphere(float radius, int slices, int segments, vector<float>& verts, vector<unsigned int>& indices, const vector<float>& center);