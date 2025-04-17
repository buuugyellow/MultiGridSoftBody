#pragma once
#define PRINT_CUDA_ERROR

#include "Simulator.h"
#include "application.hpp"
#include "glog/logging.h"

#include <string>
using namespace std;
extern string config_objName;
extern string config_objName_coarse;
extern string config_dataDir;
extern FILE* energyOutputFile;
extern vector<float> g_pointsForRender;  // �����߳���Ⱦ�̹߳���Ķ������ݣ���Ҫ���ݵİ������������� + ������ + UV����
extern vector<float> g_normalsForRender;
extern vector<float> g_uvForRender;

extern Application* g_render;
extern Simulator* g_simulator;

extern double duration_physical;

void OutputPosNormIndex(string filepath, std::vector<float> pos, std::vector<float> norm, std::vector<unsigned int> index);
float Matrix_Inverse_3(float* A, float* R);  // �������� R=inv(A)