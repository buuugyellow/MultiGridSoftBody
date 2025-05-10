#pragma once
//#define PRINT_CUDA_ERROR
#include <string>
#include "simpleMath.h"
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
extern bool g_UIShowMesh;
extern bool g_UIShowParticle;
extern float g_UIParticleR;

void renderOnce();
void printCudaError(const char* funcName);
void OutputPosNormIndex(string filepath, vector<float> pos, vector<float> norm, vector<unsigned int> index);