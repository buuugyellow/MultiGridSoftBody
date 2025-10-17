#pragma once
//#define PRINT_CUDA_ERROR
#ifdef PRINT_CUDA_ERROR
#define PRINT_CUDA_ERROR_AFTER(func) \
    do {                             \
        cudaDeviceSynchronize();     \
        printCudaError(func);        \
    } while (0)
#else
#define PRINT_CUDA_ERROR_AFTER(func)
#endif  // PRINT_CUDA_ERROR

#define BLOCK_SIZE(eleNum, threadNum) (((eleNum) + (threadNum) - 1) / (threadNum))

#include <string>
#include "simpleMath.h"
#include "Simulator.h"
#include "application.hpp"
#include "glog/logging.h"
using namespace std;
extern string config_objName;
extern string config_objName_coarse;
extern string config_dataDir;
extern string config_tempDir;
extern FILE* timeOutputFile_physical;
extern FILE* timeOutputFile_render;
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

extern double duration_physical;
extern double duration_physicalInner;
extern bool g_UIShowMesh;
extern bool g_UIShowParticle;
extern bool g_UIEnergeOrCllisioin;
extern float g_UIParticleR;
extern int g_key;
extern double g_realDuration;
extern double g_renderDuration;
extern double g_totalDuration;
extern int g_collidedVertCnt;
extern chrono::steady_clock::time_point g_systemBeginTime;

void doUI();
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void renderOnce();
void printCudaError(const char* funcName);
void OutputPosNormIndex(string filepath, vector<float> pos, vector<float> norm, vector<unsigned int> index);