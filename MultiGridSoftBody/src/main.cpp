#include <cuda_runtime.h>

#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "Simulator.h"
#include "application.hpp"
#include "global.h"
#include "gui.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_internal.h"
using namespace std;

string config_dataDir;
string config_objName;  // 单一物体
string config_objName_coarse;
string config_energyOutputCsv;
string config_energyStepInCsv;
string config_energyStepOutCsv;
FILE* energyOutputFile;
FILE* energyStepFile;           // 每次 step 结束之后的能量，用于记录收敛状态的能量
bool config_writeOrReadEnergy;  // 读取或者写入收敛能量文件
vector<float> g_conEnergy;      // 保留读取的收敛能量
vector<float> g_conEk;          // 保留读取的收敛能量
vector<float> g_conEp;          // 保留读取的收敛能量
float g_conEnergy_V2;           // 每次迭代都先跑一次收敛 step，记录能量
float g_conEk_V2;
float g_conEp_V2;

SolverType g_solverType;

Application* g_render;
Simulator* g_simulator;

mutex mtx;                        // 管理 CPU 上的顶点数据，仿真线程写，渲染线程读
vector<float> g_pointsForRender;  // 仿真线程渲染线程共享的顶点数据，需要传递的包括：顶点坐标 + 法向量 + UV坐标
vector<float> g_normalsForRender;
vector<float> g_uvForRender;
vector<float> g_pointsNormalsUVForRender;

void initCuda() {
    int nvDeviceNum = 0;
    cudaGetDeviceCount(&nvDeviceNum);
    if (nvDeviceNum == 0) {
        LOG(ERROR) << "没有发现CUDA设备";
        return;
    }
    std::vector<cudaDeviceProp> props(nvDeviceNum);
    std::vector<bool> propertySuccess(nvDeviceNum);
    for (int i = 0; i < nvDeviceNum; i++) {
        if (cudaSuccess == cudaGetDeviceProperties(&props[i], i))
            propertySuccess[i] = true;
        else
            propertySuccess[i] = false;
    }
    size_t maxMemSize = 0;
    int deviceID = -1;
    int count = 0;
    for (int i = 0; i < nvDeviceNum; i++) {
        if (propertySuccess[i])
            if (props[i].major >= 1)
                if (props[i].totalGlobalMem > maxMemSize) {
                    maxMemSize = props[i].totalGlobalMem;
                    deviceID = i;
                    count++;
                }
    }
    if (deviceID == -1) {
        LOG(ERROR) << "没有设备支持CUDA";
        return;
    }
    cudaSetDevice(deviceID);
    LOG(INFO) << "CUDA设备名称：" + std::string(props[deviceID].name);
}

void renderLoop() {
    g_render = Application::Instance();
    string shaderFolder = "../shader/";
    string hdrfile = "env1.hdr";
    string name = "HepaticTumor";
    g_render->InitRender(hdrfile, shaderFolder, name, doUI);
    // bindRenderObjs(); // 如果这个定义在全局，渲染线程会找不到这个函数
    float ssoaparas[8] = {2.0f, 1.5f, 0.9f, 0.9f, 0.009f, 2.8f};
    g_render->SetSSAOParas(ssoaparas);
    g_simulator->m_softObject->m_renderObjId = g_render->CreatePBRObj(g_simulator->m_softObject->m_name, 0.6, 0.5, 0.4, 0.2, 0.3);

    int ret = 0;
    while (!ret) {
        {
            lock_guard<mutex> lock(mtx);
            for (int i = 0; i < g_pointsNormalsUVForRender.size() / 9; i++) {
                g_pointsNormalsUVForRender[i * 9 + 0] = g_pointsForRender[i * 3 + 0];
                g_pointsNormalsUVForRender[i * 9 + 1] = g_pointsForRender[i * 3 + 1];
                g_pointsNormalsUVForRender[i * 9 + 2] = g_pointsForRender[i * 3 + 2];
                g_pointsNormalsUVForRender[i * 9 + 3] = g_normalsForRender[i * 3 + 0];
                g_pointsNormalsUVForRender[i * 9 + 4] = g_normalsForRender[i * 3 + 1];
                g_pointsNormalsUVForRender[i * 9 + 5] = g_normalsForRender[i * 3 + 2];
                // g_pointsNormalsUVForRender[i * 9 + 6] = g_uvForRender[i * 2 + 0];
                // g_pointsNormalsUVForRender[i * 9 + 7] = g_uvForRender[i * 2 + 1];
                // g_pointsNormalsUVForRender[i * 9 + 8] = 0.0f;
            }
        }
        GImGui;
        g_render->UpdateMesh(g_simulator->m_softObject->m_renderObjId, g_simulator->m_tetFaceIdx.size(),
                             g_simulator->m_tetFaceIdx.size() * sizeof(unsigned int), g_simulator->m_tetFaceIdx.data(),
                             g_pointsNormalsUVForRender.size() * sizeof(float), g_pointsNormalsUVForRender.data());
        ret = g_render->Render();
    }
    exit(ret);
}

void fileIO() {
    errno_t err = fopen_s(&energyOutputFile, config_energyOutputCsv.c_str(), "w+");
    if (err) {
        LOG(ERROR) << "打开 csv 文件失败";
    } else {
        fprintf(energyOutputFile, "iter,Energy,Ek,Ep,deltaX,error\n");
    }
    
    if (config_writeOrReadEnergy) {
        err = fopen_s(&energyStepFile, config_energyStepOutCsv.c_str(), "w+");
        if (err) {
            LOG(ERROR) << "打开 csv 文件失败";
        } else {
            fprintf(energyStepFile, "Energy,Ek,Ep\n");
        }
    } else {
        err = fopen_s(&energyStepFile, config_energyStepInCsv.c_str(), "r");
        if (err) {
            LOG(ERROR) << "打开 csv 文件失败";
        } else {
            char row[80];
            char* ptr = NULL;
            fgets(row, 80, energyStepFile);
            LOG(INFO) << "energyStepFile 首行：" << row;
            int cnt = 0;
            while (fgets(row, 80, energyStepFile) != NULL && ++cnt < 200) {
                char* token = strtok_s(row, ",", &ptr);
                g_conEnergy.push_back(stof(token));
                token = strtok_s(NULL, ",", &ptr);
                g_conEk.push_back(stof(token));
                token = strtok_s(NULL, ",", &ptr);
                g_conEp.push_back(stof(token));
            }
        }
    }
}

void init() {
    config_dataDir = "../data/";
    config_objName = "cube120_12_12";  // 单一物体
    config_objName_coarse = "cube40_4_4";
    g_solverType = PD;
    FLAGS_log_dir = "../temp/log/";
    config_energyOutputCsv = "../temp/energy.csv";
    config_energyStepInCsv = "../data/120_256iter.csv";
    config_energyStepOutCsv = "../temp/energyStepOut.csv";
    config_writeOrReadEnergy = false;
    FLAGS_logtostderr = true;
    FLAGS_stderrthreshold = 0;
    google::InitGoogleLogging("MultiGridSoftBody");
    
    fileIO();
    initCuda();
    g_simulator = &Simulator::GetInstance();
    g_simulator->Init();
    g_pointsNormalsUVForRender.resize(g_simulator->m_tetVertPos.size() * 3);
    g_pointsForRender.resize(g_simulator->m_tetVertPos.size());
    g_normalsForRender.resize(g_simulator->m_tetVertPos.size());
    thread(renderLoop).detach();
    LOG(INFO) << "main init 结束";
}

void run() {
    while (true) {
        g_simulator->Update();
        if (mtx.try_lock()) {  // 成功获取锁，此时写顶点数据，否则继续
            // 在四面体顶点中抽取出表面顶点坐标
            g_pointsForRender = g_simulator->m_tetVertPos;
            g_normalsForRender = g_simulator->m_normal;
            mtx.unlock();
        }
    }
}

int main() {
    init();
    run();
    return 0;
}