#include <iostream>
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "global.h"
#include "glfw/include/GLFW/glfw3.h"

using namespace std;

bool g_UIShowMesh = true;
bool g_UIShowParticle = true;
bool g_UIEnergeOrCllisioin = true;
float g_UIParticleR = 0.05f;
int g_key = 0;

bool l_UIWireframe = false;

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    g_key = key;
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_F1:
                Application::LoadCam("../data/cam1.txt");
                break;
            case GLFW_KEY_F2:
                Application::LoadCam("../data/cam2.txt");
                break;
            case GLFW_KEY_F3:
                Application::LoadCam("../data/cam3.txt");
                break;
            case GLFW_KEY_F11:
                Application::SaveCam();
                break;
            case GLFW_KEY_H:
                g_render->getRenderer()->m_doUI = !g_render->getRenderer()->m_doUI;
                break;
        }
    }
}

void doUI() {
    // UI 更新
    ImGui::Begin("MultiGridSoftBody", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove);
    ImGui::GetStyle().Alpha = 0.5f;
    ImGui::SetNextWindowPos(ImVec2(10, 10));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    
    ImGui::Text("TetVertCnt: %d", g_simulator->m_tetVertPos.size() / 3);
    ImGui::Text("DurationFixed: %.2fms", g_totalDuration / 1000);
    ImGui::Text("DurationReal: %.2fms", g_realDuration / 1000);
    ImGui::Text("DurationPhy: %.2fms", duration_physical / 1000);

    if (g_simulator->m_solverType == Simulator::PD) {
        ImGui::SliderInt("IterCnt", &g_simulator->m_solver->m_iterNum, 4, 128);
        ImGui::SliderFloat("VolumnStiffness", &g_simulator->m_solver->m_volumnStiffness, 1000, 10000);
        ImGui::SliderFloat("CollisionStiffness", &g_simulator->m_solver->m_collisionStiffness, 1000, 10000);
        ImGui::SliderFloat("Damping", &g_simulator->m_solver->m_damping, 0.1f, 1.0f);
    } else if (g_simulator->m_solverType == Simulator::PD_MG) {
    
    }

    ImGui::Checkbox("Mesh", &g_UIShowMesh);
    if (g_UIShowMesh) ImGui::Checkbox("Wireframe", &l_UIWireframe);

    ImGui::Checkbox("Particle", &g_UIShowParticle);
    if (g_UIShowParticle) {
        ImGui::SliderFloat("ParticleRadius", &g_UIParticleR, 0.05, 0.10);
        ImGui::Checkbox("EnergyOrCollision", &g_UIEnergeOrCllisioin);
        ImGui::Text("CollidedVertCnt: %d", g_collidedVertCnt);
    }

    ImGui::PopStyleColor();
    ImGui::End();

    // 逻辑更新，这里不用全局变量是因为渲染是单独的模块，只暴露接口
    g_render->SetShowPartical(g_UIShowParticle);
    g_render->SetParticalRadius(g_UIParticleR);
    g_render->SetActive(g_simulator->m_softObject->m_renderObjId, g_UIShowMesh);
    g_render->SetWireframe(g_simulator->m_softObject->m_renderObjId, l_UIWireframe);
}