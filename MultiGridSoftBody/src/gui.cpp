#include <iostream>
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "global.h"
#include "glfw/include/GLFW/glfw3.h"

using namespace std;

bool g_UIShowMesh = true;
bool g_UIWireframe = false;
bool g_UIShowParticle = true;
bool g_UIEnergeOrCllisioin = false;
float g_UIParticleR = 0.05f;
int g_key = 0;

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        g_key = key;
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
    
    ImGui::Text("duration_phy: %.2fms", duration_physical / 1000);

    ImGui::Checkbox("Mesh", &g_UIShowMesh);
    if (g_UIShowMesh) ImGui::Checkbox("Wireframe", &g_UIWireframe);

    ImGui::Checkbox("Particle", &g_UIShowParticle);
    if (g_UIShowParticle) {
        ImGui::SliderFloat(u8"Particle Radius", &g_UIParticleR, 0.05, 0.10);
        ImGui::Checkbox("EnergyOrCollision", &g_UIEnergeOrCllisioin);
    }

    ImGui::PopStyleColor();
    ImGui::End();

    // 逻辑更新，这里不用全局变量是因为渲染是单独的模块，只暴露接口
    g_render->SetShowPartical(g_UIShowParticle);
    g_render->SetParticalRadius(g_UIParticleR);
    g_render->SetActive(g_simulator->m_softObject->m_renderObjId, g_UIShowMesh);
    g_render->SetWireframe(g_simulator->m_softObject->m_renderObjId, g_UIWireframe);
}