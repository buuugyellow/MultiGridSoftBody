#include <iostream>
#include "gui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "global.h"

using namespace std;

bool g_UIShowParticle = false;
float g_UIParticleR = 0.05f;

void doUI() {
	//cout << "doUI" << endl;
    //ImGui::Text(u8"test");
    cout << "duration_phy: " << duration_physical / 1000 << "ms, fps_phy: " << 1e6 / duration_physical << endl;

    //ImGui::GetStyle().Alpha = 0.5f;
    //ImGui::SetNextWindowPos(ImVec2(10, 10));
    //ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    //ImGui::Begin(u8"-", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove);

    //ImGui::Checkbox("Particle", &g_UIShowParticle);
    //if (g_UIShowParticle) ImGui::SliderFloat(u8"Particle Radius", &g_UIParticleR, 0.05, 0.5);
}