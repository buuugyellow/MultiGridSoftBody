#include <iostream>
#include "gui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "global.h"

using namespace std;

void doUI() {
	//cout << "doUI" << endl;
    //ImGui::Text(u8"test");
    cout << "duration_phy: " << duration_physical / 1000 << "ms, fps_phy: " << 1e6 / duration_physical << endl;
}