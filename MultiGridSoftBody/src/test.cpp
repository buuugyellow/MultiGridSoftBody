#include <iostream>
#include <string>
#include "application.hpp"
#include "imgui.h"
using namespace std;

int main() {
	string shaderFolder = "../shader/";
	string hdrfile = "env1.hdr";
	string name = "HepaticTumor";
	Application* g_app = Application::Instance();
	g_app->InitRender(hdrfile, shaderFolder, name, doUI);

	while (1) {
		g_app->Render();
	}
	return 0;
}