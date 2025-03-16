#ifndef _IMGUI_HELPER_H_
#define _IMGUI_HELPER_H_

#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "glm/glm.hpp"

class Mesh;

class ImguiHelper
{
	static ImguiHelper* instance;
	Mesh* binded_item;
	bool enable = false;
	bool useImGuizmo = false;
public:

	void init(GLFWwindow* window);
	void bind(Mesh* item);
	void newFrame();
	bool isHover();
	void showDemo();
	void display(const glm::mat4& view_matrix, const glm::mat4& projection_matrix);
	Mesh* getBinded()
	{
		return binded_item;
	}
	glm::vec3 getLocation();
	static ImguiHelper* Instance();
	static void Free();
};

#endif