#include "opengl.hpp"
#include "ImGuizmo.h"
#include "ImguiHelper.hpp"
#include <iostream>

ImguiHelper* ImguiHelper::instance = nullptr;

void ImguiHelper::init(GLFWwindow* window)
{

	auto g_ctx = ImGui::CreateContext();
	ImGui::SetCurrentContext(g_ctx);


	auto& io = ImGui::GetIO();
	io.Fonts->AddFontFromFileTTF("c:/windows/fonts/simhei.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());

	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	io.ConfigWindowsMoveFromTitleBarOnly = true;

	if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
	{
		printf("ImGui_ImplGlfw_InitForOpenGL failed\n");
	};
	if (!ImGui_ImplOpenGL3_Init("#version 330"))
	{
		printf("ImGui_ImplOpenGL3_Init failed\n");
	};

	enable = true;


	if(useImGuizmo)ImGuizmo::SetOrthographic(false);
	if (useImGuizmo)ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
}

void ImguiHelper::bind(Mesh* item)
{
	this->binded_item = item;
}

void ImguiHelper::newFrame()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	if (useImGuizmo)ImGuizmo::BeginFrame();
	auto& io = ImGui::GetIO();
	if (useImGuizmo)ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

	ImGui::DockSpaceOverViewport(nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

}

bool ImguiHelper::isHover()
{
	return enable && ImGui::GetIO().WantCaptureMouse;
}

void ImguiHelper::showDemo()
{
	ImGui::Begin(u8"²âÊÔ´°¿Ú");

	ImGui::Text(u8"ÄãºÃ");
	ImGui::Button(u8"°´¼ü");
	ImGui::End();
}

void ImguiHelper::display(const glm::mat4& view_matrix, const glm::mat4& projection_matrix)
{
	glm::mat4 model_matrix = glm::mat4(1.0f);
	if (binded_item&& useImGuizmo)
	{
		ImGuizmo::Manipulate(
			(const float*)&view_matrix,
			(const float*)&projection_matrix,
			ImGuizmo::TRANSLATE,  // or ROTATE, or SCALE
			ImGuizmo::WORLD,      // or LOCAL
			(float*)&binded_item->m_transform
		);
	}

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

glm::vec3 ImguiHelper::getLocation()
{
	if (binded_item)
		return binded_item->m_transform[3];
	return glm::vec3(0);
}

ImguiHelper* ImguiHelper::Instance()
{
    if (!instance)
        instance = new ImguiHelper();

    return instance;
}

void ImguiHelper::Free()
{
    if (instance)
    {
        delete instance;
        instance = nullptr;
    }
}
