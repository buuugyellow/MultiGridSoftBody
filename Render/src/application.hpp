/*
 * Physically Based Rendering
 * Copyright (c) 2017-2018 Michał Siejak
 */

#define OS_API_IMPORT __declspec(dllimport)
#define OS_API_EXPORT __declspec(dllexport)

#ifdef BUILD_DLL
#define OS_API OS_API_EXPORT //如果是生成dll工程，那么导出
#else
#define OS_API OS_API_IMPORT //如果是生成使用dll的工程，那么导入
#endif 

#pragma once

#include <memory>
#include "opengl.hpp"
#include "ArcballHelper.h"
#include "imgui/imgui_internal.h"

class OS_API Application
{
public:
	Application();
	~Application();


	void InitRender(std::string& hdrfile, std::string& shaderfolder, std::string& name, void (*doUICallBack)(),
                    void (*keyCallback)(GLFWwindow* window, int key, int scancode, int action, int mods), ImGuiContext*& imGuiContext);

	ImGuiContext* GetImGuiContext();

	void UpdatePartical(int num, float* vert);

	void SetShowPartical(bool show);

	void SetParticalRadius(float r);

	int Render();

	void ShutDown();

	int CreatePBRObj(std::string name, const std::string& abeldo, const std::string& normal,
		const std::string& metallic, const std::string& roughness);

	void ClearActive();

	void SetActive(int i, bool active);

	void SetWireframe(int i, bool wireframe);

	int CreatePBRObj(std::string name, float r, float g, float b, float metallic, float roughness);

	void SetTransparentColor(int objId, float r, float g, float b, float a);

	void UpdateMesh(int idx, int eleNum, int triNum, unsigned int* triIdx, int vertNum, float* vert);

	void UpdateMeshConst(int idx, int triNum, unsigned int* triIdx, int vertNum, float* vert);

	void UpdateTransform(int idx, float* t);

	void SetHDRFile(std::string name);

	void SetShaderFolder(std::string name);

	void SetSSAOParas(float* p);

	static void SaveCam();
	static void LoadCam(const char* name);

	Utility::ArcballCamera* getCamera()
	{
		return m_camera;
	};

	Renderer* getRenderer()
	{
		return m_renderer;
	}

	static Application* Instance();

private:
	static void framebufferSizeCallback(GLFWwindow* window, int w, int h);
	static void mousePositionCallback(GLFWwindow* window, double xpos, double ypos);
	static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	static void mouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

	GLFWwindow* m_window;
	double m_prevCursorX;
	double m_prevCursorY;

	ViewSettings m_viewSettings;
	SceneSettings m_sceneSettings;

	enum class InputMode
	{
		None,
		RotatingView,
		RotatingScene,
	};
	InputMode m_mode;

	Utility::ArcballCamera* m_camera = nullptr;
	Renderer* m_renderer = nullptr;

};
