#include <stdexcept>

#include "application.hpp"
#include "ImguiHelper.hpp"
#include "glfw/include/GLFW/glfw3.h"
namespace {
	const int DisplaySizeX = 1920;
	const int DisplaySizeY = 1000;
	const int DisplaySamples = 16;

	const float ViewDistance = 150.0f;
	const float ViewFOV      = 45.0f;
	const float OrbitSpeed   = 1.0f;
	const float ZoomSpeed    = 4.0f;
}

Application* g_application = nullptr;

Application* Application::Instance() {
	if (!g_application)
	{
		g_application = new Application();
	}
	return g_application;
}

Application::Application()
	: m_window(nullptr)
	, m_prevCursorX(0.0)
	, m_prevCursorY(0.0)
	, m_mode(InputMode::None)
{
	if(!glfwInit()) {
		throw std::runtime_error("Failed to initialize GLFW library");
	}

	//m_viewSettings.distance = ViewDistance;
	//m_viewSettings.fov      = ViewFOV;

	//m_sceneSettings.lights[0].direction = glm::normalize(glm::vec3{-1.0f,  0.0f, 0.0f});
	//m_sceneSettings.lights[1].direction = glm::normalize(glm::vec3{ 1.0f,  0.0f, 0.0f});
	//m_sceneSettings.lights[2].direction = glm::normalize(glm::vec3{ 0.0f, -1.0f, 0.0f});

	//m_sceneSettings.lights[0].radiance = glm::vec3{1.0f};
	//m_sceneSettings.lights[1].radiance = glm::vec3{1.0f};
	//m_sceneSettings.lights[2].radiance = glm::vec3{1.0f};

	m_camera = new Utility::ArcballCamera(DisplaySizeX, DisplaySizeY,glm::vec3(-3,-1,10),glm::vec3(0,0,0));
	m_viewSettings.camera = m_camera;
}

Application::~Application()
{
	if(m_window) {
		glfwDestroyWindow(m_window);
	}
	delete m_camera;
	glfwTerminate();
}

void Application::SetHDRFile(std::string name) {
	m_renderer->m_sceneobject->m_hdrfile = name;
}

void Application::SetSSAOParas(float* p) {
	m_renderer->m_sceneobject->m_blurpara.x = p[0];
	m_renderer->m_sceneobject->m_blurpara.y = p[1];
	m_renderer->m_sceneobject->m_blurpara.z = p[2];
	m_renderer->m_sceneobject->m_ssaoPara[0] = p[3];
	m_renderer->m_sceneobject->m_ssaoPara[1] = p[4];
	m_renderer->m_sceneobject->m_ssaoPara[2] = p[5];
}

void Application::SetShaderFolder(std::string name) {
	m_renderer->m_sceneobject->m_shaderFolder = name;
}

void Application::ShutDown() {
	m_renderer->shutdown();
}

int Application::Render() {
	int ret = glfwWindowShouldClose(m_window);
	ImguiHelper::Instance()->newFrame();
	m_renderer->render(m_window, m_viewSettings, m_sceneSettings);
	auto view = m_viewSettings.camera->getView();
	auto proj = m_viewSettings.camera->getProj();
	ImguiHelper::Instance()->display(view, proj);
	glfwSwapBuffers(m_window);
	glfwPollEvents();
	return ret;
}

void Application::ClearActieve() {
	for (auto obj : m_renderer->m_sceneobject->m_meshes)
		obj->m_draw = false;
}

void Application::SetActieve(int i) {
	m_renderer->m_sceneobject->m_meshes[i]->m_draw = true;
}

void Application::InitRender(std::string& hdrfile, std::string& shaderfolder, std::string& name, void(*doUICallBack)()) {
	m_renderer = new Renderer();
	m_renderer->m_name = name;
	m_renderer -> doUICallBack = doUICallBack;
	m_window = m_renderer->initialize(DisplaySizeX, DisplaySizeY, DisplaySamples);
	glfwSetFramebufferSizeCallback(m_window, Application::framebufferSizeCallback);
	glfwSetCursorPosCallback(m_window, Application::mousePositionCallback);
	glfwSetMouseButtonCallback(m_window, Application::mouseButtonCallback);
	glfwSetScrollCallback(m_window, Application::mouseScrollCallback);
	glfwSetKeyCallback(m_window, Application::keyCallback);
	ImguiHelper::Instance()->init(m_window);
	m_renderer->m_sceneobject->m_hdrfile = hdrfile;
	m_renderer->m_sceneobject->m_shaderFolder = shaderfolder;
	m_renderer->m_sceneobject->initialize();
    LoadCam("../data/cam1.txt");
	glLineWidth(3);
}

ImGuiContext* Application::GetImGuiContext() { return GImGui; }

void Application::UpdatePartical(int num, float* vert) { 
	m_renderer->m_particleRenderBuffers.mNumParticles = num;
    m_renderer->UpdateBuffers(m_renderer->m_particleRenderBuffers, vert);
}

void Application::SetShowPartical(bool show) { m_renderer->m_showParticle = show; }

void Application::SetParticalRadius(float r) { m_renderer->m_particleR = r; }

void Application::SetTransparentColor(int objId, float r, float g, float b, float a) {
	if (m_renderer == nullptr)
		return;
	return m_renderer->m_sceneobject->setTransparentColor(objId,r,g,b,a);
}

int Application::CreatePBRObj(std::string name, const std::string& abeldo, const std::string& normal,
	const std::string& metallic, const std::string& roughness) {
	if (m_renderer == nullptr)
		return -1;
	return m_renderer->m_sceneobject->createPBRObj(name, abeldo, normal,
		metallic, roughness);
}

int Application::CreatePBRObj(std::string name, float r, float g, float b, float metallic, float roughness) {
	if (m_renderer == nullptr)
		return -1;
	return m_renderer->m_sceneobject->createPBRObj(name, r, g, b, metallic, roughness);
}

void Application::UpdateMesh(int idx, int eleNum, int triNum, unsigned int* triIdx, int vertNum, float* vert) {
	if (m_renderer == nullptr)
		return;
	if (idx < m_renderer->m_sceneobject->m_meshes.size())
		m_renderer->m_sceneobject->m_meshes[idx]->updateMesh(eleNum, triNum, triIdx, vertNum, vert);
}

void Application::UpdateMeshConst(int idx,  int triNum, unsigned int* triIdx, int vertNum, float* vert) {
	if (m_renderer == nullptr)
		return;
	if (idx < m_renderer->m_sceneobject->m_meshes.size())
		m_renderer->m_sceneobject->m_meshes[idx]->updateMeshConst(triNum, triIdx, vertNum, vert);
}

void Application::UpdateTransform(int idx, float* t) {
	if (m_renderer == nullptr)
		return;
	if (idx < m_renderer->m_sceneobject->m_meshes.size())
		m_renderer->m_sceneobject->m_meshes[idx]->updateTransform(t);
}


void Application::framebufferSizeCallback(GLFWwindow* window, int w, int h)
{
	Application::Instance()->getCamera()->setViewSize(0, 0, w, h);
	Application::Instance()->getRenderer()->resize(w, h);
}

void Application::mousePositionCallback(GLFWwindow* window, double xpos, double ypos)
{
	if (ImguiHelper::Instance()->isHover())
		return;
	Application::Instance()->getCamera()->inputMouse(xpos, ypos, -1, -1);
}
	
void Application::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
	if (ImguiHelper::Instance()->isHover())
		return;
	double mousx, mousy;
	glfwGetCursorPos(window, &mousx, &mousy);
	Application::Instance()->getCamera()->inputMouse(mousx, mousy, button, GLFW_PRESS == action);
}
	
void Application::mouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
	if (ImguiHelper::Instance()->isHover())
		return;
	Application::Instance()->getCamera()->inputMouseScroll(yoffset > 0 ? 120 : -120);
}
	
void Application::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if(action == GLFW_PRESS) {
		switch (key) {
		case GLFW_KEY_ESCAPE:
			Application::Instance()->m_running = false;
		case GLFW_KEY_F1:
			LoadCam("../data/cam1.txt");
			break;
		case GLFW_KEY_F2:
            LoadCam("../data/cam2.txt");
			break;
		case GLFW_KEY_F3:
            LoadCam("../data/cam3.txt");
			break;
		case GLFW_KEY_F11:
			SaveCam();
			break;
		case GLFW_KEY_H:
			Application::Instance()->getRenderer()->m_doUI = !Application::Instance()->getRenderer()->m_doUI;
			break;
		case GLFW_KEY_S:
			Application::Instance()->m_step = true;
		case GLFW_KEY_B:
			Application::Instance()->m_burn = true;
			
		}
	}

}


void Application::SaveCam() {
	FILE* fid = fopen("cam.txt","w");
	if (!fid)
		return;

	glm::vec3& e = Application::Instance()->getCamera()->eye;
	glm::vec3& t = Application::Instance()->getCamera()->target;
	glm::vec3& u = Application::Instance()->getCamera()->up;
	fprintf(fid, "%.9f \n", e.x);
	fprintf(fid, "%.9f \n", e.y);
	fprintf(fid, "%.9f \n", e.z);

	fprintf(fid, "%.9f \n", t.x);
	fprintf(fid, "%.9f \n", t.y);
	fprintf(fid, "%.9f \n", t.z);

	fprintf(fid, "%.9f \n", u.x);
	fprintf(fid, "%.9f \n", u.y);
	fprintf(fid, "%.9f \n", u.z);
	fclose(fid);		
}

void Application::LoadCam(const char* name) {
	FILE* fid = fopen(name, "r");
	if (!fid)
		return;

	glm::vec3& e = Application::Instance()->getCamera()->eye;
	glm::vec3& t = Application::Instance()->getCamera()->target;
	glm::vec3& u = Application::Instance()->getCamera()->up;
	fscanf(fid, "%f \n", &e.x);
	fscanf(fid, "%f \n", &e.y);
	fscanf(fid, "%f \n", &e.z);

	fscanf(fid, "%f \n", &t.x);
	fscanf(fid, "%f \n", &t.y);
	fscanf(fid, "%f \n", &t.z);

	fscanf(fid, "%f \n", &u.x);
	fscanf(fid, "%f \n", &u.y);
	fscanf(fid, "%f \n", &u.z);
	fclose(fid);
}