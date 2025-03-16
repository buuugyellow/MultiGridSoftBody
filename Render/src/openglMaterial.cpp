/*
 * physically based rendering
 * copyright (c) 2017-2018 michał siejak
 *
 * OpenGL 4.5 renderer.
 */

#define NOMINMAX
#define GLM_ENABLE_EXPERIMENTAL

#include <stdexcept>
#include <memory>
#include <iostream>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "opengl.hpp"
#include "glfw/include/GLFW/glfw3.h"
#include "mesh.hpp"
#include "image.hpp"
#include "glutils.hpp"
#include "ImguiHelper.hpp"



MaterialBaseColor::MaterialBaseColor(glm::vec4 color)
{
	this->m_diffColor = color;
	//load shader for basecolor
	auto basic_program = loadProgram(m_dataFolder + "basiccolor.glsl");
	m_programs["basic"] = basic_program ;
}

void MaterialBaseColor::apply(Utility::ArcballCamera* camera)
{
	auto& prog = m_programs["basic"];
	glUseProgram(prog);
	//set uniforms
	//matrix
	GLint view_location = glGetUniformLocation(prog, "view");
	if (view_location != -1) {
		glUniformMatrix4fv(view_location, 1,false, &camera->getView()[0][0]);
	}
	GLint proj_location = glGetUniformLocation(prog, "proj");
	if (proj_location != -1) {
		glUniformMatrix4fv(proj_location, 1, false, &camera->getProj()[0][0]);
	}

	//color
	GLint color_location = glGetUniformLocation(prog, "color");
	if (color_location != -1) {
		glUniform4fv(color_location,1, &m_diffColor.x);
	}
}

uint32_t MaterialBaseColor::getDefaultProgram()
{
	return m_programs["basic"];
}

MaterialPBR::MaterialPBR(std::string dataFolder)
{
	m_dataFolder = dataFolder;
	//load shader for basecolor
	auto pbr_program = loadProgramFile(m_dataFolder+"pbr_vs.glsl", m_dataFolder + "pbr_fs.glsl");
	m_programs["pbr_ibl"] = pbr_program;

}

void MaterialPBR::apply(Utility::ArcballCamera* camera)
{
	auto& prog = m_programs["pbr_ibl"];
	glUseProgram(prog);

	glBindTextureUnit(0, m_textures["pbr_a"].id);
	glBindTextureUnit(1, m_textures["pbr_n"].id);
	glBindTextureUnit(2, m_textures["pbr_m"].id);
	glBindTextureUnit(3, m_textures["pbr_r"].id);
	glBindTextureUnit(4, env_texture_id);
	glBindTextureUnit(5, irmap_texture_id);
	glBindTextureUnit(6, sp_BRDF_LUT_id);

	GLint location = glGetUniformLocation(prog, "diffuse");
	if (location != -1) {
		glUniform4fv(location, 1, &m_diffColor.x);
	}

	location = glGetUniformLocation(prog, "usetexture");
	if (location != -1) {
		glUniform1i(location, m_useTexture);
	}

	location = glGetUniformLocation(prog, "usesss");
	if (location != -1) {
		glUniform1i(location, m_useSSS);
	}

	location = glGetUniformLocation(prog, "cam_pos");
	if (location != -1) {		
		glUniform3fv(location, 1, &camera->eye.x);
	}

	location = glGetUniformLocation(prog, "material");
	if (location != -1) {
		glUniform4fv(location, 1, &m_material.x);
	}
}

uint32_t MaterialPBR::getDefaultProgram()
{
	return m_programs["pbr_ibl"];
}

void MaterialPBR::setTextures(const std::string& abeldo, const std::string& normal, const std::string& metallic, const std::string& roughness)
{
	auto albedoTexture = Renderer::createTexture(::Image::fromFile(abeldo, 3), GL_RGB, GL_SRGB8);
	auto normalTexture = Renderer::createTexture(::Image::fromFile(normal, 3), GL_RGB, GL_RGB8);
	auto metalnessTexture = Renderer::createTexture(::Image::fromFile(metallic, 1), GL_RED, GL_R8);
	auto roughnessTexture = Renderer::createTexture(::Image::fromFile(roughness, 1), GL_RED, GL_R8);

	m_textures["pbr_a"] = albedoTexture;
	m_textures["pbr_n"] = normalTexture;
	m_textures["pbr_m"] = metalnessTexture;
	m_textures["pbr_r"] = roughnessTexture;
	m_useTexture = 1;
}

MaterialPhong::MaterialPhong(std::string dataFolder)
{
	m_dataFolder = dataFolder;
	auto phong_program = loadProgram(m_dataFolder + "phong.glsl");
	m_programs["phong"] = phong_program;
}

void MaterialPhong::apply(Utility::ArcballCamera* camera)
{
	auto& prog = m_programs["phong"];
	glUseProgram(prog);

	GLint eye_location = glGetUniformLocation(prog, "eye");
	if (eye_location != -1) {
		glUniform3fv(eye_location, 1, &camera->eye.x);
	}

	GLint color_location = glGetUniformLocation(prog, "diffuse");
	if (color_location != -1) {
		glUniform4fv(color_location, 1, &m_diffColor.x);
	}
}

uint32_t MaterialPhong::getDefaultProgram()
{
	return m_programs["phong"];
}

MaterialMatcap::MaterialMatcap()
{
	//load shader for basecolor
	auto pbr_program = loadProgram(m_dataFolder + "matcap.glsl");
	m_programs["matcap"] = pbr_program;
	m_diffColor = glm::vec4(1, 0, 0,1);
}

void MaterialMatcap::setTextures(const std::string& abeldo) {
	auto albedoTexture = Renderer::createTexture(::Image::fromFile(abeldo, 3), GL_RGB, GL_SRGB8);
	m_textures["matcap"] = albedoTexture;
}

void MaterialMatcap::apply(Utility::ArcballCamera* camera)
{
	auto& prog = m_programs["matcap"];
	glUseProgram(prog);

	glBindTextureUnit(0, m_textures["matcap"].id);

	GLint view_location = glGetUniformLocation(prog, "view");
	if (view_location != -1) {
		auto view = camera->getView();
		glUniformMatrix4fv(view_location, 1,false, &view[0][0]);
	}

	GLint color_location = glGetUniformLocation(prog, "diffuse");
	if (color_location != -1) {
		glUniform4fv(color_location, 1, &m_diffColor.x);
	}

	GLint usetex_location = glGetUniformLocation(prog, "use_tex");
	if (usetex_location != -1) {
		glUniform1i(usetex_location, 1);
	}

}

uint32_t MaterialMatcap::getDefaultProgram()
{
	return m_programs["matcap"];
}
