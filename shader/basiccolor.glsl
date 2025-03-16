vertex:
#version 450 core
uniform mat4 view;
uniform mat4 proj;
uniform mat4 model;

layout(location=0) in vec3 position;
layout(location = 1) in vec3 texcoord;
//layout(location=0) out vec3 localPosition;
//layout(location = 1) out vec2 vs_uv;

void main()
{
	//vs_uv = texcoord;
	gl_Position   = proj * view * model * vec4(position, 1.0);
}

fragment:
#version 450 core
//in vec2 vs_uv;
out vec4 out_color;

uniform vec3 color;


void main()
{
	out_color = vec4(color,1.0);
}