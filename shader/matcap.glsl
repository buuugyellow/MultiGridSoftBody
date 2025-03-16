vertex:
#version 450
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;


layout(std140, binding = 0) uniform TransformUniforms
{
	mat4 mvp;
	mat4 sky_mvp;
	vec4 eyepos;
};

uniform mat4 model;

out vec3 fg_normal;
void main()
{
	vec4 world_pos = model * vec4(position,1.0);

	fg_normal = mat3(model) * normal;
	gl_Position = mvp * world_pos;
}

fragment:
#version 450
out vec4 FragColor;
in vec3 fg_normal;

uniform vec4 diffuse;
uniform int use_tex;
uniform mat4 view;
layout(binding=0) uniform sampler2D diffuse_map;
void main()
{
	if(use_tex==0)
	    FragColor = diffuse;
	else
	{
		vec2 muv = vec2(view * vec4(normalize(fg_normal), 0))*0.5+vec2(0.5,0.5);
		vec3 color = texture(diffuse_map, muv).xyz;
		float gray = color.r * 0.2126 + color.g * 0.7152 + color.b * 0.0722;
		//gray = pow(gray, 1./2.2);
		gray = pow(gray, 1./1.5);
		FragColor = vec4(gray*diffuse.xyz, diffuse.a);
	}
}
