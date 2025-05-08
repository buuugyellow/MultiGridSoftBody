vertex:
#version 450 core
uniform mat4 Model;
uniform mat4 ModelViewMatrix;
uniform float pointRadius;  // point size in world space
uniform float pointScale;   // scale to calculate size in pixels


layout(std140, binding=0) uniform TransformUniforms
{
	mat4 mvp;
	mat4 sky_mvp;
	vec4 eye;
};

uniform vec3 colors[16];
layout(location = 0) in vec3 aPos;
layout(location = 1) in int phase;
layout(location = 0) out Vertex
{
	vec3 reflectance;
	vec3 viewpos;
} vout;

void main()
{
    gl_Position = mvp * vec4(aPos, 1.0);
    vec4 viewPos = ModelViewMatrix * Model * vec4(aPos, 1.0);
	gl_PointSize = -pointScale * (pointRadius / viewPos.z);
	vout.viewpos = viewPos.xyz;
	vout.reflectance = mix(colors[phase % 8] * 2.0, vec3(1.0), 0.1);
}

fragment:
#version 450 core
layout(location = 0) in Vertex
{
	vec3 reflectance;
	vec3 viewpos;
} vin;

uniform mat4 ProjectionMatrix;
uniform vec3 lightDir;

uniform float pointRadius;  // point size in world space

layout(location = 0) out vec4 out_color;

float sqr(float x) { return x * x; }

void main()
{
	// calculate normal from texture coordinates
	vec3 normal;
	normal.xy = gl_PointCoord.xy * vec2(2.0, -2.0) + vec2(-1.0, 1.0);
	float mag = dot(normal.xy, normal.xy);
	if (mag > 1.0) discard;   // kill pixels outside circle
	normal.z = sqrt(1.0 - mag);

	vec3 Lo = vin.reflectance * max(0.0, sqr(-dot(lightDir, normal) * 0.5 + 0.5));

	out_color = vec4(pow(Lo, vec3(1.0 / 2.2)), 1.0);

	vec3 eyePos = vin.viewpos + normal * pointRadius;
	vec4 ndcPos =ProjectionMatrix * vec4(eyePos, 1.0);
	ndcPos.z /= ndcPos.w;
	gl_FragDepth = ndcPos.z * 0.5 + 0.5;
}
