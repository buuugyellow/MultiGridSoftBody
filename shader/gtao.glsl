vertex:
#version 450 core

layout(location=0) out vec2 screenPosition;

layout(std140, binding = 0) uniform TransformUniforms
{
	mat4 mvp;
	mat4 sky_mvp;
	vec4 eyepos;
};

void main()
{
	if(gl_VertexID == 0) {
		screenPosition = vec2(1.0, 2.0);
		gl_Position = vec4(1.0, 3.0, 0.0, 1.0);
	}
	else if(gl_VertexID == 1) {
		screenPosition = vec2(-1.0, 0.0);
		gl_Position = vec4(-3.0, -1.0, 0.0, 1.0);
	}
	else /* if(gl_VertexID == 2) */ {
		screenPosition = vec2(1.0, 0.0);
		gl_Position = vec4(1.0, -1.0, 0.0, 1.0);
	}
}

fragment:
#version 450

#define PI				3.1415926535897932
#define TWO_PI			6.2831853071795864
#define HALF_PI			1.5707963267948966
#define ONE_OVER_PI		0.3183098861837906

#define NUM_DIRECTIONS	16
#define NUM_STEPS		8
#define RADIUS			1.0		// in world space

layout(location=0) in  vec2 screenPosition;

layout(binding=0) uniform sampler2D gbufferNormals;
layout(binding=1) uniform sampler2D gbufferDepth;
//layout(binding=2) uniform sampler2D noise;

uniform mat4 view;
uniform vec4 projInfo;
uniform vec3 clipInfo;

out vec4 my_FragColor0;

vec4 GetViewPosition(vec2 uv, float currstep)
{
	vec2 basesize = vec2(textureSize(gbufferDepth, 0));
	vec2 coord = (uv / basesize);

	float d = texture(gbufferDepth, coord).r;
	if(d==0.0)
		d = 1.0;
	vec4 ret = vec4(0.0, 0.0, 0.0, d);

	ret.z = clipInfo.x + d * (clipInfo.y - clipInfo.x);
	ret.xy = (uv * projInfo.xy + projInfo.zw) * ret.z;

	return ret;
}

void main()
{
    // my_FragColor0 = vec4(1,0,0,1);
    // return;
    vec2 uv_wh = screenPosition * vec2(textureSize(gbufferDepth,0));
	ivec2 loc = ivec2(uv_wh);
	vec4 vpos = GetViewPosition(uv_wh, 1.0);

	//float d = texture(gbufferDepth, screenPosition).r;
    //my_FragColor0 = vec4(screenPosition,d,1);
    //return;
	if (vpos.w == 1.0) {
		my_FragColor0 = vec4(1,1,1,1);//0.0103, 0.0707, 1.0);
		return;
	}
	
	vec4 s;
	vec3 vnorm	= texelFetch(gbufferNormals, loc, 0).rgb;
	vnorm = mat3(view) * vnorm;
	//vnorm = vnorm*2.0 - vec3(1.0);
	vec3 vdir	= normalize(-vpos.xyz);
	vec3 dir, ws;

    

	// calculation uses left handed system
	vnorm.z = -vnorm.z;

	//my_FragColor0 = vec4(vnorm,1);
    //return;

	vec2 offset;
	vec2 horizons = vec2(-1.0, -1.0);

	float radius = (RADIUS * clipInfo.z) / vpos.z;
	radius = max(NUM_STEPS, radius);

    //vec2 noisecolor = texelFetch(noise, ivec2(loc.x % 4,loc.y % 4), 0).rg;

	float stepsize	= radius / NUM_STEPS;
	float phi		= 0.0;//noisecolor.x * PI;
	float ao		= 0.0;
    float division = 0;//noisecolor.y * stepsize;
	float currstep	= 1.0 + division + 0.25 * stepsize;// * params.y;
	float dist2, invdist, falloff, cosh;

	for (int k = 0; k < NUM_DIRECTIONS; ++k) {
		phi = float(k) * (PI / NUM_DIRECTIONS);
		currstep = 1.0;

		dir = vec3(cos(phi), sin(phi), 0.0);
		horizons = vec2(-1.0);

		// calculate horizon angles
		for (int j = 0; j < NUM_STEPS; ++j) {
			offset = round(dir.xy * currstep);

			// h1
			s = GetViewPosition(gl_FragCoord.xy + offset, currstep);
			ws = s.xyz - vpos.xyz;

			dist2 = dot(ws, ws);
			invdist = inversesqrt(dist2);
			cosh = invdist * dot(ws, vdir);

			horizons.x = max(horizons.x, cosh);

			// h2
			s = GetViewPosition(gl_FragCoord.xy - offset, currstep);
			ws = s.xyz - vpos.xyz;

			dist2 = dot(ws, ws);
			invdist = inversesqrt(dist2);
			cosh = invdist * dot(ws, vdir);

			horizons.y = max(horizons.y, cosh);

			// increment
			currstep += stepsize;
		}

		horizons = acos(horizons);

		// calculate gamma
		vec3 bitangent	= normalize(cross(dir, vdir));
		vec3 tangent	= cross(vdir, bitangent);
		vec3 nx			= vnorm - bitangent * dot(vnorm, bitangent);

		float nnx		= length(nx);
		float invnnx	= 1.0 / (nnx + 1e-6);			// to avoid division with zero
		float cosxi		= dot(nx, tangent) * invnnx;	// xi = gamma + HALF_PI
		float gamma		= acos(cosxi) - HALF_PI;
		float cosgamma	= dot(nx, vdir) * invnnx;
		float singamma2	= -2.0 * cosxi;					// cos(x + HALF_PI) = -sin(x)

		// clamp to normal hemisphere
		horizons.x = gamma + max(-horizons.x - gamma, -HALF_PI);
		horizons.y = gamma + min(horizons.y - gamma, HALF_PI);

		// Riemann integral is additive
		ao += nnx * 0.25 * (
			(horizons.x * singamma2 + cosgamma - cos(2.0 * horizons.x - gamma)) +
			(horizons.y * singamma2 + cosgamma - cos(2.0 * horizons.y - gamma)));

	}

	// PDF = 1 / pi and must normalize with pi because of Lambert
	ao = ao / float(NUM_DIRECTIONS);

	my_FragColor0 = vec4(ao, ao, ao, 1.0);
}
