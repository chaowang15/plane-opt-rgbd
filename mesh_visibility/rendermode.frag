#version 330 core
in vec3 fragment_color;
flat in int vertex_id;
in vec2 uv;

// Ouput data
layout(location = 0) out vec3 color;
layout(location = 1) out vec3 depth_and_vid;

uniform bool flag_show_color;
uniform bool flag_show_texture;
uniform sampler2D texture_sampler;

//uniform float near = 0.1;
//uniform float far = 10.0;
uniform float near; // set the two parameters from outside
uniform float far;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near * far) / (far + near - z * (far - near));	
}

void main()
{
	// Output color = red 
	//color = vec3(1,0,0);
	if (flag_show_color)
	{
		color = fragment_color; // Simply pass the color data
	}
	else if (flag_show_texture)
	{
		color = texture( texture_sampler, uv ).rgb; // texture color
	}
	else
	{// render depth (linearized depth will be rendered more clearly)
		float depth = LinearizeDepth(gl_FragCoord.z) / far;
		color = vec3(depth);
	}	
	depth_and_vid = vec3(gl_FragCoord.z / gl_FragCoord.w, vertex_id, 0);
}