#version 410

in vec3 fragment_color;
flat in int vertex_id;  
in vec2 uv;

layout(location = 0) out vec3 depth;
layout(location = 1) out vec3 color;

uniform bool flag_show_color;
uniform bool flag_show_texture;
uniform sampler2D texture_sampler;		

void main()
{
	depth = vec3(gl_FragCoord.z / gl_FragCoord.w, 0, vertex_id); // channel 0 is depth, channel 2 is vertex index

	//depth = fragment_color;
	//color = fragment_color;
	if (flag_show_color)
	{
		color = fragment_color; // Simply pass the color data
	}
	else if (flag_show_texture)
	{
		color = texture( texture_sampler, uv ).rgb; // texture color
	}
}
