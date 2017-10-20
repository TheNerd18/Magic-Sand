#version 400 core

//Author: Bendik Hillestad, 131333
//EXAM 2014

//Feel free to change to #version 330 core
//and add layout locations to the attributes below if your PC supports it

in vec3 vertexPosition;
in vec3 normal;

uniform mat4 View;

out float	height;
out vec3	worldNormal;

void main()
{
	gl_Position	= (View * vec4(vertexPosition, 1));

	height 		= vertexPosition.y;
	worldNormal = normal;
}