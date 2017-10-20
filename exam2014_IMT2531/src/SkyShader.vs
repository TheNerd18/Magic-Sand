#version 400 core

//Author: Bendik Hillestad, 131333
//EXAM 2014

//Feel free to change to #version 330 core
//and add a layout location to the attribute below if your PC supports it

in vec3 vertexPosition;

out vec3 worldPos;

uniform mat4 View;
uniform vec3 CameraPos;

void main()
{
	gl_Position = (View * vec4(vertexPosition + CameraPos, 1)).xyww;

	worldPos = vertexPosition;
}