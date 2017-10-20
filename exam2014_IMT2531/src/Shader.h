//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma once

#include <string>
#include <map>

#define GLEW_STATIC //Use static version
#include "D:\git\openFrameworks\libs\glew\include\GL\glew.h"

class Shader
{
public:
	Shader	();
	Shader	(const Shader&);
	~Shader	();

	bool	LoadShaders			(std::string vs, std::string ps);

	GLuint	GetProgramID		() const;

	GLint	GetUniformLocation	(const char* uniform);
	GLint	GetAttributeLocation(const char* attribute);

	void	Dispose				();
	
private:
	bool	PreProcess			(std::string &shaderCode);

	bool	LoadVertexShader	(std::string path,	GLuint &vs);
	bool	LoadPixelShader		(std::string path,	GLuint &ps);
	bool	LinkShader			(GLuint vs,			GLuint ps);

	std::map<const char*, GLint>*	uniforms;
	std::map<const char*, GLint>*	attributes;
	GLuint							program;
};