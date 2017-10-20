//Author: Bendik Hillestad, 131333
//EXAM 2014

#include "Shader.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

Shader::Shader()
{
	uniforms	= new map<const char*, GLint>();
	attributes	= new map<const char*, GLint>();
	program		= 0;
}

Shader::Shader(const Shader &other)
{
	//NO
}

Shader::~Shader()
{
	this->Dispose();
}

bool Shader::LoadShaders(string vs, string ps)
{
	GLuint	vertexShaderID	= 0;
	GLuint	pixelShaderID	= 0;
	GLint	result			= GL_TRUE;

	//Load vertex shader
	if (!this->LoadVertexShader(vs, vertexShaderID))
	{
		printf("#ERROR: Failed to load %s!\n", vs.c_str());

		result = GL_FALSE;
	}

	//Load pixel shader
	if (result && !this->LoadPixelShader(ps, pixelShaderID))
	{
		printf("#ERROR: Failed to load %s!\n", ps.c_str());

		result = GL_FALSE;
	}

	//Link the program
	if (result && !this->LinkShader(vertexShaderID, pixelShaderID))
	{
		printf("#ERROR: Failed to link the program!\n");

		result = GL_FALSE;
	}

	if (result) printf("#Info: Loaded shaders successfully!\n\n");

	//Cleanup
	if (vertexShaderID) glDeleteShader(vertexShaderID);
	if (pixelShaderID)	glDeleteShader(pixelShaderID);
	if (!result)		this->Dispose();

	return (result == GL_TRUE);
}

GLuint Shader::GetProgramID() const
{
	return this->program;
}

void Shader::Dispose()
{
	if (this->program)
	{
		glDeleteProgram(this->program);
		this->program = 0;
	}
	if (this->uniforms)
	{
		delete this->uniforms;
		this->uniforms = 0;
	}
	if (this->attributes)
	{
		delete this->attributes;
		this->attributes = 0;
	}
}

bool Shader::PreProcess(string &shaderCode)
{
	istringstream ss(shaderCode);
	string line;
	string newCode = "";
	string fileContent = "";
	int lineNr = 1;

	//Read line by line
	while (getline(ss, line))
	{
		//Look for #pragma include()
		auto start = line.find("#pragma include(\"", 0);
		if (start != string::npos)
		{
			//Get the end
			auto end = line.find("\")", start);
			if (end != string::npos && end >= start + 17)
			{
				//Extract path
				string path = line.substr(start + 17, end - (start + 17));

				//Load file
				ifstream fileStream(path, ios::in);
				if (fileStream.is_open())
				{
					string fileLine = "";
					while (getline(fileStream, fileLine)) fileContent += fileLine + '\n';

					fileStream.close();
				}
				else
				{
					printf("#ERROR: Couldn't open %s! Are you in the right directory?", path.c_str());
					getchar();

					return false;
				}

				//Preprocess file
				if (this->PreProcess(fileContent))
				{
					//Insert code into shaderCode
					newCode += fileContent + '\n';
				}
				else
				{
					printf("#ERROR: Error while attempting to process file included on line %i!\n", lineNr);
					getchar();

					return false;
				}
			}
			else
			{
				printf("#ERROR: Syntax error in file on line %i!\n", lineNr);
				getchar();

				return false;
			}
		}
		else newCode += line + '\n';

		lineNr++;
	}

	//Replace shaderCode with the new code
	shaderCode.	clear();			//Not sure if necessary. Too lazy to check at the moment.
	shaderCode.	append(newCode);	//
	newCode.	clear();			//
	fileContent.clear();			//

	return true;
}

bool Shader::LoadVertexShader(string path, GLuint &vs)
{
	GLint	result			= GL_FALSE;
	int		infoLogLength	= 0;

	//Create shader
	vs = glCreateShader(GL_VERTEX_SHADER);

	//Read vertex shader code from file
	printf("#Info: Opening %s\n", path.c_str());
	string		vertexShaderCode = "";
	ifstream	vertexShaderStream(path, ios::in);
	if (vertexShaderStream.is_open())
	{
		string line = "";
		while (getline(vertexShaderStream, line)) vertexShaderCode += line + '\n';

		vertexShaderStream.close();
	}
	else
	{
		printf("#ERROR: Couldn't open %s! Are you in the right directory?", path.c_str());
		getchar();

		return false;
	}

	//Preprocess shader code
	printf("#Info: Preprocessing code\n");
	if (!this->PreProcess(vertexShaderCode))
	{
		printf("#ERROR: Failed to preprocess shader code!\n");
		getchar();

		return false;
	}

	//Compile vertex shader
	printf("#Info: Compiling code\n");
	char const* code = vertexShaderCode.c_str();
	glShaderSource	(vs, 1, &code, NULL);
	glCompileShader	(vs);

	//Check vertex shader
	glGetShaderiv(vs, GL_COMPILE_STATUS, &result);
	glGetShaderiv(vs, GL_INFO_LOG_LENGTH, &infoLogLength);
	if (infoLogLength > 0 && !result)
	{
		std::vector<char> log(infoLogLength + 1);
		glGetShaderInfoLog(vs, infoLogLength, NULL, &log[0]);
		printf("%s\n", &log[0]);
		getchar();

		return false;
	}

	return true;
}

bool Shader::LoadPixelShader(string path, GLuint &ps)
{
	GLint	result			= GL_FALSE;
	int		infoLogLength	= 0;

	//Create shader
	ps = glCreateShader(GL_FRAGMENT_SHADER);

	//Read pixel shader code from file
	printf("#Info: Opening %s\n", path.c_str());
	string		pixelShaderCode = "";
	ifstream	pixelShaderStream(path, ios::in);
	if (pixelShaderStream.is_open())
	{
		string line = "";
		while (getline(pixelShaderStream, line)) pixelShaderCode += line + '\n';

		pixelShaderStream.close();
	}
	else
	{
		printf("#ERROR: Couldn't open %s! Are you in the right directory?", path.c_str());
		getchar();

		return false;
	}

	//Preprocess shader code
	printf("#Info: Preprocessing code\n");
	if (!this->PreProcess(pixelShaderCode))
	{
		printf("#ERROR: Failed to preprocess shader code!\n");
		getchar();

		return false;
	}

	//Compile vertex shader
	printf("#Info: Compiling code\n");
	char const* code = pixelShaderCode.c_str();
	glShaderSource	(ps, 1, &code, NULL);
	glCompileShader	(ps);

	//Check vertex shader
	glGetShaderiv(ps, GL_COMPILE_STATUS, &result);
	glGetShaderiv(ps, GL_INFO_LOG_LENGTH, &infoLogLength);
	if (infoLogLength > 0 && !result)
	{
		std::vector<char> log(infoLogLength + 1);
		glGetShaderInfoLog(ps, infoLogLength, NULL, &log[0]);
		printf("%s\n", &log[0]);
		getchar();

		return false;
	}

	return true;
}

bool Shader::LinkShader(GLuint vs, GLuint ps)
{
	GLint	result			= GL_FALSE;
	int		infoLogLength	= 0;

	printf("#Info: Linking program\n");

	//Create program
	this->program = glCreateProgram();

	//Link program
	glAttachShader	(this->program, vs);
	glAttachShader	(this->program, ps);
	glLinkProgram	(this->program);

	//Check the program
	glGetProgramiv(this->program, GL_LINK_STATUS, &result);
	glGetProgramiv(this->program, GL_INFO_LOG_LENGTH, &infoLogLength);
	if (infoLogLength > 0 && !result)
	{
		std::vector<char> log(infoLogLength + 1);
		glGetProgramInfoLog(this->program, infoLogLength, NULL, &log[0]);
		printf("%s\n", &log[0]);
		getchar();

		return false;
	}

	return true;
}

//TODO: Profile to see if worth it
GLint Shader::GetUniformLocation(const char* uniform)
{
	//Check cache for uniform location
	auto it = this->uniforms->find(uniform);

	//If found return location
	if (it != this->uniforms->end()) return (*it).second;
	else
	{
		//Attempt to find uniform location in program
		GLint loc = glGetUniformLocation(this->program, uniform);

		//If valid insert into cache
		if (loc != -1) this->uniforms->insert(pair<const char*, GLint>(uniform, loc));
		else printf("#ERROR: Uniform %s does not exist!\n", uniform);

		//Return location
		return loc;
	}
}

GLint Shader::GetAttributeLocation(const char* attribute)
{
	//Check cache for attrubute location
	auto it = this->attributes->find(attribute);

	//If found return location
	if (it != this->attributes->end()) return (*it).second;
	else
	{
		//Attempt to find attribute location in program
		GLint loc = glGetAttribLocation(this->program, attribute);

		//If valid insert into cache
		if (loc != -1) this->attributes->insert(pair<const char*, GLint>(attribute, loc));
		else printf("#ERROR: Attribute %s does not exist!\n", attribute);

		//Return location
		return loc;
	}
}