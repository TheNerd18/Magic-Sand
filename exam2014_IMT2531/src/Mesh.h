//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma once

#include "Common.h"

//Forward-declaring Shader object
class Shader;

class Mesh
{
public:
	Mesh	();
	Mesh	(const Mesh&);
	~Mesh	();

	void UpdateVertices	(GLfloat* const &vertices,	const GLuint &vertexCount);
	void UpdateIndices	(GLuint* const &indices, const GLuint &indexCount);
	void ComputeNormals	();

	void Render			(Shader* shader) const;

	void Dispose		();

private:
	GLuint		VBO;
	GLuint		VBO2;
	GLuint		IBO;

	GLfloat*	rawVertices;
	GLfloat*	rawNormals;
	GLuint*		rawIndices;

	GLuint		vertexCount;
	GLuint		normalCount;
	GLuint		indexCount;
};