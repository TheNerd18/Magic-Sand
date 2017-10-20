//Author: Bendik Hillestad, 131333
//EXAM 2014

#include "Mesh.h"
#include "Shader.h"

using namespace glm;

//Prototype declaration
vec3 ComputeNormal(vec3 f1, vec3 f2, vec3 f3);

Mesh::Mesh()
{
	this->VBO			= 0;
	this->VBO2			= 0;
	this->IBO			= 0;
	this->rawVertices	= NULL;
	this->rawNormals	= NULL;
	this->rawIndices	= NULL;
	this->vertexCount	= 0;
	this->indexCount	= 0;
}

Mesh::Mesh(const Mesh &other)
{
	//NO
}

Mesh::~Mesh()
{
	this->Dispose();
}

void Mesh::UpdateVertices(GLfloat* const &vertices, const GLuint &vertexCount)
{
	//Resize/Allocate raw buffer if needed
	if (this->rawVertices && this->vertexCount != vertexCount)
	{
		delete[] this->rawVertices;
		this->rawVertices = new GLfloat[vertexCount * 3];
	}
	else if (this->rawVertices == NULL) this->rawVertices = new GLfloat[vertexCount * 3];

	//Copy vertices
	for (GLuint i = 0; i < vertexCount * 3; i++) this->rawVertices[i] = vertices[i];

	//Prepare VBO
	if (this->VBO == 0) glGenBuffers(1, &this->VBO);
	glBindBuffer(GL_ARRAY_BUFFER, this->VBO);

	//Update buffer data
	if (this->vertexCount != vertexCount)
	{
		//Need to resize/allocate the buffer
		glBufferData(GL_ARRAY_BUFFER, vertexCount * 3 * sizeof(GLfloat), this->rawVertices, GL_DYNAMIC_DRAW);
	}
	else
	{
		//Fast copy
		GLfloat* data = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

		for (GLuint i = 0; i < vertexCount * 3; i++) data[i] = this->rawVertices[i];

		//Push
		glUnmapBuffer(GL_ARRAY_BUFFER);
		data = NULL;
	}
	
	//Update vertex count
	this->vertexCount = vertexCount;
}

void Mesh::UpdateIndices(GLuint* const &indices, const GLuint &indexCount)
{
	//Resize/Allocate raw buffer if needed
	if (this->rawIndices && this->indexCount != indexCount)
	{
		delete[] this->rawIndices;
		this->rawIndices = new GLuint[indexCount];
	}
	else if (this->rawIndices == NULL) this->rawIndices = new GLuint[indexCount];

	//Copy indices
	for (GLuint i = 0; i < indexCount; i++) this->rawIndices[i] = indices[i];

	//Prepare IBO
	if (this->IBO == 0) glGenBuffers(1, &this->IBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->IBO);

	//Update buffer data
	if (this->indexCount != indexCount)
	{
		//Need to resize/allocate the buffer
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(GLuint), this->rawIndices, GL_DYNAMIC_DRAW);
	}
	else
	{
		//Fast copy
		GLuint* data = (GLuint*)glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);

		for (GLuint i = 0; i < indexCount; i++) data[i] = this->rawIndices[i];

		//Push
		glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
		data = NULL;
	}

	//Update index count
	this->indexCount = indexCount;
}

void Mesh::ComputeNormals()
{
	//Resize/Allocate raw buffer if needed
	if (this->rawNormals && this->normalCount != this->vertexCount)
	{
		delete[] this->rawNormals;
		this->rawNormals = new GLfloat[this->vertexCount * 3];
	}
	else if (this->rawNormals == NULL) this->rawNormals = new GLfloat[this->vertexCount * 3];

	//Initialise normals to zeroes
	for (GLuint i = 0; i < this->vertexCount * 3; i++) this->rawNormals[i] = 0.0f;

	//Loop through indices
	for (GLuint i = 0; i < this->indexCount / 3; i++)
	{
		//Get indices
		GLuint	a = this->rawIndices[i * 3],
				b = this->rawIndices[i * 3 + 1],
				c = this->rawIndices[i * 3 + 2];

		//Get vertices
		vec3 A = vec3(this->rawVertices[a * 3], this->rawVertices[a * 3 + 1], this->rawVertices[a * 3 + 2]);
		vec3 B = vec3(this->rawVertices[b * 3], this->rawVertices[b * 3 + 1], this->rawVertices[b * 3 + 2]);
		vec3 C = vec3(this->rawVertices[c * 3], this->rawVertices[c * 3 + 1], this->rawVertices[c * 3 + 2]);

		//Compute face normal
		vec3 norm = ComputeNormal(A, B, C);

		//Add to normals
		this->rawNormals[a * 3]		+= norm.x;
		this->rawNormals[a * 3 + 1] += norm.y;
		this->rawNormals[a * 3 + 2] += norm.z;

		this->rawNormals[b * 3]		+= norm.x;
		this->rawNormals[b * 3 + 1] += norm.y;
		this->rawNormals[b * 3 + 2] += norm.z;

		this->rawNormals[c * 3]		+= norm.x;
		this->rawNormals[c * 3 + 1] += norm.y;
		this->rawNormals[c * 3 + 2] += norm.z;
	}

	//Average out normals
	for (GLuint i = 0; i < vertexCount; i++)
	{
		//Get normal
		vec3 norm = vec3(this->rawNormals[i * 3], this->rawNormals[i * 3 + 1], this->rawNormals[i * 3 + 2]);

		//Normalise
		norm = normalize(norm);

		//Output
		this->rawNormals[i * 3]		= norm.x;
		this->rawNormals[i * 3 + 1] = norm.y;
		this->rawNormals[i * 3 + 2] = norm.z;
	}

	//Prepare VBO2
	if (this->VBO2 == 0) glGenBuffers(1, &this->VBO2);
	glBindBuffer(GL_ARRAY_BUFFER, this->VBO2);

	//Update buffer data
	if (this->normalCount != this->vertexCount)
	{
		//Need to resize/allocate the buffer
		glBufferData(GL_ARRAY_BUFFER, this->vertexCount * 3 * sizeof(GLfloat), this->rawNormals, GL_DYNAMIC_DRAW);
	}
	else
	{
		//Fast copy
		GLfloat* data = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

		for (GLuint i = 0; i < this->vertexCount * 3; i++) data[i] = this->rawNormals[i];

		//Push
		glUnmapBuffer(GL_ARRAY_BUFFER);
		data = NULL;
	}

	//Update normal count
	this->normalCount = this->vertexCount;
}

void Mesh::Render(Shader* shader) const
{
	//Bind vertex buffer
	glBindBuffer				(GL_ARRAY_BUFFER, this->VBO);
	glVertexAttribPointer		(shader->GetAttributeLocation("vertexPosition"), 3, GL_FLOAT, GL_FALSE, 0, NULL);
	glEnableVertexAttribArray	(shader->GetAttributeLocation("vertexPosition"));

	//Bind normal buffer
	glBindBuffer				(GL_ARRAY_BUFFER, this->VBO2);
	glVertexAttribPointer		(shader->GetAttributeLocation("normal"), 3, GL_FLOAT, GL_FALSE, 0, NULL);
	glEnableVertexAttribArray	(shader->GetAttributeLocation("normal"));

	//Bind index buffer
	glBindBuffer				(GL_ELEMENT_ARRAY_BUFFER, this->IBO);

	//Draw triangles
	glDrawElements				(GL_TRIANGLES, this->indexCount, GL_UNSIGNED_INT, NULL);

	//Disable vertex attributes
	glDisableVertexAttribArray	(shader->GetAttributeLocation("vertexPosition"));
	glDisableVertexAttribArray	(shader->GetAttributeLocation("normal"));
}

void Mesh::Dispose()
{
	//Free buffers
	if (this->VBO)
	{
		glDeleteBuffers(1, &this->VBO);
		this->VBO = NULL;
	}
	if (this->VBO2)
	{
		glDeleteBuffers(1, &this->VBO2);
		this->VBO2 = NULL;
	}
	if (this->IBO)
	{
		glDeleteBuffers(1, &this->IBO);
		this->IBO = NULL;
	}

	if (this->rawVertices)
	{
		delete[] this->rawVertices;
		this->rawVertices = NULL;
	}
	if (this->rawNormals)
	{
		delete[] this->rawNormals;
		this->rawNormals = NULL;
	}
	if (this->rawIndices)
	{
		delete[] this->rawIndices;
		this->rawIndices = NULL;
	}

	this->vertexCount	= 0;
	this->normalCount	= 0;
	this->indexCount	= 0;
}

//Computes the weighted normal based on angle between the vertex f1 and its two adjacent vertices
vec3 ComputeNormal(vec3 f1, vec3 f2, vec3 f3)
{
	vec3	norm	= cross(f3 - f1, f2 - f1);
	GLfloat	weight	= acos(dot((f3 - f1), (f2 - f1)) / (length(f3 - f1) * length(f2 - f1)));

	return normalize(norm) * weight;
}