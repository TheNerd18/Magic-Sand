//Author: Bendik Hillestad, 131333
//EXAM 2014

#include "Terrain.h"
#include "Shader.h"
#include "Mesh.h"
#include "Perlin.h"

#include <stdio.h>
#include <fstream>
#include <string>
#include <vector>

using namespace glm;
using namespace std;

Terrain::Terrain()
{
	this->mesh			= NULL;
	this->heightMap		= NULL;
	this->mapWidth		= 0;
	this->mapHeight		= 0;
	this->maxHeight		= 0.0f;
	this->fullUpdate	= 0;
}

Terrain::Terrain(const Terrain &other)
{
	//NO
}

Terrain::~Terrain()
{
	this->Dispose();
}

GLboolean Terrain::LoadHeightMap(const char* path)
{
	printf("#Info: Attempting to open file %s\n", path);
	ifstream file(path, ios::in);

	//Check if successful
	if (file.good())
	{
		//Read magic number
		string magic;
		file >> magic;

		//Check if correct magic number
		if (magic != "P2")
		{
			printf("#ERROR: %s is not a valid pnm file!\n", path);

			return GL_FALSE;
		}

		//Read width, height and max value
		GLuint maxVal;
		file	>> this->mapWidth
				>> this->mapHeight
				>> maxVal;

		this->maxHeight = 0.0f;

		if (maxVal > 65535)
		{
			printf("#ERROR: Invalid max value in header!\n");

			return GL_FALSE;
		}

		//Prepare buffer
		if (this->heightMap) delete[] this->heightMap;
		this->heightMap = new GLfloat[this->mapWidth * this->mapHeight];
		if (!this->heightMap)
		{
			printf("#ERROR: Failed to allocate memory for height map!\n");

			return GL_FALSE;
		}

		//Read values
		for (GLuint y = 0; y < this->mapHeight; y++)
		{
			for (GLuint x = 0 ; x < this->mapWidth; x++)
			{
				//Read value
				GLuint val;
				file >> val;
				if (val > this->maxHeight) this->maxHeight = val;

				//Insert into height map
				this->heightMap[y * this->mapWidth + x] = val;
			}
		}

		this->fullUpdate = GL_TRUE;

		printf("#Info: Loaded successfully!\n");

		return GL_TRUE;
	}
	else
	{
		printf("#ERROR: Unable to open file %s!\n", path);

		return GL_FALSE;
	}
}

GLboolean Terrain::LoadGeoHeightMap(const char* path)
{
	printf("#Info: Attempting to open file %s\n", path);
	ifstream file(path, ios::in);

	//Check if successful
	if (file.good())
	{
		string blabla;

		//Read width and height
		file >> blabla >> this->mapWidth
			 >> blabla >> this->mapHeight;

		this->maxHeight = 0.0f;

		//Read GPS data (Unused)
		double xllcorner, yllcorner, dx, dy;
		file >> blabla >> xllcorner
			 >> blabla >> yllcorner
			 >> blabla >> dx
			 >> blabla >> dy;

		//Read NODATA_value
		int16 nodata;
		file >> blabla >> nodata;

		//Prepare buffer
		if (this->heightMap) delete[] this->heightMap;
		this->heightMap = new GLfloat[this->mapWidth * this->mapHeight];
		if (!this->heightMap)
		{
			printf("#ERROR: Failed to allocate memory for height map!\n");

			return GL_FALSE;
		}

		//Read values
		for (GLuint y = 0; y < this->mapHeight; y++)
		{
			for (GLuint x = 0; x < this->mapWidth; x++)
			{
				//Read value
				int16 val;
				file >> val;
				if (val > this->maxHeight) this->maxHeight = val;

				//Insert into height map
				this->heightMap[y * this->mapWidth + x] = val;
			}
		}

		this->fullUpdate = GL_TRUE;

		printf("#Info: Loaded successfully!\n");

		return GL_TRUE;
	}
	else
	{
		printf("#ERROR: Unable to open file %s!\n", path);

		return GL_FALSE;
	}
}

GLboolean Terrain::LoadHeightMapFromPerlinNoise(GLuint width, GLuint height, GLfloat zoom, GLfloat p)
{
	printf("#Info: Generating random height map\n");

	this->mapWidth	= width;
	this->mapHeight = height;
	this->maxHeight = 0.0f;

	//Prepare buffer
	if (this->heightMap) delete[] this->heightMap;
	this->heightMap = new GLfloat[this->mapWidth * this->mapHeight];
	if (!this->heightMap)
	{
		printf("#ERROR: Failed to allocate memory for height map!\n");

		return GL_FALSE;
	}

	//Generate values
	for (GLuint y = 0; y < this->mapHeight; y++)
	{
		for (GLuint x = 0; x < this->mapWidth; x++)
		{
			GLfloat height = PerlinNoise(x, y, zoom, p);
			if (height > this->maxHeight) this->maxHeight = height;

			this->heightMap[y * this->mapWidth + x] = height;
		}
	}

	this->fullUpdate = GL_TRUE;

	printf("#Info: Generated map successfully!\n");

	return GL_TRUE;
}

GLboolean Terrain::ComputeTerrain(const GLfloat &waterLevel, const GLfloat &zMod)
{
	printf("#Info: Computing terrain\n");

	//Check if height map is set
	if (!this->heightMap)
	{
		printf("#ERROR: Need to load or generate a height map first!\n");

		return GL_FALSE;
	}

	//Prepare mesh
	if (!this->mesh)
	{
		this->mesh = new Mesh();
		if (!this->mesh)
		{
			printf("#ERROR: Failed to allocate mesh object!\n");

			return GL_FALSE;
		}
	}

	//Prepare temporary vertex buffer
	GLfloat* vertices = new GLfloat[this->mapWidth * this->mapHeight * 3];
	if (!vertices)
	{
		printf("#ERROR: Failed to allocate memory for temporary vertex buffer!\n");

		return GL_FALSE;
	}

	//Read in vertices
	for (GLuint y = 0; y < this->mapHeight; y++)
	{
		for (GLuint x = 0; x < this->mapWidth; x++)
		{
			//Compute height
			GLfloat height = max(heightMap[y * this->mapWidth + x] - waterLevel * this->maxHeight, 0.0f) * (zMod / this->maxHeight);

			//Insert vertex
			vertices[y * this->mapWidth * 3 + x * 3]		= x - this->mapWidth / 2.0f;
			vertices[y * this->mapWidth * 3 + x * 3 + 1]	= height;
			vertices[y * this->mapWidth * 3 + x * 3 + 2]	= y - this->mapHeight / 2.0f;
		}
	}

	//Push to mesh
	this->mesh->UpdateVertices(vertices, this->mapWidth * this->mapHeight);

	//Clean up buffer
	delete[] vertices;
	vertices = NULL;

	//Check if indices need to be updated as well
	if (this->fullUpdate)
	{
		//Prepare temporary index buffer
		GLuint* indices = new GLuint[(this->mapWidth - 1) * (this->mapHeight - 1) * 6];
		if (!indices)
		{
			printf("#ERROR: Failed to allocate memory for temporary index buffer!\n");

			return GL_FALSE;
		}

		//Set up indices
		GLuint ind = 0;
		for (GLuint y = 0; y < this->mapHeight - 1; y++)
		{
			for (GLuint x = 0; x < this->mapWidth - 1; x++)
			{
				//First half
				indices[ind++] = y			* this->mapWidth + x;
				indices[ind++] = y			* this->mapWidth + x + 1;
				indices[ind++] = (y + 1)	* this->mapWidth + x;

				//Second half
				indices[ind++] = (y + 1)	* this->mapWidth + x;
				indices[ind++] = y			* this->mapWidth + x + 1;
				indices[ind++] = (y + 1)	* this->mapWidth + x + 1;
			}
		}

		//Push to mesh
		this->mesh->UpdateIndices(indices, (this->mapWidth - 1) * (this->mapHeight - 1) * 6);

		//Clean up buffer
		delete indices;
		indices = NULL;

		//Unflag fullUpdate
		this->fullUpdate = GL_FALSE;
	}

	//Compute normals
	this->mesh->ComputeNormals();

	//Return success
	printf("#Info: Terrain computed successfully!\n");

	return GL_TRUE;
}

void Terrain::Render(Shader* shader) const
{
	if (this->mesh)
	{
		this->mesh->Render(shader);
	}
}

GLfloat Terrain::SampleHeight(const GLfloat &x, const GLfloat &y) const
{
	GLuint px = (GLuint)clamp((x + this->mapWidth / 2.0f), 0.0f, this->mapWidth - 1.0f);
	GLuint py = (GLuint)clamp((y + this->mapHeight / 2.0f), 0.0f, this->mapHeight - 1.0f);

	return this->heightMap[py * this->mapWidth + px] / this->maxHeight;
}

void Terrain::Dispose()
{
	//Dispose of mesh
	if (this->mesh)
	{
		this->mesh->Dispose();
		this->mesh = NULL;
	}
	//Dispose of height map
	if (this->heightMap)
	{
		delete[] this->heightMap;
		this->heightMap = NULL;
	}

	this->mapWidth		= 0;
	this->mapHeight		= 0;
	this->maxHeight		= 0.0f;
	this->fullUpdate	= 0;
}