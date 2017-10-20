//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma once

#include "Common.h"

//Forward-declaring objects
class Mesh;
class Shader;

class Terrain
{
public:
	Terrain		();
	Terrain		(const Terrain&);
	~Terrain	();

	GLboolean	LoadHeightMap				(const char* path);											//Doesn't actually create the terrain, just prepares for it
	GLboolean	LoadGeoHeightMap			(const char* path);											//
	GLboolean	LoadHeightMapFromPerlinNoise(GLuint width, GLuint height, GLfloat zoom, GLfloat p);		//

	GLboolean	ComputeTerrain				(const GLfloat &waterLevel, const GLfloat &zMod);			//Computes the terrain based on the heightmap and input parameters
	void		Render						(Shader* shader) const;										//Renders the terrain

	GLfloat		SampleHeight				(const GLfloat &x, const GLfloat &y) const;					//Samples the heightmap

	void		Dispose						();															//Cleans up all internal objects, making the object invalid until LoadHeightMap* and ComputeTerrain are called again

private:
	Mesh*		mesh;

	GLfloat*	heightMap;
	GLuint		mapWidth;
	GLuint		mapHeight;
	GLfloat		maxHeight;

	GLboolean	fullUpdate;
};