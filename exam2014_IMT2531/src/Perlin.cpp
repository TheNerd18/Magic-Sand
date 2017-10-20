//Author: Bendik Hillestad, 131333
//EXAM 2014

#include "Perlin.h"

#include <algorithm>

using namespace glm;

const GLuint octaves = 4;

//Calculates a pseudo-random number
//using x and y coordinates as seed
GLfloat Rand(GLfloat x, GLfloat y)
{
	GLuint n	= (GLuint)x + (GLuint)y * 57;
	n			= (n << 13) ^ n;
	GLuint nn	= (n * (n*n * 60493U + 19990303U) + 1376312589U) & 0x7fffffffU;
	return 1.0f - ((GLfloat)nn / 1073741824.0f);
}

//Applies spherical interpolation between two values
GLfloat Slerp(GLfloat a, GLfloat b, GLfloat t)
{
	GLfloat ft	= t * pi<GLfloat>();
	GLfloat f	= (1.0f - cos(ft)) * 0.5f;
	return a * (1.0f - f) + b * f;
}

//Calculates coherent noise at the given coordinate
GLfloat Noise(GLfloat x, GLfloat y)
{
	//Floor our x and y coordinates
	GLfloat fX = floor(x);
	GLfloat fY = floor(y);

	//Calculate the noise at the given coordinate and around it
	GLfloat s = Rand(fX, fY);
	GLfloat t = Rand(fX + 1, fY);
	GLfloat u = Rand(fX, fY + 1);
	GLfloat v = Rand(fX + 1, fY + 1);

	//Interpolate between the values
	GLfloat i1 = Slerp(s, t, x - fX);
	GLfloat i2 = Slerp(u, v, x - fX);

	//Interpolate between our two interpolants
	return Slerp(i1, i2, y - fY);
}

//Computes the perlin noise for the given coordinate. "p" controls roughness
GLfloat PerlinNoise(GLuint x, GLuint y, GLfloat zoom, GLfloat p)
{
	GLfloat noise = 0.0f;

	//Loop through octaves
	for (GLuint a = 0; a < octaves; a++)
	{
		GLfloat freq	= pow(2.0f, (GLfloat)a);	//Calculate frequency
		GLfloat amp		= pow(p,	(GLfloat)a);	//Calculate amplitude

		//Accumulate noise
		noise += Noise((GLfloat)x * freq / zoom, (GLfloat)y * freq / zoom) * amp;
	}

	return std::max(noise * 0.5f + 0.5f, 0.0f);
}