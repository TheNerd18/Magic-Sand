//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma once

#include "Common.h"

//Just some fun from last semester's maths course

//Computes the perlin noise for the given coordinate. "p" controls roughness
GLfloat PerlinNoise(GLuint x, GLuint y, GLfloat zoom, GLfloat p);