//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma once

#define GLEW_STATIC //Use static version
#include "GL/glew.h"
// #include <SDL/SDL_opengl.h>
#include <OpenGL/glu.h>

#define GLM_FORCE_RADIANS //Using degrees with glm is deprecated.
#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"


#include <string>

bool endsWith(const std::string &str, const std::string &end);