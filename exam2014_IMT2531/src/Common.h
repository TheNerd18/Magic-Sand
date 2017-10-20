//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma once

#define GLEW_STATIC //Use static version
#include "D:\git\openFrameworks\libs\glew\include\GL\glew.h"
#include <SDL_opengl.h>
#include <gl\glu.h>

#define GLM_FORCE_RADIANS //Using degrees with glm is deprecated.
#include "D:\git\openFrameworks\libs\glm\include\glm\glm.hpp"
#include "D:\git\openFrameworks\libs\glm\include\glm\gtc\constants.hpp"
#include "D:\git\openFrameworks\libs\glm\include\glm\gtc\matrix_transform.hpp"
#include "D:\git\openFrameworks\libs\glm\include\glm\gtc\type_ptr.hpp"


#include <string>

bool endsWith(const std::string &str, const std::string &end);