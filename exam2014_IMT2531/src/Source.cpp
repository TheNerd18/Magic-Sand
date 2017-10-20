//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "SDL2main.lib")
#pragma comment(lib, "SDL2_image.lib")
#pragma comment(lib, "OpenGL32.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "glu32.lib")

#include <SDKDDKVer.h>
#include <SDL.h>

#include "Common.h"
#include "Shader.h"
#include "Camera.h"
#include "Terrain.h"
#include "Mesh.h"

#include <fstream>
#include <stdio.h>
#include <string>
#include <vector>

using namespace std;
using namespace glm;

//Prototypes
bool init();									//Starts up SDL, creates window, and initializes OpenGL
bool initGL();									//Initializes rendering program and clear color
void toggleFullscreen(GLboolean fullscreen);	//Toggles fullscreen
void events(SDL_Event& eventHandler);			//Event handler
void update(GLfloat dt);						//Per frame update
void render();									//Renders quad to the screen
void close();									//Frees media and shuts down SDL

//Constants
const char*		APPLICATION_NAME			= "Exam 2014";
const GLfloat	CAMERA_MOVE_SPEED			= 100.0f;
const GLfloat	CAMERA_ROT_SPEED			= 0.5f;
const GLfloat	UPDATE_TICK					= 1.0f / 60.0f;			//For deterministic execution

//Globals
SDL_Window*					gWindow			= NULL;					//The window we'll be rendering to
SDL_GLContext				gContext;								//OpenGL context
SDL_DisplayMode				gDisplayMode;							//The screen's display mode

GLuint						gScreenWidth;
GLuint						gScreenHeight;

GLboolean					gQuit			= GL_FALSE;				//The application will quit if this is true.
GLboolean					gFullscreen		= GL_FALSE;
GLboolean					gIsFullscreen	= GL_FALSE;
GLboolean					gHasFocus		= GL_TRUE;

string						gHeightmap		= "";

GLfloat						gTime			= 12.0f;
GLfloat						gSunApex		= 50.0f * pi<GLfloat>() / 180.f;
vec2						gSunPos			= glm::vec2(0.0f, gSunApex);
vec3						gSunDir			= glm::vec3();
vec2						gMousePos		= glm::vec2();
Camera*						gCamera			= new Camera();
Shader*						gShader			= new Shader();
Shader*						gSkyShader		= new Shader();
Terrain*					gTerrain		= new Terrain();
GLuint						gSkyVertices	= NULL;
GLuint						gSkyIndices		= NULL;
GLfloat						gWaterChange	= 0.6f;
GLfloat						gWaterLevel		= 0.0f;
GLfloat						gTerrainHeight	= 150.0f;

GLboolean					W				= GL_FALSE,
							A				= GL_FALSE,
							S				= GL_FALSE,
							D				= GL_FALSE,
							P				= GL_FALSE,
							O				= GL_FALSE,
							gAnimSun		= GL_FALSE,
							gDrivingMode	= GL_FALSE;

//Program main entry point
int main(int argc, char* args[])
{
	//Check commandline arguments (first one is just the program's location)
	if (argc > 1)
	{
		//Assume the last argument is the file to open
		gHeightmap = string(args[argc - 1]);
	}

	//Start up SDL and create window
	if (!init())
	{
		printf("Failed to initialize!\n");
	}
	else
	{
		//Main loop flag
		gQuit = GL_FALSE;

		//Event handler
		SDL_Event eventHandler;

		//Toggle fullscreen
		if (gFullscreen) toggleFullscreen(GL_TRUE);

		//Enable text input
		SDL_StartTextInput();

		//Lock mouse
		SDL_SetRelativeMouseMode(SDL_TRUE);

		//Get current ticks
		Uint32 ticks = SDL_GetTicks();
		Uint32 newTicks;

		GLfloat accumulator = 0.0f;
		GLfloat dt;

		//While application is running
		while (!gQuit)
		{
			//Handle events
			events(eventHandler);

			//Check if window has focus
			if (gHasFocus)
			{
				//Check if mouse needs to be locked again
				if (SDL_GetRelativeMouseMode() != SDL_TRUE)
				{
					//Lock mouse
					SDL_SetRelativeMouseMode(SDL_TRUE);
				}

				//Check if fullscreen needs to be toggled on/off
				if (gFullscreen && !gIsFullscreen)
				{
					//Turn fullscreen on
					toggleFullscreen(GL_TRUE);
				}
				else if (!gFullscreen && gIsFullscreen)
				{
					//Turn fullscreen off
					toggleFullscreen(GL_FALSE);
				}
			}
			else
			{
				//Check if mouse needs to be unlocked
				if (SDL_GetRelativeMouseMode() == SDL_TRUE)
				{
					//Lock mouse
					SDL_SetRelativeMouseMode(SDL_FALSE);
				}

				//Check if fullscreen needs to be toggled off
				if (gIsFullscreen)
				{
					//Turn fullscreen off
					toggleFullscreen(GL_FALSE);
				}

				//Ignore everything else
				ticks = SDL_GetTicks();
				continue;
			}

			//Calculate change in time
			newTicks	= SDL_GetTicks();
			dt			= (newTicks - ticks) / 1000.0f;
			ticks		= newTicks;

			//Display FPS in the title
			std::string title = APPLICATION_NAME + (" FPS: " + std::to_string(1.0f / dt));
			SDL_SetWindowTitle(gWindow, title.c_str());

			//Update accumulator
			accumulator += dt;

			//Clamp accumulator at 2 * UPDATE_TICK to avoid grinding to a halt
			if (accumulator > 2.0f * UPDATE_TICK) accumulator = 2.0f * UPDATE_TICK;

			//Handle updates
			while (accumulator > UPDATE_TICK)
			{
				update(UPDATE_TICK);

				accumulator -= UPDATE_TICK;
			}

			//Render
			render();
		}

		//Disable text input
		SDL_StopTextInput();

		//Unlock mouse
		SDL_SetRelativeMouseMode(SDL_FALSE);

		//Toggle fullscreen off
		if (gIsFullscreen) toggleFullscreen(GL_FALSE);
	}

	//Free resources and close SDL
	close();

	return 0;
}

bool init()
{
	//Initialization flag
	bool success = true;

	//Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		printf("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
		success = false;
	}
	else
	{
		//Ask SDL to give us a core context.
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

		//Grab display mode
		if (SDL_GetCurrentDisplayMode(0, &gDisplayMode) != 0)
		{
			printf("Couldn't get the screen's display mode! SDL Error: %s\n", SDL_GetError());
			success = false;
		}
		else
		{
			//Set window size
			gScreenWidth	= gDisplayMode.w;
			gScreenHeight	= gDisplayMode.h;

			//Create window
			gWindow = SDL_CreateWindow(APPLICATION_NAME, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, gScreenWidth / 2, gScreenHeight / 2, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
			if (gWindow == NULL)
			{
				printf("Window could not be created! SDL Error: %s\n", SDL_GetError());
				success = false;
			}
			else
			{
				//Create context
				gContext = SDL_GL_CreateContext(gWindow);
				if (gContext == NULL)
				{
					printf("OpenGL context could not be created! SDL Error: %s\n", SDL_GetError());
					success = false;
				}
				else
				{
					//Initialize GLEW
					glewExperimental = GL_TRUE;
					GLenum glewError = glewInit();
					if (glewError != GLEW_OK)
					{
						printf("Error initializing GLEW! %s\n", glewGetErrorString(glewError));
					}

					//Use Vsync
					if (SDL_GL_SetSwapInterval(1) < 0)
					{
						printf("Warning: Unable to set VSync! SDL Error: %s\n", SDL_GetError());
					}

					//Initialize OpenGL
					if (!initGL())
					{
						printf("Unable to initialize OpenGL!\n");
						success = false;
					}
				}
			}
		}
	}

	return success;
}

bool initGL()
{
	//Success flag
	bool success = true;

	GLuint vao;					// Since we are asking for a core context,
	glGenVertexArrays(1, &vao);	// we also need to create and bind a VAO.
	glBindVertexArray(vao);

	//Set clear color
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	//Enable depth
	glEnable	(GL_DEPTH_TEST);
	glDepthFunc	(GL_LESS);

	//Setup camera
	gCamera->SetPosition	(vec3(0, 200, 0));
	gCamera->SetLookDir		(vec3(0, -1, 0));
	gCamera->SetAspectRatio	(static_cast<GLfloat>(gScreenWidth) / static_cast<GLfloat>(gScreenHeight));
	gCamera->SetFOV			(60.0f * pi<GLfloat>() / 180.f);
	gCamera->SetZNear		(1.0f);
	gCamera->SetZFar		(2000.0f);

	//Calculate sun's position in cartesian coordinates
	vec3 sunCartPos = vec3(
		cos(gSunPos.y) * cos(gSunPos.x),
		sin(gSunPos.y),
		cos(gSunPos.y) * sin(gSunPos.x)
	);

	//Set sun direction
	gSunDir = -sunCartPos;

	//Load shaders
	if (!gShader->LoadShaders("Shader.vs", "Shader.ps"))
	{
		getchar();

		success = false;
	}

	if (!gSkyShader->LoadShaders("SkyShader.vs", "SkyShader.ps"))
	{
		getchar();

		success = false;
	}

	//Load/Generate heightmap
	if (!gHeightmap.empty())
	{
		//Load heightmap from file
		if (endsWith(gHeightmap, ".pnm"))
		{
			if (!gTerrain->LoadHeightMap(gHeightmap.c_str()))
			{
				getchar();

				success = false;
			}
		}
		else if (endsWith(gHeightmap, ".asc"))
		{
			if (!gTerrain->LoadGeoHeightMap(gHeightmap.c_str()))
			{
				getchar();

				success = false;
			}
		}
		else
		{
			printf("#ERROR: Unrecognized file type %s!\n", gHeightmap.substr(gHeightmap.find_last_of('.')));

			getchar();

			success = false;
		}
	}
	else
	{
		//Generate random heightmap
		if (!gTerrain->LoadHeightMapFromPerlinNoise(1024, 1024, 250.f, 0.5f))
		{
			getchar();

			success = false;
		}
	}

	//Compute terrain
	if (!gTerrain->ComputeTerrain(gWaterLevel, gTerrainHeight))
	{
		getchar();

		success = false;
	}

	//Create sky
	GLfloat* skyVertices = new GLfloat[8 * 3]
	{
		-1, 1, 1,
		1, 1, 1,
		-1, 1, -1,
		1, 1, -1,

		-1, -1, 1,
		1, -1, 1,
		-1, -1, -1,
		1, -1, -1,
	};

	GLuint* skyIndices = new GLuint[36]
	{
		0, 1, 2,
		2, 1, 3,

		4, 5, 6,
		6, 5, 7,

		0, 1, 4,
		4, 1, 5,

		2, 3, 6,
		6, 3, 7,

		1, 3, 5,
		5, 3, 7,

		0, 2, 4,
		4, 2, 6
	};

	glGenBuffers(1, &gSkyVertices);
	glBindBuffer(GL_ARRAY_BUFFER, gSkyVertices);
	glBufferData(GL_ARRAY_BUFFER, 8 * 3 * sizeof(GLfloat), skyVertices, GL_STATIC_DRAW);

	glGenBuffers(1, &gSkyIndices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gSkyIndices);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 36 * sizeof(GLuint), skyIndices, GL_STATIC_DRAW);

	delete[] skyVertices;
	delete[] skyIndices;

	return success;
}

void toggleFullscreen(GLboolean fullscreen)
{
	if (fullscreen && !gIsFullscreen)
	{
		//Turn fullscreen on
		if (SDL_SetWindowFullscreen(gWindow, SDL_WINDOW_FULLSCREEN_DESKTOP) == 0) gIsFullscreen = GL_TRUE;
		else printf("Window couldn't be set to fullscreen! SDL Error: %s\n", SDL_GetError());

		glViewport(0, 0, gScreenWidth, gScreenHeight);
	}
	else if (!fullscreen && gIsFullscreen)
	{
		//Turn fullscreen off
		if (SDL_SetWindowFullscreen(gWindow, 0) == 0) gIsFullscreen = GL_FALSE;
		else printf("Window couldn't be set to windowed! SDL Error: %s\n", SDL_GetError());

		glViewport(0, 0, gScreenWidth / 2, gScreenHeight / 2);
	}
}

void events(SDL_Event& eventHandler)
{
	gMousePos.x = 0;
	gMousePos.y = 0;

	//Handle events on queue
	while (SDL_PollEvent(&eventHandler) != 0)
	{
		//User requests quit
		if (eventHandler.type == SDL_QUIT)
		{
			gQuit = GL_TRUE;
		}

		//Check window changes
		if (eventHandler.type == SDL_WINDOWEVENT)
		{
			switch (eventHandler.window.event)
			{
			case SDL_WINDOWEVENT_FOCUS_GAINED:
				gHasFocus = GL_TRUE; break;
			case SDL_WINDOWEVENT_FOCUS_LOST:
				gHasFocus = GL_FALSE; break;
			}
		}

		//Check mouse changes
		if (eventHandler.type == SDL_MOUSEMOTION)
		{
			gMousePos.x = (GLfloat)eventHandler.motion.xrel;
			gMousePos.y = (GLfloat)-eventHandler.motion.yrel;
		}

		//Check key changes
		if (eventHandler.type == SDL_KEYDOWN)
		{
			if (eventHandler.key.keysym.mod & KMOD_ALT)
			{
				switch (eventHandler.key.keysym.sym)
				{
				//TODO: Add more keys
				case SDLK_RETURN: gFullscreen = !gFullscreen; break;
				}
			}
			else
			{
				switch (eventHandler.key.keysym.sym)
				{
				case SDLK_w:		W				= GL_TRUE;			break;
				case SDLK_a:		A				= GL_TRUE;			break;
				case SDLK_s:		S				= GL_TRUE;			break;
				case SDLK_d:		D				= GL_TRUE;			break;
				case SDLK_p:		P				= GL_TRUE;			break;
				case SDLK_o:		O				= GL_TRUE;			break;
				case SDLK_t:		gAnimSun		= !gAnimSun;		break;
				case SDLK_SPACE:	gDrivingMode	= !gDrivingMode;	break;
				case SDLK_ESCAPE:	gQuit			= true;				break;
				}
			}
		}
		if (eventHandler.type == SDL_KEYUP)
		{
			switch (eventHandler.key.keysym.sym)
			{
			case SDLK_w: W = GL_FALSE; break;
			case SDLK_a: A = GL_FALSE; break;
			case SDLK_s: S = GL_FALSE; break;
			case SDLK_d: D = GL_FALSE; break;
			case SDLK_p: P = GL_FALSE; break;
			case SDLK_o: O = GL_FALSE; break;
			}
		}
	}
}

void update(GLfloat dt)
{
	//Calculate camera motion
	glm::vec2 moveDir = vec2(0.0f, 0.0f);

	if (W) moveDir += vec2(0, 1.0f);
	if (A) moveDir += vec2(-1.0f, 0);
	if (S) moveDir += vec2(0, -1.0f);
	if (D) moveDir += vec2(1.0f, 0);

	//Check if actually moving
	GLfloat len = length(moveDir);
	if (len > FLT_EPSILON)
	{
		//Normalise
		moveDir /= len;

		//Update camera position
		gCamera->TranslateRelative(moveDir * (CAMERA_MOVE_SPEED * dt));
	}

	//Calculate camera rotation
	gCamera->Rotate(gMousePos.x * CAMERA_ROT_SPEED * dt, -gMousePos.y * CAMERA_ROT_SPEED * dt);

	//Put camera on terrain
	if (gDrivingMode)
	{
		//Get camera position
		vec3 pos = gCamera->GetCameraPosition();

		//Sample terrain height
		GLfloat h = gTerrain->SampleHeight(pos.x, pos.z);

		//Set new camera height
		pos.y = max(h - gWaterLevel, 0.0f) * gTerrainHeight + 10.0f;
		gCamera->SetPosition(pos);
	}

	//Move sun (polar coordinates)
	if (gAnimSun)
	{
		gTime = fmodf(gTime + dt, 24.0f);
		gSunPos.x = (gTime / 24.0f) * pi<GLfloat>() * 2.0f - pi<GLfloat>();
		gSunPos.y = sin(gSunPos.x + half_pi<GLfloat>()) * gSunApex;

		//Calculate sun's position in cartesian coordinates
		vec3 sunCartPos = vec3(
			cos(gSunPos.y) * cos(gSunPos.x),
			sin(gSunPos.y),
			cos(gSunPos.y) * sin(gSunPos.x)
			);

		//Get sun direction
		gSunDir = -sunCartPos;
	}

	//Update water level
	if (P && !O)
	{
		//Raise water level
		gWaterLevel += gWaterChange * dt;

		if (gWaterLevel > 0.9f) gWaterLevel = 0.9f;
	}
	else if (!P && O)
	{
		//Lower water level
		gWaterLevel -= gWaterChange * dt;

		if (gWaterLevel < 0.0f) gWaterLevel = 0.0f;
	}

	//Recompute terrain
	if (P ^ O) gTerrain->ComputeTerrain(gWaterLevel, gTerrainHeight);
}

void render()
{
	//Grab view projection matrix
	glm::mat4x4 viewMatrix;
	gCamera->GetViewProjMatrix(viewMatrix);

	//Clear buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Render terrain
	glDepthFunc(GL_LESS);
	{
		//Use shader
		glUseProgram(gShader->GetProgramID());

		//Set uniforms
		GLuint uniformLight = gShader->GetUniformLocation("LightDir");
		glUniform3fv(uniformLight, 1, value_ptr(gSunDir));

		GLuint uniformMaxZ = gShader->GetUniformLocation("MaxZ");
		glUniform1f(uniformMaxZ, (1.0f - gWaterLevel) * gTerrainHeight);

		GLuint uniformView = gShader->GetUniformLocation("View");
		glUniformMatrix4fv(uniformView, 1, GL_FALSE, value_ptr(viewMatrix));

		//Render
		gTerrain->Render(gShader);

		//Unbind
		glUseProgram(NULL);
	}

	//Render skybox
	glDepthFunc(GL_LEQUAL);
	{
		//Use sky shader
		glUseProgram(gSkyShader->GetProgramID());

		//Set uniforms
		GLuint uniformView = gSkyShader->GetUniformLocation("View");
		glUniformMatrix4fv(uniformView, 1, GL_FALSE, value_ptr(viewMatrix));

		GLuint uniformCamPos = gSkyShader->GetUniformLocation("CameraPos");
		glUniform3fv(uniformCamPos, 1, value_ptr(gCamera->GetCameraPosition()));

		GLuint uniformSun = gSkyShader->GetUniformLocation("Sun");
		glUniform2fv(uniformSun, 1, value_ptr(gSunPos));

		//Bind buffers
		glBindBuffer(GL_ARRAY_BUFFER, gSkyVertices);
		glVertexAttribPointer(gSkyShader->GetAttributeLocation("vertexPosition"), 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(gSkyShader->GetAttributeLocation("vertexPosition"));

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gSkyIndices);

		//Draw sky
		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, NULL);

		//Disable vertex attributes
		glDisableVertexAttribArray(gSkyShader->GetAttributeLocation("vertexPosition"));

		//Unbind shader
		glUseProgram(NULL);
	}

	//Update screen
	SDL_GL_SwapWindow(gWindow);
}

void close()
{
	//Destroy window
	if (gWindow)
	{
		SDL_DestroyWindow(gWindow);
		gWindow = NULL;
	}
	//Destroy terrain
	if (gTerrain)
	{
		gTerrain->Dispose();
		gTerrain = NULL;
	}
	//Destroy shader
	if (gShader)
	{
		gShader->Dispose();
		gShader = NULL;
	}

	//Quit SDL subsystems
	SDL_Quit();
}