//Author: Bendik Hillestad, 131333
//EXAM 2014

#pragma once

#include "Common.h"

class Camera
{
public:
	Camera	();
	Camera	(const Camera&);
	~Camera	();

	void SetFOV				(const GLfloat &vFov);									//Sets the camera's vertical field of view
	void SetAspectRatio		(const GLfloat &aspect);								//Sets the camera's aspect ratio. That is, width / height
	void SetZNear			(const GLfloat &zNear);									//Sets the camera's near plane
	void SetZFar			(const GLfloat &zFar);									//Sets the camera's far plane

	void SetPosition		(const glm::vec3 &pos);									//Sets the camera's position in world space
	void SetLookDir			(const glm::vec3 &lookDir);								//Sets the camera's look direction

	void Rotate				(const GLfloat &hAngle, const GLfloat &vAngle);			//Rotates the camera by the specified degrees
	void Translate			(const glm::vec3 &disp);								//Translates the camera by the specified displacement
	void TranslateRelative	(const glm::vec2 &relDisp);								//Translates the camera relative to its view direction. X being right/left, Y being forward/backward

	glm::vec3	GetCameraPosition();												//Outputs the camera's position in world space
	void		GetViewProjMatrix(glm::mat4x4 &out);								//Outputs a view projection matrix

	void Dispose();

private:
	glm::vec3 pos;
	glm::vec3 dir;
	glm::vec3 up;
	glm::vec3 right;

	GLfloat fov;
	GLfloat aspect;
	GLfloat zNear;
	GLfloat zFar;
	GLfloat hAngle;
	GLfloat vAngle;
};