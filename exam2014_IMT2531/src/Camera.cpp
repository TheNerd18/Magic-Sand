//Author: Bendik Hillestad, 131333
//EXAM 2014

#include "Camera.h"

using namespace glm;

Camera::Camera()
{
	this->pos		= vec3();
	this->dir		= vec3();
	this->up		= vec3();
	this->right		= vec3();

	this->fov		= 0.0f;
	this->aspect	= 0.0f;
	this->zNear		= 0.0f;
	this->zFar		= 0.0f;
	this->hAngle	= 0.0f;
	this->vAngle	= 0.0f;
}

Camera::Camera(const Camera &other)
{
	//NO
}

Camera::~Camera()
{
	this->Dispose();
}

void Camera::SetFOV(const GLfloat &vFov)
{
	this->fov = vFov;
}

void Camera::SetAspectRatio(const GLfloat &aspect)
{
	this->aspect = aspect;
}

void Camera::SetZNear(const GLfloat &zNear)
{
	this->zNear = zNear;
}

void Camera::SetZFar(const GLfloat &zFar)
{
	this->zFar = zFar;
}

void Camera::SetPosition(const vec3 &pos)
{
	this->pos = pos;
}

void Camera::SetLookDir(const vec3 &lookDir)
{
	this->dir = normalize(lookDir);

	//Compute angles
	this->hAngle = atan2(this->dir.z, this->dir.x);
	this->vAngle = asin(this->dir.y);

	//Compute right vector
	this->right = vec3(
		cos(this->hAngle - half_pi<GLfloat>()),
		0,
		sin(this->hAngle - half_pi<GLfloat>())
		);

	//Compute up vector
	this->up = cross(this->right, this->dir);
}

void Camera::Rotate(const GLfloat &hAngle, const GLfloat &vAngle)
{
	//return;

	this->hAngle += hAngle;
	this->vAngle += vAngle;

	//Compute direction
	this->dir = vec3(
		cos(this->vAngle) * cos(this->hAngle),
		sin(this->vAngle),
		cos(this->vAngle) * sin(this->hAngle)
	);

	//Compute right vector
	this->right = vec3(
		cos(this->hAngle - half_pi<GLfloat>()),
		0,
		sin(this->hAngle - half_pi<GLfloat>())
	);

	//Compute up vector
	this->up = cross(this->right, this->dir);
}

void Camera::Translate(const vec3 &disp)
{
	this->pos += disp;
}

void Camera::TranslateRelative(const vec2 &relDisp)
{
	this->pos += this->dir * relDisp.y + this->right * relDisp.x;
}

vec3 Camera::GetCameraPosition()
{
	return this->pos;
}

void Camera::GetViewProjMatrix(mat4x4 &out)
{
	//Calculate view matrix
	mat4x4 viewMatrix = lookAt(this->pos, this->pos + this->dir, this->up);

	//Calculate projection matrix
	mat4x4 projMatrix = perspective(this->fov, this->aspect, this->zNear, this->zFar);

	//Output view projection matrix
	out = projMatrix * viewMatrix;
}

void Camera::Dispose()
{
	this->pos		= vec3();
	this->dir		= vec3();
	this->up		= vec3();
	this->right		= vec3();

	this->fov		= 0.0f;
	this->aspect	= 0.0f;
	this->zNear		= 0.0f;
	this->zFar		= 0.0f;
	this->hAngle	= 0.0f;
	this->vAngle	= 0.0f;
}