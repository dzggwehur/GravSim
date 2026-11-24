#pragma once
#include <raylib.h>
#include "raymathDouble.h"
#include "CameraAdvanced.h"
#include <deque>
#include <string>
class Debris
{
public:
	double radius = 0.0;
	double mass = 0.0;
	Model* model = nullptr;
	Vector3d position{};
	Vector3d velocity{};
	Vector3d oldAccel{};
	std::string name = "Debris";
	bool destroyed = false;
	bool anchored = false;
};