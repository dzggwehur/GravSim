#pragma once
#include <raylib.h>
namespace Objects
{
	Model models[10];
	void Load()
	{
		models[0] = LoadModel("sun.m3d");
		models[1] = LoadModel("mercury.m3d");
		models[2] = LoadModel("venus.m3d");
		models[3] = LoadModel("earth.m3d");
		models[4] = LoadModel("mars.m3d");
		models[5] = LoadModel("jupiter.m3d");
		models[6] = LoadModel("saturn.m3d");
		models[7] = LoadModel("uranus.m3d");
		models[8] = LoadModel("neptune.m3d");
		models[9] = LoadModel("moon.m3d");
	}
}