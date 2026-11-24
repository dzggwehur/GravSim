#pragma once
#include <raylib.h>
namespace Objects
{
	Model sun;
	Model mercury;
	Model venus;
	Model earth;
	Model mars;
	Model jupiter;
	Model saturn;
	Model uranus;
	Model neptune;
	Model moon;

	void Load()
	{
		sun = LoadModel("sun.m3d");
		mercury = LoadModel("mercury.m3d");
		venus = LoadModel("venus.m3d");
		earth = LoadModel("earth.m3d");
		mars = LoadModel("mars.m3d");
		jupiter = LoadModel("jupiter.m3d");
		saturn = LoadModel("saturn.m3d");
		uranus = LoadModel("uranus.m3d");
		neptune = LoadModel("neptune.m3d");
		moon = LoadModel("moon.m3d");
	}
}
