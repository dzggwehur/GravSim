/*
* ADVANCED 1: The world has many issues: from hunger, to supply chain shortages, to climate change. Create a piece of technology that assists with humanity’s pursuit to solve one of these problems. 
ADVANCED 2: Develop a guardrail on an online system or in the future which can help improve data privacy.
*/
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <raylib.h>
#include "Simulation.h"
#include "Models.h"
#include "CameraAdvanced.h"
#include <rlgl.h>
#include <string>
#include <array>
#define _WINDOWS_
#define WIN32_LEAN_AND_MEAN
#include <rapidcsv.h>
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

Vector3d realCameraPosition{};
Camera worldCamera{};

bool paused = true;
float camSpeed = 1000000;
bool cursor = false;
int objectToTrack = -1;
double simTime = 0.0f;
bool enableCreateGUI = false;
bool highlight[8] = { false };
float createMass = 0;
float createRadius = 0;
double AU_TO_M = 149597870700.0;
std::array<char[1223], 5> floatText{};
float initialOrbitCameraDist = 0;
bool placeMode = false;
int maxSubSteps = 1000000;
//constexpr double AU_TO_M = 149597870700.0;
Simulation sim;
#include <cmath>
#include <fstream>
#include <chrono>

void GetSubstepLimit(int integrator)
{
	sim.AddObject(1.0, Vector3d(-5.0, 0.0, 0.0), Vector3d(0.0, 0.0, -std::sqrt((6.67430e-11 * 1.0) / 20.0)), 1.0, BLACK, &Objects::sun, "Substep Test Object", false);
	sim.AddObject(1.0, Vector3d(5.0, 0.0, 0.0), Vector3d(0.0, 0.0, std::sqrt((6.67430e-11 * 1.0) / 20.0)), 1.0, BLACK, &Objects::sun, "Substep Test Object 2", false);

	auto startTime = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < 250000; ++i)
	{
		sim.Update(objectToTrack, simTime, integrator, maxSubSteps);
	}
	auto endTime = std::chrono::high_resolution_clock::now();
	double duration = std::chrono::duration<double, std::milli>(endTime - startTime).count();
	std::cout << duration << " ms\n";

	maxSubSteps = std::clamp(static_cast<int>(2000.0 / std::max(duration / 1000.0, 0.001)), 1, 500000);
	simTime = 0.0;
	sim.simulationObjects.clear();
}
void UpdateGame()
{
	if (!paused)
	{
		sim.Update(objectToTrack, simTime, IAS15, maxSubSteps);
	}
	if (IsKeyPressed(KEY_ESCAPE))
	{
		if (cursor)
		{
			DisableCursor();
		}
		else
		{
			EnableCursor();
		}
		cursor = !cursor;
	}
	if ((enableCreateGUI || placeMode) && !cursor)
	{
		cursor = true;
		EnableCursor();
	}
	if (IsKeyPressed(KEY_K))
	{
		objectToTrack++;
		initialOrbitCameraDist = sim.simulationObjects[objectToTrack].radius * 12.0;
		if (objectToTrack >= sim.simulationObjects.size())
		{
			objectToTrack = -1;
		}
	}
	if (!cursor && objectToTrack == -1)
	{
		UpdateCameraCustom(&worldCamera, &camSpeed, realCameraPosition);
	}

	else if (objectToTrack != -1)
	{
		//worldCamera.target = static_cast<Vector3>(Vector3dNormalize(Vector3dSubtract(realCameraPosition, sim.simulationObjects[objectToTrack].position)));
		UpdateOrbitalCamera(worldCamera, realCameraPosition, sim.simulationObjects[objectToTrack].position, initialOrbitCameraDist, !cursor);
	}

	if (IsKeyPressed(KEY_P)) paused = !paused;
	BeginDrawing();
	ClearBackground(BLACK);

	BeginMode3D(worldCamera);
	sim.Draw(realCameraPosition);
	if (placeMode)
	{
		rlSetClipPlanes(1.0, 1000.0);
		Ray ray = GetScreenToWorldRay(GetMousePosition(), worldCamera);
		float depth = createRadius * 3.0f;

		Vector3 cursor3D = Vector3Add(ray.position, Vector3Scale(ray.direction, depth));
		rlSetClipPlanes(1.0, 123123123123123123123123123123123123123.0);
		DrawSphereWires(cursor3D, createRadius, 10, 10, GREEN);
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
		{
			sim.AddObject(createMass * 5.9720e24, cursor3D + realCameraPosition, Vector3d{}, createRadius, WHITE, &Objects::sun, floatText[2], false);
			placeMode = false;
		}
	}
	EndMode3D();
	sim.Draw2D(realCameraPosition, worldCamera);
	DrawText(std::string("Simulation Time: " + std::to_string(simTime) + " seconds").c_str(), 10, 10, 25, WHITE);
	DrawText(std::string("Simulation Timestep: " + std::to_string(sim.timestep) + " seconds per frame").c_str(), 10, 40, 25, WHITE);
	DrawText(std::string("Simulation Substeps: " + std::to_string(sim.substeps) + " substeps per frame").c_str(), 10, 70, 25, WHITE);
	if (objectToTrack != -1)
	{
		DrawText(std::string("Currently viewing object: " + sim.simulationObjects[objectToTrack].name).c_str(), 10, 100, 25, WHITE);
	}
	if (GuiButton(Rectangle{ 24, 80, 120, 30 }, "+ Add Object"))
	{
		enableCreateGUI = !enableCreateGUI;
	}
	if (objectToTrack != -1)
	{
		if (GuiButton(Rectangle{ 24, 130, 120, 30 }, "- Delete Tracked Object"))
		{
			sim.simulationObjects.erase(sim.simulationObjects.begin() + objectToTrack);
		}
	}
	if (GuiValueBoxFloat(Rectangle(10, 370, 120, 30), "Global Timestep (s/t)", floatText[3], &sim.intendedTimestep, highlight[4])) highlight[4] = !highlight[4];
	if (enableCreateGUI)
	{
		DrawRectangle(300, 300, 500, 500, WHITE);

		if (GuiValueBoxFloat(Rectangle(310, 310, 480, 50), "Mass (Earths)", floatText[0], &createMass, highlight[0])) highlight[0] = !highlight[0];
		if (GuiValueBoxFloat(Rectangle(310, 370, 480, 50), "Radius (m)", floatText[1], &createRadius, highlight[1])) highlight[1] = !highlight[1];
		if (GuiTextBox(Rectangle(310, 430, 480, 50), floatText[2], 25, highlight[2])) highlight[2] = !highlight[2];

		if (GuiButton(Rectangle(310, 490, 480, 50), "Create!"))
		{
			placeMode = true;
			enableCreateGUI = false;
		}
	}
	DrawFPS(10, 700);
	EndDrawing();

}
int main()
{
	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	GetSubstepLimit(IAS15);
	std::cout << maxSubSteps << std::endl;
	InitWindow(400, 500, "Test");
	SetExitKey(KEY_F8);

	sim.timestep = 1.0f;
	sim.substeps = 100;
	Objects::Load();
	worldCamera.position = Vector3{ -0.0f,0.0f,0.0f };
	worldCamera.target = Vector3{ 3.0f,0.0f,0.0f };
	worldCamera.projection = CAMERA_PERSPECTIVE;
	worldCamera.up = Vector3{ 0.0f,1.0f,0.0f };
	worldCamera.fovy = 45.0f;
	// ============================================================================
	// SYSTEM PLANET BOUNDARIES
	// ============================================================================
	// Equal-mass circular binary separated by exactly 8.0 meters (R = 4.0 m each)
	// Mass chosen so orbital period is ~10 seconds (vOrb ~ 2.51 m/s)
	sim.AddObject(1.98900e+30, Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), 6.96340e+08, RED, &Objects::sun, "Sun", true);

	sim.AddObjectKeplerian(3.3011e23, 2439700.0, Color{ 210, 210, 210, 255 }, &Objects::mercury, "Mercury", 0.38709936 * AU_TO_M, 0.20564070, 7.00349211 * DEG2RAD, 48.29942987 * DEG2RAD, 29.19848673 * DEG2RAD, 103.94401953 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(4.8675e24, 6051800.0, Color{ 245, 222, 179, 255 }, &Objects::venus, "Venus", 0.72333664 * AU_TO_M, 0.00676645, 3.39447882 * DEG2RAD, 76.61041710 * DEG2RAD, 54.99272092 * DEG2RAD, 280.23034092 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(5.97237e24, 6371000.0, Color{ 160, 205, 255, 255 }, &Objects::earth, "Earth", 1.00000402 * AU_TO_M, 0.01670025, 359.99674793 * DEG2RAD, 0.00000000 * DEG2RAD, 103.01850255 * DEG2RAD, 357.53558384 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(6.4171e23, 3389500.0, Color{ 245, 165, 145, 255 }, &Objects::mars, "Mars", 1.52371496 * AU_TO_M, 0.09341381, 1.84765854 * DEG2RAD, 49.48639355 * DEG2RAD, 286.68108262 * DEG2RAD, 124.48577126 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(1.8982e27, 69911000.0, Color{ 252, 202, 164, 255 }, &Objects::jupiter, "Jupiter", 5.20285798 * AU_TO_M, 0.04835311, 1.30393765 * DEG2RAD, 100.52508326 * DEG2RAD, 274.25652970 * DEG2RAD, 58.32213120 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(5.6834e26, 58232000.0, Color{ 248, 232, 180, 255 }, &Objects::saturn, "Saturn", 9.53636328 * AU_TO_M, 0.05373431, 2.48647591 * DEG2RAD, 113.59025302 * DEG2RAD, 338.90387938 * DEG2RAD, 263.09188484 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(8.6810e25, 25362000.0, Color{ 185, 240, 245, 255 }, &Objects::uranus, "Uranus", 19.18867419 * AU_TO_M, 0.04724645, 0.77203047 * DEG2RAD, 74.02752679 * DEG2RAD, 97.02876550 * DEG2RAD, 249.30525198 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(1.02413e26, 24622000.0, Color{ 165, 190, 255, 255 }, &Objects::neptune, "Neptune", 30.06998849 * AU_TO_M, 0.00860324, 1.77013190 * DEG2RAD, 131.78295405 * DEG2RAD, 273.10120236 * DEG2RAD, 314.61217249 * DEG2RAD, MeanAnomaly, "Sun");
	sim.AddObjectKeplerian(1.303e22, 1188300.0, Color{ 230, 215, 200, 255 }, &Objects::moon, "Pluto", 39.48203776 * AU_TO_M, 0.24884023, 17.14002411 * DEG2RAD, 110.30097805 * DEG2RAD, 113.75778060 * DEG2RAD, 51.17322485 * DEG2RAD, MeanAnomaly, "Sun");

	sim.AddObjectKeplerian(7.342e22, 1737400.0, Color{ 225, 225, 225, 255 }, &Objects::moon, "Moon", 384400000.0, 0.0554, 5.16 * DEG2RAD, 125.08 * DEG2RAD, 318.15 * DEG2RAD, 215.1876 * DEG2RAD, MeanAnomaly, "Earth");

	sim.AddObjectKeplerian(1.0659e16, 11080.0, Color{ 215, 200, 195, 255 }, &Objects::moon, "Phobos", 9376000.0, 0.0151, 1.093 * DEG2RAD, 169.20 * DEG2RAD, 216.30 * DEG2RAD, 351.5587 * DEG2RAD, MeanAnomaly, "Mars");
	sim.AddObjectKeplerian(1.4762e15, 6200.0, Color{ 225, 215, 205, 255 }, &Objects::moon, "Deimos", 23463200.0, 0.00033, 1.791 * DEG2RAD, 54.30 * DEG2RAD, 0.0 * DEG2RAD, 207.8504 * DEG2RAD, MeanAnomaly, "Mars");

	sim.AddObjectKeplerian(8.9319e22, 1821600.0, Color{ 255, 245, 175, 255 }, &Objects::moon, "Io", 421800000.0, 0.0041, 0.05 * DEG2RAD, 43.97 * DEG2RAD, 84.129 * DEG2RAD, 181.4171 * DEG2RAD, MeanAnomaly, "Jupiter");
	sim.AddObjectKeplerian(4.7998e22, 1560800.0, Color{ 240, 230, 215, 255 }, &Objects::moon, "Europa", 671100000.0, 0.0094, 0.47 * DEG2RAD, 219.106 * DEG2RAD, 88.970 * DEG2RAD, 314.3243 * DEG2RAD, MeanAnomaly, "Jupiter");
	sim.AddObjectKeplerian(1.4819e23, 2634100.0, Color{ 215, 210, 200, 255 }, &Objects::moon, "Ganymede", 1070400000.0, 0.0013, 0.20 * DEG2RAD, 63.552 * DEG2RAD, 192.417 * DEG2RAD, 72.7886 * DEG2RAD, MeanAnomaly, "Jupiter");
	sim.AddObjectKeplerian(1.0759e23, 2410300.0, Color{ 200, 195, 190, 255 }, &Objects::moon, "Callisto", 1882700000.0, 0.0074, 0.192 * DEG2RAD, 298.848 * DEG2RAD, 52.643 * DEG2RAD, 237.6424 * DEG2RAD, MeanAnomaly, "Jupiter");

	sim.AddObjectKeplerian(3.7493e19, 198200.0, Color{ 230, 230, 235, 255 }, &Objects::moon, "Mimas", 185539000.0, 0.0196, 1.574 * DEG2RAD, 173.0 * DEG2RAD, 332.5 * DEG2RAD, 156.7281 * DEG2RAD, MeanAnomaly, "Saturn");
	sim.AddObjectKeplerian(1.0802e20, 252100.0, Color{ 245, 250, 255, 255 }, &Objects::moon, "Enceladus", 238037000.0, 0.0047, 0.009 * DEG2RAD, 342.0 * DEG2RAD, 197.0 * DEG2RAD, 296.4878 * DEG2RAD, MeanAnomaly, "Saturn");
	sim.AddObjectKeplerian(6.1745e20, 531100.0, Color{ 235, 235, 240, 255 }, &Objects::moon, "Tethys", 294672000.0, 0.0001, 1.12 * DEG2RAD, 260.0 * DEG2RAD, 45.0 * DEG2RAD, 348.4687 * DEG2RAD, MeanAnomaly, "Saturn");
	sim.AddObjectKeplerian(1.0955e21, 561400.0, Color{ 230, 230, 235, 255 }, &Objects::moon, "Dione", 377415000.0, 0.0022, 0.019 * DEG2RAD, 57.0 * DEG2RAD, 284.0 * DEG2RAD, 326.4704 * DEG2RAD, MeanAnomaly, "Saturn");
	sim.AddObjectKeplerian(2.3065e21, 763800.0, Color{ 225, 225, 230, 255 }, &Objects::moon, "Rhea", 527068000.0, 0.001258, 0.345 * DEG2RAD, 351.0 * DEG2RAD, 241.0 * DEG2RAD, 135.4214 * DEG2RAD, MeanAnomaly, "Saturn");
	sim.AddObjectKeplerian(1.3452e23, 2574730.0, Color{ 245, 215, 165, 255 }, &Objects::moon, "Titan", 1221870000.0, 0.0288, 0.34854 * DEG2RAD, 28.06 * DEG2RAD, 180.53 * DEG2RAD, 50.7606 * DEG2RAD, MeanAnomaly, "Saturn");
	sim.AddObjectKeplerian(5.6199e18, 135000.0, Color{ 220, 210, 200, 255 }, &Objects::moon, "Hyperion", 1481009000.0, 0.123006, 0.43 * DEG2RAD, 94.0 * DEG2RAD, 220.0 * DEG2RAD, 154.3063 * DEG2RAD, MeanAnomaly, "Saturn");
	sim.AddObjectKeplerian(1.8056e21, 734500.0, Color{ 215, 210, 205, 255 }, &Objects::moon, "Iapetus", 3560820000.0, 0.028612, 15.47 * DEG2RAD, 75.79 * DEG2RAD, 271.60 * DEG2RAD, 255.5405 * DEG2RAD, MeanAnomaly, "Saturn");

	sim.AddObjectKeplerian(6.59e19, 235800.0, Color{ 220, 225, 230, 255 }, &Objects::moon, "Miranda", 129390000.0, 0.0013, 4.232 * DEG2RAD, 326.43 * DEG2RAD, 68.31 * DEG2RAD, 59.7417 * DEG2RAD, MeanAnomaly, "Uranus");
	sim.AddObjectKeplerian(1.353e21, 578900.0, Color{ 230, 235, 240, 255 }, &Objects::moon, "Ariel", 191020000.0, 0.0012, 0.260 * DEG2RAD, 22.39 * DEG2RAD, 115.35 * DEG2RAD, 259.4800 * DEG2RAD, MeanAnomaly, "Uranus");
	sim.AddObjectKeplerian(1.172e21, 584700.0, Color{ 205, 210, 215, 255 }, &Objects::moon, "Umbriel", 266000000.0, 0.0039, 0.128 * DEG2RAD, 33.48 * DEG2RAD, 84.71 * DEG2RAD, 209.4970 * DEG2RAD, MeanAnomaly, "Uranus");
	sim.AddObjectKeplerian(3.527e21, 788400.0, Color{ 225, 220, 220, 255 }, &Objects::moon, "Titania", 435910000.0, 0.0011, 0.340 * DEG2RAD, 99.77 * DEG2RAD, 284.40 * DEG2RAD, 329.2235 * DEG2RAD, MeanAnomaly, "Uranus");
	sim.AddObjectKeplerian(3.014e21, 761400.0, Color{ 220, 215, 215, 255 }, &Objects::moon, "Oberon", 583520000.0, 0.0014, 0.058 * DEG2RAD, 279.77 * DEG2RAD, 104.40 * DEG2RAD, 14.7355 * DEG2RAD, MeanAnomaly, "Uranus");

	sim.AddObjectKeplerian(2.139e22, 1353400.0, Color{ 235, 220, 225, 255 }, &Objects::moon, "Triton", 354759000.0, 0.000016, 156.885 * DEG2RAD, 177.608 * DEG2RAD, 66.142 * DEG2RAD, 194.7109 * DEG2RAD, MeanAnomaly, "Neptune");
	sim.AddObjectKeplerian(4.4e19, 210000.0, Color{ 210, 215, 220, 255 }, &Objects::moon, "Proteus", 117647000.0, 0.00053, 0.524 * DEG2RAD, 15.0 * DEG2RAD, 120.0 * DEG2RAD, 155.7154 * DEG2RAD, MeanAnomaly, "Neptune");
	sim.AddObjectKeplerian(3.1e19, 170000.0, Color{ 215, 220, 225, 255 }, &Objects::moon, "Nereid", 5513818000.0, 0.7507, 7.232 * DEG2RAD, 326.0 * DEG2RAD, 290.0 * DEG2RAD, 318.2037 * DEG2RAD, MeanAnomaly, "Neptune");

	sim.AddObjectKeplerian(1.586e21, 606000.0, Color{ 215, 215, 220, 255 }, &Objects::moon, "Charon", 19591000.0, 0.0002, 0.080 * DEG2RAD, 223.046 * DEG2RAD, 150.0 * DEG2RAD, 338.7779 * DEG2RAD, MeanAnomaly, "Pluto");







	DisableCursor();
	SetTargetFPS(60);
	rlSetClipPlanes(1.0, 123123123123123123123123123123123123123.0);
#if defined(PLATFORM_WEB)
	// 0 = use browser's native refresh rate (recommended), 1 = simulate infinite loop
	emscripten_set_main_loop(UpdateGame, 0, 1);
#else
	while (!WindowShouldClose())
	{
		UpdateGame();
	}
#endif
	CloseWindow();
	return 0;
}
