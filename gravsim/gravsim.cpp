/*
* ADVANCED 1: The world has many issues: from hunger, to supply chain shortages, to climate change. Create a piece of technology that assists with humanity’s pursuit to solve one of these problems. 
ADVANCED 2: Develop a guardrail on an online system or in the future which can help improve data privacy.
*/
#include <iostream>
#include <raylib.h>
#include "Simulation.h"
#include "Models.h"
#include "CameraAdvanced.h"
#include <rlgl.h>
#include <string>
#include <array>
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
std::array<char[1223], 5> floatText{ 0 };
float initialOrbitCameraDist = 0;
bool placeMode = false;
Simulation sim;
void UpdateGame()
{
	if (!paused)
	{
		sim.Update(objectToTrack);
		simTime += sim.timestep;
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
			sim.AddObject(createMass * 5.9720e24, cursor3D + realCameraPosition, Vector3d{}, createRadius, WHITE, &Objects::models[0], floatText[2]);
			placeMode = false;
		}
	}
	EndMode3D();
	sim.Draw2D(realCameraPosition, worldCamera);
	DrawText(std::string("Simulation Time: " + std::to_string(simTime) + " seconds").c_str(), 10, 10, 25, WHITE);
	if (objectToTrack != -1)
	{
		DrawText(std::string("Currently viewing object: " + sim.simulationObjects[objectToTrack].name).c_str(), 10, 70, 25, WHITE);
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
	if (GuiValueBoxFloat(Rectangle(10, 370, 120, 30), "Global Timestep (s/t)", floatText[3], &sim.timestep, highlight[4])) highlight[4] = !highlight[4];
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
	EndDrawing();
}

int main()
{
	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	
	InitWindow(400, 500, "Test");
	SetExitKey(KEY_F8);

	sim.timestep = 1.0f;
	sim.substeps = 10000;
	Objects::Load();
	worldCamera.position = Vector3{ -0.0f,0.0f,0.0f };
	worldCamera.target = Vector3{ 3.0f,0.0f,0.0f };
	worldCamera.projection = CAMERA_PERSPECTIVE;
	worldCamera.up = Vector3{ 0.0f,1.0f,0.0f };
	worldCamera.fovy = 45.0f;
	// Sun (Radius: 696,340 km)
// Didymos (Primary Asteroid: ~780 m diameter, mass ~5.2e11 kg)
// Sun
	sim.AddObject(1.9885e30, Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), 6.9634e8, WHITE, &Objects::models[0], "Sun");

	// Inner Planets & Asteroid Belt Objects
	sim.AddObject(3.3011e23, Vector3d(5.79e10, 0.0, 0.0), Vector3d(0.0, 0.0, 47362.0), 2.4397e6, WHITE, &Objects::models[1], "Mercury");
	sim.AddObject(4.8675e24, Vector3d(1.082e11, 0.0, 0.0), Vector3d(0.0, 0.0, 35020.0), 6.0518e6, WHITE, &Objects::models[2], "Venus");
	sim.AddObject(5.9720e24, Vector3d(1.4960e11, 0.0, 0.0), Vector3d(0.0, 0.0, 29785.1549), 6.371e6, WHITE, &Objects::models[3], "Earth");
	sim.AddObject(7.3477e22, Vector3d(1.492156e11, 0.0, 0.0), Vector3d(0.0, 0.0, 28766.8659), 1.7374e6, WHITE, &Objects::models[9], "Moon");
	sim.AddObject(6.4171e23, Vector3d(2.279e11, 0.0, 0.0), Vector3d(0.0, 0.0, 24077.0), 3.3895e6, WHITE, &Objects::models[4], "Mars");
	sim.AddObject(9.393e20, Vector3d(4.137e11, 0.0, 0.0), Vector3d(0.0, 0.0, 17905.0), 4.762e5, WHITE, &Objects::models[1], "Ceres");

	// Outer Planets (Gas and Ice Giants)
	sim.AddObject(1.8982e27, Vector3d(7.785e11, 0.0, 0.0), Vector3d(0.0, 0.0, 13070.0), 6.9911e7, WHITE, &Objects::models[5], "Jupiter");
	sim.AddObject(5.6834e26, Vector3d(1.433e12, 0.0, 0.0), Vector3d(0.0, 0.0, 9690.0), 5.8232e7, WHITE, &Objects::models[6], "Saturn");
	sim.AddObject(8.6810e25, Vector3d(2.871e12, 0.0, 0.0), Vector3d(0.0, 0.0, 6810.0), 2.5362e7, WHITE, &Objects::models[7], "Uranus");
	sim.AddObject(1.0241e26, Vector3d(4.495e12, 0.0, 0.0), Vector3d(0.0, 0.0, 5430.0), 2.4622e7, WHITE, &Objects::models[8], "Neptune");

	sim.AddObject(2.26195e14, Vector3d(0.0, 0.0, 1.4960e11), Vector3d(29770.0, 0.0, 0.0), 3000.0, GRAY, &Objects::models[2], "Asteroid");
	//Comment out the below to see the impact trajectory of the asteroid!
	sim.AddObject(5e9, Vector3d(0.0, 1.0e9, 1.4960e11), Vector3d(29770.0, -100000.0, 0.0), 500.0, RED, &Objects::models[2], "Deflector");

	DisableCursor();
	SetTargetFPS(62321320);
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