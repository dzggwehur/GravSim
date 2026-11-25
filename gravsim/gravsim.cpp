/*
* ADVANCED 1: The world has many issues: from hunger, to supply chain shortages, to climate change. Create a piece of technology that assists with humanity’s pursuit to solve one of these problems. 
ADVANCED 2: Develop a guardrail on an online system or in the future which can help improve data privacy.
*/
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <raylib.h>
#include "Simulation.hpp"
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
double initialEnergy = 0.0;
int integratorGlobal = IAS15;
//constexpr double AU_TO_M = 149597870700.0;
Simulation sim;
#include <cmath>
#include <fstream>
#include <chrono>
double CalculateEnergy(const Object& a, const Object& b)
{
	const double dx = b.position.x - a.position.x;
	const double dy = b.position.y - a.position.y;
	const double dz = b.position.z - a.position.z;

	const double r = std::sqrt(dx * dx + dy * dy + dz * dz);

	const double v1Sq =
		a.velocity.x * a.velocity.x +
		a.velocity.y * a.velocity.y +
		a.velocity.z * a.velocity.z;

	const double v2Sq =
		b.velocity.x * b.velocity.x +
		b.velocity.y * b.velocity.y +
		b.velocity.z * b.velocity.z;

	const double kinetic =
		0.5 * a.mass * v1Sq +
		0.5 * b.mass * v2Sq;

	const double potential =
		-GRAVITATIONAL_CONSTANT * a.mass * b.mass / r;

	return kinetic + potential;
}
void GetSubstepLimit(int integrator)
{
	constexpr int iterations = 10000;
	constexpr double targetFrameTimeMs = 2.0;

	sim.AddObject(1.0,
		Vector3d(-5.0, 0.0, 0.0),
		Vector3d(0.0, 0.0, -std::sqrt((GRAVITATIONAL_CONSTANT) / 20.0)), 
		Vector3d{}, 
		Vector3d{},
		1.0, BLACK, &Objects::sun, "Test Object", false);

	sim.AddObject(1.0,
		Vector3d(5.0, 0.0, 0.0),
		Vector3d(0.0, 0.0, std::sqrt((GRAVITATIONAL_CONSTANT) / 20.0)),
		Vector3d{},
		Vector3d{},
		1.0, BLACK, &Objects::sun, "Test Object 2", false);

	// Warm-up
	for (int i = 0; i < 1000; ++i)
		sim.Update(objectToTrack, simTime, integrator, maxSubSteps);

	auto start = std::chrono::steady_clock::now();

	for (int i = 0; i < iterations; ++i)
		sim.Update(objectToTrack, simTime, integrator, maxSubSteps);

	auto end = std::chrono::steady_clock::now();

	double duration =
		std::chrono::duration<double, std::milli>(end - start).count();

	double averageUpdateMs = duration / iterations;

	// Estimate maximum updates within the target frame budget.
	maxSubSteps = std::clamp(
		static_cast<int>(targetFrameTimeMs / averageUpdateMs),
		1,
		500000
	);

	std::cout << "Benchmark: " << duration << " ms\n";
	std::cout << "Average update: " << averageUpdateMs * 1000.0 << " us\n";
	std::cout << "Estimated limit: " << maxSubSteps << '\n';

	simTime = 0.0;
	sim.simulationObjects.clear();
}
void UpdateGame()
{
	if (!paused)
	{
		sim.Update(objectToTrack, simTime, integratorGlobal, maxSubSteps);
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
			//sim.AddObject(createMass * 5.9720e24, cursor3D + realCameraPosition, Vector3d{}, createRadius, WHITE, &Objects::sun, floatText[2], false);
			placeMode = false;
		}
	}
	EndMode3D();
	sim.Draw2D(realCameraPosition, worldCamera);
	DrawText(std::string("Simulation Time: " + std::to_string(simTime) + " seconds").c_str(), 10, 10, 25, WHITE);
	DrawText(std::string("Simulation Timestep: " + std::to_string(sim.timestep) + " seconds per frame").c_str(), 10, 40, 25, WHITE);
	DrawText(std::string("Simulation Substeps: " + std::to_string(sim.substeps) + "/" + std::to_string(maxSubSteps/(sim.simulationObjects.size() * sim.simulationObjects.size())) + " substeps per frame").c_str(), 10, 70, 25, WHITE);
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
	GetSubstepLimit(integratorGlobal);
	constexpr double TARGET_PERIOD = 1000.0; // seconds per orbit
	constexpr double D_OVER_RS = 55.0;       // Semi-major axis = 55 Schwarzschild radii (visible inspiral in ~5-6 orbits)
	constexpr double eccentricity = 0.75;

	// Solve for exact black hole mass M that gives TARGET_PERIOD at D = D_OVER_RS * R_s
	// From Kepler's 3rd Law: T = 2*pi * sqrt(D^3 / (2*G*M)) with R_s = 2*G*M / c^2
	const double M = (TARGET_PERIOD * C * C * C) /
		(2.0 * PI * GRAVITATIONAL_CONSTANT * std::sqrt(2.0 * D_OVER_RS * D_OVER_RS * D_OVER_RS));

	// Real Schwarzschild radius in meters (~1.05e8 m)
	const double schwarzschildRadius = (2.0 * GRAVITATIONAL_CONSTANT * M) / (C * C);

	// Semi-major axis and periapsis start positions in meters
	const double D = D_OVER_RS * schwarzschildRadius;
	const double rPeriapsis = D * (1.0 - eccentricity); // 13.75 * R_s
	const double rStart = 0.5 * rPeriapsis;             // ~7.2e8 m from center of mass

	// Exact vis-viva periapsis speed in m/s (~0.36c relative)
	const double vRelPeriapsis = std::sqrt(GRAVITATIONAL_CONSTANT * (2.0 * M) * (2.0 / rPeriapsis - 1.0 / D));
	const double vPeriapsis = 0.5 * vRelPeriapsis;

	// Tilted spin for BH_A (angularSpin = 4th arg, orientation = 5th arg)
	sim.AddObject(M, Vector3d(-rStart, 0.0, 0.0),
		Vector3d(0.0, 0.0, -vPeriapsis),
		Vector3d(4.0, 3.0, 0.0),         // angularSpin (rad/s)
		Vector3d{},                      // initial orientation (radians)
		schwarzschildRadius, RED, &Objects::sun, "BH_A", false);

	// Opposite tilted spin for BH_B
	sim.AddObject(M, Vector3d(rStart, 0.0, 0.0),
		Vector3d(0.0, 0.0, vPeriapsis),
		Vector3d(-4.0, 3.0, 0.0),        // angularSpin (rad/s)
		Vector3d{},                      // initial orientation (radians)
		schwarzschildRadius, SKYBLUE, &Objects::sun, "BH_B", false);






	initialEnergy = CalculateEnergy(sim.simulationObjects[0], sim.simulationObjects[1]);
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
