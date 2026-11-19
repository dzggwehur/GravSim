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
std::array<char[1223], 5> floatText{};
float initialOrbitCameraDist = 0;
bool placeMode = false;
Simulation sim;
void UpdateGame()
{
	if (!paused)
	{
		sim.Update(objectToTrack, simTime);
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
	EndDrawing();
}

int main()
{
	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	
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
	// Sun (Centered at origin)
	sim.AddObject(1.98850e+30, Vector3d(0, 0, 0), Vector3d(0, 0, 0), 6.96340e+08, YELLOW, &Objects::models[0], "Sun");

	// Mercury & Venus
	sim.AddObject(3.30110e+23, Vector3d(-5.67085190e+10, -5.85319600e+09, -2.76114410e+10), Vector3d(11820.130000, -1630.520000, -45560.410000), 2.43970e+06, GRAY, &Objects::models[1], "Mercury");
	sim.AddObject(4.86750e+24, Vector3d(-1.07512320e+11, 6.23145100e+09, -1.21432110e+10), Vector3d(3890.120000, -198.450000, -34780.910000), 6.05180e+06, Color{ 250, 222, 112,255 }, &Objects::models[2], "Venus");

	// Earth & Moon
	sim.AddObject(7.34200e+22, Vector3d(-2.48104890e+10, 3.56214000e+07, 1.47412340e+11), Vector3d(-29810.450000, 92.180000, -6138.120000), 1.73740e+06, LIGHTGRAY, &Objects::models[9], "Moon");
	sim.AddObject(5.97237e+24, Vector3d(-2.51943210e+10, 1.12340000e+06, 1.47412340e+11), Vector3d(-29810.450000, 0.420000, -5120.330000), 6.37100e+06, BLUE, &Objects::models[3], "Earth");
	

	// Mars, Phobos & Deimos
	sim.AddObject(1.06590e+16, Vector3d(2.07900606e+11, -3.45105740e+09, 1.12341230e+11), Vector3d(-10450.220000, 412.110000, 20052.880000), 1.12670e+04, DARKGRAY, &Objects::models[9], "Phobos");
	sim.AddObject(1.47620e+15, Vector3d(2.07891230e+11, -3.45050420e+09, 1.12364689e+11), Vector3d(-11801.520000, 412.110000, 22190.880000), 6.20000e+03, LIGHTGRAY, &Objects::models[9], "Deimos");
	sim.AddObject(6.41710e+23, Vector3d(2.07891230e+11, -3.45123400e+09, 1.12341230e+11), Vector3d(-10450.220000, 412.110000, 22190.880000), 3.38950e+06, RED, &Objects::models[4], "Mars");
	

	// Jupiter & Galilean Moons
	sim.AddObject(8.93190e+22, Vector3d(-5.96863408e+11, 1.11870194e+10, -5.28636799e+11), Vector3d(-18201.975689, 635.369353, -5393.196808), 1.82160e+06, YELLOW, &Objects::models[9], "Io");
	sim.AddObject(4.79980e+22, Vector3d(-5.96645467e+11, 1.11751127e+10, -5.28072692e+11), Vector3d(-1757.835032, 722.881418, -2735.939412), 1.56080e+06, SKYBLUE, &Objects::models[9], "Europa");
	sim.AddObject(1.48190e+23, Vector3d(-5.96772420e+11, 1.12154040e+10, -5.29383698e+11), Vector3d(-18698.602751, 291.527412, 4642.049062), 2.63410e+06, LIGHTGRAY, &Objects::models[9], "Ganymede");
	sim.AddObject(1.07590e+23, Vector3d(-5.98828550e+11, 1.11083076e+10, -5.27416052e+11), Vector3d(-4496.283843, 53.455287, 16087.223778), 2.41030e+06, GRAY, &Objects::models[9], "Callisto");
	sim.AddObject(1.89820e+27, Vector3d(-5.97218970e+11, 1.11737701e+10, -5.28410257e+11), Vector3d(-8822.608803, 235.613101, 9172.490490), 6.99110e+07, Color{ 255, 207, 153,255 }, &Objects::models[5], "Jupiter");
	

	// Saturn & Major Moons
	sim.AddObject(3.74930e+19, Vector3d(9.58044007e+11, -4.12357223e+10, 1.08918696e+12), Vector3d(-12565.837145, 701.740732, -7394.653761), 1.98200e+05, LIGHTGRAY, &Objects::models[9], "Mimas");
	sim.AddObject(1.08022e+20, Vector3d(9.57989714e+11, -4.12345869e+10, 1.08904238e+12), Vector3d(-3130.111728, 308.960639, -5588.957550), 2.52100e+05, WHITE, &Objects::models[9], "Enceladus");
	sim.AddObject(6.17449e+20, Vector3d(9.58383484e+11, -4.12321271e+10, 1.08936478e+12), Vector3d(-16703.731480, 111.866970, 12737.007824), 5.31100e+05, SKYBLUE, &Objects::models[9], "Tethys");
	sim.AddObject(1.09545e+21, Vector3d(9.57889957e+11, -4.12345047e+10, 1.08892984e+12), Vector3d(-2268.759249, 309.898553, -2359.926744), 5.61400e+05, GRAY, &Objects::models[9], "Dione");
	sim.AddObject(2.30652e+21, Vector3d(9.57688164e+11, -4.12351591e+10, 1.08910604e+12), Vector3d(-7122.416210, 262.688566, -2254.825838), 7.63800e+05, DARKGRAY, &Objects::models[9], "Rhea");
	sim.AddObject(1.34520e+23, Vector3d(9.59434097e+11, -4.12364306e+10, 1.08942353e+12), Vector3d(-8667.146483, 344.775410, 11500.919722), 2.57473e+06, Color{ 255, 190, 120 ,255 }, &Objects::models[9], "Titan");
	sim.AddObject(1.80564e+21, Vector3d(9.60829528e+11, -4.18529320e+10, 1.09135088e+12), Vector3d(-9381.683626, 964.375569, 8832.744186), 7.34500e+05, BROWN, &Objects::models[9], "Iapetus");
	sim.AddObject(5.68340e+26, Vector3d(9.58214560e+11, -4.12345600e+10, 1.08912340e+12), Vector3d(-7410.550000, 312.880000, 6230.140000), 5.82320e+07, BEIGE, &Objects::models[6], "Saturn");
	

	// Uranus & Major Moons
	sim.AddObject(6.59000e+19, Vector3d(2.14580875e+12, -2.89027350e+10, -1.98765430e+12), Vector3d(4580.120000, -540.420000, -1754.210000), 2.35800e+05, GRAY, &Objects::models[9], "Miranda");
	sim.AddObject(1.25100e+21, Vector3d(2.14567890e+12, -2.87214400e+10, -1.98765430e+12), Vector3d(10088.910000, -58.910000, 4910.440000), 5.78900e+05, LIGHTGRAY, &Objects::models[9], "Ariel");
	sim.AddObject(1.27500e+21, Vector3d(2.14541290e+12, -2.89123400e+10, -1.98765430e+12), Vector3d(4580.120000, -58.910000, 9577.120000), 5.84700e+05, DARKGRAY, &Objects::models[9], "Umbriel");
	sim.AddObject(3.40000e+21, Vector3d(2.14567890e+12, -2.93486400e+10, -1.98765430e+12), Vector3d(936.440000, -58.910000, 4910.440000), 7.88900e+05, BEIGE, &Objects::models[9], "Titania");
	sim.AddObject(3.07600e+21, Vector3d(2.14626242e+12, -2.89123400e+10, -1.98765430e+12), Vector3d(4580.120000, -3209.140000, 4910.440000), 7.61400e+05, BROWN, &Objects::models[9], "Oberon");
	sim.AddObject(8.68100e+25, Vector3d(2.14567890e+12, -2.89123400e+10, -1.98765430e+12), Vector3d(4580.120000, -58.910000, 4910.440000), 2.53620e+07, SKYBLUE, &Objects::models[7], "Uranus");
	

	// Neptune, Triton & Proteus
	sim.AddObject(5.00000e+19, Vector3d(4.46120883e+12, -9.87654755e+10, -4.12230842e+11), Vector3d(7923.993369, -116.770553, 7072.592670), 2.10000e+05, GRAY, &Objects::models[9], "Proteus");
	sim.AddObject(2.13900e+22, Vector3d(4.46115726e+12, -9.87435829e+10, -4.12691227e+11), Vector3d(4468.856970, 1556.632966, 4628.191212), 1.35340e+06, VIOLET, &Objects::models[9], "Triton");
	sim.AddObject(1.02409e+26, Vector3d(4.46123450e+12, -9.87654300e+10, -4.12345670e+11), Vector3d(485.660000, -112.340000, 5412.890000), 2.46220e+07, DARKBLUE, &Objects::models[8], "Neptune");
	

	// Pluto & Charon
	sim.AddObject(1.58600e+21, Vector3d(2.38453159e+12, -3.61245000e+11, 5.21489000e+12), Vector3d(-4185.400000, 1409.120000, 1986.850000), 6.06000e+05, LIGHTGRAY, &Objects::models[9], "Charon");

	sim.AddObject(1.30300e+22, Vector3d(2.38451200e+12, -3.61245000e+11, 5.21489000e+12), Vector3d(-4185.400000, 1215.600000, 1892.300000), 1.18830e+06, BEIGE, &Objects::models[9], "Pluto");






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