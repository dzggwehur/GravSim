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
	sim.substeps = 1000;
	Objects::Load();
	worldCamera.position = Vector3{ -0.0f,0.0f,0.0f };
	worldCamera.target = Vector3{ 3.0f,0.0f,0.0f };
	worldCamera.projection = CAMERA_PERSPECTIVE;
	worldCamera.up = Vector3{ 0.0f,1.0f,0.0f };
	worldCamera.fovy = 45.0f;
	// Sun (Radius: 696,340 km)
// Didymos (Primary Asteroid: ~780 m diameter, mass ~5.2e11 kg)
// Sun
// Epoch: January 1, 2027 (00:00:00 TDB) — Solar System Barycentric Frame (Meters, Seconds, Kilograms)
	sim.AddObject(1.98850e+30, Vector3d(-1.05402778e+08, 1.23938681e+07, 7.28858712e+08), Vector3d(9.668742, -0.211191, -5.768981), 6.96340e+08, WHITE, &Objects::models[0], "Sun");

	// Inner Planets & Asteroid Belt Objects
	sim.AddObject(3.30110e+23, Vector3d(1.01611860e+10, -6.44706493e+09, 6.82463893e+10), Vector3d(38399.555829, -2718.677415, -9825.271987), 2.43970e+06, WHITE, &Objects::models[1], "Mercury");
	sim.AddObject(4.86750e+24, Vector3d(-8.40857039e+10, 5.77655983e+09, -6.61129148e+10), Vector3d(-21933.426208, 886.933228, 27574.505179), 6.05180e+06, WHITE, &Objects::models[2], "Venus");
	sim.AddObject(5.97219e+24, Vector3d(-2.55117385e+10, 3.78716467e+06, -1.44163827e+11), Vector3d(-29816.866884, 1.002046, 5239.682537), 6.37100e+06, WHITE, &Objects::models[3], "Earth");
	sim.AddObject(7.34770e+22, Vector3d(-2.58906177e+10, -1.85635975e+07, -1.44139296e+11), Vector3d(-29801.671321, -72.313353, 6272.428631), 1.73740e+06, WHITE, &Objects::models[9], "Moon");
	sim.AddObject(6.41710e+23, Vector3d(-1.53339789e+11, 7.80847693e+09, -1.91984380e+11), Vector3d(-18042.027476, 169.647627, 13010.402026), 3.38950e+06, WHITE, &Objects::models[4], "Mars");
	sim.AddObject(1.06590e+16, Vector3d(-1.53346482e+11, 7.81231684e+09, -1.91989723e+11), Vector3d(-19142.620557, 634.739864, 14779.967431), 1.12670e+04, GRAY, &Objects::models[9], "Phobos");
	sim.AddObject(1.47620e+15, Vector3d(-1.53318835e+11, 7.79795138e+09, -1.91983528e+11), Vector3d(-17954.383135, 234.099730, 11663.787465), 6.20000e+03, LIGHTGRAY, &Objects::models[9], "Deimos");
	sim.AddObject(9.38350e+20, Vector3d(2.77620994e+11, -4.28343631e+10, -2.64441232e+11), Vector3d(-13325.011037, 2880.920200, -13434.019296), 4.69700e+05, WHITE, &Objects::models[1], "Ceres");
	sim.AddObject(2.59080e+20, Vector3d(-3.23171334e+11, 3.98070800e+10, 1.74779989e+10), Vector3d(249.845385, 598.344082, 20929.417813), 2.62700e+05, LIGHTGRAY, &Objects::models[1], "Vesta");

	// Jupiter & Moons (4 Galilean + Amalthea inner + Himalia prograde irregular + Pasiphae retrograde irregular)
	sim.AddObject(1.89820e+27, Vector3d(-5.97218970e+11, 1.11737701e+10, -5.28410257e+11), Vector3d(-8822.608803, 235.613101, 9172.490490), 6.99110e+07, WHITE, &Objects::models[5], "Jupiter");
	sim.AddObject(2.08000e+18, Vector3d(-5.97277265e+11, 1.11790977e+10, -5.28582467e+11), Vector3d(-33773.720015, -260.199681, 17641.670747), 8.35000e+04, RED, &Objects::models[9], "Amalthea");
	sim.AddObject(8.93190e+22, Vector3d(-5.96863408e+11, 1.11870194e+10, -5.28636799e+11), Vector3d(-18201.975689, 635.369353, -5393.196808), 1.82160e+06, YELLOW, &Objects::models[9], "Io");
	sim.AddObject(4.79980e+22, Vector3d(-5.96645467e+11, 1.11751127e+10, -5.28072692e+11), Vector3d(-1757.835032, 722.881418, -2735.939412), 1.56080e+06, WHITE, &Objects::models[9], "Europa");
	sim.AddObject(1.48190e+23, Vector3d(-5.96772420e+11, 1.12154040e+10, -5.29383698e+11), Vector3d(-18698.602751, 291.527412, 4642.049062), 2.63410e+06, LIGHTGRAY, &Objects::models[9], "Ganymede");
	sim.AddObject(1.07590e+23, Vector3d(-5.98828550e+11, 1.11083076e+10, -5.27416052e+11), Vector3d(-4496.283843, 53.455287, 16087.223778), 2.41030e+06, GRAY, &Objects::models[9], "Callisto");
	sim.AddObject(4.20000e+18, Vector3d(-5.99752754e+11, 8.29619005e+09, -5.16655606e+11), Vector3d(-5972.972308, -906.050477, 9048.423487), 6.98000e+04, GRAY, &Objects::models[9], "Himalia");
	sim.AddObject(3.00000e+17, Vector3d(-5.68312615e+11, 3.47665584e+09, -5.17302579e+11), Vector3d(-8875.509237, 739.797209, 10695.939441), 2.89000e+04, DARKGRAY, &Objects::models[9], "Pasiphae");

	// Saturn & Moons (7 major rounded moons + Hyperion chaotic tumbler + Phoebe retrograde captured)
	sim.AddObject(5.68340e+26, Vector3d(1.36401859e+12, -6.01992359e+10, -3.40139116e+11), Vector3d(-2853.229697, -48.708886, -9354.850337), 5.82320e+07, WHITE, &Objects::models[6], "Saturn");
	sim.AddObject(3.74930e+19, Vector3d(1.36408064e+12, -6.01256527e+10, -3.39984656e+11), Vector3d(10839.073299, -3032.385161, -13357.976474), 1.98200e+05, LIGHTGRAY, &Objects::models[9], "Mimas");
	sim.AddObject(1.08020e+20, Vector3d(1.36382817e+12, -6.02512655e+10, -3.40273694e+11), Vector3d(-10346.665588, 5201.404898, -728.453018), 2.52100e+05, WHITE, &Objects::models[9], "Enceladus");
	sim.AddObject(6.17450e+20, Vector3d(1.36402870e+12, -6.03323320e+10, -3.40401731e+11), Vector3d(-14142.726224, 742.519583, -10190.487534), 5.31100e+05, WHITE, &Objects::models[9], "Tethys");
	sim.AddObject(1.09550e+21, Vector3d(1.36387083e+12, -6.03487693e+10, -3.40451537e+11), Vector3d(-12055.244452, 2487.692457, -6217.242052), 5.61400e+05, WHITE, &Objects::models[9], "Dione");
	sim.AddObject(2.30650e+21, Vector3d(1.36382740e+12, -6.04131952e+10, -3.40581531e+11), Vector3d(-10722.243031, 2021.385349, -6967.390814), 7.63800e+05, LIGHTGRAY, &Objects::models[9], "Rhea");
	sim.AddObject(1.34520e+23, Vector3d(1.36467680e+12, -5.97843726e+10, -3.39234781e+11), Vector3d(1826.845865, -1931.593403, -12021.044915), 2.57473e+06, ORANGE, &Objects::models[9], "Titan");
	sim.AddObject(5.61990e+18, Vector3d(1.36501426e+12, -5.97000335e+10, -3.39027778e+11), Vector3d(468.155767, -1874.363332, -12225.370032), 1.35000e+05, GRAY, &Objects::models[9], "Hyperion");
	sim.AddObject(1.80560e+21, Vector3d(1.36121310e+12, -5.88894184e+10, -3.38370729e+11), Vector3d(-924.502918, 1750.360587, -7439.769019), 7.34500e+05, LIGHTGRAY, &Objects::models[9], "Iapetus");
	sim.AddObject(8.29200e+18, Vector3d(1.36014836e+12, -5.23013468e+10, -3.28152056e+11), Vector3d(-4292.349558, -132.711471, -9667.440770), 1.06500e+05, DARKGRAY, &Objects::models[9], "Phoebe");

	// Uranus & Moons (5 major moons orbiting vertically due to 98° axial tilt + Sycorax retrograde irregular)
	sim.AddObject(8.68100e+25, Vector3d(1.28681641e+12, -7.00214203e+09, -2.60379213e+12), Vector3d(-6142.609939, 89.489600, -2703.523208), 2.53620e+07, WHITE, &Objects::models[7], "Uranus");
	sim.AddObject(6.40000e+19, Vector3d(1.28689242e+12, -6.89832511e+09, -2.60378013e+12), Vector3d(-11298.457918, 4053.381175, -4314.981877), 2.35800e+05, LIGHTGRAY, &Objects::models[9], "Miranda");
	sim.AddObject(1.25100e+21, Vector3d(1.28683198e+12, -7.19051072e+09, -2.60376342e+12), Vector3d(-780.894552, 692.689333, -1622.096803), 5.78900e+05, WHITE, &Objects::models[9], "Ariel");
	sim.AddObject(1.27500e+21, Vector3d(1.28707494e+12, -7.02815543e+09, -2.60373233e+12), Vector3d(-5578.116688, 4683.394225, -3209.765226), 5.84700e+05, GRAY, &Objects::models[9], "Umbriel");
	sim.AddObject(3.40000e+21, Vector3d(1.28653150e+12, -7.33233134e+09, -2.60380653e+12), Vector3d(-3487.363632, -2238.543901, -1811.913597), 7.88400e+05, LIGHTGRAY, &Objects::models[9], "Titania");
	sim.AddObject(3.07600e+21, Vector3d(1.28735810e+12, -6.80822803e+09, -2.60369981e+12), Vector3d(-7092.440683, 3034.682143, -3320.678071), 7.61400e+05, LIGHTGRAY, &Objects::models[9], "Oberon");
	sim.AddObject(2.30000e+18, Vector3d(1.29939062e+12, -7.47788791e+09, -2.60534928e+12), Vector3d(-6504.686640, -462.228737, -2651.505475), 7.85000e+04, DARKGRAY, &Objects::models[9], "Sycorax");

	// Neptune & Moons (Proteus inner + Triton large retrograde + Nereid extreme eccentricity e=0.75)
	sim.AddObject(1.02410e+26, Vector3d(4.46098520e+12, -1.07923391e+11, -2.48439698e+11), Vector3d(-328.704222, -104.754572, -5461.336060), 2.46220e+07, WHITE, &Objects::models[8], "Neptune");
	sim.AddObject(4.40000e+19, Vector3d(4.46093715e+12, -1.07941402e+11, -2.48333797e+11), Vector3d(6059.360178, -3554.624160, -3145.964735), 2.10000e+05, GRAY, &Objects::models[9], "Proteus");
	sim.AddObject(2.13900e+22, Vector3d(4.46128947e+12, -1.08103177e+11, -2.48470555e+11), Vector3d(-1457.699366, -2603.680150, -2033.243730), 1.35340e+06, WHITE, &Objects::models[9], "Triton");
	sim.AddObject(3.10000e+19, Vector3d(4.45747756e+12, -1.03898069e+11, -2.53792629e+11), Vector3d(-524.429914, -367.316952, -4782.187558), 1.78500e+05, LIGHTGRAY, &Objects::models[9], "Nereid");

	// Pluto & Moons (Binary Pluto-Charon + circumbinary Nix & Hydra)
	sim.AddObject(1.30300e+22, Vector3d(3.02321283e+12, -4.05837541e+11, 4.38051628e+12), Vector3d(4641.543245, -1560.601959, -1867.410644), 1.18830e+06, LIGHTGRAY, &Objects::models[1], "Pluto");
	sim.AddObject(1.58600e+21, Vector3d(3.02319917e+12, -4.05836251e+11, 4.38053027e+12), Vector3d(4589.035799, -1355.403135, -1937.541457), 6.06000e+05, GRAY, &Objects::models[9], "Charon");
	sim.AddObject(4.50000e+16, Vector3d(3.02321055e+12, -4.05878404e+11, 4.38054397e+12), Vector3d(4537.428722, -1488.038593, -1799.700010), 2.49000e+04, WHITE, &Objects::models[9], "Nix");
	sim.AddObject(4.80000e+16, Vector3d(3.02318973e+12, -4.05778152e+11, 4.38050466e+12), Vector3d(4712.378674, -1530.215593, -1962.438320), 2.54500e+04, WHITE, &Objects::models[9], "Hydra");


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