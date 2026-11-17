#pragma once
#include "Object.h"
#include <deque>
#include <raymath.h>
#include <raylib.h>
#include "Constants.h"
#include "raymathDouble.h"
#include <iostream>
#include <ranges>
#include <algorithm>
#include <vector>
#include <functional>
class Simulation
{
	public:
		float timestep = 1.0f;
		int substeps = 0;
		std::vector<Object> simulationObjects;
		//__declspec(noinline)
		Vector3d calcAccel(const Vector3d& pos, Vector3d Object::* stagePos, double mass) const
		{
			double ax = 0.0, ay = 0.0, az = 0.0;
			const double pullMass = mass * 1e-12;
			const int count = static_cast<int>(simulationObjects.size());

			for (int j = 0; j < count; ++j)
			{
				const Object& obj2 = simulationObjects[j];
				if (obj2.mass < pullMass) continue;

				const Vector3d otherPos = obj2.*stagePos;
				const double dx = otherPos.x - pos.x;
				const double dy = otherPos.y - pos.y;
				const double dz = otherPos.z - pos.z;

				const double distSq = dx * dx + dy * dy + dz * dz;
				if (distSq < 1e-12) continue;

				const double dist = std::sqrt(distSq);
				const double accelMag = (GRAVITATIONAL_CONSTANT * obj2.mass) / (distSq * dist);

				ax = std::fma(dx, accelMag, ax);
				ay = std::fma(dy, accelMag, ay);
				az = std::fma(dz, accelMag, az);
			}
			return { ax, ay, az };
		}



		void RK8Full(float step)
		{
			// --- STATE 1 ---
			for (Object& obj : simulationObjects)
			{
				obj.accel1 = calcAccel(obj.position, &Object::position, obj.mass);
			}

			// --- STATE 2 ---
			for (Object& obj : simulationObjects)
			{
				obj.position2 = obj.position + obj.velocity * (step / 2.0);
				obj.velocity2 = obj.velocity + obj.accel1 * (step / 2.0);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel2 = calcAccel(obj.position2, &Object::position2, obj.mass);
			}

			// --- STATE 3 ---
			for (Object& obj : simulationObjects)
			{
				obj.position3 = obj.position + step * (0.25 * obj.velocity + 0.25 * obj.velocity2);
				obj.velocity3 = obj.velocity + step * (0.25 * obj.accel1 + 0.25 * obj.accel2);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel3 = calcAccel(obj.position3, &Object::position3, obj.mass);
			}

			// --- STATE 4 ---
			for (Object& obj : simulationObjects)
			{
				obj.position4 = obj.position + step * ((1.0 / 7.0) * obj.velocity + ((-7.0 - 3.0 * LOBATTO_SQRT21) / 98.0) * obj.velocity2 + ((21.0 + 5.0 * LOBATTO_SQRT21) / 49.0) * obj.velocity3);
				obj.velocity4 = obj.velocity + step * ((1.0 / 7.0) * obj.accel1 + ((-7.0 - 3.0 * LOBATTO_SQRT21) / 98.0) * obj.accel2 + ((21.0 + 5.0 * LOBATTO_SQRT21) / 49.0) * obj.accel3);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel4 = calcAccel(obj.position4, &Object::position4, obj.mass);
			}

			// --- STATE 5 ---
			for (Object& obj : simulationObjects)
			{
				obj.position5 = obj.position + step * (((11.0 + LOBATTO_SQRT21) / 84.0) * obj.velocity + ((18.0 + 4.0 * LOBATTO_SQRT21) / 63.0) * obj.velocity3 + ((21.0 - LOBATTO_SQRT21) / 252.0) * obj.velocity4);
				obj.velocity5 = obj.velocity + step * (((11.0 + LOBATTO_SQRT21) / 84.0) * obj.accel1 + ((18.0 + 4.0 * LOBATTO_SQRT21) / 63.0) * obj.accel3 + ((21.0 - LOBATTO_SQRT21) / 252.0) * obj.accel4);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel5 = calcAccel(obj.position5, &Object::position5, obj.mass);
			}

			// --- STATE 6 ---
			for (Object& obj : simulationObjects)
			{
				obj.position6 = obj.position + step * (((5.0 + LOBATTO_SQRT21) / 48.0) * obj.velocity + ((9.0 + LOBATTO_SQRT21) / 36.0) * obj.velocity3 + ((-231.0 + 14.0 * LOBATTO_SQRT21) / 360.0) * obj.velocity4 + ((63.0 - 7.0 * LOBATTO_SQRT21) / 80.0) * obj.velocity5);
				obj.velocity6 = obj.velocity + step * (((5.0 + LOBATTO_SQRT21) / 48.0) * obj.accel1 + ((9.0 + LOBATTO_SQRT21) / 36.0) * obj.accel3 + ((-231.0 + 14.0 * LOBATTO_SQRT21) / 360.0) * obj.accel4 + ((63.0 - 7.0 * LOBATTO_SQRT21) / 80.0) * obj.accel5);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel6 = calcAccel(obj.position6, &Object::position6, obj.mass);
			}

			// --- STATE 7 ---
			for (Object& obj : simulationObjects)
			{
				obj.position7 = obj.position + step * (((10.0 - LOBATTO_SQRT21) / 42.0) * obj.velocity + ((-432.0 + 92.0 * LOBATTO_SQRT21) / 315.0) * obj.velocity3 + ((633.0 - 145.0 * LOBATTO_SQRT21) / 90.0) * obj.velocity4 + ((-504.0 + 115.0 * LOBATTO_SQRT21) / 70.0) * obj.velocity5 + ((63.0 - 13.0 * LOBATTO_SQRT21) / 35.0) * obj.velocity6);
				obj.velocity7 = obj.velocity + step * (((10.0 - LOBATTO_SQRT21) / 42.0) * obj.accel1 + ((-432.0 + 92.0 * LOBATTO_SQRT21) / 315.0) * obj.accel3 + ((633.0 - 145.0 * LOBATTO_SQRT21) / 90.0) * obj.accel4 + ((-504.0 + 115.0 * LOBATTO_SQRT21) / 70.0) * obj.accel5 + ((63.0 - 13.0 * LOBATTO_SQRT21) / 35.0) * obj.accel6);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel7 = calcAccel(obj.position7, &Object::position7, obj.mass);
			}

			// --- STATE 8 ---
			for (Object& obj : simulationObjects)
			{
				obj.position8 = obj.position + step * ((1.0 / 14.0) * obj.velocity + ((14.0 - 3.0 * LOBATTO_SQRT21) / 126.0) * obj.velocity5 + ((13.0 - 3.0 * LOBATTO_SQRT21) / 63.0) * obj.velocity6 + (1.0 / 9.0) * obj.velocity7);
				obj.velocity8 = obj.velocity + step * ((1.0 / 14.0) * obj.accel1 + ((14.0 - 3.0 * LOBATTO_SQRT21) / 126.0) * obj.accel5 + ((13.0 - 3.0 * LOBATTO_SQRT21) / 63.0) * obj.accel6 + (1.0 / 9.0) * obj.accel7);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel8 = calcAccel(obj.position8, &Object::position8, obj.mass);
			}

			// --- STATE 9 ---
			for (Object& obj : simulationObjects)
			{
				obj.position9 = obj.position + step * ((1.0 / 32.0) * obj.velocity + ((91.0 - 21.0 * LOBATTO_SQRT21) / 576.0) * obj.velocity5 + (11.0 / 72.0) * obj.velocity6 + ((-385.0 - 75.0 * LOBATTO_SQRT21) / 1152.0) * obj.velocity7 + ((63.0 + 13.0 * LOBATTO_SQRT21) / 128.0) * obj.velocity8);
				obj.velocity9 = obj.velocity + step * ((1.0 / 32.0) * obj.accel1 + ((91.0 - 21.0 * LOBATTO_SQRT21) / 576.0) * obj.accel5 + (11.0 / 72.0) * obj.accel6 + ((-385.0 - 75.0 * LOBATTO_SQRT21) / 1152.0) * obj.accel7 + ((63.0 + 13.0 * LOBATTO_SQRT21) / 128.0) * obj.accel8);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel9 = calcAccel(obj.position9, &Object::position9, obj.mass);
			}

			// --- STATE 10 ---
			for (Object& obj : simulationObjects)
			{
				obj.position10 = obj.position + step * ((1.0 / 14.0) * obj.velocity + (1.0 / 9.0) * obj.velocity5 + ((-733.0 - 147.0 * LOBATTO_SQRT21) / 2205.0) * obj.velocity6 + ((515.0 + 111.0 * LOBATTO_SQRT21) / 504.0) * obj.velocity7 + ((-51.0 - 11.0 * LOBATTO_SQRT21) / 56.0) * obj.velocity8 + ((132.0 + 28.0 * LOBATTO_SQRT21) / 245.0) * obj.velocity9);
				obj.velocity10 = obj.velocity + step * ((1.0 / 14.0) * obj.accel1 + (1.0 / 9.0) * obj.accel5 + ((-733.0 - 147.0 * LOBATTO_SQRT21) / 2205.0) * obj.accel6 + ((515.0 + 111.0 * LOBATTO_SQRT21) / 504.0) * obj.accel7 + ((-51.0 - 11.0 * LOBATTO_SQRT21) / 56.0) * obj.accel8 + ((132.0 + 28.0 * LOBATTO_SQRT21) / 245.0) * obj.accel9);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel10 = calcAccel(obj.position10, &Object::position10, obj.mass);
			}

			// --- STATE 11 ---
			for (Object& obj : simulationObjects)
			{
				obj.position11 = obj.position + step * (((-42.0 + 7.0 * LOBATTO_SQRT21) / 18.0) * obj.velocity5 + ((-18.0 + 28.0 * LOBATTO_SQRT21) / 45.0) * obj.velocity6 + ((-273.0 - 53.0 * LOBATTO_SQRT21) / 72.0) * obj.velocity7 + ((301.0 + 53.0 * LOBATTO_SQRT21) / 72.0) * obj.velocity8 + ((28.0 - 28.0 * LOBATTO_SQRT21) / 45.0) * obj.velocity9 + ((49.0 - 7.0 * LOBATTO_SQRT21) / 18.0) * obj.velocity10);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel11 = calcAccel(obj.position11, &Object::position11, obj.mass);
			}

			// --- FINAL UPDATE ---
			for (Object& obj : simulationObjects)
			{
				obj.position += (step / 180.0) * (9.0 * obj.velocity + 49.0 * obj.velocity8 + 64.0 * obj.velocity9 + 49.0 * obj.velocity10 + 9.0 * obj.velocity + step * (((-42.0 + 7.0 * LOBATTO_SQRT21) / 18.0) * obj.accel5 + ((-18.0 + 28.0 * LOBATTO_SQRT21) / 45.0) * obj.accel6 + ((-273.0 - 53.0 * LOBATTO_SQRT21) / 72.0) * obj.accel7 + ((301.0 + 53.0 * LOBATTO_SQRT21) / 72.0) * obj.accel8 + ((28.0 - 28.0 * LOBATTO_SQRT21) / 45.0) * obj.accel9 + ((49.0 - 7.0 * LOBATTO_SQRT21) / 18.0) * obj.accel10));
				obj.velocity += (step / 180.0) * (9.0 * obj.accel1 + 49.0 * obj.accel8 + 64.0 * obj.accel9 + 49.0 * obj.accel10 + 9.0 * obj.accel11);
			}

		}
		void Collision(int& track)
		{
			//Check collision
			for (auto [index, obj] : std::views::enumerate(simulationObjects))
			{
				for (auto [index2, obj2] : std::views::enumerate(simulationObjects))
				{
					double dist = Vector3dDistance(obj.position, obj2.position);
					if (&obj2 == &obj || obj2.destroyed) continue;
					if (dist < obj.radius + obj2.radius)
					{
						std::cout << "Collision!";
						if (obj.mass > obj2.mass)
						{
							if (track == index2)
							{
								track = index;
							}
							float impulse = obj2.mass * Vector3dLength(obj.velocity - obj2.velocity);
							obj.velocity += (Vector3dNormalize(obj.position - obj2.position) * impulse) / obj.mass;
							obj2.destroyed = true;
						}
						else
						{
							if (track == index)
							{
								track = index2;
							}
							float impulse = obj.mass * Vector3dLength(obj.velocity - obj2.velocity);
							obj2.AppliedImpulse += (Vector3dNormalize(obj2.position - obj.position) * impulse) / obj2.mass;
							obj.destroyed = true;
						}
						if (obj.destroyed == true || obj2.destroyed == true)
						{
							break;
						}
					}
				}
			}
			std::erase_if(simulationObjects, [](const Object& obj) { return obj.destroyed; });
		}
		void Update(int& objectToTrack)
		{
			double slowDownThreshold = 15000000.0;
			double lowestDistance = 218323239.0 * 2313223231.0;
			for (Object& obj : simulationObjects)
			{
				for (const Object& obj2 : simulationObjects)
				{
					double dist = Vector3dDistance(obj.position, obj2.position);
					if (dist < lowestDistance)
					{
						if (dist < 0.000001f) continue;
						lowestDistance = dist;
					}
				}
			}
			double ratio = std::clamp(lowestDistance / slowDownThreshold, 0.0, 1.0);

			float timestep2 = std::clamp((float)(1 * ratio), 0.00001f, 1.0f);
			double time = (timestep / substeps)*timestep2;
			for (int i = 0; i < substeps; i++)
			{
				RK8Full(time);
				Collision(objectToTrack);
			}
			for (Object& item : simulationObjects)
			{
				item.trail.push_back(item.position);
				if (item.trail.size() > 1000) item.trail.pop_front();
			}
		}
		void AddObject(float mass, Vector3d position, Vector3d initialVelocity, float radius, Color color,Model* model,std::string name)
		{
			
			Object obj;
			obj.mass = mass;
			obj.position = position;
			obj.velocity = initialVelocity;
			obj.radius = radius;
			obj.color = color;
			obj.model = model;
			obj.name = name;
			simulationObjects.emplace_back(obj);
		}
		void Draw(Vector3d realPosition)
		{
			for (const Object& obj : simulationObjects)
			{
				DrawModel(*obj.model, static_cast<Vector3>(obj.position - realPosition), (float)obj.radius, obj.color);
				//trails
				if (obj.trail.size() >= 2) {
					Vector3 prev = ConvertRegular(obj.trail[0] - realPosition);
					for (size_t i = 1; i < obj.trail.size(); ++i) {
						Vector3 curr = ConvertRegular(obj.trail[i] - realPosition);
						DrawLine3D(prev, curr, Color{ 255, 0, 0, 255 });
						prev = curr;
					}
				}

			}
		}
		void Draw2D(Vector3d realPosition, Camera cam)
		{
			for (const Object& obj : simulationObjects)
			{
				Vector2 screenPos = GetWorldToScreen(static_cast<Vector3>(obj.position - realPosition), cam);
				Vector3 toTarget = Vector3Subtract(static_cast<Vector3>(obj.position - realPosition), cam.position);
				Vector3 camForward = Vector3Subtract(cam.target, Vector3{ 0 });
				if (Vector3DotProduct(toTarget, camForward) > 0) {
					if (screenPos.x > 0 && screenPos.y > 0) DrawText(obj.name.c_str(), screenPos.x - (MeasureText(obj.name.c_str(), 25) / 2), screenPos.y, 25, WHITE);
				}
			}
		}
};