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
class Simulation
{
	public:
		float timestep = 1.0f;
		int substeps = 0;
		std::vector<Object> simulationObjects;
		void RK1()
		{
			double lowestDistance = 218323239*2313223231;
			for (Object& obj : simulationObjects)
			{

				obj.accel1 = {};
				obj.accel2 = {};
				obj.accel3 = {};
				obj.accel4 = {};
				
				for (const Object& obj2 : simulationObjects)
				{
					double dist = Vector3dDistance(obj.position, obj2.position);
					if (dist < 0.000001f) continue;
					double force = (GRAVITATIONAL_CONSTANT * obj2.mass) / (dist * dist);
					obj.accel1 += Vector3dNormalize(Vector3dSubtract(obj2.position, obj.position))*force;

				}
			}
		}
		void RK2(float step)
		{
			for (Object& obj : simulationObjects)
			{
				obj.position2 = obj.position + obj.velocity * (step / 2);
				obj.velocity2 = obj.velocity + obj.accel1 * (step / 2);
			}
			for (Object& obj : simulationObjects)
			{
				for (const Object& obj2 : simulationObjects)
				{
					double dist = Vector3dDistance(obj.position2, obj2.position2);
					if (dist < 0.000001f) continue;
					double force = (GRAVITATIONAL_CONSTANT * obj2.mass) / (dist * dist);
					obj.accel2 += Vector3dNormalize(Vector3dSubtract(obj2.position2, obj.position2)) * force;
				}
			}
		}
		void RK3(float step)
		{
			for (Object& obj : simulationObjects)
			{
				obj.position3 = obj.position + obj.velocity2 * (step / 2);
				obj.velocity3 = obj.velocity + obj.accel2 * (step / 2);
			}
			for (Object& obj : simulationObjects)
			{
				for (const Object& obj2 : simulationObjects)
				{
					double dist = Vector3dDistance(obj.position3, obj2.position3);
					if (dist < 0.000001f) continue;
					double force = (GRAVITATIONAL_CONSTANT * obj2.mass) / (dist * dist);
					obj.accel3 += Vector3dNormalize(Vector3dSubtract(obj2.position3, obj.position3)) * force;
				}
			}
		}
		void RK4(float step)
		{
			for (Object& obj : simulationObjects)
			{
				obj.position4 = obj.position + obj.velocity3 * step;
				obj.velocity4 = obj.velocity + obj.accel3 * step;
			}
			for (Object& obj : simulationObjects)
			{
				
				for (const Object& obj2 : simulationObjects)
				{
					double dist = Vector3dDistance(obj.position4, obj2.position4);
					if (dist < 0.000001f) continue;
					double force = (GRAVITATIONAL_CONSTANT * obj2.mass) / (dist * dist);
					obj.accel4 += Vector3dNormalize(Vector3dSubtract(obj2.position4, obj.position4)) * force;
				}
			}
		}
		void Final(float step, int& track)
		{
			for (Object& obj : simulationObjects)
			{
				obj.position += (step / 6) * (obj.velocity + 2 * obj.velocity2 + 2 * obj.velocity3 + obj.velocity4);
				obj.velocity += (step / 6) * (obj.accel1 + 2 * obj.accel2 + 2 * obj.accel3 + obj.accel4);

			}
			

			//Check collision
			for (auto [index, obj] : std::views::enumerate(simulationObjects))
			{
				for (auto [index2, obj2] : std::views::enumerate(simulationObjects))
				{
					double dist = Vector3dDistance(obj.position, obj2.position);
					if (&obj2 == &obj || obj2.destroyed) continue;
					if (dist < obj.radius+obj2.radius)
					{
						std::cout << "Collision!";
						if (obj.mass > obj2.mass)
						{
							if (track == index2)
							{
								track = index;
							}
							float impulse = obj2.mass * Vector3dLength(obj.velocity - obj2.velocity);
							obj.velocity += (Vector3dNormalize(obj.position - obj2.position) * impulse)/obj.mass;
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
			double lowestDistance = 218323239 * 2313223231;
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
				RK1();
				RK2(time);
				RK3(time);
				RK4(time);
				Final(time, objectToTrack);
			}
			for (Object& item : simulationObjects)
			{
				item.trail.push_back(item.position);
				if (item.trail.size() > 100000) item.trail.pop_front();
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
					for (size_t i = 0; i < obj.trail.size() - 1; ++i) {
						DrawLine3D(ConvertRegular(obj.trail[i] - realPosition), ConvertRegular(obj.trail[i + 1] - realPosition), Color{ 255, 0, 0, 255 });
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