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
#include <math.h>
#include <cmath>
class Simulation
{
	public:
		double timestep = 1.0;
		double intendedTimestep = 1.0;
		int substeps = 0;
		double substepSize = 1.0;
		double tolerance = 1e-12;

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
		void RK2Full(double step)
		{
			// --- STAGE 1 (Start of step) ---
			for (Object& obj : simulationObjects)
			{
				obj.accel1 = calcAccel(obj.position, &Object::position, obj.mass);
			}

			// --- STAGE 2 (Half-step midpoint) ---
			for (Object& obj : simulationObjects)
			{
				obj.position2 = obj.position + obj.velocity * (step * 0.5);
				obj.velocity2 = obj.velocity + obj.accel1 * (step * 0.5);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel2 = calcAccel(obj.position2, &Object::position2, obj.mass);
			}

			// --- FINAL UPDATE ---
			for (Object& obj : simulationObjects)
			{
				obj.position += step * obj.velocity2;
				obj.velocity += step * obj.accel2;
			}
		}
		void RK4Full(double step)
		{
			for (Object& obj : simulationObjects)
			{
				obj.accel1 = calcAccel(obj.position, &Object::position, obj.mass);
			}
			for (Object& obj : simulationObjects)
			{
				obj.position2 = obj.position + obj.velocity * (step / 2);
				obj.velocity2 = obj.velocity + obj.accel1 * (step / 2);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel2 = calcAccel(obj.position2, &Object::position2, obj.mass);
			}
			for (Object& obj : simulationObjects)
			{
				obj.position3 = obj.position + obj.velocity2 * (step / 2);
				obj.velocity3 = obj.velocity + obj.accel2 * (step / 2);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel3 = calcAccel(obj.position3, &Object::position3, obj.mass);
			}

			for (Object& obj : simulationObjects)
			{
				obj.position4 = obj.position + obj.velocity3 * step;
				obj.velocity4 = obj.velocity + obj.accel3 * step;
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel4 = calcAccel(obj.position4, &Object::position4, obj.mass);
			}
			for (Object& obj : simulationObjects)
			{
				obj.position += (step / 6) * (obj.velocity + 2 * obj.velocity2 + 2 * obj.velocity3 + obj.velocity4);
				obj.velocity += (step / 6) * (obj.accel1 + 2 * obj.accel2 + 2 * obj.accel3 + obj.accel4);

			}
		}
		double RK78Full(double step, double tolerance)
		{
			// --- STATE 1 ---
			for (Object& obj : simulationObjects)
			{
				obj.accel1 = calcAccel(obj.position, &Object::position, obj.mass);
			}

			// --- STATE 2 ---
			for (Object& obj : simulationObjects)
			{
				obj.position2 = obj.position + step * ((1.0 / 18.0) * obj.velocity);
				obj.velocity2 = obj.velocity + step * ((1.0 / 18.0) * obj.accel1);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel2 = calcAccel(obj.position2, &Object::position2, obj.mass);
			}

			// --- STATE 3 ---
			for (Object& obj : simulationObjects)
			{
				obj.position3 = obj.position + step * ((1.0 / 48.0) * obj.velocity + (1.0 / 16.0) * obj.velocity2);
				obj.velocity3 = obj.velocity + step * ((1.0 / 48.0) * obj.accel1 + (1.0 / 16.0) * obj.accel2);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel3 = calcAccel(obj.position3, &Object::position3, obj.mass);
			}

			// --- STATE 4 ---
			for (Object& obj : simulationObjects)
			{
				obj.position4 = obj.position + step * ((1.0 / 32.0) * obj.velocity + (3.0 / 32.0) * obj.velocity3);
				obj.velocity4 = obj.velocity + step * ((1.0 / 32.0) * obj.accel1 + (3.0 / 32.0) * obj.accel3);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel4 = calcAccel(obj.position4, &Object::position4, obj.mass);
			}

			// --- STATE 5 ---
			for (Object& obj : simulationObjects)
			{
				obj.position5 = obj.position + step * ((5.0 / 16.0) * obj.velocity - (75.0 / 64.0) * obj.velocity3 + (75.0 / 64.0) * obj.velocity4);
				obj.velocity5 = obj.velocity + step * ((5.0 / 16.0) * obj.accel1 - (75.0 / 64.0) * obj.accel3 + (75.0 / 64.0) * obj.accel4);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel5 = calcAccel(obj.position5, &Object::position5, obj.mass);
			}

			// --- STATE 6 ---
			for (Object& obj : simulationObjects)
			{
				obj.position6 = obj.position + step * ((3.0 / 80.0) * obj.velocity + (3.0 / 16.0) * obj.velocity4 + (3.0 / 20.0) * obj.velocity5);
				obj.velocity6 = obj.velocity + step * ((3.0 / 80.0) * obj.accel1 + (3.0 / 16.0) * obj.accel4 + (3.0 / 20.0) * obj.accel5);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel6 = calcAccel(obj.position6, &Object::position6, obj.mass);
			}

			// --- STATE 7 ---
			for (Object& obj : simulationObjects)
			{
				obj.position7 = obj.position + step * ((29443841.0 / 614563906.0) * obj.velocity + (77736538.0 / 692538347.0) * obj.velocity4 - (28693883.0 / 1125000000.0) * obj.velocity5 + (23124283.0 / 1800000000.0) * obj.velocity6);
				obj.velocity7 = obj.velocity + step * ((29443841.0 / 614563906.0) * obj.accel1 + (77736538.0 / 692538347.0) * obj.accel4 - (28693883.0 / 1125000000.0) * obj.accel5 + (23124283.0 / 1800000000.0) * obj.accel6);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel7 = calcAccel(obj.position7, &Object::position7, obj.mass);
			}

			// --- STATE 8 ---
			for (Object& obj : simulationObjects)
			{
				obj.position8 = obj.position + step * ((16016141.0 / 946692911.0) * obj.velocity + (61564180.0 / 158732637.0) * obj.velocity4 + (22789713.0 / 633445777.0) * obj.velocity5 + (545815736.0 / 2771057229.0) * obj.velocity6 - (180193667.0 / 1043307555.0) * obj.velocity7);
				obj.velocity8 = obj.velocity + step * ((16016141.0 / 946692911.0) * obj.accel1 + (61564180.0 / 158732637.0) * obj.accel4 + (22789713.0 / 633445777.0) * obj.accel5 + (545815736.0 / 2771057229.0) * obj.accel6 - (180193667.0 / 1043307555.0) * obj.accel7);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel8 = calcAccel(obj.position8, &Object::position8, obj.mass);
			}

			// --- STATE 9 ---
			for (Object& obj : simulationObjects)
			{
				obj.position9 = obj.position + step * ((39632708.0 / 573591083.0) * obj.velocity - (433636366.0 / 683701615.0) * obj.velocity4 - (421739975.0 / 2616292301.0) * obj.velocity5 + (100302831.0 / 723423059.0) * obj.velocity6 + (790204164.0 / 839813087.0) * obj.velocity7 + (800635310.0 / 3783071287.0) * obj.velocity8);
				obj.velocity9 = obj.velocity + step * ((39632708.0 / 573591083.0) * obj.accel1 - (433636366.0 / 683701615.0) * obj.accel4 - (421739975.0 / 2616292301.0) * obj.accel5 + (100302831.0 / 723423059.0) * obj.accel6 + (790204164.0 / 839813087.0) * obj.accel7 + (800635310.0 / 3783071287.0) * obj.accel8);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel9 = calcAccel(obj.position9, &Object::position9, obj.mass);
			}

			// --- STATE 10 ---
			for (Object& obj : simulationObjects)
			{
				obj.position10 = obj.position + step * ((246121993.0 / 1340847787.0) * obj.velocity - (37695042795.0 / 15268766246.0) * obj.velocity4 - (309121744.0 / 1061227803.0) * obj.velocity5 - (12992083.0 / 490766935.0) * obj.velocity6 + (6005943493.0 / 2108947869.0) * obj.velocity7 + (393006217.0 / 1396673457.0) * obj.velocity8 + (123872331.0 / 1001029789.0) * obj.velocity9);
				obj.velocity10 = obj.velocity + step * ((246121993.0 / 1340847787.0) * obj.accel1 - (37695042795.0 / 15268766246.0) * obj.accel4 - (309121744.0 / 1061227803.0) * obj.accel5 - (12992083.0 / 490766935.0) * obj.accel6 + (6005943493.0 / 2108947869.0) * obj.accel7 + (393006217.0 / 1396673457.0) * obj.accel8 + (123872331.0 / 1001029789.0) * obj.accel9);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel10 = calcAccel(obj.position10, &Object::position10, obj.mass);
			}

			// --- STATE 11 ---
			for (Object& obj : simulationObjects)
			{
				obj.position11 = obj.position + step * ((-1028468189.0 / 846180014.0) * obj.velocity + (8478235783.0 / 508512852.0) * obj.velocity4 + (1311729495.0 / 1432422823.0) * obj.velocity5 - (10304129995.0 / 1701304382.0) * obj.velocity6 - (48777925059.0 / 3047939560.0) * obj.velocity7 + (15336726248.0 / 1032824649.0) * obj.velocity8 - (45442868181.0 / 3398467696.0) * obj.velocity9 + (3065993473.0 / 597172653.0) * obj.velocity10);
				obj.velocity11 = obj.velocity + step * ((-1028468189.0 / 846180014.0) * obj.accel1 + (8478235783.0 / 508512852.0) * obj.accel4 + (1311729495.0 / 1432422823.0) * obj.accel5 - (10304129995.0 / 1701304382.0) * obj.accel6 - (48777925059.0 / 3047939560.0) * obj.accel7 + (15336726248.0 / 1032824649.0) * obj.accel8 - (45442868181.0 / 3398467696.0) * obj.accel9 + (3065993473.0 / 597172653.0) * obj.accel10);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel11 = calcAccel(obj.position11, &Object::position11, obj.mass);
			}

			// --- STATE 12 ---
			for (Object& obj : simulationObjects)
			{
				obj.position12 = obj.position + step * ((185892177.0 / 718116043.0) * obj.velocity - (3185094517.0 / 667107341.0) * obj.velocity4 - (477755414.0 / 1098053517.0) * obj.velocity5 - (703635378.0 / 230739211.0) * obj.velocity6 + (5731566787.0 / 1027545527.0) * obj.velocity7 + (5232866602.0 / 850066563.0) * obj.velocity8 - (4093664535.0 / 808688257.0) * obj.velocity9 + (3962137247.0 / 1805957418.0) * obj.velocity10 + (65686358.0 / 487910083.0) * obj.velocity11);
				obj.velocity12 = obj.velocity + step * ((185892177.0 / 718116043.0) * obj.accel1 - (3185094517.0 / 667107341.0) * obj.accel4 - (477755414.0 / 1098053517.0) * obj.accel5 - (703635378.0 / 230739211.0) * obj.accel6 + (5731566787.0 / 1027545527.0) * obj.accel7 + (5232866602.0 / 850066563.0) * obj.accel8 - (4093664535.0 / 808688257.0) * obj.accel9 + (3962137247.0 / 1805957418.0) * obj.accel10 + (65686358.0 / 487910083.0) * obj.accel11);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel12 = calcAccel(obj.position12, &Object::position12, obj.mass);
			}

			// --- STATE 13 ---
			for (Object& obj : simulationObjects)
			{
				obj.position13 = obj.position + step * ((403863854.0 / 491063109.0) * obj.velocity - (5068492393.0 / 434740067.0) * obj.velocity4 - (411421997.0 / 543043805.0) * obj.velocity5 + (652783627.0 / 914296604.0) * obj.velocity6 + (11173962825.0 / 925320556.0) * obj.velocity7 - (13158990841.0 / 6184727034.0) * obj.velocity8 + (3936647629.0 / 1978049680.0) * obj.velocity9 - (160528059.0 / 685178525.0) * obj.velocity10 + (248638103.0 / 1413531060.0) * obj.velocity11);
				obj.velocity13 = obj.velocity + step * ((403863854.0 / 491063109.0) * obj.accel1 - (5068492393.0 / 434740067.0) * obj.accel4 - (411421997.0 / 543043805.0) * obj.accel5 + (652783627.0 / 914296604.0) * obj.accel6 + (11173962825.0 / 925320556.0) * obj.accel7 - (13158990841.0 / 6184727034.0) * obj.accel8 + (3936647629.0 / 1978049680.0) * obj.accel9 - (160528059.0 / 685178525.0) * obj.accel10 + (248638103.0 / 1413531060.0) * obj.accel11);
			}
			for (Object& obj : simulationObjects)
			{
				obj.accel13 = calcAccel(obj.position13, &Object::position13, obj.mass);
			}

			// --- FINAL UPDATE & 7TH/8TH ORDER ERROR ESTIMATION ---
	// --- FINAL UPDATE & 7TH/8TH ORDER ERROR ESTIMATION ---
			double maxErrorRatio = 0.0;

			for (Object& obj : simulationObjects)
			{
				// 8th-order solution
				obj.pos8 = obj.position + step * (
					(14005451.0 / 335480064.0) * obj.velocity -
					(59238493.0 / 1068277825.0) * obj.velocity6 +
					(181606767.0 / 758867731.0) * obj.velocity7 +
					(561292985.0 / 797845732.0) * obj.velocity8 -
					(1041891430.0 / 1371343529.0) * obj.velocity9 +
					(760417239.0 / 1151165299.0) * obj.velocity10 +
					(118820643.0 / 751138087.0) * obj.velocity11 -
					(528747749.0 / 2220607170.0) * obj.velocity12 +
					(1.0 / 4.0) * obj.velocity13
					);
				obj.vel8 = obj.velocity + step * (
					(14005451.0 / 335480064.0) * obj.accel1 -
					(59238493.0 / 1068277825.0) * obj.accel6 +
					(181606767.0 / 758867731.0) * obj.accel7 +
					(561292985.0 / 797845732.0) * obj.accel8 -
					(1041891430.0 / 1371343529.0) * obj.accel9 +
					(760417239.0 / 1151165299.0) * obj.accel10 +
					(118820643.0 / 751138087.0) * obj.accel11 -
					(528747749.0 / 2220607170.0) * obj.accel12 +
					(1.0 / 4.0) * obj.accel13
					);

				// 7th-order embedded solution
				obj.pos7 = obj.position + step * (
					(13451932.0 / 455176623.0) * obj.velocity -
					(808719846.0 / 976000145.0) * obj.velocity6 +
					(1757004468.0 / 5645159321.0) * obj.velocity7 +
					(656045339.0 / 265891186.0) * obj.velocity8 -
					(3867574721.0 / 1518517206.0) * obj.velocity9 +
					(465885868.0 / 322736535.0) * obj.velocity10 +
					(53011238.0 / 667516719.0) * obj.velocity11 +
					(2.0 / 45.0) * obj.velocity12
					);
				obj.vel7 = obj.velocity + step * (
					(13451932.0 / 455176623.0) * obj.accel1 -
					(808719846.0 / 976000145.0) * obj.accel6 +
					(1757004468.0 / 5645159321.0) * obj.accel7 +
					(656045339.0 / 265891186.0) * obj.accel8 -
					(3867574721.0 / 1518517206.0) * obj.accel9 +
					(465885868.0 / 322736535.0) * obj.accel10 +
					(53011238.0 / 667516719.0) * obj.accel11 +
					(2.0 / 45.0) * obj.accel12
					);

				const Vector3d pDiff = obj.pos8 - obj.pos7;
				const Vector3d vDiff = obj.vel8 - obj.vel7;
				const double posErr = std::sqrt(pDiff.x * pDiff.x + pDiff.y * pDiff.y + pDiff.z * pDiff.z);
				const double velErr = std::sqrt(vDiff.x * vDiff.x + vDiff.y * vDiff.y + vDiff.z * vDiff.z);
				const double posScale = std::max(1.0, std::sqrt(obj.pos8.x * obj.pos8.x + obj.pos8.y * obj.pos8.y + obj.pos8.z * obj.pos8.z));
				const double velScale = std::max(1.0, std::sqrt(obj.vel8.x * obj.vel8.x + obj.vel8.y * obj.vel8.y + obj.vel8.z * obj.vel8.z));
				const double ratio = std::max(posErr / posScale, velErr / velScale) / tolerance;
				if (ratio > maxErrorRatio) maxErrorRatio = ratio;
			}

			// Commit all objects together only if the entire system passed the tolerance check
			if (maxErrorRatio <= 1.0)
			{
				for (Object& obj : simulationObjects)
				{
					obj.position = obj.pos8;
					obj.velocity = obj.vel8;
				}
			}

			return maxErrorRatio;
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
		void Update(int& objectToTrack,double& globalTime)
		{
			timestep = intendedTimestep;
			int maxSubsteps = 1000000 / (simulationObjects.size() * simulationObjects.size());
			double time = (timestep / substeps);
			double worstRatio = 0.0;
			for (int i = 0; i < substeps; i++)
			{
				double ratio = RK78Full(time, tolerance);
				if (!(ratio <= 1.0))
				{
					double scale = std::isfinite(ratio) ? std::clamp(std::pow(ratio, 0.125) / 0.9, 1.5, 10.0) : 4.0;
					if (substeps < maxSubsteps)
					{
						substeps = std::min(maxSubsteps, (int)std::ceil(substeps * scale));
					}
					else
					{
						timestep /= scale;
					}
					time = timestep / substeps;
					ratio = RK78Full(time, tolerance);
				}
				if (std::isfinite(ratio)) worstRatio = std::max(worstRatio, ratio);
				globalTime += time;
				Collision(objectToTrack);
			}
			if (worstRatio > 0.0 && worstRatio < 0.5 && substeps > 1)
			{
				double scale = std::clamp(std::pow(worstRatio, 0.125) / 0.9, 0.5, 1.0);
				substeps = std::max(1, (int)std::floor(substeps * scale));
			}

			for (Object& item : simulationObjects)
			{
				item.trail.push_back(item.position);
				if (item.trail.size() > 3000) item.trail.pop_front();
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
				DrawModel(*obj.model, static_cast<Vector3>(obj.position - realPosition), (float)obj.radius, WHITE);
				//trails
				if (obj.trail.size() >= 2) {
					Vector3 prev = ConvertRegular(obj.trail[0] - realPosition);
					for (size_t i = 1; i < obj.trail.size(); ++i) {
						Vector3 curr = ConvertRegular(obj.trail[i] - realPosition);
						DrawLine3D(prev, curr, obj.color);
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