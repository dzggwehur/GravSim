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
#include "Debris.h"
#include <string>
#include <algorithm>
class Simulation
{
public:
	double timestep = 1.0;
	double olddT = 1.0;
	double intendedTimestep = 1.0;
	int substeps = 0;
	double substepSize = 1.0;
	double tolerance = 1e-15;
	std::vector<Object> simulationObjects;
	std::vector<Debris> debrisObjects;
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

			const Vector3d& otherPos = obj2.*stagePos;
			const double dx = otherPos.x - pos.x;
			const double dy = otherPos.y - pos.y;
			const double dz = otherPos.z - pos.z;

			const double distSq = dx * dx + dy * dy + dz * dz;
			if (distSq < 1e-12) continue;

			const double invDist = 1.0 / std::sqrt(distSq);
			const double accelMag = (GRAVITATIONAL_CONSTANT * obj2.mass) * invDist * invDist * invDist;

			ax = std::fma(dx, accelMag, ax);
			ay = std::fma(dy, accelMag, ay);
			az = std::fma(dz, accelMag, az);
		}
		return { ax, ay, az };
	}



	void Yoshida8Full(double step)
	{
		// Symmetric 15-stage weights (w[0..6], w[7], w[6..0])
		static constexpr double w[15] = {
			 0.74167036435061295344822780,  // w1
			-0.40910082580003159399730010,  // w2
			 0.19075471029623837995387626,  // w3
			-0.57386247111608226665638773,  // w4
			 0.29906418130365592384446354,  // w5
			 0.33462491824529818378495798,  // w6
			 0.31529309239676659663205666,  // w7
			-0.79688793935291635401978884,  // w0 (center)
			 0.31529309239676659663205666,  // w7
			 0.33462491824529818378495798,  // w6
			 0.29906418130365592384446354,  // w5
			-0.57386247111608226665638773,  // w4
			 0.19075471029623837995387626,  // w3
			-0.40910082580003159399730010,  // w2
			 0.74167036435061295344822780   // w1
		};

		// First half-drift (c1 = 0.5 * w[0])
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position += (0.5 * w[0] * step) * obj.velocity;
		}

		for (int s = 0; s < 15; s++)
		{
			// 1. Evaluate acceleration at current position
			for (Object& obj : simulationObjects)
			{
				if (obj.anchored) continue;
				obj.accel1 = calcAccel(obj.position, &Object::position, obj.mass);
			}

			// 2. Full velocity kick (d_s = w[s])
			for (Object& obj : simulationObjects)
			{
				if (obj.anchored) continue;
				obj.velocity += (w[s] * step) * obj.accel1;
			}

			// 3. Position drift to next stage (combining adjacent half-drifts: 0.5*(w[s] + w[s+1]))
			const double driftWeight = (s < 14) ? 0.5 * (w[s] + w[s + 1]) : 0.5 * w[14];
			for (Object& obj : simulationObjects)
			{
				if (obj.anchored) continue;
				obj.position += (driftWeight * step) * obj.velocity;
			}
		}
	}

	double RK78Full(double step, double tolerance,bool& fsalValid)
	{
		// --- STATE 1 ---
		if (!fsalValid)
		{
			for (Object& obj : simulationObjects)
			{
				if (obj.anchored) continue;
				obj.accel1 = calcAccel(obj.position, &Object::position, obj.mass);
			}
		}
		// --- STATE 2 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position2 = obj.position + step * ((1.0 / 18.0) * obj.velocity);
			obj.velocity2 = obj.velocity + step * ((1.0 / 18.0) * obj.accel1);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel2 = calcAccel(obj.position2, &Object::position2, obj.mass);
		}

		// --- STATE 3 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position3 = obj.position + step * ((1.0 / 48.0) * obj.velocity + (1.0 / 16.0) * obj.velocity2);
			obj.velocity3 = obj.velocity + step * ((1.0 / 48.0) * obj.accel1 + (1.0 / 16.0) * obj.accel2);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel3 = calcAccel(obj.position3, &Object::position3, obj.mass);
		}

		// --- STATE 4 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position4 = obj.position + step * ((1.0 / 32.0) * obj.velocity + (3.0 / 32.0) * obj.velocity3);
			obj.velocity4 = obj.velocity + step * ((1.0 / 32.0) * obj.accel1 + (3.0 / 32.0) * obj.accel3);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel4 = calcAccel(obj.position4, &Object::position4, obj.mass);
		}

		// --- STATE 5 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position5 = obj.position + step * ((5.0 / 16.0) * obj.velocity - (75.0 / 64.0) * obj.velocity3 + (75.0 / 64.0) * obj.velocity4);
			obj.velocity5 = obj.velocity + step * ((5.0 / 16.0) * obj.accel1 - (75.0 / 64.0) * obj.accel3 + (75.0 / 64.0) * obj.accel4);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel5 = calcAccel(obj.position5, &Object::position5, obj.mass);
		}

		// --- STATE 6 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position6 = obj.position + step * ((3.0 / 80.0) * obj.velocity + (3.0 / 16.0) * obj.velocity4 + (3.0 / 20.0) * obj.velocity5);
			obj.velocity6 = obj.velocity + step * ((3.0 / 80.0) * obj.accel1 + (3.0 / 16.0) * obj.accel4 + (3.0 / 20.0) * obj.accel5);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel6 = calcAccel(obj.position6, &Object::position6, obj.mass);
		}

		// --- STATE 7 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position7 = obj.position + step * ((29443841.0 / 614563906.0) * obj.velocity + (77736538.0 / 692538347.0) * obj.velocity4 - (28693883.0 / 1125000000.0) * obj.velocity5 + (23124283.0 / 1800000000.0) * obj.velocity6);
			obj.velocity7 = obj.velocity + step * ((29443841.0 / 614563906.0) * obj.accel1 + (77736538.0 / 692538347.0) * obj.accel4 - (28693883.0 / 1125000000.0) * obj.accel5 + (23124283.0 / 1800000000.0) * obj.accel6);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel7 = calcAccel(obj.position7, &Object::position7, obj.mass);
		}

		// --- STATE 8 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position8 = obj.position + step * ((16016141.0 / 946692911.0) * obj.velocity + (61564180.0 / 158732637.0) * obj.velocity4 + (22789713.0 / 633445777.0) * obj.velocity5 + (545815736.0 / 2771057229.0) * obj.velocity6 - (180193667.0 / 1043307555.0) * obj.velocity7);
			obj.velocity8 = obj.velocity + step * ((16016141.0 / 946692911.0) * obj.accel1 + (61564180.0 / 158732637.0) * obj.accel4 + (22789713.0 / 633445777.0) * obj.accel5 + (545815736.0 / 2771057229.0) * obj.accel6 - (180193667.0 / 1043307555.0) * obj.accel7);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel8 = calcAccel(obj.position8, &Object::position8, obj.mass);
		}

		// --- STATE 9 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position9 = obj.position + step * ((39632708.0 / 573591083.0) * obj.velocity - (433636366.0 / 683701615.0) * obj.velocity4 - (421739975.0 / 2616292301.0) * obj.velocity5 + (100302831.0 / 723423059.0) * obj.velocity6 + (790204164.0 / 839813087.0) * obj.velocity7 + (800635310.0 / 3783071287.0) * obj.velocity8);
			obj.velocity9 = obj.velocity + step * ((39632708.0 / 573591083.0) * obj.accel1 - (433636366.0 / 683701615.0) * obj.accel4 - (421739975.0 / 2616292301.0) * obj.accel5 + (100302831.0 / 723423059.0) * obj.accel6 + (790204164.0 / 839813087.0) * obj.accel7 + (800635310.0 / 3783071287.0) * obj.accel8);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel9 = calcAccel(obj.position9, &Object::position9, obj.mass);
		}

		// --- STATE 10 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position10 = obj.position + step * ((246121993.0 / 1340847787.0) * obj.velocity - (37695042795.0 / 15268766246.0) * obj.velocity4 - (309121744.0 / 1061227803.0) * obj.velocity5 - (12992083.0 / 490766935.0) * obj.velocity6 + (6005943493.0 / 2108947869.0) * obj.velocity7 + (393006217.0 / 1396673457.0) * obj.velocity8 + (123872331.0 / 1001029789.0) * obj.velocity9);
			obj.velocity10 = obj.velocity + step * ((246121993.0 / 1340847787.0) * obj.accel1 - (37695042795.0 / 15268766246.0) * obj.accel4 - (309121744.0 / 1061227803.0) * obj.accel5 - (12992083.0 / 490766935.0) * obj.accel6 + (6005943493.0 / 2108947869.0) * obj.accel7 + (393006217.0 / 1396673457.0) * obj.accel8 + (123872331.0 / 1001029789.0) * obj.accel9);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel10 = calcAccel(obj.position10, &Object::position10, obj.mass);
		}

		// --- STATE 11 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position11 = obj.position + step * ((-1028468189.0 / 846180014.0) * obj.velocity + (8478235783.0 / 508512852.0) * obj.velocity4 + (1311729495.0 / 1432422823.0) * obj.velocity5 - (10304129995.0 / 1701304382.0) * obj.velocity6 - (48777925059.0 / 3047939560.0) * obj.velocity7 + (15336726248.0 / 1032824649.0) * obj.velocity8 - (45442868181.0 / 3398467696.0) * obj.velocity9 + (3065993473.0 / 597172653.0) * obj.velocity10);
			obj.velocity11 = obj.velocity + step * ((-1028468189.0 / 846180014.0) * obj.accel1 + (8478235783.0 / 508512852.0) * obj.accel4 + (1311729495.0 / 1432422823.0) * obj.accel5 - (10304129995.0 / 1701304382.0) * obj.accel6 - (48777925059.0 / 3047939560.0) * obj.accel7 + (15336726248.0 / 1032824649.0) * obj.accel8 - (45442868181.0 / 3398467696.0) * obj.accel9 + (3065993473.0 / 597172653.0) * obj.accel10);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel11 = calcAccel(obj.position11, &Object::position11, obj.mass);
		}

		// --- STATE 12 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position12 = obj.position + step * ((185892177.0 / 718116043.0) * obj.velocity - (3185094517.0 / 667107341.0) * obj.velocity4 - (477755414.0 / 1098053517.0) * obj.velocity5 - (703635378.0 / 230739211.0) * obj.velocity6 + (5731566787.0 / 1027545527.0) * obj.velocity7 + (5232866602.0 / 850066563.0) * obj.velocity8 - (4093664535.0 / 808688257.0) * obj.velocity9 + (3962137247.0 / 1805957418.0) * obj.velocity10 + (65686358.0 / 487910083.0) * obj.velocity11);
			obj.velocity12 = obj.velocity + step * ((185892177.0 / 718116043.0) * obj.accel1 - (3185094517.0 / 667107341.0) * obj.accel4 - (477755414.0 / 1098053517.0) * obj.accel5 - (703635378.0 / 230739211.0) * obj.accel6 + (5731566787.0 / 1027545527.0) * obj.accel7 + (5232866602.0 / 850066563.0) * obj.accel8 - (4093664535.0 / 808688257.0) * obj.accel9 + (3962137247.0 / 1805957418.0) * obj.accel10 + (65686358.0 / 487910083.0) * obj.accel11);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel12 = calcAccel(obj.position12, &Object::position12, obj.mass);
		}

		// --- STATE 13 ---
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.position13 = obj.position + step * ((403863854.0 / 491063109.0) * obj.velocity - (5068492393.0 / 434740067.0) * obj.velocity4 - (411421997.0 / 543043805.0) * obj.velocity5 + (652783627.0 / 914296604.0) * obj.velocity6 + (11173962825.0 / 925320556.0) * obj.velocity7 - (13158990841.0 / 6184727034.0) * obj.velocity8 + (3936647629.0 / 1978049680.0) * obj.velocity9 - (160528059.0 / 685178525.0) * obj.velocity10 + (248638103.0 / 1413531060.0) * obj.velocity11);
			obj.velocity13 = obj.velocity + step * ((403863854.0 / 491063109.0) * obj.accel1 - (5068492393.0 / 434740067.0) * obj.accel4 - (411421997.0 / 543043805.0) * obj.accel5 + (652783627.0 / 914296604.0) * obj.accel6 + (11173962825.0 / 925320556.0) * obj.accel7 - (13158990841.0 / 6184727034.0) * obj.accel8 + (3936647629.0 / 1978049680.0) * obj.accel9 - (160528059.0 / 685178525.0) * obj.accel10 + (248638103.0 / 1413531060.0) * obj.accel11);
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel13 = calcAccel(obj.position13, &Object::position13, obj.mass);
		}
		// --- FINAL UPDATE & DOP853 8(5,3) ERROR ESTIMATION ---
		const double tolSq = tolerance * tolerance;
		double maxErrorRatio = 0.0;

		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			// 8th-order solution (stored in position14 / velocity14)
			obj.position14 = obj.position + step * (
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
			obj.velocity14 = obj.velocity + step * (
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
		}
		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;
			obj.accel14 = calcAccel(obj.position14, &Object::position14, obj.mass);
		}

		for (Object& obj : simulationObjects)
		{
			if (obj.anchored) continue;

			// Exact 5th/6th-order embedded error difference (b_i - b5_i, sum = 0)
			const Vector3d pErr5 = step * (
				(14005451.0 / 335480064.0 - 120071.0 / 1283958.0) * obj.velocity +
				(-59238493.0 / 1068277825.0 - 513248.0 / 405405.0) * obj.velocity6 +
				(181606767.0 / 758867731.0 - 800000000.0 / 140206715649.0) * obj.velocity7 +
				(561292985.0 / 797845732.0 + 1562500000.0 / 1262513223.0) * obj.velocity8 -
				(1041891430.0 / 1371343529.0) * obj.velocity9 +
				(760417239.0 / 1151165299.0 - 17387000.0 / 22333311.0) * obj.velocity10 +
				(118820643.0 / 751138087.0) * obj.velocity11 +
				(-528747749.0 / 2220607170.0 - 719123.0 / 7662270.0) * obj.velocity12 +
				(1.0 / 4.0) * obj.velocity13
				);
			const Vector3d vErr5 = step * (
				(14005451.0 / 335480064.0 - 120071.0 / 1283958.0) * obj.accel1 +
				(-59238493.0 / 1068277825.0 - 513248.0 / 405405.0) * obj.accel6 +
				(181606767.0 / 758867731.0 - 800000000.0 / 140206715649.0) * obj.accel7 +
				(561292985.0 / 797845732.0 + 1562500000.0 / 1262513223.0) * obj.accel8 -
				(1041891430.0 / 1371343529.0) * obj.accel9 +
				(760417239.0 / 1151165299.0 - 17387000.0 / 22333311.0) * obj.accel10 +
				(118820643.0 / 751138087.0) * obj.accel11 +
				(-528747749.0 / 2220607170.0 - 719123.0 / 7662270.0) * obj.accel12 +
				(1.0 / 4.0) * obj.accel13
				);

			// Exact 3rd-order embedded error difference (b_i - b3_i, sum = 0)
			const Vector3d pErr3 = step * (
				(14005451.0 / 335480064.0 - 79.0 / 558.0) * obj.velocity -
				(59238493.0 / 1068277825.0) * obj.velocity6 +
				(181606767.0 / 758867731.0) * obj.velocity7 +
				(561292985.0 / 797845732.0 - 20000.0 / 29853.0) * obj.velocity8 -
				(1041891430.0 / 1371343529.0) * obj.velocity9 +
				(760417239.0 / 1151165299.0) * obj.velocity10 +
				(118820643.0 / 751138087.0) * obj.velocity11 -
				(528747749.0 / 2220607170.0) * obj.velocity12 +
				(1.0 / 4.0) * obj.velocity13 -
				(121.0 / 642.0) * obj.velocity14
				);
			const Vector3d vErr3 = step * (
				(14005451.0 / 335480064.0 - 79.0 / 558.0) * obj.accel1 -
				(59238493.0 / 1068277825.0) * obj.accel6 +
				(181606767.0 / 758867731.0) * obj.accel7 +
				(561292985.0 / 797845732.0 - 20000.0 / 29853.0) * obj.accel8 -
				(1041891430.0 / 1371343529.0) * obj.accel9 +
				(760417239.0 / 1151165299.0) * obj.accel10 +
				(118820643.0 / 751138087.0) * obj.accel11 -
				(528747749.0 / 2220607170.0) * obj.accel12 +
				(1.0 / 4.0) * obj.accel13 -
				(121.0 / 642.0) * obj.accel14
				);
			
			const double posErr5Sq = pErr5.x * pErr5.x + pErr5.y * pErr5.y + pErr5.z * pErr5.z;
			const double velErr5Sq = vErr5.x * vErr5.x + vErr5.y * vErr5.y + vErr5.z * vErr5.z;
			const double posErr3Sq = pErr3.x * pErr3.x + pErr3.y * pErr3.y + pErr3.z * pErr3.z;
			const double velErr3Sq = vErr3.x * vErr3.x + vErr3.y * vErr3.y + vErr3.z * vErr3.z;

			const double posScaleSq = std::max(1.0, obj.position14.x * obj.position14.x + obj.position14.y * obj.position14.y + obj.position14.z * obj.position14.z) * tolSq;
			const double velScaleSq = std::max(1.0, obj.velocity14.x * obj.velocity14.x + obj.velocity14.y * obj.velocity14.y + obj.velocity14.z * obj.velocity14.z) * tolSq;

			const double err5 = std::sqrt(std::max(posErr5Sq / posScaleSq, velErr5Sq / velScaleSq));
			const double err3 = std::sqrt(std::max(posErr3Sq / posScaleSq, velErr3Sq / velScaleSq));

			const double denom = err5 + 0.01 * err3;
			const double ratio = (denom > 0.0) ? ((err5 * err5) / denom) : 0.0;
			if (ratio > maxErrorRatio) maxErrorRatio = ratio;
		}
		if (maxErrorRatio <= 1.0)
		{
			for (Object& obj : simulationObjects)
			{
				if (obj.anchored) continue;
				obj.position = obj.position14;
				obj.velocity = obj.velocity14;
				obj.accel1 = obj.accel14;
				
			}
			fsalValid = true;
		}
		return maxErrorRatio;
	}



	void FullAdaptiveRK78(int& objectToTrack, double& globalTime)
	{
		bool fsalValid = false;
		timestep = intendedTimestep;
		if (simulationObjects.size() == 0) {
			globalTime += intendedTimestep; return;
		};
		int maxSubsteps = std::max(1000000 / (simulationObjects.size() * simulationObjects.size()), (unsigned long long)1);
		double time = (timestep / substeps);
		double worstRatio = 0.0;
		for (int i = 0; i < substeps; i++)
		{
			double ratio = RK78Full(time, tolerance,fsalValid);
			while (!(ratio <= 1.0))
			{
				double scale = std::isfinite(ratio) ? std::clamp(std::pow(ratio, 0.125) / 0.9, 1.5, 10.0) : 4.0;
				if (substeps < maxSubsteps)
				{
					int oldSubsteps = substeps;
					substeps = std::min(maxSubsteps, (int)std::ceil(substeps * scale));
					i = (int)std::round((double)i * substeps / oldSubsteps);
				}
				else
				{
					timestep /= scale;
				}
				time = timestep / substeps;
				ratio = RK78Full(time, tolerance, fsalValid);
			}
			if (std::isfinite(ratio)) worstRatio = std::max(worstRatio, ratio);
			globalTime += time;
			Collision(objectToTrack,fsalValid);
		}
		if (worstRatio > 0.0 && worstRatio < 0.5 && substeps > 1)
		{
			double scale = std::clamp(std::pow(worstRatio, 0.125) / 0.9, 0.5, 1.0);
			substeps = std::max(1, (int)std::floor(substeps * scale));
		}
	}

	void Collision(int& track,bool& fsalValid)
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
					fsalValid = false;
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


	void Update(int& objectToTrack, double& globalTime, int integrator)
	{
		switch (integrator)
		{
			case RK78:
			{
				FullAdaptiveRK78(objectToTrack, globalTime);
				break;
			}
			case Yoshida8:
			{
				timestep = intendedTimestep;
				Yoshida8Full(timestep);
				globalTime += timestep;
				bool fsalValid = true;
				Collision(objectToTrack,fsalValid);
				break;
			}
		}
		UpdateGravDebris();
		for (Object& item : simulationObjects)
		{
			item.trail.push_back(item.position);
			if (item.trail.size() > 3000) item.trail.pop_front();
		}
	}
	void AddObject(double mass, Vector3d position, Vector3d initialVelocity, double radius, Color color, Model* model, const std::string& name,bool anchored)
	{

		Object obj;
		obj.mass = mass;
		obj.position = position;
		obj.velocity = initialVelocity;
		obj.radius = radius;
		obj.color = color;
		obj.model = model;
		obj.name = name;
		obj.anchored = anchored;
		simulationObjects.emplace_back(obj);
	}
	double ConvertEccentricToTrueAnomaly(double eccentricAnomaly, double eccentricity)
	{
		double cosE, sinE,sqrtTerm,trueAnomaly;
		if (eccentricity < 1.0)
		{
			cosE = std::cos(eccentricAnomaly);
			sinE = std::sin(eccentricAnomaly);
			sqrtTerm = std::sqrt(1.0 - eccentricity * eccentricity);
			trueAnomaly = std::atan2(sqrtTerm * sinE, cosE - eccentricity);
		}
		else
		{
			cosE = std::cosh(eccentricAnomaly);
			sinE = std::sinh(eccentricAnomaly);
			sqrtTerm = std::sqrt(eccentricity * eccentricity - 1.0);
			trueAnomaly = std::atan2(sqrtTerm * sinE, eccentricity - cosE);
			
		}
		return trueAnomaly;
	}
	void FindObjectByName(const std::string& name, Object*& foundObject)
	{
		for (Object& obj : simulationObjects)
		{
			if (obj.name == name)
			{
				foundObject = &obj;
				return;
			}
		}
		foundObject = nullptr;
	}
	void AddObjectKeplerian(double mass, double radius, Color color, Model* model, const std::string& name, double semimajorAxis, double eccentricity, double inclination, double longitudeOfAscendingNode, double argumentOfPeriapsis,double xAnomaly,int anomalyType, const std::string& parentName = "Sun",bool anchored = false)
	{

		Object obj;
		obj.mass = mass;
		obj.anchored = anchored;
		Vector3d position = { 0, 0, 0 };
		Vector3d initialVelocity = { 0, 0, 0 };
		double trueAnomaly = xAnomaly;
		Object* parent = nullptr;
		FindObjectByName(parentName, parent);
		if (parent == nullptr)
		{
			std::clog << "Parent object not found: " << parentName << std::endl;
			return;
		}
		if (eccentricity == 1.0)
		{
			eccentricity = 1.0 - 1e-12;
		}
		if (eccentricity > 1.0 && semimajorAxis > 0.0)
		{
			semimajorAxis = -semimajorAxis;
		}
		if (anomalyType == MeanAnomaly)
		{
			double currentEccentricAnomaly = 0.0;
			if (eccentricity > 0.8 && eccentricity < 1.0)
			{
				currentEccentricAnomaly = PI;
			}
			else if (eccentricity < 1.0)
			{
				currentEccentricAnomaly = xAnomaly;
			}
			else
			{
				if (std::abs(xAnomaly) > 1.0)
				{
					currentEccentricAnomaly = (xAnomaly >= 0.0)
						? std::log(2.0 * xAnomaly / eccentricity + 1.8)
						: -std::log(-2.0 * xAnomaly / eccentricity + 1.8);
				}
				else
				{
					currentEccentricAnomaly = xAnomaly / (eccentricity - 1.0);
				}


			}
			int iterations = 5;
			iterations = std::clamp(static_cast<int>(5.0 / std::abs(1.0 - eccentricity)), 10, 100);
			if (eccentricity < 1.0)
			{
				for (int i = 0; i < iterations; i++)
				{
					currentEccentricAnomaly = currentEccentricAnomaly - (currentEccentricAnomaly - eccentricity * std::sin(currentEccentricAnomaly) - xAnomaly) / (1.0 - eccentricity * std::cos(currentEccentricAnomaly));
				}
			}
			else
			{
				for (int i = 0; i < iterations; i++)
				{
					currentEccentricAnomaly = currentEccentricAnomaly - (eccentricity * std::sinh(currentEccentricAnomaly) - currentEccentricAnomaly - xAnomaly) / (eccentricity * std::cosh(currentEccentricAnomaly) - 1.0);
				}
			}
			trueAnomaly = ConvertEccentricToTrueAnomaly(currentEccentricAnomaly, eccentricity);
		}
		else if (anomalyType == EccentricAnomaly)
		{
			trueAnomaly = ConvertEccentricToTrueAnomaly(xAnomaly, eccentricity);
		}
		double mu = GRAVITATIONAL_CONSTANT * (mass + parent->mass);
		double parameter = semimajorAxis * (1.0 - eccentricity * eccentricity);
		double orbitRadius = parameter / (1.0 + eccentricity * std::cos(trueAnomaly));
		double specificAngularMomentum = std::sqrt(mu * parameter);
		double argumentOfLatitude = argumentOfPeriapsis + trueAnomaly;
		position = {
			orbitRadius * (std::cos(longitudeOfAscendingNode) * std::cos(argumentOfLatitude) - std::sin(longitudeOfAscendingNode) * std::sin(argumentOfLatitude) * std::cos(inclination)),
			orbitRadius * (std::sin(argumentOfLatitude) * std::sin(inclination)),
			orbitRadius * (std::sin(longitudeOfAscendingNode) * std::cos(argumentOfLatitude) + std::cos(longitudeOfAscendingNode) * std::sin(argumentOfLatitude) * std::cos(inclination))
		};
		initialVelocity = {
			-(mu / specificAngularMomentum) * (std::cos(longitudeOfAscendingNode) * (std::sin(argumentOfLatitude) + eccentricity * std::sin(argumentOfPeriapsis)) + std::sin(longitudeOfAscendingNode) * (std::cos(argumentOfLatitude) + eccentricity * std::cos(argumentOfPeriapsis)) * std::cos(inclination)),
			 (mu / specificAngularMomentum) * ((std::cos(argumentOfLatitude) + eccentricity * std::cos(argumentOfPeriapsis)) * std::sin(inclination)),
			-(mu / specificAngularMomentum) * (std::sin(longitudeOfAscendingNode) * (std::sin(argumentOfLatitude) + eccentricity * std::sin(argumentOfPeriapsis)) - std::cos(longitudeOfAscendingNode) * (std::cos(argumentOfLatitude) + eccentricity * std::cos(argumentOfPeriapsis)) * std::cos(inclination))
		};



		obj.position = position + parent->position;
		obj.velocity = initialVelocity + parent->velocity;

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
		for (const Debris& debris : debrisObjects)
		{
			DrawModel(*debris.model, static_cast<Vector3>(debris.position - realPosition), (float)debris.radius, WHITE);
		}
	}
	void Draw2D(Vector3d realPosition, Camera cam)
	{
		for (const Object& obj : simulationObjects)
		{
			Vector2 screenPos = GetWorldToScreen(static_cast<Vector3>(obj.position - realPosition), cam);
			Vector3 toTarget = Vector3Subtract(static_cast<Vector3>(obj.position - realPosition), cam.position);
			Vector3 camForward = Vector3Subtract(cam.target, Vector3{ 0 });
			float size = std::clamp((float)log10(std::max((float)obj.radius, 1.0f))*5.0f, 1.0f, 50.0f);
			if (Vector3DotProduct(toTarget, camForward) > 0) {
				if (screenPos.x > 0 && screenPos.y > 0) DrawText(obj.name.c_str(), screenPos.x - (MeasureText(obj.name.c_str(), size) / 2), screenPos.y, size, WHITE);
			}
		}
		for (const Debris& debris : debrisObjects)
		{
			Vector2 screenPos = GetWorldToScreen(static_cast<Vector3>(debris.position - realPosition), cam);
			Vector3 toTarget = Vector3Subtract(static_cast<Vector3>(debris.position - realPosition), cam.position);
			Vector3 camForward = Vector3Subtract(cam.target, Vector3{ 0 });
			if (Vector3DotProduct(toTarget, camForward) > 0) {
				if (screenPos.x > 0 && screenPos.y > 0) DrawCircle(screenPos.x, screenPos.y, 2, WHITE);
			}
		}
	}
	//Debris functions
	Vector3d calcAccelDebris(const Vector3d& pos) const
	{
		double ax = 0.0, ay = 0.0, az = 0.0;
		const int count = static_cast<int>(simulationObjects.size());

		for (int j = 0; j < count; ++j)
		{
			const double dx = simulationObjects[j].position.x - pos.x;
			const double dy = simulationObjects[j].position.y - pos.y;
			const double dz = simulationObjects[j].position.z - pos.z;

			const double distSq = dx * dx + dy * dy + dz * dz;
			if (distSq < 1e-12) continue;

			const double invDist = 1.0 / std::sqrt(distSq);
			const double accelMag = (GRAVITATIONAL_CONSTANT * simulationObjects[j].mass) * invDist * invDist * invDist;

			ax = std::fma(dx, accelMag, ax);
			ay = std::fma(dy, accelMag, ay);
			az = std::fma(dz, accelMag, az);
		}
		return { ax, ay, az };
	}

	void AddDebris(Vector3d position, Vector3d initialVelocity, double radius, Color color, Model* model, std::string name)
	{
		Debris debris;
		debris.position = position;
		debris.velocity = initialVelocity;
		debris.radius = radius;
		debris.model = model;
		debris.name = name;
		debrisObjects.emplace_back(debris);
	}
	void AddDebrisKeplerian(double mass, double radius, Model* model, const std::string& name, double semimajorAxis, double eccentricity, double inclination, double longitudeOfAscendingNode, double argumentOfPeriapsis, double xAnomaly, int anomalyType, const std::string& parentName = "Sun", bool anchored = false)
	{

		Debris debris;
		debris.mass = mass;
		Vector3d position = { 0, 0, 0 };
		Vector3d initialVelocity = { 0, 0, 0 };
		double trueAnomaly = xAnomaly;
		Object* parent = nullptr;
		FindObjectByName(parentName, parent);
		if (parent == nullptr)
		{
			std::clog << "Parent object not found: " << parentName << std::endl;
			return;
		}
		if (eccentricity == 1.0)
		{
			eccentricity = 1.0 - 1e-12;
		}
		if (eccentricity > 1.0 && semimajorAxis > 0.0)
		{
			semimajorAxis = -semimajorAxis;
		}
		if (anomalyType == MeanAnomaly)
		{
			double currentEccentricAnomaly = 0.0;
			if (eccentricity > 0.8 && eccentricity < 1.0)
			{
				currentEccentricAnomaly = PI;
			}
			else if (eccentricity < 1.0)
			{
				currentEccentricAnomaly = xAnomaly;
			}
			else
			{
				if (std::abs(xAnomaly) > 1.0)
				{
					currentEccentricAnomaly = (xAnomaly >= 0.0)
						? std::log(2.0 * xAnomaly / eccentricity + 1.8)
						: -std::log(-2.0 * xAnomaly / eccentricity + 1.8);
				}
				else
				{
					currentEccentricAnomaly = xAnomaly / (eccentricity - 1.0);
				}


			}
			int iterations = 5;
			if (eccentricity != 1.0)
			{
				iterations = std::min(100, static_cast<int>(5.0 / (1.0 - eccentricity)));
			}
			else
			{
				iterations = 100;
			}
			if (eccentricity < 1.0)
			{
				for (int i = 0; i < iterations; i++)
				{
					currentEccentricAnomaly = currentEccentricAnomaly - (currentEccentricAnomaly - eccentricity * std::sin(currentEccentricAnomaly) - xAnomaly) / (1.0 - eccentricity * std::cos(currentEccentricAnomaly));
				}
			}
			else
			{
				for (int i = 0; i < iterations; i++)
				{
					currentEccentricAnomaly = currentEccentricAnomaly - (eccentricity * std::sinh(currentEccentricAnomaly) - currentEccentricAnomaly - xAnomaly) / (eccentricity * std::cosh(currentEccentricAnomaly) - 1.0);
				}
			}
			trueAnomaly = ConvertEccentricToTrueAnomaly(currentEccentricAnomaly, eccentricity);
		}
		else if (anomalyType == EccentricAnomaly)
		{
			trueAnomaly = ConvertEccentricToTrueAnomaly(xAnomaly, eccentricity);
		}
		double mu = GRAVITATIONAL_CONSTANT * (mass + parent->mass);
		double parameter = semimajorAxis * (1.0 - eccentricity * eccentricity);
		double orbitRadius = parameter / (1.0 + eccentricity * std::cos(trueAnomaly));
		double specificAngularMomentum = std::sqrt(mu * parameter);
		double argumentOfLatitude = argumentOfPeriapsis + trueAnomaly;
		position = {
			orbitRadius * (std::cos(longitudeOfAscendingNode) * std::cos(argumentOfLatitude) - std::sin(longitudeOfAscendingNode) * std::sin(argumentOfLatitude) * std::cos(inclination)),
			orbitRadius * (std::sin(argumentOfLatitude) * std::sin(inclination)),
			orbitRadius * (std::sin(longitudeOfAscendingNode) * std::cos(argumentOfLatitude) + std::cos(longitudeOfAscendingNode) * std::sin(argumentOfLatitude) * std::cos(inclination))
		};
		initialVelocity = {
			-(mu / specificAngularMomentum) * (std::cos(longitudeOfAscendingNode) * (std::sin(argumentOfLatitude) + eccentricity * std::sin(argumentOfPeriapsis)) + std::sin(longitudeOfAscendingNode) * (std::cos(argumentOfLatitude) + eccentricity * std::cos(argumentOfPeriapsis)) * std::cos(inclination)),
			 (mu / specificAngularMomentum) * ((std::cos(argumentOfLatitude) + eccentricity * std::cos(argumentOfPeriapsis)) * std::sin(inclination)),
			-(mu / specificAngularMomentum) * (std::sin(longitudeOfAscendingNode) * (std::sin(argumentOfLatitude) + eccentricity * std::sin(argumentOfPeriapsis)) - std::cos(longitudeOfAscendingNode) * (std::cos(argumentOfLatitude) + eccentricity * std::cos(argumentOfPeriapsis)) * std::cos(inclination))
		};



		debris.position = position + parent->position;
		debris.velocity = initialVelocity + parent->velocity;

		debris.radius = radius;
		debris.model = model;
		debris.name = name;
		debris.anchored = anchored;
		debrisObjects.emplace_back(debris);
	}
	void UpdateGravDebris()
	{
		for (Debris& debris : debrisObjects)
		{
			debris.position += debris.velocity * timestep + 0.5 * debris.oldAccel * (timestep * timestep);
			Vector3d accel = calcAccelDebris(debris.position);
			debris.velocity += 0.5 * (debris.oldAccel + accel) * timestep;
			debris.oldAccel = accel;
			//CollisionDebris();
		}

	}
};