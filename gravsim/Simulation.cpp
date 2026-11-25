#include "Simulation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <ranges>
#include <utility>
//Calculate global gravitational pull.
double pnOrder = 2.6;
Vector3d Simulation::CalcAccel(const Vector3d& pos, const Vector3d& vel, Vector3d Object::* stagePos, Vector3d Object::* stageVel, double mass) const
{
	double ax = 0.0, ay = 0.0, az = 0.0;
	int count = simulationObjects.size();
	for (int j = 0; j < count; ++j)
	{
		const Object& obj2 = simulationObjects[j];
		const Vector3d& otherPos = obj2.*stagePos;
		const double dx = otherPos.x - pos.x;
		const double dy = otherPos.y - pos.y;
		const double dz = otherPos.z - pos.z;

		const double distSq = dx * dx + dy * dy + dz * dz;
		if (distSq < 1e-50) continue;

		const double invDist = 1.0 / std::sqrt(distSq);
		const double invDist2 = invDist * invDist;
		const double gm = GRAVITATIONAL_CONSTANT * obj2.mass;
		double accelMag = gm * invDist2 * invDist; // multiplies dx (which is -r * n_hat)

		const double pnFactor = gm * invDist * INV_CSQUARED; // (G * m2 / r) / c^2
		if (pnFactor > 1e-12 && pnOrder >= 1.0)
		{
			const Vector3d& otherVel = obj2.*stageVel;
			const double vx = vel.x - otherVel.x;
			const double vy = vel.y - otherVel.y;
			const double vz = vel.z - otherVel.z;

			const double vSq = vx * vx + vy * vy + vz * vz;
			const double dDotV = dx * vx + dy * vy + dz * vz;
			const double rDot = -dDotV * invDist; // True n_hat . v (positive when moving apart)

			const double mTotal = mass + obj2.mass;
			const double eta = (mTotal > 0.0) ? (mass * obj2.mass) / (mTotal * mTotal) : 0.0;
			const double gmTotalOverR = GRAVITATIONAL_CONSTANT * mTotal * invDist;

			// --- 1PN ---
			double radCorrection = (
				(4.0 + 2.0 * eta) * gmTotalOverR
				- (1.0 + 3.0 * eta) * vSq
				+ 1.5 * eta * rDot * rDot
				);
			double velCorrection = -2.0 * (2.0 - eta) * rDot;

			// --- 2PN ---
			if (pnOrder >= 2.0)
			{
				const double rDot2 = rDot * rDot;
				const double rDot4 = rDot2 * rDot2;
				const double v4 = vSq * vSq;
				const double gm2_r2 = gmTotalOverR * gmTotalOverR;

				radCorrection += INV_CSQUARED * (
					-0.75 * (12.0 + 29.0 * eta) * gm2_r2
					- eta * (3.0 - 4.0 * eta) * v4
					- (15.0 / 8.0) * eta * (1.0 - 3.0 * eta) * rDot4
					+ 1.5 * eta * (3.0 - 4.0 * eta) * vSq * rDot2
					+ 0.5 * eta * (13.0 - 4.0 * eta) * gmTotalOverR * vSq
					+ (2.0 + 25.0 * eta + 2.0 * eta * eta) * gmTotalOverR * rDot2
					);

				velCorrection += INV_CSQUARED * (
					-0.5 * rDot * (
						eta * (15.0 + 4.0 * eta) * vSq
						- (4.0 + 41.0 * eta + 8.0 * eta * eta) * gmTotalOverR
						- 3.0 * eta * (3.0 + 2.0 * eta) * rDot2
						)
					);
			}

			// --- 2.5PN (Gravitational Radiation Reaction) ---
			if (pnOrder >= 2.5)
			{
				const double invC = std::sqrt(INV_CSQUARED);
				const double invC3 = INV_CSQUARED * invC; // (1/c^2 already in pnFactor -> total 1/c^5)
				const double radFactor25 = (8.0 / 5.0) * GRAVITATIONAL_CONSTANT * mass * invDist * invC3;

				// Note: radial term is negated relative to A_2.5PN because accelMag multiplies dx (-r * n_hat)
				radCorrection += radFactor25 * rDot * (3.0 * vSq + (17.0 / 3.0) * gmTotalOverR);
				velCorrection += -radFactor25 * (vSq + 3.0 * gmTotalOverR);
			}

			// Apply radial (along dx) and velocity (along vx) PN corrections
			accelMag += 0.5*pnFactor * invDist2 * radCorrection;

			const double pnVel = 0.5*pnFactor * invDist * velCorrection;
			ax = std::fma(vx, pnVel, ax);
			ay = std::fma(vy, pnVel, ay);
			az = std::fma(vz, pnVel, az);
		}

		ax = std::fma(dx, accelMag, ax);
		ay = std::fma(dy, accelMag, ay);
		az = std::fma(dz, accelMag, az);
	}
	return { ax, ay, az };
}



//O(n^2) collision detection and response. Only used in the integrators; substepping is used to avoid tunneling. Uses Newtonian kinematics (F = 0.5mv^2).
void Simulation::Collision(int& track, bool& fsalValid,double time)
{
	//Check collision
	for (auto [index, obj] : std::views::enumerate(simulationObjects))
	{
		double speed = sqrt(obj.angularSpin.x * obj.angularSpin.x +
			obj.angularSpin.y * obj.angularSpin.y +
			obj.angularSpin.z * obj.angularSpin.z);

		if (speed > 1e-8) {
			// Pass (x, y, z) directly — raymath maps pitch=X, yaw=Y, roll=Z!
			Quaternion q = QuaternionFromEuler(
				(float)obj.orientation.x,
				(float)obj.orientation.y,
				(float)obj.orientation.z
			);

			Vector3 axis = {
				(float)(obj.angularSpin.x / speed),
				(float)(obj.angularSpin.y / speed),
				(float)(obj.angularSpin.z / speed)
			};
			Quaternion deltaQ = QuaternionFromAxisAngle(axis, (float)(speed * time));

			// World-space rotation: deltaQ * q
			q = QuaternionNormalize(QuaternionMultiply(deltaQ, q));

			Vector3 newEuler = QuaternionToEuler(q);
			obj.orientation.x = (double)newEuler.x;
			obj.orientation.y = (double)newEuler.y;
			obj.orientation.z = (double)newEuler.z;
		}




		for (auto [index2, obj2] : std::views::enumerate(simulationObjects))
		{
			double dist = Vector3dDistance(obj.position, obj2.position);
			if (&obj2 == &obj || obj2.destroyed) continue;
			if (dist < obj.radius + obj2.radius)
			{
				std::cout << "coll\n";
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

//Convert Eccentric Anomaly to True Anomaly
double Simulation::ConvertEccentricToTrueAnomaly(double eccentricAnomaly, double eccentricity)
{
	double cosE, sinE, sqrtTerm, trueAnomaly;
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

//Finds an object by name.
void Simulation::FindObjectByName(const std::string& name, Object*& foundObject)
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

//Updates the simulation by performing a single timestep using the specified integrator. It also handles collision detection and response, and updates the trails of all objects in the simulation.
void Simulation::Update(int& objectToTrack, double& globalTime, int integrator, int maxSubSteps)
{
	bool fsalValid;
	switch (integrator)
	{
		case IAS15:
		{
			FullAdaptiveIAS15(objectToTrack, globalTime, maxSubSteps);
			break;
		}
		case DOP853:
		{
			FullAdaptiveDOP853(objectToTrack, globalTime, maxSubSteps);
			break;
		}
		case RK43:
		{
			timestep = intendedTimestep;
			FullAdaptiveRK43(objectToTrack, globalTime, maxSubSteps);
			globalTime += timestep;
			Collision(objectToTrack, fsalValid, timestep);
			break;
		}
		case RK21:
		{
			FullAdaptiveRK21(objectToTrack, globalTime, maxSubSteps);
			break;
		}
		case ExplicitEuler:
		{
			timestep = intendedTimestep;
			EulerFull(timestep);
			globalTime += timestep;
			Collision(objectToTrack, fsalValid,timestep);
			break;
		}

		case Yoshida8:
		{
			timestep = intendedTimestep;
			Yoshida8Full(timestep);
			globalTime += timestep;
			Collision(objectToTrack, fsalValid, timestep);
			break;
		}
		case SymplecticEuler:
		{
			timestep = intendedTimestep;
			SymplecticEulerFull(timestep);
			globalTime += timestep;
			Collision(objectToTrack, fsalValid, timestep);
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

//Adds an object to the simulation with specified properties
void Simulation::AddObject(double mass, Vector3d position, Vector3d initialVelocity, Vector3d angularMomentum, Vector3d orientation, double radius, Color color, Model* model, const std::string& name, bool anchored)
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
	obj.angularSpin = angularMomentum;
	obj.orientation = orientation;
	simulationObjects.emplace_back(obj);
}

//Identical to AddObject besides the fact that it instead uses Keplerian orbital elements and requires a specified parent.
void Simulation::AddObjectKeplerian(double mass, double radius, Color color, Model* model, const std::string& name, double semimajorAxis, double eccentricity, double inclination, double longitudeOfAscendingNode, double argumentOfPeriapsis, double xAnomaly, int anomalyType, Vector3d angularMomentum, Vector3d orientation, const std::string& parentName, bool anchored)
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

//Draws the simulation in 3D space relative to a specified position.
void Simulation::Draw(Vector3d realPosition)
{
	for (const Object& obj : simulationObjects)
	{
		obj.model->transform = MatrixRotateZYX(Vector3dToVector3(obj.orientation));
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
		obj.model->transform = MatrixIdentity();

	}
	for (const Debris& debris : debrisObjects)
	{
		DrawModel(*debris.model, static_cast<Vector3>(debris.position - realPosition), (float)debris.radius, WHITE);
	}
}

//2D labels for the simulation
void Simulation::Draw2D(Vector3d realPosition, Camera cam)
{
	for (const Object& obj : simulationObjects)
	{
		Vector2 screenPos = GetWorldToScreen(static_cast<Vector3>(obj.position - realPosition), cam);
		Vector3 toTarget = Vector3Subtract(static_cast<Vector3>(obj.position - realPosition), cam.position);
		Vector3 camForward = Vector3Subtract(cam.target, Vector3{ 0 });
		float size = std::clamp((float)log10(std::max((float)obj.radius, 1.0f)) * 5.0f, 1.0f, 50.0f);
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
//THE DEBRIS SYSTEM
//Identical to CalcAccel, however is optimized for debris objects, which do not have mass and are not affected by other debris objects, being O(n^2 + D) instead of O(n^2 + D^2). This is because debris objects are only affected by simulation objects, and not other debris objects.
Vector3d Simulation::CalcAccelDebris(const Vector3d& pos) const
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

//Runs the debris system, updating the position and velocity of each debris object based on its current acceleration and the timestep. Uses a simple symplectic Verlet method.
void Simulation::UpdateGravDebris()
{
	for (Debris& debris : debrisObjects)
	{
		debris.position += debris.velocity * timestep + 0.5 * debris.oldAccel * (timestep * timestep);
		Vector3d accel = CalcAccelDebris(debris.position);
		debris.velocity += 0.5 * (debris.oldAccel + accel) * timestep;
		debris.oldAccel = accel;
		//CollisionDebris();
	}

}

//AddObject, but for debris. Does not include color, as it would be far too computationally taxxing to render trails for thousands of debris objects.
void Simulation::AddDebris(Vector3d position, Vector3d initialVelocity, double radius, Color color, Model* model, std::string name)
{
	Debris debris;
	debris.position = position;
	debris.velocity = initialVelocity;
	debris.radius = radius;
	debris.model = model;
	debris.name = name;
	debrisObjects.emplace_back(debris);
}

//The same as AddDebris, but uses Keplerian orbital elements and requires a specified parent.
void Simulation::AddDebrisKeplerian(double mass, double radius, Model* model, const std::string& name, double semimajorAxis, double eccentricity, double inclination, double longitudeOfAscendingNode, double argumentOfPeriapsis, double xAnomaly, int anomalyType, const std::string& parentName, bool anchored)
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
