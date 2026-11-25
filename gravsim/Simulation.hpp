#pragma once

#include <string>
#include "Object.h"
#include <raylib.h>
#include "Constants.h"
#include "raymathDouble.h"
#include "Debris.h"
#include <vector>

class Simulation
{
public:
	double timestep = 0.0001;
	double olddT = 0.0001;
	double intendedTimestep = 0.0001;
	int substeps = 0;
	double substepSize = 0.0001;
	double tolerance = 1e-9;
	std::vector<Object> simulationObjects;
	std::vector<Debris> debrisObjects;

private:
	Vector3d CalcAccel(const Vector3d& pos, const Vector3d& vel, Vector3d Object::* stagePos, Vector3d Object::* stageVel, double mass) const;
	void Collision(int& track, bool& fsalValid, double time);
	double ConvertEccentricToTrueAnomaly(double eccentricAnomaly, double eccentricity);
	void FindObjectByName(const std::string& name, Object*& foundObject);

	double IAS15Full(double step, double tolerance);
	double DOP853Full(double step, double tolerance, bool& fsalValid);
	double RK43Full(double step, double tolerance, bool& fsalValid);
	double RK21Full(double step, double tolerance, bool& fsalValid);
	void EulerFull(double step);

	void FullAdaptiveIAS15(int& objectToTrack, double& globalTime, int providedMaxSubsteps);
	void FullAdaptiveDOP853(int& objectToTrack, double& globalTime, int providedMaxSubsteps);
	void FullAdaptiveRK43(int& objectToTrack, double& globalTime, int providedMaxSubsteps);
	void FullAdaptiveRK21(int& objectToTrack, double& globalTime, int providedMaxSubsteps);

	void Yoshida8Full(double step);
	void SymplecticEulerFull(double step);

	Vector3d CalcAccelDebris(const Vector3d& pos) const;
	void UpdateGravDebris();

public:
	void Update(int& objectToTrack, double& globalTime, int integrator, int maxSubSteps);
	void AddObject(double mass, Vector3d position, Vector3d initialVelocity, Vector3d angularMomentum, Vector3d orientation, double radius, Color color, Model* model, const std::string& name, bool anchored);
	void AddObjectKeplerian(double mass, double radius, Color color, Model* model, const std::string& name, double semimajorAxis, double eccentricity, double inclination, double longitudeOfAscendingNode, double argumentOfPeriapsis, double xAnomaly, int anomalyType, Vector3d angularMomentum, Vector3d orientation, const std::string& parentName = "Sun", bool anchored = false);
	void Draw(Vector3d realPosition);
	void Draw2D(Vector3d realPosition, Camera cam);
	void AddDebris(Vector3d position, Vector3d initialVelocity, double radius, Color color, Model* model, std::string name);
	void AddDebrisKeplerian(double mass, double radius, Model* model, const std::string& name, double semimajorAxis, double eccentricity, double inclination, double longitudeOfAscendingNode, double argumentOfPeriapsis, double xAnomaly, int anomalyType, const std::string& parentName = "Sun", bool anchored = false);
};
