#pragma once
#include <raylib.h>
#include "raymathDouble.h"
#include "CameraAdvanced.h"
#include <deque>
class Object
{
	public:
		double mass=0.0f;
		double radius=0.0f;
		Model* model=nullptr;
		std::deque<Vector3d> trail;
		Vector3d AppliedImpulse{};
		Vector3d accel1{};
		Vector3d accel2{};
		Vector3d accel3{};
		Vector3d accel4{};
		Vector3d position{};
		Vector3d velocity{};
		Vector3d position2{};
		Vector3d velocity2{};
		Vector3d position3{};
		Vector3d velocity3{};
		Vector3d position4{};
		Vector3d velocity4{};
		Color color=WHITE;
		std::string name="";
		bool destroyed=false;
};