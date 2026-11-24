#pragma once
#include <raylib.h>
#include "raymathDouble.h"
#include "CameraAdvanced.h"
#include <deque>
class Object
{
	public:
		double mass=0.0;
		double radius=0.0;
		Model* model=nullptr;
		std::deque<Vector3d> trail;
		Vector3d AppliedImpulse{};
		Vector3d accel1{}, accel2{}, accel3{}, accel4{}, accel5{}, accel6{}, accel7{}, accel8{}, accel9{}, accel10{}, accel11{}, accel12{}, accel13{}, accel14{}, accel15{}, accel16{};
		Vector3d position{}, position2{}, position3{}, position4{}, position5{}, position6{}, position7{}, position8{}, position9{}, position10{}, position11{}, position12{}, position13{},position14{}, position15{}, position16{};
		Vector3d velocity{}, velocity2{}, velocity3{}, velocity4{}, velocity5{}, velocity6{}, velocity7{}, velocity8{}, velocity9{}, velocity10{}, velocity11{}, velocity12{}, velocity13{}, velocity14{}, velocity15{}, velocity16{};
		Vector3d g[7]{}, b[7]{}, e[7]{}; 
		Vector3d csx{}, csv{};

		Color color=WHITE;
		std::string name="";
		bool destroyed=false;
		bool anchored = false;
};