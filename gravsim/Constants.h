#pragma once
constexpr double GRAVITATIONAL_CONSTANT = 6.6743e-11;
constexpr double LOBATTO_SQRT21 = 4.5825756949558400065880471937280084889844565767679719026072421239;
enum Integrators
{
	IAS15 = 0,
	Yoshida8 = 1,
	DOP853 = 2,
	RK43 = 3,
	RK21 = 4,
};
enum Anomalies
{
	MeanAnomaly = 0,
	TrueAnomaly = 1,
	EccentricAnomaly = 2,

};