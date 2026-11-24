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
//Convert int to integrator name string
constexpr const char* IntegratorName(int integrator)
{
	switch (integrator)
	{
	case IAS15:
		return "IAS15";
	case Yoshida8:
		return "Yoshida8";
	case DOP853:
		return "DOP853";
	case RK43:
		return "RK43";
	case RK21:
		return "RK21";
	default:
		return "Unknown Integrator";
	}
}
enum Anomalies
{
	MeanAnomaly = 0,
	TrueAnomaly = 1,
	EccentricAnomaly = 2,

};