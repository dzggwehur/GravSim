#pragma once
constexpr double GRAVITATIONAL_CONSTANT = 6.6743e-11;
constexpr double LOBATTO_SQRT21 = 4.5825756949558400065880471937280084889844565767679719026072421239;
constexpr double C = 299792458.0;
constexpr double C_SQUARED = 89875517873681760.0;
constexpr double INV_CSQUARED = 1.0 / C_SQUARED;
//Non-symplectic are 0-254. Symplectic integrators are 255 and above for organizational purposes.
enum Integrators
{
	IAS15 = 0,
	DOP853 = 1,
	RK43 = 2,
	RK21 = 3,
	ExplicitEuler = 4,

	Yoshida8 = 255,
	SymplecticEuler = 258,
};
//Convert int to integrator name string
constexpr const char* IntegratorName(int integrator)
{
	switch (integrator)
	{
	case IAS15:
		return "IAS15";
	case DOP853:
		return "DOP853";
	case RK43:
		return "RK43";
	case RK21:
		return "RK21";
	case ExplicitEuler:
		return "Explicit Euler";


	case Yoshida8:
		return "Yoshida8";
	case SymplecticEuler:
		return "Symplectic Euler";


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