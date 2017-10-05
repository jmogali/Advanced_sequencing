#include "Coordinates.h"
#include <math.h>

Coordinates::Coordinates(double dLoc_x, double dLoc_y, double dLoc_z)
{
	m_dLoc_x = dLoc_x;
	m_dLoc_y = dLoc_y;
	m_dLoc_z = dLoc_z;
}

Coordinates::Coordinates(const Coordinates &pt)
{
	m_dLoc_x = pt.m_dLoc_x;
	m_dLoc_y = pt.m_dLoc_y;
	m_dLoc_z = pt.m_dLoc_z;
}

double Coordinates::getDist_XY(const Coordinates &loc) const
{
	double dx = loc.get_X_loc() - m_dLoc_x;
	double dy = loc.get_Y_loc() - m_dLoc_y;
	return sqrt((dx * dx) + (dy * dy));
}

double Coordinates::getDist_XYZ(const Coordinates &loc) const
{
	double dx = loc.get_X_loc() - m_dLoc_x;
	double dy = loc.get_Y_loc() - m_dLoc_y;
	double dz = loc.get_Z_loc() - m_dLoc_z;
	return sqrt((dx * dx) + (dy * dy) + (dz*dz));
}