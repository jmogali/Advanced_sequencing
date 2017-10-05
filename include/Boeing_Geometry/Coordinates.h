#pragma once
#ifndef COORDINATES_H
#define COORDINATES_H

class Coordinates
{
	private:
		double m_dLoc_x, m_dLoc_y, m_dLoc_z;
	
	public:
		Coordinates(double dLoc_x, double dLoc_y , double dLoc_z);
		Coordinates(const Coordinates &pt);
		inline double get_X_loc() const { return m_dLoc_x; };
		inline double get_Y_loc() const { return m_dLoc_y; };
		inline double get_Z_loc() const { return m_dLoc_z; };
		double getDist_XY(const Coordinates &loc) const;
		double getDist_XYZ(const Coordinates &loc) const;
};

#endif
