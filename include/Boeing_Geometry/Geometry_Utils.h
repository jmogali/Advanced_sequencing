#pragma once
#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include "Coordinates.h"
#include <algorithm>

bool onSegment(const Coordinates& p, const Coordinates& q, const Coordinates &r)	
{
	if ((q.get_X_loc() <= std::max(p.get_X_loc(), r.get_X_loc())) && (q.get_X_loc() >= std::min(p.get_X_loc(), r.get_X_loc())) &&
		(q.get_Y_loc() <= std::max(p.get_Y_loc(), r.get_Y_loc())) && (q.get_Y_loc() >= std::min(p.get_Y_loc(), r.get_Y_loc())))
	{
		return true;
	}
	return false;
}

int orientation(const Coordinates &p, const Coordinates &q, const Coordinates &r)
{
	double val = ((q.get_Y_loc() - p.get_Y_loc()) * (r.get_X_loc() - q.get_X_loc())) - ((q.get_X_loc() - p.get_X_loc()) * (r.get_Y_loc() - q.get_Y_loc()));
	if (val == 0) return 0;  // colinear
	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool doIntersect(const Coordinates &p1, const Coordinates &q1, const Coordinates &p2, const Coordinates &q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

#endif
