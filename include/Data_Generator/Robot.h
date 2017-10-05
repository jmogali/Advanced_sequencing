#pragma once
#ifndef ROBOT_H
#define ROBOT_H

#include "Data_Constants.h"
#include "Coordinates.h"

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)

#include "Hole.h"

class Robot
{
	private:
		Coordinates m_loc;
		std::set<size_t> m_set_Frames;
		bool isPointReachable(const Coordinates &coord);
		bool isPointReachable(size_t uiFrameNum);

	public:
		Robot(const Coordinates &loc, const std::set<size_t> &set_frames);
		Robot(const Robot &robot);
		bool isHoleReachable(const Hole &hole);
		bool getPoseToPoint(const Coordinates &coord, Polygon &pose);
		bool getPoseEnvelope(const Coordinates &coord1, const Coordinates &coord2 , Polygon &pose);
		bool getPoseEnvelope(const std::vector<Coordinates> &vec_coord, Polygon &pose);
		double compute_time(const Coordinates &loc1 , const Coordinates &loc2);
};

#endif
