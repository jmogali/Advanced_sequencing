#include "Robot.h"
#include "Windows_Linux.h"
#ifdef WINDOWS
#include "stdafx.h"
#endif
#include <iostream>
using namespace std;

Robot::Robot(const Coordinates &loc , const std::set<size_t> &set_frames) : m_loc(loc)
{
	for (auto it = set_frames.begin(); it != set_frames.end(); it++)
	{
		m_set_Frames.emplace(*it);
	}
}

Robot::Robot(const Robot &robot) : m_loc(robot.m_loc)
{
	for (auto it = robot.m_set_Frames.begin(); it != robot.m_set_Frames.end(); it++)
	{
		m_set_Frames.emplace(*it);
	}
}

bool Robot::isHoleReachable(const Hole &hole)
{	
	if (false == isPointReachable(hole.getLoc()))
		return false;

	if (false == isPointReachable(hole.getFrame()))
		return false;

	return true;
}

bool Robot::isPointReachable(const Coordinates &coord)
{
	return true;
}

bool Robot::isPointReachable(size_t uiFrameNum)
{
	if (m_set_Frames.find(uiFrameNum) == m_set_Frames.end())
		return false;

	return true;
}

bool Robot::getPoseToPoint(const Coordinates &coord, Polygon &pose)
{
	if(false == isPointReachable(coord)) return false;

	Polygon temp;
	//Temporary
	temp.outer().push_back(Point(coord.get_X_loc() + 1, coord.get_Y_loc() + 1));
	temp.outer().push_back(Point(coord.get_X_loc() - 1, coord.get_Y_loc() - 1));
	temp.outer().push_back(Point(m_loc.get_X_loc() + 1, m_loc.get_Y_loc() + 1));
	temp.outer().push_back(Point(m_loc.get_X_loc() - 1, m_loc.get_Y_loc() - 1));

	boost::geometry::convex_hull(temp, pose);
	
	/*for (size_t uiCount = 0; uiCount < pose.outer().size(); uiCount++)
	{
		cout << pose.outer().at(uiCount).get<0>() << " , " << pose.outer().at(uiCount).get<1>() << "\n";
	}
	cout << endl;*/
	
	return true;
}

bool Robot::getPoseEnvelope(const Coordinates &coord1, const Coordinates &coord2, Polygon &pose)
{
	Polygon temp1 , temp2;
	if (false == getPoseToPoint(coord1 , temp1)) return false;
	if (false == getPoseToPoint(coord2 , temp2)) return false;

	Polygon temp;
	for (size_t uiCount = 0; uiCount < temp1.outer().size(); uiCount++)
	{
		temp.outer().push_back(Point(temp1.outer().at(uiCount).get<0>() , temp1.outer().at(uiCount).get<1>()));
	}
	
	for (size_t uiCount = 0; uiCount < temp2.outer().size(); uiCount++)
	{
		temp.outer().push_back(Point(temp2.outer().at(uiCount).get<0>(), temp2.outer().at(uiCount).get<1>()));
	}
	
	boost::geometry::convex_hull(temp, pose);
	return true;
}

bool Robot::getPoseEnvelope(const std::vector<Coordinates> &vec_coord, Polygon &pose)
{
	if (vec_coord.size() == 1)
	{
		return getPoseToPoint(vec_coord[0], pose);
	}
	else if (vec_coord.size() <= 2)
	{
		return getPoseEnvelope(vec_coord[0] , vec_coord[1] , pose);
	}

	for (size_t uiCount = 0; uiCount < vec_coord.size(); uiCount++)
	{
		if (false == isPointReachable(vec_coord[uiCount])) return false;
	}

	std::vector<Polygon> vec_result;
	getPoseEnvelope(vec_coord[0], vec_coord[1], pose);

	for (size_t uiCount = 1; uiCount < vec_coord.size() - 1; uiCount++)
	{
		Polygon temp;
		getPoseEnvelope(vec_coord[uiCount], vec_coord[uiCount + 1], temp);
		boost::geometry::union_(temp, pose, vec_result);
		assert(1 == vec_result.size());
		pose.clear();

		for (size_t uiVtx = 0; uiVtx < vec_result[0].outer().size(); uiVtx++)
		{
			pose.outer().push_back(Point(vec_result[0].outer().at(uiVtx).get<0>(), vec_result[0].outer().at(uiVtx).get<1>()));
		}

		for (size_t uiPolyCount = 0; uiPolyCount < vec_result.size(); uiPolyCount++)
		{
			vec_result[uiPolyCount].clear();
		}
	}
	return true;
}

double Robot::compute_time(const Coordinates &loc1, const Coordinates &loc2)
{
	double dTime = loc1.getDist_XY(loc2);
	return (dTime / ROBOT_SPEED);
}