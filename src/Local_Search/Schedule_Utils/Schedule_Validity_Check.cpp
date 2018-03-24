#include "Schedule_Validity_Check.h"

Vertex_Schedule::Vertex_Schedule(size_t uiInd, size_t uiStart, size_t uiEnd, size_t uiWait) : m_uiInd(uiInd), m_uiStart(uiStart), m_uiEnd(uiEnd), m_uiWait(uiWait)
{}

bool doIntervalsOverlap(const Vertex_Schedule& v1, const Vertex_Schedule& v2)
{
	// does open interval overlap
	if ((v2.m_uiStart < v1.m_uiEnd) && (v1.m_uiStart < v2.m_uiEnd))
		return true;
	else
		return false;
}

bool Schedule_Validity_Check::check_sequence_info(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph)
{
	size_t uiNumRobots = rob_seq.size();
	bool bValid = false;
	if (vec_rob_sch.size() != uiNumRobots)
	{
		std::cout << "Generated vertex schedule has fewer robots than input number of robots \n";
		return false;
	}

	for (size_t uiRobot = 0; uiRobot < uiNumRobots; uiRobot++)
	{
		bValid = check_sequence_info_robot(uiRobot, rob_seq, vec_rob_sch, graph);
		if (false == bValid)
		{
			cout << "Basic sequence check failed for robot " << uiRobot << endl;
			return false;
		}
	}
	return true;
}

bool Schedule_Validity_Check::check_sequence_info_robot(size_t uiRobot, const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph)
{
	size_t uiHoleInd1, uiHoleInd2;
	size_t uiCount1 = 0, uiCount2 = 1;
	const auto &vec_rob_iv = graph.get_IV_Vec();

	for (auto it_hole1 = rob_seq[uiRobot].cbegin() ; it_hole1 != rob_seq[uiRobot].cend() ; it_hole1++ , uiCount1 = uiCount1 + 2, uiCount2 = uiCount2 + 2)
	{
		uiHoleInd1 = *it_hole1;

		if (vec_rob_sch[uiRobot][uiCount1].m_uiInd != uiHoleInd1) return false;
		if (vec_rob_sch[uiRobot][uiCount1].m_uiEnd - vec_rob_sch[uiRobot][uiCount1].m_uiStart < graph.getTime(uiHoleInd1)) return false;

		if (uiHoleInd1 == *rob_seq[uiRobot].cbegin()) break;
		auto it_hole2 = it_hole1;
		it_hole2++;
		uiHoleInd2 = *it_hole2;
		const auto &vec_iv = vec_rob_iv[uiRobot].map.at(uiHoleInd1).map.at(uiHoleInd2).vec;

#ifdef WINDOWS
		assert(1 == vec_iv.size());
#else
		if (1 != vec_iv.size()) exit(-1);
#endif

		if (vec_rob_sch[uiRobot][uiCount1 + 1].m_uiInd != vec_iv[0].getInd()) return false;
		if (vec_rob_sch[uiRobot][uiCount1 + 1].m_uiEnd - vec_rob_sch[uiRobot][uiCount1 + 1].m_uiStart < graph.getTime(vec_iv[0].getInd())) return false;
	}
	return true;
}

void Schedule_Validity_Check::populate_start_timemap(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times)
{
	for (size_t uiRobot = 0; uiRobot < vec_rob_sch.size(); uiRobot++)
	{
		for (size_t uiCount = 0; uiCount < vec_rob_sch[uiRobot].size(); uiCount++)
		{
			auto it_insert = map_start_times.emplace(vec_rob_sch[uiRobot][uiCount].m_uiInd , std::make_pair(uiRobot, uiCount));
#ifdef WINDOWS
			assert(true == it_insert.second);
#else
			if (false != it_insert.second)
			{
				cout << "Some other index already previously exists \n";
				exit(-1);
			}
#endif
		}
	}
}

bool Schedule_Validity_Check::check_collision_enabling(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph)
{
	std::unordered_map<size_t, std::pair<size_t , size_t>> map_start_times;  // <vertex index, <robot, position of vertex>>
	populate_start_timemap(vec_rob_sch, map_start_times);
	
	bool bValid = check_enabling_info(vec_rob_sch, graph, map_start_times);
	if (false == bValid) return false;

	bValid = check_collision_info(vec_rob_sch, graph, map_start_times);
	if (false == bValid) return false;

	return true;
}

bool Schedule_Validity_Check::check_enabling_info(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times)
{	
	bool bValid;
	for (size_t uiRobot = 0; uiRobot < vec_rob_sch.size(); uiRobot++)
	{
		bValid = check_enabling_info_robot(uiRobot, vec_rob_sch, graph, map_start_times);
		if (false == bValid)
		{
			cout << "Enabling failed for robot: " << uiRobot << endl;
			return false;
		}
	}
	return true;
}

bool Schedule_Validity_Check::check_enabling_info_robot(size_t uiRobot, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times)
{
	auto &vec_enabler = graph.get_Enablers();
	bool bValid;

	for (size_t uiCount = 0; uiCount < vec_rob_sch[uiRobot].size(); uiCount++)
	{
		size_t uiInd = vec_rob_sch[uiRobot][uiCount].m_uiInd;
		if ("H" != graph.getType(uiInd)) continue;
		bValid = false;

		for(auto it_enabler = vec_enabler.at(uiInd).set.begin() ; it_enabler != vec_enabler.at(uiInd).set.end() ; it_enabler++)
		{
			auto pr = map_start_times.at(it_enabler->getInd());
			if (vec_rob_sch[pr.first][pr.second].m_uiEnd <= vec_rob_sch[uiRobot][uiCount].m_uiStart) 
			{
				bValid = true;
				break;
			}
		}

		if (false == bValid)
		{
			cout << "Enabling failed for vertex " << uiInd << endl;
			return false;
		}
	}
	return true;
}

bool Schedule_Validity_Check::check_collision_info(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times)
{
	bool bValid;
	for (size_t uiRobot = 0; uiRobot < vec_rob_sch.size(); uiRobot++)
	{
		bValid = check_collision_info_robot(uiRobot, vec_rob_sch, graph, map_start_times);
		if (false == bValid)
		{
			cout << "Collision check failed for robot: " << uiRobot << endl;
			return false;
		}
	}
	return true;
}

bool Schedule_Validity_Check::check_collision_info_robot(size_t uiRobot, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times)
{
	bool bValid;
	for (size_t uiOtherRobot = 0; uiOtherRobot < vec_rob_sch.size(); uiOtherRobot++)
	{
		if (uiOtherRobot == uiRobot) continue;
		bValid = check_collision_info_robot_robot(uiRobot, uiOtherRobot, vec_rob_sch, graph, map_start_times);
		if (false == bValid)
		{
			cout << "Collision check failed for robot: " << uiRobot << " with robot: " << uiOtherRobot << endl;
			return false;
		}
	}
	return true;
}

bool Schedule_Validity_Check::check_collision_info_robot_robot(size_t uiRobot, size_t uiOtherRobot, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph, const std::unordered_map<size_t, std::pair<size_t, size_t>> &map_start_times)
{
	for (size_t uiCount1 = 0; uiCount1 < vec_rob_sch[uiRobot].size(); uiCount1++)
	{
		size_t uiInd1 = vec_rob_sch[uiRobot][uiCount1].m_uiInd;
		for (size_t uiCount2 = 0; uiCount2 < vec_rob_sch[uiOtherRobot].size(); uiCount2++)
		{
			if (false == doIntervalsOverlap(vec_rob_sch[uiRobot][uiCount1], vec_rob_sch[uiOtherRobot][uiCount2])) continue;

			if (true == graph.areColliding(Coll_Pair(uiInd1, uiRobot, vec_rob_sch[uiOtherRobot][uiCount2].m_uiInd, uiOtherRobot)))
			{
				cout << "Collision detected for Indices (" << uiInd1 << "with robot:" << uiRobot << " and " << vec_rob_sch[uiOtherRobot][uiCount2].m_uiInd << " with robot: " << uiOtherRobot <<endl;
				return false;
			}
		}
	}
	return true;
}

bool Schedule_Validity_Check::check_vertex_schedule(const std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Layout_LS &graph)
{
	bool bValid;

	bValid = check_sequence_info(rob_seq, vec_rob_sch, graph);
	if (false == bValid) return false;

	bValid = check_collision_enabling(vec_rob_sch, graph);
	if (false == bValid) return false;

	return true;
}