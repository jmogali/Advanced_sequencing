#include "Greedy_Heuristic.h"

const Alternative_Graph& Greedy_Heuristic::get_complete_alt_graph(int iOption)
{
	if (false == m_bComplete_Graph)
	{
		if ((1 == iOption) || (3 == iOption)) insert_missing_enabling_arcs();
		if ((2 == iOption) || (3 == iOption)) fill_back_pruned_collision_arcs();
		m_bComplete_Graph = true;
	}
	return m_alt_graph;
}

void Greedy_Heuristic::fill_back_pruned_collision_arcs()
{
	for (size_t uiRobot1 = 0; uiRobot1 < m_uiNumRobots; uiRobot1++)
	{
		for (size_t uiRobot2 = uiRobot1 + 1; uiRobot2 < m_uiNumRobots; uiRobot2++)
		{
			fill_back_pruned_collision_arcs(uiRobot1, uiRobot2);
		}
	}
}

void Greedy_Heuristic::fill_back_pruned_collision_arcs(size_t uiRobot1, size_t uiRobot2)
{
	bool bCollide, bArcExists;
	size_t uiPos1 = 0 , uiPos2;
	size_t uiNext1, uiNext2;

	for (auto it_1 = m_rob_seq[uiRobot1].begin(); it_1 != m_rob_seq[uiRobot1].end(); it_1++ , uiPos1++)
	{
		auto res1 = m_alt_graph.get_next_vtx_same_job(*it_1);
		if (false == res1.first) continue;
		uiNext1 = res1.second;

		uiPos2 = 0;
		for (auto it_2 = m_rob_seq[uiRobot2].begin(); it_2 != m_rob_seq[uiRobot2].end(); it_2++ , uiPos2++)
		{
			if (uiPos2 >= m_coll_filter.get_lower_bound_pos(*it_1, uiRobot2)) break;
			if (uiPos1 >= m_coll_filter.get_lower_bound_pos(*it_2, uiRobot1)) break;

			bCollide = m_graph.areColliding(Coll_Pair(*it_1, uiRobot1, *it_2, uiRobot2));
			if (false == bCollide) continue;

			auto res2 = m_alt_graph.get_next_vtx_same_job(*it_2);
			if (false == res2.first) continue;
			uiNext2 = res2.second;

			if (m_rob_hole_times.at(uiRobot1).at(uiNext1).m_uiStartTime <= m_rob_hole_times.at(uiRobot2).at(*it_2).m_uiStartTime)
			{
				bArcExists = m_alt_graph.containsPrecArc(arc(uiNext1, *it_2));
				if(false == bArcExists) m_alt_graph.add_prec_arc(uiNext1, *it_2, 0);
			}
			else if(m_rob_hole_times.at(uiRobot2).at(uiNext2).m_uiStartTime <= m_rob_hole_times.at(uiRobot1).at(*it_1).m_uiStartTime)
			{
				bArcExists = m_alt_graph.containsPrecArc(arc(uiNext2, *it_1));
				if(false == bArcExists) m_alt_graph.add_prec_arc(uiNext2, *it_1, 0);
			}
		}
	}
}

void Greedy_Heuristic::insert_missing_enabling_arcs()
{
	for (size_t uiRobot = 0; uiRobot < m_uiNumRobots; uiRobot++)
	{
		insert_missing_enabling_arcs(uiRobot);
	}
}

void Greedy_Heuristic::insert_missing_enabling_arcs(const size_t c_uiGivenRobot)
{
	size_t uiMinVtx, uiEnablerVtx, uiEarliestTime, uiTime;
	bool bArcExists;
	const auto &vec_enablers = m_graph.get_Enablers();

	for (auto it = m_rob_seq[c_uiGivenRobot].begin(); it != m_rob_seq[c_uiGivenRobot].end(); it++)
	{
		uiEarliestTime = std::numeric_limits<size_t>::max();
		uiMinVtx = std::numeric_limits<size_t>::max();
		if ("H" != m_graph.getType(*it)) continue;
		
		if (m_map_enabler_pos_vert.end() != m_map_enabler_pos_vert.find(*it))
		{
			for (size_t uiOtherRobot = 0; uiOtherRobot < m_uiNumRobots; uiOtherRobot++)
			{
				if (uiOtherRobot == c_uiGivenRobot) continue;
				uiEnablerVtx = m_map_enabler_pos_vert.at(*it).at(uiOtherRobot).second;
				uiTime = m_rob_hole_times.at(uiOtherRobot).at(uiEnablerVtx).m_uiStartTime;
				if (uiEarliestTime > uiTime)
				{
					uiEarliestTime = uiTime;
					uiMinVtx = uiEnablerVtx;
				}
			}
		}
		else
		{
			for (auto it_enabler = vec_enablers.at(*it).set.begin(); it_enabler != vec_enablers.at(*it).set.end(); it_enabler++)
			{
				uiEnablerVtx = it_enabler->getInd();
				auto res = m_alt_graph.get_next_vtx_same_job(uiEnablerVtx);
				if (false == res.first) continue;
				uiEnablerVtx = res.second;

				size_t uiEnbalerOwner = m_alt_graph.get_vertex_ownership(uiEnablerVtx);
				uiTime = m_rob_hole_times.at(uiEnbalerOwner).at(uiEnablerVtx).m_uiStartTime;

				if (uiEarliestTime > uiTime)
				{
					uiEarliestTime = uiTime;
					uiMinVtx = uiEnablerVtx;
				}
			}
		}

#ifdef WINDOWS
		assert(std::numeric_limits<size_t>::max() != uiMinVtx);
#else
		if (std::numeric_limits<size_t>::max() == uiMinVtx)
		{
			cout << "Mistake in enablers timings \n";
			exit(-1);
		}
#endif

		bArcExists = m_alt_graph.containsPrecArc(arc(uiMinVtx, *it));
		if (false == bArcExists) m_alt_graph.add_prec_arc(uiMinVtx, *it, 0);
	}
}

