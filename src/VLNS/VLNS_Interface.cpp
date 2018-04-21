#include "Dyn_Node_Desc.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <boost/algorithm/string.hpp>
#include "Costs_Container.h"
#include "Greedy_Heuristic.h"
#include "Enabling_Graph.h"

using namespace std;

struct Dyn_Node_Desc *pstAuxNodeInfo = (struct Dyn_Node_Desc*)NULL;
extern "C" int optimize_tsp(struct Dyn_Node_Desc *pstAuxNodeInfo);

struct Costs_Container *pstCosts = (struct Costs_Container*)NULL;

void parse_gen_TSP_node_desc(char const* const strFile)
{
	ifstream myFile(strFile);
	std::string line;
	size_t iNodeCount = 0, uiNode;
	size_t uiSplusSize, uiSminusSize;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(" "), boost::token_compress_on);

			if (elems[0] == "NODE_COUNT:")
			{
				size_t uiTotalNodes = (size_t)(atoi)(elems[1].c_str());
				pstAuxNodeInfo = (Dyn_Node_Desc*)malloc(uiTotalNodes * sizeof(Dyn_Node_Desc));
			}
			else if (elems[0] == "Node:")
			{
				uiNode = (size_t)(atoi)(elems[1].c_str());
				pstAuxNodeInfo[uiNode].m_iOffset = (atoi)(elems[3].c_str());
			}
			else if (elems[0] == "S+:")
			{
				uiSplusSize = elems.size() - 1;
				if (0 == uiSplusSize)
				{
					pstAuxNodeInfo[uiNode].m_Splus = (int*)NULL;
					continue;
				}

				pstAuxNodeInfo[uiNode].m_Splus = (int*)malloc(uiSplusSize * sizeof(int));

				for (size_t uiCount = 1; uiCount < elems.size(); uiCount++)
				{
					pstAuxNodeInfo[uiNode].m_Splus[uiCount - 1] = (atoi)(elems[uiCount].c_str());
				}
			}
			else if (elems[0] == "S-:")
			{
				uiSminusSize = elems.size() - 1;
				if (0 == uiSminusSize)
				{
					pstAuxNodeInfo[uiNode].m_Sminus = (int*)NULL;
					continue;
				}

				pstAuxNodeInfo[uiNode].m_Sminus = (int*)malloc(uiSminusSize * sizeof(int));

				for (size_t uiCount = 1; uiCount < elems.size(); uiCount++)
				{
					pstAuxNodeInfo[uiNode].m_Sminus[uiCount - 1] = (atoi)(elems[uiCount].c_str());
				}
			}
		}
	}
	myFile.close();
}

void assign_new_indices(const size_t c_uiRobot, size_t uiStart, size_t uiEnd, std::vector<std::list<size_t>> &rob_seq, const Layout_LS &graph, std::unordered_map<size_t, size_t> &map_hole_new_ind)
{
	auto it_start = rob_seq[c_uiRobot].begin();
	std::advance(it_start, uiStart);

	auto it_end = it_start;
	std::advance(it_end, (int)((int)uiEnd - (int)uiStart) + 1);

	assert("IV" != graph.getType(*it_start));
	assert("IV" != graph.getType(*it_end));

	size_t uiInd = 0;
	for (auto it = it_start; it != it_end; it++)
	{
		if ("IV" == graph.getType(*it)) continue;
		map_hole_new_ind.emplace(*it, uiInd);
		uiInd++;
	}
}

inline bool Descending_sort(struct Interval a, struct Interval b) { return a.m_uiLow > b.m_uiLow; }

// the merged vec_intervals is in reverse order i.e. earliest interval is at the end
void mergeIntervals(std::vector<Interval> &vec_intervals)
{
	// Sort Intervals in decreasing order of start time
	std::sort(vec_intervals.begin(), vec_intervals.end(), Descending_sort);

	unsigned int uiIndex = 0; // Stores uiIndex of last element in output array (modified arr[])

	// Traverse all input Intervals
	for (int iCount = 0; iCount< vec_intervals.size(); iCount++)
	{
		// If this is not first Interval and overlaps
		// with the previous one
		if (uiIndex != 0 && vec_intervals[uiIndex - 1].m_uiLow <= vec_intervals[iCount].m_uiHigh)
		{
			while (uiIndex != 0 && vec_intervals[uiIndex - 1].m_uiLow <= vec_intervals[iCount].m_uiHigh)
			{
				// Merge previous and current Intervals
				vec_intervals[uiIndex - 1].m_uiHigh = max(vec_intervals[uiIndex - 1].m_uiHigh, vec_intervals[iCount].m_uiHigh);
				vec_intervals[uiIndex - 1].m_uiLow = min(vec_intervals[uiIndex - 1].m_uiLow, vec_intervals[iCount].m_uiLow);
				uiIndex--;
			}
		}
		else // Doesn't overlap with previous, add to solution
		{
			vec_intervals[uiIndex] = vec_intervals[iCount];
		}

		uiIndex++;
	}

	//removes the intervals that are no longer necessary
	vec_intervals.resize(uiIndex);
}

void populate_enabler_time_window_for_vtx(size_t uiVtx, Hole_Overlap_Enabler_Info *pInfo, const Greedy_Heuristic &heur)
{
	
}


//we assume that rob_seq does contain intermediate vertices
void populate_cost_container(const size_t c_uiRobot, size_t uiStart, size_t uiEnd, std::vector<std::list<size_t>> &rob_seq, const Greedy_Heuristic &heur, const Layout_LS &graph)
{
	std::unordered_map<size_t, size_t> map_hole_new_ind;
	assign_new_indices(c_uiRobot, uiStart, uiEnd, rob_seq, graph, map_hole_new_ind);
	


}

void perform_TSP_Move(std::string strAuxNodeFile, const Greedy_Heuristic &heur)
{
	

	assert(pstAuxNodeInfo != (struct Dyn_Node_Desc*)NULL);
	optimize_tsp(pstAuxNodeInfo);
}

//TW- time window
void populate_TW_info(const size_t c_uiVtx, const size_t c_uiRobot, struct Interval stHorizon, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, Hole_Overlap_Enabler_Info *pInfo, const Layout_LS &graph)
{
	std::vector<Interval> vec_taboo_regions;
	std::vector<Interval> vec_feasible_regions;
	size_t uiNumIntervals = 0;

	for (size_t uiOtherRobot = 0; uiOtherRobot < vec_rob_sch.size(); uiOtherRobot++)
	{
		if (c_uiRobot == uiOtherRobot) continue;

		for (auto it = vec_rob_sch[uiOtherRobot].begin(); it != vec_rob_sch[uiOtherRobot].end(); it++)
		{
			if (it->m_uiEnd <= stHorizon.m_uiLow) continue;
			if (it->m_uiStart >= stHorizon.m_uiHigh) break;

			if (false == graph.areColliding(Coll_Pair(c_uiVtx, c_uiRobot, it->m_uiInd, uiOtherRobot))) continue;

			vec_taboo_regions.emplace_back(Interval());
			vec_taboo_regions[vec_taboo_regions.size() - 1].m_uiLow = std::max(it->m_uiStart, (size_t)stHorizon.m_uiLow);
			vec_taboo_regions[vec_taboo_regions.size() - 1].m_uiHigh = std::min(it->m_uiEnd, (size_t)stHorizon.m_uiHigh);
		}
	}

	mergeIntervals(vec_taboo_regions);
	assert(vec_taboo_regions.rbegin()->m_uiLow >= stHorizon.m_uiLow);
	assert(vec_taboo_regions.begin()->m_uiHigh <= stHorizon.m_uiHigh);

	if (0 == vec_taboo_regions.size())
	{
		pInfo->m_uiNumIntervals = 1;
		pInfo->m_parIntervals = (struct Interval*)malloc(sizeof(struct Interval));
		pInfo->m_parIntervals[0].m_uiLow = stHorizon.m_uiLow;
		pInfo->m_parIntervals[0].m_uiHigh = stHorizon.m_uiHigh;
	}
	else
	{
		if (stHorizon.m_uiLow < vec_taboo_regions[0].m_uiLow) pInfo->m_uiNumIntervals++;
		pInfo->m_uiNumIntervals = pInfo->m_uiNumIntervals + vec_taboo_regions.size() - 1;
		if (vec_taboo_regions.rbegin()->m_uiHigh < stHorizon.m_uiHigh) pInfo->m_uiNumIntervals++;
		pInfo->m_parIntervals = (Interval*)malloc(pInfo->m_uiNumIntervals * sizeof(Interval));

		if (stHorizon.m_uiLow < vec_taboo_regions[0].m_uiLow)
		{
			pInfo->m_parIntervals[0].m_uiLow = stHorizon.m_uiLow;
			pInfo->m_parIntervals[0].m_uiHigh = vec_taboo_regions[0].m_uiLow;
			uiNumIntervals++;
		}

		for (size_t uiCount = 0; uiCount < vec_taboo_regions.size() - 1; uiCount++)
		{
			pInfo->m_parIntervals[uiCount + 1].m_uiLow = vec_taboo_regions[uiCount].m_uiHigh;
			pInfo->m_parIntervals[uiCount + 1].m_uiHigh = vec_taboo_regions[uiCount + 1].m_uiLow;
			uiNumIntervals++;
		}

		if (vec_taboo_regions.rbegin()->m_uiHigh < stHorizon.m_uiHigh)
		{
			pInfo->m_parIntervals[uiNumIntervals].m_uiLow = vec_taboo_regions[vec_taboo_regions.size()-1].m_uiHigh;
			pInfo->m_parIntervals[uiNumIntervals].m_uiHigh = stHorizon.m_uiHigh;
			uiNumIntervals++;
		}
		assert(uiNumIntervals == pInfo->m_uiNumIntervals);
	}	
}

size_t populate_enabler_info(const size_t c_uiVtx, const size_t c_uiRobot, const Greedy_Heuristic &heur, Hole_Overlap_Enabler_Info *pInfo, const Enabling_Graph &en_graph, const Layout_LS &graph, std::unordered_map<size_t, size_t> &map_hole_new_ind)
{
	const auto &vec_enablers = graph.get_Enablers();
	std::set<size_t> set_enablers;
	size_t uiEnablerCount = 0;
	
	for (auto it = vec_enablers[c_uiVtx].set.begin(); it != vec_enablers[c_uiVtx].set.end(); it++)
	{
		auto pr = heur.get_robot_owner(c_uiVtx);
		assert(true == pr.first);
		if (c_uiRobot == pr.second) set_enablers.emplace(map_hole_new_ind.at(it->getInd()));
		uiEnablerCount++;
	}
	
	pInfo->m_piEnablers = (int*)malloc(uiEnablerCount * sizeof(int));
	pInfo->m_uiNumEnablers = uiEnablerCount;

	uiEnablerCount = 0;
	for (auto it = set_enablers.begin(); it != set_enablers.end(); it++)
	{
		pInfo->m_piEnablers[uiEnablerCount] = *it;
		uiEnablerCount++;
	}

	size_t uiMaxTime = inFinity , uiEnabledVtx, uiEnabler, uiEnabledTime;
	const auto& map_enabled = en_graph.get_Node_vec();
	const auto& vtx_vec_enabled = map_enabled.at(c_uiVtx).get_neighs();
	auto res = heur.get_vtx_start_time(c_uiVtx);
	assert(true == res.first);
	const size_t c_ui_c_uiVtx_ProcTime = graph.getTime(c_uiVtx);
	const size_t c_ui_c_uiVtx_CompTime = res.second + c_ui_c_uiVtx_ProcTime; //completion time of c_uiVtx

	//*it_enabled are the vertices enabled by c_uiVtx
	for (auto it_enabled = vtx_vec_enabled.begin(); it_enabled != vtx_vec_enabled.end(); it_enabled++)
	{
		size_t uiEnablingVts = 0;
		uiEnabledVtx = *it_enabled;
		res = heur.get_vtx_start_time(uiEnabledVtx);
		assert(true == res.first);
		size_t uiEnabledVtxStartTime = res.second;

		//need to check if uiEnabledVtx is actually enabled by uiVtx
		for (auto it_enabler = vec_enablers.at(uiEnabledVtx).set.begin(); it_enabler != vec_enablers.at(uiEnabledVtx).set.end(); it_enabler++)
		{
			uiEnabler = it_enabler->getInd();
			res = heur.get_vtx_start_time(uiEnabler);
			assert(true == res.first);
			uiEnabledTime = res.second + graph.getTime(uiEnabler);

			if (uiEnabledVtxStartTime >= uiEnabledTime) uiEnablingVts++;
		}

		if (1 == uiEnablingVts)
		{
			if (uiEnabledVtxStartTime >= c_ui_c_uiVtx_CompTime)
			{
				uiMaxTime = std::min(uiMaxTime, uiEnabledVtxStartTime - c_ui_c_uiVtx_ProcTime);
			}
		}
	}
	return uiMaxTime;
}