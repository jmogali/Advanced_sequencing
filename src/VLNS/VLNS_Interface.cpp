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
#include "VLNS_Interface.h"

using namespace std;

struct Dyn_Node_Desc *pstAuxNodeInfo = (struct Dyn_Node_Desc*)NULL;
size_t uiTotalAuxNodes;

//extern "C" int optimize_tsp(struct Dyn_Node_Desc *pstAuxNodeInfo);
extern "C" int optimize_tsp(struct Dyn_Node_Desc *pstAuxNodeInfo, struct Costs_Container *pstCosts, int iNumVts, int kVal, int* new_tour, int bFirstIter, const char* cFolderPath, const int c_uiStartTime);
extern "C" void free_buffers();

struct Costs_Container *pstCosts = (struct Costs_Container*)NULL;

void free_Aux_Node_Info_container();
void free_cost_container();
size_t populate_vtx_end_horizon(const size_t c_uiVtx, const size_t c_uiRobot, const Greedy_Heuristic &heur, unsigned int uiHole, const Enabling_Graph &en_graph, const Layout_LS &graph, const std::unordered_map<size_t, size_t> &map_hole_new_ind);
void populate_enabler_info(const size_t c_uiVtx, const size_t c_uiRobot, const Greedy_Heuristic &heur, unsigned int uiHole, const Enabling_Graph &en_graph, const Layout_LS &graph, const std::unordered_map<size_t, size_t> &map_hole_new_ind, const std::unordered_set<size_t> &set_vts);
void populate_TW_info(const size_t c_uiVtx, const size_t c_uiRobot, struct Interval stHorizon, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, unsigned int uiHole, const Layout_LS &graph);
size_t populate_cost_container(const size_t c_uiRobot, size_t uiStartInd, size_t uiEndInd, std::vector<std::list<size_t>> &rob_seq, const Greedy_Heuristic &heur, const Layout_LS &graph, const Enabling_Graph &en_graph, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::unordered_map<size_t, size_t> &map_hole_new_ind);
void populate_traversal_times(size_t uiStartVtx, size_t uiEndVtx, const size_t c_uiRobot, const std::unordered_map<size_t, size_t> &map_hole_new_ind, const Layout_LS &graph);
void populate_enabler_time_window_for_vtx(const size_t c_uiStartTime, const size_t c_uiPlanEndTime, const size_t c_uiVtx, unsigned int uiHole, const Greedy_Heuristic &heur, const std::unordered_map<size_t, size_t> &map_hole_new_ind, const Layout_LS &graph, const Enabling_Graph &en_graph, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const std::unordered_set<size_t> &set_vts);
void select_sub_seq_for_opt(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::tuple<size_t, size_t, size_t> &rob_start_end, const Layout_LS &graph);
void generate_new_tour(const size_t c_uiRobot, const size_t c_uiStartInd, const size_t c_uiEndInd, std::list<size_t> &old_tour, int* new_tour, const size_t c_uiTour_Len, const Layout_LS &graph, const std::unordered_map<size_t, size_t> &map_hole_new_ind);

void parse_gen_TSP_node_desc(char const* const strFile)
{
	ifstream myFile(strFile);
	std::string line;
	size_t iNodeCount = 0, uiNode;
	size_t uiSplusSize, uiSminusSize, uiSSize;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(" "), boost::token_compress_on);

			if (elems[0] == "NODE_COUNT:")
			{
				size_t uiTotalNodes = (size_t)(atoi)(elems[1].c_str());
				uiTotalAuxNodes = uiTotalNodes;
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
				uiSSize = uiSplusSize;
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
				assert(uiSSize == uiSminusSize);
				pstAuxNodeInfo[uiNode].m_ui_S_Size = (int)uiSminusSize;
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
	assert(map_hole_new_ind.empty());
	auto it_start = rob_seq[c_uiRobot].begin();
	std::advance(it_start, uiStart);

	auto it_end = it_start;

	//it_end has a +1, because of the way in which the for loop is defined below
	std::advance(it_end, (int)((int)uiEnd - (int)uiStart)); 

	assert("IV" != graph.getType(*it_start));
	assert("IV" != graph.getType(*it_end));

	size_t uiInd = 0;
	for (auto it = it_start; it != rob_seq[c_uiRobot].end(); it++)
	{
		if ("IV" == graph.getType(*it)) continue;
		map_hole_new_ind.emplace(*it, uiInd);
		if (it == it_end) break;
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

void populate_enabler_time_window_for_vtx(const size_t c_uiStartTime, const size_t c_uiPlanEndTime, const size_t c_uiVtx, unsigned int uiHole, const Greedy_Heuristic &heur, const std::unordered_map<size_t, size_t> &map_hole_new_ind, const Layout_LS &graph, const Enabling_Graph &en_graph, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const std::unordered_set<size_t> &set_vts)
{
	auto res = heur.get_robot_owner(c_uiVtx);
	assert(true == res.first);
	const size_t c_uiRobot = res.second;
	
	const size_t c_uiEndTime = std::min(populate_vtx_end_horizon(c_uiVtx, c_uiRobot, heur, uiHole, en_graph, graph, map_hole_new_ind), c_uiPlanEndTime);
	
	Interval stHorizon;
	stHorizon.m_uiLow = (int)c_uiStartTime;
	stHorizon.m_uiHigh = (int)c_uiEndTime;

	populate_TW_info(c_uiVtx, c_uiRobot, stHorizon, vec_rob_sch, uiHole, graph);
	populate_enabler_info(c_uiVtx, c_uiRobot, heur, uiHole, en_graph, graph, map_hole_new_ind, set_vts);
}

void populate_traversal_times(size_t uiStartVtx, size_t uiEndVtx, const size_t c_uiRobot, const std::unordered_map<size_t, size_t> &map_hole_new_ind, const Layout_LS &graph)
{
	size_t uiVtx1, uiVtx2;
	size_t uiVtx1NewInd, uiVtx2NewInd;
	size_t uiTime12, uiTime21;
	const auto &vec_rob_iv = graph.get_IV_Vec();
	
	for (auto it1 = map_hole_new_ind.begin(); it1 != map_hole_new_ind.end(); it1++)
	{
		uiVtx1 = it1->first;
		uiVtx1NewInd = it1->second;

		auto it2 = it1;
		it2++;
		for (; it2 != map_hole_new_ind.end(); it2++)
		{
			uiVtx2 = it2->first;
			uiVtx2NewInd = it2->second;

			if ((uiVtx1 == uiEndVtx) && (uiVtx2 == uiStartVtx)) pstCosts->m_pparTravTime[uiVtx1NewInd][uiVtx2NewInd] = 0;
			else if((uiVtx1 == uiEndVtx) && (uiVtx2 != uiStartVtx)) pstCosts->m_pparTravTime[uiVtx1NewInd][uiVtx2NewInd] = inFinity;
			else if(vec_rob_iv[c_uiRobot].map.at(uiVtx1).map.end() == vec_rob_iv[c_uiRobot].map.at(uiVtx1).map.find(uiVtx2))
			{			
				pstCosts->m_pparTravTime[uiVtx1NewInd][uiVtx2NewInd] = inFinity;
			}
			else
			{
				const auto &vec_iv_12 = vec_rob_iv[c_uiRobot].map.at(uiVtx1).map.at(uiVtx2).vec;
				uiTime12 = 0;

				for (auto it_iv = vec_iv_12.begin(); it_iv != vec_iv_12.end(); it_iv++) uiTime12 += graph.getTime(*it_iv);
				pstCosts->m_pparTravTime[uiVtx1NewInd][uiVtx2NewInd] = (int)uiTime12;
			}
			
			if ((uiVtx2 == uiEndVtx) && (uiVtx1 == uiStartVtx)) pstCosts->m_pparTravTime[uiVtx2NewInd][uiVtx1NewInd] = 0;
			else if ((uiVtx2 == uiEndVtx) && (uiVtx1 != uiStartVtx)) pstCosts->m_pparTravTime[uiVtx2NewInd][uiVtx1NewInd] = inFinity;
			else if (vec_rob_iv[c_uiRobot].map.at(uiVtx2).map.find(uiVtx1) == vec_rob_iv[c_uiRobot].map.at(uiVtx2).map.end())
			{
				pstCosts->m_pparTravTime[uiVtx2NewInd][uiVtx1NewInd] = inFinity;
			}
			else
			{
				const auto &vec_iv_21 = vec_rob_iv[c_uiRobot].map.at(uiVtx2).map.at(uiVtx1).vec;
				uiTime21 = 0;

				for (auto it_iv = vec_iv_21.begin(); it_iv != vec_iv_21.end(); it_iv++) uiTime21 += graph.getTime(*it_iv);
				pstCosts->m_pparTravTime[uiVtx2NewInd][uiVtx1NewInd] = (int)uiTime21;
			}
		}
	}
}

//we assume that rob_seq does contain intermediate vertices
size_t populate_cost_container(const size_t c_uiRobot, size_t uiStartInd, size_t uiEndInd, std::vector<std::list<size_t>> &rob_seq, const Greedy_Heuristic &heur, const Layout_LS &graph, const Enabling_Graph &en_graph, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::unordered_map<size_t, size_t> &map_hole_new_ind)
{
	assign_new_indices(c_uiRobot, uiStartInd, uiEndInd, rob_seq, graph, map_hole_new_ind);
		
	auto it_start = rob_seq[c_uiRobot].begin();
	std::advance(it_start, uiStartInd);
	assert(*it_start == vec_rob_sch[c_uiRobot][uiStartInd].m_uiInd);

	auto it_end = it_start;
	std::advance(it_end, uiEndInd - uiStartInd);
	assert(*it_end == vec_rob_sch[c_uiRobot][uiEndInd].m_uiInd);

	pstCosts = (struct Costs_Container*) malloc(sizeof(Costs_Container));
	pstCosts->m_iNumVtx = (int)map_hole_new_ind.size();
	pstCosts->m_parTimeWindows = (struct Hole_Overlap_Enabler_Info*) malloc(pstCosts->m_iNumVtx * sizeof(Hole_Overlap_Enabler_Info));
	pstCosts->m_parProcTime = (int*) malloc(pstCosts->m_iNumVtx * sizeof(int));

	size_t uiStartTime = vec_rob_sch[c_uiRobot][uiStartInd].m_uiStart;
	size_t uiEndTime = vec_rob_sch[c_uiRobot][uiEndInd].m_uiEnd;

	std::unordered_set<size_t> set_vts;
	for (auto it = it_start; it != rob_seq[c_uiRobot].end(); it++)
	{
		set_vts.emplace(*it);
		if (it_end == it) break;
	}

	unsigned int uiHole = 0;
	for (auto it = it_start; it != rob_seq[c_uiRobot].end(); it++)
	{
		if ("IV" == graph.getType(*it)) continue;
		populate_enabler_time_window_for_vtx(uiStartTime, uiEndTime, *it, uiHole, heur, map_hole_new_ind, graph, en_graph, vec_rob_sch, set_vts);
		
		if(0 != uiHole) pstCosts->m_parProcTime[uiHole] = (int)graph.getTime(*it);
		else pstCosts->m_parProcTime[uiHole] = 0; // we adjust the processing time of first hole alone this way.

		if (it_end == it) break;
		uiHole++;
	}

	pstCosts->m_pparTravTime = (int**)malloc(pstCosts->m_iNumVtx * sizeof(int*));
	for (uiHole = 0; uiHole < (unsigned int)pstCosts->m_iNumVtx; uiHole++)
	{
		pstCosts->m_pparTravTime[uiHole] = (int*)malloc(pstCosts->m_iNumVtx * sizeof(int));
	}

	populate_traversal_times(*it_start, *it_end, c_uiRobot, map_hole_new_ind, graph);
	return vec_rob_sch[c_uiRobot][uiStartInd].m_uiEnd;
}

void perform_TSP_Move(std::string strTSPFileFolder, std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, const Greedy_Heuristic &heur, const Layout_LS &graph, const Enabling_Graph &en_graph, int kVal)
{
	int iFirstIter = 0;
	if (pstAuxNodeInfo == (struct Dyn_Node_Desc*)NULL)
	{
		std::string strAuxNodeFile = strTSPFileFolder + "auxnode_desc.nod";
		parse_gen_TSP_node_desc(strAuxNodeFile.c_str());
		iFirstIter = 1;
	}
	assert(pstAuxNodeInfo != (struct Dyn_Node_Desc*)NULL);
	
	//<robot, start, end> , start and end do not correspond to actual hole index, but rather just their positions
	std::tuple<size_t, size_t, size_t> rob_start_end; 
	select_sub_seq_for_opt(vec_rob_sch, rob_start_end, graph);
	
	std::unordered_map<size_t, size_t> map_hole_new_ind;
	const size_t c_uiStartTime = populate_cost_container(std::get<0>(rob_start_end), std::get<1>(rob_start_end), std::get<2>(rob_start_end), rob_seq, heur, graph, en_graph, vec_rob_sch, map_hole_new_ind);
	const int c_uiTourLen = pstCosts->m_iNumVtx;

	int* new_tour;
	new_tour = (int*)malloc(c_uiTourLen * sizeof(int));

	int iRetVal = optimize_tsp(pstAuxNodeInfo, pstCosts, c_uiTourLen, kVal, new_tour, iFirstIter, strTSPFileFolder.c_str(), (int)c_uiStartTime);
	assert(0 == iRetVal);

	generate_new_tour(std::get<0>(rob_start_end), std::get<1>(rob_start_end), std::get<2>(rob_start_end), rob_seq[std::get<0>(rob_start_end)], new_tour, c_uiTourLen, graph, map_hole_new_ind);

	free(new_tour);
	new_tour = NULL;

	free_cost_container();
}

//TW- time window
void populate_TW_info(const size_t c_uiVtx, const size_t c_uiRobot, struct Interval stHorizon, const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, unsigned int uiHole, const Layout_LS &graph)
{
	std::vector<Interval> vec_taboo_regions;
	std::vector<Interval> vec_feasible_regions;
	size_t uiNumIntervals;

	for (size_t uiOtherRobot = 0; uiOtherRobot < vec_rob_sch.size(); uiOtherRobot++)
	{
		if (c_uiRobot == uiOtherRobot) continue;

		for (auto it = vec_rob_sch[uiOtherRobot].begin(); it != vec_rob_sch[uiOtherRobot].end(); it++)
		{
			if (it->m_uiEnd <= stHorizon.m_uiLow) continue;
			if (it->m_uiStart >= stHorizon.m_uiHigh) break;

			if (false == graph.areColliding(Coll_Pair(c_uiVtx, c_uiRobot, it->m_uiInd, uiOtherRobot))) continue;

			vec_taboo_regions.emplace_back(Interval());
			vec_taboo_regions[vec_taboo_regions.size() - 1].m_uiLow = (int)std::max(it->m_uiStart, (size_t)stHorizon.m_uiLow);
			vec_taboo_regions[vec_taboo_regions.size() - 1].m_uiHigh = (int)std::min(it->m_uiEnd, (size_t)stHorizon.m_uiHigh);
		}
	}

	mergeIntervals(vec_taboo_regions);
	
	if (0 != vec_taboo_regions.size())
	{
		assert(vec_taboo_regions.rbegin()->m_uiLow >= stHorizon.m_uiLow);
		assert(vec_taboo_regions.begin()->m_uiHigh <= stHorizon.m_uiHigh);
		assert(false == ((vec_taboo_regions[0].m_uiLow == stHorizon.m_uiLow) & (vec_taboo_regions[0].m_uiHigh == stHorizon.m_uiHigh)));
	}

	//merge intervals code outputs latest time window the earliest, so it is reversed temporally
	if (0 == vec_taboo_regions.size())
	{
		pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals = 1;
		pstCosts->m_parTimeWindows[uiHole].m_parIntervals = (struct Interval*)malloc(sizeof(struct Interval));
		pstCosts->m_parTimeWindows[uiHole].m_parIntervals[0].m_uiLow = stHorizon.m_uiLow;
		pstCosts->m_parTimeWindows[uiHole].m_parIntervals[0].m_uiHigh = stHorizon.m_uiHigh;
	}
	else
	{
		pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals = 0;
		uiNumIntervals = 0;
		if (stHorizon.m_uiLow < vec_taboo_regions[vec_taboo_regions.size()-1].m_uiLow) pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals++;
		pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals = pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals + (int)(vec_taboo_regions.size()) - 1;
		if (vec_taboo_regions.begin()->m_uiHigh < stHorizon.m_uiHigh) pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals++;
		
		pstCosts->m_parTimeWindows[uiHole].m_parIntervals = (Interval*)malloc(pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals * sizeof(Interval));

		if (stHorizon.m_uiLow < vec_taboo_regions[vec_taboo_regions.size()-1].m_uiLow)
		{
			pstCosts->m_parTimeWindows[uiHole].m_parIntervals[0].m_uiLow = stHorizon.m_uiLow;
			pstCosts->m_parTimeWindows[uiHole].m_parIntervals[0].m_uiHigh = vec_taboo_regions[vec_taboo_regions.size() - 1].m_uiLow;
			uiNumIntervals++;
		}

		for (auto it = vec_taboo_regions.rbegin(); it != vec_taboo_regions.rend(); it++)
		{
			auto it_next = it;
			it_next++;
			if (vec_taboo_regions.rend() == it_next) break;
			pstCosts->m_parTimeWindows[uiHole].m_parIntervals[uiNumIntervals].m_uiLow = it->m_uiHigh;
			pstCosts->m_parTimeWindows[uiHole].m_parIntervals[uiNumIntervals].m_uiHigh = it_next->m_uiLow;
			uiNumIntervals++;
		}

		if (vec_taboo_regions.begin()->m_uiHigh < stHorizon.m_uiHigh)
		{
			pstCosts->m_parTimeWindows[uiHole].m_parIntervals[uiNumIntervals].m_uiLow = vec_taboo_regions[0].m_uiHigh;
			pstCosts->m_parTimeWindows[uiHole].m_parIntervals[uiNumIntervals].m_uiHigh = stHorizon.m_uiHigh;
			uiNumIntervals++;
		}
		assert(uiNumIntervals == pstCosts->m_parTimeWindows[uiHole].m_uiNumIntervals);
	}	
}

void populate_enabler_info(const size_t c_uiVtx, const size_t c_uiRobot, const Greedy_Heuristic &heur, unsigned int uiHole, const Enabling_Graph &en_graph, const Layout_LS &graph, const std::unordered_map<size_t, size_t> &map_hole_new_ind, const std::unordered_set<size_t> &set_vts)
{
	const auto &vec_enablers = graph.get_Enablers();
	std::set<size_t> set_enablers;
	pstCosts->m_parTimeWindows[uiHole].m_uiOtherRobotEnableTime = inFinity;

	if (0 == vec_enablers[c_uiVtx].set.size())
	{
		pstCosts->m_parTimeWindows[uiHole].m_uiOtherRobotEnableTime = 0;
		pstCosts->m_parTimeWindows[uiHole].m_piEnablers = NULL;
		pstCosts->m_parTimeWindows[uiHole].m_uiNumEnablers = 0;
		return;
	}

	for (auto it = vec_enablers[c_uiVtx].set.begin(); it != vec_enablers[c_uiVtx].set.end(); it++)
	{
		auto pr = heur.get_robot_owner(it->getInd());
		assert(true == pr.first);
		if ( (c_uiRobot == pr.second) && (set_vts.end() != set_vts.find(it->getInd())))
		{
			set_enablers.emplace(map_hole_new_ind.at(it->getInd()));			
		}
		else
		{
			auto res = heur.get_vtx_completion_time(it->getInd());
			assert(true == res.first);
			pstCosts->m_parTimeWindows[uiHole].m_uiOtherRobotEnableTime = std::min(pstCosts->m_parTimeWindows[uiHole].m_uiOtherRobotEnableTime, (int)res.second);
		}
	}

	pstCosts->m_parTimeWindows[uiHole].m_uiNumEnablers = (unsigned int)set_enablers.size();
	if(set_enablers.size() > 0)pstCosts->m_parTimeWindows[uiHole].m_piEnablers = (int*)malloc(set_enablers.size() * sizeof(int));
	else pstCosts->m_parTimeWindows[uiHole].m_piEnablers = NULL;

	size_t uiEnablerCount = 0;
	for (auto it = set_enablers.begin(); it != set_enablers.end(); it++)
	{
		pstCosts->m_parTimeWindows[uiHole].m_piEnablers[uiEnablerCount] = (int)(*it);
		uiEnablerCount++;
	}
}

size_t populate_vtx_end_horizon(const size_t c_uiVtx, const size_t c_uiRobot, const Greedy_Heuristic &heur, unsigned int uiHole, const Enabling_Graph &en_graph, const Layout_LS &graph, const std::unordered_map<size_t, size_t> &map_hole_new_ind)
{
	size_t uiMaxTime = inFinity , uiEnabledVtx, uiEnabler, uiEnabledTime;
	const auto &vec_enablers = graph.get_Enablers();
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

		auto pr = heur.get_robot_owner(*it_enabled);
		assert(true == pr.first);
		if (c_uiRobot == pr.second) continue;

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
				uiMaxTime = std::min(uiMaxTime, uiEnabledVtxStartTime);
			}
		}
	}
	return uiMaxTime;
}

void free_cost_container()
{
	for (size_t uiVtx = 0; uiVtx < pstCosts->m_iNumVtx; uiVtx++)
	{
		free(pstCosts->m_parTimeWindows[uiVtx].m_parIntervals);
		pstCosts->m_parTimeWindows[uiVtx].m_parIntervals = NULL;
		
		free(pstCosts->m_parTimeWindows[uiVtx].m_piEnablers);		
		pstCosts->m_parTimeWindows[uiVtx].m_piEnablers = NULL;

		free(pstCosts->m_pparTravTime[uiVtx]);
		pstCosts->m_pparTravTime[uiVtx] = NULL;
	}
	
	free(pstCosts->m_parTimeWindows);
	pstCosts->m_parTimeWindows = NULL;

	free(pstCosts->m_parProcTime);
	pstCosts->m_parProcTime = NULL;

	free(pstCosts->m_pparTravTime);
	pstCosts->m_pparTravTime = NULL;

	free(pstCosts);
	pstCosts = NULL;
}

void free_Aux_Node_Info_container()
{
	for (size_t uiNodes = 0; uiNodes < uiTotalAuxNodes; uiNodes++)
	{
		free(pstAuxNodeInfo[uiNodes].m_Sminus);
		pstAuxNodeInfo[uiNodes].m_Sminus = NULL;

		free(pstAuxNodeInfo[uiNodes].m_Splus);
		pstAuxNodeInfo[uiNodes].m_Splus = NULL;
	}

	free(pstAuxNodeInfo);
	pstAuxNodeInfo = NULL;
}

void select_sub_seq_for_opt(const std::vector<std::vector<Vertex_Schedule>> &vec_rob_sch, std::tuple<size_t, size_t, size_t> &rob_start_end, const Layout_LS &graph)
{
	size_t ui_Bottleneck_robot , uiMaxMakespan = std::numeric_limits<size_t>::min();
	for (size_t uiRobot = 0; uiRobot < vec_rob_sch.size(); uiRobot++)
	{
		if (vec_rob_sch[uiRobot].rbegin()->m_uiEnd > uiMaxMakespan)
		{
			uiMaxMakespan = vec_rob_sch[uiRobot].rbegin()->m_uiEnd;
			ui_Bottleneck_robot = uiRobot;
		}
	}

	std::pair<size_t, size_t> pr_st_end;
	size_t uiMaxLength = std::numeric_limits<size_t>::min(), uiStart, uiEnd;
	bool bStart = false;

	for (size_t uiInd = 0; uiInd < vec_rob_sch[ui_Bottleneck_robot].size(); uiInd++)
	{
		if (false == bStart)
		{
			if (0 == vec_rob_sch[ui_Bottleneck_robot][uiInd].m_uiWait)
			{
				uiStart = uiInd;
				bStart = true;
			}
		}
		else if (bStart == true)
		{
			if (0 < vec_rob_sch[ui_Bottleneck_robot][uiInd].m_uiWait)
			{
				uiEnd = uiInd - 1;
				if (uiEnd - uiStart > uiMaxLength)
				{
					uiMaxLength = uiEnd - uiStart + 1;
					pr_st_end.first = uiStart;
					pr_st_end.second = uiEnd;
				}
				bStart = false;
			}					
		}
	}

	//adjust start and end so they begin and end at hole locations
	while ("IV" == graph.getType(vec_rob_sch[ui_Bottleneck_robot][pr_st_end.first].m_uiInd)) pr_st_end.first++;
	while ("IV" == graph.getType(vec_rob_sch[ui_Bottleneck_robot][pr_st_end.second].m_uiInd)) pr_st_end.second--;
	uiMaxLength = pr_st_end.second - pr_st_end.first + 1;

	if (0 == pr_st_end.second - pr_st_end.first)
	{
		pr_st_end.first = 0;
		pr_st_end.second = vec_rob_sch[ui_Bottleneck_robot].size() - 1;
	}

	std::get<0>(rob_start_end) = ui_Bottleneck_robot;
	std::get<1>(rob_start_end) = pr_st_end.first;
	std::get<2>(rob_start_end) = pr_st_end.second;	
}

//need to change this
void generate_new_tour(const size_t c_uiRobot, const size_t c_uiStartInd, const size_t c_uiEndInd, std::list<size_t> &old_tour, int* new_tour, const size_t c_uiTour_Len, const Layout_LS &graph, const std::unordered_map<size_t, size_t> &map_hole_new_ind)
{
	size_t uiOldTourSize = old_tour.size(), uiNewTourSize;
	std::unordered_map<size_t, size_t> map_hole_old_ind;
	for (auto it = map_hole_new_ind.begin(); it != map_hole_new_ind.end(); it++) map_hole_old_ind.emplace(it->second, it->first);

	auto it_start = old_tour.begin();
	std::advance(it_start, c_uiStartInd);

	auto it_end = it_start;
	std::advance(it_end, c_uiEndInd - c_uiStartInd);

	assert(*it_start == map_hole_old_ind.at(new_tour[0]));
	assert(*it_end == map_hole_old_ind.at(new_tour[c_uiTour_Len-1]));

	auto it_erase = it_start;
	while (it_erase != it_end) it_erase = old_tour.erase(it_erase);
	it_erase = old_tour.erase(it_erase); //after this step, it_erase points to next element after the tour ends
	auto it_insert = it_erase;
	
	const auto &vec_rob_iv = graph.get_IV_Vec();
	
	for (size_t uiCount = 0; uiCount < c_uiTour_Len - 1; uiCount++)
	{
		old_tour.insert(it_insert, map_hole_old_ind.at(new_tour[uiCount]));
		const auto &vec_iv = vec_rob_iv[c_uiRobot].map.at(map_hole_old_ind.at(new_tour[uiCount])).map.at(map_hole_old_ind.at(new_tour[uiCount+1])).vec;
		for (auto it_iv = vec_iv.cbegin(); it_iv != vec_iv.cend(); it_iv++)
		{
			old_tour.insert(it_insert, it_iv->getInd());
		}
	}
	old_tour.insert(it_insert, map_hole_old_ind.at(new_tour[c_uiTour_Len-1]));
	uiNewTourSize = old_tour.size();
	assert(uiOldTourSize == uiNewTourSize);
}

void free_TSP_buffers()
{
	free_Aux_Node_Info_container();
	free_buffers();
}