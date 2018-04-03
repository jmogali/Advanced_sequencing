#include "Hole_exchanges.h"

size_t get_first_occurence_index(size_t uiVtx, size_t uiRobot, const std::vector<State_pos> &vec_state_path)
{
	for (size_t uiIndex = 0; uiIndex < vec_state_path.size(); uiIndex++)
	{
		if (uiVtx == vec_state_path[uiIndex].m_vec_rob_vtx[uiRobot]) return uiIndex;
	}
	return std::numeric_limits<size_t>::max();
}

void get_new_left_right_hole_state_offset(size_t &uiLeftOffset, size_t &uiRightOffset, const size_t c_uiCenter, const std::vector<State_pos> &vec_state_path, const Layout_LS &graph)
{
	size_t uiLeft, uiRight;

	for (uiLeft = (c_uiCenter - uiLeftOffset) - 1 ; uiLeft >=0; uiLeft--)
	{
		auto it_begin = vec_state_path[uiLeft].m_vec_rob_vtx.begin();
		auto it_end = vec_state_path[uiLeft].m_vec_rob_vtx.end();
		bool bFound = true;

		for (auto it = it_begin; it != it_end; it++)
		{
			if ("IV" == graph.getType(*it))
			{
				bFound = false;
				break;
			}
		}
		if (bFound) break;
	}

	for (uiRight = (c_uiCenter + uiRightOffset) + 1; uiRight < vec_state_path.size(); uiRight++)
	{
		auto it_begin = vec_state_path[uiRight].m_vec_rob_vtx.begin();
		auto it_end = vec_state_path[uiRight].m_vec_rob_vtx.end();
		bool bFound = true;

		for (auto it = it_begin; it != it_end; it++)
		{
			if ("IV" == graph.getType(*it))
			{
				bFound = false;
				break;
			}
		}
		if (bFound) break;
	}

	uiLeftOffset = c_uiCenter - uiLeft;
	uiRightOffset = uiRight - c_uiCenter;
}

bool Hole_Exchange::check_if_retraction_feasible(size_t uiVtx, size_t uiRobot, const std::vector<State_pos> &vec_state_path, const std::vector<std::list<size_t>> &inp_seq)
{
	bool bFeasible = false;
	size_t uiLeftOffset = 1, uiRightOffset = 1 , uiIter = 0;
	size_t uiIndex = get_first_occurence_index(uiVtx, uiRobot, vec_state_path);
	std::vector<std::pair<std::list<size_t>::const_iterator, std::list<size_t>::const_iterator>> vec_start_end_itr;
	std::vector<std::list<size_t>> rob_sub_seq;
	std::unordered_set<size_t> set_comp_verts;

	while (!bFeasible)
	{
		get_new_left_right_hole_state_offset(uiLeftOffset, uiRightOffset, uiIndex, vec_state_path, m_graph);
		
		if (0 == uiIter)
		{
			construct_rob_sub_sequences(rob_sub_seq, uiIndex - uiLeftOffset, uiIndex + uiRightOffset, inp_seq, vec_state_path, vec_start_end_itr, set_comp_verts);
			uiIter++;
		}		
		else
		{

		}
		

		if (uiRightOffset + uiLeftOffset > 10) break;
	}
	return bFeasible;
}