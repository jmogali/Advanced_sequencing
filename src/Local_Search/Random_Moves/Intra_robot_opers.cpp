#include "Local_Search.h"
#include "Geometry_Utils.h"

std::pair<size_t, size_t> generate_valid_pos_len_pair(std::mt19937 &rng, const std::list<size_t> &seq)
{
	size_t uiPos, uiMaxLen, uiLen;
	uiPos = 1 + rand() % (seq.size() - 2);
	uiMaxLen = seq.size() - 1 - uiPos;
	assert(uiMaxLen != 0);

	if (c_uiMax_SE_Length > uiMaxLen)
	{
		std::uniform_int_distribution<size_t> unif_len(1, uiMaxLen);
		uiLen = unif_len(rng);
	}
	else
	{
		std::uniform_int_distribution<size_t> unif_len(1, c_uiMax_SE_Length);
		uiLen = unif_len(rng);
	}

	return std::make_pair(uiPos, uiLen);
}

//<pos, len> pair
inline bool check_validity_for_intra_swap(const std::pair<size_t, size_t> &pr1, const std::pair<size_t, size_t> &pr2)
{
	return pr1.first + pr1.second < pr2.first ? true : false;	 
}

std::vector<std::pair<size_t, size_t>> get_valid_pos_length_pair2(std::mt19937 &rng, const std::list<size_t> &seq)
{
	std::vector<std::pair<size_t, size_t>> vec_pos_len;
	bool bValid = false;

	while (1)
	{
		auto pr1 = generate_valid_pos_len_pair(rng, seq);
		auto pr2 = generate_valid_pos_len_pair(rng, seq);

		if (pr1.first == pr2.first) { continue; }
		else if (pr1.first > pr2.first) { std::swap(pr1, pr2); };

		bValid = check_validity_for_intra_swap(pr1, pr2);

		if (false == bValid) continue;

		vec_pos_len.emplace_back(pr1.first, pr1.second);
		vec_pos_len.emplace_back(pr2.first, pr2.second);
		break;
	}

	return vec_pos_len;
}

bool swap_Intra_sequence(size_t uiPos1, size_t uiLen1, size_t uiPos2, size_t uiLen2, std::list<size_t> &seq, size_t uiRobot, const Layout_LS &graph)
{
	assert(uiPos1 < uiPos2);
	assert(uiPos2 + uiLen2 <= seq.size() - 1);

	auto it1 = seq.begin();
	std::advance(it1, uiPos1);
	auto it2 = it1;
	std::advance(it2, uiPos2 - uiPos1);

	if ( (1 == uiPos1) && (false == graph.doesEdgeExist(uiRobot, *seq.begin(), *it2)) )  return false;
	
	auto it1_end = it1;
	std::advance(it1_end, uiLen1);
	auto it2_end = it2;
	std::advance(it2_end, uiLen2);

	seq.splice(it1, seq, it2, it2_end);
	if (uiPos1 + uiLen1 == uiPos2) return true;
	seq.splice(it2_end, seq, it1, it1_end);
	return true;
}

bool Local_Search::swap_intra_sequence(size_t uiRobot, std::vector<std::list<size_t>> &rob_seq)
{
	bool bValid;
	std::list<size_t>& seq = rob_seq[uiRobot];
	size_t uiPos1, uiPos2;
	size_t uiLen1, uiLen2;
	
	if (seq.size() <= 3) return false;

	auto vec_pairs = get_valid_pos_length_pair2(m_rng , seq);
	uiPos1 = vec_pairs[0].first;
	uiLen1 = vec_pairs[0].second;
	uiPos2 = vec_pairs[1].first;
	uiLen2 = vec_pairs[1].second;

	bValid = swap_Intra_sequence(uiPos1, uiLen1, uiPos2, uiLen2, seq, uiRobot, m_graph);	

	/*if (bValid)
	{
		bValid = check_validity_of_sequence(rob_seq);
		if (false == bValid)
		{
			return false;
		};
	}*/
	return bValid;
}

bool doIntersect(const Layout_LS &graph, std::list<size_t>::const_iterator it_11, std::list<size_t>::const_iterator it_12, std::list<size_t>::const_iterator it_21, std::list<size_t>::const_iterator it_22)
{
	Coordinates loc11 = graph.getLoc(*it_11);
	Coordinates loc12 = graph.getLoc(*it_12);
	Coordinates loc21 = graph.getLoc(*it_21);
	Coordinates loc22 = graph.getLoc(*it_22);

	return doIntersect(loc11, loc12, loc21, loc22);
}

bool Check_if_sub_sequence_enabled(const std::set<size_t> &set_comp_verts_before_seg , std::list<size_t>::const_iterator it_start, std::list<size_t>::const_iterator it_end, const Layout_LS &graph)
{
	const auto &vec_enablers = graph.get_Enablers();
	size_t uiHole;
	std::set<size_t> set_reversed_segment_verts;

	for (auto it = it_end; it != it_start; it--)
	{
		uiHole = *it;
		for (auto it_enabler = vec_enablers[uiHole].set.begin(); it_enabler != vec_enablers[uiHole].set.end(); it_enabler++)
		{
			if (set_comp_verts_before_seg.end() != set_comp_verts_before_seg.find(it_enabler->getInd())) continue;
			if (set_reversed_segment_verts.end() != set_reversed_segment_verts.find(it_enabler->getInd())) continue;
			else return false;
		}
		set_reversed_segment_verts.emplace(*it);
	}
	return true;
}

bool Local_Search::Two_opt_intra_sequence(const size_t c_uiRobot, std::vector<std::list<size_t>> &rob_seq)
{
	std::list<size_t> &seq = rob_seq[c_uiRobot];
	if (seq.size() <= 3) return false;
	int iDist;
	std::set<size_t> set_comp_verts_before_seg;

	for (auto it1 = seq.begin(); it1 != seq.end(); it1++)
	{
		set_comp_verts_before_seg.emplace(*it1);

		auto it_next_1 = it1;
		it_next_1++;		
		if (*it_next_1 == m_node_data.m_rob_depo[c_uiRobot].second) { break; }
		
		auto it2 = it_next_1;
		it2++;                 // we increment this so that the common point of connecting the line segments *it1, *it_next_1 = *it2 , *it_next_2 does not show up as intersection
		
		for (; it2 != seq.end(); it2++)
		{
			if (*it2 == m_node_data.m_rob_depo[c_uiRobot].second) { break; }
			auto it_next_2 = it2;
			it_next_2++;
			
			if (false == m_graph.doesEdgeExist(c_uiRobot, *it1, *it2)) continue;
			if (false == m_graph.doesEdgeExist(c_uiRobot, *it_next_1, *it_next_2)) continue;
				
			iDist = (int)(m_graph.getEdgeDist(c_uiRobot, *it1, *it_next_1) + m_graph.getEdgeDist(c_uiRobot, *it2, *it_next_2));
			iDist = iDist - (int)(m_graph.getEdgeDist(c_uiRobot, *it1, *it2) + m_graph.getEdgeDist(c_uiRobot, *it_next_1, *it_next_2));
			
			if (iDist > 0) continue;
			else
			{
				if (false == Check_if_sub_sequence_enabled(set_comp_verts_before_seg, it1, it2, m_graph)) continue;

				std::reverse(it_next_1 , it_next_2);
				return true;				
			}
		}
	}
	return false;
}


