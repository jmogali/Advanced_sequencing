#include "Local_Search.h"

struct String_Info
{
	const size_t m_uiPos;
	const size_t m_uiLen;
	const double m_dAvgDist;
	String_Info(size_t uiPos, size_t uiLen, double dAvgDist) : m_uiPos(uiPos), m_uiLen(uiLen), m_dAvgDist(dAvgDist) {};
};

size_t get_best_position_to_insert( size_t uiVtx1, size_t uiVtx2, std::list<size_t> &r, std::mt19937 &rng, size_t uiRobot, const Layout_LS& graph)
{
	std::vector<size_t> vec_pos;
	std::vector<size_t> vec_CDF;

	size_t uiPos = 0 , uiCDF = 0;
	auto it_next = r.begin();
	it_next++;
	for (auto it = r.begin(); it != r.end(); it++, uiPos++)
	{
		if (it_next == r.end()) break;
		if (true == graph.doesEdgeExist(uiRobot, *it, uiVtx1))
		{
			if (true == graph.doesEdgeExist(uiRobot, uiVtx2, *it_next))
			{
				if (true == graph.doesEdgeExist(uiRobot, *it, *it_next))
				{
					int iDist = (int)(graph.getEdgeDist(uiRobot, *it, uiVtx1) + graph.getEdgeDist(uiRobot, uiVtx2, *it_next));
					iDist = iDist - (int)graph.getEdgeDist(uiRobot, *it, *it_next);

					uiCDF += std::max((size_t)(100 * exp(-1.0 * iDist)), (size_t)1);
					vec_CDF.emplace_back(uiCDF);
					vec_pos.emplace_back(uiPos + 1);
				}
			}
		}
		it_next++;
	}
	
	if (0 == vec_pos.size()) return std::numeric_limits<size_t>::max();
	else return vec_pos[generate_rand_ind_from_cdf(uiCDF, rng, vec_CDF)];
}

//uiPos1 points to the first location that needs to be removed in r1, uiLen1 is the length to be removed
bool string_Exchange(std::list<size_t> &r1, const std::pair<size_t, size_t> &pr1, size_t uiRobot1, std::list<size_t> &r2, const std::pair<size_t, size_t> &pr2, size_t uiRobot2, std::mt19937 &rng, const Layout_LS& graph)
{
	size_t uiPos1 = pr1.first, uiLen1 = pr1.second;
	size_t uiPos2 = pr2.first, uiLen2 = pr2.second;

	assert((uiLen1 != 0) && (uiLen2 != 0));
	if (uiPos1 + uiLen1 >= r1.size()) return false;
	if (uiPos2 + uiLen2 >= r2.size()) return false;

	auto it1 = r1.begin();
	std::advance(it1, uiPos1);

	auto it2 = r2.begin();
	std::advance(it2, uiPos2);

	if( (1 == uiPos1) || (1 == uiPos2) )
	{
		if(1 == uiPos1)
			if (false == graph.doesEdgeExist(uiRobot1, graph.getDepotMap().at(uiRobot1).getFromInd(), *it2)) return false;
		
		if(1 == uiPos2)
			if (false == graph.doesEdgeExist(uiRobot2, graph.getDepotMap().at(uiRobot2).getFromInd(), *it1)) return false;
	}

	auto it_copy1 = it1;
	std::vector<size_t> vec1;
	for (size_t uiCount = 0; uiCount < uiLen1; uiCount++)
	{
		vec1.push_back(*it_copy1);
		it_copy1++;
	}
	r1.erase(it1, it_copy1);
	
	auto it_copy2 = it2;
	std::vector<size_t> vec2;
	for (size_t uiCount = 0; uiCount < uiLen2; uiCount++)
	{
		vec2.push_back(*it_copy2);
		it_copy2++;
	}
	r2.erase(it2, it_copy2);

	//insert removed sub-strings in position where euclidean distance is minimum
	size_t uiInsertPos1;
	
	if (1 == uiPos1) uiInsertPos1 = 1;
	else uiInsertPos1 = get_best_position_to_insert(vec2[0], vec2[vec2.size() - 1], r1, rng, uiRobot1, graph);

	auto it_insert = r1.begin();
	std::advance(it_insert, uiInsertPos1);
	r1.insert(it_insert, vec2.begin(), vec2.end());

	size_t uiInsertPos2;
	if (1 == uiPos2) uiInsertPos2 = 1;
	else uiInsertPos2 = get_best_position_to_insert(vec1[0], vec1[vec1.size() - 1], r2, rng, uiRobot2, graph);

	it_insert = r2.begin();
	std::advance(it_insert, uiInsertPos2);
	r2.insert(it_insert, vec1.begin(), vec1.end());

	return true;
}

// vertex denoted by it1 will be added, vertex denoted by it1_end will not be added, code will add vertices into r2, before uiPos2, vertex denoted 
bool string_relocate(std::list<size_t> &r1, const std::pair<size_t, size_t> &pr1, size_t uiRobot1, std::list<size_t> &r2, size_t uiPos2, size_t uiRobot2, const Layout_LS& graph)
{
	size_t uiPos1 = pr1.first, uiLen1 = pr1.second;
	
	if (uiPos1 + uiLen1 >= r1.size()) return false;
	
	//denotes first vertex to be moved from r1
	auto it1 = r1.begin();
	std::advance(it1, uiPos1);

	//*it1_last denotes last vertext to be removed
	auto it1_last = it1;
	std::advance(it1_last, uiLen1 - 1);
	
	//*it1_end denotes end iterator
	auto it1_end = it1_last;
	std::advance(it1_end, 1);

	//*it2_start is the vertex after which the the sequence is added
	auto it2_start = r2.begin();
	std::advance(it2_start, uiPos2-1);

	//*it2 is the vertex before which it should be added
	auto it2 = it2_start;
	std::advance(it2, 1);

	if (1 == uiPos1)
	{
		if (false == graph.doesEdgeExist(uiRobot1, graph.getDepotMap().at(uiRobot1).getFromInd(), *it1_end))
			return false;
	}
	if (1 == uiPos2)
	{
		if (false == graph.doesEdgeExist(uiRobot2, graph.getDepotMap().at(uiRobot2).getFromInd(), *it1)) 
			return false;
	}	

	if(false == graph.doesEdgeExist(uiRobot2, *it2_start, *it1)) return false;
	if (false == graph.doesEdgeExist(uiRobot2, *it1_last, *it2)) return false;

	r2.splice(it2 , r1, it1, it1_end);
	return true;
}

size_t getMaxLen(const std::vector<String_Info> &vec_pos_len)
{
	size_t uiMaxLen = std::numeric_limits<size_t>::min();

	for (size_t uiCount = 0; uiCount < vec_pos_len.size(); uiCount++)
	{
		uiMaxLen = std::max(uiMaxLen, vec_pos_len[uiCount].m_uiLen);
	}

	return uiMaxLen;
}

size_t generate_bounded_sub_string_length(size_t uiMaxLen, std::mt19937 &rng, size_t uiLSMaxExchange)
{
	size_t uiLen;
	if (uiMaxLen >= uiLSMaxExchange)
	{
		std::uniform_int_distribution<size_t> unif_len(1, uiLSMaxExchange);
		uiLen = unif_len(rng);
	}
	else
	{
		std::uniform_int_distribution<size_t> unif_len(1, uiMaxLen);
		uiLen = unif_len(rng);
	}
	return uiLen;
}

size_t generate_rand_ind_from_cdf(const size_t uiNormFactor, std::mt19937 &rng, const std::vector<size_t> &vec_CDF)
{
	std::uniform_real_distribution<> unif_pdf(0, (double)uiNormFactor);
	double dRandVal = unif_pdf(rng);
	size_t uiPrev = 0, uiCurr, uiInd = std::numeric_limits<size_t>::max();

	for (size_t uiCount = 0; uiCount < vec_CDF.size(); uiCount++)
	{
		uiCurr = vec_CDF[uiCount];
		if (uiPrev == uiCurr) continue;
		if ((uiPrev <= dRandVal) && (dRandVal <= uiCurr))
		{
			uiInd = uiCount;
			break;
		}
		uiPrev = uiCurr;
	}

	assert(uiInd != std::numeric_limits<size_t>::max());
	return uiInd;
}

size_t compute_and_generate_rand_ind(std::mt19937 &rng, const size_t c_uiLen, const std::vector<String_Info> &vec_pos_len)
{
	size_t uiCDF = 0;
	std::vector<size_t> vec_CDF;

	//compute CDF
	for (size_t uiCount = 0; uiCount < vec_pos_len.size(); uiCount++)
	{
		if (vec_pos_len[uiCount].m_uiLen >= c_uiLen)
		{
			uiCDF += (size_t)(100 * exp(-.01 * vec_pos_len[uiCount].m_dAvgDist));
		}
		vec_CDF.push_back(uiCDF);
	}

	return generate_rand_ind_from_cdf(uiCDF , rng, vec_CDF);
}

//<position, length>
std::pair<size_t, size_t> get_valid_random_sub_string_for_exchange(size_t uiRobot, const std::vector<String_Info> &vec_pos_len, std::mt19937 &rng, size_t uiLSMaxExchange)
{
	size_t uiMaxLen, uiInd;
	
	uiMaxLen = getMaxLen(vec_pos_len);
	assert(std::numeric_limits<size_t>::min() != uiMaxLen);
	const size_t c_uiLen = generate_bounded_sub_string_length(uiMaxLen , rng, uiLSMaxExchange);	

	uiInd = compute_and_generate_rand_ind(rng, c_uiLen, vec_pos_len);
	assert(vec_pos_len[uiInd].m_uiLen >= c_uiLen);
	
	size_t uiDisp;
	if (vec_pos_len[uiInd].m_uiLen == c_uiLen) uiDisp = 0;
	else uiDisp = rand() % (vec_pos_len[uiInd].m_uiLen - c_uiLen + 1);
		
	return std::make_pair(vec_pos_len[uiInd].m_uiPos + uiDisp, c_uiLen);
}

void get_common_nodes_in_seq(size_t uiRobot, size_t uiOtherRobot, const std::vector<std::list<size_t>> &rob_seq, const Node_Partitions& node_data, std::vector<String_Info> &vec_pos_len, const Layout_LS &graph)
{
	const auto &comm_nodes = node_data.get_common_nodes(uiRobot, uiOtherRobot);
	bool bBegin = false;
	size_t uiStart, uiInd = 0;
	double dAvgDist;
	const auto& vec_depo = graph.getDepotMap();
	const auto& stOtherRobotDepoLoc = vec_depo.at(uiOtherRobot).getLoc();

	for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++, uiInd++)
	{
		if (comm_nodes.find(*it) != comm_nodes.end())
		{
			if (false == bBegin)
			{
				uiStart = uiInd;
				bBegin = true;
				dAvgDist = 0;
			}			
		}
		else
		{
			if (true == bBegin)
			{
				vec_pos_len.emplace_back(String_Info(uiStart , uiInd - uiStart, (dAvgDist/(double)(uiInd - uiStart))));
				bBegin = false;
			}
		}

		if(true == bBegin) dAvgDist += stOtherRobotDepoLoc.getDist_XY(graph.getLoc(*it));
	}
}

bool Local_Search::string_exchange(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq)
{
	bool bValid;
	std::vector<String_Info> vec_pos_len_1;   // first is index, second is length
	std::vector<String_Info> vec_pos_len_2;
	
	get_common_nodes_in_seq(uiRobot1, uiRobot2, rob_seq, m_node_data, vec_pos_len_1, m_graph);
	get_common_nodes_in_seq(uiRobot2, uiRobot1, rob_seq, m_node_data, vec_pos_len_2, m_graph);

	if ((vec_pos_len_1.size() == 0) || (vec_pos_len_2.size() == 0)) return false;

	auto pr1 = get_valid_random_sub_string_for_exchange(uiRobot1, vec_pos_len_1, m_rng, c_uiMax_SE_Length);
	auto pr2 = get_valid_random_sub_string_for_exchange(uiRobot2, vec_pos_len_2, m_rng, c_uiMax_SE_Length);

	bValid = string_Exchange(rob_seq[uiRobot1], pr1, uiRobot1, rob_seq[uiRobot2], pr2, uiRobot2, m_rng ,m_graph);
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


bool Local_Search::string_relocation(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq)
{
	std::vector<String_Info> vec_pos_len_1;   // first is index, second is length
	std::vector<String_Info> vec_pos_len_2;
	size_t uiRobot , uiPosOther;
	bool bValid;

	get_common_nodes_in_seq(uiRobot1, uiRobot2, rob_seq, m_node_data, vec_pos_len_1, m_graph);
	get_common_nodes_in_seq(uiRobot2, uiRobot1, rob_seq, m_node_data, vec_pos_len_2, m_graph);

	if ((vec_pos_len_1.size() == 0) && (vec_pos_len_2.size() == 0)) return false;
	if (vec_pos_len_1.size() == 0) { uiRobot = uiRobot2; }
	else if (vec_pos_len_2.size() == 0) { uiRobot = uiRobot1; }
	else{ uiRobot = rand() % 2 == 0 ? uiRobot1 : uiRobot2; }
	
	if (uiRobot == uiRobot1)
	{
		auto pr = get_valid_random_sub_string_for_exchange(uiRobot1, vec_pos_len_1, m_rng, c_uiMax_SE_Length);
		auto it_vtx1 = rob_seq[uiRobot1].begin();
		std::advance(it_vtx1, pr.first);
		auto it_vtx2 = it_vtx1;
		std::advance(it_vtx2, pr.second-1);
		uiPosOther = get_best_position_to_insert(*it_vtx1, *it_vtx2, rob_seq[uiRobot2], m_rng, uiRobot2, m_graph);
		bValid = string_relocate(rob_seq[uiRobot1], pr, uiRobot1, rob_seq[uiRobot2], uiPosOther, uiRobot2, m_graph);
	}
	else
	{
		auto pr = get_valid_random_sub_string_for_exchange(uiRobot2, vec_pos_len_2, m_rng, c_uiMax_SE_Length);
		auto it_vtx1 = rob_seq[uiRobot2].begin();
		std::advance(it_vtx1, pr.first);
		auto it_vtx2 = it_vtx1;
		std::advance(it_vtx2, pr.second-1);
		uiPosOther = get_best_position_to_insert(*it_vtx1, *it_vtx2, rob_seq[uiRobot1], m_rng, uiRobot1, m_graph);
		bValid = string_relocate(rob_seq[uiRobot2], pr, uiRobot2,  rob_seq[uiRobot1], uiPosOther, uiRobot1, m_graph);
	}	

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


