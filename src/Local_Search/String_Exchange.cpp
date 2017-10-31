#include "Local_Search.h"

//uiPos1 points to the first location that needs to be removed in r1, uiLen1 is the length to be removed
bool string_Exchange(std::list<size_t> &r1, const std::pair<size_t, size_t> &pr1, size_t uiRobot1, std::list<size_t> &r2, const std::pair<size_t, size_t> &pr2, size_t uiRobot2, const Layout_LS& graph)
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

	for (size_t uiCount = 0; uiCount < uiLen2; uiCount++)
	{
		r1.insert(it_copy1, vec2[uiCount]);
	}

	for (size_t uiCount = 0; uiCount < uiLen1; uiCount++)
	{
		r2.insert(it_copy2, vec1[uiCount]);
	}
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

size_t getMaxLen(const std::vector<std::pair<size_t, size_t>> &vec_pos_len)
{
	size_t uiMaxLen = std::numeric_limits<size_t>::min();

	for (size_t uiCount = 0; uiCount < vec_pos_len.size(); uiCount++)
	{
		uiMaxLen = std::max(uiMaxLen, vec_pos_len[uiCount].second);
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
	std::uniform_int_distribution<size_t> unif_pdf(0, uiNormFactor);
	size_t uiRandVal = unif_pdf(rng);
	size_t uiPrev = 0, uiCurr, uiInd = -1;

	for (size_t uiCount = 0; uiCount < vec_CDF.size(); uiCount++)
	{
		uiCurr = vec_CDF[uiCount];
		if (uiPrev == uiCurr) continue;
		if ((uiPrev <= uiRandVal) && (uiRandVal <= uiCurr))
		{
			uiInd = uiCount;
			break;
		}
		uiPrev = uiCurr;
	}

	assert(uiInd != -1);
	return uiInd;
}

size_t compute_and_generate_rand_ind(std::mt19937 &rng, size_t uiLen, const std::vector<std::pair<size_t, size_t>> &vec_pos_len)
{
	size_t uiCDF = 0;
	std::vector<size_t> vec_CDF;

	//compute CDF
	for (size_t uiCount = 0; uiCount < vec_pos_len.size(); uiCount++)
	{
		uiCDF += std::max(vec_pos_len[uiCount].second - uiLen + 1, (size_t)0);
		vec_CDF.push_back(uiCDF);
	}

	return generate_rand_ind_from_cdf(uiCDF , rng, vec_CDF);
}

std::pair<size_t, size_t> get_valid_random_sub_string_for_exchange(size_t uiRobot, const std::vector<std::pair<size_t, size_t>> &vec_pos_len, std::mt19937 &rng, size_t uiLSMaxExchange)
{
	size_t uiMaxLen, uiLen, uiInd;
	
	uiMaxLen = getMaxLen(vec_pos_len);
	assert(std::numeric_limits<size_t>::min() != uiMaxLen);
	uiLen = generate_bounded_sub_string_length(uiMaxLen , rng, uiLSMaxExchange);	

	uiInd = compute_and_generate_rand_ind(rng, uiLen, vec_pos_len);
	assert(vec_pos_len[uiInd].second >= uiLen);
	size_t uiDisp = rand() % (vec_pos_len[uiInd].second - uiLen + 1);
		
	return std::make_pair(vec_pos_len[uiInd].first + uiDisp, uiLen);
}

void get_common_nodes_in_seq(size_t uiRobot, size_t uiOtherRobot, std::vector<std::list<size_t>> &rob_seq, const Node_Partitions& node_data, std::vector<std::pair<size_t, size_t>> &vec_pos_len)
{
	const auto &comm_nodes = node_data.get_common_nodes(uiRobot, uiOtherRobot);
	bool bBegin = false;
	size_t uiStart, uiInd = 0;;
	
	for (auto it = rob_seq[uiRobot].begin(); it != rob_seq[uiRobot].end(); it++, uiInd++)
	{
		if (comm_nodes.find(*it) != comm_nodes.end())
		{
			if (false == bBegin)
			{
				uiStart = uiInd;
				bBegin = true;
			}			
		}
		else
		{
			if (true == bBegin)
			{
				vec_pos_len.emplace_back(uiStart , uiInd - uiStart);
				bBegin = false;
			}
		}
	}
}

bool Local_Search::string_exchange(size_t uiRobot1, size_t uiRobot2, std::vector<std::list<size_t>> &rob_seq)
{
	bool bValid;
	std::vector<std::pair<size_t, size_t>> vec_pos_len_1;   // first is index, second is length
	std::vector<std::pair<size_t, size_t>> vec_pos_len_2;
	
	get_common_nodes_in_seq(uiRobot1, uiRobot2, rob_seq, m_node_data, vec_pos_len_1);
	get_common_nodes_in_seq(uiRobot2, uiRobot1, rob_seq, m_node_data, vec_pos_len_2);

	if ((vec_pos_len_1.size() == 0) || (vec_pos_len_2.size() == 0)) return false;

	auto pr1 = get_valid_random_sub_string_for_exchange(uiRobot1, vec_pos_len_1, m_rng, c_uiMax_SE_Length);
	auto pr2 = get_valid_random_sub_string_for_exchange(uiRobot2, vec_pos_len_2, m_rng, c_uiMax_SE_Length);

	bValid = string_Exchange(rob_seq[uiRobot1], pr1, uiRobot1, rob_seq[uiRobot2], pr2, uiRobot2, m_graph);
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
	std::vector<std::pair<size_t, size_t>> vec_pos_len_1;   // first is index, second is length
	std::vector<std::pair<size_t, size_t>> vec_pos_len_2;
	size_t uiRobot , uiPosOther;
	bool bValid;

	get_common_nodes_in_seq(uiRobot1, uiRobot2, rob_seq, m_node_data, vec_pos_len_1);
	get_common_nodes_in_seq(uiRobot2, uiRobot1, rob_seq, m_node_data, vec_pos_len_2);

	if ((vec_pos_len_1.size() == 0) && (vec_pos_len_2.size() == 0)) return false;
	if (vec_pos_len_1.size() == 0) { uiRobot = uiRobot2; }
	else if (vec_pos_len_2.size() == 0) { uiRobot = uiRobot1; }
	else{ uiRobot = rand() % 2 == 0 ? uiRobot1 : uiRobot2; }
	
	if (uiRobot == uiRobot1)
	{
		auto pr = get_valid_random_sub_string_for_exchange(uiRobot1, vec_pos_len_1, m_rng, c_uiMax_SE_Length);
		std::uniform_int_distribution<size_t> unif_pos(1, rob_seq[uiRobot2].size() - 2);
		uiPosOther = unif_pos(m_rng);
		bValid = string_relocate(rob_seq[uiRobot1], pr, uiRobot1, rob_seq[uiRobot2], uiPosOther, uiRobot2, m_graph);
	}
	else
	{
		auto pr = get_valid_random_sub_string_for_exchange(uiRobot2, vec_pos_len_2, m_rng, c_uiMax_SE_Length);
		std::uniform_int_distribution<size_t> unif_pos(1, rob_seq[uiRobot1].size() - 2);
		uiPosOther = unif_pos(m_rng);
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


