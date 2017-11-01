#include "Local_Search.h"
#include <numeric>

void print_schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	for (size_t uiRobot = 0; uiRobot < full_rob_sch.size(); uiRobot++)
	{
		cout << "Robot: " << uiRobot << endl;
		for (auto it = full_rob_sch[uiRobot].begin(); it != full_rob_sch[uiRobot].end(); it++)
		{
			it->print_schedule();
			cout << "\n";
		}
		cout << "\n\n";
	}
}

size_t getMakeSpan_From_Schedule(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	size_t uiMakeSpan = std::numeric_limits<size_t>::min();
	size_t uiRobot_MakeSpan;

	for (size_t uiRobot = 0; uiRobot < full_rob_sch.size(); uiRobot++)
	{
		uiRobot_MakeSpan = full_rob_sch[uiRobot][full_rob_sch[uiRobot].size() - 1].m_uiEnd;
		if (uiMakeSpan < uiRobot_MakeSpan) uiMakeSpan = uiRobot_MakeSpan;		
	}
	return uiMakeSpan;
}

Local_Search::Local_Search(const Node_Partitions &node_data, const Layout_LS &graph) :m_rng(m_rd()) , m_node_data(node_data) , m_graph(graph)
{
	srand(time(0));
}

std::tuple<bool, size_t, size_t> Local_Search::inter_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType)
{
	size_t uiRobot1, uiRobot2;
	bool bChange = false;
	std::vector<size_t> vec_robots(m_node_data.m_uiNumRobots);
	std::iota(vec_robots.begin(), vec_robots.end(), 0);
	std::shuffle(vec_robots.begin(), vec_robots.end(), m_rng);

	for (size_t uiCount1 = 0; uiCount1 < m_node_data.m_uiNumRobots - 1; uiCount1++)
	{
		uiRobot1 = vec_robots[uiCount1];
		for (size_t uiCount2 = uiCount1 + 1; uiCount2 < m_node_data.m_uiNumRobots; uiCount2++)
		{
			uiRobot2 = vec_robots[uiCount2];
			
			if ("STRING_EXCHANGE" == strType)
			{
				bChange = string_exchange(uiRobot1, uiRobot2, rob_seq);
			}
			else if ("STRING_RELOCATION" == strType)
			{
				bChange = string_relocation(uiRobot1, uiRobot2, rob_seq);
			}
			
			if (true == bChange) return std::make_tuple(true, uiRobot1 , uiRobot2);
		}		
	}
	return std::make_tuple(false , std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());
}

std::pair<bool, size_t> Local_Search::intra_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType)
{
	size_t uiRobot;
	bool bChange = false;
	std::vector<size_t> vec_robots(m_node_data.m_uiNumRobots);
	std::iota(vec_robots.begin(), vec_robots.end(), 0);
	std::shuffle(vec_robots.begin(), vec_robots.end(), m_rng);

	for (size_t uiCount = 0; uiCount < vec_robots.size(); uiRobot++)
	{
		uiRobot = vec_robots[uiCount];

		if ("SWAP_INTRA_SEQUENCE" == strType)
		{
			bChange = swap_intra_sequence(uiRobot , rob_seq);
		}
		else if ("STRING_CROSS_INTRA_SEQUENCE" == strType)
		{
			bChange = string_cross_intra_sequence(uiRobot, rob_seq , true);
		}
		
		if (true == bChange) return std::make_pair(true, uiRobot);
	}
	return std::make_pair(false , std::numeric_limits<size_t>::max());
}

std::tuple<bool, size_t, size_t> Local_Search::wait_based_oper(std::vector<std::list<size_t>> &rob_seq, const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strType)
{
	//<R_Ind, makespan>
	std::vector<std::pair<size_t, size_t>> vec_rob_ind_mkspan;
	
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		vec_rob_ind_mkspan.emplace_back(std::make_pair(uiRobot, full_rob_sch[uiRobot].rbegin()->m_uiEnd));
	}

	std::sort(vec_rob_ind_mkspan.begin(), vec_rob_ind_mkspan.end(), sort_by_max_second_val());

	//iterate over all robots
	for(size_t uiCount = 0; uiCount < vec_rob_ind_mkspan.size(); uiCount++)
	{
		auto res = wait_based_swap_for_robot(full_rob_sch, rob_seq, vec_rob_ind_mkspan[uiCount].first, strType);
		if (true == std::get<0>(res)) return res;
	}
	return std::make_tuple(false, std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());
}

void Local_Search::perform_local_search(std::string strFolderPath)
{
	std::vector<std::list<size_t>> rob_seq;
	std::vector<std::list<size_t>> old_rob_seq;

	std::vector<std::vector<Vertex_Schedule>> full_rob_sch_prev;
	
	generate_constructive_sequence_VBSS(rob_seq);
	bool bValid = check_validity_of_sequence(rob_seq), bSuccess = true;
	if (false == bValid) { cout << "Initial seq generated is invalid\n"; }
	
	std::string strType;
	size_t uiIter = 0, uiMakeSpan, uiMakeSpan_legacy, uiBestSol = std::numeric_limits<size_t>::max();
	Power_Set power;
	bool bFirst_Feasible_Sequence = false;

	Greedy_Heuristic heur(m_node_data.m_uiNumRobots, m_graph, power);

#ifdef ENABLE_LEGACY_CODE
	Greedy_Heuristic_old heur_legacy(m_node_data.m_uiNumRobots, m_graph, power);
#endif

	std::vector<size_t> vec_late_accep(c_uiLate_Acceptace_Length, std::numeric_limits<size_t>::max());
	
	std::clock_t start_time;
	start_time = std::clock();

	while (uiIter < 5000000)
	{
		uiMakeSpan = std::numeric_limits<size_t>::max();
		uiMakeSpan_legacy = std::numeric_limits<size_t>::max();
		
		bValid = check_validity_of_sequence(rob_seq);
#ifdef WINDOWS		
		assert(true == bValid);
#else
		if (false == bValid)
		{
			cout << "Sequence generated is invalid \n";
			exit(1);
		}
#endif

		std::vector<std::vector<Vertex_Schedule>> full_rob_sch;
		std::vector<std::vector<Vertex_Schedule>> full_rob_sch_legacy;

		int iRetVal = perform_greedy_scheduling(heur, rob_seq, full_rob_sch, strFolderPath);
#ifdef ENABLE_LEGACY_CODE			
		int iRetVal_legacy = perform_greedy_scheduling_old(heur_legacy, rob_seq, full_rob_sch_legacy);
#endif

#ifdef ENABLE_LEGACY_CODE
		if (iRetVal != iRetVal_legacy)
		{
			print_sequence(rob_seq);
#ifdef WINDOWS	
			assert(iRetVal == iRetVal_legacy);
#else 
			cout << "assert(iRetVal == iRetVal_legacy)";
			exit(-1);				
#endif		
		}
#endif

		uiMakeSpan = (iRetVal == 1) ? getMakeSpan_From_Schedule(full_rob_sch) : std::numeric_limits<size_t>::max();
#ifdef ENABLE_LEGACY_CODE			
		uiMakeSpan_legacy = (iRetVal_legacy == 1) ? getMakeSpan_From_Schedule(full_rob_sch_legacy) : std::numeric_limits<size_t>::max();
#endif			
		if (uiBestSol > uiMakeSpan)	uiBestSol = uiMakeSpan;	

#ifdef ENABLE_LEGACY_CODE
		if (uiMakeSpan != uiMakeSpan_legacy)
		{
			print_sequence(rob_seq);
#ifdef WINDOWS
			assert(uiMakeSpan == uiMakeSpan_legacy);
#else

			cout << "Makespans not equal";
			exit(1);
#endif		
		}
#endif
			
#ifdef ENABLE_LEGACY_CODE			
		cout << " Iteration: " << uiIter << " , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") << " , Makespan: " << uiMakeSpan << " , Old Makespan: " << uiMakeSpan_legacy << endl;
#else			
		cout << " Iteration: " << uiIter <<" , " << (iRetVal == 1 ? "SUCCESS " : "UNSUCCESSFULL ") <<" , Makespan: " << uiMakeSpan <<" , Best Sol: "<< uiBestSol<< endl;
#endif			
			
		bSuccess = iRetVal == 1 ? true : false;			

		if (false == bFirst_Feasible_Sequence)
		{
			if (false == bSuccess)
			{
				rob_seq.clear();
				generate_constructive_sequence_VBSS(rob_seq);
				uiIter++;
				continue;
			}
			else
			{
				bFirst_Feasible_Sequence = true;
				cout << "Tag: Initial Makespan: " << uiMakeSpan << endl;
				std::fill(vec_late_accep.begin(), vec_late_accep.end(), uiMakeSpan);				
			}
		}

		if (true == bSuccess)
		{
			if (uiMakeSpan > vec_late_accep[uiIter % c_uiLate_Acceptace_Length])
			{
				rob_seq = old_rob_seq;
				full_rob_sch = full_rob_sch_prev;
			}
			else
			{
				vec_late_accep[uiIter % c_uiLate_Acceptace_Length] = uiMakeSpan;
				old_rob_seq = rob_seq;
				full_rob_sch_prev = full_rob_sch;
			}
		}
		else if (false == bSuccess)
		{
			rob_seq = old_rob_seq;
			full_rob_sch = full_rob_sch_prev;
		}
		
		generate_new_sequence(full_rob_sch, heur, rob_seq, bSuccess);
		uiIter++;	

		if (((std::clock() - start_time) / (double)CLOCKS_PER_SEC) > LS_SEARCH_TIME) break;
	}
	cout << "Tag: Best Makespan: " << uiBestSol << endl;
	cout<< "Tag: Total Iterations: " << uiIter << endl;
}

void Local_Search::convert_hole_seq_to_full_seq(const std::vector<std::list<size_t>> &rob_seq, std::vector<std::list<size_t>> &full_rob_seq)
{
	full_rob_seq.resize(m_node_data.m_uiNumRobots);
	const auto &vec_rob_iv = m_graph.get_IV_Vec();

	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		auto it1 = rob_seq[uiRobot].begin();
		full_rob_seq[uiRobot].emplace_back(*it1);
		auto it2 = it1;
		it2++;

		for (; it2 != rob_seq[uiRobot].end(); it2++)
		{
			const auto &vec_iv = vec_rob_iv[uiRobot].map.at(*it1).map.at(*it2).vec;
			for (auto it_iv = vec_iv.begin(); it_iv != vec_iv.end(); it_iv++)
			{
				full_rob_seq[uiRobot].emplace_back(it_iv->getInd());
			}
			full_rob_seq[uiRobot].emplace_back(*it2);
			it1 = it2;
		}
	}
}

size_t Local_Search::get_bottleneck_robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	size_t uiMaxMakeSpan = std::numeric_limits<size_t>::min();
	size_t uiBottle_Neck_Robot = 0;
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		if (full_rob_sch[uiRobot].rbegin()->m_uiEnd >= uiMaxMakeSpan)
		{
			uiMaxMakeSpan = full_rob_sch[uiRobot].rbegin()->m_uiEnd;
			uiBottle_Neck_Robot = uiRobot;
		}
	}
	return uiBottle_Neck_Robot;
}

void Local_Search::get_Wait_Holes_For_Robot(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, size_t uiRobot, std::vector<std::pair<size_t, size_t>> &vec_wait_ind_pos)
{
	size_t uiPos = 0;
	assert(0 == vec_wait_ind_pos.size());

	for (size_t uiInd = 0; uiInd < full_rob_sch[uiRobot].size(); )
	{
		if("IV" == m_graph.getType(full_rob_sch[uiRobot][uiInd].m_uiInd)) continue;
		if (0 < full_rob_sch[uiRobot][uiInd].m_uiWait) vec_wait_ind_pos.emplace_back(std::make_pair(full_rob_sch[uiRobot][uiInd].m_uiInd, uiPos));
		uiPos++;
	}	
}

void Local_Search::generate_new_sequence(const std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, const Greedy_Heuristic &heur, std::vector<std::list<size_t>> &rob_seq, bool bSuccess)
{
	size_t uiChoice;
	std::string strType;
	bool bChange = false, bWait = false;
	
	if (true == bSuccess) bWait = heur.doRobotsWait();

	do
	{
		std::vector<std::list<size_t>> new_rob_seq = rob_seq;
		
		// if the sequence was feasible and waits were detected, then try to resolve via wait operations, else do random
		if( (true == bSuccess) && (true == bWait)) uiChoice = rand() % 5;
		else uiChoice = rand() % 2;
		
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES
		cout << "Local Search Choice: " << uiChoice << endl;
#endif
		
		if (0 == uiChoice)
		{
			strType = rand() % 2 ? "STRING_EXCHANGE" : "STRING_RELOCATION";

#ifdef PRINT_LOCAL_OPERATOR_MESSAGES			
			cout << "String Type: " << strType << endl;
#endif
			
			auto res = inter_rand_oper(new_rob_seq, strType);

			if (true == std::get<0>(res))
			{
				rob_seq[std::get<1>(res)] = new_rob_seq[std::get<1>(res)];
				rob_seq[std::get<2>(res)] = new_rob_seq[std::get<2>(res)];
				bChange = true;
			}
		}
		else if(1 == uiChoice)
		{
			strType = rand() % 2 ? "SWAP_INTRA_SEQUENCE" : "STRING_CROSS_INTRA_SEQUENCE";
			
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES
			cout << "String Type: " << strType << endl;
#endif
			
			auto res = intra_rand_oper(new_rob_seq, strType);
			if (true == std::get<0>(res))
			{
				rob_seq[std::get<1>(res)] = new_rob_seq[std::get<1>(res)];
				bChange = true;
			}
		}
		else
		{
			strType = rand() % 2 ? "INTER_SEQUENCE" : "INTRA_SEQUENCE";
			
#ifdef PRINT_LOCAL_OPERATOR_MESSAGES			
			cout << "String Type: " << strType << endl;
#endif
			
			auto res = wait_based_oper(new_rob_seq, full_rob_sch, strType);
			
			if (true == std::get<0>(res))
			{
				rob_seq[std::get<1>(res)] = new_rob_seq[std::get<1>(res)];
				if (std::numeric_limits<size_t>::max() != std::get<2>(res))
				{
					rob_seq[std::get<2>(res)] = new_rob_seq[std::get<2>(res)];
				}
				bChange = true;
			}
		}
	} while (false == bChange);
}

int Local_Search::perform_greedy_scheduling(Greedy_Heuristic &heur, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch, std::string strFolderPath)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,38,40,222,42,130,324,325,133,34,135,137,128,131,138,43,32,125,328,317,223,139,227,315,323,322,220,35,216,219,47,225,49,143,224,217,37,320,28,142,51,141,41,53,29,226,230,31,45,234,232,214,327,236,321,26,231,210,134,235,146,46,145,238,136,330,208,240,242,36,48,206,140,239,50,233,25,202,22,198,212,329,39,213,332,229,123,55,21,132,311,27,218,44,246,23,313,56,221,211,126,199,207,119,209,129,19,24,243,57,203,117,196,61,150,59,118,113,114,58,152,122,205,204,15,121,16,248,195,312,30,326,228,215,12,241,33,17,310,54,331,127,336,8,52,62,333,124,200,151,63,308,309,237,7,4,250,120,6,318,197,314,144,13,67,335,306,115,148,244,11,5,247,65,60,316,156,159,157,194,147,163,158,9,251,161,149,160,201,110,18,254,68,116,14,338,72,109,112,73,536,167,258,168,75,170,249,79,255,80,107,155,84,154,337,70,340,262,341,266,165,345,74,105,111,252,77,76,173,166,81,108,349,106,264,20,256,175,344,104,176,304,169,300,178,162,265,180,343,181,101,346,172,82,102,103,179,348,85,259,171,78,177,342,185,69,164,66,187,188,261,87,71,191,99,174,267,83,88,64,347,245,186,183,271,352,305,190,100,153,356,299,182,260,269,184,10,357,192,303,298,272,294,360,263,301,257,353,359,270,307,363,290,92,89,364,289,302,91,273,293,365,276,93,274,367,456,451,450,361,280,350,368,370,278,292,86,284,279,371,96,374,296,295,378,358,286,373,376,95,285,379,380,372,193,375,291,275,283,97,287,277,377,90,382,268,369,98,297,282,189,281,94,288,381,253,354,355,383,1});
	//full_rob_seq.push_back({ 2,704,414,602,603,708,600,510,711,699,419,605,421,698,507,598,696,715,503,601,505,695,719,499,712,415,498,425,596,594,504,716,718,509,501,721,417,693,496,700,694,709,599,514,592,702,518,710,429,511,597,690,430,706,500,604,513,426,492,593,517,412,705,418,520,508,521,427,590,434,701,431,494,522,713,692,497,703,319,491,523,432,515,586,516,687,527,433,584,683,722,428,723,519,408,726,682,606,697,580,424,730,512,576,506,529,420,734,409,588,733,607,493,611,587,735,411,612,406,585,738,707,575,583,435,742,410,681,613,438,614,615,413,725,743,732,488,591,724,423,530,617,490,610,526,589,678,677,685,407,403,714,440,746,416,745,720,532,595,674,729,670,581,691,727,437,620,405,442,624,736,749,525,741,621,623,485,750,533,689,618,676,534,616,609,753,608,538,535,673,754,757,675,744,672,577,759,627,439,351,339,334,444,739,531,737,756,761,489,495,679,728,539,731,540,441,487,484,622,446,528,399,401,443,483,686,582,579,524,762,758,752,763,680,402,542,502,395,628,619,393,545,544,740,574,546,751,747,632,479,392,390,445,422,631,404,550,760,400,398,629,541,630,755,684,578,671,625,448,388,397,633,480,554,555,547,543,396,482,394,552,669,717,436,557,688,637,635,626,386,481,452,553,748,639,549,387,449,642,556,646,634,389,645,641,558,650,644,647,652,656,537,643,653,551,660,649,648,655,651,662,654,636,661,548,665,385,658,453,561,565,455,638,562,564,486,569,447,666,391,559,664,570,571,563,459,366,362,454,640,566,663,461,667,567,572,657,384,463,462,659,668,460,464,467,468,466,472,458,471,560,475,476,474,478,477,470,457,469,573,473,465,568,3});
	return heur.compute_greedy_sol(full_rob_seq, full_rob_sch, strFolderPath);
}

int Local_Search::perform_greedy_scheduling_old(Greedy_Heuristic_old &heur_old, const std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<Vertex_Schedule>> &full_rob_sch)
{
	std::vector<std::list<size_t>> full_rob_seq;
	convert_hole_seq_to_full_seq(rob_seq, full_rob_seq);
	//full_rob_seq.push_back({ 0,109,31,1626,52,2724,15,747,37,1942,44,2331,55,2889,18,917,45,2353,22,1112,24,1242,46,2391,6,232,8,353,21,1088,54,2827,10,473,33,1699,16,770,5,182,12,558,9,405,19,941,14,679,23,1167,25,1277,27,1376,17,830,11,520,26,1309,4,129,13,610,7,298,20,1008,28,1452,40,3127,1 });
	//full_rob_seq.push_back({ 2,3179,86,7140,84,6998,72,6232,87,7171,50,4757,41,4196,66,5829,74,6343,67,5889,69,6015,64,5703,78,6624,89,7340,90,7354,38,4020,85,7039,48,4634,49,4694,43,4336,76,6468,62,5557,61,5482,51,4861,81,6813,83,6934,73,6287,77,6561,91,7463,82,6866,70,6079,63,5603,42,4248,53,4969,59,5340,39,4047,47,4551,30,3494,79,6676,75,6412,71,6146,65,5758,68,5973,88,7243,57,5228,58,5292,56,5165,60,5445,80,6695,29,3382,32,3579,34,3710,35,3776,36,7482,3 });
	return heur_old.compute_greedy_sol(full_rob_seq, full_rob_sch);
}


