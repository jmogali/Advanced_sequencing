#include "Local_Search.h"
#include <numeric>

void Local_Search::generate_new_sequence_rand_moves(std::vector<std::list<size_t>> &rob_seq)
{
	size_t uiChoice;
	std::string strType;
	bool bChange = false;

	do
	{
		std::vector<std::list<size_t>> new_rob_seq = rob_seq;
		uiChoice = rand() % 3;

#ifdef PRINT_LOCAL_OPERATOR_MESSAGES
		cout << "Local Search Choice: " << uiChoice << endl;
#endif

		if (uiChoice <= 1)
		{
			uiChoice = rand() % 2;
			if (0 == uiChoice) strType = "STRING_EXCHANGE";
			else if (1 == uiChoice) strType = "STRING_RELOCATION";

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
		else
		{
			strType = "SWAP_INTRA_SEQUENCE";

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
	} while (false == bChange);
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

			if (true == bChange) return std::make_tuple(true, uiRobot1, uiRobot2);
		}
	}
	return std::make_tuple(false, std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());
}

std::pair<bool, size_t> Local_Search::intra_rand_oper(std::vector<std::list<size_t>> &rob_seq, std::string strType)
{
	size_t uiRobot;
	bool bChange = false;
	std::vector<size_t> vec_robots(m_node_data.m_uiNumRobots);
	std::iota(vec_robots.begin(), vec_robots.end(), 0);
	std::shuffle(vec_robots.begin(), vec_robots.end(), m_rng);

	for (size_t uiCount = 0; uiCount < vec_robots.size(); uiCount++)
	{
		uiRobot = vec_robots[uiCount];

		if ("SWAP_INTRA_SEQUENCE" == strType)
		{
			bChange = swap_intra_sequence(uiRobot, rob_seq);
		}

		if (true == bChange) return std::make_pair(true, uiRobot);
	}
	return std::make_pair(false, std::numeric_limits<size_t>::max());
}