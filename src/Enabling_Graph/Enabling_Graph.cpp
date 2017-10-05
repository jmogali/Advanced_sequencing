#include "Enabling_Graph.h"
#include "Init_Sequence_Generator.h"

Enabling_Node::Enabling_Node(size_t uiInd) : m_uiInd(uiInd)
{}

bool Enabling_Node::isNodeReachable(size_t uiRobot) const
{
	return m_map_reachable_robots.find(uiRobot) != m_map_reachable_robots.end() ? true : false;
}

Enabling_Graph::Enabling_Graph(const Layout_Graph &graph)
{
	const auto &vec_enabler = graph.get_Enablers();
	for (size_t uiCount = 0; uiCount < vec_enabler.size(); uiCount++)
	{
		m_vec_nodes.push_back(Enabling_Node(uiCount));
	}

	for (size_t uiCount = 0; uiCount < vec_enabler.size(); uiCount++)
	{
		const auto &map_enabled = vec_enabler[uiCount].set;
		for (auto it = map_enabled.begin(); it != map_enabled.end(); it++)
		{
			add_neigh(it->getInd(), uiCount);
		}
	}

	const auto &vec_rob_iv = graph.get_IV_Vec();
	for (size_t uiRobot = 0; uiRobot < graph.get_num_robots(); uiRobot++)
	{
		for (auto it1 = vec_rob_iv[uiRobot].map.begin(); it1 != vec_rob_iv[uiRobot].map.end(); it1++)
		{
			add_robot(it1->first.getInd(), uiRobot);
		}
	}
	//End depots are not added on purpose
}

void Enabling_Graph::compute_rand_biased_enabled_seq_from_start_vtx(size_t uiStart, std::list<size_t> &seq_enab, const Layout_LS &graph, std::mt19937 &rng)
{
	std::vector<std::string> vec_visit_status(m_vec_nodes.size(), "WHITE");
	return traverse_graph(uiStart, seq_enab, vec_visit_status, graph, rng);		
}

bool comparePairs(const std::pair<double, size_t>& lhs, const std::pair<double, size_t>& rhs)
{
	if (lhs.first < rhs.first) return true;
	else if (lhs.first > rhs.first) return false;
	else if (lhs.second < rhs.second) return true;
	else return false;
}

void Enabling_Graph::traverse_graph(size_t uiStart, std::list<size_t> &seq_enab, std::vector<std::string> &vec_visit_status, const Layout_LS &graph, std::mt19937 &rng)
{
	vec_visit_status[uiStart] = "GRAY";
	seq_enab.push_back(uiStart);
	std::list<std::pair<double, size_t>> list_neighs;
	
	{
		const auto &vec_neighs = m_vec_nodes[uiStart].get_neighs();
		for (size_t uiCount = 0; uiCount < vec_neighs.size(); uiCount++)
		{
			list_neighs.emplace_back(exp(-0.005 * graph.getLoc(uiStart).getDist_XY(graph.getLoc(vec_neighs[uiCount]))), vec_neighs[uiCount]);
		}		
	}

	std::list<size_t> new_biased_list_order;
	size_t uiNeighSize = list_neighs.size();
	bias_node_search_by_weights(rng, new_biased_list_order, list_neighs);
	assert(uiNeighSize == new_biased_list_order.size());

	for (auto it = new_biased_list_order.begin(); it != new_biased_list_order.end() ; it++)
	{
		if ("WHITE" != vec_visit_status[*it]) continue;
		traverse_graph(*it, seq_enab, vec_visit_status, graph, rng);
	}
	vec_visit_status[uiStart] = "BLACK";
}

/*
void Enabling_Graph::traverse_graph(size_t uiStart, std::list<size_t> &seq_enab, std::vector<std::string> &vec_visit_status, const Layout_LS &graph)
{
	vec_visit_status[uiStart] = "GRAY";
	seq_enab.push_back(uiStart);
	std::vector<std::pair<double, size_t>> vec_sorted_neighs;

	{
		const auto &vec_neighs = m_vec_nodes[uiStart].get_neighs();
		for (size_t uiCount = 0; uiCount < vec_neighs.size(); uiCount++)
		{
			vec_sorted_neighs.emplace_back(graph.getLoc(uiStart).getDist_XY(graph.getLoc(vec_neighs[uiCount])), vec_neighs[uiCount]);
		}
		std::sort(vec_sorted_neighs.begin(), vec_sorted_neighs.end(), comparePairs);
	}

	for (size_t uiCount = 0; uiCount < vec_sorted_neighs.size(); uiCount++)
	{
		if ("WHITE" != vec_visit_status[vec_sorted_neighs[uiCount].second]) continue;
		traverse_graph(vec_sorted_neighs[uiCount].second, seq_enab, vec_visit_status, graph);
	}
	vec_visit_status[uiStart] = "BLACK";
}
*/
