#include "Local_Search.h"
#include "Enabling_Graph.h"

void Local_Search::generate_rand_allocation_of_nodes_to_robots(std::vector<std::vector<size_t>> &vec_com_hole_par)
{
	vec_com_hole_par.resize(m_node_data.m_uiNumRobots);
	size_t uiRobot, rand_num;	

	//assign all local holes to robots and starting depot at begining
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		vec_com_hole_par[uiRobot].push_back(m_node_data.m_rob_depo[uiRobot].first);
		for (auto it = m_node_data.m_vec_local_nodes[uiRobot].set.begin(); it != m_node_data.m_vec_local_nodes[uiRobot].set.end(); it++)
		{
			vec_com_hole_par[uiRobot].push_back(*it);
		}
	}

	//randomly assign a common hole to one of its reachable robots 
	for (auto it = m_node_data.m_map_common_nodes.begin(); it != m_node_data.m_map_common_nodes.end(); it++)
	{
		rand_num = rand() % it->second.set.size();
		std::unordered_set<size_t>::const_iterator it_set(it->second.set.begin());
		std::advance(it_set, rand_num);
		uiRobot = *it_set;
		vec_com_hole_par[uiRobot].push_back(it->first);
	}

	//shuffle the allocations sparing the start depot, after shuffling add end depot
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		std::shuffle(vec_com_hole_par[uiRobot].begin() + 1, vec_com_hole_par[uiRobot].end(), m_rng);
		vec_com_hole_par[uiRobot].push_back(m_node_data.m_rob_depo[uiRobot].second);
	}	
}

void Local_Search::generate_nearest_neigh_alloc_to_robots(std::vector<std::vector<size_t>> &vec_com_hole_par)
{
	vec_com_hole_par.resize(m_node_data.m_uiNumRobots);
	size_t uiRobot, rand_num;

	//assign all local holes to robots and starting depot at begining
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		vec_com_hole_par[uiRobot].push_back(m_node_data.m_rob_depo[uiRobot].first);
		for (auto it = m_node_data.m_vec_local_nodes[uiRobot].set.begin(); it != m_node_data.m_vec_local_nodes[uiRobot].set.end(); it++)
		{
			vec_com_hole_par[uiRobot].push_back(*it);
		}
	}

	//randomly assign a common hole to one of its reachable robots 
	for (auto it = m_node_data.m_map_common_nodes.begin(); it != m_node_data.m_map_common_nodes.end(); it++)
	{	
		std::unordered_set<size_t> set_near_robots;
		m_graph.get_nearest_robots_for_hole(it->first, set_near_robots);
		rand_num = rand() % set_near_robots.size();
		std::unordered_set<size_t>::const_iterator it_set(set_near_robots.begin());
		std::advance(it_set, rand_num);
		uiRobot = *it_set;
		vec_com_hole_par[uiRobot].push_back(it->first);
	}

	//add end depot
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		vec_com_hole_par[uiRobot].push_back(m_node_data.m_rob_depo[uiRobot].second);
	}
}

void Local_Search::generate_rand_sequence(std::vector<std::list<size_t>> &rob_seq)
{
	std::vector<std::vector<size_t>> vec_com_hole_par;
	generate_rand_allocation_of_nodes_to_robots(vec_com_hole_par);
	rob_seq.resize(m_node_data.m_uiNumRobots);
	
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		rob_seq[uiRobot].insert(rob_seq[uiRobot].begin(), vec_com_hole_par[uiRobot].begin(), vec_com_hole_par[uiRobot].end());
	}
}

void Local_Search::remove_holes_that_are_not_assigned_to_robots_from_enabler_seq(size_t c_uiNumRobots, std::vector<std::vector<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &vec_rob_enab_seq)
{
	for (size_t uiRobot = 0; uiRobot < c_uiNumRobots; uiRobot++)
	{
		for (auto it = vec_rob_enab_seq[uiRobot].begin(); it != vec_rob_enab_seq[uiRobot].end();)
		{
			// hole sequence returned by enabler graph is used to generate sequence for holes in vec_com_hole_par[uiRobot]
			auto it_vec = std::find(vec_com_hole_par[uiRobot].begin(), vec_com_hole_par[uiRobot].end(), *it);  // inefficient, but its okay since one time operation
			if (it_vec == vec_com_hole_par[uiRobot].end())
			{
				it = vec_rob_enab_seq[uiRobot].erase(it);
			}
			else
			{
				vec_com_hole_par[uiRobot].erase(it_vec);  // erasing helps us figure out what the remaining vertices are
				it++;
			}
		}
	}
}

void Local_Search::add_holes_not_reachable_by_enabler_sequence(size_t c_uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &vec_rob_enab_seq)
{
	rob_seq.resize(m_node_data.m_uiNumRobots);
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		// insert robots that have not been assigned 
		if (0 == vec_com_hole_par[uiRobot].size())
		{
			rob_seq[uiRobot] = vec_rob_enab_seq[uiRobot];
			continue;
		}

		for (size_t uiCount = 0; uiCount < vec_com_hole_par[uiRobot].size(); uiCount++)
		{
			auto it_insert = vec_rob_enab_seq[uiRobot].begin();
			double dist = std::numeric_limits<double>::max(), dInsertionDist;
			for (auto it1 = vec_rob_enab_seq[uiRobot].begin(); it1 != vec_rob_enab_seq[uiRobot].end(); it1++)
			{
				auto it2 = it1;
				it2++;
				if (it2 == vec_rob_enab_seq[uiRobot].end()) break;

				dInsertionDist = m_graph.getLoc(vec_com_hole_par[uiRobot][uiCount]).getDist_XY(m_graph.getLoc(*it1));
				dInsertionDist += m_graph.getLoc(vec_com_hole_par[uiRobot][uiCount]).getDist_XY(m_graph.getLoc(*it2));

				if (dist < dInsertionDist) continue;
				dist = dInsertionDist;
				it_insert = it2;
			}
			vec_rob_enab_seq[uiRobot].insert(it_insert, vec_com_hole_par[uiRobot][uiCount]);
		}
		rob_seq[uiRobot] = vec_rob_enab_seq[uiRobot];
	}
}

void Local_Search::assign_hole_seq_to_robots_from_enabling_seq(size_t c_uiNumRobots, std::vector<std::list<size_t>> &rob_seq, std::vector<std::vector<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &vec_rob_enab_seq)
{
	remove_holes_that_are_not_assigned_to_robots_from_enabler_seq(c_uiNumRobots, vec_com_hole_par, vec_rob_enab_seq);
	add_holes_not_reachable_by_enabler_sequence(c_uiNumRobots, rob_seq, vec_com_hole_par, vec_rob_enab_seq);
}


void Local_Search::generate_constructive_sequence(std::vector<std::list<size_t>> &rob_seq)
{
	std::vector<std::vector<size_t>> vec_com_hole_par;
	//generate_rand_allocation_of_nodes_to_robots(vec_com_hole_par);    // randomly allocates common holes to robots
	generate_nearest_neigh_alloc_to_robots(vec_com_hole_par);

	std::vector<std::list<size_t>> vec_rob_enab_seq;
	vec_rob_enab_seq.resize(m_node_data.m_uiNumRobots);
	Enabling_Graph en_graph( reinterpret_cast<const Layout_Graph&> (m_graph) );
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		// traverses the enabling graph from a given starting vertex
		en_graph.compute_rand_biased_enabled_seq_from_start_vtx(m_node_data.m_rob_depo[uiRobot].first, vec_rob_enab_seq[uiRobot], m_graph, m_rng);
		vec_rob_enab_seq[uiRobot].push_back(m_node_data.m_rob_depo[uiRobot].second);
	}

	assign_hole_seq_to_robots_from_enabling_seq(m_node_data.m_uiNumRobots, rob_seq, vec_com_hole_par, vec_rob_enab_seq);	
}
