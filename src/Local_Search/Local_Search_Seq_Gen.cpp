#include "Local_Search.h"
#include "Enabling_Graph.h"
#include "Init_Sequence_Generator.h"
#include "Sequence_Visualizer.h"

void Local_Search::allocate_holes_to_robots_common_with_bias(std::vector<std::unordered_set<size_t>> &vec_com_hole_par, std::string strBias)
{
	assert(("BIAS" == strBias) || ("RANDOM" == strBias));
	vec_com_hole_par.resize(m_node_data.m_uiNumRobots);
	size_t uiRobot, rand_num;

	//assign all local holes to robots and starting depot at begining
	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		vec_com_hole_par[uiRobot].emplace(m_node_data.m_rob_depo[uiRobot].first); // add start depot
		vec_com_hole_par[uiRobot].emplace(m_node_data.m_rob_depo[uiRobot].second); // add end depot
		// add holes local to each robot
		for (auto it = m_node_data.m_vec_local_nodes[uiRobot].set.begin(); it != m_node_data.m_vec_local_nodes[uiRobot].set.end(); it++)
		{
			vec_com_hole_par[uiRobot].emplace(*it);
		}
	}

	//assign a common hole to one of its reachable robots randomly or with bias
	for (auto it = m_node_data.m_map_common_nodes.begin(); it != m_node_data.m_map_common_nodes.end(); it++)
	{
		if ("BIAS" == strBias)
		{
			std::vector<std::pair<size_t, double>> vec_robot_dist;	// robot, distance
			for (auto it_robot = it->second.set.begin(); it_robot != it->second.set.end(); it_robot++)
			{
				auto loc1 = m_graph.getLoc(m_node_data.m_rob_depo[*it_robot].first);
				auto loc2 = m_graph.getLoc(it->first);
				vec_robot_dist.push_back(std::make_pair(*it_robot, loc2.getDist_XY(loc1)));
			}
			uiRobot = rand_select_list_pair_with_bias(m_rng, vec_robot_dist, "LOW_DIST", m_dWeight_Factor * 2.5);
			vec_com_hole_par[uiRobot].emplace(it->first);
		}
		else if("RANDOM" == strBias)
		{
			rand_num = rand() % it->second.set.size();
			std::unordered_set<size_t>::const_iterator it_set(it->second.set.begin());
			std::advance(it_set, rand_num);
			uiRobot = *it_set;
			vec_com_hole_par[uiRobot].emplace(it->first);
		}
	}
}

bool Local_Search::add_new_enabled_holes(const std::vector<size_t> &vec_rob_curr_node, std::unordered_set<size_t> &set_rem_enabled_holes, const std::unordered_set<size_t> &set_completed_verts)
{
	const auto& map_enabled = m_en_graph.get_Node_vec();
	bool bAdded = false;

	for (size_t uiRobot = 0; uiRobot < vec_rob_curr_node.size(); uiRobot++)
	{
		const auto& vtx_vec_enabled = map_enabled.at(vec_rob_curr_node[uiRobot]).get_neighs();
		for (auto it = vtx_vec_enabled.begin(); it != vtx_vec_enabled.end(); it++)
		{
			size_t uiNeigh = *it;
			if (uiNeigh == m_node_data.m_rob_depo[uiRobot].second) continue;			
			auto it_neigh_holes = set_completed_verts.find(uiNeigh);

			if (set_completed_verts.end() == it_neigh_holes)
			{
				if (set_rem_enabled_holes.end() == set_rem_enabled_holes.find(uiNeigh))
				{
					set_rem_enabled_holes.emplace(uiNeigh);
					bAdded = true;
				}
			}
		}
	}
	return bAdded;
}

void Local_Search::gen_seq_VBSS_march_for_robot(std::vector<std::unordered_set<size_t>> &vec_com_hole_par, std::vector<std::list<size_t>> &rob_seq)
{
	std::unordered_set<size_t> set_compl_verts;
	std::unordered_set<size_t> set_rem_enabled_holes;
	std::vector<size_t> vec_rob_curr_node;
	
	//add start depots for each robot
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		rob_seq[uiRobot].emplace_back(m_node_data.m_rob_depo[uiRobot].first);
		vec_rob_curr_node.emplace_back(m_node_data.m_rob_depo[uiRobot].first);
		set_compl_verts.emplace(m_node_data.m_rob_depo[uiRobot].first);
	}
	
	bool bAdded = add_new_enabled_holes(vec_rob_curr_node, set_rem_enabled_holes, set_compl_verts);
	assert(true == bAdded);

	while (set_rem_enabled_holes.size() != 0)
	{
		for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
		{
			std::list<std::pair<size_t, double>> list_dist;
			size_t uiVtx;
			uiVtx = vec_rob_curr_node[uiRobot];
			
			for (auto it_cand = set_rem_enabled_holes.begin(); it_cand != set_rem_enabled_holes.end(); it_cand++)
			{
				if (true == m_graph.doesEdgeExist(uiRobot, uiVtx, *it_cand))
				{
					if (vec_com_hole_par[uiRobot].end() != vec_com_hole_par[uiRobot].find(*it_cand))
					{
						list_dist.push_back(std::make_pair(*it_cand, (m_graph.getEdgeDist(uiRobot, uiVtx, *it_cand)/10.0)));
					}
				}
			}

			if (0 == list_dist.size()) continue;
			uiVtx = rand_select_list_pair_with_bias(m_rng, list_dist, "LOW_DIST", m_dWeight_Factor * 0.05);

			vec_rob_curr_node[uiRobot] = uiVtx;
			rob_seq[uiRobot].emplace_back(uiVtx);
			set_compl_verts.emplace(uiVtx);
			set_rem_enabled_holes.erase(uiVtx);
		}
		bAdded = add_new_enabled_holes(vec_rob_curr_node, set_rem_enabled_holes, set_compl_verts);
	}

	assert(false == bAdded);
	//add start depots for each robot, erase start and end depots from vec_com_hope_par
	for (size_t uiRobot = 0; uiRobot < rob_seq.size(); uiRobot++)
	{
		rob_seq[uiRobot].emplace_back(m_node_data.m_rob_depo[uiRobot].second);
#ifdef WINDOWS		
		assert(rob_seq[uiRobot].size() == vec_com_hole_par[uiRobot].size());
#else
		if(rob_seq[uiRobot].size() != vec_com_hole_par[uiRobot].size())
		{
			cout << "Certain holes are not covered in VBSS march\n";
			exit(-1);
		}
#endif		
	}
}

void Local_Search::generate_constructive_sequence_VBSS(std::vector<std::list<size_t>> &rob_seq)
{
	assert(0 == rob_seq.size());
	std::vector<std::unordered_set<size_t>> vec_com_hole_par; //set_common_holes_partition to robots
	allocate_holes_to_robots_common_with_bias(vec_com_hole_par, "BIAS");

	std::vector<std::list<size_t>> vec_rob_enab_seq;
	vec_rob_enab_seq.resize(m_node_data.m_uiNumRobots);
	rob_seq.resize(m_node_data.m_uiNumRobots);

	gen_seq_VBSS_march_for_robot(vec_com_hole_par, rob_seq);
}
