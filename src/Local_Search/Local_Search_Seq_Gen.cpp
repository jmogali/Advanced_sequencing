#include "Local_Search.h"
#include "Enabling_Graph.h"
#include "Init_Sequence_Generator.h"

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
			uiRobot = rand_select_list_pair_with_bias(m_rng, vec_robot_dist, "LOW_DIST", 5);
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

void Local_Search::gen_seq_VBSS_march_for_robot(size_t uiRobot, std::unordered_set<size_t> &set_holes, std::list<size_t> &hole_seq, const Enabling_Graph &en_graph)
{
	assert(true == hole_seq.empty());
	size_t uiInpSize = set_holes.size() , uiErase;
	std::unordered_set<size_t> set_seen_verts;
	std::unordered_set<size_t> set_curr_enabled_holes;
	const auto& map_enablers = en_graph.get_Node_vec();
	size_t uiVtx = m_node_data.m_rob_depo[uiRobot].first;
	set_seen_verts.emplace(m_node_data.m_rob_depo[uiRobot].first);	
	
	uiErase = set_holes.erase(m_node_data.m_rob_depo[uiRobot].first);
	assert(1 == uiErase);
	
	hole_seq.emplace_back(uiVtx);	// add the start depot vertex

	do
	{
		const auto& vtx_vec_enablers = map_enablers.at(uiVtx).get_neighs();
		for (auto it = vtx_vec_enablers.begin(); it != vtx_vec_enablers.end(); it++)
		{
			size_t uiNeigh = *it;
			if (uiNeigh == m_node_data.m_rob_depo[uiRobot].second) continue;
			auto it_holes = set_holes.find(uiNeigh);
			if (set_holes.end() != it_holes)
			{
				if (set_seen_verts.end() == set_seen_verts.find(uiNeigh))
				{
					set_curr_enabled_holes.emplace(uiNeigh);
					set_seen_verts.emplace(uiNeigh);
					set_holes.erase(it_holes);					
				}
			}
		}

		std::list<std::pair<size_t, double>> list_dist;
		for (auto it_cand = set_curr_enabled_holes.begin(); it_cand != set_curr_enabled_holes.end(); it_cand++)
		{
			auto loc1 = m_graph.getLoc(*it_cand);
			auto loc2 = m_graph.getLoc(uiVtx);
			assert(true == m_graph.doesEdgeExist(uiRobot, uiVtx, *it_cand));
			list_dist.push_back(std::make_pair(*it_cand, loc1.getDist_XY(loc2)));
		}

		uiVtx = rand_select_list_pair_with_bias(m_rng, list_dist, "LOW_DIST", 0.05);
		hole_seq.emplace_back(uiVtx); // add the vertex that was chosen
		uiErase = set_curr_enabled_holes.erase(uiVtx);
		assert(1 == uiErase);
	} while (false == set_curr_enabled_holes.empty());

	assert(0 == set_curr_enabled_holes.size());
	
	//add depot, add unenabled holes in between
	hole_seq.push_back(m_node_data.m_rob_depo[uiRobot].second);
	set_seen_verts.emplace(m_node_data.m_rob_depo[uiRobot].second);
	set_holes.erase(m_node_data.m_rob_depo[uiRobot].second);

	//adds holes that could not reached by the enabling march but still needs to be covered by the robot through shortest diatnce insertion
	for (auto it_unenabled = set_holes.begin(); it_unenabled != set_holes.end(); )
	{
		auto it_insert = hole_seq.begin();
		double dist = std::numeric_limits<double>::max(), dInsertionDist;
		size_t uiUnEnabledHole = *it_unenabled;

		for (auto it = hole_seq.begin(); it != hole_seq.end(); it++)
		{
			auto it_next = it;
			it_next++;
			if (it_next == hole_seq.end()) break;

			if(false == m_graph.doesEdgeExist(uiRobot, *it, uiUnEnabledHole)) continue;
			if (false == m_graph.doesEdgeExist(uiRobot, uiUnEnabledHole, *it_next)) continue;

			dInsertionDist = m_graph.getLoc(*it).getDist_XY(m_graph.getLoc(uiUnEnabledHole));
			dInsertionDist += m_graph.getLoc(uiUnEnabledHole).getDist_XY(m_graph.getLoc(*it_next));

			if (dist < dInsertionDist) continue;		
			
			dist = dInsertionDist;
			it_insert = it_next;
		}

		assert(dist != std::numeric_limits<double>::max());
		hole_seq.insert(it_insert, uiUnEnabledHole);
		set_seen_verts.emplace(uiUnEnabledHole);
		it_unenabled = set_holes.erase(it_unenabled);
	}

	/*if (uiInpSize - 1 != set_seen_verts.size())
	{
		//cout << "Robot: "<< uiRobot <<" , Unenabled hole size: "<< set_holes.size() <<endl;
		for (auto it_unenabled = set_holes.begin(); it_unenabled != set_holes.end(); )
		{
			if (*it_unenabled == m_node_data.m_rob_depo[uiRobot].second)
			{
				it_unenabled++;
				continue;
			}
			assert(set_seen_verts.end() == set_seen_verts.find(*it_unenabled));
			hole_seq.emplace_back(*it_unenabled);
			set_seen_verts.emplace(*it_unenabled);
			it_unenabled = set_holes.erase(it_unenabled);
		}
	}*/

	size_t uiOutSize = hole_seq.size();
#ifdef WINDOWS	
	assert(uiInpSize == set_seen_verts.size());
	assert(0 == set_holes.size());
	assert(uiInpSize == uiOutSize);
#else
	if ((uiInpSize != set_seen_verts.size()) || (0 != set_holes.size()) || (uiInpSize != uiOutSize))
	{
		cout << "Sequence generation error \n";
		exit(1);
	}
#endif	
}


void Local_Search::generate_constructive_sequence_VBSS(std::vector<std::list<size_t>> &rob_seq)
{
	assert(0 == rob_seq.size());
	std::vector<std::unordered_set<size_t>> set_com_hole_par; //set_common_holes_partition to robots
	allocate_holes_to_robots_common_with_bias(set_com_hole_par, "BIAS");

	std::vector<std::list<size_t>> vec_rob_enab_seq;
	vec_rob_enab_seq.resize(m_node_data.m_uiNumRobots);
	Enabling_Graph en_graph(reinterpret_cast<const Layout_Graph&> (m_graph));
	rob_seq.resize(m_node_data.m_uiNumRobots);

	for (size_t uiRobot = 0; uiRobot < m_node_data.m_uiNumRobots; uiRobot++)
	{
		gen_seq_VBSS_march_for_robot(uiRobot, set_com_hole_par[uiRobot], rob_seq[uiRobot], en_graph);
	}
}
