#include "Special_Parser.h"
#include <string>

Special_Parser::Special_Parser() {}

void Special_Parser::populate_comp_holes(std::string strFilledHoles)
{
	ifstream myFile(strFilledHoles);
	std::string line;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(", "));

			boost::algorithm::trim(elems[0]);
			m_set_comp_holes.emplace(elems[0]);
		}
	}
}

void populate_enabler_index_hole_map(std::string strEnablerFile, std::unordered_map < size_t, std::string> &map_index_hole)
{
	ifstream myFile(strEnablerFile);
	std::string line;
	size_t uiOffsetRow = 1, uiOffsetColumn = 2, uiLine = 0;
	
	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			if (uiLine < uiOffsetRow)
			{
				uiLine++;
				continue;
			}

			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(","));
			if (1 == elems.size()) break;

			boost::algorithm::trim(elems[0]);
			boost::algorithm::trim(elems[1]);

			map_index_hole.emplace(atoi(elems[0].c_str()), elems[1]);

			uiLine++;
		}
	}
}

void Special_Parser::populate_adjacency_info(std::string strEnablerFile)
{
	std::unordered_map < size_t, std::string> map_index_hole;
	populate_enabler_index_hole_map(strEnablerFile, map_index_hole);
	
	ifstream myFile(strEnablerFile);
	std::string line;
	size_t uiOffsetRow = 1, uiOffsetColumn = 2, uiLine = 0;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			if (uiLine < uiOffsetRow)
			{
				uiLine++;
				continue;
			}

			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(","));
			if (1 == elems.size()) break;

			boost::algorithm::trim(elems[1]);

			//m_set_adj_buffer.emplace(elems[1], std::set<std::string>());

			for (size_t uiIndex = uiOffsetColumn; uiIndex < elems.size() - 1; uiIndex++)
			{
				boost::algorithm::trim(elems[uiIndex]);
				if (elems[uiIndex] == "T")
				{
					//m_set_adj_buffer[elems[1]].emplace(map_index_hole.at(uiIndex - uiOffsetColumn));

					std::string strHole = map_index_hole.at(uiIndex - uiOffsetColumn);
					if (m_set_adj_buffer.end() == m_set_adj_buffer.find(strHole))
					{
						m_set_adj_buffer.emplace(strHole, std::set<std::string>());
					}
					m_set_adj_buffer[strHole].emplace(elems[1]);
				}
			}
			uiLine++;
		}
	}
}

void Special_Parser::populate_reachable_holes()
{
	std::vector<std::string> vec_reachable_holes;
	std::set<std::string> set_reachable_holes;
	std::string strHole;
		
	for (auto it = m_set_comp_holes.begin(); it != m_set_comp_holes.end(); it++)
	{
		for (auto it_enabled = m_set_adj_buffer.at(*it).begin(); it_enabled != m_set_adj_buffer.at(*it).end(); it_enabled++)
		{
			if (set_reachable_holes.end() != set_reachable_holes.find(*it_enabled)) continue;
			
			vec_reachable_holes.emplace_back(*it_enabled);
			set_reachable_holes.emplace(*it_enabled);
		}
	}

	for (size_t uiIndex = 0; uiIndex < vec_reachable_holes.size(); uiIndex++)
	{
		strHole = vec_reachable_holes[uiIndex];
		for (auto it_enabled = m_set_adj_buffer.at(strHole).begin(); it_enabled != m_set_adj_buffer.at(strHole).end(); it_enabled++)
		{
			if (set_reachable_holes.end() != set_reachable_holes.find(*it_enabled)) continue;
			
			vec_reachable_holes.emplace_back(*it_enabled);
			set_reachable_holes.emplace(*it_enabled);
		}
	}

	//trim completed holes
	for (auto it = m_set_comp_holes.begin(); it != m_set_comp_holes.end(); it++)
	{
		set_reachable_holes.erase(*it);
	}

	size_t uiHole = 0;
	for (auto it = set_reachable_holes.begin(); it != set_reachable_holes.end(); it++)
	{
		m_map_hole_index.emplace(*it, uiHole);
		m_map_index_hole.emplace(uiHole, *it);
		uiHole++;
	}
	m_uiNumHoles = uiHole;
}

void populate_distance_index_hole_map(std::string strDistFile, std::unordered_map<size_t, std::string> &map_index_hole)
{
	ifstream myFile(strDistFile);
	std::string line;
	size_t uiOffsetRow = 2, uiOffsetColumn = 2, uiLine = 0;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			if (uiLine < uiOffsetRow)
			{
				uiLine++;
				continue;
			}

			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(","));
			if (1 == elems.size()) break;

			boost::algorithm::trim(elems[0]);
			boost::algorithm::trim(elems[1]);

			map_index_hole.emplace(atoi(elems[0].c_str()), elems[1]);

			uiLine++;
		}
	}
}

void Special_Parser::populate_distance_buffer(std::string strDistFile)
{
	m_vec_dist_buffer.resize(m_uiNumHoles);
	for (size_t uiHole = 0; uiHole < m_uiNumHoles; uiHole++) m_vec_dist_buffer[uiHole].resize(m_uiNumHoles);

	std::unordered_map<size_t, std::string> map_index_hole;
	populate_distance_index_hole_map(strDistFile, map_index_hole);

	ifstream myFile(strDistFile);
	std::string line;
	size_t uiOffsetRow = 2, uiOffsetColumn = 2, uiLine = 0;

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			if (uiLine < uiOffsetRow)
			{
				uiLine++;
				continue;
			}

			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(","));

			boost::algorithm::trim(elems[0]);
			boost::algorithm::trim(elems[1]);

			auto it_find = m_map_hole_index.find(elems[1]);
			if (m_map_hole_index.end() == it_find) continue; //either not reachable or complete

			size_t uiHoleIndex = it_find->second;
			size_t uiCheckCount = 0;
			for (size_t uiIndex = uiOffsetColumn; uiIndex < elems.size() - 1; uiIndex++)
			{
				it_find = m_map_hole_index.find(map_index_hole.at(uiIndex - uiOffsetColumn));
				if (m_map_hole_index.end() == it_find) continue; //either not reachable or complete
				size_t uiOtherHoleIndex = it_find->second;

				m_vec_dist_buffer[uiHoleIndex][uiOtherHoleIndex] = atof(elems[uiIndex].c_str());
				uiCheckCount++;
			}
			assert(uiCheckCount = m_uiNumHoles);
			uiLine++;
		}
	}
}

void Special_Parser::populate_hole_coord(std::string strHoleFile)
{
	ifstream myFile(strHoleFile);
	std::string line;
	m_vec_hole_coords.resize(m_uiNumHoles, Coordinates(0,0,0));

	if (myFile.is_open())
	{
		while (getline(myFile, line))
		{
			std::vector<std::string> elems;
			boost::split(elems, line, boost::is_any_of(", "));

			boost::algorithm::trim(elems[0]);
			auto pr = isValidHole(elems[0]);
			if(false == pr.first) continue;

			m_vec_hole_coords[pr.second] = Coordinates(atof(elems[1].c_str()), atof(elems[2].c_str()), atof(elems[3].c_str()));
		}
	}
}

void Special_Parser::parse_files(std::string strHoleFile, std::string strDistFile, std::string strEnablerFile, std::string strFilledHoles)
{	
	populate_comp_holes(strFilledHoles);
	populate_adjacency_info(strEnablerFile);
	populate_reachable_holes();
	populate_distance_buffer(strDistFile);
	populate_hole_coord(strHoleFile);
}

std::pair<bool, size_t> Special_Parser::isValidHole(std::string strHole) const
{
	auto it_find = m_map_hole_index.find(strHole);
	if (m_map_hole_index.end() == it_find) return std::make_pair(false , std::numeric_limits<size_t>::max());
	else return std::make_pair(true , it_find->second);
}