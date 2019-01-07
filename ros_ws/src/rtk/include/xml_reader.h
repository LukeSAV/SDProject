#pragma once
#include <string>
#include <map>
#include <vector>
#include "Node.h"
#include "XMLParser/rapidxml.hpp"
void xml_reader(const char* filename); // , std::map<std::string, Node> &node_map, std::map<std::string, std::pair<std::vector<std::string>, std::string>> &way_map
	// Map of unique identifiers to their respective nodes
	// Map of unique identifiers to their respective ways/landmarks (containing the node ids as strings and the name of the landmark)
