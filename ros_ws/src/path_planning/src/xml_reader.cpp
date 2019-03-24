#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "../include/XMLParser/rapidxml_utils.hpp"
#include "../include/XMLParser/rapidxml_print.hpp"
#include "../include/xml_reader.h"
#include "../include/GlobalNode.h"
#include "../include/GlobalMap.h"

void xml_reader(const char* filename) { // , std::map<std::string, GlobalNode> &node_map, std::map<std::string, std::pair<std::vector<std::string>, std::string>> &way_map
	rapidxml::file<> xmlFile(filename); 
	rapidxml::xml_document<> doc;
	doc.parse<rapidxml::parse_validate_closing_tags>(xmlFile.data());
	rapidxml::xml_node<>* root_node = doc.first_node("osm");
	
	for(rapidxml::xml_node<>* coord_node = root_node->first_node("node"); coord_node; coord_node = coord_node->next_sibling()) {
		std::string node_name = std::string(coord_node->name());
		if(node_name == "node") {
			GlobalMap::node_map.insert(std::pair<std::string, GlobalNode>(coord_node->first_attribute("id")->value(), GlobalNode(coord_node->first_attribute("lat")->value(), coord_node->first_attribute("lon")->value())));	
		}
		else if(node_name == "path") {
			GlobalMap::path_map.insert(std::pair<std::string, GlobalNode>(coord_node->first_attribute("id")->value(), GlobalNode(coord_node->first_attribute("lat")->value(), coord_node->first_attribute("lon")->value())));	
		}
		else if(node_name == "way") {
			std::vector<std::string> node_vec; // Vector contains ids corresponding to each node of the landmark
			std::string way_uid = coord_node->first_attribute("id")->value();
			std::string landmark_name;
			for(rapidxml::xml_node<>* way_node = coord_node->first_node("nd"); way_node; way_node = way_node->next_sibling()) {
				std::string way_node_name = std::string(way_node->name());
				
				if(way_node_name == "nd") {
					node_vec.push_back(way_node->first_attribute("ref")->value());
				}
				else if(way_node_name == "tag") {
					std::string k = way_node->first_attribute("k")->value();
					if(k == "name") {
						landmark_name = way_node->first_attribute("v")->value();
					}
				}
			}
			auto map_element = std::pair<std::vector<std::string>, std::string>(node_vec, landmark_name);
			GlobalMap::landmark_map.insert(std::pair<std::string, std::pair<std::vector<std::string>, std::string>>(way_uid, map_element));
		}
	}
}
