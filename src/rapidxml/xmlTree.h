#pragma once

#include "rapidxml.hpp"
#include <string>
#include <sstream>

class xmlTree {
	rapidxml::xml_document<> doc;
public:
	void begin();
	rapidxml::xml_node<>* add(const char* nodeName);
	int end(const std::string& fileName);
	rapidxml::xml_node<>* add(rapidxml::xml_node<>* node, const char* childName, const char* childValue);
	rapidxml::xml_node<>* add(rapidxml::xml_node<>* node, const char* childName);

	template<typename ValueType>
	rapidxml::xml_node<>* add(rapidxml::xml_node<>* parent, const char* childName, ValueType childValue) {
		std::ostringstream stringValue;
		stringValue << childValue;
		return add(parent, childName, stringValue.str().c_str());
	}
};
