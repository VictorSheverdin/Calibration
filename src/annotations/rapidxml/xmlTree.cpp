#include "xmlTree.h"

namespace rapidxml {
	namespace internal {
		template <class OutIt, class Ch>
		inline OutIt print_children(OutIt out, const xml_node<Ch>* node, int flags, int indent);

		template <class OutIt, class Ch>
		inline OutIt print_attributes(OutIt out, const xml_node<Ch>* node, int flags);

		template <class OutIt, class Ch>
		inline OutIt print_data_node(OutIt out, const xml_node<Ch>* node, int flags, int indent);

		template <class OutIt, class Ch>
		inline OutIt print_cdata_node(OutIt out, const xml_node<Ch>* node, int flags, int indent);

		template <class OutIt, class Ch>
		inline OutIt print_element_node(OutIt out, const xml_node<Ch>* node, int flags, int indent);

		template <class OutIt, class Ch>
		inline OutIt print_declaration_node(OutIt out, const xml_node<Ch>* node, int flags, int indent);

		template <class OutIt, class Ch>
		inline OutIt print_comment_node(OutIt out, const xml_node<Ch>* node, int flags, int indent);

		template <class OutIt, class Ch>
		inline OutIt print_doctype_node(OutIt out, const xml_node<Ch>* node, int flags, int indent);

		template <class OutIt, class Ch>
		inline OutIt print_pi_node(OutIt out, const xml_node<Ch>* node, int flags, int indent);
	}
}
#include "rapidxml_print.hpp"
#include <fstream>
#include <sstream>
using namespace rapidxml;


void xmlTree::begin() {
	xml_node<>* header = doc.allocate_node(node_declaration);
	header->append_attribute(doc.allocate_attribute("version", "1.0"));
	header->append_attribute(doc.allocate_attribute("encoding", "utf-8"));
	doc.append_node(header);
}

xml_node<>* xmlTree::add(const char* nodeName) {
	xml_node<>* child = doc.allocate_node(node_element, nodeName);
	doc.append_node(child);
	return child;
}


xml_node<>* xmlTree::add(rapidxml::xml_node<>* parent, const char* childName, const char* childValue) {
	xml_node<>* child = doc.allocate_node(node_element, childName);
	child->value(doc.allocate_string(childValue));
	parent->append_node(child);
	return child;
}

xml_node<>* xmlTree::add(rapidxml::xml_node<>* parent, const char* childName) {
	xml_node<>* child = doc.allocate_node(node_element, childName);
	parent->append_node(child);
	return child;
}

//#include <Windows.h>

int xmlTree::end(const std::string& fileName) {
	std::ofstream file(fileName, std::ios::out);
	if (!file) return -1;
	file << doc;
	file.close();
	doc.clear();
	return 0;
}
