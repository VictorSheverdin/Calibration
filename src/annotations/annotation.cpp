#include "annotation.h"

#include <vector>
#include <string>
#include "src/rapidxml/xmlTree.h"

#include <fstream>
#include <streambuf>
#include <cstdlib>
#include <cmath>
#define CHECKOK(X) if(!(X)) return -1

//#include <iostream>

int Annotation::LoadFromFile(std::string file){
	rapidxml::xml_document<> doc;
	std::ifstream t(file);
	if(!t) return -1;
	std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
	t.close();
	doc.parse<0>((char*)str.c_str());
	rapidxml::xml_node<>* annot = doc.first_node();
	CHECKOK(annot);
	
	rapidxml::xml_node<>* ann_filename = annot->first_node("filename");
	CHECKOK(ann_filename);
	filename = std::string(ann_filename->value());
	
	rapidxml::xml_node<>* ann_size = annot->first_node("size");
	CHECKOK(ann_size);
	rapidxml::xml_node<>* ann_width = ann_size->first_node("width");
	CHECKOK(ann_width);
	size.width = atoi(ann_width->value());
	rapidxml::xml_node<>* ann_height = ann_size->first_node("height");
	CHECKOK(ann_height);
	size.height = atoi(ann_height->value());
	rapidxml::xml_node<>* ann_depth = ann_size->first_node("depth");
	CHECKOK(ann_depth);
	size.depth = atoi(ann_depth->value());
	
	rapidxml::xml_node<>* ann_position = annot->first_node("position");
	CHECKOK(ann_size);
	rapidxml::xml_node<>* ann_x = ann_position->first_node("x");
	CHECKOK(ann_x);
	position.x = atof(ann_x->value());
	rapidxml::xml_node<>* ann_y = ann_position->first_node("y");
	CHECKOK(ann_y);
	position.y = atof(ann_y->value());
	rapidxml::xml_node<>* ann_z = ann_position->first_node("z");
	CHECKOK(ann_z);
	position.z = atof(ann_z->value());
	
	rapidxml::xml_node<>* ann_obj = ann_position;
    while( ( ann_obj = ann_obj->next_sibling("object") ) != nullptr ){
		AnnotatedObject obj;
		rapidxml::xml_node<>* ann_name = ann_obj->first_node("name");
		CHECKOK(ann_name);
		obj.name = std::string(ann_name->value());
		
		rapidxml::xml_node<>* ann_bndbox = ann_obj->first_node("bndbox");
		CHECKOK(ann_bndbox);
		rapidxml::xml_node<>* ann_xmin = ann_bndbox->first_node("xmin");
		CHECKOK(ann_xmin);
		obj.bndbox.xmin = atoi(ann_xmin->value());
		rapidxml::xml_node<>* ann_ymin = ann_bndbox->first_node("ymin");
		CHECKOK(ann_ymin);
		obj.bndbox.ymin = atoi(ann_ymin->value());
		rapidxml::xml_node<>* ann_xmax = ann_bndbox->first_node("xmax");
		CHECKOK(ann_xmax);
		obj.bndbox.xmax = atoi(ann_xmax->value());
		rapidxml::xml_node<>* ann_ymax = ann_bndbox->first_node("ymax");
		CHECKOK(ann_ymax);
		obj.bndbox.ymax = atoi(ann_ymax->value());
		
		rapidxml::xml_node<>* ann_distance = ann_bndbox->first_node("distance");
		CHECKOK(ann_distance);
		obj.bndbox.distance = atof(ann_distance->value());
		
		objects.push_back(obj);
	}
	return 0;
}

int Annotation::SaveToFile(std::string file){
	xmlTree tree;
	tree.begin();
	rapidxml::xml_node<>* ann_frame = tree.add("annotation");
	tree.add(ann_frame, "filename", filename);
	rapidxml::xml_node<>* ann_size = tree.add(ann_frame, "size");
	tree.add(ann_size, "width", size.width);
	tree.add(ann_size, "height", size.height);
	tree.add(ann_size, "depth", size.depth);
	rapidxml::xml_node<>* ann_pos = tree.add(ann_frame, "position");
	tree.add(ann_pos, "x", position.x);
	tree.add(ann_pos, "y", position.y);
	tree.add(ann_pos, "z", position.z);
	
	for (std::vector<AnnotatedObject>::iterator i = objects.begin() ; i != objects.end(); ++i){
		rapidxml::xml_node<>* obj = tree.add(ann_frame, "object");
		tree.add(obj, "name", i->name);
		rapidxml::xml_node<>* box = tree.add(obj, "bndbox");
		tree.add(box, "xmin", i->bndbox.xmin);
		tree.add(box, "ymin", i->bndbox.ymin);
		tree.add(box, "xmax", i->bndbox.xmax);
		tree.add(box, "ymax", i->bndbox.ymax);
		tree.add(box, "distance", i->bndbox.distance);
	}
	
	return tree.end(file);
}

std::vector<Annotation> LoadAnnotations(std::vector<std::string> files){
	std::vector<Annotation> annotations;
	Annotation a;
    for ( size_t i = 0; i < files.size(); i++){
		if (a.LoadFromFile(files[i]) == 0)
			annotations.push_back(a);
	}
	return annotations;
}

void SaveAnnotations(std::vector<Annotation> annotations, std::vector<std::string> files){
	int s = (annotations.size() < files.size()) ? annotations.size() : files.size();
	for ( int i = 0; i < s; i++)
		annotations[i].SaveToFile(files[i]);
}

double CompareDistances(Annotation a1, Annotation a2){
	int s = (a1.objects.size() < a2.objects.size()) ? a1.objects.size() : a2.objects.size();
	double d = 0;
	for ( int i = 0; i < s; i++)
		d += pow((a1.objects[i].bndbox.distance - a2.objects[i].bndbox.distance), 2);
	return sqrt(d/s);
}

double CompareCoordinates(Annotation a1, Annotation a2){
	return sqrt(pow(a1.position.x - a2.position.x, 2) + pow(a1.position.y - a2.position.y, 2) + pow(a1.position.z - a2.position.z, 2));
}

double CompareTrajectories(std::vector<Annotation> ann1, std::vector<Annotation> ann2){
	int s = (ann1.size() < ann2.size()) ? ann1.size() : ann2.size();
	double len = 0;
	double dist = CompareCoordinates(ann1[0], ann2[0]);
	for (int i = 1; i < s; i++){
        len += (CompareCoordinates(ann1[i], ann1[i-1])+CompareCoordinates(ann2[i], ann2[i-1]))/2;
		dist += CompareCoordinates(ann1[i], ann2[i]);
		//std::cout<<dist<<std::endl<<len<<std::endl<<std::endl;
	}
	if(len == 0) return 0;
	return dist/(s*len);
}
