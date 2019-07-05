#include <vector>
#include <string>

struct Position{
	double x;
	double y;
	double z;
};

struct Size{
	uint32_t width;
	uint32_t height;
	uint32_t depth = 3;
};

struct BoundingBox{
	uint32_t xmin;
	uint32_t ymin;
	uint32_t xmax;
	uint32_t ymax;
	double distance;
};

struct AnnotatedObject{
	std::string name;
	BoundingBox bndbox;
};

class Annotation
{
public:
	std::string filename;
	Size size;
	Position position;
	std::vector<AnnotatedObject> objects;
	
	int LoadFromFile(std::string file);
	int SaveToFile(std::string file);
	
	Annotation(){}
	Annotation(std::string file){ LoadFromFile(file); }
};

std::vector<Annotation> LoadAnnotations(std::vector<std::string> files);
void SaveAnnotations(std::vector<Annotation> annotations, std::vector<std::string> files);
double CompareDistances(Annotation a1, Annotation a2);

double CompareCoordinates(Annotation a1, Annotation a2);
double CompareTrajectories(std::vector<Annotation> ann1, std::vector<Annotation> ann2); //ann1 - data, ann2 - ground truth