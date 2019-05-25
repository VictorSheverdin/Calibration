#include <vector>
#include <string>
#include <iostream>
#include "src/annotations/annotation.h"

#include <QDir>

int main ( int argc, char** argv )
{
	if (argc != 3){
		std::cout<<"Run the program as following: comparedirs path_gt path_data\n";
		return 1;
	}

    QDir dir1( argv[1] );
    auto files1 = dir1.entryList(QStringList() << "*.xml" , QDir::Files);

    QDir dir2( argv[2] );
    auto files2 = dir2.entryList(QStringList() << "*.xml" , QDir::Files);

	std::vector<Annotation> ann1, ann2;

    for( auto& i: files1 ){
        auto fileName = dir1.absoluteFilePath( i );
        ann1.push_back( Annotation( fileName.toStdString() ) );
	}

    for( auto& i: files2 ){
        auto fileName = dir2.absoluteFilePath( i );
        ann2.push_back(Annotation( fileName.toStdString() ) );
    }

	std::cout << CompareTrajectories(ann1, ann2) << std::endl;

	return 0;

}
