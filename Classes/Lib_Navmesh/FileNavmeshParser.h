#ifndef _FILE_NAVMESH_PARSER_H_
#define _FILE_NAVMESH_PARSER_H_

#include <cocos2d.h>
#include "NavmeshGraph.h"
#include "cocostudio\CocoStudio.h"

using namespace std;
USING_NS_CC;

namespace recast_navigation {

	class FileNavmeshParser
	{
	public:

		FileNavmeshParser(NavmeshGraph& root);
		~FileNavmeshParser();

		void parse(const string& filename);

		void saveToFile(const string& filename);
	protected:

	private:
		NavmeshGraph& _root;
	};

	typedef FileNavmeshParser FNP;
}


#endif
