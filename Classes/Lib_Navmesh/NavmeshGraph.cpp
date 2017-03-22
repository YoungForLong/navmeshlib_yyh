#include "NavmeshGraph.h"
#include <map>
#include <queue>
#include "FileNavmeshParser.h"

recast_navigation::NavmeshGraph::NavmeshGraph() :
	_size(0)
{
}

recast_navigation::NavmeshGraph::~NavmeshGraph()
{
	traversal([](GraphNode* node) {
		delete node;
		node = nullptr;
	});
}

bool recast_navigation::NavmeshGraph::loadFromFile(const string & filename)
{
	FileNavmeshParser parser(*this);

	parser.parse(filename);

	return true;
}

void recast_navigation::NavmeshGraph::saveTofile(const string & filename)
{
	FileNavmeshParser saver(*this);
	saver.saveToFile(filename);
}

vector<Vec2> recast_navigation::NavmeshGraph::AStarSearch(Vec2 start, Vec2 end)
{
	return vector<Vec2>();
}

int recast_navigation::NavmeshGraph::pointInWhichPoly(const Vec2 & point)
{
	int ret = 0;
	traversal([&ret,point](GraphNode* node) {
		if (node->poly.containsPoint(point))
		{
			ret = node->idx;
			return;
		}
	});

	if (ret != 0)
	{
		return ret;
	}
	else
	{
		CCLOG("cannot find this point in map, it's likely that inside the obstacle set, point£º %f, %f", point.x, point.y);
		return 0;
	}
}

bool recast_navigation::NavmeshGraph::LOS_test(const Vec2 heading, Edge e)
{
	Vec3 h3d = Vec3(heading.x, heading.y, 0);
	Vec3 f3d = Vec3(e.from.x, e.from.y, 0);
	Vec3 t3d = Vec3(e.to.x, e.to.y, 0);

	Vec3 test_a = Vec3::ZERO;
	Vec3 test_b = Vec3::ZERO;
	
	Vec3::cross(f3d, h3d, &test_a);
	Vec3::cross(t3d, h3d, &test_b);

	bool tag_a = test_a.z > 0;
	bool tag_b = test_b.z > 0;

	return tag_a == tag_b;
}

void recast_navigation::NavmeshGraph::traversal(function<void(GraphNode*)> func) const
{
	for (auto iter = _nodeMap.cbegin(); iter != _nodeMap.cend(); ++iter)
	{
		func(iter->second);
	}
}

void recast_navigation::NavmeshGraph::addPoly(GraphNode* node)
{
	_nodeMap.insert(make_pair(node->idx,node));
	_size++;
}

void recast_navigation::NavmeshGraph::sliceEdge(Vec2 from, Vec2 to)
{
}

void recast_navigation::NavmeshGraph::combineEdge(Vec2 from, Vec2 to)
{
}
