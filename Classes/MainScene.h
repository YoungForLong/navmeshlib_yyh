#ifndef _MAIN_SCENE_H_
#define _MAIN_SCENE_H_

#include "Lib_Navmesh\NavmeshGraph.h"
#include "cocos2d.h"

using namespace recast_navigation;
using namespace std;
USING_NS_CC;

struct Vec2_Node {
	Vec2 _key;
	Node* _value;
};

class MainScene :public Layer
{
public:
	const float presion = 10.0f;

	const float width = 2000;
	const float height = 2000;

public:

	virtual bool init();

	static Scene* createScene();

	void addStaticStyle();

	void addTouchReaction();

	void drawPoint(const Vec2& point);

	void drawPoly(const ConvexPolygon& poly);
	void drawLine(const Vec2& from, const Vec2& to);
	void drawSolidPoly(const ConvexPolygon& poly);

	void save();
	/**
	* @param point ref point to output
	*/
	bool isAlreadyExist(Vec2& point);

	/**
	* @param point ref point to output
	*/
	void handleCloseAxis(Vec2& point);

	bool constructPoly(const Vec2& point);

	void cancel();

	bool hasCommonEdge(const ConvexPolygon& one, const ConvexPolygon& other);

	CREATE_FUNC(MainScene);

private:
	vector<Vec2> _currentVerts;
	vector<ConvexPolygon> _polys;

	vector<Vec2_Node> _nodeMap;

	vector<Node*> _polysInView;

	Sprite* _vertsAndEdges;
};


#endif
