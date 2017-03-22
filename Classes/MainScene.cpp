#include "MainScene.h"

// show editor
bool MainScene::init()
{
	if(!Layer::init())
		return false;

	_vertsAndEdges = Sprite::create();
	this->addChild(_vertsAndEdges, 10);

	addStaticStyle();

	addTouchReaction();

	return true;
}

// file write read test
//bool MainScene::init()
//{
//	if (!Layer::init())
//		return false;
//
//	NavmeshGraph ng;
//	ng.loadFromFile("navmesh.json");
//	
//	ng.saveTofile("navmesh.json");
//
//	return true;
//}


Scene* MainScene::createScene()
{
	auto scene = Scene::create();
	scene->addChild(MainScene::create());
	return scene;
}

void MainScene::addStaticStyle()
{
	const int bg_z_order = -1;
	auto bg = LayerColor::create(Color4B::WHITE);
	this->addChild(bg, bg_z_order);

	auto sp = Sprite::create("navmesh_test.png");
	sp->setAnchorPoint(Vec2::ZERO);
	sp->setScale(0.5f);
	
	this->addChild(sp, 0);
}

void MainScene::addTouchReaction()
{
	auto eventListener = EventListenerTouchOneByOne::create();
	
	eventListener->onTouchBegan = [this](Touch* touch,Event* e)->bool {
		auto pos = touch->getLocation();

		if (!this->isAlreadyExist(pos))
		{
			this->handleCloseAxis(pos);
			this->drawPoint(pos);
		}

		this->constructPoly(pos);
		
		return true;
	};

	this->getEventDispatcher()->addEventListenerWithSceneGraphPriority(eventListener, this);

	
	auto keyListener = EventListenerKeyboard::create();

	keyListener->onKeyPressed = [this](EventKeyboard::KeyCode code, Event* e) {
		//撤销功能
		if (code == EventKeyboard::KeyCode::KEY_K)
		{
			cancel();
		}

		//保存功能
		if (code == EventKeyboard::KeyCode::KEY_S)
		{
			save();
		}
	};

	this->getEventDispatcher()->addEventListenerWithSceneGraphPriority(keyListener, this);
}

void MainScene::drawPoint(const Vec2& point)
{
	const float r = 2.0f;
	const float point_z_order = 10;

	auto node = DrawNode::create();
	node->drawCircle(point, 2.0f, 360, 30, false, Color4F::MAGENTA);
	
	Vec2_Node nodeWithPos;
	nodeWithPos._key = point;
	nodeWithPos._value = node;
	_nodeMap.push_back(nodeWithPos);

	_vertsAndEdges->addChild(node, 10);
}

void MainScene::drawPoly(const ConvexPolygon& poly)
{
	auto lastPoint = poly.vertexArr.at(0);
	auto size = poly.vertexArr.size();
	for (int i = 1; i < size; ++i)
	{
		auto curPoint = poly.vertexArr.at(i);
		drawLine(lastPoint, curPoint);
		lastPoint = curPoint;
	}

	drawLine(lastPoint, poly.vertexArr.at(0));
}

void MainScene::drawLine(const Vec2 & from, const Vec2 & to)
{
	auto render = DrawNode::create();
	render->drawLine(from, to, Color4F(0.76f, 0.45f, 0.0f, 1));

	_vertsAndEdges->addChild(render, 9);
}

void MainScene::drawSolidPoly(const ConvexPolygon & poly)
{
	auto render = DrawNode::create();

	int length = poly.vertexArr.size();
	Vec2* vertArr = new Vec2[length];
	for (int i = 0; i < length; ++i)
	{
		vertArr[i] = poly.vertexArr.at(i);
	}

	render->drawSolidPoly(vertArr, length, Color4F(0.13f, 0.1f, 0.6f, 1));
	
	_vertsAndEdges->addChild(render, 5);
	_polysInView.push_back(render);
}

void MainScene::save()
{
	NavmeshGraph ng;

	for (int i = 0; i < _polys.size(); ++i)
	{
		vector<int> siblings;
		for (int j = 0; j < _polys.size(); ++j)
		{
			if (i != j)
			{
				if (hasCommonEdge(_polys[i], _polys[j]))
				{
					// 我们认为id从1开始
					siblings.push_back(j + 1);
				}
			}
		}

		GraphNode* node = new GraphNode(i + 1, _polys[i], sea, siblings);
		ng.addPoly(node);
	}

	ng.saveTofile("navmesh.json");
}

bool MainScene::isAlreadyExist(Vec2 & point)
{
	for (auto iter = _nodeMap.begin(); iter != _nodeMap.end(); ++iter)
	{
		if (iter->_key.getDistanceSq(point) < presion*presion)
		{
			point = iter->_key;
			return true;
		}
	}
}

void MainScene::handleCloseAxis(Vec2& point)
{
	point.x = fabs(point.x - width) < presion ? width : point.x;
	point.x = fabs(point.x - 0) < presion ? 0 : point.x;

	point.y = fabs(point.y - height) < presion ? height : point.y;
	point.y = fabs(point.y - 0) < presion ? 0 : point.y;
}

bool MainScene::constructPoly(const Vec2 & point)
{
	if (_currentVerts.size() != 0)
	{
		if (point.distanceSquared(_currentVerts.at(0)) < presion*presion)
		{
			CCLOG("Poly: ");
			for (auto iter = _currentVerts.begin(); iter != _currentVerts.end(); ++iter)
			{
				CCLOG("Verts: %f, %f", iter->x, iter->y);
			}

			ConvexPolygon poly(_currentVerts);
			_polys.push_back(poly);
			drawPoly(poly);
			drawSolidPoly(poly);

			_currentVerts.clear();
			return true;
		}
	}

	_currentVerts.push_back(point);
	return false;
}

void MainScene::cancel()
{
	if (!this->_currentVerts.empty())
	{
		this->_currentVerts.pop_back();

		auto sp = this->_nodeMap.at(this->_nodeMap.size() - 1);
		this->_nodeMap.pop_back();

		this->_vertsAndEdges->removeChild(sp._value, true);
	}
	else
	{
		auto poly = _polysInView.back();
		_polysInView.pop_back();

		this->_vertsAndEdges->removeChild(poly, true);

		_polys.pop_back();
	}
}

bool MainScene::hasCommonEdge(const ConvexPolygon & one, const ConvexPolygon & other)
{
	int commonVertCount = 0;
	for (int i = 0; i < one.vertexArr.size(); ++i)
	{
		for (int j = 0; j < other.vertexArr.size(); ++j)
		{
			if (one.vertexArr[i].distanceSquared(other.vertexArr[j]) < 1)
			{
				commonVertCount++;
				if (commonVertCount >= 2)
					return true;
			}
		}
	}

	return false;
}

