#include "GraphRender.h"

GraphRender::GraphRender(NavmeshGraph & root):
	_root(root)
{
	_canvas = Node::create();
	_canvas->retain();
}

GraphRender::~GraphRender()
{
	_canvas->release();
}

void GraphRender::init()
{
	_root.traversal([this](GraphNode* node) {
		auto poly = node->poly;
		this->drawPoly(poly);
		this->drawIndex(poly.centroid, node->idx);
	});
}

void GraphRender::drawLine(const Edge & e, Color4F color)
{
	auto render = DrawNode::create();
	render->drawLine(e.from, e.to, color);

	_canvas->addChild(render, 10);
}

void GraphRender::drawPoly(const ConvexPolygon & poly)
{
	auto lastPoint = poly.vertexArr.at(0);
	auto size = poly.vertexArr.size();
	for (int i = 1; i < size; ++i)
	{
		auto curPoint = poly.vertexArr.at(i);
		drawLine(Edge(lastPoint, curPoint));
		lastPoint = curPoint;
	}

	drawLine(Edge(lastPoint, poly.vertexArr.at(0)));
}

void GraphRender::drawSolidPoly(int idx)
{
	auto poly = _root.getNodeById(idx)->poly;

	auto render = DrawNode::create();

	int length = poly.vertexArr.size();
	Vec2* vertArr = new Vec2[length];
	for (int i = 0; i < length; ++i)
	{
		vertArr[i] = poly.vertexArr.at(i);
	}

	render->drawSolidPoly(vertArr, length, Color4F(0.13f, 0.1f, 0.6f, 0.5f));

	_canvas->addChild(render, 0);
}

void GraphRender::drawIndex(const Vec2 & centroid, int idx)
{
	auto label = Label::createWithSystemFont(StringUtils::format("%d", idx),
		"Microsoft YaHei",
		16);

	label->setPosition(centroid);
	label->setColor(Color3B::BLACK);

	_canvas->addChild(label,100);
}

void GraphRender::addToLayer(Layer * container,float scale)
{
	_canvas->setScale(scale);
	container->addChild(_canvas);
}
