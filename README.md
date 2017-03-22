# navmeshlib_yyh

----
## 简介

基于多边形的2D导航网格库，提供简单的编辑器。

图形界面使用cocos2d-x引擎，底层数学库也使用cocos2d-x的Vec2,Vec3,Matrix等文件，和cocos绝配，如需扩展至其他平台或引擎，请自行改写数学库内容。

导航网格的生成使用编辑器人工编辑，由于自动生成的三角形在寻路中效率低于正多边形，所以此处的网格数据需要通过编辑器生成，数据格式为json。

----
## 库文件使用说明
为了简便性考虑，采用直接使用源文件的形式，在使用时，请copy “Classes\Lib_Navmesh”下所有文件至你的文件目录，比如cocos2d-x项目的Classes目录。

**库代码示例：**

``` cpp
	bool TestLayer::init()
{
	if (!Layer::init())
		return false;

	auto bg = LayerColor::create(Color4B::WHITE);
	this->addChild(bg, -10);

	//test navmesh graph
	NavmeshGraph nvmgh;

	nvmgh.loadFromFile("navmesh.json");

	// 画图
	GraphRender render(nvmgh);
	render.init();
	render.addToLayer(this, 1.0f);

	// 寻路
	AStarSearch pathFinder(nvmgh);

	Vec2 begin(10, 10);
	Vec2 end(900,900);

	// test
	bool  tag = nvmgh.getNodeById(23)->poly.containsPoint(begin);

	auto node1 = DrawNode::create();
	node1->drawCircle(begin, 2.0f, 360, 30, false, Color4F::MAGENTA);
	node1->drawCircle(end, 2.0f, 360, 30, false, Color4F::GREEN);
	this->addChild(node1, 10);

	auto nodes = pathFinder.result(begin, end);

	//画相交线
	for (int i = 1; i < nodes.size(); ++i)
	{
		auto poly1 = nvmgh.getNodeById(nodes[i - 1])->poly;
		auto poly2 = nvmgh.getNodeById(nodes[i])->poly;
		render.drawLine(poly1.findCommonEdge(poly2), Color4F::YELLOW);
	}


	//画路径网格
	CCLOG("%s","route:");
	for (auto iter = nodes.cbegin(); iter != nodes.cend(); ++iter)
	{
		CCLOG("node is: %d", *iter);
		render.drawSolidPoly(*iter);
	}

	//画最终路径
	auto path = pathFinder.tp_path(begin, end);
	CCLOG("Way Point %d: %f, %f", 1, path[0].x, path[0].y);
	for (int i = 1; i < path.size(); ++i)
	{
		render.drawLine(Edge(path[i], path[i - 1]), Color4F::GREEN);
		CCLOG("Way Point %d: %f, %f", i + 1, path[i].x, path[i].y);
	}


	return true;
}
```

**用例和API说明**

提供使用的主要文件为：
``` cpp
#include "Lib_Navmesh\FileNavmeshParser.h"
#include "Lib_Navmesh\GraphRender.h"
#include "Lib_Navmesh\AStarSearch.h"
```
- 第一个文件是数据和json的交互文件，支持重载

- 第二个文件是导航网格的渲染

- 第三个文件是A star的寻路

具体API请参照test用例

----

## 编辑器使用指南

推荐按照此流程使用编辑器：
- 使用其他工具作出底图，底图不应超过1000*1000px
- 底图替换“navmesh_test.png”文件
- 编译并打开编辑器，对每个多边形进行描述，逆时针点击一个多边形每个点直到点击到第一个点。（如果画错了，按k键撤销一个点）
- 画完所有多边形，不要画不可访问的障碍物多边形。
- 按s键保存

**注意**
- 由于编辑器功能还在完善中，所以暂时不支持输入障碍多边形自动生成导航网格。
- 多变形面积不应过大，这样会使 A star 结果不准确。
- 多边形必须确保为凸多边形，目前为止若不为凸多边形，会触发断言。
