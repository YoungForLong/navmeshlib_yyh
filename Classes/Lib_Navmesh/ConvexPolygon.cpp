#include "ConvexPolygon.h"

using namespace recast_navigation;

const Edge Edge::NIL(Vec2::ZERO, Vec2::ZERO);

recast_navigation::ConvexPolygon::ConvexPolygon(Vec2 arg, ...)
{
	va_list argArr;

	Vec2 temp = arg;

	va_start(argArr, arg);

	while (temp != nullptr)
	{
		temp = va_arg(argArr, Vec2);
		vertexArr.push_back(temp);
	}

	va_end(argArr);

	if (!checkConvex())
	{
		vertexArr.clear();
		CCLOG("%s","it's not a convex poly");
	}

	calculateCentroid();
}

recast_navigation::ConvexPolygon::ConvexPolygon(const initializer_list<Vec2>& argList)
{
	for (auto iter = argList.begin(); iter != argList.end(); ++iter)
	{
		vertexArr.push_back(*iter);
	}

	if (!checkConvex())
	{
		vertexArr.clear();
		assert(0 &&
			"it's not a convex poly");
	}

	calculateCentroid();
}

recast_navigation::ConvexPolygon::ConvexPolygon(const vector<Vec2>& arr)
{
	for (auto iter = arr.begin(); iter != arr.end(); ++iter)
	{
		vertexArr.push_back(*iter);
	}

	if (!checkConvex())
	{
		vertexArr.clear();
		assert(0 &&
			"it's not a convex poly");
	}

	calculateCentroid();
}

std::vector<Triangle> recast_navigation::ConvexPolygon::divide()
{
	vector<Triangle> ret;

	auto iter = vertexArr.cbegin();
	const Vec2 first_vertex = *iter;
	
	// turn to the next
	iter++;

	while (true)
	{
		Triangle temp;
		temp.vertexes[0] = first_vertex;
		temp.vertexes[1] = *(iter++);

		if (iter == vertexArr.cend())
			break;
		temp.vertexes[2] = *iter;

		ret.push_back(temp);
	}

	return ret;
}

bool recast_navigation::ConvexPolygon::containsPoint(const Vec2 & p) const
{
	auto length = vertexArr.size();

	//第一次检测
	bool lastTest = tri_point_z(p, vertexArr[0], vertexArr[1]);
	
	for (int i = 1; i < length - 1; ++i)
	{
		bool curTest = tri_point_z(p, vertexArr[i], vertexArr[i + 1]);
		if (curTest != lastTest)
			return false;
		lastTest = curTest;
	}
	
	//最后一次检测
	bool curTest = tri_point_z(p, vertexArr[length - 1], vertexArr[0]);
	if (curTest != lastTest)
		return false;

	return true;
}

vector<Edge> recast_navigation::ConvexPolygon::getAllEdges() const
{
	vector<Edge> edgeArr;
	
	auto curVert = vertexArr.at(0);

	for (int i = 1; i < vertexArr.size(); ++i)
	{
		Edge e(curVert, vertexArr.at(i));

		edgeArr.push_back(e);
	}

	Edge last = { vertexArr.back(),vertexArr.front() };
	edgeArr.push_back(last);

	return edgeArr;
}

Edge recast_navigation::ConvexPolygon::findCommonEdge(const ConvexPolygon& other) const
{
	vector<Vec2> e;
	for (int i = 0; i < vertexArr.size(); ++i)
	{
		for (int j = 0; j < other.vertexArr.size(); ++j)
		{
			if (vertexArr[i].distanceSquared(other.vertexArr[j]) < 1)
			{
				e.push_back(vertexArr[i]);
			}
		}
	}

	assert(e.size() == 2
		&& "error: no common edge");

	return Edge(e[0], e[1]);
}

bool recast_navigation::ConvexPolygon::checkConvex()
{
	if (vertexArr.size() < 3)
		return false;

	if (vertexArr.size() == 3)
		return true;

	bool tagZ = tagZ = tri_point_z(vertexArr[0], vertexArr[1], vertexArr[2]);
	for (int i = 3; i < vertexArr.size(); ++i)
	{
		auto tag = tri_point_z(vertexArr[i - 2], vertexArr[i - 1], vertexArr[i]);
		if (tag != tagZ)
		{
			return false;
		}
	}

	//倒数第二个点和最后一个点
	auto last = vertexArr[vertexArr.size() - 1];
	auto lastbutone = vertexArr[vertexArr.size() - 2];

	bool tag = tri_point_z(lastbutone, last, vertexArr[0]);
	if (tag != tagZ)
		return false;

	tag = tri_point_z(last, vertexArr[0], vertexArr[1]);
	if (tag != tagZ)
		return false;

	return true;
}

//float recast_navigation::ConvexPolygon::tri_point_to_angle(const Vec2 & p1, const Vec2 & p2, const Vec2 & p3)
//{
//	Vec2 v1 = p1 - p2;
//	Vec2 v2 = p3 - p2;
//
//	float cos_angle = v1.cross(v2) / (v1.length()*v2.length());
//
//	return cos_angle;
//}

bool recast_navigation::ConvexPolygon::tri_point_z(const Vec2 & p1, const Vec2 & p2, const Vec2 & p3) const
{
	Vec3 ret;
	auto e1 = Vec2(p2 - p1);
	auto e2 = Vec2(p3 - p2);
	Vec3::cross(Vec3(e1.x, e1.y, 0), Vec3(e2.x, e2.y, 0), &ret);

	return ret.z > 0;
}

void recast_navigation::ConvexPolygon::calculateCentroid()
{
	// 质心等于分解质心的加权平均

	Vec2 sumWeight = Vec2::ZERO;
	float sumA = 0.0f;

	auto tris = this->divide();

	for (auto iter = tris.begin(); iter != tris.end(); ++iter)
	{
		Vec2 eachC = (*iter).centroid();
		float eachArea = (*iter).area();

		sumA += eachArea;
		sumWeight += eachC*eachArea;
	}

	centroid = sumWeight / sumA;
}

