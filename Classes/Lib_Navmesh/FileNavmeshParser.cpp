#include "FileNavmeshParser.h"
#include "json/stringbuffer.h"
#include "json/writer.h"

recast_navigation::FileNavmeshParser::FileNavmeshParser(NavmeshGraph& root):
	_root(root)
{

}

recast_navigation::FileNavmeshParser::~FileNavmeshParser()
{
	
}

void recast_navigation::FileNavmeshParser::parse(const string & filename)
{
	ssize_t size = 0;
	auto data = FileUtils::getInstance()->getFileData(filename, "r", &size);
	if (size == 0)
	{
		CCLOG("File not found: %s", filename.c_str());
	}

	rapidjson::Document doc;

	string dataStr((const char*)data, size);
	delete[] data;
	data = nullptr;

	doc.Parse<0>(dataStr.c_str());

	if (doc.HasParseError())
	{
		auto error = doc.GetParseError();
		CCLOG("rapidxml error: %d", error);
		return;
	}

	if (!doc.HasMember("navmesh_graph"))
		return;

	// 存放多边形的数组
	auto& graph = doc["navmesh_graph"];
		
	if (!graph.IsArray())
		return;

	for (rapidjson::SizeType i = 0; i < graph.Size(); ++i)
	{
		vector<Vec2> verts;
		int index;
		vector<int> siblings;

		// 多边形
		auto& poly = graph[i];

		// id
		if (poly.HasMember("index"))
		{
			index = poly["index"].GetInt();
		}
		
		// 点集
		if (poly.HasMember("verts"))
		{
			auto& vertsObj = poly["verts"];
			for (rapidjson::SizeType j = 0; j < vertsObj.Size(); ++j)
			{
				float x = vertsObj[j]["x"].GetDouble();
				float y = vertsObj[j]["y"].GetDouble();
				verts.push_back(Vec2(x, y));
			}
		}
		
		// 邻居集合
		if (poly.HasMember("siblings"))
		{
			auto& siblingsObj = poly["siblings"];
			for (rapidjson::SizeType k = 0; k < siblingsObj.Size(); ++k)
			{
				auto& bro = siblingsObj[k];
				siblings.push_back(bro.GetInt());
			}
		}
		

		ConvexPolygon convexPolygon(verts);

		auto pNode = new GraphNode(index, convexPolygon, PolyType::sea, siblings);
		_root.addPoly(pNode);
	}
}

void recast_navigation::FileNavmeshParser::saveToFile(const string & filename)
{
	rapidjson::Document doc;
	doc.SetObject();
	auto& allocator = doc.GetAllocator();
	
	rapidjson::Value polyArr(rapidjson::kArrayType);

	_root.traversal([&allocator,&polyArr](GraphNode* node) {
		// poly
		rapidjson::Value polyObj(rapidjson::kObjectType);
		polyObj.AddMember("index", node->idx, allocator);
		
		// verts
		rapidjson::Value vertArr(rapidjson::kArrayType);
		for (auto i = 0; i < node->poly.vertexArr.size(); ++i)
		{
			auto point = node->poly.vertexArr.at(i);
			rapidjson::Value vertObj(rapidjson::kObjectType);
			vertObj.AddMember("x", point.x, allocator);
			vertObj.AddMember("y", point.y, allocator);

			vertArr.PushBack(vertObj, allocator);
		}
		polyObj.AddMember("verts", vertArr, allocator);
		
		// siblings
		rapidjson::Value siblingArr(rapidjson::kArrayType);
		for (auto i = 0; i < node->siblings.size(); ++i)
		{
			siblingArr.PushBack(node->siblings.at(i), allocator);
		}
		polyObj.AddMember("siblings", siblingArr, allocator);
		// end poly

		//add to graph
		polyArr.PushBack(polyObj, allocator);
	});

	doc.AddMember("navmesh_graph", polyArr, allocator);

	rapidjson::StringBuffer buf;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
	doc.Accept(writer);

	string saveStr(buf.GetString(), buf.GetSize());

	auto fullpath = FileUtils::getInstance()->getWritablePath() + filename;

	FileUtils::getInstance()->writeStringToFile(saveStr, fullpath);
}
