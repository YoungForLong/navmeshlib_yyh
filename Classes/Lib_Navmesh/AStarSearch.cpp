#include "AStarSearch.h"


recast_navigation::AStarSearch::AStarSearch(NavmeshGraph& source):
	_sourceMap(source),
	_route(source.size() + 1,0)// 0号位不用
{
}

recast_navigation::AStarSearch::~AStarSearch()
{
}

vector<int> recast_navigation::AStarSearch::result(const Vec2 & start, const Vec2 & end)
{
	_start = start;
	_end = end;
	search();

	stack<int> path;
	int parent = _endNode;
	path.push(parent);
	while (parent != _startNode)
	{
		parent = _route[parent];
		path.push(parent);
	}

	// 反序列化
	vector<int> ret;

	while (!path.empty())
	{
		int idx = path.top();
		path.pop();

		ret.push_back(idx);
	}

	return ret;
}

vector<Vec2> recast_navigation::AStarSearch::LOS_path(const Vec2 & start, const Vec2 & end)
{
	auto nodes = result(start, end);

	//找到所有公共边，亦即穿出边
	vector<Edge> allCommonEdges;
	for (int i = 1; i < nodes.size(); ++i)
	{
		auto poly1 = _sourceMap.getNodeById(nodes[i - 1])->poly;
		auto poly2 = _sourceMap.getNodeById(nodes[i])->poly;
		allCommonEdges.push_back(poly1.findCommonEdge(poly2));
	}

	vector<Vec2> ret;
	ret.push_back(start);
	
	for (int i = 0; i < allCommonEdges.size(); ++i)
	{
		//视线向量
		Vec2 n = end - ret.back();

		auto tempE = allCommonEdges[i];
		if (tempE.containsPoint(ret.back()))
			continue;

		if (NavmeshGraph::LOS_test(n, tempE))
		{
			Vec2 tunrningPoint = (tempE.from.distanceSquared(end) < tempE.to.distanceSquared(end)) ? tempE.from : tempE.to;
			ret.push_back(tunrningPoint);
		}
	}

	ret.push_back(end);

	return ret;
}

vector<Vec2> recast_navigation::AStarSearch::tp_path(const Vec2 & start, const Vec2 & end)
{
	auto nodes = result(start, end);

	//找到所有公共边，亦即穿出边
	vector<Edge> allCommonEdges;
	for (int i = 1; i < nodes.size(); ++i)
	{
		auto poly1 = _sourceMap.getNodeById(nodes[i - 1])->poly;
		auto poly2 = _sourceMap.getNodeById(nodes[i])->poly;
		allCommonEdges.push_back(poly1.findCommonEdge(poly2));
	}

	vector<Vec2> ret;
	ret.push_back(start);

	bool isNewTp = true;
	Vec2 convergenceN[2] = { Vec2::ZERO,Vec2::ZERO };

	int length = allCommonEdges.size();
	int leftId = 0;
	int rightId = 0;
	for (int i = 0; i < length; ++i)
	{
		auto curStart = ret.back();

		if (isNewTp)
		{
			// 收敛的夹角，两个向量，0为left，1为right
			convergenceN[0] = allCommonEdges[i].from - curStart; 
			convergenceN[1] = allCommonEdges[i].to - curStart;

			if (left_or_right(convergenceN[0], convergenceN[1]) == 1)
			{
				// swap
				auto temp = convergenceN[0];
				convergenceN[0] = convergenceN[1];
				convergenceN[1] = temp;
			}

			leftId = i;
			rightId = i;

			isNewTp = false;
			continue;
		}

		Vec2 curTwo[2] = { allCommonEdges[i].from - curStart,allCommonEdges[i].to - curStart };
		if (left_or_right(curTwo[0], curTwo[1]) == 1)
		{
			// swap
			auto temp = curTwo[0];
			curTwo[0] = curTwo[1];
			curTwo[1] = temp;
		}

		//判断现在的两个新向量是否在以前向量之间
		int curL_on_conL = left_or_right(curTwo[0], convergenceN[0]);
		int curL_on_conR = left_or_right(curTwo[0], convergenceN[1]);
		int curR_on_conL = left_or_right(curTwo[1], convergenceN[0]);
		int curR_on_conR = left_or_right(curTwo[1], convergenceN[1]);

		if (curL_on_conL == 0 && curR_on_conR == 1)// 在夹角外,包含
			continue;
		else if (curR_on_conL == 0) // 在夹角外，左边
		{
			// 收敛夹角的左点为拐点
			auto turningPoint = convergenceN[0] + curStart;
			ret.push_back(turningPoint);
			isNewTp = true;
			i = leftId;
		}
		else if (curL_on_conR == 1) // 在夹角外，右边
		{
			// 右点为拐点
			auto turningPoint = convergenceN[1] + curStart;
			ret.push_back(turningPoint);
			isNewTp = true;
			i = rightId;
		}
		else // 在夹角内
		{
			if (curL_on_conL == 1)
			{
				convergenceN[0] = curTwo[0];
				leftId = i;
			}
			
			if (curR_on_conR == 0)
			{
				convergenceN[1] = curTwo[1];
				rightId = i;
			}
		}
	}

	// 处理最后一个点
	auto curTp = ret.back();
	auto endN = end - curTp;
	int end_on_conL = left_or_right(endN, convergenceN[0]);
	int end_on_conR = left_or_right(endN, convergenceN[1]);

	if (end_on_conL == 0)
	{
		ret.push_back(convergenceN[0] + curTp);
	}
	else if (end_on_conR == 1)
	{
		ret.push_back(convergenceN[1] + curTp);
	}

	ret.push_back(end);

	return ret;
}

int recast_navigation::AStarSearch::left_or_right(const Vec2 & origin, const Vec2 & target)
{
	if (origin.getDistanceSq(target) < 1)
		return 3;

	Vec3 o3d(origin.x, origin.y, 0);
	Vec3 t3d(target.x, target.y, 0);

	Vec3 result(Vec3::ZERO);

	Vec3::cross(o3d, t3d, &result);

	if (result.z > 0)
		return 1;
	else
		return 0;
}

bool recast_navigation::AStarSearch::search()
{
	// 储存未被存入树中，但已经被访问过的节点
	priority_queue<GraphNode*,vector<GraphNode*>,cmp> OPEN;

	// map的id从1开始
	const int mapSize = _sourceMap.size() + 1;

	// 是否已经被访问且放入树中
	vector<bool> CLOSE(mapSize, false);
	
	// 是否还在OPEN表中
	vector<bool> isInOpen(mapSize, false);

	// 源节点入队
	int startId = _sourceMap.pointInWhichPoly(_start);
	_startNode = startId;
	OPEN.push(_sourceMap.getNodeById(startId));
	isInOpen.at(startId) = true;
	//_FValueArr.at(startId) = 0.0f;

	while (!OPEN.empty())
	{
		//F值最小的出队列
		auto curNode = OPEN.top();
		OPEN.pop();
		CLOSE.at(curNode->idx) = true;//所有出队列的节点，标记为CLOSE
		isInOpen.at(curNode->idx) = false;

		//找到目标节点，退出循环
		if (curNode->poly.containsPoint(_end))
		{
			_endNode = curNode->idx;
			return true;
		}
		
		//把相邻节点全部入队，前提是不在OPEN表中，不在CLOSE表中（未加入树）
		for (int i = 0; i < curNode->siblings.size(); ++i)
		{
			auto next = _sourceMap.getNodeById(curNode->siblings.at(i));

			if (CLOSE[next->idx])
				continue;

			if (isInOpen[next->idx] == false)
			{
				// 储存路径树的父节点id
				_route[next->idx] = curNode->idx;
				
				// 重新计算FValue
				next->FValue = F_func(curNode, next);
				next->FValue += curNode->FValue;

				// 更新OPEN表
				OPEN.push(next);
				isInOpen.at(next->idx) = true;
			}
			else // 如果在OPEN表中，需要重新计算，比较是上一个父节点迭加小还是现在这个节点迭加起来小
			{
				float curFValue = next->FValue;
				float newFValue = F_func(curNode, next);
				
				if (newFValue < curFValue)
				{
					// 更换父节点和FValue
					_route[next->idx] = curNode->idx;
					next->FValue = newFValue;
				}
			}
		}// end for

	}// end while

	return false;
}

float recast_navigation::AStarSearch::F_func(const GraphNode* last,const GraphNode* cur)
{
	return H_func(last->poly.centroid, cur->poly.centroid)
		+ G_func(cur->poly.centroid);
}

float recast_navigation::AStarSearch::H_func(const Vec2 & last, const Vec2 & cur)
{
	return last.distanceSquared(cur);
}

float recast_navigation::AStarSearch::G_func(const Vec2 & centroid)
{
	return centroid.distanceSquared(_end);
}
