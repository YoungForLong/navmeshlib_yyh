#ifndef _A_STAR_SEARCH_H_
#define _A_STAR_SEARCH_H_

#include "NavmeshGraph.h"

namespace recast_navigation {

	const float infinity_float = 3.402823466e+38F;

	//functor
	struct cmp {
		bool operator()(GraphNode* one, GraphNode* another)
		{
			return one->FValue > another->FValue;
		}
	};

	class AStarSearch
	{
	public:
		AStarSearch(NavmeshGraph& source);
		~AStarSearch();

		vector<int> result(const Vec2& start, const Vec2& end);

		// �������߷���·��
		vector<Vec2> LOS_path(const Vec2& start, const Vec2& end);

		// turning point path, �յ㷨��·��
		vector<Vec2> tp_path(const Vec2& start, const Vec2& end);

		/**
		* @return 0 means on target's left, 1 means right, 3 maens equal
		*/
		static int left_or_right(const Vec2& origin, const Vec2& target);

	protected:
		bool search();

		// �˴�Ϊ�˼�����������ʹ��ƽ��
		float F_func(const GraphNode* last, const GraphNode* cur);

		// H = dist(last, cur)
		float H_func(const Vec2& last, const Vec2& cur);
		
		// G = dist(centroid, end)
		float G_func(const Vec2& centroid);

	private:
		Vec2 _start;
		Vec2 _end;
		
		// �������·���ĳ�ʼ�ڵ��Ŀ�Ľڵ㣬�����ظ�����
		int _startNode;
		int _endNode;

		NavmeshGraph& _sourceMap;
		vector<int> _route; // ��ŵ������·��������������Ҷ�ӵĸ��ڵ�
	};
}

#endif
