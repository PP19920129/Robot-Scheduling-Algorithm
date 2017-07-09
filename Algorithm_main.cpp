#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <queue>
#include <algorithm> 
#include <string>
#include <iostream>
#include <fstream>
using namespace std;


//global.h
/***********************与地图有关的参数***************************/
#define XMAX 100  //行数，也就是y
#define YMAX 100  //列数，也就是x
int XSIZE = 0;
int YSIZE = 0;
#define INF  200000000
#define  N  10000  //车辆数目
/*****************************************************************/



/***********************与地图上每个点有关的参数***************************/
const char STARTNODE = 'I';
const char ENDNODE = 'E';
const char BARRIER = 'B';
const char PARK = 'P';
const char ROAD = 'X';
const char PARKCARIN = 'H';//hold
typedef struct AStarNode
{
	int s_x;                            // 坐标(最终输出路径需要)
	int s_y;
	int s_to_start;                     //此点到起点的距离，只计算一次，数据保留
	int s_to_end;                       //此点到终点的距离，只计算一次，数据保留
	int s_to_P1;                        //此点到某个具体点的距离，每计算一个点，先重置数据
	char s_style;                       //结点类型：起始点，终点，障碍物，空车位，有车车位
	struct AStarNode * s_parent;               //父节点
	struct AStarNode * s_parent_to_end;
	struct AStarNode * s_parent_to_P1;
	int s_is_in_closetable;             //是否在close表中
	int s_is_in_closetable_to_end;
	int s_is_in_closetable_to_P1;
	int s_is_in_opentable;              //是否在open表中
	int s_is_in_opentable_to_end;
	int s_is_in_opentable_to_P1;
	int lengthInQueue = 0;                //等于0是在距离远的队列
}AStarNode, *pAStarNode;



int timeCostFinal = 0, disCostFinal = 0;
int timeCostTemp = 0, disCostTemp = 0;
typedef struct ROBOT2
{
	int num;
	int s_x;
	int s_y;
	int end_time;
	int currLocationType;    //定义当前机器人在出口（1）还是车位（2）,在入口只有一开始（0）
	int numInParkArray;     //当前机器人在车位序列中的序号，如不在车位，则为-1
}ROBOT;
ROBOT robots[N];
int robotsNumber = 0;





AStarNode *start_node;                  // 起始点
AStarNode *end_node;                    // 结束点
int open_node_count = 0;                //open_table表的下标号
int open_node_count_to_end = 0;
int open_node_count_to_P1 = 0;
int P_NUM1 = 0, P_NUM2 = 0,P_NUM3=0;
AStarNode  map_maze[XMAX][YMAX];      // 节点数组
pAStarNode open_table[10000];              // open表，实际是个二叉堆，存的是通路上的结构体点的指针
pAStarNode open_table_to_end[10000];
pAStarNode open_table_to_P1[10000];
AStarNode *cur_nodes;
AStarNode *park_nodes;

AStarNode  cur_nodes2;
AStarNode  park_nodes2;


/***********************成本相关的参数（以后改为输入）***************************/
int k, p, a, b;



/***********************与车位分配有关的参数***************************/

pAStarNode Parking[100];  //额外存所有停车位的指针，用来进行车位的全排列
int Parking_top = -1;
int fullArr2D[500][N] = { 0 };
int c_num = 0;
int CAR_N = 0;
int end2start;//Di找到起点到终点的距离
int weightArray[10000];

struct cmpPathLength
{
	//对结构体运算符（）进行重载，新的运算符（）针对的对象是结构体
	bool operator()(AStarNode *a, AStarNode *b)
	{
		return ((a->s_to_start + a->s_to_end) > (b->s_to_start + b->s_to_end) ? 1 :
			((a->s_to_start + a->s_to_end) == (b->s_to_start + b->s_to_end)) ?
			((a->s_to_end>b->s_to_end) ? 1 : 0)
			: 0);
	}

};
std::priority_queue<AStarNode*, std::vector<AStarNode*>, cmpPathLength> QueuePathLength;
std::priority_queue<AStarNode*, std::vector<AStarNode*>, cmpPathLength> QueuePathLengthTop;
int weightSignal;
pAStarNode path_stack[10000][100];    // 保存路径的堆栈
int top[10000] = { 0 };



//DI.h
void swap(int idx1, int idx2)
{
	pAStarNode tmp = open_table[idx1];
	open_table[idx1] = open_table[idx2];
	open_table[idx2] = tmp;
}

void swap_to_end(int idx1, int idx2)
{
	pAStarNode tmp = open_table_to_end[idx1];
	open_table_to_end[idx1] = open_table_to_end[idx2];
	open_table_to_end[idx2] = tmp;
}

void swap_to_P1(int idx1, int idx2)
{
	pAStarNode tmp = open_table_to_P1[idx1];
	open_table_to_P1[idx1] = open_table_to_P1[idx2];
	open_table_to_P1[idx2] = tmp;
}


// open_table是一个堆
void adjust_heap(int nIndex)  //输入结点的索引nIndex
{
	int curr = nIndex;
	int child = curr * 2 + 1;   // 得到左孩子idx( 下标从0开始，所有左孩子是curr*2+1 )
	int parent = (curr - 1) / 2;  // 得到父节点idx

	if (nIndex < 0 || nIndex >= open_node_count)  //如果越界了，返回
	{
		return;
	}

	// 往下调整( 要比较左右孩子和cuur parent )
	while (child < open_node_count)
	{
		// 小根堆  得到较小的子节点

		if (child + 1 < open_node_count && open_table[child]->s_to_start  > open_table[child + 1]->s_to_start)
		{
			++child;//<span style="white-space:pre">              </span>// 判断左右孩子大小
		}

		if (open_table[curr]->s_to_start <= open_table[child]->s_to_start)
		{
			break;
		}
		else
		{
			swap(child, curr);            // 交换节点//
			curr = child;               // 再判断当前孩子节点
			child = curr * 2 + 1;           // 再判断左孩子
		}
	}


	if (curr != nIndex)
	{
		return;
	}

	// 往上调整( 只需要比较cuur child和parent )
	while (curr != 0)
	{
		if (open_table[curr]->s_to_start > open_table[parent]->s_to_start)
		{
			break;
		}
		else
		{
			swap(curr, parent);
			curr = parent;
			parent = (curr - 1) / 2;
		}
	}
}
void adjust_heap_to_end(int nIndex)  //输入结点的索引nIndex
{
	int curr = nIndex;
	int child = curr * 2 + 1;   // 得到左孩子idx( 下标从0开始，所有左孩子是curr*2+1 )
	int parent = (curr - 1) / 2;  // 得到父节点idx

	if (nIndex < 0 || nIndex >= open_node_count_to_end)  //如果越界了，返回
	{
		return;
	}

	// 往下调整( 要比较左右孩子和cuur parent )
	while (child < open_node_count_to_end)
	{
		// 小根堆  得到较小的子节点

		if (child + 1 < open_node_count_to_end && open_table_to_end[child]->s_to_end  > open_table_to_end[child + 1]->s_to_end)
		{
			++child;//<span style="white-space:pre">              </span>// 判断左右孩子大小
		}

		if (open_table_to_end[curr]->s_to_end <= open_table_to_end[child]->s_to_end)
		{
			break;
		}
		else
		{
			swap_to_end(child, curr);            // 交换节点//
			curr = child;               // 再判断当前孩子节点
			child = curr * 2 + 1;           // 再判断左孩子
		}
	}


	if (curr != nIndex)
	{
		return;
	}

	// 往上调整( 只需要比较cuur child和parent )
	while (curr != 0)
	{
		if (open_table_to_end[curr]->s_to_end > open_table_to_end[parent]->s_to_end)
		{
			break;
		}
		else
		{
			swap_to_end(curr, parent);
			curr = parent;
			parent = (curr - 1) / 2;
		}
	}
}


void adjust_heap_to_P1(int nIndex)  //输入结点的索引nIndex
{
	int curr = nIndex;
	int child = curr * 2 + 1;   // 得到左孩子idx( 下标从0开始，所有左孩子是curr*2+1 )
	int parent = (curr - 1) / 2;  // 得到父节点idx

	if (nIndex < 0 || nIndex >= open_node_count_to_P1)  //如果越界了，返回
	{
		return;
	}

	// 往下调整( 要比较左右孩子和cuur parent )
	while (child < open_node_count_to_P1)
	{
		// 小根堆  得到较小的子节点

		if (child + 1 < open_node_count_to_P1 && open_table_to_P1[child]->s_to_P1  > open_table_to_P1[child + 1]->s_to_P1)
		{
			++child;//<span style="white-space:pre">              </span>// 判断左右孩子大小
		}

		if (open_table_to_P1[curr]->s_to_P1 <= open_table_to_P1[child]->s_to_P1)
		{
			break;
		}
		else
		{
			swap_to_P1(child, curr);            // 交换节点//
			curr = child;               // 再判断当前孩子节点
			child = curr * 2 + 1;           // 再判断左孩子
		}
	}


	if (curr != nIndex)
	{
		return;
	}

	// 往上调整( 只需要比较cuur child和parent )
	while (curr != 0)
	{
		if (open_table_to_P1[curr]->s_to_P1 > open_table_to_P1[parent]->s_to_P1)
		{
			break;
		}
		else
		{
			swap_to_P1(curr, parent);
			curr = parent;
			parent = (curr - 1) / 2;
		}
	}
}

// 判断邻居点是否可以进入open表
void insert_to_opentable(int x, int y, pAStarNode curr_node, int w)
{
	int i;
	//kTable = 0;

	if (curr_node->s_style == PARK)
	{
		if (map_maze[x][y].s_style != BARRIER && map_maze[x][y].s_style !=PARK )        // 不是障碍物
		{
			if (!map_maze[x][y].s_is_in_closetable)   // 不在闭表中
			{
				if (map_maze[x][y].s_is_in_opentable) // 在open表中
				{
					// 需要判断是否是一条更优化的路径
					if (map_maze[x][y].s_to_start > curr_node->s_to_start + w)    // 如果更优化
					{
						map_maze[x][y].s_to_start = curr_node->s_to_start + w;
						map_maze[x][y].s_parent = curr_node;

						//找到i
						for (i = 0; i < open_node_count; ++i)
						{
							if (open_table[i]->s_x == map_maze[x][y].s_x && open_table[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap(i);                   // 下面调整点
					}
				}
				else              // 不在open中
				{

					map_maze[x][y].s_to_start = curr_node->s_to_start + w;
					map_maze[x][y].s_parent = curr_node;
					map_maze[x][y].s_is_in_opentable = 1;
					open_table[open_node_count++] = &(map_maze[x][y]);
					adjust_heap(open_node_count - 1);
				}
			}
		}
	}
	else
	{
		if (map_maze[x][y].s_style != BARRIER )        // 不是障碍物
		{
			if (!map_maze[x][y].s_is_in_closetable)   // 不在闭表中
			{
				if (map_maze[x][y].s_is_in_opentable) // 在open表中
				{
					// 需要判断是否是一条更优化的路径
					if (map_maze[x][y].s_to_start > curr_node->s_to_start + w)    // 如果更优化
					{
						map_maze[x][y].s_to_start = curr_node->s_to_start + w;
						map_maze[x][y].s_parent = curr_node;

						//找到i
						for (i = 0; i < open_node_count; ++i)
						{
							if (open_table[i]->s_x == map_maze[x][y].s_x && open_table[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap(i);                   // 下面调整点
					}
				}
				else              // 不在open中
				{

					map_maze[x][y].s_to_start = curr_node->s_to_start + w;
					map_maze[x][y].s_parent = curr_node;
					map_maze[x][y].s_is_in_opentable = 1;
					open_table[open_node_count++] = &(map_maze[x][y]);
					adjust_heap(open_node_count - 1);
				}
			}
		}
	}
	
}

void insert_to_opentable_to_end(int x, int y, pAStarNode curr_node, int w)
{
	int i;
	//kTable = 0;
	if (curr_node->s_style == PARK)
	{
		if (map_maze[x][y].s_style != BARRIER && map_maze[x][y].s_style != PARK)        // 不是障碍物
		{
			if (!map_maze[x][y].s_is_in_closetable_to_end)   // 不在闭表中
			{
				if (map_maze[x][y].s_is_in_opentable_to_end) // 在open表中
				{
					// 需要判断是否是一条更优化的路径
					if (map_maze[x][y].s_to_end > curr_node->s_to_end + w)    // 如果更优化
					{
						map_maze[x][y].s_to_end = curr_node->s_to_end + w;
						map_maze[x][y].s_parent_to_end = curr_node;

						//找到i
						for (i = 0; i < open_node_count_to_end; ++i)
						{
							if (open_table_to_end[i]->s_x == map_maze[x][y].s_x && open_table_to_end[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_end(i);                   // 下面调整点
					}
				}
				else              // 不在open中
				{
					map_maze[x][y].s_to_end = curr_node->s_to_end + w;
					map_maze[x][y].s_parent_to_end = curr_node;
					map_maze[x][y].s_is_in_opentable_to_end = 1;
					open_table_to_end[open_node_count_to_end++] = &(map_maze[x][y]);
					adjust_heap_to_end(open_node_count_to_end - 1);
				}
			}
		}
	}
	else
	{
		if (map_maze[x][y].s_style != BARRIER)        // 不是障碍物
		{
			if (!map_maze[x][y].s_is_in_closetable_to_end)   // 不在闭表中
			{
				if (map_maze[x][y].s_is_in_opentable_to_end) // 在open表中
				{
					// 需要判断是否是一条更优化的路径
					if (map_maze[x][y].s_to_end > curr_node->s_to_end + w)    // 如果更优化
					{
						map_maze[x][y].s_to_end = curr_node->s_to_end + w;
						map_maze[x][y].s_parent_to_end = curr_node;

						//找到i
						for (i = 0; i < open_node_count_to_end; ++i)
						{
							if (open_table_to_end[i]->s_x == map_maze[x][y].s_x && open_table_to_end[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_end(i);                   // 下面调整点
					}
				}
				else              // 不在open中
				{
					map_maze[x][y].s_to_end = curr_node->s_to_end + w;
					map_maze[x][y].s_parent_to_end = curr_node;
					map_maze[x][y].s_is_in_opentable_to_end = 1;
					open_table_to_end[open_node_count_to_end++] = &(map_maze[x][y]);
					adjust_heap_to_end(open_node_count_to_end - 1);
				}
			}
		}
	}
}

void insert_to_opentable_to_P1(int x, int y, pAStarNode curr_node, int w)
{
	int i;
	//kTable = 0;

	//if (map_maze[x][y].s_style != BARRIER && map_maze[x][y].s_style != PARK && map_maze[x][y].s_style != PARKCARIN)        // 不是障碍物
	//如果当前的点是车位，那么它的邻域不可以是车位或者障碍或者停了车的车位
	//如果当前点是过道，那么无所谓，只要不是障碍就可以
	if (curr_node->s_style == PARK || curr_node->s_style == PARKCARIN)
	{
		if (map_maze[x][y].s_style != BARRIER&& map_maze[x][y].s_style != PARK && map_maze[x][y].s_style != PARKCARIN)
		{
			if (!map_maze[x][y].s_is_in_closetable_to_P1)   // 不在闭表中
			{
				if (map_maze[x][y].s_is_in_opentable_to_P1) // 在open表中
				{
					// 需要判断是否是一条更优化的路径
					if (map_maze[x][y].s_to_P1 > curr_node->s_to_P1 + w)    // 如果更优化
					{
						map_maze[x][y].s_to_P1 = curr_node->s_to_P1 + w;
						map_maze[x][y].s_parent_to_P1 = curr_node;

						//找到i
						for (i = 0; i < open_node_count_to_P1; ++i)
						{
							if (open_table_to_P1[i]->s_x == map_maze[x][y].s_x && open_table_to_P1[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_P1(i);                   // 下面调整点
					}
				}
				else              // 不在open中
				{
					map_maze[x][y].s_to_P1 = curr_node->s_to_P1 + w;
					map_maze[x][y].s_parent_to_P1 = curr_node;
					map_maze[x][y].s_is_in_opentable_to_P1 = 1;
					open_table_to_P1[open_node_count_to_P1++] = &(map_maze[x][y]);
					adjust_heap_to_P1(open_node_count_to_P1 - 1);
				}
			}
		}
	}
	else
	{
		if (map_maze[x][y].s_style != BARRIER)
		{
			if (!map_maze[x][y].s_is_in_closetable_to_P1)   // 不在闭表中
			{
				if (map_maze[x][y].s_is_in_opentable_to_P1) // 在open表中
				{
					// 需要判断是否是一条更优化的路径
					if (map_maze[x][y].s_to_P1 > curr_node->s_to_P1 + w)    // 如果更优化
					{
						map_maze[x][y].s_to_P1 = curr_node->s_to_P1 + w;
						map_maze[x][y].s_parent_to_P1 = curr_node;

						//找到i
						for (i = 0; i < open_node_count_to_P1; ++i)
						{
							if (open_table_to_P1[i]->s_x == map_maze[x][y].s_x && open_table_to_P1[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_P1(i);                   // 下面调整点
					}
				}
				else              // 不在open中
				{
					map_maze[x][y].s_to_P1 = curr_node->s_to_P1 + w;
					map_maze[x][y].s_parent_to_P1 = curr_node;
					map_maze[x][y].s_is_in_opentable_to_P1 = 1;
					open_table_to_P1[open_node_count_to_P1++] = &(map_maze[x][y]);
					adjust_heap_to_P1(open_node_count_to_P1 - 1);
				}
			}
		}
	}

}

// 查找邻居
// 对上下左右4个邻居进行查找

void get_neighbors(pAStarNode curr_node)
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;
	
	// 下面对于4个邻居进行处理
	//考察当前点的右边的点
	if ((x + 1) > 0 && (x + 1) < XSIZE && y >= 0 && y < YSIZE)
	{
		insert_to_opentable(x + 1, y, curr_node, 1);

	}

	if ((x - 1) >= 0 && (x - 1) < XSIZE && y >= 0 && y < YSIZE)
	{
		insert_to_opentable(x - 1, y, curr_node, 1);

	}

	if (x >= 0 && x < XSIZE && (y + 1) > 0 && (y + 1) < YSIZE)
	{
		insert_to_opentable(x, y + 1, curr_node, 1);

	}

	if (x >= 0 && x < XSIZE && (y - 1) >= 0 && (y - 1) < YSIZE)
	{
		insert_to_opentable(x, y - 1, curr_node, 1);

	}
}

void get_neighbors_to_end(pAStarNode curr_node)
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;
	// 下面对于4个邻居进行处理
	//考察当前点的右边的点
	if ((x + 1) > 0 && (x + 1) < XSIZE && y >= 0 && y < YSIZE)
	{
		insert_to_opentable_to_end(x + 1, y, curr_node, 1);

	}

	if ((x - 1) >= 0 && (x - 1) < XSIZE && y >= 0 && y < YSIZE)
	{
		insert_to_opentable_to_end(x - 1, y, curr_node, 1);

	}

	if (x >= 0 && x < XSIZE && (y + 1) > 0 && (y + 1) < YSIZE)
	{
		insert_to_opentable_to_end(x, y + 1, curr_node, 1);

	}

	if (x >= 0 && x < XSIZE && (y - 1) >= 0 && (y - 1) < YSIZE)
	{
		insert_to_opentable_to_end(x, y - 1, curr_node, 1);

	}
}

void get_neighbors_to_P1(pAStarNode curr_node)
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;
	// 下面对于4个邻居进行处理
	//考察当前点的右边的点
	if ((x + 1) > 0 && (x + 1) < XSIZE && y >= 0 && y < YSIZE)
	{
		insert_to_opentable_to_P1(x + 1, y, curr_node, 1);

	}

	if ((x - 1) >= 0 && (x - 1) < XSIZE && y >= 0 && y < YSIZE)
	{
		insert_to_opentable_to_P1(x - 1, y, curr_node, 1);

	}

	if (x >= 0 && x < XSIZE && (y + 1) > 0 && (y + 1) < YSIZE)
	{
		insert_to_opentable_to_P1(x, y + 1, curr_node, 1);

	}

	if (x >= 0 && x < XSIZE && (y - 1) >= 0 && (y - 1) < YSIZE)
	{
		insert_to_opentable_to_P1(x, y - 1, curr_node, 1);

	}
}



//map.h
/*********************思路*************************
//要完成的工作是根据输入进来的任务排序（结构体数组），给它们分配车位，使成本最低，遍历所有的车位安排方案
第一种思路是把车位按照（未使用&&到起点和终点距离短）排序，放在队列里面，每个任务从里面选择一个最上面的车
第二种思路是从M个车位里，选出N个组全排列，计算这Amn中方法的成本，先考虑第二种
**************************************************/
typedef struct TASKING
{
	//车辆信息
	int num;
	int time_in;
	int time_in_true;
	int time_out;
	int time_out_true;
	int time_wait;
	int mass;
	int mass_num;

	//机器人信息
	int robot_number;                 //机器人总数量
	int robot_numbering;              //当前机器人编号
	int robot_numbering_out;

	//任务信息
	int task_type;                    //0为取车，1为停车

	//车位信息
	int park_x;
	int park_y;
	//int numInParkArray;          //在当前的车位安排方案中占了第几个位置，即fullArr2D[360][N]中某一行的第几列

	//成本信息
	int time_in_out;                 //车辆延时信息
	int distance;                    //当前任务行驶里程
	int s_is_in_closetable;          //当前任务是否放弃,1表示放弃，0不放弃
	int COST;                        //当前任务总成本
	struct TASKING *coupleOut;         //每一个停车任务，有一个配对的出车任务，它俩绑定在一起
	struct TASKING *coupleIn;        //初始化的时候就要设置好，停车任务对应的出车任务是哪个
}TASKING, *pTASKING;
TASKING task_H[10000];
TASKING taskFromP[10000];
TASKING taskOptical[10000];

void swap_tasking(int idx1, int idx2)
{
	int temp1, temp2, temp4, temp5, temp6, temp7;
	struct TASKING *temp8, *temp9;
	if (taskFromP[idx1].task_type == 1 && taskFromP[idx2].task_type == 1)
	{
		taskFromP[idx1].coupleOut->coupleIn = &taskFromP[idx2];
		taskFromP[idx2].coupleOut->coupleIn = &taskFromP[idx1];
		temp1 = taskFromP[idx1].num;
		temp2 = taskFromP[idx1].mass;
		temp4 = taskFromP[idx1].time_in;
		temp5 = taskFromP[idx1].time_out;
		temp6 = taskFromP[idx1].time_wait;
		temp7 = taskFromP[idx1].task_type;
		temp9 = taskFromP[idx1].coupleOut;
		taskFromP[idx1].num = taskFromP[idx2].num;
		taskFromP[idx1].mass = taskFromP[idx2].mass;
		taskFromP[idx1].time_in = taskFromP[idx2].time_in;
		taskFromP[idx1].time_out = taskFromP[idx2].time_out;
		taskFromP[idx1].time_wait = taskFromP[idx2].time_wait;
		taskFromP[idx1].task_type = taskFromP[idx2].task_type;
		taskFromP[idx1].coupleOut = taskFromP[idx2].coupleOut;
		taskFromP[idx2].num = temp1;
		taskFromP[idx2].mass = temp2;
		taskFromP[idx2].time_in = temp4;
		taskFromP[idx2].time_out = temp5;
		taskFromP[idx2].time_wait = temp6;
		taskFromP[idx2].task_type = temp7;
		taskFromP[idx2].coupleOut = temp9;
	}
	else if (taskFromP[idx1].task_type == 0 && taskFromP[idx2].task_type == 0)
	{
		taskFromP[idx1].coupleIn->coupleOut = &taskFromP[idx2];
		taskFromP[idx2].coupleIn->coupleOut = &taskFromP[idx1];
		temp1 = taskFromP[idx1].num;
		temp2 = taskFromP[idx1].mass;
		temp4 = taskFromP[idx1].time_in;
		temp5 = taskFromP[idx1].time_out;
		temp6 = taskFromP[idx1].time_wait;
		temp7 = taskFromP[idx1].task_type;
		temp9 = taskFromP[idx1].coupleIn;
		taskFromP[idx1].num = taskFromP[idx2].num;
		taskFromP[idx1].mass = taskFromP[idx2].mass;
		taskFromP[idx1].time_in = taskFromP[idx2].time_in;
		taskFromP[idx1].time_out = taskFromP[idx2].time_out;
		taskFromP[idx1].time_wait = taskFromP[idx2].time_wait;
		taskFromP[idx1].task_type = taskFromP[idx2].task_type;
		taskFromP[idx1].coupleIn = taskFromP[idx2].coupleIn;
		taskFromP[idx2].num = temp1;
		taskFromP[idx2].mass = temp2;
		taskFromP[idx2].time_in = temp4;
		taskFromP[idx2].time_out = temp5;
		taskFromP[idx2].time_wait = temp6;
		taskFromP[idx2].task_type = temp7;
		taskFromP[idx2].coupleIn = temp9;
	}
	else if (taskFromP[idx1].task_type == 1 && taskFromP[idx2].task_type == 0)
	{
		taskFromP[idx1].coupleOut->coupleIn = &taskFromP[idx2];
		taskFromP[idx2].coupleIn->coupleOut = &taskFromP[idx1];
		temp1 = taskFromP[idx1].num;
		temp2 = taskFromP[idx1].mass;
		temp4 = taskFromP[idx1].time_in;
		temp5 = taskFromP[idx1].time_out;
		temp6 = taskFromP[idx1].time_wait;
		temp7 = taskFromP[idx1].task_type;
		temp8 = taskFromP[idx1].coupleIn;
		temp9 = taskFromP[idx1].coupleOut;
		taskFromP[idx1].num = taskFromP[idx2].num;
		taskFromP[idx1].mass = taskFromP[idx2].mass;
		taskFromP[idx1].time_in = taskFromP[idx2].time_in;
		taskFromP[idx1].time_out = taskFromP[idx2].time_out;
		taskFromP[idx1].time_wait = taskFromP[idx2].time_wait;
		taskFromP[idx1].task_type = taskFromP[idx2].task_type;
		taskFromP[idx1].coupleIn = taskFromP[idx2].coupleIn;
		taskFromP[idx1].coupleOut = taskFromP[idx2].coupleOut;
		taskFromP[idx2].num = temp1;
		taskFromP[idx2].mass = temp2;
		taskFromP[idx2].time_in = temp4;
		taskFromP[idx2].time_out = temp5;
		taskFromP[idx2].time_wait = temp6;
		taskFromP[idx2].task_type = temp7;
		taskFromP[idx2].coupleIn = temp8;
		taskFromP[idx2].coupleOut = temp9;
	}
	else
	{
		taskFromP[idx1].coupleIn->coupleOut = &taskFromP[idx2];
		taskFromP[idx2].coupleOut->coupleIn = &taskFromP[idx1];
		temp1 = taskFromP[idx1].num;
		temp2 = taskFromP[idx1].mass;
		temp4 = taskFromP[idx1].time_in;
		temp5 = taskFromP[idx1].time_out;
		temp6 = taskFromP[idx1].time_wait;
		temp7 = taskFromP[idx1].task_type;
		temp8 = taskFromP[idx1].coupleIn;
		temp9 = taskFromP[idx1].coupleOut;
		taskFromP[idx1].num = taskFromP[idx2].num;
		taskFromP[idx1].mass = taskFromP[idx2].mass;
		taskFromP[idx1].time_in = taskFromP[idx2].time_in;
		taskFromP[idx1].time_out = taskFromP[idx2].time_out;
		taskFromP[idx1].time_wait = taskFromP[idx2].time_wait;
		taskFromP[idx1].task_type = taskFromP[idx2].task_type;
		taskFromP[idx1].coupleIn = taskFromP[idx2].coupleIn;
		taskFromP[idx1].coupleOut = taskFromP[idx2].coupleOut;
		taskFromP[idx2].num = temp1;
		taskFromP[idx2].mass = temp2;
		taskFromP[idx2].time_in = temp4;
		taskFromP[idx2].time_out = temp5;
		taskFromP[idx2].time_wait = temp6;
		taskFromP[idx2].task_type = temp7;
		taskFromP[idx2].coupleIn = temp8;
		taskFromP[idx2].coupleOut = temp9;
	}
}

/********************使用DI算法找两个点之间的最短路径*************************/
int findLengthFromP2P(pAStarNode P1, pAStarNode P2)
{
	int i, j;
	//先把P1有关的重置
	if (P1->s_x == P2->s_x && P1->s_y == P2->s_y)
		return 0;
	
	open_node_count_to_P1 = 0;
	for (i = 0; i < XSIZE; i++)
	{
		for (j = 0; j < YSIZE; j++)
		{
			map_maze[i][j].s_to_P1 = INF;
			map_maze[i][j].s_is_in_closetable_to_P1 = 0;
			map_maze[i][j].s_is_in_opentable_to_P1 = 0;
			map_maze[i][j].s_parent_to_P1 = NULL;
		}
	}
	open_table_to_P1[open_node_count_to_P1++] = P1;
	P1->s_is_in_opentable_to_P1 = 1;
	P1->s_parent_to_P1 = NULL;
	P1->s_to_P1 = 0;
	pAStarNode curr_node = P1;
	while (1)
	{
		get_neighbors_to_P1(curr_node);
		curr_node->s_is_in_closetable_to_P1 = 1;       // 已经在close表中了
		open_table_to_P1[0] = open_table_to_P1[--open_node_count_to_P1];  // 最后一个点放到第一个点，然后进行堆调整
		adjust_heap_to_P1(0);               // 调整堆
		curr_node = open_table_to_P1[0];
		if (open_node_count_to_P1 == 0)             // 没有路径到达
		{
			break;
		}
	}

	return map_maze[P2->s_x][P2->s_y].s_to_P1;

}
/*****************************************************************************/

void swap2(int *m, int *n)
{
	int temp;
	temp = *m;
	*m = *n;
	*n = temp;
}


/*******************************全排列分配函数**************************/
void arrange(int * arr, int len, int num, int k, int *arr2D)
{
	int i, j;
	int arr2D_top = 0;
	if (k == num)  ///结束条件
	{
		for (i = 0; i<num; i++)
		{
			fullArr2D[c_num][i] = arr[i];
			//		printf("%d  ",fullArr2D[c_num][i]);
		}
		c_num++;
		//printf("        hh%d\n",c_num);
		return;
	}
	for (j = num; j<len; j++) ///遍历第一个位置的所有情况
	{
		swap2(&arr[j], &arr[num]);
		///num在这里相当于是起点，K相当于是终点
		arrange(arr, len, num + 1, k, arr2D);
		///还原，继续下一次遍历
		swap2(&arr[j], &arr[num]);
	}
}
/*****************************************************************************/



/*******************列出所有可能的车位的全排列情况*********************/
//void fullArrangePark(AStarNode maze[XSIZE][YSIZE], int arr2D[][6], int *c)
void fullArrangePark(AStarNode maze[XMAX][YMAX], int *arr2D, int *c)
{
	int fullArr1D[100];
	int fullArr1D_top = -1;
	int sig = 0;
	int i, j;
//	int temp;
	for (i = 0; i < XSIZE; i++)
	{
		for (j = 0; j < YSIZE; j++)
		{
			if (maze[i][j].s_style == PARK)
			{

				Parking[++Parking_top] = &maze[i][j];      //把所有车位类型的点的指针取出来放进Parking里面
				fullArr1D[++fullArr1D_top] = sig;          //
				sig++;
			}
		}
	}
	arrange(fullArr1D, P_NUM1, 0, N, arr2D);
}
/***********************************************************************/


bool cmp(const int a, const int b)
{
	return a>b;
}





int main()
{
	int arrayTh=-1;                  //已经分配到第多少个任务
	AStarNode *curr_node;           // 当前点
	int finished = 0;               //是否找到路径
	char maze[XMAX][YMAX];          //赋值给 map_maze I代表起点，E代表终点，B代表障碍物, X代表道路，P代表停车位
	//注意二维矩阵第一个参数是行数，也就是y，第二个是列数，也就是x，总是要颠倒一下
	//这里定义的是直观上的，x就是第几行，也就是i，y就是j
	int i, j, k2, m;
	int numofStart = 0, numofEnd = 0;
	int isSaturate = 0;

	/****************************************输入地图数据*****************************************/
	cin >> k >> p >> a >> b;
	cin >> XSIZE >> YSIZE;

	for (i = 0; i<XSIZE; i++)
	{
		for (j = 0; j<YSIZE; j++)
		{
			cin >> maze[i][j];
			if (maze[i][j] == 'P')
				P_NUM1++;
		}
	}
	cin >> CAR_N;
	for (i = 0; i<CAR_N; i++)
	{
		cin>>task_H[i].num;  // 循环读
		cin>>task_H[i].time_in;
		cin>>task_H[i].time_out;
		cin>>task_H[i].time_wait;
		cin >> task_H[i].mass;
	}

	/*************************************************************************************************/




	/********************************判断地图且确定起点到每点的路径*************************************/
	//可到达的点按照到起点的长度排列在open_table[]里面
	//这里做了一点改动，把所有点初始化的范围扩大了
	for (i = 0; i < XSIZE; i++)
	{
		for (j = 0; j < YSIZE; j++)
		{
			map_maze[i][j].s_to_start = INF;
			map_maze[i][j].s_to_end = INF;
			map_maze[i][j].s_to_P1 = INF;
			map_maze[i][j].s_is_in_closetable = 0;
			map_maze[i][j].s_is_in_opentable = 0;
			map_maze[i][j].s_is_in_closetable_to_end = 0;
			map_maze[i][j].s_is_in_opentable_to_end = 0;
			map_maze[i][j].s_is_in_closetable_to_P1 = 0;
			map_maze[i][j].s_is_in_opentable_to_P1 = 0;
			map_maze[i][j].s_style = maze[i][j];
			map_maze[i][j].s_x = i;
			map_maze[i][j].s_y = j;
			map_maze[i][j].s_parent = NULL;
			map_maze[i][j].s_parent_to_end = NULL;
			map_maze[i][j].s_parent_to_P1 = NULL;

			if (map_maze[i][j].s_style == STARTNODE)  // 起点，没有考虑两个I的情况
			{
				if (!(map_maze[i][j].s_x == 0 || map_maze[i][j].s_y == 0 || map_maze[i][j].s_x == XSIZE - 1 || map_maze[i][j].s_y == YSIZE - 1))
				{
					//printf("起点不在地图边缘");
					printf("NO\n");
					return 0;
				}
				map_maze[i][j].s_to_start = 0;
				start_node = &(map_maze[i][j]);
				numofStart++;
			}
			if (map_maze[i][j].s_style == ENDNODE)    // 终点
			{
				if (!(map_maze[i][j].s_x == 0 || map_maze[i][j].s_y == 0 || map_maze[i][j].s_x == XSIZE - 1 || map_maze[i][j].s_y == YSIZE - 1))
				{
					//printf("终点不在地图边缘");
					printf("NO\n");
					return 0;
				}
				end_node = &(map_maze[i][j]);
				numofEnd++;
			}
		}
	}
	if (numofStart != 1)
	{
		//printf("起点数量错误");
		//system("pause");
		printf("NO\n");
		return 0;
	}
	if (numofEnd != 1)
	{
		//printf("终点数量错误");
		//system("pause");
		printf("NO\n");
		return 0;
	}

	//下面使用DI算法得到起点到其余点的路径
	open_table[open_node_count++] = start_node;     // 起点肯定在open表，加入;open_node_count先0后1
	start_node->s_is_in_opentable = 1;              // 起点已加入open表
	start_node->s_parent = NULL;
	curr_node = start_node;
	while (1)
	{
		if (curr_node->s_style == 'P')
		{
			int mode_P = 0;
			P_NUM2++;
			if (curr_node->s_y + 1 < YSIZE && map_maze[curr_node->s_x][curr_node->s_y + 1].s_style == 'X')
				mode_P++;
			if (curr_node->s_y - 1 >= 0 && map_maze[curr_node->s_x][curr_node->s_y - 1].s_style == 'X')
				mode_P++;
			if (curr_node->s_x + 1 < XSIZE && map_maze[curr_node->s_x + 1][curr_node->s_y].s_style == 'X')
				mode_P++;
			if (curr_node->s_x - 1 >= 0 && map_maze[curr_node->s_x - 1][curr_node->s_y].s_style == 'X')
				mode_P++;
			if (mode_P > 1)
			{
				//printf("%d,%d停车位不符合要求,一个车位有不止一个过道\n", curr_node->s_x, curr_node->s_y);
				//printf("mode_P：%d  ", mode_P);
				printf("NO\n");
				return 0;
			}
			if (mode_P < 1)
			{
				//printf("%d,%d停车位不符合要求,车位被障碍物或者其他车位包围\n", curr_node->s_x, curr_node->s_y);
				//printf("mode_P：%d  ", mode_P);
				printf("NO\n");
				return 0;
			}
		}
		get_neighbors(curr_node);                       //对于每个点，访问它的邻点，curr_node代表当前点
		curr_node->s_is_in_closetable = 1;              //已经在close表中了
		open_table[0] = open_table[--open_node_count];  //最后一个点放到第一个点，然后进行堆调整
		adjust_heap(0);                                 //调整堆
		curr_node = open_table[0];
		if (open_node_count == 0)                       //没有路径到达
		{
			break;
		}

	}
	//printf("停车位的数量：%d  \n", P_NUM2);

	
	/**************************************************************************************/



	/**********************终点到各个点的路径并建立最短和路径的队列*************************/
	/*
	再计算每点到终点的距离，然后把计算每点到起始点的距离和，把距离和放到优先队列里面
	*/
	end_node->s_to_end = 0;
	open_table_to_end[open_node_count_to_end++] = end_node;
	end_node->s_is_in_opentable_to_end = 1;
	start_node->s_parent_to_end = NULL;
	curr_node = end_node;
	while (1)
	{
		if (curr_node->s_style == 'P')
		{
			P_NUM3++;
		}
		get_neighbors_to_end(curr_node);
		curr_node->s_is_in_closetable_to_end = 1;                            //已经在close表中了
		open_table_to_end[0] = open_table_to_end[--open_node_count_to_end];  //最后一个点放到第一个点，然后进行堆调整
		adjust_heap_to_end(0);                                               //调整堆
		curr_node = open_table_to_end[0];
		if (open_node_count_to_end == 0)                                     //没有路径到达
		{
			break;
		}
	}
	if (P_NUM1 == P_NUM2&&P_NUM3==P_NUM1)
	{
		//printf("地图符合要求\n");
		printf("YES\n");

	}
	else
	{
		//printf("地图不符合要求\n");
		printf("NO\n");
		return 0;  //没有找到路径
	}

	end2start = findLengthFromP2P(start_node, end_node);

	/****************************************任务时间排序*****************************************/
	for (i = 0; i<CAR_N; i++)
	{
		taskFromP[i * 2].num = task_H[i].num;                      //第几个车
		taskFromP[i * 2].mass = task_H[i].mass;
		taskFromP[i * 2].time_in = task_H[i].time_in;
		taskFromP[i * 2].time_out = task_H[i].time_out;
		taskFromP[i * 2].time_wait = task_H[i].time_wait;
		taskFromP[i * 2].task_type = 1;

		weightArray[i] = task_H[i].mass;

		taskFromP[i * 2 + 1].num = task_H[i].num;
		taskFromP[i * 2 + 1].mass = task_H[i].mass;
		taskFromP[i * 2 + 1].time_in = task_H[i].time_in;
		taskFromP[i * 2 + 1].time_out = task_H[i].time_out;
		taskFromP[i * 2 + 1].time_wait = task_H[i].time_wait;
		taskFromP[i * 2 + 1].task_type = 0;

		taskFromP[i * 2].coupleOut = &taskFromP[i * 2 + 1];
		taskFromP[i * 2 + 1].coupleIn = &taskFromP[i * 2];
	}
	///排序
	int time1, time2;
	for (i = 0; i<CAR_N * 2; i++)
	{
		for (j = CAR_N * 2 - 1; j>i; j--)
		{
			if (taskFromP[j].task_type == 1) time1 = taskFromP[j].time_in;
			else time1 = taskFromP[j].time_out;
			if (taskFromP[j - 1].task_type == 1) time2 = taskFromP[j - 1].time_in;
			else time2 = taskFromP[j - 1].time_out;
			if (time1 < time2)
			{
				swap_tasking(j, j - 1);
			}
		}
	}
	for (i = 0; i < CAR_N; i++)
	{
		taskFromP[i].time_in_true = taskFromP[i].time_in;
		taskFromP[i].time_out_true = taskFromP[i].time_out;
	}

	/*************************************************************************************************/



	/*******************************车位按照到入口和出口的距离和来排序******************************************/
	//这里考虑对半分，距离大的和距离小的从中间分成两个队列
	for (i = 0; i < XSIZE; i++)
	{
		for (j = 0; j < YSIZE; j++)
		{
			if (map_maze[i][j].s_style == 'P')
				QueuePathLength.push(&map_maze[i][j]);
		}
	}
	//如果任务数小于车位数，则只取前任务数个车位放入到好车位序列里面
	if (CAR_N < P_NUM1)
	{
		for (i = 0; i < CAR_N / 2; i++)
		{
			AStarNode *temp;
			temp = QueuePathLength.top();
			temp->lengthInQueue = 1;
			QueuePathLength.pop();
			QueuePathLengthTop.push(temp);
		}
	}
	else
	{
		for (i = 0; i < P_NUM1 /10; i++)
		{
			AStarNode *temp;
			temp = QueuePathLength.top();
			temp->lengthInQueue = 1;
			QueuePathLength.pop();
			QueuePathLengthTop.push(temp);
		}
	}


	

	std::sort(weightArray, weightArray + CAR_N, cmp);
	weightSignal = weightArray[CAR_N / 2];
	//考虑在读入任务信息的时候，就建立一个排序的序列，以车重为标准，这样可以大体知道最重的前30个车有多重，那么后面
	//安排车位的时候，就有数了，也就是有个标准，重于这个标准的车位就安排在Top20里面，如果Top20里面没有再安排在外边
	/*************************************************************************************************/

	int start_time;               ///到达当前任务起点的时间
	int start_time_least = 20000000;         ///到达当前任务起点的时间最小时间
	int start_time_max = -1;           ///到达当前任务起点的时间最大时间
	int end_time_max = -1;             ///到达当前任务起点的时间最小时间
	int r_num;
	int result = 0;
	int result_least;
	int result_num;
	cur_nodes = &cur_nodes2;
	park_nodes = &park_nodes2;
	int mode;


	/**********************************************主循环************************************************/
	int time_waste = 1;                       //用来判断是否还需要再加机器人了，如果现在的time_waste已经是0了，就没必要再加机器人了
	result_least = INF;
	//机器人数量的循环，最外层的情况
	for (j = 1; time_waste>0 && j<100; j=j+1)  ///机器人数量j
	{
		/*******************************每个机器人数量方案下重置状态************************************/
		int timeCost = 0;
		int disCost = 0;
		arrayTh = -1;
		for (i = 0; i<CAR_N * 2; i++)
		{
			//每次重置任务某些变量的值
			taskFromP[i].park_x = 0;                      //第几个车
			taskFromP[i].park_y = 0;
			taskFromP[i].s_is_in_closetable = 0;
			taskFromP[i].COST = 0;
			taskFromP[i].time_in_out = 0;
			taskFromP[i].distance = 0;
			taskFromP[i].time_in_true = taskFromP[i].time_in;
			taskFromP[i].time_out_true = taskFromP[i].time_out;
		}
		///初始化机器人的当前位置；
		for (i = 0; i<j; i++)   ///任务遍历
		{
			cur_nodes = &cur_nodes2;
			robots[i].s_x = start_node->s_x;  ///表示在起点
			robots[i].s_y = start_node->s_y;
			robots[i].end_time = 0;
			robots[i].currLocationType = 0;
		}
		robotsNumber = j;
		/********************************************************************************************/

		for (i = 0; i<CAR_N * 2; i += j)   
		{
			for (k2 = 0; k2<j && (i + k2<CAR_N * 2); k2++) ///每次遍历j个任务
			{
				//当前任务可能放弃也可能没放弃，这里如果被放弃，那么一定是个出库任务
				//如果没放弃，那么可能是入库也可能是出库任务
				//如果是停车，那就分配车位，如果有车位就分配并继续执行任务，如果没有车位就直接加上罚时并跳到下一个
				//如果是取车，没有别的可能，只有执行出库任务
				if (taskFromP[k2 + i].s_is_in_closetable == 1)
				{
					if (taskFromP[k2 + i].task_type == 0)
						continue;
				}
				/*******************************每次把连续的一段入库任务分配好*********************************/
				int constantInNum = 0;                                                       //连续入库任务的个数
				if (k2 + i > arrayTh && taskFromP[k2 + i].task_type == 1)                        //如果该车位还没有被分配
				{
					for (int pp = k2 + i; taskFromP[pp].task_type == 1; pp++)                //统计以当前任务为起点，有多少个连续入库任务
					{
						constantInNum++;
					}
					arrayTh = constantInNum + k2 + i;                                        //arrayTh表示当前分配了多少个车位
					if (constantInNum > QueuePathLength.size() + QueuePathLengthTop.size())  //如果车位不够用，放弃abanNum个当前最重的车
					{
						int abanNum = constantInNum - QueuePathLength.size() - QueuePathLengthTop.size();
						//每次对从k2+i到k2+i+constantInNum范围内的任务排序
						for (int ii =k2+i; ii<constantInNum+k2+i; ii++)
						{
							
							int temp = 0;
							for (int jj = k2 + i; jj<constantInNum + k2 + i; jj++)
							{
								if (ii == jj)
									continue;
								if (taskFromP[ii].mass<taskFromP[jj].mass)
								{
									temp++;
								}
								else if (taskFromP[ii].mass == taskFromP[jj].mass)
								{
									int iitime=0, jjtime=0;
									if (taskFromP[ii].num>1)
										iitime += task_H[taskFromP[ii].num-1].time_in - task_H[taskFromP[ii].num-2].time_in;
									if (taskFromP[ii].num<CAR_N)
										iitime += task_H[taskFromP[ii].num].time_in - task_H[taskFromP[ii].num-1].time_in;
									if (taskFromP[jj].num>1)
										jjtime += task_H[taskFromP[jj].num-1].time_in - task_H[taskFromP[jj].num - 2].time_in;
									if (taskFromP[jj].num<CAR_N)
										jjtime += task_H[taskFromP[jj].num].time_in - task_H[taskFromP[jj].num-1].time_in;
									if (iitime > jjtime)
										temp++;
									else if (iitime == jjtime)
									{
										if (ii < jj)
											temp++;
									}									
										
								}
							}
							taskFromP[ii].mass_num = temp;
						}
					

						//放弃重的
						for (int ii = k2 + i; ii < constantInNum + k2 + i; ii++)
						{
							if (taskFromP[ii].mass_num < abanNum)
							{
								taskFromP[ii].s_is_in_closetable = 1;
								taskFromP[ii].coupleOut->s_is_in_closetable = 1;
							}
							
						}////////////////放弃重的

						int mMedium=(constantInNum-abanNum)/5+abanNum;
						int mMediumNum;
						for (mMediumNum = k2 + i; mMediumNum < constantInNum + k2 + i; mMediumNum++)
						{
							if (taskFromP[mMediumNum].mass_num == mMedium)
							{
								weightSignal = taskFromP[mMediumNum].mass;
								break;
							}								
						}


						//分配轻的
						for (int iii = abanNum; iii < constantInNum; iii++)
						{
							int jjj;
							for (jjj = k2 + i; jjj < constantInNum + k2 + i; jjj++)
							{
								if (taskFromP[jjj].mass_num == iii)
									break;
							}
							AStarNode *temp1;
							//如果最后剩下的任务小于优先车位的数量，那么直接在优先车位里面分配
							if (CAR_N * 2 - jjj < QueuePathLengthTop.size())
							{
								temp1 = QueuePathLengthTop.top();
								QueuePathLengthTop.pop();
								taskFromP[jjj].park_x = temp1->s_x;
								taskFromP[jjj].park_y = temp1->s_y;
								taskFromP[jjj].coupleOut->park_x = temp1->s_x;
								taskFromP[jjj].coupleOut->park_y = temp1->s_y;
							}

							else if (taskFromP[k2 + i + jjj].mass >= weightSignal)
							{

								if (!QueuePathLengthTop.empty())
								{
									temp1 = QueuePathLengthTop.top();
									QueuePathLengthTop.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
								else
								{
									temp1 = QueuePathLength.top();
									QueuePathLength.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
							}//优先分配近的车位
							else
							{
								if (!QueuePathLength.empty())
								{
									temp1 = QueuePathLength.top();
									QueuePathLength.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
								else
								{
									temp1 = QueuePathLengthTop.top();
									QueuePathLengthTop.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
							}//分配差车位
							/**********************验证最理想情况下是否还是大于放弃代价******************/
							int costTemp;
							costTemp = (map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].s_to_start + map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].s_to_end)*k*taskFromP[jjj].mass;
							costTemp += b*map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].s_to_end;
							if (costTemp > p)
							{
								taskFromP[jjj].s_is_in_closetable = 1;
								taskFromP[jjj].coupleOut->s_is_in_closetable = 1;
								
								if (map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].lengthInQueue == 1)
									QueuePathLengthTop.push(&map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y]);
								else
									QueuePathLength.push(&map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y]);
								continue;
							}
							/*****************************************************************************/
						}
					}/////////////////////////车位不够用，先放弃一部分再分配
					else//车位够用
					{

						//每次对从k2+i到k2+i+constantInNum范围内的任务排序
						for (int ii = k2 + i; ii<constantInNum + k2 + i; ii++)
						{

							int temp = 0;
							for (int jj = k2 + i; jj<constantInNum + k2 + i; jj++)
							{
								if (ii == jj)
									continue;
								if (taskFromP[ii].mass<taskFromP[jj].mass)
								{
									temp++;
								}
								else if (taskFromP[ii].mass == taskFromP[jj].mass)
								{
									int iitime = 0, jjtime = 0;
									if (taskFromP[ii].num>1)
										iitime += task_H[taskFromP[ii].num - 1].time_in - task_H[taskFromP[ii].num - 2].time_in;
									if (taskFromP[ii].num<CAR_N)
										iitime += task_H[taskFromP[ii].num].time_in - task_H[taskFromP[ii].num - 1].time_in;
									if (taskFromP[jj].num>1)
										jjtime += task_H[taskFromP[jj].num - 1].time_in - task_H[taskFromP[jj].num - 2].time_in;
									if (taskFromP[jj].num<CAR_N)
										jjtime += task_H[taskFromP[jj].num].time_in - task_H[taskFromP[jj].num - 1].time_in;
									if (iitime > jjtime)
										temp++;
									else if (iitime == jjtime)
									{
										if (ii < jj)
											temp++;
									}

								}
							}
							taskFromP[ii].mass_num = temp;
						}
						
						
						weightSignal = weightArray[3*CAR_N / 5];
						for (int iii = 0; iii < constantInNum; iii++)
						{
							int jjj;
							for (jjj = k2 + i; jjj < constantInNum + k2 + i; jjj++)
							{
								if (taskFromP[jjj].mass_num == iii)
									break;
							}
							AStarNode *temp1;
							//如果最后剩下的任务小于优先车位的数量，那么直接在优先车位里面分配
							if (CAR_N * 2 - jjj < QueuePathLengthTop.size())
							{
								temp1 = QueuePathLengthTop.top();
								QueuePathLengthTop.pop();
								taskFromP[jjj].park_x = temp1->s_x;
								taskFromP[jjj].park_y = temp1->s_y;
								taskFromP[jjj].coupleOut->park_x = temp1->s_x;
								taskFromP[jjj].coupleOut->park_y = temp1->s_y;
							}
							else if (taskFromP[jjj].mass >= weightSignal)
							{

								if (!QueuePathLengthTop.empty())
								{
									temp1 = QueuePathLengthTop.top();
									QueuePathLengthTop.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
								else
								{
									temp1 = QueuePathLength.top();
									QueuePathLength.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
							}//优先分配近的车位
							else
							{
								if (!QueuePathLength.empty())
								{
									temp1 = QueuePathLength.top();
									QueuePathLength.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
								else
								{
									temp1 = QueuePathLengthTop.top();
									QueuePathLengthTop.pop();
									taskFromP[jjj].park_x = temp1->s_x;
									taskFromP[jjj].park_y = temp1->s_y;
									taskFromP[jjj].coupleOut->park_x = temp1->s_x;
									taskFromP[jjj].coupleOut->park_y = temp1->s_y;
								}
							}//分配差车位
							/**********************验证最理想情况下是否还是大于放弃代价******************/
							int costTemp;
							costTemp = (map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].s_to_start + map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].s_to_end)*k*taskFromP[jjj].mass;
							costTemp += b*map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].s_to_end;
							if (costTemp > p)
							{
								taskFromP[jjj].s_is_in_closetable = 1;
								taskFromP[jjj].coupleOut->s_is_in_closetable = 1;

								if (map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y].lengthInQueue == 1)
									QueuePathLengthTop.push(&map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y]);
								else
									QueuePathLength.push(&map_maze[taskFromP[jjj].park_x][taskFromP[jjj].park_y]);
								continue;
							}
							/*****************************************************************************/
						}
					}

				}
				/****************************************************************/
				if (taskFromP[k2 + i].task_type == 1)
				{
					if (taskFromP[k2 + i].s_is_in_closetable == 1)
					{
						taskFromP[k2 + i].COST = p * 1;
						timeCost += taskFromP[i + k2].COST;
						continue;
					}
					
					/**********************验证最理想情况下是否还是大于放弃代价******************/
					int costTemp;
					costTemp = (map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_start + map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_end)*k*taskFromP[i + k2].mass;
					costTemp += b*map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_end;
					if (costTemp > p)
					{
						taskFromP[i + k2].s_is_in_closetable = 1;
						taskFromP[k2 + i].coupleOut->s_is_in_closetable = 1;
						taskFromP[i + k2].COST = p * 1;
						timeCost += taskFromP[i + k2].COST;
						if (map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].lengthInQueue == 1)
							QueuePathLengthTop.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
						else
							QueuePathLength.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
						continue;
					}
					/*****************************************************************************/


					start_time_least = INF;
					start_time_max = -1;
					end_time_max = -1;
					mode = 0;
					for (m = 0; m<j; m++)  //对于入库任务，每次找离起点最近的机器人
					{
						int length;
						length = map_maze[robots[m].s_x][robots[m].s_y].s_to_start;
						start_time = robots[m].end_time + length;
						if (start_time <= taskFromP[i + k2].time_in)
						{
							if (end_time_max<robots[m].end_time)
							{
								r_num = m;
								end_time_max = robots[m].end_time;
							}
							mode = 1;              //mode=1表示车到达入口的时间小于任务要求的入库时间
						}
						else if (mode == 0)
						{
							if (start_time<start_time_least)
							{
								start_time_least = start_time;
								r_num = m;
							}
						}
					}//至此分配完了机器人，下一步便是执行入库任务
					taskFromP[i + k2].robot_numbering = r_num;
					if (mode == 1)//如果到达入口的时间小于任务要求的入库时间，则更新当前时间
					{
						robots[r_num].end_time = taskFromP[i + k2].time_in;
						taskFromP[i + k2].time_in_out = 0;
						taskFromP[i + k2].distance = map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_start;
						taskFromP[i + k2].COST = k*taskFromP[i + k2].distance*taskFromP[i + k2].mass + b*taskFromP[i + k2].time_in_out; //该任务成本为路径*系数+延时*系数
						timeCost += b*taskFromP[i + k2].time_in_out;
						disCost += k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
						robots[r_num].end_time += taskFromP[i + k2].distance;                          //调整当前机器人的时间轴
						robots[r_num].s_x = taskFromP[i + k2].park_x;            //调整当前机器人的坐标
						robots[r_num].s_y = taskFromP[i + k2].park_y;            //调整当前机器人的坐标
						robots[r_num].currLocationType = 2;                                 //调整当前机器人的位置类型
						map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_style = PARKCARIN;
					}
					else if (mode == 0)
					{
						/**********************再验证非理想情况下是否还是大于放弃代价******************/
						int costTemp2;
						int timeTemp, time_in_outTemp;
						timeTemp = robots[r_num].end_time + map_maze[robots[r_num].s_x][robots[r_num].s_y].s_to_start;
						time_in_outTemp = 6*(timeTemp - taskFromP[i + k2].time_in);
						costTemp2 = costTemp + time_in_outTemp*b;
						if (costTemp2 > p)
						{
							taskFromP[i + k2].s_is_in_closetable = 1;
							taskFromP[k2 + i].coupleOut->s_is_in_closetable = 1;
							taskFromP[i + k2].COST = p * 1;
							timeCost += taskFromP[i + k2].COST;
							if (map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].lengthInQueue == 1)
								QueuePathLengthTop.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
							else
								QueuePathLength.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
							continue;
						}
						/*****************************************************************************/

						if (start_time_least>taskFromP[i + k2].time_in + taskFromP[i + k2].time_wait)
						{
							//time_waste = 1;
							taskFromP[i + k2].s_is_in_closetable = 1;
							taskFromP[k2 + i].coupleOut->s_is_in_closetable = 1;
							taskFromP[i + k2].COST = p * 1;
							timeCost += taskFromP[i + k2].COST;
							if (map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].lengthInQueue == 1)
								QueuePathLengthTop.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
							else
								QueuePathLength.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
							continue;
						}
						else
						{
							robots[r_num].end_time = robots[r_num].end_time + map_maze[robots[r_num].s_x][robots[r_num].s_y].s_to_start;
							taskFromP[i + k2].time_in_out = robots[r_num].end_time - taskFromP[i + k2].time_in;
							taskFromP[i + k2].time_in_true = robots[r_num].end_time;
							//time_waste = 1;
							taskFromP[i + k2].distance = map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_start;
							taskFromP[i + k2].COST = k*taskFromP[i + k2].distance*taskFromP[i + k2].mass + b*taskFromP[i + k2].time_in_out; //该任务成本为路径*系数+延时*系数
							timeCost += b*taskFromP[i + k2].time_in_out;
							disCost += k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
							robots[r_num].end_time += taskFromP[i + k2].distance;                          //调整当前机器人的时间轴
							robots[r_num].s_x = taskFromP[i + k2].park_x;            //调整当前机器人的坐标
							robots[r_num].s_y = taskFromP[i + k2].park_y;            //调整当前机器人的坐标
							robots[r_num].currLocationType = 2;                                 //调整当前机器人的位置类型
							map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_style = PARKCARIN;
						}
					}//if(mode==0)

				}//if (taskFromP[k2 + i].task_type == 1)
				///出库任务
				else if (taskFromP[i + k2].task_type == 0)
				{
					mode = 0;
					start_time_least = INF;
					start_time_max = 0;
					end_time_max = 0;
					for (m = 0; m<j; m++)    ///遍历j个机器人
					{
						int length;

						if (robots[m].currLocationType == 1)
						{
							length = map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_end;
						}
						else if (robots[m].currLocationType == 0)
						{
							length = map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_start;
						}
						else
						{
							length = findLengthFromP2P(&map_maze[robots[m].s_x][robots[m].s_y], &map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
						}
						start_time = robots[m].end_time + length;
						if (start_time <= taskFromP[i + k2].time_out)
						{
							if (end_time_max<robots[m].end_time)
							{
								r_num = m;
								end_time_max = robots[m].end_time;
							}
							mode = 1;
						}
						else if (mode == 0)
						{
							if (start_time<start_time_least)
							{
								start_time_least = start_time;
								r_num = m;
							}
						}
					}
					taskFromP[i + k2].robot_numbering = r_num;
					if (mode == 1)//如果到达车位的时间小于任务要求的出库时间，则更新当前时间
					{
						robots[r_num].end_time = taskFromP[i + k2].time_out + map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_end;
						taskFromP[i + k2].time_in_out = robots[r_num].end_time - taskFromP[i + k2].time_out;
						taskFromP[i + k2].distance = map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_end;
						taskFromP[i + k2].COST = b*taskFromP[i + k2].time_in_out + k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
						timeCost += b*taskFromP[i + k2].time_in_out;
						disCost += k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
						robots[r_num].s_x = end_node->s_x;
						robots[r_num].s_y = end_node->s_y;
						robots[r_num].currLocationType = 1;
						map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_style = PARK;
						if (map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].lengthInQueue == 1)
							QueuePathLengthTop.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
						else
							QueuePathLength.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
					}
					else if (mode == 0)
					{
						//time_waste = 1;
						robots[r_num].end_time = start_time_least;
						taskFromP[i + k2].time_out_true = robots[r_num].end_time;
						robots[r_num].end_time += map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_end;
						taskFromP[i + k2].time_in_out = robots[r_num].end_time - taskFromP[i + k2].time_out;
						taskFromP[i + k2].distance = map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_end;
						taskFromP[i + k2].COST = b*taskFromP[i + k2].time_in_out + k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
						timeCost += b*taskFromP[i + k2].time_in_out;
						disCost += k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
						robots[r_num].s_x = end_node->s_x;
						robots[r_num].s_y = end_node->s_y;
						robots[r_num].currLocationType = 1;
						map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_style = PARK;
						if (map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].lengthInQueue == 1)
							QueuePathLengthTop.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
						else
							QueuePathLength.push(&map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y]);
					}

				}//else if (taskFromP[i + k2].task_type == 0)

			}//for (k2 = 0; k2<j && (i + k2<CAR_N * 2); k2++) ///每次遍历j个任务
		}
		
		result = timeCost + disCost + robotsNumber*a;
		if ((result - robotsNumber*a) / p == CAR_N)
		{
			result = result - robotsNumber*a;
			robotsNumber = 0;
		}
			

		//cout << j << "  " << result << endl;

		if (result<result_least)
		{
			result_least = result;
			result_num = robotsNumber;
			timeCostFinal = timeCost;
			disCostFinal = disCost;
			for (int m1 = 0; m1 < CAR_N * 2; m1++)
			{
				taskOptical[m1] = taskFromP[m1];
				if (taskOptical[m1].task_type == 1)
				{
					taskOptical[m1].time_out_true = taskOptical[m1].coupleOut->time_out_true;
					taskOptical[m1].robot_numbering_out = taskOptical[m1].coupleOut->robot_numbering;
				}

			}
			
		}
		if (timeCost == timeCostTemp && disCost == disCostTemp)
		{
			isSaturate++;
			if (isSaturate == 2)
				time_waste = 0;
		}
		timeCostTemp = timeCost;
		disCostTemp = disCost;
	}
	


	/***************************************把结果直接输出到控制台*************************************/
	printf("%d %d %d\n",result_num,timeCostFinal,disCostFinal);
	for (i = 0; i < CAR_N * 2; i++)
	{
		if (taskOptical[i].task_type == 1)
		{
			printf("%d ",taskOptical[i].num);
			if (taskOptical[i].s_is_in_closetable == 1)
				printf("yes\n");
			else
			{
				printf("no ");

				printf("%d ", taskOptical[i].robot_numbering);				
				printf("%d ", taskOptical[i].time_in_true);
				AStarNode* tempNodeIn = &map_maze[taskOptical[i].park_x][taskOptical[i].park_y];
				printf("(%d,%d) ", start_node->s_x, start_node->s_y);
				vector<AStarNode*> tempVector;
				while (tempNodeIn->s_style != 'I')
				{
					tempVector.push_back(tempNodeIn);
					//printf("(%d,%d) ", tempNode->s_x, tempNode->s_y);
					tempNodeIn = tempNodeIn->s_parent;
				}
				for (int m2 = tempVector.size()-1; m2 >= 0; m2--)
				{
					tempNodeIn = tempVector[m2];
					printf("(%d,%d) ", tempNodeIn->s_x, tempNodeIn->s_y);
				}

				printf("%d ", taskOptical[i].robot_numbering_out);
				printf("%d ", taskOptical[i].time_out_true);
				printf("(%d,%d) ", taskOptical[i].park_x, taskOptical[i].park_y);
				AStarNode* tempNode = map_maze[taskOptical[i].park_x][taskOptical[i].park_y].s_parent_to_end;
				while (tempNode->s_style != 'E')
				{
					printf("(%d,%d) ", tempNode->s_x, tempNode->s_y);
					tempNode = tempNode->s_parent_to_end;
				}
				printf("(%d,%d)\n", end_node->s_x, end_node->s_y);
			}
		}
	}
	/*****************************************************************************************************/


	/*********************************************把结果保存到txt中***************************************/
	/*ofstream outfile;
	outfile.open("C:\\Users\\gxy\\Desktop\\result.txt");
	outfile << "YES\n";
	outfile << result_num << " " << timeCostFinal << " " << disCostFinal << endl;

	for (i = 0; i < CAR_N * 2; i++)
	{
		if (taskOptical[i].task_type == 1)
		{
			outfile << taskOptical[i].num << " ";
			if (taskOptical[i].s_is_in_closetable == 1)
				outfile << "yes\n" ;
			else
			{
				outfile << "no ";
				outfile << taskOptical[i].robot_numbering << " " << taskOptical[i].time_in_true << " ";
				AStarNode* tempNodeIn = &map_maze[taskOptical[i].park_x][taskOptical[i].park_y];
				outfile << "(" << start_node->s_x << "," << start_node->s_y << ") ";
				
				vector<AStarNode*> tempVector;
				while (tempNodeIn->s_style != 'I')
				{
					tempVector.push_back(tempNodeIn);
					tempNodeIn = tempNodeIn->s_parent;
				}
				for (int m2 = tempVector.size()-1; m2 >= 0; m2--)
				{
					tempNodeIn = tempVector[m2];
					outfile << "(" << tempNodeIn->s_x << "," << tempNodeIn->s_y << ") ";
				}
				outfile << taskOptical[i].robot_numbering_out << " ";
				outfile << taskOptical[i].time_out_true << " ";
				outfile << "(" << taskOptical[i].park_x << "," << taskOptical[i].park_y << ") ";
				AStarNode* tempNode = map_maze[taskOptical[i].park_x][taskOptical[i].park_y].s_parent_to_end;
				while (tempNode->s_style != 'E')
				{
					outfile << "(" << tempNode->s_x << "," << tempNode->s_y << ") ";
					tempNode = tempNode->s_parent_to_end;
				}
				outfile << "(" << end_node->s_x << "," << end_node->s_y << ")\n";
			}
		}
	}*/

	
	return 0;
}
