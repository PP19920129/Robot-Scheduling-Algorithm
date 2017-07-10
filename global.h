#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <queue>
#include <algorithm> 
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

#ifndef GLOBAL_H
#define GLOBAL_H

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
int P_NUM1 = 0, P_NUM2 = 0, P_NUM3 = 0;
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

#endif