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

/***********************���ͼ�йصĲ���***************************/
#define XMAX 100  //������Ҳ����y
#define YMAX 100  //������Ҳ����x
int XSIZE = 0;
int YSIZE = 0;
#define INF  200000000
#define  N  10000  //������Ŀ
/*****************************************************************/

/***********************���ͼ��ÿ�����йصĲ���***************************/
const char STARTNODE = 'I';
const char ENDNODE = 'E';
const char BARRIER = 'B';
const char PARK = 'P';
const char ROAD = 'X';
const char PARKCARIN = 'H';//hold
typedef struct AStarNode
{
	int s_x;                            // ����(�������·����Ҫ)
	int s_y;
	int s_to_start;                     //�˵㵽���ľ��룬ֻ����һ�Σ����ݱ���
	int s_to_end;                       //�˵㵽�յ�ľ��룬ֻ����һ�Σ����ݱ���
	int s_to_P1;                        //�˵㵽ĳ�������ľ��룬ÿ����һ���㣬����������
	char s_style;                       //������ͣ���ʼ�㣬�յ㣬�ϰ���ճ�λ���г���λ
	struct AStarNode * s_parent;               //���ڵ�
	struct AStarNode * s_parent_to_end;
	struct AStarNode * s_parent_to_P1;
	int s_is_in_closetable;             //�Ƿ���close����
	int s_is_in_closetable_to_end;
	int s_is_in_closetable_to_P1;
	int s_is_in_opentable;              //�Ƿ���open����
	int s_is_in_opentable_to_end;
	int s_is_in_opentable_to_P1;
	int lengthInQueue = 0;                //����0���ھ���Զ�Ķ���
}AStarNode, *pAStarNode;



int timeCostFinal = 0, disCostFinal = 0;
int timeCostTemp = 0, disCostTemp = 0;
typedef struct ROBOT2
{
	int num;
	int s_x;
	int s_y;
	int end_time;
	int currLocationType;    //���嵱ǰ�������ڳ��ڣ�1�����ǳ�λ��2��,�����ֻ��һ��ʼ��0��
	int numInParkArray;     //��ǰ�������ڳ�λ�����е���ţ��粻�ڳ�λ����Ϊ-1
}ROBOT;
ROBOT robots[N];
int robotsNumber = 0;





AStarNode *start_node;                  // ��ʼ��
AStarNode *end_node;                    // ������
int open_node_count = 0;                //open_table����±��
int open_node_count_to_end = 0;
int open_node_count_to_P1 = 0;
int P_NUM1 = 0, P_NUM2 = 0, P_NUM3 = 0;
AStarNode  map_maze[XMAX][YMAX];      // �ڵ�����
pAStarNode open_table[10000];              // open��ʵ���Ǹ�����ѣ������ͨ·�ϵĽṹ����ָ��
pAStarNode open_table_to_end[10000];
pAStarNode open_table_to_P1[10000];
AStarNode *cur_nodes;
AStarNode *park_nodes;

AStarNode  cur_nodes2;
AStarNode  park_nodes2;


/***********************�ɱ���صĲ������Ժ��Ϊ���룩***************************/
int k, p, a, b;



/***********************�복λ�����йصĲ���***************************/

pAStarNode Parking[100];  //���������ͣ��λ��ָ�룬�������г�λ��ȫ����
int Parking_top = -1;
int fullArr2D[500][N] = { 0 };
int c_num = 0;
int CAR_N = 0;
int end2start;//Di�ҵ���㵽�յ�ľ���
int weightArray[10000];

struct cmpPathLength
{
	//�Խṹ������������������أ��µ������������ԵĶ����ǽṹ��
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
pAStarNode path_stack[10000][100];    // ����·���Ķ�ջ
int top[10000] = { 0 };

#endif