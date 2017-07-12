#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED

#include "DI.h"
#include "global.h"
#include <string>


//dijikstra
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



#endif