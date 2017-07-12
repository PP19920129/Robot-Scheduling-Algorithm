#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED

#include "DI.h"
#include "global.h"
#include <string>


//dijikstra
//map.h
/*********************˼·*************************
//Ҫ��ɵĹ����Ǹ�������������������򣨽ṹ�����飩�������Ƿ��䳵λ��ʹ�ɱ���ͣ��������еĳ�λ���ŷ���
��һ��˼·�ǰѳ�λ���գ�δʹ��&&�������յ����̣����򣬷��ڶ������棬ÿ�����������ѡ��һ��������ĳ�
�ڶ���˼·�Ǵ�M����λ�ѡ��N����ȫ���У�������Amn�з����ĳɱ����ȿ��ǵڶ���
**************************************************/
typedef struct TASKING
{
	//������Ϣ
	int num;
	int time_in;
	int time_in_true;
	int time_out;
	int time_out_true;
	int time_wait;
	int mass;
	int mass_num;

	//��������Ϣ
	int robot_number;                 //������������
	int robot_numbering;              //��ǰ�����˱��
	int robot_numbering_out;

	//������Ϣ
	int task_type;                    //0Ϊȡ����1Ϊͣ��

	//��λ��Ϣ
	int park_x;
	int park_y;
	//int numInParkArray;          //�ڵ�ǰ�ĳ�λ���ŷ�����ռ�˵ڼ���λ�ã���fullArr2D[360][N]��ĳһ�еĵڼ���

	//�ɱ���Ϣ
	int time_in_out;                 //������ʱ��Ϣ
	int distance;                    //��ǰ������ʻ���
	int s_is_in_closetable;          //��ǰ�����Ƿ����,1��ʾ������0������
	int COST;                        //��ǰ�����ܳɱ�
	struct TASKING *coupleOut;         //ÿһ��ͣ��������һ����Եĳ���������������һ��
	struct TASKING *coupleIn;        //��ʼ����ʱ���Ҫ���úã�ͣ�������Ӧ�ĳ����������ĸ�
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

/********************ʹ��DI�㷨��������֮������·��*************************/
int findLengthFromP2P(pAStarNode P1, pAStarNode P2)
{
	int i, j;
	//�Ȱ�P1�йص�����
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
		curr_node->s_is_in_closetable_to_P1 = 1;       // �Ѿ���close������
		open_table_to_P1[0] = open_table_to_P1[--open_node_count_to_P1];  // ���һ����ŵ���һ���㣬Ȼ����жѵ���
		adjust_heap_to_P1(0);               // ������
		curr_node = open_table_to_P1[0];
		if (open_node_count_to_P1 == 0)             // û��·������
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


/*******************************ȫ���з��亯��**************************/
void arrange(int * arr, int len, int num, int k, int *arr2D)
{
	int i, j;
	int arr2D_top = 0;
	if (k == num)  ///��������
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
	for (j = num; j<len; j++) ///������һ��λ�õ��������
	{
		swap2(&arr[j], &arr[num]);
		///num�������൱������㣬K�൱�����յ�
		arrange(arr, len, num + 1, k, arr2D);
		///��ԭ��������һ�α���
		swap2(&arr[j], &arr[num]);
	}
}
/*****************************************************************************/



/*******************�г����п��ܵĳ�λ��ȫ�������*********************/
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

				Parking[++Parking_top] = &maze[i][j];      //�����г�λ���͵ĵ��ָ��ȡ�����Ž�Parking����
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