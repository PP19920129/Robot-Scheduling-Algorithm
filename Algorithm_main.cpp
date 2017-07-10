#include "global.h"
#include "map.h"
#include "DI.h"

using namespace std;

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
	return 0;
}
