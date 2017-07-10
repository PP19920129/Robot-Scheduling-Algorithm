#include "global.h"
#include "map.h"
#include "DI.h"

using namespace std;

int main()
{
	int arrayTh=-1;                  //�Ѿ����䵽�ڶ��ٸ�����
	AStarNode *curr_node;           // ��ǰ��
	int finished = 0;               //�Ƿ��ҵ�·��
	char maze[XMAX][YMAX];          //��ֵ�� map_maze I������㣬E�����յ㣬B�����ϰ���, X�����·��P����ͣ��λ
	//ע���ά�����һ��������������Ҳ����y���ڶ�����������Ҳ����x������Ҫ�ߵ�һ��
	//���ﶨ�����ֱ���ϵģ�x���ǵڼ��У�Ҳ����i��y����j
	int i, j, k2, m;
	int numofStart = 0, numofEnd = 0;
	int isSaturate = 0;

	/****************************************�����ͼ����*****************************************/
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
		cin>>task_H[i].num;  // ѭ����
		cin>>task_H[i].time_in;
		cin>>task_H[i].time_out;
		cin>>task_H[i].time_wait;
		cin >> task_H[i].mass;
	}

	/*************************************************************************************************/




	/********************************�жϵ�ͼ��ȷ����㵽ÿ���·��*************************************/
	//�ɵ���ĵ㰴�յ����ĳ���������open_table[]����
	//��������һ��Ķ��������е��ʼ���ķ�Χ������
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

			if (map_maze[i][j].s_style == STARTNODE)  // ��㣬û�п�������I�����
			{
				if (!(map_maze[i][j].s_x == 0 || map_maze[i][j].s_y == 0 || map_maze[i][j].s_x == XSIZE - 1 || map_maze[i][j].s_y == YSIZE - 1))
				{
					//printf("��㲻�ڵ�ͼ��Ե");
					printf("NO\n");
					return 0;
				}
				map_maze[i][j].s_to_start = 0;
				start_node = &(map_maze[i][j]);
				numofStart++;
			}
			if (map_maze[i][j].s_style == ENDNODE)    // �յ�
			{
				if (!(map_maze[i][j].s_x == 0 || map_maze[i][j].s_y == 0 || map_maze[i][j].s_x == XSIZE - 1 || map_maze[i][j].s_y == YSIZE - 1))
				{
					//printf("�յ㲻�ڵ�ͼ��Ե");
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
		//printf("�����������");
		//system("pause");
		printf("NO\n");
		return 0;
	}
	if (numofEnd != 1)
	{
		//printf("�յ���������");
		//system("pause");
		printf("NO\n");
		return 0;
	}

	//����ʹ��DI�㷨�õ���㵽������·��
	open_table[open_node_count++] = start_node;     // ���϶���open������;open_node_count��0��1
	start_node->s_is_in_opentable = 1;              // ����Ѽ���open��
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
				//printf("%d,%dͣ��λ������Ҫ��,һ����λ�в�ֹһ������\n", curr_node->s_x, curr_node->s_y);
				//printf("mode_P��%d  ", mode_P);
				printf("NO\n");
				return 0;
			}
			if (mode_P < 1)
			{
				//printf("%d,%dͣ��λ������Ҫ��,��λ���ϰ������������λ��Χ\n", curr_node->s_x, curr_node->s_y);
				//printf("mode_P��%d  ", mode_P);
				printf("NO\n");
				return 0;
			}
		}
		get_neighbors(curr_node);                       //����ÿ���㣬���������ڵ㣬curr_node����ǰ��
		curr_node->s_is_in_closetable = 1;              //�Ѿ���close������
		open_table[0] = open_table[--open_node_count];  //���һ����ŵ���һ���㣬Ȼ����жѵ���
		adjust_heap(0);                                 //������
		curr_node = open_table[0];
		if (open_node_count == 0)                       //û��·������
		{
			break;
		}

	}
	//printf("ͣ��λ��������%d  \n", P_NUM2);

	
	/**************************************************************************************/



	/**********************�յ㵽�������·����������̺�·���Ķ���*************************/
	/*
	�ټ���ÿ�㵽�յ�ľ��룬Ȼ��Ѽ���ÿ�㵽��ʼ��ľ���ͣ��Ѿ���ͷŵ����ȶ�������
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
		curr_node->s_is_in_closetable_to_end = 1;                            //�Ѿ���close������
		open_table_to_end[0] = open_table_to_end[--open_node_count_to_end];  //���һ����ŵ���һ���㣬Ȼ����жѵ���
		adjust_heap_to_end(0);                                               //������
		curr_node = open_table_to_end[0];
		if (open_node_count_to_end == 0)                                     //û��·������
		{
			break;
		}
	}
	if (P_NUM1 == P_NUM2&&P_NUM3==P_NUM1)
	{
		//printf("��ͼ����Ҫ��\n");
		printf("YES\n");

	}
	else
	{
		//printf("��ͼ������Ҫ��\n");
		printf("NO\n");
		return 0;  //û���ҵ�·��
	}

	end2start = findLengthFromP2P(start_node, end_node);

	/****************************************����ʱ������*****************************************/
	for (i = 0; i<CAR_N; i++)
	{
		taskFromP[i * 2].num = task_H[i].num;                      //�ڼ�����
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
	///����
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



	/*******************************��λ���յ���ںͳ��ڵľ����������******************************************/
	//���￼�Ƕ԰�֣������ĺ;���С�Ĵ��м�ֳ���������
	for (i = 0; i < XSIZE; i++)
	{
		for (j = 0; j < YSIZE; j++)
		{
			if (map_maze[i][j].s_style == 'P')
				QueuePathLength.push(&map_maze[i][j]);
		}
	}
	//���������С�ڳ�λ������ֻȡǰ����������λ���뵽�ó�λ��������
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
	//�����ڶ���������Ϣ��ʱ�򣬾ͽ���һ����������У��Գ���Ϊ��׼���������Դ���֪�����ص�ǰ30�����ж��أ���ô����
	//���ų�λ��ʱ�򣬾������ˣ�Ҳ�����и���׼�����������׼�ĳ�λ�Ͱ�����Top20���棬���Top20����û���ٰ��������
	/*************************************************************************************************/

	int start_time;               ///���ﵱǰ��������ʱ��
	int start_time_least = 20000000;         ///���ﵱǰ��������ʱ����Сʱ��
	int start_time_max = -1;           ///���ﵱǰ��������ʱ�����ʱ��
	int end_time_max = -1;             ///���ﵱǰ��������ʱ����Сʱ��
	int r_num;
	int result = 0;
	int result_least;
	int result_num;
	cur_nodes = &cur_nodes2;
	park_nodes = &park_nodes2;
	int mode;


	/**********************************************��ѭ��************************************************/
	int time_waste = 1;                       //�����ж��Ƿ���Ҫ�ټӻ������ˣ�������ڵ�time_waste�Ѿ���0�ˣ���û��Ҫ�ټӻ�������
	result_least = INF;
	//������������ѭ�������������
	for (j = 1; time_waste>0 && j<100; j=j+1)  ///����������j
	{
		/*******************************ÿ����������������������״̬************************************/
		int timeCost = 0;
		int disCost = 0;
		arrayTh = -1;
		for (i = 0; i<CAR_N * 2; i++)
		{
			//ÿ����������ĳЩ������ֵ
			taskFromP[i].park_x = 0;                      //�ڼ�����
			taskFromP[i].park_y = 0;
			taskFromP[i].s_is_in_closetable = 0;
			taskFromP[i].COST = 0;
			taskFromP[i].time_in_out = 0;
			taskFromP[i].distance = 0;
			taskFromP[i].time_in_true = taskFromP[i].time_in;
			taskFromP[i].time_out_true = taskFromP[i].time_out;
		}
		///��ʼ�������˵ĵ�ǰλ�ã�
		for (i = 0; i<j; i++)   ///�������
		{
			cur_nodes = &cur_nodes2;
			robots[i].s_x = start_node->s_x;  ///��ʾ�����
			robots[i].s_y = start_node->s_y;
			robots[i].end_time = 0;
			robots[i].currLocationType = 0;
		}
		robotsNumber = j;
		/********************************************************************************************/

		for (i = 0; i<CAR_N * 2; i += j)   
		{
			for (k2 = 0; k2<j && (i + k2<CAR_N * 2); k2++) ///ÿ�α���j������
			{
				//��ǰ������ܷ���Ҳ����û�����������������������ôһ���Ǹ���������
				//���û��������ô���������Ҳ�����ǳ�������
				//�����ͣ�����Ǿͷ��䳵λ������г�λ�ͷ��䲢����ִ���������û�г�λ��ֱ�Ӽ��Ϸ�ʱ��������һ��
				//�����ȡ����û�б�Ŀ��ܣ�ֻ��ִ�г�������
				if (taskFromP[k2 + i].s_is_in_closetable == 1)
				{
					if (taskFromP[k2 + i].task_type == 0)
						continue;
				}
				/*******************************ÿ�ΰ�������һ�������������*********************************/
				int constantInNum = 0;                                                       //�����������ĸ���
				if (k2 + i > arrayTh && taskFromP[k2 + i].task_type == 1)                        //����ó�λ��û�б�����
				{
					for (int pp = k2 + i; taskFromP[pp].task_type == 1; pp++)                //ͳ���Ե�ǰ����Ϊ��㣬�ж��ٸ������������
					{
						constantInNum++;
					}
					arrayTh = constantInNum + k2 + i;                                        //arrayTh��ʾ��ǰ�����˶��ٸ���λ
					if (constantInNum > QueuePathLength.size() + QueuePathLengthTop.size())  //�����λ�����ã�����abanNum����ǰ���صĳ�
					{
						int abanNum = constantInNum - QueuePathLength.size() - QueuePathLengthTop.size();
						//ÿ�ζԴ�k2+i��k2+i+constantInNum��Χ�ڵ���������
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
					

						//�����ص�
						for (int ii = k2 + i; ii < constantInNum + k2 + i; ii++)
						{
							if (taskFromP[ii].mass_num < abanNum)
							{
								taskFromP[ii].s_is_in_closetable = 1;
								taskFromP[ii].coupleOut->s_is_in_closetable = 1;
							}
							
						}////////////////�����ص�

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


						//�������
						for (int iii = abanNum; iii < constantInNum; iii++)
						{
							int jjj;
							for (jjj = k2 + i; jjj < constantInNum + k2 + i; jjj++)
							{
								if (taskFromP[jjj].mass_num == iii)
									break;
							}
							AStarNode *temp1;
							//������ʣ�µ�����С�����ȳ�λ����������ôֱ�������ȳ�λ�������
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
							}//���ȷ�����ĳ�λ
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
							}//����λ
							/**********************��֤������������Ƿ��Ǵ��ڷ�������******************/
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
					}/////////////////////////��λ�����ã��ȷ���һ�����ٷ���
					else//��λ����
					{

						//ÿ�ζԴ�k2+i��k2+i+constantInNum��Χ�ڵ���������
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
							//������ʣ�µ�����С�����ȳ�λ����������ôֱ�������ȳ�λ�������
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
							}//���ȷ�����ĳ�λ
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
							}//����λ
							/**********************��֤������������Ƿ��Ǵ��ڷ�������******************/
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
					
					/**********************��֤������������Ƿ��Ǵ��ڷ�������******************/
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
					for (m = 0; m<j; m++)  //�����������ÿ�������������Ļ�����
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
							mode = 1;              //mode=1��ʾ��������ڵ�ʱ��С������Ҫ������ʱ��
						}
						else if (mode == 0)
						{
							if (start_time<start_time_least)
							{
								start_time_least = start_time;
								r_num = m;
							}
						}
					}//���˷������˻����ˣ���һ������ִ���������
					taskFromP[i + k2].robot_numbering = r_num;
					if (mode == 1)//���������ڵ�ʱ��С������Ҫ������ʱ�䣬����µ�ǰʱ��
					{
						robots[r_num].end_time = taskFromP[i + k2].time_in;
						taskFromP[i + k2].time_in_out = 0;
						taskFromP[i + k2].distance = map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_to_start;
						taskFromP[i + k2].COST = k*taskFromP[i + k2].distance*taskFromP[i + k2].mass + b*taskFromP[i + k2].time_in_out; //������ɱ�Ϊ·��*ϵ��+��ʱ*ϵ��
						timeCost += b*taskFromP[i + k2].time_in_out;
						disCost += k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
						robots[r_num].end_time += taskFromP[i + k2].distance;                          //������ǰ�����˵�ʱ����
						robots[r_num].s_x = taskFromP[i + k2].park_x;            //������ǰ�����˵�����
						robots[r_num].s_y = taskFromP[i + k2].park_y;            //������ǰ�����˵�����
						robots[r_num].currLocationType = 2;                                 //������ǰ�����˵�λ������
						map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_style = PARKCARIN;
					}
					else if (mode == 0)
					{
						/**********************����֤������������Ƿ��Ǵ��ڷ�������******************/
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
							taskFromP[i + k2].COST = k*taskFromP[i + k2].distance*taskFromP[i + k2].mass + b*taskFromP[i + k2].time_in_out; //������ɱ�Ϊ·��*ϵ��+��ʱ*ϵ��
							timeCost += b*taskFromP[i + k2].time_in_out;
							disCost += k*taskFromP[i + k2].distance*taskFromP[i + k2].mass;
							robots[r_num].end_time += taskFromP[i + k2].distance;                          //������ǰ�����˵�ʱ����
							robots[r_num].s_x = taskFromP[i + k2].park_x;            //������ǰ�����˵�����
							robots[r_num].s_y = taskFromP[i + k2].park_y;            //������ǰ�����˵�����
							robots[r_num].currLocationType = 2;                                 //������ǰ�����˵�λ������
							map_maze[taskFromP[i + k2].park_x][taskFromP[i + k2].park_y].s_style = PARKCARIN;
						}
					}//if(mode==0)

				}//if (taskFromP[k2 + i].task_type == 1)
				///��������
				else if (taskFromP[i + k2].task_type == 0)
				{
					mode = 0;
					start_time_least = INF;
					start_time_max = 0;
					end_time_max = 0;
					for (m = 0; m<j; m++)    ///����j��������
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
					if (mode == 1)//������ﳵλ��ʱ��С������Ҫ��ĳ���ʱ�䣬����µ�ǰʱ��
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

			}//for (k2 = 0; k2<j && (i + k2<CAR_N * 2); k2++) ///ÿ�α���j������
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
	


	/***************************************�ѽ��ֱ�����������̨*************************************/
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
