#ifndef DI_H_INCLUDED
#define DI_H_INCLUDED

#include "global.h"




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


// open_table��һ����
void adjust_heap(int nIndex)  //�����������nIndex
{
	int curr = nIndex;
	int child = curr * 2 + 1;   // �õ�����idx( �±��0��ʼ������������curr*2+1 )
	int parent = (curr - 1) / 2;  // �õ����ڵ�idx

	if (nIndex < 0 || nIndex >= open_node_count)  //���Խ���ˣ�����
	{
		return;
	}

	// ���µ���( Ҫ�Ƚ����Һ��Ӻ�cuur parent )
	while (child < open_node_count)
	{
		// С����  �õ���С���ӽڵ�

		if (child + 1 < open_node_count && open_table[child]->s_to_start  > open_table[child + 1]->s_to_start)
		{
			++child;//<span style="white-space:pre">              </span>// �ж����Һ��Ӵ�С
		}

		if (open_table[curr]->s_to_start <= open_table[child]->s_to_start)
		{
			break;
		}
		else
		{
			swap(child, curr);            // �����ڵ�//
			curr = child;               // ���жϵ�ǰ���ӽڵ�
			child = curr * 2 + 1;           // ���ж�����
		}
	}


	if (curr != nIndex)
	{
		return;
	}

	// ���ϵ���( ֻ��Ҫ�Ƚ�cuur child��parent )
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
void adjust_heap_to_end(int nIndex)  //�����������nIndex
{
	int curr = nIndex;
	int child = curr * 2 + 1;   // �õ�����idx( �±��0��ʼ������������curr*2+1 )
	int parent = (curr - 1) / 2;  // �õ����ڵ�idx

	if (nIndex < 0 || nIndex >= open_node_count_to_end)  //���Խ���ˣ�����
	{
		return;
	}

	// ���µ���( Ҫ�Ƚ����Һ��Ӻ�cuur parent )
	while (child < open_node_count_to_end)
	{
		// С����  �õ���С���ӽڵ�

		if (child + 1 < open_node_count_to_end && open_table_to_end[child]->s_to_end  > open_table_to_end[child + 1]->s_to_end)
		{
			++child;//<span style="white-space:pre">              </span>// �ж����Һ��Ӵ�С
		}

		if (open_table_to_end[curr]->s_to_end <= open_table_to_end[child]->s_to_end)
		{
			break;
		}
		else
		{
			swap_to_end(child, curr);            // �����ڵ�//
			curr = child;               // ���жϵ�ǰ���ӽڵ�
			child = curr * 2 + 1;           // ���ж�����
		}
	}


	if (curr != nIndex)
	{
		return;
	}

	// ���ϵ���( ֻ��Ҫ�Ƚ�cuur child��parent )
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


void adjust_heap_to_P1(int nIndex)  //�����������nIndex
{
	int curr = nIndex;
	int child = curr * 2 + 1;   // �õ�����idx( �±��0��ʼ������������curr*2+1 )
	int parent = (curr - 1) / 2;  // �õ����ڵ�idx

	if (nIndex < 0 || nIndex >= open_node_count_to_P1)  //���Խ���ˣ�����
	{
		return;
	}

	// ���µ���( Ҫ�Ƚ����Һ��Ӻ�cuur parent )
	while (child < open_node_count_to_P1)
	{
		// С����  �õ���С���ӽڵ�

		if (child + 1 < open_node_count_to_P1 && open_table_to_P1[child]->s_to_P1  > open_table_to_P1[child + 1]->s_to_P1)
		{
			++child;//<span style="white-space:pre">              </span>// �ж����Һ��Ӵ�С
		}

		if (open_table_to_P1[curr]->s_to_P1 <= open_table_to_P1[child]->s_to_P1)
		{
			break;
		}
		else
		{
			swap_to_P1(child, curr);            // �����ڵ�//
			curr = child;               // ���жϵ�ǰ���ӽڵ�
			child = curr * 2 + 1;           // ���ж�����
		}
	}


	if (curr != nIndex)
	{
		return;
	}

	// ���ϵ���( ֻ��Ҫ�Ƚ�cuur child��parent )
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

// �ж��ھӵ��Ƿ���Խ���open��
void insert_to_opentable(int x, int y, pAStarNode curr_node, int w)
{
	int i;
	//kTable = 0;

	if (curr_node->s_style == PARK)
	{
		if (map_maze[x][y].s_style != BARRIER && map_maze[x][y].s_style != PARK)        // �����ϰ���
		{
			if (!map_maze[x][y].s_is_in_closetable)   // ���ڱձ���
			{
				if (map_maze[x][y].s_is_in_opentable) // ��open����
				{
					// ��Ҫ�ж��Ƿ���һ�����Ż���·��
					if (map_maze[x][y].s_to_start > curr_node->s_to_start + w)    // ������Ż�
					{
						map_maze[x][y].s_to_start = curr_node->s_to_start + w;
						map_maze[x][y].s_parent = curr_node;

						//�ҵ�i
						for (i = 0; i < open_node_count; ++i)
						{
							if (open_table[i]->s_x == map_maze[x][y].s_x && open_table[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap(i);                   // ���������
					}
				}
				else              // ����open��
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
		if (map_maze[x][y].s_style != BARRIER)        // �����ϰ���
		{
			if (!map_maze[x][y].s_is_in_closetable)   // ���ڱձ���
			{
				if (map_maze[x][y].s_is_in_opentable) // ��open����
				{
					// ��Ҫ�ж��Ƿ���һ�����Ż���·��
					if (map_maze[x][y].s_to_start > curr_node->s_to_start + w)    // ������Ż�
					{
						map_maze[x][y].s_to_start = curr_node->s_to_start + w;
						map_maze[x][y].s_parent = curr_node;

						//�ҵ�i
						for (i = 0; i < open_node_count; ++i)
						{
							if (open_table[i]->s_x == map_maze[x][y].s_x && open_table[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap(i);                   // ���������
					}
				}
				else              // ����open��
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
		if (map_maze[x][y].s_style != BARRIER && map_maze[x][y].s_style != PARK)        // �����ϰ���
		{
			if (!map_maze[x][y].s_is_in_closetable_to_end)   // ���ڱձ���
			{
				if (map_maze[x][y].s_is_in_opentable_to_end) // ��open����
				{
					// ��Ҫ�ж��Ƿ���һ�����Ż���·��
					if (map_maze[x][y].s_to_end > curr_node->s_to_end + w)    // ������Ż�
					{
						map_maze[x][y].s_to_end = curr_node->s_to_end + w;
						map_maze[x][y].s_parent_to_end = curr_node;

						//�ҵ�i
						for (i = 0; i < open_node_count_to_end; ++i)
						{
							if (open_table_to_end[i]->s_x == map_maze[x][y].s_x && open_table_to_end[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_end(i);                   // ���������
					}
				}
				else              // ����open��
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
		if (map_maze[x][y].s_style != BARRIER)        // �����ϰ���
		{
			if (!map_maze[x][y].s_is_in_closetable_to_end)   // ���ڱձ���
			{
				if (map_maze[x][y].s_is_in_opentable_to_end) // ��open����
				{
					// ��Ҫ�ж��Ƿ���һ�����Ż���·��
					if (map_maze[x][y].s_to_end > curr_node->s_to_end + w)    // ������Ż�
					{
						map_maze[x][y].s_to_end = curr_node->s_to_end + w;
						map_maze[x][y].s_parent_to_end = curr_node;

						//�ҵ�i
						for (i = 0; i < open_node_count_to_end; ++i)
						{
							if (open_table_to_end[i]->s_x == map_maze[x][y].s_x && open_table_to_end[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_end(i);                   // ���������
					}
				}
				else              // ����open��
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

	//if (map_maze[x][y].s_style != BARRIER && map_maze[x][y].s_style != PARK && map_maze[x][y].s_style != PARKCARIN)        // �����ϰ���
	//�����ǰ�ĵ��ǳ�λ����ô�������򲻿����ǳ�λ�����ϰ�����ͣ�˳��ĳ�λ
	//�����ǰ���ǹ�������ô����ν��ֻҪ�����ϰ��Ϳ���
	if (curr_node->s_style == PARK || curr_node->s_style == PARKCARIN)
	{
		if (map_maze[x][y].s_style != BARRIER&& map_maze[x][y].s_style != PARK && map_maze[x][y].s_style != PARKCARIN)
		{
			if (!map_maze[x][y].s_is_in_closetable_to_P1)   // ���ڱձ���
			{
				if (map_maze[x][y].s_is_in_opentable_to_P1) // ��open����
				{
					// ��Ҫ�ж��Ƿ���һ�����Ż���·��
					if (map_maze[x][y].s_to_P1 > curr_node->s_to_P1 + w)    // ������Ż�
					{
						map_maze[x][y].s_to_P1 = curr_node->s_to_P1 + w;
						map_maze[x][y].s_parent_to_P1 = curr_node;

						//�ҵ�i
						for (i = 0; i < open_node_count_to_P1; ++i)
						{
							if (open_table_to_P1[i]->s_x == map_maze[x][y].s_x && open_table_to_P1[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_P1(i);                   // ���������
					}
				}
				else              // ����open��
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
			if (!map_maze[x][y].s_is_in_closetable_to_P1)   // ���ڱձ���
			{
				if (map_maze[x][y].s_is_in_opentable_to_P1) // ��open����
				{
					// ��Ҫ�ж��Ƿ���һ�����Ż���·��
					if (map_maze[x][y].s_to_P1 > curr_node->s_to_P1 + w)    // ������Ż�
					{
						map_maze[x][y].s_to_P1 = curr_node->s_to_P1 + w;
						map_maze[x][y].s_parent_to_P1 = curr_node;

						//�ҵ�i
						for (i = 0; i < open_node_count_to_P1; ++i)
						{
							if (open_table_to_P1[i]->s_x == map_maze[x][y].s_x && open_table_to_P1[i]->s_y == map_maze[x][y].s_y)
							{
								break;
							}
						}

						adjust_heap_to_P1(i);                   // ���������
					}
				}
				else              // ����open��
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

// �����ھ�
// ����������4���ھӽ��в���

void get_neighbors(pAStarNode curr_node)
{
	int x = curr_node->s_x;
	int y = curr_node->s_y;

	// �������4���ھӽ��д���
	//���쵱ǰ����ұߵĵ�
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
	// �������4���ھӽ��д���
	//���쵱ǰ����ұߵĵ�
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
	// �������4���ھӽ��д���
	//���쵱ǰ����ұߵĵ�
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

#endif // MAP_H_INCLUDED
