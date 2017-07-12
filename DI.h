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
		if (map_maze[x][y].s_style != BARRIER && map_maze[x][y].s_style != PARK)        // 不是障碍物
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
		if (map_maze[x][y].s_style != BARRIER)        // 不是障碍物
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

#endif // MAP_H_INCLUDED
