#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

// 定义节点类
class Node
{
public:
    int x, y;
    Node *parent;
    int g; // 从起始点到当前节点的步数
    int h; // 从当前节点到目标点的预估步数
    int f; // f = g + h

    Node(int x, int y) : x(x), y(y), parent(nullptr), g(0), h(0), f(0) {}
};

// 计算当前节点到目标节点的预估步数（这里使用曼哈顿距离）
int heuristic(Node *current, Node *goal)
{
    return abs(current->x - goal->x) + abs(current->y - goal->y);
}

// 定义比较函数对象用于优先队列
class CompareNodes
{
public:
    bool operator()(Node *a, Node *b)
    {
        return a->f > b->f;
    }
};

// A* 算法
int AStarAlgorithm(std::vector<std::vector<int>> &map, Node *start, Node *goal)
{
    std::priority_queue<Node *, std::vector<Node *>, CompareNodes> openSet;
    std::vector<Node *> closedSet;

    openSet.push(start);

    while (!openSet.empty())
    {
        Node *current = openSet.top();
        openSet.pop();

        // 到达目标节点
        if (current->x == goal->x && current->y == goal->y)
        {
            return current->g;
        }

        closedSet.push_back(current);

        // 计算当前节点相邻的节点
        int dx[] = {-1, 0, 1, 0};
        int dy[] = {0, 1, 0, -1};

        for (int i = 0; i < 4; i++)
        {
            int nextX = current->x + dx[i];
            int nextY = current->y + dy[i];

            // 检查是否在边界内且不是障碍物
            if (nextX >= 0 && nextX < map.size() && nextY >= 0 && nextY < map[0].size() && map[nextX][nextY] == 0)
            {
                Node *neighbor = new Node(nextX, nextY);
                neighbor->parent = current;
                neighbor->g = current->g + 1;
                neighbor->h = heuristic(neighbor, goal);
                neighbor->f = neighbor->g + neighbor->h;

                // 检查是否已经在关闭列表中
                bool inClosedSet = false;
                for (const auto &node : closedSet)
                {
                    if (node->x == neighbor->x && node->y == neighbor->y)
                    {
                        inClosedSet = true;
                        break;
                    }
                }

                if (!inClosedSet)
                {
                    // 检查是否已经在打开列表中
                    bool inOpenSet = false;
                    std::vector<Node *> openSetCopy;
                    while (!openSet.empty())
                    {
                        Node *temp = openSet.top();
                        openSet.pop();
                        openSetCopy.push_back(temp);
                        if (temp->x == neighbor->x && temp->y == neighbor->y)
                        {
                            inOpenSet = true;
                        }
                    }
                    for (const auto &node : openSetCopy)
                    {
                        openSet.push(node);
                    }

                    if (!inOpenSet)
                    {
                        openSet.push(neighbor);
                    }
                }
            }
        }
    }

    return -1; // 未找到路径
}

int main()
{
    // 定义地图
    std::vector<std::vector<int>> map = {
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};

    // 定
}