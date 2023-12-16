#include "planner.h"
#include <math.h>
#include <queue>
#include <vector>
#include <iostream>
#include <climits>
#include <stack>
#include <unordered_map>
#include <time.h>

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct cell
{
    int g, h, f;
    pair<int,int> parent;
    cell()
        : parent(-1, -1)
        , g(-1)
        , h(-1)
        , f(-1)
    {
    }
};

struct cell_heu
{
    int g;
    cell_heu()
        : g(-1)
    {
    }
};

typedef pair<int, pair<int, int>> heu_cell_data;
typedef pair<int, pair<int, int>> heu_cell_data_dji;
stack<pair<int,int>> Path;
vector<vector<cell>> grid2d;
vector<vector<cell_heu>> cost_grid;

bool isValid(int x, int y, int x_size, int y_size, int* map, int collision_thresh){
    if( ((int)map[GETMAPINDEX(x,y,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(x,y,x_size,y_size)] < collision_thresh) ){
        return true;
    }
    return false;
}

// Heuristic function using euclidean distance
/*double euclidean_distance(int x1, int y1, int x2, int y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
*/

void heuristic_fun(
    int* map, 
    int collision_thresh, 
    int x_size, 
    int y_size, 
    int robotposeX, 
    int robotposeY, 
    int target_steps, 
    int* target_traj, 
    int targetposeX, 
    int targetposeY
)
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int rx,ry;

    int goalx = robotposeX;
    int goaly = robotposeY;

    vector<vector<bool>> closelist(x_size+1, vector<bool>(y_size+1, false));
    priority_queue<heu_cell_data_dji, vector<heu_cell_data_dji>, greater<heu_cell_data_dji>> openlist;
    cost_grid.clear();
    cost_grid.resize(x_size+1, vector<cell_heu>(y_size+1));
    for (rx = 0; rx<x_size; rx++)
    {
        for(ry = 0; ry<y_size; ry++)
        {
            cost_grid[rx][ry].g = INT_MAX;
        }
    }

    robotposeX = (int)target_traj[target_steps-1];
    robotposeY = (int)target_traj[target_steps-1+target_steps];

    //defining robot start pose
    rx = robotposeX;
    ry = robotposeY;

    //start node
    cost_grid[rx][ry].g = 0;
    openlist.push(make_pair(0,make_pair(rx,ry)));
    int px, py, new_x, new_y, new_g;
    bool found_path = false;

    while (!openlist.empty() && !found_path)
    {
        bool next_node = false;
        while (!next_node)
        {
            heu_cell_data_dji node_new = openlist.top();
            openlist.pop();
            px = node_new.second.first;
            py = node_new.second.second;

            if(closelist[px][py] == false)
            {
                next_node = true;
            }
        }
        closelist[px][py] = true;

        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            new_x = px + dX[dir];
            new_y = py + dY[dir];

            if (new_x >= 1 && new_y >= 1 && new_x <= x_size && new_y <= y_size)
            {
                if (isValid(new_x,new_y,x_size,y_size,map,collision_thresh))
                {
                    if (closelist[new_x][new_y] == false)
                    {
                        new_g = cost_grid[px][py].g + (int)map[GETMAPINDEX(new_x,new_y,x_size,y_size)];

                        if (cost_grid[new_x][new_y].g == INT_MAX || cost_grid[new_x][new_y].g > new_g)
                        {
                            openlist.push(make_pair(new_g, make_pair(new_x,new_y)));
                            cost_grid[new_x][new_y].g = new_g;
                        }
                    }
                }
            }
        }
    }
}

stack<pair<int,int>> backtrack(vector<vector<cell>> &grid, int goalposeX, int goalposeY)
{
    int row = goalposeX;
    int col = goalposeY;
    stack<pair<int,int>> Path;

    while(!(grid[row][col].parent.first == row && grid[row][col].parent.second == col))
    {
        Path.push(make_pair(row,col));
        int temp_row = grid[row][col].parent.first;
        int temp_col = grid[row][col].parent.second;
        row = temp_row;
        col = temp_col;
    }
    Path.push(make_pair(row,col));
    return Path;
}

stack<pair<int, int>> astar (
    int *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int* action_ptr
)
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    //Define Openlist and Closelist
    priority_queue<heu_cell_data, vector<heu_cell_data>, greater<heu_cell_data>> OPENLIST;
    vector<vector<bool>> CLOSEDLIST(x_size+1, vector<bool> (y_size+1, false));
    vector<vector<cell>> grid2d(x_size+1, vector<cell>(y_size+1));

    int i,j;

    for (i = 0; i <x_size; i++) {
        for (j = 0; j <y_size; j++) {
            grid2d[i][j].f = INT_MAX;
            grid2d[i][j].g = INT_MAX;
            grid2d[i][j].h = INT_MAX;
            grid2d[i][j].parent = make_pair(-1, -1);
        }
    }

    //Define target
    int goalposeX = (int)target_traj[target_steps-1];
    int goalposeY = (int)target_traj[target_steps-1+target_steps];

    //Define robot start pose
    i = robotposeX;
    j = robotposeY;

    //create a start node in the graph
    grid2d[i][j].g = 0;
    grid2d[i][j].h = 0;
    grid2d[i][j].f = 0;
    grid2d[i][j].parent = make_pair(i, j);

    //add node's f,x,y to the openlist
    OPENLIST.push(make_pair(0,make_pair(i, j)));
    int r_x, r_y, newx, newy, newg, newh, newf;
    bool found = false;

    while (!OPENLIST.empty() && !found)
    {
        bool next = false;
        while (!next)
        {
            heu_cell_data node = OPENLIST.top();
            OPENLIST.pop();
            r_x = node.second.first;
            r_y = node.second.second;

            if(CLOSEDLIST[r_x][r_y] == false){
                next=true;
            }
        }
        //put the node's x,y to closed list if not already
        CLOSEDLIST[r_x][r_y] = true;
        //check if goal
        if (r_x == goalposeX && r_y == goalposeY)
        {
            found = true;
            return backtrack(grid2d, goalposeX, goalposeY);
        }

        //explore neighbors
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            newx = r_x + dX[dir];
            newy = r_y + dY[dir];

            if (newx >= 1 && newy >= 1 && newx <= x_size && newy <= y_size)
            {
                if (isValid(newx,newy,x_size,y_size,map,collision_thresh)) 
                {
                    if (CLOSEDLIST[newx][newy] == false)
                    {
                        newg = grid2d[r_x][r_y].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                        newh = cost_grid[newx][newy].g;
                        newf = newg + newh;

                        //updates the g-value if its greater than its previous g-val and pushes it to the open list
                        if (grid2d[newx][newy].g == INT_MAX || grid2d[newx][newy].g > newg)
                        {
                            OPENLIST.push(make_pair(newf, make_pair(newx,newy)));
                            grid2d[newx][newy].f = newf;
                            grid2d[newx][newy].g = newg;
                            grid2d[newx][newy].h = newh;
                            grid2d[newx][newy].parent = make_pair(r_x,r_y);
                        }
                    }
                }
            }
        }
    }
}

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    if (curr_time == 0)
    {
        heuristic_fun(map, 
                      collision_thresh, 
                      x_size, 
                      y_size, 
                      robotposeX, 
                      robotposeY, 
                      target_steps, 
                      target_traj, 
                      targetposeX, 
                      targetposeY);
        Path = astar(map,
                     collision_thresh,
                     x_size,
                     y_size,
                     robotposeX,
                     robotposeY,
                     target_steps,
                     target_traj,
                     targetposeX,
                     targetposeY,
                     action_ptr);
        pair<int, int> p = Path.top();
        Path.pop();
        robotposeX = p.first;
        robotposeY = p.second;
    }
    if (!Path.empty())
    {
        pair<int, int> p = Path.top();
        Path.pop();
        robotposeX = p.first;
        robotposeY = p.second;
    }
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}