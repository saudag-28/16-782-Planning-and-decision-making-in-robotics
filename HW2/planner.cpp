/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 
#include <stack>
#include <unordered_map>
#include <queue>
#include <chrono>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

static double calc_plan_quality(double*** plan, int* planlength, int numofDOFs) 
{
    double total_dist = 0.0;
    for (int i = 0; i < *planlength - 1; i++)
    {
        double* curr_config = (*plan)[i];
        double* next_config = (*plan)[i+1];
        double curr_dist = 0.0;
        double curr_diff = 0.0;
        double min_joint_angle = 0.0;

        for (int joint_idx=0; joint_idx<numofDOFs; joint_idx++)
        {
            curr_diff = curr_config[joint_idx] - next_config[joint_idx];
            min_joint_angle = std::min(curr_diff, 2*M_PI - curr_diff);
            curr_dist += pow(std::fabs(min_joint_angle), 2);
        }
        total_dist += sqrt(curr_dist);
    }
    return total_dist;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

struct Node_rrt
{
	vector<double> angles_rrt;
	Node_rrt* parent; //a pointer to the parent node
};

int status = 0; //initialise it with 0
int trapped = 0;
int found_goal = 0;
int reached = 0;

// void printTree(vector<Node_rrt*>&tree)
// {
// 	// Printing the elements in the tree vector
//     for (const auto* node : tree) {
//         // Print the configuration of the current node
//         for (const auto& angle : node->angles_rrt) {
//             std::cout << angle << " ";
//         }
//         std::cout << std::endl;
//     }
// }

// void printTreeNodes(Node_rrt* node)
// {
// 	for (const auto& angle : node->angles_rrt)
// 	{
// 		cout << angle << " ";
// 	}
// 	cout << endl;
// }

bool isGoal(const std::vector<double>& config, double *armgoal_anglesV_rad)
{
	double distance_square = 0.0;
	double distance_euclid = 0.0;

	for (size_t i = 0; i<config.size(); ++i)
	{
		distance_euclid += (config[i] - armgoal_anglesV_rad[i])*(config[i] - armgoal_anglesV_rad[i]);
	}
	distance_square = sqrt(distance_euclid);
	if (distance_square < 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//generate random nodes for RRT
vector<double> generatenodes_rrt_goal_bias(int numofDOFs, double* armgoal_anglesV_rad)
{
	double goal_bias = 0.5;
	vector<double> rand_angles;
	rand_angles.reserve(numofDOFs);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> distribution(0.0, 2*M_PI);

	double randomValue = distribution(gen);
	if (randomValue < goal_bias)
	{
		for (int i = 0; i < numofDOFs; i++)
		{
			rand_angles.push_back(armgoal_anglesV_rad[i]);
		}
	}
	else
	{
		for (int i = 0; i < numofDOFs; i++)
		{
			rand_angles.push_back(distribution(gen));
		}
	}
	return rand_angles;
}

Node_rrt* nearest_node(vector<Node_rrt*> &tree, vector<double> &q)
{
	//point the nearestnode to null
	Node_rrt* q_near = nullptr;
	double minDistance = std::numeric_limits<double>::max();

	//iterate through the tree nodes and find the nearest node to q and save it in q_near
	for (Node_rrt* node : tree)
	{
		double distance = 0.0;
		for (size_t i = 0; i < node->angles_rrt.size(); ++i)
		{
			distance += std::pow(node->angles_rrt[i] - q[i], 2);
		}
		distance = sqrt(distance);

		if (distance < minDistance)
		{
			minDistance = distance;
			q_near = node;
		}
	}
	return q_near;
}

bool connect_rrt(Node_rrt* q_near, vector<double> &q_new_angles, vector<double> &valid_angles, int numofDOFs, double *map, int x_size, int y_size)
{		
    //straight interpolation between two nodes checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(q_near->angles_rrt[j] - q_new_angles[j]))
            distance = fabs(q_near->angles_rrt[j] - q_new_angles[j]);
    }
    int numofsamples = (int)(distance/(PI/10));

    int firstinvalidconf = 1;
	//valid_angles.clear();

    for (i = 0; i < numofsamples; i++){

		std::vector<double> sample_config;

        for(j = 0; j < numofDOFs; j++){
            sample_config.push_back(q_near->angles_rrt[j] + ((double)(i)/(numofsamples-1))*(q_new_angles[j] - q_near->angles_rrt[j]));
        }

		// Print the sample
        // std::cout << "Samples are: ";
        // for (const auto& angle : sample_config) {
        //     std::cout << angle << " ";
        // }
        // std::cout << std::endl;

        if(!IsValidArmConfiguration(sample_config.data(), numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;

			// if (i == 0) //first sample is invalid
			// {
			// 	cout << "trapped" << endl;
			// 	return false;
			// }
            
			//cout << "returns last valid angles just before hitting an obstacle" << endl;
			return false;
        }
		// cout << "working" << endl;
		valid_angles.clear();
        for (const auto& angle : sample_config) {
            valid_angles.push_back(angle);
        }
    } 
}

bool new_config(vector<double> &q, Node_rrt* q_near, vector<double> &valid_angles, int numofDOFs, double epsilon, double *map, int x_size, int y_size)
{
	vector<double> q_new_angles;

	//calculate arm configuration at the epsilon distance
	for (size_t i = 0; i < numofDOFs; ++i)
	{
		double dist_to_q = q[i] - q_near->angles_rrt[i];

		dist_to_q = (dist_to_q > 0) ? std::min(epsilon, dist_to_q) : std::max(-epsilon, dist_to_q);

		//calculate a q_new based on the epsilon distance 
		q_new_angles.push_back(q_near->angles_rrt[i] + dist_to_q);
	}

	// cout << "Newly generated angles are: " << endl;
	// for (size_t i = 0; i < q_new_angles.size(); ++i) {
    // 	std::cout << q_new_angles[i] << " ";
	// }
	// std::cout << std::endl; 
	// cout << '\n';

	//Interpolate from q_near to q
	if (connect_rrt(q_near, q_new_angles, valid_angles, numofDOFs, map, x_size, y_size))
	{
		// cout << "Valid angles are: " << endl;

		// for (size_t i = 0; i < valid_angles.size(); ++i) {
		// 	std::cout << valid_angles[i] << " ";
		// }
		// std::cout << std::endl; 
		// cout << '\n';

		return true;
	}
	else
	{
		// cout << "Trapped" << endl;
		return false;
	}
}

bool extend(vector<Node_rrt*> &tree, vector<double> &q, double epsilon, int numofDOFs, double* map, int x_size, int y_size, double *armgoal_anglesV_rad, Node_rrt* goalNode)
{
	vector<double> valid_angles;
	// cout << "First, lets print the tree: " << endl;
	// printTree(tree);
	// cout << '\n';

	//find nearest vertex in the tree to q - q_near
	Node_rrt* q_near = nearest_node(tree, q);

	// cout << "Now let's extend the tree from this q_near towards the random node q by " << epsilon << " distance" << endl;
	// cout << '\n';

	//Extend the tree from the nearest node towards the random node and check validity of the new node. Do LPM

	if (new_config(q, q_near, valid_angles, numofDOFs, epsilon, map, x_size, y_size))
	{
		Node_rrt* newNode = new Node_rrt{valid_angles, q_near};
		tree.push_back(newNode);

		// cout << "Adding q_new node to the tree: " << endl;
		// printTreeNodes(newNode);
		// cout <<  "With parents: " << endl;
		// printTreeNodes(q_near);
		// cout << '\n';

		if (isGoal(newNode->angles_rrt, armgoal_anglesV_rad))
		{
			goalNode->parent = newNode;
			found_goal = 1;
			return true;
		}
		return true;
	}
	else
	{
		// cout << "Trapped, therefore no node added to the tree." << endl;
		return false;
	}
}

vector<Node_rrt*> build_rrt(double *armstart_anglesV_rad, double *armgoal_anglesV_rad, int numofDOFs, int k, double epsilon, double* map, int x_size, int y_size)
{
	//declare a pointer vector tree which will point to the child nodes of the tree
	vector<Node_rrt*> tree;
	vector<double> armStart_angles;
	vector<double> armGoal_angles;

	//save startConfig angles and goalConfig angles in a vector depending on the number of DOFs
	for (int i = 0; i < numofDOFs; i++)
	{
		armStart_angles.push_back(armstart_anglesV_rad[i]);
		armGoal_angles.push_back(armgoal_anglesV_rad[i]);
	}

	Node_rrt* goalNode = new Node_rrt{armGoal_angles, nullptr};

	//initialise the empty tree with qinit node
	Node_rrt* startNode = new Node_rrt{armStart_angles, nullptr};
	tree.push_back(startNode);

	// cout << "Hello! Lets print the tree" << endl;
	// printTree(tree);
	// cout << '\n';

	for (int j = 0; j < k; j++)
	{
		// cout << "A random node:" << endl;
		vector<double> q_rand = generatenodes_rrt_goal_bias(numofDOFs, armgoal_anglesV_rad);

		extend(tree, q_rand, epsilon, numofDOFs, map, x_size, y_size, armgoal_anglesV_rad, goalNode);
		//delete q_rand;
		
		if (found_goal)
		{
			// cout << "Found the goal!" << endl;
			cout << "Tree size if goal found early: " << tree.size() << endl;
			break;
		}
	
		// cout << '\n';
		// cout << "j is " << j << endl;
		// cout << '\n';
	}

	cout << "Tree size if goal not found early: " << tree.size() << endl;
	
	//return path
	vector<Node_rrt*> final_path;
	if (!found_goal)
	{
		// cout << "No path found" << endl;
		//printTree(tree);
		for (Node_rrt* node : tree)
		{
			if (isGoal(node->angles_rrt, armgoal_anglesV_rad))
			{
				goalNode->parent = node;
				break;
			}
		}
	}

	// cout << "goal node parent is: " << goalNode->parent << endl;

	if (goalNode != nullptr)
	{
		while (goalNode != nullptr)
		{
			final_path.push_back(goalNode);
			goalNode = goalNode->parent;
		}
		reverse(final_path.begin(), final_path.end());
	}

	// cout << "Final path is: " << endl;
	// for (const Node_rrt* node : final_path) {
    //     std::cout << "Node Angles: ";
    //     for (double angle : node->angles_rrt) {
    //         std::cout << angle << " ";
    //     }
    //     std::cout << std::endl;
    // } 
	
	return final_path;
}

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    double epsilon = 1;
	int k = 10000;
	vector<Node_rrt*> path = build_rrt(armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, k, epsilon, map, x_size, y_size);

	if (path.empty())
	{
		cout << "No path found" << endl;
		return;
	}

	*planlength = (int)path.size();
	*plan = (double**)malloc(path.size()*sizeof(double*));

	for (int i = 0; i < path.size(); i++)
	{
		(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
		for (int j = 0; j < numofDOFs; j++)
		{
			(*plan)[i][j] = path[i]->angles_rrt[j];
		}
		//delete path[i];
	}
	//path.clear();
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

//int s = 0; //to save if REACHED, ADVANCED or TRAPPED

struct Node_rrt_c
{
	vector<double> angles_rrt_c;
	Node_rrt_c* parent_rrt_c; //a pointer to the parent node
};

enum ExtendResult {
    Trapped,
    Reached,
    Advanced
};

void printTree_rrt_c(vector<Node_rrt_c*> &tree)
{
	// Printing the elements in the tree vector
    for (const auto* node : tree) {
        // Print the configuration of the current node
        for (const auto& angle : node->angles_rrt_c) {
            std::cout << angle << " ";
        }
        std::cout << std::endl;
    }
}

void printTreeNodes_rrt_c(Node_rrt_c* node)
{
	for (const auto& angle : node->angles_rrt_c)
	{
		cout << angle << " ";
	}
	cout << endl;
}

double distance(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return std::sqrt(sum);
}

//generate random nodes for RRT_Connect
vector<double> generatenodes_rrt_c(int numofDOFs)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> distribution(0.0, 2*M_PI);

	vector<double> rand_angles;
	rand_angles.reserve(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) {
        rand_angles.push_back(distribution(gen));
    }
	return rand_angles;
}

Node_rrt_c* nearest_node_2(vector<Node_rrt_c*> &tree, vector<double> &q)
{
	//point the nearestnode to null
	Node_rrt_c* q_near = nullptr;
	double minDistance = std::numeric_limits<double>::max();

	//iterate through the tree nodes and find the nearest node to q and save it in q_near
	for (Node_rrt_c* node : tree)
	{
		double distance = 0.0;
		for (size_t i = 0; i < node->angles_rrt_c.size(); ++i)
		{
			distance += std::pow(node->angles_rrt_c[i] - q[i], 2);
		}
		distance = sqrt(distance);

		if (distance < minDistance)
		{
			minDistance = distance;
			q_near = node;
		}
	}
	
	return q_near;
}

bool connect_rrt_c(Node_rrt_c* q_near_a, vector<double> &q_new_angles, vector<double> &valid_angles, int numofDOFs, double *map, int x_size, int y_size)
{		
    //straight interpolation between two nodes checking for the validity of samples

	double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(q_near_a->angles_rrt_c[j] - q_new_angles[j]))
            distance = fabs(q_near_a->angles_rrt_c[j] - q_new_angles[j]);
    }

    int numofsamples = (int)(distance/(PI/20));

    int firstinvalidconf = 1;

    for (int i = 0; i < numofsamples; i++){

		std::vector<double> sample_config;

        for(int j = 0; j < numofDOFs; j++){
            sample_config.push_back(q_near_a->angles_rrt_c[j] + ((double)(i)/(numofsamples-1))*(q_new_angles[j] - q_near_a->angles_rrt_c[j]));
        }

		// Print the sample
        // std::cout << "Samples are: ";
        // for (const auto& angle : sample_config) {
        //     std::cout << angle << " ";
        // }
        // std::cout << std::endl;

        if(!IsValidArmConfiguration(sample_config.data(), numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
        
			return false;
        }
		// cout << "working" << endl;
		valid_angles.clear();
        for (const auto& angle : sample_config) {
            valid_angles.push_back(angle);
        }
    } 
}

bool new_config_rrt_c(vector<double> &q, Node_rrt_c* q_near_a, vector<double> &valid_angles, double epsilon, int numofDOFs, double *map, int x_size, int y_size)
{
	vector<double> q_new_angles;
	
	//calculate arm configuration at the epsilon distance
	for (size_t i = 0; i < numofDOFs; ++i)
	{
		double dist_to_q = q[i] - q_near_a->angles_rrt_c[i];

		dist_to_q = (dist_to_q > 0) ? std::min(epsilon, dist_to_q) : std::max(-epsilon, dist_to_q);

		//calculate a q_new based on the epsilon distance 
		q_new_angles.push_back(q_near_a->angles_rrt_c[i] + dist_to_q);
	}

	// cout << "Newly generated angles are: " << endl;
	// for (size_t i = 0; i < q_new_angles.size(); ++i) {
    // 	std::cout << q_new_angles[i] << " ";
	// }
	// std::cout << std::endl; 
	// cout << '\n';

	//Interpolate from q_near to q_new_angles
	if (connect_rrt_c(q_near_a, q_new_angles, valid_angles, numofDOFs, map, x_size, y_size))
	{
		// cout << "Valid angles are: " << endl;

		// for (size_t i = 0; i < valid_angles.size(); ++i) {
		// 	std::cout << valid_angles[i] << " ";
		// }
		// std::cout << std::endl; 
		// cout << '\n';

		return true;
	}
	else
	{
		// cout << "Trapped" << endl;
		return false;
	}
}

ExtendResult extend_rrt_c(vector<Node_rrt_c*> &tree, vector<double> &q, double epsilon, int numofDOFs, double* map, int x_size, int y_size)
{
	vector<double> valid_angles;

	//find nearest vertex in the tree to q - q_near
	Node_rrt_c* q_near_a = nearest_node_2(tree, q);

	// cout << "q_near is: " << endl;
	// printTreeNodes_rrt_c(q_near_a);

	if (new_config_rrt_c(q, q_near_a, valid_angles, epsilon, numofDOFs, map, x_size, y_size))
	{
		Node_rrt_c* newNode = new Node_rrt_c{valid_angles, q_near_a};
		tree.push_back(newNode);

		// cout << "Adding q_new node to the tree: " << endl;
		// printTreeNodes_rrt_c(newNode);
		// cout <<  "With parents: " << endl;
		// printTreeNodes_rrt_c(q_near_a);
		// cout << '\n';

		if (distance(valid_angles, q) < epsilon)
		{
			return Reached;
		}
		else{
			return Advanced;
		}
	}
	return Trapped;
}

ExtendResult connect(vector<Node_rrt_c*> &tree, vector<double> &q, double epsilon_rrtc, int numofDOFs, double* map, int x_size, int y_size)
{
	ExtendResult s;
	do
	{
		s = extend_rrt_c(tree, q, epsilon_rrtc, numofDOFs, map, x_size, y_size);
		// cout << "s inside is: " << s << endl;
	} while (s == Advanced);
	// cout << "s is: " << s << endl;
	return s;
}

vector<Node_rrt_c*> rrt_connect_planner(double *armstart_anglesV_rad, double *armgoal_anglesV_rad, int numofDOFs, int k_rrtc, double epsilon_rrtc, double* map, int x_size, int y_size)
{
	vector<Node_rrt_c*> tree_a;
	vector<Node_rrt_c*> tree_b;
	vector<double> armStart_angles_rrt_c;
	vector<double> armGoal_angles_rrt_c;

	//save startConfig angles and goalConfig angles in a vector depending on the number of DOFs
	for (int i = 0; i < numofDOFs; i++)
	{
		armStart_angles_rrt_c.push_back(armstart_anglesV_rad[i]);
		armGoal_angles_rrt_c.push_back(armgoal_anglesV_rad[i]);
	}

	Node_rrt_c* goalNode_rrt_c = new Node_rrt_c{armGoal_angles_rrt_c, nullptr};
	Node_rrt_c* startNode_rrt_c = new Node_rrt_c{armStart_angles_rrt_c, nullptr};
	tree_a.push_back(startNode_rrt_c);
	tree_b.push_back(goalNode_rrt_c);

	for (int i = 0; i < k_rrtc; ++i)
	{
		vector<double> q_rand = generatenodes_rrt_c(numofDOFs);

		//if(extend_rrt_c(tree_a, q_rand_c, q_new_rrt_c, epsilon_rrtc, numofDOFs, map, x_size, y_size))
		if (extend_rrt_c(tree_a, q_rand, epsilon_rrtc, numofDOFs, map, x_size, y_size) != Trapped)
		{
			if (connect(tree_b, tree_a.back()->angles_rrt_c, epsilon_rrtc, numofDOFs, map, x_size, y_size) == Reached)
			{

				cout << "Tree A size is: " << tree_a.size() << endl;
				cout << "Tree B size is: " << tree_b.size() << endl;

				vector<Node_rrt_c*> path_rrtc;

                Node_rrt_c* currentNode_rrtc = tree_a.back();
                while (currentNode_rrtc != nullptr) {
                    path_rrtc.insert(path_rrtc.begin(), currentNode_rrtc);
                    currentNode_rrtc = currentNode_rrtc->parent_rrt_c;
                }
				currentNode_rrtc = tree_b.back();
				while (currentNode_rrtc != nullptr) {
					path_rrtc.push_back(currentNode_rrtc);
					currentNode_rrtc = currentNode_rrtc->parent_rrt_c;
				}
				// cout << "Final path is: " << endl;
				// for (const Node_rrt_c* node : path_rrtc) {
				// 	std::cout << "Node Angles: ";
				// 	for (double angle : node->angles_rrt_c) {
				// 		std::cout << angle << " ";
				// 	}
				// 	std::cout << std::endl;
				// } 

				//std::reverse(path_rrtc.begin(), path_rrtc.end());
                return path_rrtc;
			}	
		}
		// cout << "Before swapping" << endl;
		// cout << "Tree A is: " << endl;
		// printTree_rrt_c(tree_a);
		// cout << '\n';
		// cout << "Tree B is: " << endl;
		// printTree_rrt_c(tree_b);
		// cout << '\n';

		swap(tree_a, tree_b);

		// cout << "After swapping" << endl;
		// cout << "Tree A is: " << endl;
		// printTree_rrt_c(tree_a);
		// cout << '\n';
		// cout << "Tree B is: " << endl;
		// printTree_rrt_c(tree_b);
		// cout << '\n';

		// cout << "i is: " << i << endl;
	}
	return {};
}

static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	int k_rrtc = 500;
	double epsilon_rrtc = 1;
    vector<Node_rrt_c*> path = rrt_connect_planner(armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, k_rrtc, epsilon_rrtc, map, x_size, y_size);

	if (path.empty())
	{
		cout << "No path found" << endl;
		return;
	}

	*planlength = (int)path.size();
	*plan = (double**)malloc(path.size()*sizeof(double*));

	for (int i = 0; i < path.size(); i++)
	{
		(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
		for (int j = 0; j < numofDOFs; j++)
		{
			(*plan)[i][j] = path[i]->angles_rrt_c[j];
		}
	}
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

int reached_rrt_s = 0;

struct Node_rrt_s
{
	vector<double> angles_rrt_s;
	Node_rrt_s* parent_rrt_s; //a pointer to the parent node
	double cost;
};

void printTree_rrt_s(vector<Node_rrt_s*>&tree)
{
	// Printing the elements in the tree vector
    for (const auto* node : tree) {
        // Print the configuration of the current node
        for (const auto& angle : node->angles_rrt_s) {
            std::cout << angle << " ";
        }
        std::cout << std::endl;
    }
}

// void printTreeNodes_rrt_s(Node_rrt_s* node)
// {
// 	for (const auto& angle : node->angles_rrt_s)
// 	{
// 		cout << angle << " ";
// 	}
// 	cout << endl;
// }

Node_rrt_s* generatenodes_rrt_s(int numofDOFs)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> distribution(0.0, 2*M_PI);

	vector<double> rand_angles;
	rand_angles.reserve(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) {
        rand_angles.push_back(distribution(gen));
    }
	return new Node_rrt_s{rand_angles, nullptr};
}

//define a goal region
// cont double EPSILON = 0.7
// for (all numofDOFs)
//		sum += (difference between the corresponding joint)^2
// return sqrt(sum)

bool isGoal_rrt_s(const std::vector<double>& config, double *armgoal_anglesV_rad)
{
	double distance_square = 0.0;
	double distance_euclid = 0.0;

	for (size_t i = 0; i<config.size(); ++i)
	{
		distance_euclid += (config[i] - armgoal_anglesV_rad[i])*(config[i] - armgoal_anglesV_rad[i]);
	}
	distance_square = sqrt(distance_euclid);
	if (distance_square < 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool ObstacleFree(Node_rrt_s* x_near, Node_rrt_s* q_new_rrt_s, int numofDOFs, double *map, int x_size, int y_size)
{		
    //straight interpolation between two nodes checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(q_new_rrt_s->angles_rrt_s[j] - x_near->angles_rrt_s[j]))
            distance = fabs(q_new_rrt_s->angles_rrt_s[j] - x_near->angles_rrt_s[j]);
    }
    int numofsamples = (int)(distance/(PI/20));

    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){

		std::vector<double> sample_config;

        for(j = 0; j < numofDOFs; j++){
			sample_config.push_back(q_new_rrt_s->angles_rrt_s[j] + ((double)(i)/(numofsamples-1))*(x_near->angles_rrt_s[j] - q_new_rrt_s->angles_rrt_s[j]));
        }

		// // Print the sample
        // std::cout << "Samples are: ";
        // for (const auto& angle : sample_config) {
        //     std::cout << angle << " ";
        // }
        // std::cout << std::endl;

        if(!IsValidArmConfiguration(sample_config.data(), numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            // printf("ERROR: Cannot Connect\n");
			return false;
        }
    }
	// cout << "Can connect" << endl;    
	return true;
}

Node_rrt_s* nearest_node_rrt_s(vector<Node_rrt_s*> &tree_rrt_s, Node_rrt_s* q_rrt_s)
{
	//point the nearestnode to null
	Node_rrt_s* q_near_rrt_s = nullptr;
	double minDistance = std::numeric_limits<double>::max();

	//iterate through the tree nodes and find the nearest node to q and save it in q_near
	for (Node_rrt_s* node : tree_rrt_s)
	{
		double distance = 0.0;
		for (size_t i = 0; i < node->angles_rrt_s.size(); ++i)
		{
			distance += std::pow(node->angles_rrt_s[i] - q_rrt_s->angles_rrt_s[i], 2);
		}
		distance = sqrt(distance);

		if (distance < minDistance)
		{
			minDistance = distance;
			q_near_rrt_s = node;
		}
	}
	
	return q_near_rrt_s;
}

double rrt_star_radius(vector<Node_rrt_s*> &tree_rrt_s, double epsilon_rrt_s, int numofDOFs)
{
	double r;
	double delta = pow(M_PI, 2.5)/3.32335;
	int gamma = 2;

	if (tree_rrt_s.size() > 1)
	{
		r = std::min(pow( gamma / delta * log(tree_rrt_s.size()) / tree_rrt_s.size(), 1.0/(double)numofDOFs), epsilon_rrt_s);
	}
	else
	{
		r = epsilon_rrt_s;
	}
	return r;
}

double calculate_distance(Node_rrt_s* sample_node_rrt_s, Node_rrt_s* neighbor_node_rrt_s)
{
	//cout << "In euclid" << endl;
	double distance_square = 0.0;
	double distance_euclid = 0.0;

	for (size_t i = 0; i<sample_node_rrt_s->angles_rrt_s.size(); ++i)
	{
		distance_euclid += (sample_node_rrt_s->angles_rrt_s[i] - neighbor_node_rrt_s->angles_rrt_s[i])*(sample_node_rrt_s->angles_rrt_s[i] - neighbor_node_rrt_s->angles_rrt_s[i]);
	}
	distance_square = sqrt(distance_euclid);
	return distance_square;
}

vector<Node_rrt_s*> Near(vector<Node_rrt_s*> &tree_rrt_s, Node_rrt_s* q_new_rrt_s, double r)
{
	vector<Node_rrt_s*> nearnodes;

	for (Node_rrt_s* node: tree_rrt_s)
	{
		double distance = calculate_distance(node, q_new_rrt_s);
		if (distance <= r)
		{
			nearnodes.push_back(node);
		}
	}
	return nearnodes;
}

double Cost(Node_rrt_s* node) {
    // Calculate the cost to reach this node from the start.
    double cost_val = 0;
    Node_rrt_s* current = node;
    while (current->parent_rrt_s != nullptr) {
        cost_val += current->cost; // Add the cost from the parent to the current node
        current = current->parent_rrt_s; // Move to the parent node
    }
    return cost_val;
}

double Line(const Node_rrt_s* from, const Node_rrt_s* to, int numofDOFs) {
    // Calculate the cost of a direct line between two nodes (joint configurations).

    // Calculate Euclidean distance in joint space
    double distance = 0.0;
    for (size_t i = 0; i < numofDOFs; ++i) {
        // Calculate squared difference for each joint angle and sum them up
        distance += std::pow(from->angles_rrt_s[i] - to->angles_rrt_s[i], 2);
    }

    // Take the square root to get the Euclidean distance
    return std::sqrt(distance);
}

Node_rrt_s* Steer (Node_rrt_s* nearest, Node_rrt_s* x, int numofDOFs, double epsilon_rrt_s)
{
	vector<double> q_new_angles_rrt_s;
	vector<double> valid_angles_rrt_s;

	//calculate arm configuration at the epsilon distance
	for (size_t i = 0; i < numofDOFs; ++i)
	{
		double dist_to_q = x->angles_rrt_s[i] - nearest->angles_rrt_s[i];

		dist_to_q = (dist_to_q > 0) ? std::min(epsilon_rrt_s, dist_to_q) : std::max(-epsilon_rrt_s, dist_to_q);

		//calculate a q_new based on the epsilon distance 
		q_new_angles_rrt_s.push_back(nearest->angles_rrt_s[i] + dist_to_q);
	}

	// cout << "Newly generated angles are: " << endl;
	// for (size_t i = 0; i < q_new_angles_rrt_s.size(); ++i) {
    // 	std::cout << q_new_angles_rrt_s[i] << " ";
	// }
	// std::cout << std::endl; 
	// cout << '\n';

	return new Node_rrt_s{q_new_angles_rrt_s, nullptr};
}

void extend_rrt_s(vector<Node_rrt_s*> &tree_rrt_s, Node_rrt_s* q_rrt_s, double epsilon_rrt_s, int numofDOFs, double* map, int x_size, int y_size)
{
	Node_rrt_s* q_near_rrt_s = nearest_node_rrt_s(tree_rrt_s, q_rrt_s);

	// cout << "q_near is:" << endl;
	// printTreeNodes_rrt_s(q_near_rrt_s);
	// cout << '\n';

	Node_rrt_s* q_new_rrt_s = Steer(q_near_rrt_s, q_rrt_s, numofDOFs, epsilon_rrt_s);

	if (ObstacleFree(q_near_rrt_s, q_new_rrt_s, numofDOFs, map, x_size, y_size))
	{
		//q_new_rrt_s->parent_rrt_s = q_near_rrt_s;
		//tree_rrt_s.push_back(q_new_rrt_s);
		double c;

		double r = rrt_star_radius(tree_rrt_s, epsilon_rrt_s, numofDOFs);

		// cout << "radius is: " << r << endl;
		// cout << '\n';

		Node_rrt_s* min_node = q_near_rrt_s;
		vector<Node_rrt_s*> near_nodes = Near(tree_rrt_s, q_new_rrt_s, r);

		for (Node_rrt_s* near : near_nodes)
		{
			if (ObstacleFree(near, q_new_rrt_s, numofDOFs, map, x_size, y_size))
			{
				c = Cost(near) + Line(near, q_new_rrt_s, numofDOFs);   //look here if something goes wrong
				if (c < Cost(q_new_rrt_s))
				{
					min_node = near;
				}
			}
		}
		q_new_rrt_s->parent_rrt_s = min_node;
		q_new_rrt_s->cost = c;
		tree_rrt_s.push_back(q_new_rrt_s);

		for (Node_rrt_s* near : near_nodes)
		{
			if (near != min_node && ObstacleFree(near, q_new_rrt_s, numofDOFs, map, x_size, y_size) && Cost(near) > Cost(q_new_rrt_s) + Line(q_new_rrt_s, near, numofDOFs))
			{
				near->parent_rrt_s = nullptr;  //confirm this again - todo
				near->parent_rrt_s = q_new_rrt_s;
			}
		}
	}
}

vector<Node_rrt_s*> build_rrt_s(double *armstart_anglesV_rad, double *armgoal_anglesV_rad, int numofDOFs, int k_rrt_s, double epsilon_rrt_s, double* map, int x_size, int y_size)
{
	//declare a pointer vector tree which will point to the child nodes of the tree
	vector<Node_rrt_s*> tree_rrt_s;
	vector<double> armStart_angles_rrt_s;
	vector<double> armGoal_angles_rrt_s;
	Node_rrt_s* q_rand_rrt_s;

	//save startConfig angles and goalConfig angles in a vector depending on the number of DOFs
	for (int i = 0; i < numofDOFs; i++)
	{
		armStart_angles_rrt_s.push_back(armstart_anglesV_rad[i]);
		armGoal_angles_rrt_s.push_back(armgoal_anglesV_rad[i]);
	}

	Node_rrt_s* goalNode_rrt_s = new Node_rrt_s{armGoal_angles_rrt_s, nullptr};

	//initialise the empty tree with qinit node
	Node_rrt_s* startNode_rrt_s = new Node_rrt_s{armStart_angles_rrt_s, nullptr, 0};
	tree_rrt_s.push_back(startNode_rrt_s);

	// cout << "Hello! Lets print the tree" << endl;
	// printTree_rrt_s(tree_rrt_s);
	// cout << '\n';


	for (int j = 0; j < k_rrt_s; j++)
	{
		// cout << "A random node:" << endl;
		q_rand_rrt_s = generatenodes_rrt_s(numofDOFs);
		// printTreeNodes_rrt_s(q_rand_rrt_s);
		// cout << '\n';

		extend_rrt_s(tree_rrt_s, q_rand_rrt_s, epsilon_rrt_s, numofDOFs, map, x_size, y_size);
	
		// cout << '\n';
		// cout << "j is " << j << endl;
		// cout << '\n';
	}

	cout << "Tree size is: " << tree_rrt_s.size() << endl;
	// printTree_rrt_s(tree_rrt_s);

	
	//return path
	vector<Node_rrt_s*> final_path_rrt_s;

	//printTree(tree);
	for (Node_rrt_s* node : tree_rrt_s)
	{
		if (isGoal_rrt_s(node->angles_rrt_s, armgoal_anglesV_rad))
		{
			goalNode_rrt_s->parent_rrt_s = node;
			break;
		}
	}

	// cout << "goal node parent is: " << goalNode_rrt_s->parent_rrt_s << endl;

	if (goalNode_rrt_s != nullptr)
	{
		while (goalNode_rrt_s != nullptr)
		{
			final_path_rrt_s.push_back(goalNode_rrt_s);
			goalNode_rrt_s = goalNode_rrt_s->parent_rrt_s;
		}
		reverse(final_path_rrt_s.begin(), final_path_rrt_s.end());
	}

	// cout << "Final path is: " << endl;
	// for (const Node_rrt_s* node : final_path_rrt_s) {
    //     std::cout << "Node Angles: ";
    //     for (double angle : node->angles_rrt_s) {
    //         std::cout << angle << " ";
    //     }
    //     std::cout << std::endl;
    // } 
	return final_path_rrt_s;
}

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    double epsilon_rrt_s = 1;
	int k_rrt_s = 20000;
	vector<Node_rrt_s*> path_rrt_s = build_rrt_s(armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, k_rrt_s, epsilon_rrt_s, map, x_size, y_size);

	if (path_rrt_s.empty())
	{
		cout << "No path found" << endl;
		return;
	}

	*planlength = (int)path_rrt_s.size();
	*plan = (double**)malloc(path_rrt_s.size()*sizeof(double*));

	for (int i = 0; i < path_rrt_s.size(); i++)
	{
		(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
		for (int j = 0; j < numofDOFs; j++)
		{
			(*plan)[i][j] = path_rrt_s[i]->angles_rrt_s[j];
		}
	}
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

//G(V,E) ----> V: nodes in the graph, E: implies collision free path between the two configurations - save the node ids

int connections = 5; //for line 6 of the algorithm

struct Node
{
	vector<double> angles;
	int id;
	vector<std::pair<int, double>> distance_neighbor;  //distance to the neighbor node along with its id (to make an edge)
	double cost = std::numeric_limits<double>::infinity();
};

// Print nodes
void printvectornodes(vector<Node>& graph)
{
	for (const auto& node : graph) {
		std::cout << "Node ID: " << node.id << ", Angles: ";
		for (const auto& angle : node.angles) {
			std::cout << angle << " ";
		}
		std::cout << std::endl;
	}
}

void printNode(const Node& node)
{
    // Print the ID of the node
    std::cout << "Node ID: " << node.id << std::endl;

    // Print the angles of the node
    std::cout << "Angles: ";
    for (const auto& angle : node.angles)
    {
        std::cout << angle << " ";
    }
    std::cout << std::endl;
}

//Local Planner - interpolates between two nodes and checks for multiple points whether they are collision free or not
bool connect(vector<Node>&graph, int nodeID, Node& g, int numofDOFs, double *map, int x_size, int y_size)
{		
    //straight interpolation between two nodes checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(graph[nodeID].angles[j] - g.angles[j]))
            distance = fabs(graph[nodeID].angles[j] - g.angles[j]);

			//cout << "connect - dist" << distance << endl;
    }
    int numofsamples = (int)(distance/(PI/20));

    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){

        std::vector<double> sample_config;

        for(j = 0; j < numofDOFs; j++){
            sample_config.push_back(graph[nodeID].angles[j] + ((double)(i)/(numofsamples-1))*(g.angles[j] - graph[nodeID].angles[j]));
        }
        if(!IsValidArmConfiguration(sample_config.data(), numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            //printf("ERROR: Cannot Connect\n");
			return false;
        }
    }
	//cout << "Can connect" << endl;    
	return true;
}

//generate random nodes
Node generatenodes(int numofDOFs)
{
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_real_distribution<double> distribution(0.0, 2*M_PI);

	Node random_node;
	for (int i=0; i<numofDOFs; ++i)
	{
		random_node.angles.push_back(distribution(generator));
	}
	return random_node;
}

//for knn
double euclidean_distance(Node& sample_node, Node& neighbor_node)
{
	//cout << "In euclid" << endl;
	double distance_square = 0.0;
	double distance_euclid = 0.0;

	for (size_t i = 0; i<sample_node.angles.size(); ++i)
	{
		distance_euclid += (sample_node.angles[i] - neighbor_node.angles[i])*(sample_node.angles[i] - neighbor_node.angles[i]);
	}
	distance_square = sqrt(distance_euclid);
	return distance_square;
}

vector<Node> knn(vector<Node>& graph, int nodeID, int k, int numofDOFs)
{
	vector<Node> nearest_neighbors;
	vector<std::pair<int, double>> distances; //to save distances from neighbors to queryNode along with the neighbors nodeID
	int count = 0;

	if (graph.size() == 1)
	{
		return nearest_neighbors;
	}

	//calculate distances from the queryNode to all the other nodes in the graph
	for (size_t i = 0; i<graph.size(); ++i)
	{
		//discard the current node
		if (!(nodeID == graph[i].id))
		{
			double distance = euclidean_distance(graph[nodeID], graph[i]);

			distances.push_back(std::make_pair(graph[i].id, distance));
		}
	}
	//cout << '\n';
	
	if (k >= graph.size()-1)
	{
		//cout << "k is GREATER than graph.size()" << endl;
		for (int j = 0; j<graph.size(); j++)
		{
			if (!(nodeID == graph[j].id))
			{				
				Node k_node;
				k_node.distance_neighbor.push_back(std::make_pair(distances[j].first, distances[j].second));
				k_node.id = distances[j].first;

				for (int a = 0; a<numofDOFs; a++)
				{
					k_node.angles.push_back(graph[distances[j].first].angles[a]);
				}
				nearest_neighbors.push_back(k_node);
			}
		}
		return nearest_neighbors;
	}

	else
	{
		//sort these distances in ascending order
		sort(distances.begin(), distances.end(), [](const auto& lhs, const auto& rhs)
		{
			return lhs.second < rhs.second;
		});

		for (int i = 0; i<k; ++i)
		{
			if (!(nodeID == graph[i].id))
			{				
				Node k_node;
				k_node.distance_neighbor.push_back(std::make_pair(distances[i].first, distances[i].second));
				k_node.id = distances[i].first;

				for (int a = 0; a<numofDOFs; a++)
				{
					k_node.angles.push_back(graph[distances[i].first].angles[a]);
				}
				nearest_neighbors.push_back(k_node);
			}
		}
		//return nearest_neighbors of graph[nodeID].id
		return nearest_neighbors;
	}
}

void build_roadmap(vector<Node> &graph, int num_nodes, int x_size, int y_size, int numofDOFs, double *map, double ***plan)
{
	//initialise a graph which will save the nodes of our configuration space
		
	int n = 0;
	int k = 15;

	while (n < num_nodes)
	{
		Node sample_node = generatenodes(numofDOFs);

		//if sample is valid then add to the graphs as a vertex
		if (IsValidArmConfiguration(sample_node.angles.data(), numofDOFs, map, x_size, y_size))
		{
			graph.push_back(sample_node);

			//assign id to every node you add to the graph as a vertex
			graph[n].id = n;
			int nodeID = n;

			//keep a count of the number of vertices in our graph
			n++;

			//find k nearest neighbors of graph[nodeID-1].id
			vector<Node> nearest_neighbors = knn(graph, nodeID, k, numofDOFs);

			//for each q belonging to the neighborhood of the current node
			for (int i = 0; i<nearest_neighbors.size(); i++)
			{
				Node g = nearest_neighbors[i];

				if (connect(graph, nodeID, g, numofDOFs, map, x_size, y_size) && (graph[g.id].distance_neighbor.size() < connections))
				{
					graph[nodeID].distance_neighbor.push_back(std::make_pair(g.distance_neighbor[0].first, g.distance_neighbor[0].second));
					graph[g.id].distance_neighbor.push_back(std::make_pair(graph[nodeID].distance_neighbor[i].first, graph[nodeID].distance_neighbor[i].second));
				}
			}
		}
	}
}

void query(vector<Node> &graph, double *armstart_anglesV_rad, double *armgoal_anglesV_rad, int numofDOFs, int num_nodes, double *map, int x_size, int y_size)
{
	int k_q = 15;
	//connect q_init and q_goal to the final_graph - consider them as any other random samples

	//add q_init and q_goal to the graph first
	Node q_init;
	Node q_goal;
	for (int i = 0; i < numofDOFs; i++)
	{
		q_init.angles.push_back(armstart_anglesV_rad[i]);
		q_goal.angles.push_back(armgoal_anglesV_rad[i]);
	}
	q_init.id = num_nodes;
	q_goal.id = num_nodes + 1;

	graph.push_back(q_init);
	graph.push_back(q_goal);

	//find nearest neighbors
	vector<Node> q_init_neighbors = knn(graph, (num_nodes), k_q, numofDOFs);

	//for each neighbor nodes
	for (int j = 0; j < q_init_neighbors.size(); j++)
	{
		cout << "first query" << endl;
		Node g_init = q_init_neighbors[j];
		cout << "Neighbors of startNode" << endl;
		printNode(g_init);

		if (connect(graph, q_init.id, q_init_neighbors[j], numofDOFs, map, x_size, y_size) && (graph[g_init.id].distance_neighbor.size() < connections))
		{
			graph[q_init.id].distance_neighbor.push_back(std::make_pair(g_init.distance_neighbor[0].first, g_init.distance_neighbor[0].second));
			graph[g_init.id].distance_neighbor.push_back(std::make_pair(graph[q_init.id].distance_neighbor[j].first, graph[q_init.id].distance_neighbor[j].second));
		}
	}
	
	//find nearest neighbors
	vector<Node> q_goal_neighbors = knn(graph, (num_nodes+1), k_q, numofDOFs);

	//for each neighbor nodes
	for (int l = 0; l < q_goal_neighbors.size(); l++)
	{
		cout << "second query" << endl;
		Node g_goal = q_goal_neighbors[l];

		if (connect(graph, q_goal.id, q_goal_neighbors[l], numofDOFs, map, x_size, y_size) && (graph[g_goal.id].distance_neighbor.size() < connections))
		{
			graph[q_goal.id].distance_neighbor.push_back(std::make_pair(g_goal.distance_neighbor[0].first, g_goal.distance_neighbor[0].second));
			graph[g_goal.id].distance_neighbor.push_back(std::make_pair(graph[q_goal.id].distance_neighbor[l].first, graph[q_goal.id].distance_neighbor[l].second));
		}
	}
}

vector<int> dijkstra(const vector<Node> &graph, int q_init_ID, int q_goal_ID)
{    
    if (q_init_ID < 0 || q_init_ID >= graph.size() || q_goal_ID < 0 || q_goal_ID >= graph.size()) {
        cout << "Invalid start or goal node ID." << endl;
        return {}; // Return an empty path
    }

    std::priority_queue<std::pair<double, int>, vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> OPENLIST;
    vector<double> dist(graph.size(), std::numeric_limits<double>::infinity());
    vector<int> prev(graph.size(), -1); 
    vector<bool> visited(graph.size(), false); // to track visited nodes

    dist[q_init_ID] = 0;
    OPENLIST.push({0, q_init_ID});

    while (!OPENLIST.empty())
    {
        int current = OPENLIST.top().second;
        OPENLIST.pop();

        if (visited[current]) {
            continue; // skip processing for nodes already visited
        }
        visited[current] = true;

        if (current == q_goal_ID)
        {
            break;
        }

        if (current < 0 || current >= graph.size()) {
            cout << "Invalid node ID encountered: " << current << endl;
            continue;
        }

        for (const auto &neighbor : graph[current].distance_neighbor)
        {
            int next = neighbor.first;
            if (next < 0 || next >= graph.size()) {
                cout << "Invalid neighbor ID encountered: " << next << endl;
                continue;
            }
            
            if (visited[next]) {
                continue; // skip processing for neighbors already visited
            }

            double tentative_dist = dist[current] + neighbor.second;
            if (tentative_dist < dist[next])
            {
                dist[next] = tentative_dist;
                prev[next] = current;
                OPENLIST.push({dist[next], next});
            }
        }
    }

    // Clearing the priority queue
    while (!OPENLIST.empty()) {
        OPENLIST.pop();
    }

    vector<int> path;
    for(int at = q_goal_ID; at != -1; at = prev[at])
    {
        if (at < 0 || at >= graph.size()) {
            cout << "Invalid node ID during path reconstruction: " << at << endl;
            return {}; // Return an empty path if an error is encountered
        }
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    if(dist[q_goal_ID] != std::numeric_limits<double>::infinity())
    {
        cout << "Shortest distance to goal: " << dist[q_goal_ID] << endl;
    }
    else
    {
        cout << "No path found to the goal node." << endl;
    }

    return path;
}

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	const int num_nodes = 50000; //number of nodes to put in the roadmap, so generate randomnodes on the go

	vector<Node> graph;
	build_roadmap(graph, num_nodes, x_size, y_size, numofDOFs, map, plan);

	//ADD STARTNODE AND GOALNODE TO THE GRAPH
	query(graph, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, num_nodes, map, x_size, y_size);

	//call dijkstra
	vector<int> shortestPath = dijkstra(graph, num_nodes, num_nodes+1);

	int currentNode = num_nodes+1;  //num_nodes+1 = goalnodeID
	while (currentNode != num_nodes) //num_nodes = startnodeID
	{
		shortestPath.push_back(currentNode);
		currentNode = graph[currentNode].id;
	}
	shortestPath.push_back(num_nodes);
	std::reverse(shortestPath.begin(), shortestPath.end());

	*planlength = static_cast<int>(shortestPath.size());
	*plan = (double**)malloc(*planlength * sizeof(double*));

	for (int i = 0; i < *planlength; ++i)
	{
		(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
		for (int j = 0; j < numofDOFs; ++j)
		{
			(*plan)[i][j] = graph[shortestPath[i]].angles[j];
		}
	}
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */

int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
		auto start = std::chrono::high_resolution_clock::now();
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		std::cout << "Time taken by RRT function: " << duration.count() << " milliseconds" << std::endl;
		double plan_quality = calc_plan_quality(&plan, &planlength, numOfDOFs);
		std::cout << "PLan quality RRT: " << plan_quality << std::endl;
    }
    else if (whichPlanner == RRTCONNECT)
    {
		auto start = std::chrono::high_resolution_clock::now();
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		std::cout << "Time taken by RRT Connect function: " << duration.count() << " milliseconds" << std::endl;
		double plan_quality = calc_plan_quality(&plan, &planlength, numOfDOFs);
		std::cout << "PLan quality RRT: " << plan_quality << std::endl;
    }
    else if (whichPlanner == RRTSTAR)
    {
		auto start = std::chrono::high_resolution_clock::now(); 
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		std::cout << "Time taken by RRT Star function: " << duration.count() << " milliseconds" << std::endl;
		double plan_quality = calc_plan_quality(&plan, &planlength, numOfDOFs);
		std::cout << "PLan quality RRT: " << plan_quality << std::endl;
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
