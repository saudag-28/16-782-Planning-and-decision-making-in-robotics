# ACADEMIC INTEGRITY
If you are currently enrolled in the Graduate 16-782 Planning and Decision Making in Robotics, or the Undergraduate 16-350 Planning Techniques for Robotics course at Carnegie Mellon University, please refer to CMUs Academic Integrity Policy before referring to any of the contents of this repository.

# ALGORITHM IMPLEMENTED
I have implemented an A* search algorithm. The idea behind implementing this algorithm is based on the thought process that this algorithm will give the optimal solution from the robot start position to the goal position provided the heuristics are admissible and hence consistent.
To chase the target, I have defined my desired Goal position as the last coordinate of the target’s trajectory. 
Initially I had implemented the A* algorithm in the planner function which was getting called at every time step. This slowed down the overall process and took a long time to find a solution especially in larger map sizes. 
Since the algorithm has to search for a path only once, I decided to call the A* algorithm (astar function in the code)  at the first timestep, i.e. curr_time = 0. After the algorithm has found its path, I save the path in a global variable and return the robot’s desired trajectory coordinates to the planner caller function. Thus all the computations related to heuristic function and search algorithm are implemented only once, ensuring better efficiency of the program.

## HEURISTIC FUCTION
Since the heuristic distances had to be admissible, I implemented a 2D backward Dijkstra algorithm, with my desired Goal position as the Start node and the robot start position as the Goal position for the dijkstra algorithm. This algorithm would give me g-values from the desired goal node to every node in the graph, and thus these g-values will serve as my heuristic distances when I implement the actual search algorithm. Since Dijkstra gives the most optimal cost between two nodes, these heuristic distances are admissible and consistent.

## DATA STRUCTURES AND MEMORY MANAGEMENT
To store the openlist, I have used priority_queues with vectors. I have defined a datatype named heu_cell_data which stores 3 elements - node’s f-value and it’s x & y coordinates (since we are only looking at the f-values of any node, we don’t have to save other cell parameters in the openlist) and defined a custom priority on the priority queue which will always pop the node with lowest f-value from the openlist vector. The insertion and deletion operation on a priority queue has a time complexity of Olog(n).
For closedlist I have used 2Dvectors with datatype as boolean. Since I am not revisiting the nodes stored in the closedlist, I have initialized a 2D vector of the size of map grid with false values, and as I keep expanding nodes, I set the corresponding node coordinates to true. 
I am using stack data structure to save the Path after performing the backtrack function. Stack container adapter in Cpp follows the Last-In-First-Out principle, therefore it will pop out the nodes from the robot's start pose till the goal pose.
I have used an explicit graph method to save nodes and made use of vector containers to save cell details (which are defined in the structure cell and cell_heu).


![Figure_1](https://github.com/saudag-28/16-782-Planning-and-decision-making-in-robotics/assets/69856812/4540f988-14a4-4146-81ce-282430fd0919)


