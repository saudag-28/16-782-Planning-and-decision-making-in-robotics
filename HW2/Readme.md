# ACADEMIC INTEGRITY
If you are currently enrolled in the Graduate 16-782 Planning and Decision Making in Robotics, or the Undergraduate 16-350 Planning Techniques for Robotics course at Carnegie Mellon University, please refer to CMUs Academic Integrity Policy before referring to any of the contents of this repository.

# NOTE
This planner does not take self-collision into account.

# RRT 
In this planner to store the tree I have made use of the vector container which is of datatype Node_rrt (which is also a pointer to point to the parent of the current node). I am generating random samples using the uniform real distribution function of cpp. I am also using Goal-biasing on the first random variable I generate from the above function. I have set the goal biased value to be 0.5, so everytime the random variable value is less than the goal bias value, the function will return Goal configuration as the random sample for the RRT algorithm to explore. For the extend function, to find the nearest vertex in the tree to q_random, I iterate through the tree and calculate the distance between every vertex in the tree to the q_random node. After this I find a new angle configuration at the epsilon distance (q_new_angles) and interpolate between q_near and q_new_angles and check for collision for each sampled configuration. If the sample is not collision free then I return false (Trapped), else I return true. If it’s true then I add the vertex to the tree. If the newly added vertex is in the Goal region then I exit the loop and return the path, otherwise I continue exploring. For the Goal region function I calculate the euclidean distance between the vertex and the goal configuration and check if the distance is within a threshold value.
## RESULTS
According to the statistics I generated after running the planner with 20 random start and goal configurations, the performance of the RRT planner gives a 100% success rate and it generates less nodes in the tree. The planner has an average planning time of less than 5 seconds.
![rrt9](https://github.com/saudag-28/16-782-Planning-and-decision-making-in-robotics/assets/69856812/63318c84-3e7e-4cdb-bcb8-7f32ad770cd7)


# RRT Connect
The implementation of RRT Connect is similar to the RRT algorithm but it only changes in the CONNECT function where the planner extends towards a node until it hits an obstacle or it reaches the node. Since the planner explores from the start as well as the goal configuration there is no need to do goal-biasing.
## RESULTS
Since the tree explores from both the configurations, the planner is fast and it also generates a solution with less number of nodes. The success rate is low maybe because I am generating fewer nodes in the graph.
![rrt-c9](https://github.com/saudag-28/16-782-Planning-and-decision-making-in-robotics/assets/69856812/156a5799-d427-4c89-9ace-31570eba0676)


# RRT*
This planner is similar to RRT except the re-wiring. According to the algorithm, to explore the neighboring nodes around the newly added vertex (for re-wiring), I have used the radius formula given in the slides with gamma = 2.
## RESULTS
The planner takes a longer time to run because everytime after adding a new vertex to the graph it checks if it can improve the path to go to a node and updates the cost value accordingly. Out of all the algorithms, RRT* takes longer than a second to generate a path but it is still less than 5 seconds.
![rrt-s9](https://github.com/saudag-28/16-782-Planning-and-decision-making-in-robotics/assets/69856812/40f8d308-ffc3-4a7c-a000-3bc29d5ff52f)


# CONCLUSION
![image](https://github.com/saudag-28/16-782-Planning-and-decision-making-in-robotics/assets/69856812/14ce371c-b8bc-4737-8ae0-a1670082a612)
According to the statistics we can conclude that RRT is a better planner than the rest of the planners because of the high success rate. The planners could still be improved if I include the following edge case - if the newly generated node is an invalid configuration, then return the last valid configuration just before hitting an obstacle. This update will ensure that the tree has good enough vertices instead of completely discarding the path to go to an invalid configuration. 
For average path quality I calculate the Euclidean distance between each consecutive pair of waypoints and sum up these waypoints to quantify how much the robot needs to travel (in terms of joint angles) along the path, i.e. it measures the smoothness or directness of the path taken in the joint space. A path with smaller Euclidean differencesbetween consecutive waypoints is generally more desirable. So according to the above statistics, RRTConnect has the best path quality followed by RRT and RRT*.

