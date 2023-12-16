# ACADEMIC INTEGRITY
If you are currently enrolled in the Graduate 16-782 Planning and Decision Making in Robotics, or the Undergraduate 16-350 Planning Techniques for Robotics course at Carnegie Mellon University, please refer to CMUs Academic Integrity Policy before referring to any of the contents of this repository.

# ALGORITHM IMPLEMENTED
For this symbolic planner, to speed up the process of exploring states, I have precomputed all the GroundedActions (along with their preconditions and effects). \
Initially, I generate all the combinations and permutations of the Symbols. I then iterate through each Action and each combination of the Symbols. I then map (container used: unordered_map) every argument of the Action and Condition with the Symbols and then save all the above information in a GroundedAction object. I save each of these objects in a vector (which is a vector of all GroundedActions). \
During A* search, the algorithm will iterate through this vector of GroundedActions and check if it’s a valid action or not.

## HEURISTIC FUNCTION
I have implemented the following heuristic function - \
**(Number of Goal State literals not present in the current state) / (max number of literals added by an action)** \
This heuristic is admissible and it guarantees optimality.

## RESULTS
![image](https://github.com/saudag-28/16-782-Planning-and-decision-making-in-robotics/assets/69856812/fabd8a6f-942d-4de7-b492-a4afb1686dea)
