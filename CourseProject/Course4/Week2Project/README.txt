# My solution - RRT path planning, writing on python3

The algorithm of RRT is actually quite straight forward. 
Points are randomly generated and connected to the closest available node. 
Each time a vertex is created, a check must be made that the vertex lies outside of an obstacle. 
Furthermore, chaining the vertex to its closest neighbor must also avoid obstacles. 
The algorithm ends when a node is generated within the goal region, or a limit is hit.


## implementation RRT algorithm takes only obstacles position and build grid with graph and find minimal path for a some node.

## Useful links:
https://medium.com/@theclassytim/robotic-path-planning-rrt-and-rrt-212319121378
https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf