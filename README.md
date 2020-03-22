# A* Implementation 

This is the implementation of A* algorithm for rigid robots. This is our implementation of Project 3 as part of the course ENPM661.

## Dependencies
This code was tested with the following dependencies:
- Python 3
- Opencv 3.4.5
- Numpy

## Directories list

Project2
├── astar.py
└── README.md



## Instructions

The directory `Project3` has two files astar.py and README. The file `astar.py` contains the `main()` method.

### Instructions to run the python script
- Open new terminal window in the current directory and run the command
```
python3 astar.py
```  
- Enter the start node, goal node (in format [x,y,theta]), radius, clearance of robot and step_size in the cartesian coordinates.
- If the arguments are passed properly, the robot will start exploration and a file `output.avi` will be saved which shows the representation.
- The shortest path is stored in the `shortest_path` attribute of the Map class. 



## Results

The time taken by point robot for execution from start point `[50,30,60]` to goal point `[150,150,60]` is approximately 14 mins with step size 2. The results are shown in the `output.avi` where the blue dots are explored nodes, green dots are the visited nodes and red points represent the shortest path.


