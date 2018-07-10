# FCND Project 2: Motion Planning #
This is the readme for the python motion planning project for FCND course offered by Udacity. The file include all the rubric points and how they were addressed and specifically where in the code each step was handled.
More details about the project devoloped by Udacity, contributors, and licensing, can be found [here](../master/README_Udacity.md).

![Quad Image](./misc/enroute.png)

## Starter Code ##

### Explain `motion_planning.py` and `planning_utils.py` ###
`motion_planning.py`, a modified version of `backyard_flyer_solution.py`, includes a basic planning implementation that produces waypoints for the drone to follw. The method that runs the planning module, `plan_path() `, is called right before the drone take off and execute `a_star()` from `planning_utils.py` as one of its step. The detailed psudocode is described below.

1) load map information and obstacles from `colliders.csv` and construct a grid `create_grid()`.
2) Select start location and destination. 
3) Call `a_star()` to find a path from starting point to destination.
4) Produce waypoints from the found path in the previous step and send them to drone using `send_waypoints()`.


## Path Planning Algorithm ##

### Global home position ###
After reading the first line of the csv file, `lat0` and `lon0` were extracted as floating point values and were used to set the global home position using `self.set_home_position()`. 

```py
# read lat0, lon0 from colliders into floating point values
header = open('colliders.csv').readline()
s = re.findall(r"[-+]?\d*\.\d+|\d+", header)
lat0 = float(s[1])
lon0 = float(s[2])

# set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)

# retrieve current global position
print('current global home: x = %.2f, y = %0.2f, z = %0.2f' % (self.global_position[0],self.global_position[1],self.global_position[2]))

```

### Set current local position ###
The local position was determined relative to global home using the `global_to_local()` function.

```py
local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)
```

### Set grid start position from local position ###
This is another step in adding flexibility to the start location. As long as it works you're good to go!

### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


## Project: 3D Motion Planning


---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!


