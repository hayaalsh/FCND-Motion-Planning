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
The start location on the grid was set to the current local positin using:
```py
grid_start = (int(np.ceil(local_north - north_offset)), int(np.ceil(local_east - east_offset)))
```

### Set grid goal position from geodetic coords ###
Goal position specified by the user through the command line was also converted from global to local frame and was mapped on the grid. 
```py
local_goal_north, local_goal_east, _ = global_to_local(self.global_goal_position, self.global_home)
grid_goal = (int(np.ceil(local_goal_north - north_offset)), int(np.ceil(local_goal_east - east_offset)))
```

To run the code, you need to determin the goal position or else defult values of lat = ?, log = ?, alt = ? will be used.
```
python motion_planning.py --lat_goal <lat> --lon_goal <lon> --alt_goal <alt>
```

### A* search with diagonal motion ###
In `planning_utils.py`, the following was added to `valid_actions()` function after adding new action to `Action()` enum class to include diagonal motions on the grid that have a cost of sqrt(2)
```py
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
```

### Cull waypoints ###
Collinearity test, `prune_path()`, was applied on the optained path to prune it from unnecessary waypoints.

```py
def prune_path(path, epsilon=1e-5):
    """
    Returns prune path.
    """
    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(p1, p2, p3):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])

        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1

    return pruned_path
```


## Execute the flight ##
This is simply a check on whether it all worked. Send the waypoints and the autopilot should fly you from start to goal!

At the moment there is some mismatch between the colliders map and actual buildings in the scene. To ensure success build in a 5+ m safety margin around obstacles. Try some different goal locations. Also try starting from a different point in the city. Your reviewer will also try some random locations so be sure to test your solution! There is no firm constraint or requirement on how accurately you land exactly on the goal location. Just so long as your planner functions as expected.

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].


