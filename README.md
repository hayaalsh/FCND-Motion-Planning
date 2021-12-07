# FCND Project 2: Motion Planning #

https://user-images.githubusercontent.com/22026004/145100220-d517aa19-aac8-466c-920c-f0cbb2942a33.mov

This is the readme for the python motion planning project for FCND course offered by Udacity. The file include all the rubric points and how they were addressed and specifically where in the code each step was handled.
More details about the project devoloped by Udacity, contributors, and licensing, can be found [here](../master/README_Udacity.md).

## Starter Code ##

### Explain `motion_planning.py` and `planning_utils.py` ###
`motion_planning.py`, a modified version of `backyard_flyer_solution.py`, includes a basic planning implementation that produces waypoints for the drone to follw. The method that runs the planning module, `plan_path() `, is called right before the drone take off and execute `a_star()` from `planning_utils.py` as one of its step. The detailed psudocode is described below.

1) load map information and obstacles from `colliders.csv` and construct a grid `create_grid()`.
2) Select start location and destination. 
3) Call `a_star()` to find a path from starting point to destination.
4) Produce waypoints from the found path in the previous step and send them to drone using `send_waypoints()`.

---

## Path Planning Algorithm ##
### Global home position ###
After reading the first line of the csv file, `lat0` and `lon0` were extracted as floating point values and were used to set the global home position using `self.set_home_position()`. 

```py
# read lat0, lon0 from colliders into floating point values
header = open('colliders.csv').readline()
s = re.findall(r"[-+]?\d*\.\d+|\d+", header)
lat0 = float(s[1])
lon0 = float(s[3]) # s[0] and s[2] are empty spaces

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
The desired goal can be entered in the deodeti coodrdiantes through the command line as follows, otherther wise the defult values of these paramters will be used.
```
python motion_planning.py --lon -122.39743738 --lat 37.79263316 --alt -5
```    
An allternative is to use function `motion_planning_select()` that I wrote to read user input on the map and choose it as a gole posion.

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
Collinearity test, `prune_path()`, was applied on the optained path to prune it from unnecessary waypoints that lies on the same line

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

---
## Execute the flight ##
To test the code, we tried starting points and destinations. The A* search was able to find a path if exist. Both resulting path ploted on the 2.5D map and the simulator execution for different senarios are shown below. I first used the original function `motion_planning()` and were I only define an endding position.
<p align="center">
<img src="misc/1_Haya.gif" width="700"/>
</p>

The other function is with some modification I made, `motion_planning_select()` were I select the start and end point from the 2.5D map. Note that the drone will have to travel to the start point to execute the path. Therefore, make sure to select a point that is feasiable and close to the starting point. Adittionally, make sure to not select the start and end point on top of each other to avoid error.
<p align="center">
<img src="misc/4_Haya.gif" width="700"/>
</p>

## Video Documentation ##
A short video of the drone using `motion_planning_select()` can be found below.
[![](http://img.youtube.com/vi/0T1lHZbxE9M/0.jpg)](http://www.youtube.com/watch?v=0T1lHZbxE9M "Planning FCND")
