## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

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

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes `create_grid()`, `valid_actions()`, `a_star()` as utility functions available for implementing the planning problem. 

`motion_planning.py` is the main file for the project and it mainly starts with the function `plan_path()`.

The steps involved into the complete planning are as follows :-

* It reads the starting point location from `colliders.csv` file.
* It defines the grid, altitude and margins from smooth transitions.
* Finally it uses `a_star()` function for planning the optimal path for the problem.



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here I should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. This was accomplished in my code by following way:-

```python
  # : read lat0, lon0 from colliders into floating point values
  lat, lon = self.read_lat_lon('colliders.csv')

  # : set home position to (lon0, lat0, 0)
  self.set_home_position(lon, lat, 0)


```
Here I have created a function `read_lat_lon()` which reads from `colliders.csv` and gives us the `lat`, `lon`.
After above steps it was set using the given function `set_home_position()`.


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

Using code:

`self.local_position`

I have set local_position wherever needed.




#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

```python     

start_pos = (grid_start[0] + int(self.local_position[0]), grid_start[1] + int(self.local_position[1]))

```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

```python
local_goal = global_to_local((lon0, lat0, 0), self.global_home)
    grid_goal = (grid_start[0] + int(local_goal[0]), grid_start[1] + int(local_goal[1]))
    
```    


Here multiple lat0, lon0 is provided based on the simulators current pos.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.


Thee following code is used in my project to prune path and for collinearity check of the points.

```python

def collinearity_check(p1, p2, p3, epsilon=1e-2):
    mat = np.vstack((point(p1), point(p2), point(p3)))
    return np.abs(np.linalg.det(mat)) < epsilon


def prune_path(path):
    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path
    
# and it is used here as follows:-    
waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in prune_path(path)]
```

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


