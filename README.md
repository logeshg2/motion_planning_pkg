## Motion Planning Pkg

Trying to implement motion planning algorithms like RRT (and its variants) for robotic arms.


### Installation:

1. Git clone repo:
```bash
git clone https://github.com/logeshg2/motion_planning_pkg.git
```

2. Install dependencies:
```bash
pip3 install -r requirements.txt
```

3. Install **mpkg**:
```bash
pip3 install -e .
```


### Example Usage:

1. Differential Inverse Kinematics:
```bash
python3 examples/diff_ik_eg.py
```

2. RRT Star Planner (with and without collision objects):
```bash
# without collision objects
python3 examples/rrt_star_planner_eg.py

# with collision objects
python3 examples/rrt_star_planner_collision_eg.py
```

![rrt_star_fanuc_gif](./images/fanuc_rrt_star.gif)