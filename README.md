## Disciplina de Tópicos em Sistemas Robóticos
### Autor: Kauã Dalla Riva Cucco Barbosa

## Dependencies:

- Ubuntu 22.04 (Jammy Jellyfish) – 64 bits.
- ROS 2 Humble: [to install humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- Follow [step by step](https://github.com/viniciuslg91/stage/blob/main/README.md).
- This repository must be located in the `/src` folder of your ROS 2 Humble workspace. The other two Stage packages (from the previous step) must also be in the same `src` directory.

## To run the launch:
#### Terminal 1:
```
cd <your_workspace>
colcon build
source install/setup.bash
ros2 launch stage_ros2 stage.launch.py world:=new_cave enforce_prefixes:=false one_tf_tree:=true
```

## To run the controller:
#### Terminal 2:
```
cd <your_workspace>
colcon build
source install/setup.bash
ros2 run stage_navigation robot_navigation_node
```

### Changing the Waypoints:
To modify the waypoints, follow these steps:
1. **Enable or disable odometry:**
   Edit line 37 of the file:
   `stage_navigation/stage_navigation/robot_navigation_node.py`
   
   Choose whether the simulator will use odometry.

3. **If odometry is enabled:**

   * Open `state_estimator.py` and update the target points.
   * Replace the corresponding values in lines 39–42 of `robot_navigation_node.py` with the updated estimated error values of the target points.

4. **Update the final target points:**
   Modify lines 48–51 in `robot_navigation_node.py` to reflect the new waypoints (target that the robot will actually reach).
