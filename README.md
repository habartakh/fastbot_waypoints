# Test a ROS2 Action Server using GTest

The goal of this package is to write test cases for an action server in ROS2.

The `/fastbot_as` action server provides a custom message in order to move the robot to a certain position using the coordinates [X,Y,Z].

The test directory contains the necessary files to check if:

The end position [X, Y] of the robot is the expected one based on the goal sent.
The end rotation [Yaw] of the robot is the expected one based on the goal sent.

The `fastbot_action_server_test.cpp` contains a test suite that checks if the final position and orientation of the robot correspond to the goal sent to the Action server.


## Test different cases

In order to run tests, open the file `fastbot_action_server_test.cpp`, and inside the Test suite `TEST_F(FastbotActionServerTest, RobotReachedGoal)`, modify the values of the following variables: 
- `goal.position.x` 
- `goal.position.y`
-  `goal_yaw` 

Here are some examples that pass the tests: 
- X = 0.5 / Y = 0.7 / Yaw = 1.57
- X = 0.9 / Y = 0.0 / Yaw = 0.0
- X = 1.0 / Y = 1.0 / Yaw = 0.8
- X = 0.4 / Y = 0.0 / Yaw = 3.1

After the modifications, build and source the workspace (`ros2_ws` in our case): 
```
cd ~/ros2_ws && colcon build && source install/setup.bash
```

Run the action server first: 
```
ros2 run fastbot_waypoints fastbot_action_server
```
Then run the tests: 
```
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```


### Test failing conditions

#### Case 1: X = -0.2 & Y = 0.0 & Yaw = 3.14
The robot gets out of the scope of the environment falls. The wheels keep moving endlessly until the GTest times out.  

#### Case 2: X = 0.9 & Y = -0.3 & Yaw = 3.14
The robot crashes against the couch. The wheels get stuck but the action server keeps trying to correct the position until the GTest times out. 

#### Case 3: X = none OR Y = none OR Yaw = none
If any of the goal variables does not correspond to a `double`, the tests end up failing since they expect numerical values.

#### Case 4: X = 100 OR Y = 100
If the goal values are too big or out of the scope of the environment, the robot crashes into the obstacles or the wall while trying to go towards the goal.
