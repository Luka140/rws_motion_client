# RWS Motion Client
This package contains a node which interacts with the RWS interface of the [ABB ROS2 driver](https://github.com/PickNikRobotics/abb_ros2/tree/humble) to perform a simple grinding motion.
Note that for this to work correctly, the robot should be in automatic mode, and **single-cycle** mode **not continuous**.

## Installation and dependencies
In addition to the ABB ROS2 driver, there are some other dependencies for the other hardware components. Note that there may be more dependencies from these packages themselves. 
- [ABB ROS2 driver](https://github.com/PickNikRobotics/abb_ros2/tree/humble) (`humble` branch)
- [`data_gathering`](https://github.com/Luka140/data_gathering/tree/moving_grinder) (`moving_grinder` branch)
- [`ferrobotics_acf`](https://github.com/Luka140/ferrobotics_acf/tree/humble) (`humble` branch)
- [`data_gathering_msgs`](https://github.com/Luka140/data_gathering_msgs/tree/moving_grinder) (`moving_grinder` branch)
- [`stamped_std_msgs`](https://github.com/Luka140/stamped_std_msgs)
```
git clone git@github.com:PickNikRobotics/abb_ros2.git -b humble
git clone git@github.com:Luka140/data_gathering.git -b moving_grinder
git clone git@github.com:Luka140/ferrobotics_acf.git -b humble
git clone git@github.com:Luka140/data_gathering_msgs.git -b moving_grinder
git clone git@github.com:Luka140/stamped_std_msgs.git
```

## Launch
A launch file is included to launch the RWS interface. 
```
ros2 launch rws_motion_client rws.launch.py
```
This can be used to quickly test whether the RWS ROS services are working correctly. 

To launch the `rws_motion_client` use
```
ros2 launch rws_motion_client motion_client.launch.py
```
This launches the `rws_motion_client`, `rws_client` for the RWS services, the [`grinder_node`](https://github.com/Luka140/data_gathering/blob/moving_grinder/data_gathering/grinder_node.py) which acts as a PLC interface to control the grinder, and the Ferrobotics [`acf` node](https://github.com/Luka140/ferrobotics_acf/blob/master/src/acf.py).

Note that for the full intended use of this package, it is used with the `data_gathering` [`test_coordinator`](https://github.com/Luka140/data_gathering/blob/moving_grinder/data_gathering/test_coordinator.py). This coordinates multiple tests, rosbag recordings, and scans. All components are then launched from the [`data_gathering` launch file](https://github.com/Luka140/data_gathering/blob/moving_grinder/data_gathering.launch.py):
```
ros2 launch data_gathering data_gathering.launch.py
```

## Nodes 

### rws_client
This is a node from the [ABB ROS2 driver](https://github.com/PickNikRobotics/abb_ros2/tree/humble) that provides various services to interact with a RAPID script. In addition to this, it publishes the robot state.
For an overview, see the [docs](https://github.com/PickNikRobotics/abb_ros2/blob/humble/docs/RWSQuickStart.md).

### rws_motion_client
This node interacts with the `rws_client` and other nodes to perform a simple grind. An overview of the logic followed by the node is included further [down below](#execution-flow) and in a [code comment](https://github.com/Luka140/rws_motion_client/blob/main/src/rws_motion_client.cpp).

#### Topics
Published Topics

- `/acf/force` (`stamped_std_msgs::msg::Float32Stamped`)
        Publishes ACF (Active Contact Flange) force setpoint.
- `~/grind_settings` (`data_gathering_msgs::msg::GrindSettings`)
        Publishes grinding operation settings to enter them into the rosbag recording.

Subscribed Topics

- `/rws_client/system_states` (`abb_robot_msgs::msg::SystemState`)
        Monitors system states of the ABB robot controller. The 'motor on' state is continuously checked during tests. If the motor state switches off, it indicates that the E-stop was pressed or something else went wrong. In this case, the node will attempt to retract the ACF, and turn off the grinder.
  this does mean that this **E-stop link depends on the polling rate of the `rws_client` node**. For a more robust approach, this could be rewritten such that the E-stop IO state of the ABB is checked on a timer using the `rws_client` `get_io_signal` request. The motor-on check was implemented to prevent additional actions for the RWS interface, as this information will be published anyway.
- `/acf/telem` (`ferrobotics_acf::msg::ACFTelemStamped`)
  Receives telemetry data from the ACF sensor. This is used to check whether the ACF is approaching its maximum extension. If this occurs the test is aborted, as reaching maximum ACF extension means that the force on the test object is unknown since part or all of the force could be pushing against the ACF's own endstop.
  
#### Services

Provided Services

  - `~/start_grind_move` (`data_gathering_msgs::srv::StartGrindTest`)
        The main service interface to start a grinding motion sequence according to the [execution flow](#execution-flow) below. 

Called Services

- `/rws_client/set_rapid_bool` (`abb_robot_msgs::srv::SetRAPIDBool`)
        Sets boolean flags in the RAPID code.
- `/rws_client/get_rapid_bool` (`abb_robot_msgs::srv::GetRAPIDBool`)
        Retrieves boolean flags from the RAPID code.
- `/rws_client/pp_to_main` (`abb_robot_msgs::srv::TriggerWithResultCode`)
        Moves the ABB controller’s program pointer to the main routine.
- `/rws_client/start_rapid` (`abb_robot_msgs::srv::TriggerWithResultCode`)
        Starts the RAPID execution.
- `/rws_client/set_rapid_num` (`abb_robot_msgs::srv::SetRAPIDNum`)
        Sets numeric variables in the RAPID code for the TCP speed and number of grinding passes.
- `/grinder_node/enable_grinder` (`data_gathering_msgs::srv::StartGrinder`)
        Enables the grinder with a certain RPM.
- `/grinder_node/disable_grinder` (`data_gathering_msgs::srv::StopGrinder`)
        Disables the grinder.

#### Parameters
Parameters representing the names of RAPID variables

- `symbol_home_flag` (default: `"waiting_at_home"`)
        RAPID flag indicating the robot is waiting at the home position.
- `symbol_grind0_flag` (default: `"waiting_at_grind0"`)
        RAPID flag indicating the robot is waiting at the initial grind contact position.
- `symbol_grind_done_flag` (default: `"finished_grind_pass"`)
        RAPID flag indicating that all grind passes are finished.
- `symbol_run_status_flag` (default: `"run_status"`)
        RAPID flag indicating whether the RAPID script is running.
- `symbol_nr_passes` (default: `"num_pass"`)
        Name of the variable for setting the number of grinder passes.
- `symbol_tcp_speed` (default: `"tcp_feedrate"`)
        Name of the variable for setting the TCP feed rate.

Other parameters
- `bool_timer_period` (default: `100`)
  Timer interval for checking boolean RAPID flags (milliseconds).
- `grinder_spinup_duration` (default: `4`)
        Time (seconds) for the grinder to spin up and stabilize before starting motion.
- `max_tcp_speed` (default: `35.0`)
  Maximum allowable TCP speed (mm/s). In place to prevent executing if a high feed rate is set (by accident). 
- `max_acf_extension` (default: `34.5`)
        Maximum allowable ACF extension (mm). When this is exceeded the test will be aborted. 


#### Execution flow
The `rws_motion_client` follows the steps below. The 'symbol_x' refer to the names of the variables as set in the RAPID code. They can be set in ROS using the corresponding parameter. 

1.  Wait for a test service call.
	This includes: force, RPM, TCP speed, passes

2. Check if currently running. 
    This is first done with a boolean flag in this node (service available).
    If this is true, the `'symbol_run_status_flag'` bool in RAPID is checked. If this is false proceed, otherwise reject the request.

3. use '/rws_client/pp_to_main' to set the program pointer to the start of main

4. Set the params for the number of passes (`'symbol_nr_passes'`) and the TCP speed (`'symbol_tcp_speed'`)  based on the values in the request.

5. Start the RAPID program with `'/rws_client/start_rapid'`

6. Wait until the home position is reached.
    This is done by checking the `'symbol_home_flag'` bool on a timer. When the bool turns true, the robot has arrived at home and will wait there until it is set to false again.

7. Start the grinder with the desired rpm (`'/grinder_node/enable_grinder'`) while still at the home position. 
    Then wait for `'grinder_spinup_duration'` number of seconds for the grinder to spin up.

8. Reset the `'symbol_home_flag'` (from step 6.) to false again. 
    The robot will now proceed to move to the first grind position. 

9. Wait until the first grinding position is reached 
    This is done by checking the `'symbol_grind0_flag'` bool on a timer. When the bool turns true, the robot has arrived at the first grinding position and will wait there until it is set to false again.

10. Extend the ACF by publishing the requested grinding force on `'/acf/force'`.

11. Reset the `'symbol_grind0_flag'` (from step 8.) to false again. 
    The robot will now execute the grinding pass for the requested number of times. 

12. Wait until the robot is done grinding. 
    This is done by checking the `'symbol_grind_done_flag'` bool on a timer. When the bool turns true, the robot has finished grinding. It will NOT wait in this position but immediately move to the home position.

13. Retract the grinder (`'/grinder_node/disable_grinder'`) and turn off the ACF (`'/acf/force'`). Then reset the `'symbol_grind_done_flag'` boolean to false.

14. Wait until the robot has reached the home position again. 
    This is done by checking the `'symbol_run_status'` bool on a timer. When this bool is false, the entire RAPID script has been executed. When this occurs, send back the service response. 
