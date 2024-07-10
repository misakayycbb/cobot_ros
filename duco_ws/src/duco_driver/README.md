# DUCO Robot Driver

* A  driver for the `duco_gcr5/duco_gcr10/duco_gcr14/duco_gcr20` robot arms from SIASUN robots. <br>
  
### Features
___

* You can control the robot by using both teach pendant(through *`Duco API`*) and `MoveIt`.<br>
  When lauching the driver node, the default controller is `ros-controller`, when you want to switch to the `robot-controller`, you should publish a message, like:<br> 
  ```
  rostopic pub -1 duco_driver/controller_switch std_msgs/Int32 -- 0
  ```
  The last argument mean the robot is controlled by robot-controller, when you want to go back to ros-controller, just change the value **"0"** to **"1"**.

	* Action interface on `/follow_joint_trajectory` for seamless integration with MoveIt.

	* Publishes robot joint state on `/joint_states`

	* Publishes IO state on `/duco_driver/io_states`

	* Service call to set outputs.

* Added support for `duco_control`.

	* As ros_control wants to have control over the robot at all times, ros_control compatibility is set via a parameter at launch-time.
	* With ros_control active, the driver doesn't open the action_lib interface. This is handled by ros_control instead.
	* Currently one controller is available, controlling the joint position of the robot, useable for trajectroy execution.

### Installation
---
* As the driver communicates with the robot via ethernet and depends on reliable continous communication, it is not possible to reliably control a SIASUN Robot from a virtual machine.

* Just clone the repository into your catkin working directory and make it with catkin_make.

* Note that this package depends on duco_msgs, hardware_interface, and controller_manager so it cannot directly be used with ROS versions for indigo and kinetic.

### Usage

```
roslaunch robot_name_moveit_config robot_name_moveit_planning_executing.launch
roslaunch robot_name_moveit_config moveit_rviz.launch config:=true
```
There is a parameter named `"/duco_controller/velocity_scale_factor"`, you can use it to sacle the velocity of the trajectory. The bigger the parameter is, faster the robot will move.
### Test
---
* `tsetIO`: The message of topic `/duco_driver/io_states` is published at a rate of 100Hz. To set IO states, you need to establish a ServiceClient to call servive on `/duco_driver/set_io`
* `testSiasunAPI`: There are plenty of functionality you can use to control the robot, see this [declaration](RobotServiceInterface.h). If you want to control the robot more efficiency, it is recommanded to use the Siasun API to realize the functions.

