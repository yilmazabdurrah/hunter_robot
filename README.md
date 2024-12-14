# hunter_robot

```bash
sudo apt install ros-humble-ackermann-*

ros2 run joint_state_publisher_gui joint_state_publisher_gui 

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ackermann_steering_controller/reference -p stamped:=true

```

  <!-- REF: https://youtu.be/BcjHyhV0kIs?si=dUpg7IF-kHSUgB-w -->
  <!-- Robot website: https://robosavvy.co.uk/agilex-hunter-2b.html -->
  <!-- The values are from: https://github.com/agilexrobotics/ugv_gazebo_sim/blob/master/hunter/hunter2_base/urdf/hunter2_base_gazebo.xacro -->
