Migrated official ros1 gopigo3 simulation pkg to ros2 humble with ignition gazebo (Gazebo Sim, version 6.17.0).

Project Status: In Progress

Migration Checklist:
- updated urdf files
- updated sdf files, added turtlebot3_world
- updated launch files
- added gz_bridge.yaml
- updated dependecies in package.xml 
- updated setup.py

working features:
- publishes joint states, tf, odom, robot_description, clock.
- instead of /cmd_vel the ros2 topic is called "diff_cont/cmd_vel_unstamped". works.

not working features:
- Distance sensor without GPU. (renders but does not update, that's why no collision is detected, therefore /scan topic will show .inf readings)

missing features:
- camera, imu

<img width="1381" height="883" alt="gopgio3_gazebo_sim_ros2humble" src="https://github.com/user-attachments/assets/0b890dd4-a700-43f8-9fa9-0fd842ef5105" />

helpfull links:
https://gazebosim.org/docs/fortress/migrating_gazebo_classic_ros2_packages/
