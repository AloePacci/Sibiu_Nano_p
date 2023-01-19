# Sibiu Nano +
Trying to make a ROV an AUV

1. Install ROS https://docs.ros.org/en/foxy/Installation.html   (recomended Ubuntu 20.4 [$] binaries, but also works on windows[%]) and configure it in bash

2. install colcon https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html

2. clone this git in any folder

3. $cd ./ASV_Loyola_US

4. resolve package dependencies $rosdep install -i --from-path src --rosdistro foxy -y

5. $colcon build



- ![#1589F0](https://via.placeholder.com/15/1589F0/000000?text=+) `Needs`
$sudo apt-get install python3-rosdep2 ~nros-foxy-rqt*

- ![#1589F0](https://via.placeholder.com/15/1589F0/000000?text=+) `Packages used`


- ![#1589F0](https://via.placeholder.com/15/1589F0/000000?text=+) `Extra`
Cuando se trabaje con drones estudiar el uso de distintas ROS_DOMAIN_ID 
(C++ Only) En windows necesitamos control de visibilidad https://docs.microsoft.com/en-us/cpp/cpp/dllexport-dllimport

Diferencias ROS-ROS2 https://design.ros2.org/articles/changes.html
colcon build --symlink-install for developers
