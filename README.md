# LUUAVproject

The project files for the MEng project at Lancaster University
Follow the install instructions in the report appendix for setup guide.

Hector Quadrotor 

The following commands install all of the relevant Hector files for testing, ensure the terminal has loaded the correct filespace, the src folder of the catkin workspace. Installed in this section include the quadrotor urdf models for use in Gazebo and RVIZ, the localization and SLAM algorithms. Additionally, dependencies are installed to prevent catkin_make errors and some recurring but non-dangerous warning messages are disabled for ease of reading the launch status in terminal. 

The following install commands are to execute when in catkin_ws/src folder. 

sudo apt-get install ros-kinetic-ros-control 

sudo apt-get install ros-kinetic-gazebo-ros-control 

sudo apt-get install ros-kinetic-unique-identifier 

sudo apt-get install ros-kinetic-geographic-info 

sudo apt-get install ros-kinetic-laser-geometry 

sudo apt-get install ros-kinetic-tf-conversions 

sudo apt-get install ros-kinetic-tf2-geometry-msgs 

sudo apt-get install ros-kinetic-joy 

git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor 

git clone -b catkin https://github.com/tu-darmstadt-ros-pkg/hector_localization 

git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_gazebo 

git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_models 

git clone -b catkin https://github.com/tu-darmstadt-ros-pkg/hector_slam 

sed -i -e 's/option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" ON)/option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" OFF)/g' hector_quadrotor/hector_quadrotor/hector_quadrotor_gazebo/urdf/CMakeLists.txt 

sed -i -e 's/option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" ON)/option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" OFF)/g' hector_quadrotor/hector_quadrotor/hector_quadrotor_gazebo/urdf/CMakeLists.txt 

# this is to deactivate warnings 
sed -i -e 's/add_dependencies(pose_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt 

sed -i -e 's/add_dependencies(takeoff_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt 

sed -i -e 's/add_dependencies(hector_quadrotor_controllers hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor/hector_quadrotor_controllers/CMakeLists.txt 

 

To prevent catkin_make errors, delete the hector_gazebo_thermal_camera file, as this folder requires unnecessary packages, the thermal camera is not used in this project. 

cd .. 

catkin_make 

source devel/setup.bash 

OctoMap 

Installing the OctoMap SLAM server, algorithm and RVIZ plugins is achieved by the commands: 

sudo apt-get install ros-kinetic-octomap 

sudo apt-get install ros-kinetic-octomap-rviz-plugins 

 

Additionally, Python scripts are fully enabled by entering:  

pip install pynput 

sudo apt get install ros-kinetic-ros-numpy 

 

Navigation 

The trajectory planning and optimization algorithm, teb_local_planner was installed via apt. 

sudo apt get install teb_local_planner 

For the OctoMap algorithm, it is also necessary to install the pointcloud to laser scan package, as this converts the 3D occupancy map to a 2D occupancy map for the navigation. 

sudo apt get install  ros-kinect-pointcloud-to-laserscan 
