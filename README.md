# OpenVINS and VINS Mono AirSim
 
Testing visual-inertial odometry in AirSimNH environment using VINS-Mono and OpenVINS

Everything is ran on ROS melodic (Ubuntu 18.04). AirSimNH is ran on Windows 10.                                                         

This repository only includes the config files for VINS-Mono and OpenVINS. For the entire repository for both and the AirSim ROS package go here:                                                                                                      
VINS Mono: https://github.com/HKUST-Aerial-Robotics/VINS-Mono                                               
OpenVINS: https://github.com/rpng/open_vins                                                                 
AirSim ROS Package: https://github.com/CodexLabsLLC/Colosseum


Tested using 2 different routes:
- Orbit trajectory (orbit.py)
- Square trajectory (along the outer road of the environment) (myorbit.py)


Demo Videos:
- VINS-Mono Orbit Trajectory
  https://youtu.be/VAl_4yFatig
- VINS-Mono Square Trajectory
  https://youtu.be/WTew4qCyfS8
- OpenVINS Orbit Trajectory
  https://youtu.be/DxzV4--dPS0
- OpenVINS Square Trajectory
  https://youtu.be/0_tFjXAO33A

How to Run:                                                                                               
Make sure to catkin build all the repositories first

Terminal 1:                                                                                                                                                                      
cd <path-to-Colosseum_repository>/ros                                                                                                                                         
source devel/setup.bash                                                                                                                                                          
roslaunch airsim_ros_pkgs airsim_node.launch                                
 
Terminal 2:                                                                                                                                                                      
cd <your_catkin_ws>                                                                                                                                                            
source devel_setup.bash                                                                                                                                                          
rosrun my_airsim publisher_node.py
 
Terminal 3:                                                                                                                                                                       
cd <path-to-my_airsim>/scripts/multirotor                                                                                                                                        
python3 subscriber_node.py

Terminal 4:                                                                                                                                                                      
cd <your_catkin_ws>                                                                                                                                                               
source devel_setup.bash                                                                                                                                                          
roslaunch vins_estimator vins_rviz.launch          # For VINS-Mono                                                                                                               
rosrun rviz rviz -d <path-to-open_vins>/ov_msckf/launch/display.rviz          # For OpenVINS                                 
 
Terminal 5:                                                                                                                                                                      
cd <your_catkin_ws>                                                                                                                                                               
source devel/setup.bash                                                                                                                                                           
roslaunch vins_estimator euroc.launch          # For VINS-Mono                                                                                                                   
roslaunch ov_msckf subscribe.launch          # For OpenVINS
  
Terminal 6:                                                                                                                                                                    
cd <path-to-my_airsim>/scripts/multirotor                                                                                                                                       
python3 orbit.py          # For orbit trajectory                                                                                                                                 
python3 myorbit.py          # For square trajectory
 


Conditions:                                                                                                
Orbit:
- 45m altitude
- 150m radius
- 10m/s max speed

Square:
- 40m altitude
- 250m length
- 10m/s max speed


![image](https://github.com/Nicholaskoh1/OpenVINS-and-VINS-Mono-AIrSim/assets/124341280/8c654d64-3bb8-4d87-a2f2-6c9ba8341834)


Comparison:                                                                                                
Under similar configurations (except IMU parameters), both VINS-Mono and OpenVINS were tested in the AirSimNH environment.                                                  

VINS-Mono generates relatively more accurate orbit and straight line trajectories.

VINS-Mono has a pose-graph estimation feature [1], that makes the second orbit more accurate as it corrects drifts over time

VINS-Mono utilizes standard deviation for its IMU parameters, as compared to the actual values for OpenVINS (Both taken from open sources [2])

---------------------------------------------------------------------------------------------------------------
                                                                                                           

For VINS-Mono:                                                                                            
Too much jerk can lead to system failure and rebooting.

Smooth and steady movements are required for successful operation of the drone.

Example:                                                                                                      
Sudden changes from high speed to stopping

VINS-Mono requires a slow start-off speed of around 2m/s.

The starting movement for VINS-Mono is important for trajectory accuracy.

If VINS-Mono travels correctly within the initial time frame, it tends to complete the entire planned path.

---------------------------------------------------------------------------------------------------------------

For OpenVINS:                                                                                              
OpenVINS does not have a rebooting function.

OpenVINS initialization requires a jerk to start off, but it can be switched off in settings [3].

Switching off the initialization jerk is recommended when zero velocity update is enabled for handling stationary cases [4].

OpenVINS trajectory drift is more unpredictable compared to VINS-Mono.

---------------------------------------------------------------------------------------------------------------

Limitations:                                                                                           
There are certain limitations that the AirSimNH environment that may limit the accuracy of the results:
- Lack of features (eg. trees)
 
- Trajectory deviates visibly once it reaches areas with a lack of trees

- Prevents down facing camera from detecting features from slightly higher altitudes

- At more than 45m, drone will undergo intensely jerking and glide down back to 0m


Parameters have to be accurately calculated (camera distortion parameters, camera projection parameters, extrinsic rotation from camera to imu, extrinsic translation from camera to imu, imu parameters). 

Long use of computer will also lead to more and more deviation in results.

---------------------------------------------------------------------------------------------------------------

Test Results: (Yellow is the best run)                                                                                             
![image](https://github.com/Nicholaskoh1/OpenVINS-and-VINS-Mono-AIrSim/assets/124341280/450a3e58-03f1-4f7b-bcd8-35cb74ee2d62)

![image](https://github.com/Nicholaskoh1/OpenVINS-and-VINS-Mono-AIrSim/assets/124341280/8e9878d6-ca19-40e1-8884-e789e6ba5e64)

---------------------------------------------------------------------------------------------------------------

Orbit Euclidean Distance between Estimated Coordinates and Actual Coordinates at each point:          
OpenVINS:                                                                                                  
Run 1: Total Euclidean Distance = 18.70 + 35.44 + 31.88 + 1.73 + 14.87 + 24.41 + 31.78 + 9.22 ≈ 168.03

Run 2: Total Euclidean Distance = 15.07 + 28.77 + 28.78 + 22.38 + 15.74 + 16.49 + 14.87 + 7.62 ≈ 149.72

Run 3: Total Euclidean Distance = 27.39 + 36.11 + 27.90 + 19.13 + 13.35 + 27.78 + 14.73 + 17.61 ≈ 183.00

Run 4: Total Euclidean Distance = 14.42 + 16.73 + 16.00 + 12.57 + 12.21 + 22.87 + 25.83 + 6.32 ≈ 126.95

Run 5: Total Euclidean Distance = 30.95 + 36.92 + 29.97 + 36.67 + 40.68 + 40.09 + 27.73 + 34.17 ≈ 277.18

Run 6: Total Euclidean Distance = 14.04 + 22.78 + 17.65 + 16.92 + 16.79 + 19.43 + 28.49 + 24.34 ≈ 160.44

Run 7: Total Euclidean Distance = 20.92 + 23.87 + 26.93 + 18.14 + 15.62 + 21.50 + 29.59 + 18.75 ≈ 174.32

Run 8: Total Euclidean Distance = 20.92 + 23.87 + 26.93 + 18.14 + 19.00 + 21.50 + 29.59 + 18.75 ≈ 178.70

Run 9: Total Euclidean Distance = 19.52 + 26.57 + 29.16 + 21.26 + 30.69 + 32.89 + 27.83 + 20.24 ≈ 208.16

Run 10: Total Euclidean Distance = 14.97 + 24.84 + 29.02 + 20.81 + 32.71 + 28.55 + 24.30 + 20.15 ≈ 195.35


VINS-Mono:                                                                                                    
Run 1: Total Euclidean Distance = 8.60 + 8.66 + 5.00 + 2.00 + 28.99 + 4.12 + 20.04 + 7.00 ≈ 84.41

Run 2: Total Euclidean Distance = 10.00 + 10.44 + 22.14 + 2.83 + 27.73 + 12.53 + 23.00 + 7.07 ≈ 115.74

Run 3: Total Euclidean Distance = 10.00 + 10.00 + 22.14 + 2.83 + 29.24 + 12.53 + 23.00 + 7.07 ≈ 116.81

Run 4: Total Euclidean Distance = 10.00 + 10.00 + 21.63 + 2.00 + 29.37 + 12.81 + 23.02 + 7.00 ≈ 115.83

Run 5: Total Euclidean Distance = 19.24 + 13.00 + 8.60 + 2.00 + 20.74 + 9.00 + 29.00 + 7.07 ≈ 108.65

Run 6: Total Euclidean Distance = 28.28 + 20.04 + 9.00 + 5.00 + 29.00 + 9.00 + 28.84 + 7.07 ≈ 146.23

Run 7: Total Euclidean Distance = 20.32 + 9.00 + 7.00 + 11.00 + 27.00 + 9.00 + 27.46 + 7.07 ≈ 117.85

Run 8: Total Euclidean Distance = 12.08 + 5.00 + 2.83 + 14.00 + 24.14 + 6.00 + 23.00 + 7.00 ≈ 93.05

Run 9: Total Euclidean Distance = 6.93 + 10.05 + 9.00 + 14.00 + 24.04 + 9.00 + 29.06 + 7.07 ≈ 109.15

Run 10: Total Euclidean Distance = 64.50 + 9.00 + 8.00 + 9.00 + 27.37 + 9.00 + 28.17 + 7.07 ≈ 162.11


References:                                                                                           
[1] https://ieeexplore.ieee.org/document/8421746?arnumber=8421746&source=authoralert                          

[2] https://blog.csdn.net/wxm__/article/details/126510527 

[3] https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf

[4] https://docs.openvins.com/classov__init_1_1StaticInitializer.html 


