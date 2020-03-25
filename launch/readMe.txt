set up:
1. in  nvidia: 
        export ROS_MASTER_URI=http://IP_OF_NVIDIA:11311
        export ROS_HOSTNAME=IP_OF_NVIDIA

    in PC:
        export ROS_MASTER_URI=http://IP_OF_NVIDIA:11311
        export ROS_HOSTNAME=IP_OF_PC
    ( add to ~/.bashrc )

Startup:
    in nvidia:
         cd catkin_ws;
         source ./deve/setup.bash
         roslaunch zmcrobot_ros startUp.launch

1. mapping:

        in nvidia:
         roslaunch zmcrobot_ros astra_laser.launch
         roslaunch zmcrobot_ros astra_gmapping.launch

        in PC:
         cd catkin_ws;
         source ./deve/setup.bash
         roslaunch startFilter.launch  #not neccessory
        rviz   #to see the visual
        rqt    #to see visual info ...
    
    2. navigation
        
        in nvidia:
         roslaunch zmcrobot_ros astra_laser.launch
         roslaunch zmcrobot_ros astra_navigate.launch

        in PC:
         cd catkin_ws;
         source ./deve/setup.bash
         roslaunch startFilter.launch  #not neccessory
        rviz   #to see the visual
        rqt    #to see visual info ...

3. view astra video
      in nvidia
        roslaunch astra_launch astra.launch
        roslaunch zmcrobot_ros camera.launch


//run cmds

     in PC:

         rosrun zmcrobot_ros exec_cmd

         (cmds:  cm; calibrate imu; sc; save the calibrate;  mf0/1/2, alpha; choise filter type; im0/1,0-1; use the imu for yaw )