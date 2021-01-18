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
         roslaunch zmcrobot_ros startUp.launch publish_tf:=true
         # publish_tp:=true do not use ekf; when fase, need startFilter.launch in PC

1. mapping:

        in nvidia:
         roslaunch zmcrobot_ros astra_laser.launch

        in PC:
        cd catkin_ws;
        source ./deve/setup.bash
        roslaunch startFilter.launch   # not neccessory when publish_tf:=true
        roslaunch zmcrobot_ros astra_gmapping.launch
        rviz   #to see the visual
        rqt    #to see visual info ...
        roslaunch zmcrobot_ros startOdomPath.launch  #can show odom path in rviz

    2. navigation
        
        in nvidia:
         roslaunch zmcrobot_ros astra_laser.launch

        in PC:
         cd catkin_ws;
         source ./deve/setup.bash
         roslaunch zmcrobot_ros astra_navigate.launch
         roslaunch startFilter.launch  #not neccessory when publish_tf:=true
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