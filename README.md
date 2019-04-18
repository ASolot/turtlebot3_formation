# turtlebot3_formation

How to install

0. Install the required dependency packages: 
        - multirobot_map_merge

TODO: include all the required dependencies


1. Copy the folder to the catkin_ws workspance 
2. Run  pip install -r requirements.txt to install all the required Python packages
3. Run catkin make to rebuild your workspance 
4. Copy the ros\models\unit_cylinder_0 folder to Gazebo's model folder ~/.gazebo/models

To centralise the process of running experiments, we make use of the deploy.sh script. 

    Simulation


    Real-world

    As remote ssh-ing onto robots from a script is not supported in a reliable fashion without the use of third party software, we need to run the configuration for each robot manually. However, we automatically generate the commands that need to be run using the provided script, making it easier to copy-paste in the right terminal.


To change numerical constants used as part of the controllers and detection algorithms, use the config.py file. 