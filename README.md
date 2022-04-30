# 2022_Cave

CAVE Group2 GDP 2022.
All the prerequisuite bridge and libraries are in requirements folder.
->Install python3.6-dev 
->Build cv_bridge_ws by specifying the executable as python3 by giving absolute path.

rospkg_name - group2_gdp
Note: source cv_bridge setup.bash file in terminal
Perception:
-> Run perception_main.py for object detection.

Planning:
-> Run planning.py for generating 2d map and generate track.

Controller:
-> Run controller_main.py for pure_pursuit controller.

For the complete solution need to run 3 nodes in parallel.


For testing the controller:
-> Run sub_data.py which will record the gps positions of the vehicle is xls format.
-> Run controller_trial.py.

