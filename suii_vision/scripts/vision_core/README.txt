instructions to run vision_core
All steps need to be execuded in different suii terminals


#step 1
cd 
source envpy3
cd /home/suii/catkin_ws/src/suii/suii_vision/scripts/vision_core
python3 vision_manager.py

#step 2
rosrun suii_vision vision_wrapper.py

#step 3
rosservice call /get_scan_all "{}"


