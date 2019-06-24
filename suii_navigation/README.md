# Navigation

Navigation is responsible for everything that has to do with autonomous driving of the robot. The teb_local_planner was used. This planner makes use of several configuration files to operate discussed [here](https://github.com/RoboHubEindhoven/suii/wiki/Navigation-Config). A navigation manager was made to interface between move_base and the rest of the system, discussed [here](https://github.com/RoboHubEindhoven/suii/wiki/Navigation-Manager-Node). A utilitarian node for calculating distance between the robot's position and a goal was made to aid the mission planning software. This node is discussed [here](https://github.com/RoboHubEindhoven/suii/wiki/Navigation-Euclidean-Calculator-Node).

## Important to know

Navigation is now tweaked to the following specs:
The smallest gap between two objects the robot can fit through is 62cm. If the gap is any smaller the robot wil refuse to drive through and replan to find another path if able. This is the minimum possible distance navigation wil reliably fit through without risking collision during normal travel. Also the robot needs 25cm to come to a complete stop after traveling at maximum velocity.

## Getting Started

This is the repository for the Navigation solution for Suii. In the repository you will find motorcalibration software, the navigation manager and the euclidean calculator. 

### Prerequisites

This package requires the following packages to be installed:

* `laser_filters`
* `teb_local_planner`
* `costmap_converter`
* `global_planner`
* `local_planner`
* `amcl`

To install the packages use the following command:

```
sudo apt-get install ros-kinetic-<package_name>
```
Do take care to replace the underscores in the pakcage names with the normal '-' sign.


Before running navigation the make sure the latest version of the suii package is cloned on the robot using the following command:
```
git clone https://github.com/RoboHubEindhoven/suii.git 
```


## Usage

#### Motor calibration
Motor calibration will happen automatically when lowlevel is initiated by pressing the green followed by the blue button. 

#### Running navigation
To run navigation the following command needs to be typed in a terminal:

```
roslaunch suii_bringup navigation.launch map_obstacle:=<map_name> map_amcl:=<map_name>
```

On the place of <map_name> the name of te created map needs to be filled in. The map_obstacles is for navigating the environment. The map_amcl is for localizing the robot. The to variables dont nesseserally need to use the same map.


#### Running the nodes
The command for running the manager node and the euclidean calculator is as follows:

```
rosrun suii_navigation manager.py
```
and
```
rosrun suii_navigation euclidean_calculator.py
```

#### Configuration files
In the Suii_Bringup package there is a map named 'config.' in this file all the configurationfiles are located for navigation.



## Authors

* **Lowie Koolen and Marian Lepadatu** - *in name of RoboHub Eindhoven* - [RoboHub Eindhoven website](https://robohub-eindhoven.nl/)

