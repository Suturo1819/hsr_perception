# hsr_perception
This package provides an executable that runs all modules for the perception used during Suturo 2018/19. It includes action servers running specific pipelines in [RoboSherlock](https://github.com/Suturo1819/robosherlock). 

## Dependencies
The package depends on the follwoing packages
* [RoboSherlock] (https://github.com/Suturo1819/robosherlock "robosherlock")
* [The Suturo RoboSherlock Package] (https://github.com/Suturo1819/rs_hsrb_perception "rs_hsrb_perception")
* [Suturo Perception Msgs] (https://github.com/Suturo1819/suturo_msgs/tree/master/suturo_perception_msgs "suturo_perception_msgs")
* actionlib
* geometry msgs

## Installation
Make sure that you have **RoboSherlock installed**. If that is the case, a simple `catkin build` is enough

## Usage
To start the module, type
`rosrun hsr_perception perception_server`

This starts all action servers handling requests for the perception module. As a server is started, an individual [RoboSherlock](https://github.com/Suturo1819/robosherlock) instance is created. The pipeline associated with the instance will be processed as soon as an action client sends a request to the specific server.

### Actions
Currently there are two actions and their respective servers defined:
* `PerceiveTable`
* `PerceiveShelf`
