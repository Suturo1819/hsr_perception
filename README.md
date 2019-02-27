# hsr_perception
This package includes an action server for the hsr perception module.

## Dependencies
The package depends on the follwoing packages
* RoboSherlock [https://github.com/Suturo1819/robosherlock]
* Suturo Perception Msgs [https://github.com/Suturo1819/suturo_msgs/tree/master/suturo_perception_msgs]
* actionlib
* geometry msgs

## Installation
Make sure that you have **RoboSherlock installed**. If that is the case, a simple `catkin build` is enough

## Usage
To start the server, type
`rosrun hsr_perception perception_server`

The server waits now for a client that connects to it and specifies the pipeline that should be executed as a goal. By now, the only two working pipelines are **table** and **shelf**.
As a result, the client gets a **list** with ObjectDetectionData (see `suturo_perception_msgs`).
