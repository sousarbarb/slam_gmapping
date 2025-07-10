# [slam_gmapping](https://github.com/ros-perception/slam_gmapping/)

**Modified by Ricaardo B. Sousa**

This fork only adds an offline node to process multiple ROS bag files, generate
the 2D occupancy grid map through ROS API, and save the robot pose data into TUM
format (need to check if the pose data is indeed laser pose, or for the base).

## Usage

```sh
$ rosrun gmapping slam_gmapping_offline --help
Usage: rosrun gmapping slam_gmapping_offline -b BAGFILE1 [BAGFILE2 BAGFILE3 ...]

Options:
  -h [ --help ]              Display this information.
  -b [ --bags ] BAGFILE      ROS bag files to process
  --scantopic TOPIC (=/scan) Topic name for the 2D laser scanner data
  -s [ --start ] SEC (=0)    start SEC seconds into the bag files
  -d [ --duration ] SEC      play only SEC seconds from the bag files
  --log FILENAME             log the robot estimated data (laser pose) into TUM
                             files
```

See the ROS launch file
[slam_gmapping_offline.launch](/gmapping/launch/slam_gmapping_offline.launch)
for an example on how to launch the node similar to the online version
(including parametrizing the GMapping system through ROS parameters).
