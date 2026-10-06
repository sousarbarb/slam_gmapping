# slam_gmapping (fork): tf2-based online and offline GMapping nodes

Fork of [ros-perception/slam_gmapping](https://github.com/ros-perception/slam_gmapping)
(version 1.4.2, ROS 1 Noetic), modified by Ricardo B. Sousa.

The fork adds two nodes built on a common, tf2-based wrapper class
(`SLAMGMappingROS1API`):

- **`slam_gmapping_online`**: live SLAM, same role as the upstream
  `slam_gmapping` node;
- **`slam_gmapping_offline`**: reads one or more bag files directly (no
  `rosbag play`, no `/clock`), processes them as fast as possible, and
  optionally logs the estimated `base_frame` trajectory in TUM format.

The upstream nodes (`slam_gmapping`, `slam_gmapping_nodelet`,
`slam_gmapping_replay`) are still built and installed; their source files are
unchanged.

Main additions with respect to upstream:

- offline processing of multiple bags, with `/tf_static` preloaded, start
  offset and duration;
- TUM logs of the `base_frame` pose (three variants, see
  [Output files](#output-files-tum-logs));
- fixed random seed for repeatable offline runs (`--seed`);
- `~particlecloud` (`geometry_msgs/PoseArray`) with the pose of every particle;
- `map_update_interval <= 0` disables map updates while scans are processed
  (offline: the map is built once, at the end, e.g. for timing evaluation).

The full list of changes is in [MODIFICATIONS.md](MODIFICATIONS.md).

## Contents

- [Repository layout](#repository-layout)
- [Nodes](#nodes)
- [Build](#build)
- [Usage](#usage)
  - [slam_gmapping_online](#slam_gmapping_online)
  - [slam_gmapping_offline](#slam_gmapping_offline)
  - [Original nodes](#original-nodes)
- [Launch files](#launch-files)
- [Parameters](#parameters)
- [ROS API](#ros-api)
- [Output files (TUM logs)](#output-files-tum-logs)
- [Debugging](#debugging)
- [TODO](#todo)
- [License and credits](#license-and-credits)

## Repository layout

```
slam_gmapping/                        metapackage (unchanged)
gmapping/
  config/
    slam_gmapping.yaml                parameter file (example / defaults)
    rosconsole.config                 rosconsole DEBUG configuration
  launch/
    slam_gmapping_online.launch       online node
    slam_gmapping_offline.launch      offline node
    slam_gmapping_pr2.launch          upstream example (original node)
  rviz/rviz.rviz                      rviz configuration (map, scan, TF)
  src/
    slam_gmapping_ros1_api.{h,cpp}    common wrapper (tf2)               [new]
    slam_gmapping_ros1_online*.{h,cpp}   online node                     [new]
    slam_gmapping_ros1_offline*.{h,cpp}  offline node                    [new]
    slam_gmapping.{h,cpp}, main.cpp,
    nodelet.cpp, replay.cpp           original nodes (tf1)          [upstream]
  test/                               upstream rostests (original nodes only)
  nodelet_plugins.xml                 nodelet plugin of the original node
MODIFICATIONS.md                      modifications log of this fork
```

## Nodes

| Executable | Input | TF library | Origin |
| --- | --- | --- | --- |
| `slam_gmapping_online` | live `scan` topic + TF | tf2 | this fork |
| `slam_gmapping_offline` | bag file(s), read directly | tf2 | this fork |
| `slam_gmapping` | live `scan` topic + TF | tf | upstream |
| `slam_gmapping_nodelet` (library, nodelet `SlamGMappingNodelet`) | live `scan` topic + TF | tf | upstream |
| `slam_gmapping_replay` | one bag file, `/tf` only | tf | upstream |

`slam_gmapping_online` and `slam_gmapping` use the same topic and service
names; do not run both at the same time.

## Build

Requirements: ROS 1 Noetic (Ubuntu 20.04), a C++17 compiler (GCC 9 is fine)
and `openslam_gmapping` (`ros-noetic-openslam-gmapping` from apt, or
[from source](https://github.com/ros-perception/openslam_gmapping) in the same
workspace).

```sh
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone git@github.com:sousarbarb/slam_gmapping.git
# optional, only if openslam_gmapping is not installed from apt:
# git clone https://github.com/ros-perception/openslam_gmapping.git

cd ~/ros_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y

catkin_make --source src --build build_release --force-cmake  \
  -DCATKIN_DEVEL_PREFIX=devel_release                         \
  -DCMAKE_BUILD_TYPE=Release                                  \
  -DCMAKE_INSTALL_PREFIX=install_release

source devel_release/setup.bash
```

Notes:

- Run `catkin_make` from the workspace root: the relative paths given to
  `--source`, `--build`, `-DCATKIN_DEVEL_PREFIX` and `-DCMAKE_INSTALL_PREFIX`
  are resolved against the current directory.
- `--force-cmake` re-runs CMake on every build (slower, but always picks up
  changed CMake arguments).
- For debugging, use a separate set of spaces (e.g. `build_debug`,
  `devel_debug`, `install_debug`) with `-DCMAKE_BUILD_TYPE=Debug` or
  `RelWithDebInfo`, so the release build is not overwritten.
- The command above only builds into `devel_release`. Appending `install`
  populates `install_release`, but `launch/`, `config/` and `rviz/` are not
  installed yet (see [TODO](#todo)), so source `devel_release` to use the
  launch files.

## Usage

### slam_gmapping_online

Live SLAM from a `sensor_msgs/LaserScan` topic and odometry provided through TF.
Requirements:

- `odom_frame -> base_frame` in TF (odometry);
- `base_frame -> <laser frame>` in TF (usually static), where the laser frame is
  the `header.frame_id` of the scans;
- the laser mounted planar: its z axis parallel to the z axis of `base_frame`,
  pointing up or down (upside-down lasers are supported). Otherwise the node
  warns `Laser has to be mounted planar!` and keeps waiting.

```sh
# launch file (loads gmapping/config/slam_gmapping.yaml)
roslaunch gmapping slam_gmapping_online.launch scan_topic:=/scan

# or directly, with private parameters on the command line
rosrun gmapping slam_gmapping_online scan:=/scan _base_frame:=base_link \
  _odom_frame:=odom _particles:=30

# save the map (in another terminal)
rosrun map_server map_saver -f my_map
```

With a bag as input, the online node needs simulated time:

```sh
roscore
roslaunch gmapping slam_gmapping_online.launch use_sim_time:=true
rosbag play --clock my_run.bag
```

For repeatable runs from bags, prefer the offline node: the online node is
always seeded from the current time and its result depends on the playback
timing.

### slam_gmapping_offline

Reads the bag files directly and processes them message by message, without
real-time pacing. The GMapping parameters are ROS parameters in the node's
private namespace (default node name: `slam_gmapping_offline`); the options
below are command-line arguments.

```
$ rosrun gmapping slam_gmapping_offline --help
Usage: rosrun gmapping slam_gmapping_offline -b BAGFILE1 [BAGFILE2 BAGFILE3 ...]

Options:
  -h [ --help ]              Display this information.
  -b [ --bags ] BAGFILE      ROS bag files to process
  --scantopic TOPIC (=/scan) Topic name for the 2D laser scanner data
  -s [ --start ] SEC (=0)    start SEC seconds into the bag files
  -d [ --duration ] SEC      play only SEC seconds from the bag files
  --log FILENAME             log the robot estimated data (base_frame pose)
                             into TUM files
  --seed N (=0)              seed for GMapping's random number generator (0:
                             from time)
  --spin BOOL (=1)           keep spinning after processing the bags
                             (map_saver, rviz); false: exit
```

| Option | Default | Description |
| --- | --- | --- |
| `-b`, `--bags` | required | One or more bag files. All bags are read as a single view, in timestamp order (e.g. a split recording, or bags with different topics). |
| `--scantopic` | `/scan` | `sensor_msgs/LaserScan` topic, exact string match with the topic name in the bag (including the leading `/`). |
| `-s`, `--start` | `0` | Offset [s] from the first message of all bags. |
| `-d`, `--duration` | to the end | Seconds to process after the start. |
| `--log` | disabled | Base file name of the TUM logs (see [Output files](#output-files-tum-logs)). The directory is created if missing. |
| `--seed` | `0` | Seed of GMapping's random number generator (`drand48`); `0` uses the current time. With the same bags, parameters and seed, runs are intended to be repeatable. |
| `--spin` | `true` | `true`: keep the node alive after processing (latched `/map` for `map_saver` and rviz). `false`: exit (batch runs). Accepts `true/false`, `1/0`, `yes/no`, `on/off`. |

Processing steps:

1. Open all bags and build a single, time-ordered view.
2. Load every `/tf_static` message of all bags into the tf2 buffer, regardless
   of `--start`.
3. For each message in `[start, start + duration]`:
   - scans on `--scantopic` go through a tf2 message filter (target
     `odom_frame`, queue of 10 scans) into the GMapping callback;
   - messages of type `tf2_msgs/TFMessage` on any topic go into the tf2
     buffer (as static only if the topic is `/tf_static`);
   - all other topics are ignored.
4. Print the elapsed wall time of step 3. This excludes the final map update.
5. Build and publish the map one last time, write the `_traj` log, close the
   logs and the bags.
6. `--spin true`: keep spinning; `--spin false`: exit.

The node fails with `no scan was processed (check the scan topic and the TF
tree)` if no scan made it through initialisation.

```sh
roscore   # rosrun needs a master (roslaunch starts one automatically)

# single bag, defaults
rosrun gmapping slam_gmapping_offline -b /data/run.bag

# two bags, other scan topic, 120 s starting 10 s in, logs, fixed seed,
# GMapping parameters as private ROS parameters
rosrun gmapping slam_gmapping_offline \
  -b /data/run_0.bag /data/run_1.bag --scantopic /front/scan \
  -s 10 -d 120 --log /data/results/run.txt --seed 42 \
  _particles:=50 _map_update_interval:=5.0

# while the node is spinning: save the map
rosrun map_server map_saver -f /data/results/run_map
```

Timing evaluation: with `map_update_interval <= 0`, no map is built while the
bags are processed, so the elapsed time printed in step 4 covers scan
processing only (scan matching and particle filter); the map is built once
afterwards.

Caveats:

- Odometry must be in the bags as TF (`odom_frame -> base_frame` on `/tf`).
  `nav_msgs/Odometry` topics are not read.
- Under `roslaunch`, the working directory of the node is `ROS_HOME`
  (default `~/.ros`): relative paths in `-b` and `--log` are resolved there.
  Use absolute paths.
- Do not set `/use_sim_time`: nothing publishes `/clock`, so
  `ros::Time::now()` would stay at zero.
- The node is initialised without a SIGINT handler. Ctrl+C while the bags are
  being processed terminates the process immediately: the `_traj` log is not
  written and the other logs may be truncated. Let the processing finish (use
  `--duration` to shorten a run).
- The banner `Press SPACE to pause/resume processing, 'q' to quit...` is a
  leftover: the interactive mode is disabled (see
  [MODIFICATIONS.md](MODIFICATIONS.md)).
- GMapping itself prints its internal state to stdout (it is verbose).

### Original nodes

`slam_gmapping`, `slam_gmapping_nodelet` and `slam_gmapping_replay` are
upstream code, unchanged; see the [gmapping ROS wiki](http://wiki.ros.org/gmapping)
for their documentation. They read the same parameters as the new nodes (see
[Parameters](#parameters)), with two differences: they do not publish
`~particlecloud`, and `map_update_interval <= 0` keeps the upstream meaning
(the map is updated at every filter update).

```sh
rosrun gmapping slam_gmapping scan:=/scan
roslaunch gmapping slam_gmapping_pr2.launch

# nodelet
rosrun nodelet nodelet standalone SlamGMappingNodelet

# replay: one bag, transforms from /tf only (no /tf_static)
rosrun gmapping slam_gmapping_replay --bag_filename /data/run.bag \
  --scan_topic /scan [--seed N] [--max_duration_buffer SEC] [--on_done CMD]
```

## Launch files

Both launch files load the parameters from a YAML file into the node's private
namespace (`params_file`, default
[`gmapping/config/slam_gmapping.yaml`](gmapping/config/slam_gmapping.yaml)).
Parameters set with `<param>` after the `<rosparam>` element override the YAML.

### slam_gmapping_online.launch

| Argument | Default | Description |
| --- | --- | --- |
| `scan_topic` | `scan` | Topic remapped onto the node's `scan` input. |
| `params_file` | `$(find gmapping)/config/slam_gmapping.yaml` | GMapping parameters. |
| `node_name` | `slam_gmapping_online` | Node name (and private namespace). |
| `use_sim_time` | `false` | Sets `/use_sim_time` to `true` (bag playback with `--clock`). |
| `rviz` | `false` | Starts rviz with `gmapping/rviz/rviz.rviz`. |
| `debug` | `false` | Sets `ROSCONSOLE_CONFIG_FILE` to `gmapping/config/rosconsole.config` (DEBUG). |
| `launch_prefix` | (empty) | Node launch prefix, e.g. `'xterm -e gdb --args'`. |

```sh
roslaunch gmapping slam_gmapping_online.launch scan_topic:=/front/scan rviz:=true
roslaunch gmapping slam_gmapping_online.launch params_file:=/abs/path/my_robot.yaml
```

### slam_gmapping_offline.launch

| Argument | Default | Description |
| --- | --- | --- |
| `bags` | required | Bag file(s), space-separated, absolute paths (no spaces in paths). |
| `scan_topic` | `/scan` | `--scantopic` |
| `start` | `0.0` | `--start` |
| `duration` | `0.0` | `--duration`; `0` or less: process to the end (option not passed). |
| `log` | (empty) | `--log`; empty: logging disabled (option not passed). |
| `seed` | `0` | `--seed` |
| `spin` | `true` | `--spin` |
| `params_file` | `$(find gmapping)/config/slam_gmapping.yaml` | GMapping parameters. |
| `node_name` | `slam_gmapping_offline` | Node name (and private namespace). |
| `required` | `false` | Shut down the whole launch when the node exits (use with `spin:=false`). |
| `rviz` | `false` | Starts rviz with `gmapping/rviz/rviz.rviz`. |
| `debug` | `false` | Sets `ROSCONSOLE_CONFIG_FILE` to `gmapping/config/rosconsole.config` (DEBUG). |
| `launch_prefix` | (empty) | Node launch prefix, e.g. `'xterm -e gdb --args'`. |

The command line of the node is assembled with `$(eval ...)`, so `--duration`
and `--log` are only passed when set.

```sh
roslaunch gmapping slam_gmapping_offline.launch bags:=/data/run.bag

roslaunch gmapping slam_gmapping_offline.launch \
  bags:="/data/run_0.bag /data/run_1.bag" scan_topic:=/front/scan \
  start:=10 duration:=120 log:=/data/results/run.txt seed:=42

# batch run: the node exits after processing and takes roslaunch down with it
roslaunch gmapping slam_gmapping_offline.launch bags:=/data/run.bag \
  log:=/data/results/run.txt spin:=false required:=true
```

## Parameters

All parameters are read once, at startup, from the node's private namespace
(`~`). The same names are used by all nodes in this package (new and original).
[`gmapping/config/slam_gmapping.yaml`](gmapping/config/slam_gmapping.yaml) lists
all of them with the default values. A value of the wrong type (e.g. a quoted
number) is ignored and the node silently uses the default; roscpp converts
integers to doubles and rounds doubles given to integer parameters.

### ROS wrapper

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `base_frame` | string | `base_link` | Robot base frame. Used for the laser planarity check and, in the offline node, for the logged poses. |
| `odom_frame` | string | `odom` | Odometry frame. `odom_frame -> base_frame` must be in TF. |
| `map_frame` | string | `map` | Frame of the map and of the estimated poses. Coincides with `odom_frame` at the first processed scan. |
| `throttle_scans` | int | `1` | Process 1 out of every N scans. |
| `map_update_interval` | double | `5.0` | Seconds (scan time) between map recomputations. `<= 0` (modified): no map updates while processing scans; the offline node builds the map once at the end; the online node then never builds nor publishes a map. |
| `transform_publish_period` | double | `0.05` | Online node: period [s] of the `map_frame -> odom_frame` broadcast; `0` disables it. Offline node: unused (only sets the default of `tf_delay`). |
| `tf_delay` | double | `transform_publish_period` | Online node: the `map_frame -> odom_frame` transform is stamped `now + tf_delay`. |

### Laser and scan matcher

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `maxRange` | double | `range_max - 0.01` | Maximum range of the sensor [m]. Default taken from the first scan. |
| `maxUrange` | double | `maxRange` | Maximum usable range [m]; beams are cropped to this value. For obstacle-free regions to appear as free space: `maxUrange < real sensor range <= maxRange`. |
| `sigma` | double | `0.05` | Sigma of the greedy endpoint matching. |
| `kernelSize` | int | `1` | Kernel in which to look for a correspondence. |
| `lstep` | double | `0.05` | Optimisation step in translation [m]. |
| `astep` | double | `0.05` | Optimisation step in rotation [rad]. |
| `iterations` | int | `5` | Iterations of the scan matcher. |
| `lsigma` | double | `0.075` | Sigma of a beam for the likelihood computation. |
| `ogain` | double | `3.0` | Gain used when evaluating the likelihood, to smooth the resampling effects. |
| `lskip` | int | `0` | Use only every (n+1)-th beam (`0`: all beams). |
| `minimumScore` | double | `0.0` | Minimum scan-matching score to accept the match. Scores go up to 600+; e.g. `50` helps against pose jumps in open spaces with short-range lasers. |

### Motion model

Standard deviations of the odometry error model.

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `srr` | double | `0.1` | Translation error as a function of translation. |
| `srt` | double | `0.2` | Translation error as a function of rotation. |
| `str` | double | `0.1` | Rotation error as a function of translation. |
| `stt` | double | `0.2` | Rotation error as a function of rotation. |

### Filter updates and resampling

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `linearUpdate` | double | `1.0` | Process a scan after the robot translates this far [m]. |
| `angularUpdate` | double | `0.5` | Process a scan after the robot rotates this far [rad]. |
| `temporalUpdate` | double | `-1.0` | Process a scan if the last processed one is older than this [s]; `< 0` disables time-based updates. |
| `resampleThreshold` | double | `0.5` | Resample when `Neff < resampleThreshold * particles`. |
| `particles` | int | `30` | Number of particles. |

A scan that triggers none of the three conditions only propagates the
particles with the motion model: no scan matching, no weight update, no map
update.

### Map

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `xmin`, `ymin` | double | `-100.0` | Initial map extent [m] (the map grows when needed). |
| `xmax`, `ymax` | double | `100.0` | Initial map extent [m]. |
| `delta` | double | `0.05` | Map resolution [m/cell]. |
| `occ_thresh` | double | `0.25` | Cells with occupancy above this value are published as `100`, the others as `0`; unobserved cells as `-1`. |

### Likelihood sampling

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `llsamplerange` | double | `0.01` | Translational sampling range [m]. |
| `llsamplestep` | double | `0.01` | Translational sampling step [m]. |
| `lasamplerange` | double | `0.005` | Angular sampling range [rad]. |
| `lasamplestep` | double | `0.005` | Angular sampling step [rad]. |

## ROS API

Relative names (`scan`, `map`, ...) are resolved in the node's namespace;
`~` is the node's private namespace. No node in this package provides actions.

### slam_gmapping_online

Subscribed topics:

| Topic | Type | Notes |
| --- | --- | --- |
| `scan` | `sensor_msgs/LaserScan` | Through a tf2 message filter (target `odom_frame`, queue of 10). |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | tf2 transform listener. |

Published topics:

| Topic | Type | Notes |
| --- | --- | --- |
| `map` | `nav_msgs/OccupancyGrid` | Latched. At every map update (`map_update_interval > 0`). |
| `map_metadata` | `nav_msgs/MapMetaData` | Latched. Together with `map`. |
| `~entropy` | `std_msgs/Float64` | Latched. Entropy of the normalised particle weights, at every map update, only if `> 0`. |
| `~particlecloud` | `geometry_msgs/PoseArray` | Every scan after initialisation; `frame_id = map_frame`, stamp of the scan. Poses of GMapping's particles, i.e. of the centred laser frame, not of `base_frame`. |
| `/tf` | `tf2_msgs/TFMessage` | `map_frame -> odom_frame`. |

Services:

| Service | Type | Notes |
| --- | --- | --- |
| `dynamic_map` | `nav_msgs/GetMap` | Returns the latest map; the call fails while no map has been built. |

TF:

| Direction | Transform | Notes |
| --- | --- | --- |
| required | `<laser frame> -> base_frame` | From `header.frame_id` of the scans; usually static. |
| required | `base_frame -> odom_frame` | Odometry, available at the scan stamps. |
| provided | `map_frame -> odom_frame` | Every `transform_publish_period` s, stamped `now + tf_delay`; identity until the first scan is processed. |

### slam_gmapping_offline

Inputs read from the bags (not subscribed): the `sensor_msgs/LaserScan` topic
given by `--scantopic`, and every `tf2_msgs/TFMessage` topic (`/tf_static` is
treated as static). The TF requirements are the same as for the online node
and must be satisfied by the transforms recorded in the bags.

Published topics:

| Topic | Type | Notes |
| --- | --- | --- |
| `map` | `nav_msgs/OccupancyGrid` | Latched. At every map update (`map_update_interval > 0`) and once after the bags. |
| `map_metadata` | `nav_msgs/MapMetaData` | Latched. Together with `map`. |
| `~particlecloud` | `geometry_msgs/PoseArray` | Same as the online node. |
| `/tf` | `tf2_msgs/TFMessage` | `map_frame -> base_frame` (best particle) at every scan, **only when `--log` is set**, stamped with `ros::Time::now()` (wall time). |

Services: none. The offline node does not publish `~entropy` nor
`map_frame -> odom_frame`.

### Original nodes

As upstream ([ROS wiki](http://wiki.ros.org/gmapping)): subscribe `scan` and
`/tf`; publish `map`, `map_metadata`, `~entropy` and `map_frame -> odom_frame`;
provide `dynamic_map`. `slam_gmapping_replay` reads the bag directly (`/tf` and
the scan topic only).

## Output files (TUM logs)

With `--log /data/results/run.txt`, the offline node writes:

| File | Rows | Content |
| --- | --- | --- |
| `run_gmapping_pose.txt` | One per scan after initialisation (if `odom_frame -> base_frame` is available at its stamp) | Pose of the best particle at that scan. Between filter updates, all particles are propagated by sampling the motion model, so this pose random-walks around odometry until the next update. |
| `run_gmapping_tf.txt` | Same as above | `map_frame -> odom_frame` from the last filter update composed with `odom_frame -> base_frame` at the scan stamp: what a `/tf` consumer obtains from the online / original nodes. |
| `run_gmapping_traj.txt` | One per filter update, written after the bags | Trajectory of the final best particle, consistent with the final map (includes the corrections made by resampling). The usual choice for trajectory-error evaluation. |

Format, one pose per line, no header, 9 decimals:

```
timestamp x y z qx qy qz qw
```

- `timestamp`: `header.stamp` of the scan [s];
- pose of `base_frame` in `map_frame`. GMapping estimates the pose of the
  centred laser frame; the static `laser -> base_frame` transform (looked up
  once, at the first logged scan) is applied before logging;
- planar: `z = 0`, roll and pitch are zero (only the yaw is kept).

Example evaluation with [evo](https://github.com/MichaelGrupp/evo):

```sh
evo_ape tum groundtruth.txt /data/results/run_gmapping_traj.txt -a --plot
```

## Debugging

DEBUG log level: launch with `debug:=true` (uses
[`gmapping/config/rosconsole.config`](gmapping/config/rosconsole.config)), or
export `ROSCONSOLE_CONFIG_FILE` before `rosrun`.

GDB: build with `-DCMAKE_BUILD_TYPE=Debug` or `RelWithDebInfo` (see
[Build](#build)) and use the `launch_prefix` argument:

```sh
roslaunch gmapping slam_gmapping_offline.launch bags:=/data/run.bag \
  launch_prefix:="xterm -e gdb --args"

# (gdb) run
# if the program breaks:
# (gdb) backtrace
# (gdb) thread apply all bt
```

## License and credits

BSD and Apache 2.0, as upstream (see `gmapping/package.xml`). GMapping by
Giorgio Grisetti, Cyrill Stachniss and Wolfram Burgard
([OpenSLAM](https://openslam-org.github.io/gmapping.html)); ROS wrapper by
Brian Gerkey and contributors. The `~particlecloud` output follows a request by
Héber Sobreira (INESC TEC).
