# Modifications log

Fork of [ros-perception/slam_gmapping](https://github.com/ros-perception/slam_gmapping),
based on upstream commit `eec8606` (merge of PR #110, after release 1.4.2).

The original node sources are unchanged: `gmapping/src/slam_gmapping.{h,cpp}`,
`main.cpp`, `nodelet.cpp`, `replay.cpp`. The upstream catkin changelog
(`gmapping/CHANGELOG.rst`) is left as is; changes of this fork are logged here.

## Files

| File | Status |
| --- | --- |
| `gmapping/src/slam_gmapping_ros1_api.{h,cpp}` | new: common wrapper (tf2) |
| `gmapping/src/slam_gmapping_ros1_online{,_node}.{h,cpp}` | new: online node |
| `gmapping/src/slam_gmapping_ros1_offline{,_node}.{h,cpp}` | new: offline node |
| `gmapping/launch/slam_gmapping_{online,offline}.launch` | new |
| `gmapping/config/slam_gmapping.yaml` | new: parameter file |
| `gmapping/config/rosconsole.config` | new: DEBUG logging |
| `gmapping/rviz/rviz.rviz` | new |
| `gmapping/CMakeLists.txt` | modified: new targets (C++17), dependencies, install |
| `gmapping/package.xml` | modified: format 3, `<depend>` entries |
| `.clang-format`, `.gitignore`, `README.md` | new / modified |

## Behaviour differences (new nodes only)

- tf2 instead of tf; scan message filter queue of 10 scans (upstream: 5).
- `map_update_interval <= 0`: no map updates while scans are processed
  (upstream: map updated at every filter update). The offline node builds the
  map once after the bags; the online node never builds one.
- `~particlecloud` (`geometry_msgs/PoseArray`) published.
- Laser planarity check uses a unit up-vector (fix, see `2c91464`).

## Log (newest first)

### 2026-10-06

- `4051469` offline: fixed seed and `base_frame` logged poses.
  - New option `--seed N` (`0`: seed from the current time).
  - New option `--spin BOOL` (default `true`); `false` exits after processing
    (batch runs, with `required="true"` in the launch file).
  - Logged poses are now of `base_frame` (previously of the centred laser
    frame estimated by GMapping), planar (`z = 0`, yaw only).
  - Three logs instead of one: `_gmapping_pose`, `_gmapping_tf` (new) and
    `_gmapping_traj` (new, final best-particle trajectory).
  - TF `map_frame -> base_frame` now uses the `base_frame` pose.
  - `run()` throws if no scan was processed, instead of updating the map of an
    uninitialised mapper.

### 2026-08-03

- `1bdeb7f` publish `~particlecloud` (all particles, every scan, in
  `map_frame`) in both new nodes. Credit: Héber Sobreira (INESC TEC).
- `f84d4b0` `map_update_interval <= 0` disables map updates while processing
  scans (timing evaluation); `updateMap()` no longer takes the scan (caches
  the beam count and the stamp of the last scan).

### 2026-07-08

- `0193853` offline: interactive mode (SPACE to pause/resume, `q` to quit)
  disabled; the code is commented out (see the commit diff to re-enable it).

### 2026-06-04

- `086fc33` offline: publish TF `map_frame -> base_frame` (best particle) at
  every scan, for rviz (only when `--log` is set).

### 2026-06-02

- `20352d4` fix: the directory part of `--log` was ignored (the file was always
  written in the working directory); log suffix renamed from `_gmapping_laser`
  to `_gmapping_pose`.

### 2025-09-12

- `edf992c` CMake: remove `Boost::program_options` from the `add_dependencies`
  of the offline target.

### 2025-08-06

- `96caa62`, `37e2015` terminal handling fixes for the interactive mode
  (restore canonical mode, filter escape sequences).

### 2025-07-29

- `c598272` offline: load all `/tf_static` messages of all bags before
  processing (static transforms available when `--start > 0`); `scan_topic`
  and `start` launch arguments.
- `757c094` offline: pause/resume/quit with time information;
  `config/rosconsole.config`; GMapping info stream to stdout; laser angles in
  degrees in the debug output; rviz configuration update.
- `fccf2db` README update.

### 2025-07-10

- `2c91464` fix: the planarity check transformed a `Vector3Stamped` with
  `z = 1 + laser height`; tf2 applies only the rotation to vectors, so the
  `|z| = 1` test failed for lasers mounted above `base_frame`. Now a unit
  vector. The offline node prints the elapsed time of the processing loop.
- `fea6fbd` TUM pose logging (`--log`); pose logged at every scan (not only at
  filter updates); unused `program_options` removed from the API.
- `c51e899` offline: `ros::spin()` after processing (`map_saver`, rviz).
- `4999aa3`, `0211980` README updates.

### 2025-07-09

- `7fcc4a9` new node `slam_gmapping_offline` and its launch file.
- `e3785b7` new common class `SLAMGMappingROS1API` (tf2) and node
  `slam_gmapping_online`; launch file, rviz configuration, `.clang-format`,
  `package.xml` format 3, C++17 targets.
