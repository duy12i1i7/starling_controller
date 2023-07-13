# PX4 Examples

In general the files themselves should be fairly self explanatory. See the [root README.md](../README.md) for details on *linux/windows*. As a brief breakdown:

- **core** - Starts the core simulator/sitl/mavros/ros-web-bridge stack only. Recommended for low level use or use primarily with a GCS (QGC or Mission Planner)
- **ui-example** - Starts core with a very simple example user interface on [`localhost:3000`](https://localhost:3000). 
- **simple-offboard** - Starts the core simulator with a [simple-offboard controller](https://github.com/StarlingUAS/starling_simple_offboard), [simple-allocator](https://github.com/StarlingUAS/starling_allocator) and [Trajectory follower Web GUI](https://github.com/StarlingUAS/starling_ui_dashly). The web gui has a trajectory uploader, and the simple-offboard has a trajecotry follower module built in. Ideal for those who want to test path planning or other higher level functionality. 
- **simple-offboard.2-drones**/**simple-offboard.3-drones** - manually starts 2 or 3 core stacks with associated simple-offboard controllers for simple multi-drone testing. **Warning: this can be very CPU intensive.** 