# Navigation Infrastructure

## Planner Server
Run planner server: 
```bash
ros2 run planner_server planner_server
```

The planner server provides the `navigate_to_goal` action, which navigates the robot from the given start coordinate to the given goal coordinate within the given costmap. The action publishes the robot's current distance from the start coordinate to the feedback topic. The action's interface is as follows:
```
# Input
nav_msgs/OccupancyGrid costmap
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	MapMetaData info
		builtin_interfaces/Time map_load_time
			int32 sec
			uint32 nanosec
		float32 resolution
		uint32 width
		uint32 height
		geometry_msgs/Pose origin
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
	int8[] data
Coordinate2D start
	int64 x
	int64 y
Coordinate2D goal
	int64 x
	int64 y
---

# Final output
bool success
---

# Feedback
geometry_msgs/Pose distance_from_start
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1
```

You can call the action from the terminal with this command, inputting your own data for the start, goal, and costmap (the planner_server node must be running for this command to work):
```
ros2 action send_goal /navigate_to_goal infra_interfaces/action/NavigateToGoal "{costmap: {header: {frame_id: 'map'}, info: {width: 2, height: 2, resolution: 1.0, origin: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}, data: [0, 0, 0, 0]}, start: {x: 0, y: 0}, goal: {x: 0, y: 1}}"
```