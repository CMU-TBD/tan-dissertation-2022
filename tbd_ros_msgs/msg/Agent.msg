# An abstraction of a agent's state

# (required)
Header header
string agent_type_unique # A unique string that describe the type of agent
string agent_id #ID for the robot.

# (Optional)

# High level concepts of the robot
geometry_msgs/PoseStamped attention
geometry_msgs/PolygonStamped bounding_box

# Detail states
sensor_msgs/JointState joints_state
sensor_msgs/BatteryState battery_state
geometry_msgs/PoseStamped center_pose
