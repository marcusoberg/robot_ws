string mode
string instance
string reason
string source
uint64 session
uint64 sequence
uint64 delivered_sequence
uint64 episode
int64 simulation_time_ns
int64 execution_time_ns
string action_spec_id
string[] joints
float64[] measured_positions
float64[] measured_velocities
float64[] commanded_reference
bool recording
string command_id
string command_result
bool command_accepted
string motion_execution
uint64 motion_id
float64 motion_elapsed_seconds
bool motion_complete
# Post-homing arm pose from the backend, in radians.
float64[] standby_positions
# Workspace FK at the measured joint positions.
string tcp_link
geometry_msgs/PoseStamped tcp_pose
