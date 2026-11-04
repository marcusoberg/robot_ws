uint64 session
string instance
uint64 sequence
string action_spec_id
float64[] positions
# Optional feed-forward velocities in joint units/s; empty means zero.
float64[] velocities
# Default preserves sampled-position producers and simulation.
uint8 SAMPLED=0
uint8 JOINT_MOVE=1
uint8 JOINT_PATH=2
uint8 LIVE_TARGET=3
uint8 HEARTBEAT=4
uint8 motion_kind
uint64 motion_id
float64 duration
float64 smoothing_period
# JOINT_PATH: increasing seconds from zero and row-major full joint positions.
float64[] trajectory_times
float64[] trajectory_positions
