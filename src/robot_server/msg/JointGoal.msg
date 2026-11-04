uint64 session
string instance
uint64 sequence
string action_spec_id
float64[] positions
# Optional sampled-source velocity; the Python API computes its own derivatives.
float64[] velocities
# Sampled and live targets use the API servo stream.
uint8 SAMPLED=0
uint8 JOINT_MOVE=1
uint8 LIVE_TARGET=3
uint8 HEARTBEAT=4
uint8 motion_kind
uint64 motion_id
float64 duration
