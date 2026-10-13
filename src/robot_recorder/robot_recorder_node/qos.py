from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LATEST = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST
)
EVENTS = QoSProfile(
    depth=64, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
)
DESCRIPTION = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# Delivery counters and identity gaps remain necessary regardless of these settings.

# Canonical captures use bounded retransmission; display projections stay latest.
# Recorder delivery counters still determine completeness independently of QoS.
CAPTURES = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
