from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_sensor = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)