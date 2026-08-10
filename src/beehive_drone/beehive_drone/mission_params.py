#!/usr/bin/env python3
"""Default configuration shared by all beehive mission nodes."""

import math


class MissionConfig:
    # Internal navigation frame for the real vehicle. The mission, PCL map,
    # and MAVROS local pose must all be aligned to the same local odometry frame.
    WORLD_FRAME = "map"
    PCL_FRAME = "plantation"

    # ==========================================
    # 1. Parameter Misi & Eksplorasi (mission_state_machine.py)
    # ==========================================
    FLIGHT_ALTITUDE = 3.0        # meter
    EXPLORE_SPEED = 1.0          # m/s
    CRAB_SPEED = 0.5             # m/s
    END_OF_ROW_DIST = 10.0       # meter
    END_OF_FARM_DIST = 20.0      # meter
    APPROACH_SAFE_DIST = 2.0     # meter
    HOVERING_PERIODE = 30.0      # periode dikali 1.0 adalah waktu hovering sebelum loitering
    POST_ORBIT_HOVER_TIME = 3.0  # detik stabilisasi setelah satu orbit selesai
    HOME_ALIGN_TIME = 2.0        # detik mempertahankan arah menuju titik takeoff
    HOME_HOVER_TIME = 3.0        # detik hover di atas titik takeoff sebelum land
    HOME_POSITION_TOLERANCE = 0.7  # meter
    HOME_YAW_TOLERANCE = math.radians(10.0)

    # RC/pilot takeover. Any confirmed flight-mode change away from the
    # autonomous mode permanently disables autonomy for the current process.
    ENABLE_RC_TAKEOVER = True
    RC_TAKEOVER_CONFIRM_SEC = 0.30

    # Map safety gate
    REQUIRE_TREE_MAP = True
    MAP_STARTUP_TIMEOUT_SEC = 35.0
    MAP_LOSS_GRACE_SEC = 3.0
    MIN_READY_TREES = 1

    # Stable hover detection
    HOVER_ALT_TOLERANCE = 0.25
    HOVER_SPEED_TOLERANCE = 0.20
    HOVER_STABLE_SEC = 1.5

    # ==========================================
    # 5. Parameter Pemetaan Pohon (tree_mapper.py)
    # ==========================================
    TREE_MERGE_DISTANCE = 6.0           # meter (Jarak minimum untuk menggabungkan pohon yang sama)
    TREE_MAX_CONFIDENCE = 1.0           # Maksimum confidence untuk pohon
    TREE_NEW_CONFIDENCE = 0.2      # Confidence awal untuk pohon baru
    TREE_CONFIDENCE_INCREMENT = 0.25    # Penambahan confidence setiap deteksi
    TREE_CONFIDENCE_DECAY = 0.01         # Penurunan confidence setiap deteksi hilang
    TREE_TIMEOUT = 30.0                 # detik
