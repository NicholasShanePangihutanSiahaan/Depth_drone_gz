int32 id

float32 x
float32 y
float32 z

float32 confidence

bool inspected

bool validated

# Current sensor visibility is separate from persistent map membership.
bool visible

# Seconds since the last accepted ZED/PCL observation.
float32 observation_age

uint8 orbit_count
