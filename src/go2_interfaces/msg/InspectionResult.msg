string station_id
geometry_msgs/Pose station_pose
bool visual_anomaly_detected
bool telemetry_anomaly_detected
string anomaly_description
float32 confidence          # reduced to 0.6 when collected under degraded sensing
builtin_interfaces/Time inspected_at
bool degraded_mode          # true if lidar was offline during this inspection
