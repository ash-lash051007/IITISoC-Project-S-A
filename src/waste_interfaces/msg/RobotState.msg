# Periodic telemetry a robot publishes about itself.

string robot_name           # e.g. "tb3_0", "tb3_1"
int32 robot_id
geometry_msgs/Pose pose
string status                # IDLE, NAVIGATING, ASSIGNED, BUSY, OFFLINE
int32 current_task_id        # -1 if none
float32 battery_level        # 0-100
builtin_interfaces/Time last_update
