# Target.msg - a single detected waste target in the shared database

int32 target_id
geometry_msgs/Point position
string status              # UNASSIGNED, ASSIGNED, LOCKED, COMPLETED
int32 assigned_robot_id    # -1 if unassigned
float32 priority           # higher = more urgent, default 1.0
builtin_interfaces/Time detected_at
int32 detecting_robot_id
