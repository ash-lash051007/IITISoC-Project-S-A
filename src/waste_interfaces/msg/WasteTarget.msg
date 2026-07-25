# A single waste target tracked by the shared database.
# target_id == ArUco marker ID (0-11), so it is already a globally unique key.

int32 target_id
geometry_msgs/Point position
string waste_type          # e.g. "hazardous", "recyclable", "general"
string status               # PENDING, ASSIGNED, COLLECTED
float32 confidence
string first_detected_by
string last_detected_by
int32 detection_count
string assigned_robot        # robot currently working this target, "" if none
string collected_by          # robot that completed this target, "" if not yet collected
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
