# RobotState.msg - published periodically by each robot

int32 robot_id
geometry_msgs/Point position
bool available            # false if mid-task or offline
int32 current_target_id   # -1 if idle
float32 battery_level     # 0-100, optional, default 100
