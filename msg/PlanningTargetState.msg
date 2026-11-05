# One authoritative editable target. In bimanual mode target_pose is relative.
bool bimanual
string server_id
uint64 revision
string marker_namespace
string frame_id
string[] arm_names
string[] tool_frames
geometry_msgs/Pose[] tool_poses
geometry_msgs/PoseStamped target_pose
bool visible
bool locked
string owner_id
string action_name
uint8[16] goal_id
string status
