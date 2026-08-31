# Published once by pose_sampler_node (Desktop) after the robot has reached and
# settled at a calibration pose. header.stamp carries the settle timestamp.
std_msgs/Header header
int32 pose_index
geometry_msgs/Point p_base
bool is_validation
