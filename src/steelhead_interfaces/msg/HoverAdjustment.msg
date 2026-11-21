uint8 PARTIAL = 0
uint8 FULL = 1

geometry_msgs/Wrench input
uint8 type

# Heading change, used by hover_at_depth when hold_yaw is true.
# change_yaw true: held heading becomes current heading + yaw_offset (rad, +ve = turn left).
# change_yaw false: held heading is left unchanged.
bool change_yaw
float64 yaw_offset
