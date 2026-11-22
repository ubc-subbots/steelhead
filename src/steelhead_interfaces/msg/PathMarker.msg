# Orange path marker seen by the bottom camera, one message per image.
# Directions are relative to the robot: the top of the image is the robot's front.
std_msgs/Header header

# True if a marker was found in this image; the other fields are 0 otherwise
bool detected

# Turn (rad, -pi/2 to pi/2) needed to line the robot up with the marker, using
# the marker end nearest the robot's heading. Positive means turn left.
float32 heading_error

# Distance from the image centre to the marker's centre line, as a fraction of
# half the image width. Positive means the line is to the robot's right.
float32 lateral_offset

# Centre of the visible part of the marker, as a fraction of half the image
# height. Positive means behind the image centre (towards the robot's back).
float32 centroid_y
