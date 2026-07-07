# Available pipeline types
string TYPE_EXAMPLE = "example"
string TYPE_WAYPOINT_MARKER = "waypoint_marker"
string TYPE_GATE_SEARCH = "task_gate_search"
string TYPE_GATE_APPROACH = "task_gate_approach"
string TYPE_GATE_YOLO = "task_gate_yolo"
string TYPE_GATE_DONE = "task_gate_done"
string TYPE_NAVIGATE_FORWARD = "task_navigate_forward"
string TYPE_PITCH_FLIP = "task_pitch_flip"
string TYPE_STAY_STILL = "task_stay_still"
string TYPE_FORCE_UP = "task_force_up"
string TYPE_SWAY_LEFT = "task_sway_left"
string TYPE_SWAY_RIGHT = "task_sway_right"
string TYPE_NAVIGATE_BACKWARDS = "task_navigate_backwards"

# type must be one of the above pipeline types (i.e "circle_approach")
string type
