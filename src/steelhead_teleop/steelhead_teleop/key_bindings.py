"""
Single source of truth for teleop key bindings.

"""

from geometry_msgs.msg import Wrench

from steelhead_interfaces.srv import ActuatorsCommand

# key: (field, axis, sign)
WRENCH_BINDINGS = {
    "w": ("force", "x", +1),
    "s": ("force", "x", -1),
    "a": ("force", "y", +1),
    "d": ("force", "y", -1),
    "q": ("force", "z", +1),
    "z": ("force", "z", -1),
    "up": ("torque", "x", -1),
    "down": ("torque", "x", +1),
    "e": ("torque", "y", -1),
    "c": ("torque", "y", +1),
    "left": ("torque", "z", +1),
    "right": ("torque", "z", -1),
}

ACTUATOR_BINDINGS = {
    "o": ActuatorsCommand.Request.FIRE_LEFT_TORPEDO,
    "p": ActuatorsCommand.Request.FIRE_RIGHT_TORPEDO,
    "k": ActuatorsCommand.Request.OPEN_CLAW,
    "l": ActuatorsCommand.Request.CLOSE_CLAW,
}

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def wrench_for_key(key_name, force_mags, torque_mags):
    """
    Builds the Wrench for a bound key, or returns None if the key is unbound.

    @param key_name: Canonical key name (a WRENCH_BINDINGS key)
    @param force_mags: [x, y, z] force magnitudes
    @param torque_mags: [x, y, z] torque magnitudes
    """
    binding = WRENCH_BINDINGS.get(key_name)
    if binding is None:
        return None
    field, axis, sign = binding
    mags = force_mags if field == "force" else torque_mags
    msg = Wrench()
    setattr(getattr(msg, field), axis, sign * mags[_AXIS_INDEX[axis]])
    return msg
