from tank_msgs.msg import State


def state_str(state: int) -> str:
    if state == State.GOOD:
        return "GOOD"
    elif state == State.UNKNOWN:
        return "UNKNOWN"
    elif state == State.LEVEL_CRITICAL_LOW:
        return "LEVEL_CRITICAL_LOW"
    elif state == State.LEVEL_CRITICAL_HIGH:
        return "LEVEL_CRITICAL_HIGH"
    elif state == State.MEASUREMENT_DISABLED:
        return "MEASUREMENT_DISABLED"
    elif state == State.TANK_OPEN:
        return "TANK_OPEN"
    return "INVALID"
