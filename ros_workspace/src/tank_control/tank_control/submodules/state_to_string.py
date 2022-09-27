
from tank_msgs.msg import LevelState, QualityState, TemperatureState


def level_state_pretty_str(state: LevelState) -> str:
    if state == LevelState.GOOD:
        return "[INFO] Tank is operating properly."
    elif state == LevelState.UNKNOWN:
        return "[WARN] Tank's state can't be determined."
    elif state == LevelState.LEVEL_CRITICAL_LOW:
        return "[CRITICAL] Too small liquid level to operate properly! Pump dry run possible!"
    elif state == LevelState.LEVEL_CRITICAL_HIGH:
        return "[CRITICAL] Too big liquid level to operate properly! Possible overflow and sensor damage!"
    elif state == LevelState.MEASUREMENT_DISABLED:
        return "[INFO] Measurement is disabled via physical button or ROS2 service."
    elif state == LevelState.TANK_OPEN:
        return "[WARN] Tank is open."
    return "INVALID"


def level_state_str(state: QualityState) -> str:
    if state == QualityState.GOOD:
        return "GOOD"
    elif state == QualityState.UNKNOWN:
        return "UNKNOWN"
    elif state == QualityState.BAD_TDS:
        return "BAD_TDS"
    elif state == QualityState.BAD_PH:
        return "BAD_PH"
    elif state == QualityState.BAD_ALL:
        return "BAD_ALL"
    return "INVALID"


def quality_state_str(state: LevelState) -> str:
    if state == LevelState.GOOD:
        return "GOOD"
    elif state == LevelState.UNKNOWN:
        return "UNKNOWN"
    elif state == LevelState.LEVEL_CRITICAL_LOW:
        return "LEVEL_CRITICAL_LOW"
    elif state == LevelState.LEVEL_CRITICAL_HIGH:
        return "LEVEL_CRITICAL_HIGH"
    elif state == LevelState.MEASUREMENT_DISABLED:
        return "MEASUREMENT_DISABLED"
    elif state == LevelState.TANK_OPEN:
        return "TANK_OPEN"
    return "INVALID"


def temperature_state_str(state: TemperatureState) -> str:
    if state == TemperatureState.GOOD:
        return "GOOD"
    elif state == TemperatureState.UNKNOWN:
        return "UNKNOWN"
    elif state == TemperatureState.TOO_COLD:
        return "TOO_COLD"
    elif state == TemperatureState.TOO_HOT:
        return "TOO_HOT"
    return "INVALID"
