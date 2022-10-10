
from tank_msgs.msg import LevelState, QualityState, TemperatureState, PumpState


def level_state_str(state: LevelState) -> str:
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
    return "INVALID STATE"


def quality_state_str(state: QualityState) -> str:
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
    return "INVALID STATE"


def quality_state_pretty_str(state: QualityState) -> str:
    if state == QualityState.GOOD:
        return "[INFO] Nutrition if of good quality"
    elif state == QualityState.UNKNOWN:
        return "[WARN] Quality of nutrition is not known"
    elif state == QualityState.BAD_TDS:
        return "[CRITICAL] Amount of total dissolved solids is not within desired range"
    elif state == QualityState.BAD_PH:
        return "[CRITICAL] Nutrition pH is not within desired range"
    elif state == QualityState.BAD_ALL:
        return "[CRITICAL] Both TDS and pH are not within desired range"
    return "INVALID STATE"


def temperature_state_str(state: TemperatureState) -> str:
    if state == TemperatureState.GOOD:
        return "GOOD"
    elif state == TemperatureState.UNKNOWN:
        return "UNKNOWN"
    elif state == TemperatureState.TOO_COLD:
        return "TOO_COLD"
    elif state == TemperatureState.TOO_HOT:
        return "TOO_HOT"
    return "INVALID STATE"


def temperature_state_pretty_str(state: TemperatureState) -> str:
    if state == TemperatureState.GOOD:
        return "[INFO] Liquid temperature is within desired range"
    elif state == TemperatureState.UNKNOWN:
        return "[WARN] Liquid temperature is unknown"
    elif state == TemperatureState.TOO_COLD:
        return "[CRITICAL] Liquid is too cold"
    elif state == TemperatureState.TOO_HOT:
        return "[CRITICAL] Liquid is too hot"
    return "INVALID STATE"


def pump_state_str(state: PumpState) -> str:
    if state == PumpState.STATE_OFF:
        return "OFF"
    elif state == PumpState.STATE_ON:
        return "ON"
    return "INVALID STATE"


def pump_state_pretty_str(state: PumpState) -> str:
    if state == PumpState.STATE_OFF:
        return "[INFO] Pump is not running"
    elif state == PumpState.STATE_ON:
        return "[INFO] Pump is running"
    return "INVALID STATE"
