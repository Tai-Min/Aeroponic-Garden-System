import RPi.GPIO as GPIO
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


class ClosureSensor:
    def __init__(self, pin: int, name, callback=None, edge=GPIO.BOTH) -> None:
        self.__pin = pin
        GPIO.setup(self.__pin, GPIO.IN)
        if callback:
            GPIO.add_event_detect(
                self.__pin, edge, lambda _: callback(name))

    def is_open(self) -> bool:
        return GPIO.input(self.__pin)


class Button(ClosureSensor):
    def __init__(self, pin: int, name, callback=None, edge=GPIO.RISING) -> None:
        self.__pin = pin
        GPIO.setup(self.__pin, GPIO.IN)
        if callback:
            GPIO.add_event_detect(
                self.__pin, edge, lambda _: callback(name))

    def is_pressed(self) -> bool:
        return not GPIO.input(self.__pin)


class LED:
    def __init__(self, pin: int) -> None:
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, 3)
        self._pwm.start(0)

    def __del__(self) -> None:
        self._pwm.ChangeDutyCycle(0)


class LevelLED(LED):
    def set_state(self, state: LevelState) -> None:
        if state == LevelState.GOOD:
            self._pwm.ChangeDutyCycle(0)
        elif state == LevelState.UNKNOWN:
            self._pwm.ChangeDutyCycle(50)
            self._pwm.ChangeFrequency(1)
        elif state == LevelState.LEVEL_CRITICAL_HIGH:
            self._pwm.ChangeDutyCycle(50)
            self._pwm.ChangeFrequency(2)
        elif state == LevelState.LEVEL_CRITICAL_LOW:
            self._pwm.ChangeDutyCycle(50)
            self._pwm.ChangeFrequency(0.5)
        elif state in [LevelState.TANK_OPEN, LevelState.MEASUREMENT_DISABLED]:
            self._pwm.ChangeDutyCycle(100)


class QualityLED(LED):
    def set_state(self, state: QualityState) -> None:
        if state == QualityState.GOOD:
            self._pwm.ChangeDutyCycle(0)
        elif state == QualityState.UNKNOWN:
            self._pwm.ChangeDutyCycle(50)
            self._pwm.ChangeFrequency(1)
        elif state == QualityState.BAD_PH:
            self._pwm.ChangeDutyCycle(50)
            self._pwm.ChangeFrequency(2)
        elif state == QualityState.BAD_TDS:
            self._pwm.ChangeDutyCycle(50)
            self._pwm.ChangeFrequency(4)
        elif state == QualityState.BAD_ALL:
            self._pwm.ChangeDutyCycle(100)
