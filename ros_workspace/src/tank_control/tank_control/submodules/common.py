import RPi.GPIO as GPIO
from tank_msgs.msg import LevelState


def state_pretty_str(state: LevelState) -> str:
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


def state_str(state: LevelState) -> str:
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
