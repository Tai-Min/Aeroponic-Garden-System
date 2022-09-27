import pathlib
import RPi.GPIO as GPIO
from tank_msgs.msg import LevelState, QualityState


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


def read_ds18b20(addr: str) -> tuple:
    path = pathlib.Path("/sys/bus/w1/devices", addr, "temperature")

    try:
        with open(path, "r") as device:
            val = device.readline()
            if val:
                val = float(val) / 1000.0
                return (val, True)
            else:
                return (0, False)
    except:
        return (0, False)
