import time
import rclpy
from rclpy.node import Node
from tank_msgs.msg import State, Measurement
from tank_msgs.srv import SetEnabled
import RPi.GPIO as GPIO
from .submodules.common import state_str


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
        self.__pwm = GPIO.PWM(pin, 3)
        self.__pwm.start(0)

    def __del__(self) -> None:
        self.__pwm.ChangeDutyCycle(0)

    def set_state(self, state: int) -> None:
        if state == State.GOOD:
            self.__pwm.ChangeDutyCycle(0)
        elif state == State.UNKNOWN:
            self.__pwm.ChangeDutyCycle(50)
            self.__pwm.ChangeFrequency(1)
        elif state == State.LEVEL_CRITICAL_HIGH:
            self.__pwm.ChangeDutyCycle(50)
            self.__pwm.ChangeFrequency(2)
        elif state == State.LEVEL_CRITICAL_LOW:
            self.__pwm.ChangeDutyCycle(50)
            self.__pwm.ChangeFrequency(0.5)
        elif state in [State.TANK_OPEN, State.MEASUREMENT_DISABLED]:
            self.__pwm.ChangeDutyCycle(100)


class LevelObserver(Node):
    def __init__(self) -> None:
        super().__init__("level_observer")

        self.declare_parameter("frame_id", "tank")
        self.declare_parameter("min_level", 0.0)
        self.declare_parameter("max_level", 0.2)
        self.declare_parameter("check_period", 1.0)
        self.declare_parameter("value_invalidate_time", 5.0)

        self.declare_parameter("closure_sensor_pin", 27)
        self.declare_parameter("led_pin", 17)
        self.declare_parameter("button_pin", 18)

        self.__frame = self.get_parameter(
            "frame_id").get_parameter_value().string_value
        self.get_logger().info(f"Frame ID is: {self.__frame}")
        self.__min_level = self.get_parameter(
            "min_level").get_parameter_value().double_value
        self.get_logger().info(f"Minimum liquid level is: {self.__min_level}m")
        self.__max_level = self.get_parameter(
            "max_level").get_parameter_value().double_value
        self.get_logger().info(f"Maximum liquid level is: {self.__max_level}m")
        self.__check_period = self.get_parameter(
            "check_period").get_parameter_value().double_value
        self.get_logger().info(
            f"Level will be checked once per: {self.__check_period}s")
        self.__value_invalidate_time = self.get_parameter(
            "value_invalidate_time").get_parameter_value().double_value
        self.get_logger().info(
            f"Sensor reading will be invalidated after: {self.__value_invalidate_time}s")

        self.fst = True
        self.__enabled = True
        self.__previous_state = State.UNKNOWN
        self.__level_sensor_value = -1
        self.__last_value_stamp = -1

        closure_sensor_pin = self.get_parameter(
            "closure_sensor_pin").get_parameter_value().integer_value
        self.get_logger().info(
            f"Closure sensor GPIO (BCM) is: {closure_sensor_pin}")
        self.__closure_sensor = ClosureSensor(
            closure_sensor_pin, "closure sensor", self.__on_timer_callback)
        disable_button_pin = self.get_parameter(
            "button_pin").get_parameter_value().integer_value
        self.get_logger().info(
            f"Disable button GPIO (BCM) is: {disable_button_pin}")
        self.__disable_button = Button(
            disable_button_pin, "disable button", self.__on_btn_pressed_callback)
        led_pin = self.get_parameter(
            "led_pin").get_parameter_value().integer_value
        self.get_logger().info(
            f"LED GPIO (BCM) is: {led_pin}")
        self.__led = LED(led_pin)

        self.__level_sensor_subscriber = self.create_subscription(
            Measurement, "level_raw", self.__on_level_received_callback, 10)
        self.__level_publisher = self.create_publisher(
            Measurement, "level", 10)
        self.__state_publisher = self.create_publisher(State, "state", 10)
        self.__control_service = self.create_service(
            SetEnabled, "set_enabled", self.__on_set_enable_callback)
        self.__timer = self.create_timer(
            self.__check_period, self.__on_timer_callback)

        self.get_logger().info("Level observer is running")

    def __on_level_received_callback(self, msg: Measurement) -> None:
        self.__level_sensor_value = msg.val
        self.__last_value_stamp = time.time()

    def __on_btn_pressed_callback(self, name) -> None:
        if self.__disable_button.is_pressed():
            return

        self.__enabled = not self.__enabled

        if self.__enabled:
            self.get_logger().info("Measurement enabled via button press")
        else:
            self.get_logger().info("Measurement disabled via button press")

        self.__on_timer_callback(name)

    def __on_set_enable_callback(self, request: SetEnabled.Request, response: SetEnabled.Request) -> None:

        if request.val:
            self.get_logger().info("Measurement enabled via service call")
        else:
            self.get_logger().info("Measurement disabled via service call")

        self.__enabled = request.val
        self.__on_timer_callback("service call")
        return response

    def __on_timer_callback(self, name: str = None) -> None:
        if name:
            self.get_logger().info(
                f"Timer callback called async by {name} event")

        if self.__last_value_stamp >= 0 and (time.time() - self.__last_value_stamp) > self.__value_invalidate_time:
            self.__last_value_stamp = -1
            self.__level_sensor_value = -1
            self.get_logger().warn("Measurement invalidated due to too old value")

        state_msg = State()

        if not self.__enabled:
            state_msg.state = State.MEASUREMENT_DISABLED
        elif self.__closure_sensor.is_open():
            state_msg.state = State.TANK_OPEN
        elif self.__level_sensor_value < 0:
            state_msg.state = State.UNKNOWN
        elif self.__level_sensor_value >= self.__max_level:
            state_msg.state = State.LEVEL_CRITICAL_HIGH
        elif self.__level_sensor_value <= self.__min_level:
            state_msg.state = State.LEVEL_CRITICAL_LOW
        else:
            state_msg.state = State.GOOD

        if state_msg.state != self.__previous_state or self.fst:

            if not self.fst:
                self.get_logger().info(
                    f"Tank's state changed from {self.__previous_state}"
                    f" ({state_str(self.__previous_state)}) to"
                    f" {state_msg.state} ({state_str(state_msg.state)})")

            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = self.__frame
            self.__state_publisher.publish(state_msg)
            self.__led.set_state(state_msg.state)
            self.__previous_state = state_msg.state

            self.fst = False

        if state_msg.state in [State.LEVEL_CRITICAL_HIGH, State.LEVEL_CRITICAL_LOW, State.GOOD]:
            level_msg = Measurement()
            level_msg.header.stamp = self.get_clock().now().to_msg()
            level_msg.header.frame_id = self.__frame
            level_msg.val = float(self.__level_sensor_value)
            self.__level_publisher.publish(level_msg)


def main(args=None):
    # GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    rclpy.init(args=args)
    level_observer = LevelObserver()
    rclpy.spin(level_observer)
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except:
        GPIO.cleanup()
