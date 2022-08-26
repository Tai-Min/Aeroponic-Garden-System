import rclpy
from rclpy.node import Node
from tank_msgs.msg import State, Measurement
from tank_msgs.srv import SetEnabled
import RPi.GPIO as GPIO


class ClosureSensor:
    def __init__(self, pin: int, callback=None, edge=GPIO.FALLING) -> None:
        self.__pin = pin
        GPIO.setup(self.__pin, GPIO.IN)
        if callback:
            GPIO.add_event_detect(self.__pin, edge, lambda: callback(True))

    def is_open(self) -> bool:
        return GPIO.input(self.__pin)


class Button(ClosureSensor):
    def is_pressed(self) -> bool:
        return not self.is_open()


class LED:
    def __init__(self, pin: int) -> None:
        GPIO.setup(pin, GPIO.OUT)
        self.__pwm = GPIO.PWM(pin, 3)
        self.__pwm.start(0)

    def set_state(self, state: int) -> None:
        if state == State.GOOD:
            self.__pwm.ChangeDutyCycle(0)
        elif state in [State.UNKNOWN, State.LEVEL_CRITICAL_HIGH, State.LEVEL_CRITICAL_LOW]:
            self.__pwm.ChangeDutyCycle(50)
        elif state in [State.TANK_OPEN, State.MEASUREMENT_DISABLED]:
            self.__pwm.ChangeDutyCycle(100)


class LevelObserver(Node):
    def __init__(self) -> None:
        super().__init__("level_observer")

        self.declare_parameter("frame_id", "tank")
        self.declare_parameter("min_level", 0.0)
        self.declare_parameter("max_level", 0.2)
        self.declare_parameter("check_period", 1.0)

        self.declare_parameter("closure_sensor_pin", 27)
        self.declare_parameter("led_pin", 17)
        self.declare_parameter("button_pin", 18)

        self.__frame = self.get_parameter(
            "frame_id").get_parameter_value().string_value
        self.__min_level = self.get_parameter(
            "min_level").get_parameter_value().double_value
        self.__max_level = self.get_parameter(
            "max_level").get_parameter_value().double_value
        self.__check_period = self.get_parameter(
            "check_period").get_parameter_value().double_value

        self.__enabled = True
        self.__previous_state = State.UNKNOWN
        self.__level_sensor_value = -1

        self.__closure_sensor = ClosureSensor(self.get_parameter(
            "closure_sensor_pin").get_parameter_value().integer_value, self.__on_timer_callback)
        self.__disable_button = Button(self.get_parameter(
            "button_pin").get_parameter_value().integer_value, self.__on_btn_pressed_callback)
        self.__led = LED(self.get_parameter(
            "led_pin").get_parameter_value().integer_value)

        self.__level_sensor_subscriber = self.create_subscription(
            Measurement, "/level_raw", self.__on_level_received_callback, 10)
        self.__level_publisher = self.create_publisher(
            Measurement, "level", 10)
        self.__state_publisher = self.create_publisher(State, "state", 10)
        self.__control_service = self.create_service(
            SetEnabled, "set_enabled", self.__on_set_enable_callback)
        self.__timer = self.create_timer(
            self.__check_period, self.__on_timer_callback)

    def __state_str(self, state: int) -> str:
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

    def __on_level_received_callback(self, msg: Measurement) -> None:
        self.__level_sensor_value = msg.val

    def __on_btn_pressed_callback(self) -> None:
        self.__enabled = not self.__enabled

    def __on_set_enable_callback(self, request: SetEnabled.Request, response: SetEnabled.Request) -> None:

        if request.val:
            self.get_logger().info("Measurement enabled via service")
        else:
            self.get_logger().info("Measurement disabled via service")

        self.__enabled = request.val
        return response

    def __on_timer_callback(self, btn_call: bool = False) -> None:
        if btn_call:
            self.get_logger().info("Callback called by button press")

        state_msg = State()

        if self.__level_sensor_value < 0:
            state_msg.state = State.UNKNOWN
        elif self.__closure_sensor.is_open():
            state_msg.state = State.TANK_OPEN
        elif not self.__enabled or not self.__disable_button.is_pressed():
            state_msg.state = State.MEASUREMENT_DISABLED
        elif self.__level_sensor_value >= self.__max_level:
            state_msg.state = State.LEVEL_CRITICAL_HIGH
        elif self.__level_sensor_value <= self.__min_level:
            state_msg.state = State.LEVEL_CRITICAL_LOW
        else:
            state_msg.state = State.GOOD

        if state_msg.state != self.__previous_state:
            self.get_logger().info(
                f"Tank's state changed from {self.__previous_state}"
                f" ({self.__state_str(self.__previous_state)}) to"
                f" {state_msg.state} ({self.__state_str(state_msg.state)})")

            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = self.__frame
            self.__state_publisher.publish(state_msg)
            self.__led.set_state(state_msg.state)
            self.__previous_state = state_msg.state

        if state_msg.state in [State.LEVEL_CRITICAL_HIGH, State.LEVEL_CRITICAL_LOW, State.GOOD]:
            level_msg = Measurement()
            level_msg.header.stamp = self.get_clock().now().to_msg()
            level_msg.header.frame_id = self.__frame
            level_msg.val = float(self.__level_sensor_value)
            self.__level_publisher.publish(level_msg)


def main(args=None):
    GPIO.setwarnings(False)
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
