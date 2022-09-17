import time
from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from tank_msgs.action import DrivePump
from tank_msgs.msg import State
import RPi.GPIO as GPIO
from .submodules.common import state_str


class PumpDriver(Node):
    def __init__(self) -> None:
        super().__init__("pump_driver")

        self.__var_lock = Lock()
        self.__tank_state = State.UNKNOWN

        self.declare_parameter("pump_pin", 3)
        self.declare_parameter("tank_state_topic", "state")

        self.__pump = self.get_parameter(
            "pump_pin").get_parameter_value().integer_value
        self.get_logger().info(f"Pump GPIO (BCM) is: {self.__pump}")
        GPIO.setup(self.__pump, GPIO.OUT)

        tank_state_topic = self.get_parameter(
            "tank_state_topic").get_parameter_value().string_value
        self.get_logger().info(f"Listens to state from: {tank_state_topic}")
        self.__level_sensor_subscriber = self.create_subscription(
            State, tank_state_topic, self.__on_state_received_callback, 10)
        self.__server = ActionServer(
            self, DrivePump, "drive", self.__on_drive_callback
        )
        self.get_logger().info("Pump driver is running")

    def __del__(self) -> None:
        GPIO.output(self.__pump, GPIO.LOW)

    def __on_state_received_callback(self, state_msg: State) -> None:
        with self.__var_lock:
            self.__tank_state = state_msg.state

    def __on_drive_callback(self, handle: ServerGoalHandle) -> DrivePump.Result:
        self.get_logger().info(f"Driving pump for {handle.request.seconds}")

        result = DrivePump.Result()
        feedback_msg = DrivePump.Feedback()

        GPIO.output(self.__pump, GPIO.HIGH)

        for i in range(handle.request.seconds):
            with self.__var_lock:
                if self.__tank_state not in [State.GOOD, State.LEVEL_CRITICAL_HIGH]:
                    GPIO.output(self.__pump, GPIO.LOW)
                    handle.succeed()
                    result.res = False
                    self.get_logger().info(
                        "Stopped driving the pump due to invalid state of the tank "
                        f"(required either GOOD or LEVEL_CRITICAL_HIGH, received {state_str(self.__tank_state)})")
                    return result

            feedback_msg.fulfillment = int(i / handle.request.seconds * 100)
            handle.publish_feedback(feedback_msg)
            time.sleep(1)

        GPIO.output(self.__pump, GPIO.LOW)

        handle.succeed()

        self.get_logger().info("Driving pump finished")

        result.res = True
        return result


def main(args=None):
    # GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    rclpy.init(args=args)
    level_observer = PumpDriver()
    rclpy.spin(level_observer)
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except:
        GPIO.cleanup()
