import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from tank_msgs.action import DrivePump
import RPi.GPIO as GPIO


class PumpDriver(Node):
    def __init__(self) -> None:
        super().__init__("pump_driver")

        self.declare_parameter("pump_pin", 17)
        self.pump = self.get_parameter(
            "pump_pin").get_parameter_value().integer_value
        GPIO.setup(self.pump, GPIO.OUT)

        self.server = ActionServer(
            self, DrivePump, "drive", self.drive_pump
        )
        self.get_logger().info("Pump driver is running")

    def drive_pump(self, handle) -> DrivePump.Result:
        self.get_logger().info(f"Driving pump for {handle.request.seconds}")

        feedback_msg = DrivePump.Feedback()

        GPIO.output(self.pump, GPIO.HIGH)

        for i in range(handle.request.seconds):
            feedback_msg.fulfillment = int(i / handle.request.seconds * 100)
            handle.publish_feedback(feedback_msg)
            time.sleep(1)

        GPIO.output(self.pump, GPIO.LOW)

        handle.succeed()

        self.get_logger().info(f"Driving pump finished")

        result = DrivePump.Result()
        result.res = True
        return result


def main(args=None):
    GPIO.setwarnings(False)
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
