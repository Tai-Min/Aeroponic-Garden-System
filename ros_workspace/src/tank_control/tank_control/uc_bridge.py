import re
import serial
import rclpy
from rclpy.node import Node
from tank_msgs.msg import Measurement


class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("uc_bridge")

        self.declare_parameter("water_tank_frame", "water_tank")
        self.declare_parameter("nutri_tank_frame", "nutri_tank")

        self.declare_parameter("port", "/dev/ttyUSB0")

        self.serial = serial.Serial(self.get_parameter(
            "port").get_parameter_value().string_value, timeout=1)

        self.water_tank_frame = self.frame = self.get_parameter(
            "water_tank_frame").get_parameter_value().string_value
        self.nutri_tank_frame = self.frame = self.get_parameter(
            "nutri_tank_frame").get_parameter_value().string_value

        self.processors = [self.process_level_sensor,
                           self.process_ec_sensor, self.process_ph_sensor]

        self.msg_publishers = {}
        self.msg_publishers["u0"] = self.create_publisher(
            Measurement, "water_tank/level_raw", 10)
        self.msg_publishers["u1"] = self.create_publisher(
            Measurement, "nutri_tank/level_raw", 10)
        self.msg_publishers["ph"] = self.create_publisher(
            Measurement, "nutri_tank/ph", 10)
        self.msg_publishers["ec"] = self.create_publisher(
            Measurement, "nutri_tank/ec", 10)

        self.timer = self.create_timer(0, self.timer_callback)

    def timer_callback(self) -> None:
        """
        Process serial readings from attached sensor
        and publish them.
        """
        payload = self.serial.readline().decode("utf-8")
        payload = payload.replace("\n", "")

        self.get_logger().info(
            f"Processing {payload}", throttle_duration_sec=10)

        for processor in self.processors:
            if processor(payload):
                return

        self.get_logger().warn(
            f"{payload} does not match any known sensor pattern")

    def process_level_sensor(self, payload: str) -> bool:
        """
        Try to process ultrasonic sensor payload and publish it.
        Returns false if given payload does not match
        sensor's payload pattern.
        Returns true on succesful payload parse, even when
        payload checksum is invalid.
        """
        pattern = "^u[0-1]:[0-9]{1,5}\*[0-9]{1,3}$"

        if not re.search(pattern, payload):
            return False

        key, payload = payload.split(":")
        value, checksum = payload.split("*")

        computed_checksum = self.compute_checksum(value)
        if computed_checksum != int(checksum):
            self.get_logger().warn(
                f"Checksum of {key} payload does not match (received: {checksum}, computed: {computed_checksum})")
            return True

        value = float(value) / 1000  # To meters.

        self.get_logger().info(
            f"Processed {key} payload", throttle_duration_sec=10)

        msg = Measurement()
        msg.header.frame_id = self.water_tank_frame if key == "u0" else self.nutri_tank_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.val = value
        self.msg_publishers[key].publish(msg)

        return True

    def process_ph_sensor(self, payload: str) -> bool:
        """
        Try to process pH sensor payload and publish it.
        Returns false if given payload does not match
        sensor's payload pattern.
        Returns true on succesful payload parse, even when
        payload checksum is invalid.
        """
        pattern = "^ph:[0-9]{1,5}\*[0-9]{1,3}$"

        if not re.search(pattern, payload):
            return False

        payload = payload[3:]
        value, checksum = payload.split("*")

        computed_checksum = self.compute_checksum(value)
        if computed_checksum != int(checksum):
            self.get_logger().warn(
                f"Checksum of pH payload does not match (received: {checksum}, computed: {computed_checksum})")
            return True

        self.get_logger().info("Processed pH payload", throttle_duration_sec=10)

        msg = Measurement()
        msg.header.frame_id = self.nutri_tank_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.val = 2137
        self.msg_publishers["ph"].publish(msg)

        return True

    def process_ec_sensor(self, payload: str) -> bool:
        """
        Try to process EC sensor payload and publish it.
        Returns false if given payload does not match
        sensor's payload pattern.
        Returns true on succesful payload parse, even when
        payload checksum is invalid.
        """
        pattern = "^ec:[0-9]{1,5}\*[0-9]{1,3}$"

        if not re.search(pattern, payload):
            return False

        payload = payload[3:]
        value, checksum = payload.split("*")

        computed_checksum = self.compute_checksum(value)
        if computed_checksum != int(checksum):
            self.get_logger().warn(
                f"Checksum of EC payload does not match (received: {checksum}, computed: {computed_checksum})")
            return True

        self.get_logger().info("Processed EC payload", throttle_duration_sec=10)

        msg = Measurement()
        msg.header.frame_id = self.nutri_tank_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.val = 2137
        self.msg_publishers["ec"].publish(msg)

        return True

    def compute_checksum(self, val: str) -> int:
        """
        Compute checksum of given value string.
        For checksum computation a value
        between : and * (exclusively) should be used.
        """
        checksum = 0
        for c in val:
            checksum ^= (int(c) + 48)
        return checksum


def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
