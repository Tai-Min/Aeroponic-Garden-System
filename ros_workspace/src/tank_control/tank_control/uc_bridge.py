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
        self.declare_parameter("max_recv_errors", 10)

        port = self.get_parameter("port").get_parameter_value().string_value
        self.__serial = serial.Serial(port, timeout=1)
        self.get_logger().info(f"uC serial port is: {port}")

        self.__water_tank_frame = self.get_parameter(
            "water_tank_frame").get_parameter_value().string_value
        self.get_logger().info(
            f"Water tank frame is: {self.__water_tank_frame}")
        self.__nutri_tank_frame = self.get_parameter(
            "nutri_tank_frame").get_parameter_value().string_value
        self.get_logger().info(
            f"Nutri tank frame is: {self.__nutri_tank_frame}")

        self.__processors = [self.__process_level_sensor,
                             self.__process_tds_sensor, self.__process_ph_sensor]
        self.__max_recv_errors = self.get_parameter(
            "max_recv_errors").get_parameter_value().integer_value
        self.get_logger().info(
            f"Bridge will fail at: {self.__max_recv_errors} errors in a row")

        self.__err_cntr = 0

        self.__msg_publishers = {}
        self.__msg_publishers["u0"] = self.create_publisher(
            Measurement, f"{self.__water_tank_frame}/level_raw", 10)
        self.__msg_publishers["u1"] = self.create_publisher(
            Measurement, f"{self.__nutri_tank_frame}/level_raw", 10)
        self.__msg_publishers["ph"] = self.create_publisher(
            Measurement, f"{self.__nutri_tank_frame}/ph_raw", 10)
        self.__msg_publishers["tds"] = self.create_publisher(
            Measurement, f"{self.__nutri_tank_frame}/tds_raw", 10)

        self.__timer = self.create_timer(0.1, self.__on_timer_callback)

        self.get_logger().info("uC bridge is running")

    def __on_timer_callback(self) -> None:
        """
        Process serial readings from attached sensor
        and publish them.
        """
        try:
            payload = self.__serial.readline().decode("utf-8")
            payload = payload.replace("\n", "")
        except:
            return

        if not payload:
            return

        self.get_logger().info(
            f"Processing {payload}", throttle_duration_sec=10)

        for processor in self.__processors:
            if processor(payload):
                self.__err_cntr = 0
                return

        self.__err_cntr += 1
        self.get_logger().warn(
            f"{payload} does not match any known sensor pattern,"
            f" increasing error counter to {self.__err_cntr} / {self.__max_recv_errors}")
        
        if self.__err_cntr >= self.__max_recv_errors:
            self.get_logger().error(
                "Too many consecutive invalid payloads, stopping node")
            self.destroy_node()

    def __process_level_sensor(self, payload: str) -> bool:
        """
        Try to process ultrasonic sensor payload and publish it.
        Returns false if given payload does not match
        sensor's payload pattern.
        Returns true on succesful payload parse, even when
        payload checksum is invalid.
        """
        pattern = "^u[0-1]:-?[0-9]{1,5}\*[0-9]{1,3}$"

        if not re.search(pattern, payload):
            return False

        key, payload = payload.split(":")
        value, checksum = payload.split("*")

        computed_checksum = self.__compute_checksum(value)
        if computed_checksum != int(checksum):
            self.get_logger().warn(
                f"Checksum of {key} payload does not match (received: {checksum}, computed: {computed_checksum})")
            return True

        value = float(value) / 1000  # To meters.

        self.get_logger().info(
            f"Processed {key} payload (value: {value}m)", throttle_duration_sec=10)

        msg = Measurement()
        msg.header.frame_id = self.__water_tank_frame if key == "u0" else self.__nutri_tank_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.val = value
        self.__msg_publishers[key].publish(msg)

        return True

    def __process_ph_sensor(self, payload: str) -> bool:
        """
        Try to process pH sensor payload and publish it.
        Returns false if given payload does not match
        sensor's payload pattern.
        Returns true on succesful payload parse, even when
        payload checksum is invalid.
        """
        pattern = "^ph:-?[0-9]{1,5}\*[0-9]{1,3}$"

        if not re.search(pattern, payload):
            return False

        payload = payload[3:]
        value, checksum = payload.split("*")

        computed_checksum = self.__compute_checksum(value)
        if computed_checksum != int(checksum):
            self.get_logger().warn(
                f"Checksum of pH payload does not match (received: {checksum}, computed: {computed_checksum})")
            return True

        value = float(value) / 1000.0  # To pH.

        self.get_logger().info(
            f"Processed pH payload (value: {value}pH)", throttle_duration_sec=10)

        msg = Measurement()
        msg.header.frame_id = self.__nutri_tank_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.val = float(value)
        self.__msg_publishers["ph"].publish(msg)

        return True

    def __process_tds_sensor(self, payload: str) -> bool:
        """
        Try to process TDS sensor payload and publish it.
        Returns false if given payload does not match
        sensor's payload pattern.
        Returns true on succesful payload parse, even when
        payload checksum is invalid.
        """
        pattern = "^tds:-?[0-9]{1,5}\*[0-9]{1,3}$"

        if not re.search(pattern, payload):
            return False

        payload = payload[4:]
        value, checksum = payload.split("*")

        computed_checksum = self.__compute_checksum(value)
        if computed_checksum != int(checksum):
            self.get_logger().warn(
                f"Checksum of TDS payload does not match (received: {checksum}, computed: {computed_checksum})")
            return True

        self.get_logger().info(
            f"Processed TDS payload (value: {value}ppm)", throttle_duration_sec=10)

        msg = Measurement()
        msg.header.frame_id = self.__nutri_tank_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.val = float(value)
        self.__msg_publishers["tds"].publish(msg)

        return True

    def __compute_checksum(self, val: str) -> int:
        """
        Compute checksum of given value string.
        For checksum computation a value
        between : and * (exclusively) should be used.
        """
        checksum = 0
        for c in val:
            checksum ^= ord(c)
        return checksum


def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
