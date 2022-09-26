import time
import rclpy
from rclpy.node import Node, SetParametersResult, Parameter
from tank_msgs.msg import QualityState, Measurement
import RPi.GPIO as GPIO
from .submodules.common import quality_state_str, QualityLED


class QualityObserver(Node):
    def __init__(self) -> None:
        super().__init__("quality_observer")

        self.declare_parameter("frame_id", "tank")

        self.declare_parameter("desired_ppm", 500.0)
        self.declare_parameter("min_ppm", 0.0)
        self.declare_parameter("max_ppm", 1000.0)

        self.declare_parameter("desired_ph", 7.5)
        self.declare_parameter("min_ppm", 0.0)
        self.declare_parameter("max_ppm", 14.0)

        self.declare_parameter("led_pin", 22)

        self.__frame = self.get_parameter(
            "frame_id").get_parameter_value().string_value
        self.get_logger().info(f"Frame ID is: {self.__frame}")

        self.__check_period = self.get_parameter(
            "check_period").get_parameter_value().double_value
        self.get_logger().info(
            f"Quality will be checked once per: {self.__check_period}s")

        self.__desired_ppm = self.get_parameter(
            "desired_ppm").get_parameter_value().double_value
        self.get_logger().info(f"Desired PPM is: {self.__desired_ppm}ppm")
        self.__min_ppm = self.get_parameter(
            "min_ppm").get_parameter_value().double_value
        self.get_logger().info(f"Minimum PPM is: {self.__min_ppm}ppm")
        self.__max_ppm = self.get_parameter(
            "max_ppm").get_parameter_value().double_value
        self.get_logger().info(f"Maximum PPM is: {self.__max_ppm}ppm")
        self.__tds_invalidate_time = self.get_parameter(
            "ppm_value_invalidate_time").get_parameter_value().double_value
        self.get_logger().info(
            f"TDS sensor reading will be invalidated after: {self.__tds_invalidate_time}s")

        self.__desired_ph = self.get_parameter(
            "desired_ph").get_parameter_value().double_value
        self.get_logger().info(f"Desired pH is: {self.__desired_ph}pH")
        self.__min_ph = self.get_parameter(
            "min_ph").get_parameter_value().double_value
        self.get_logger().info(f"Minimum pH is: {self.__min_ph}pH")
        self.__max_ph = self.get_parameter(
            "max_ph").get_parameter_value().double_value
        self.get_logger().info(f"Maximum pH is: {self.__max_ph}pH")
        self.__ph_invalidate_time = self.get_parameter(
            "ph_value_invalidate_time").get_parameter_value().double_value
        self.get_logger().info(
            f"pH sensor reading will be invalidated after: {self.__ph_invalidate_time}s")

        self.__fst = True
        self.__previous_state = QualityState.UNKNOWN
        self.__tds_sensor_value = -1
        self.__last_tds_value_stamp = -1
        self.__ph_sensor_value = -1
        self.__last_ph_value_stamp = -1

        led_pin = self.get_parameter(
            "led_pin").get_parameter_value().integer_value
        self.get_logger().info(
            f"LED GPIO (BCM) is: {led_pin}")
        self.__led = QualityLED(led_pin)

        self.__tds_sensor_subscriber = self.create_subscription(
            Measurement, "tds_raw", self.__on_tds_received_callback, 10)
        self.__ph_sensor_subscriber = self.create_subscription(
            Measurement, "ph_raw", self.__on_ph_received_callback, 10)
        self.__tds_publisher = self.create_publisher(
            Measurement, "tds", 10)
        self.__ph_publisher = self.create_publisher(
            Measurement, "ph", 10)
        self.__quality_publisher = self.create_publisher(
            QualityState, "quality", 10)
        self.__timer = self.create_timer(
            self.__check_period, self.__on_timer_callback)

        self.add_on_set_parameters_callback(self.__on_set_parameters_callback)

        self.get_logger().info("Quality observer is running")

    def __on_set_parameters_callback(self, params: list) -> SetParametersResult:
        success = True
        param: Parameter

        # Check if all passed params are valid.
        for param in params:
            if param.type != Parameter.Type.DOUBLE:
                success = False
                break
            if param.name not in ["desired_ppm", "min_ppm",
                                  "max_ppm", "desired_ph",
                                  "min_ph", "max_ph"]:
                success = False
                break

        if success:
            for param in params:
                if param.name == "desired_ppm":
                    self.__desired_ppm = float(param.value)
                elif param.name == "min_ppm":
                    self.__min_ppm = float(param.value)
                elif param.name == "max_ppm":
                    self.__max_ppm = float(param.value)
                elif param.name == "desired_ph":
                    self.__desired_ph = float(param.value)
                elif param.name == "min_ph":
                    self.__min_ph = float(param.value)
                elif param.name == "max_ph":
                    self.__max_ph = float(param.value)

        return SetParametersResult(successful=success)

    def __on_tds_received_callback(self, msg: Measurement) -> None:
        self.__tds_sensor_value = msg.val
        self.__last_tds_value_stamp = time.time()

    def __on_ph_received_callback(self, msg: Measurement) -> None:
        self.__ph_sensor_value = msg.val
        self.__last_ph_value_stamp = time.time()

    def __on_timer_callback(self) -> None:
        if self.__last_tds_value_stamp >= 0 and (time.time() - self.__last_tds_value_stamp) > self.__tds_invalidate_time:
            self.__tds_sensor_value = -1
            self.__last_tds_value_stamp = -1
            self.get_logger().warn("TDS measurement invalidated due to too old value")

        if self.__last_ph_value_stamp >= 0 and (time.time() - self.__last_ph_value_stamp) > self.__ph_invalidate_time:
            self.__ph_sensor_value = -1
            self.__last_ph_value_stamp = -1
            self.get_logger().warn("pH measurement invalidated due to too old value")

        state_msg = QualityState()

        if self.__tds_sensor_value < 0 or self.__ph_sensor_value < 0:
            state_msg.state = QualityState.UNKNOWN
        elif (self.__tds_sensor_value > self.__max_ppm or self.__tds_sensor_value < self.__min_ppm) and \
                (self.__ph_sensor_value > self.__max_ph or self.__ph_sensor_value < self.__min_ph):
            state_msg.state = QualityState.BAD_ALL
        elif (self.__tds_sensor_value > self.__max_ppm or self.__tds_sensor_value < self.__min_ppm):
            state_msg.state = QualityState.BAD_TDS
        elif (self.__ph_sensor_value > self.__max_ph or self.__ph_sensor_value < self.__min_ph):
            state_msg.state = QualityState.BAD_PH
        else:
            state_msg.state = QualityState.GOOD

        if state_msg.state != self.__previous_state or self.__fst:
            if not self.__fst:
                self.get_logger().info(
                    f"Liquid state changed from {self.__previous_state}"
                    f" ({quality_state_str(self.__previous_state)}) to"
                    f" {state_msg.state} ({quality_state_str(state_msg.state)})")

            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = self.__frame
            self.__quality_publisher.publish(state_msg)
            self.__led.set_state(state_msg.state)
            self.__previous_state = state_msg.state
            self.__fst = False

            if state_msg.state != QualityState.UNKNOWN:
                ph_msg = Measurement()
                ph_msg.header.stamp = self.get_clock().now().to_msg()
                ph_msg.header.frame_id = self.__frame
                ph_msg.val = float(self.__ph_sensor_value)

                tds_msg = Measurement()
                tds_msg.header.stamp = self.get_clock().now().to_msg()
                tds_msg.header.frame_id = self.__frame
                tds_msg.val = float(self.__tds_sensor_value)

                self.__ph_publisher.publish(ph_msg)
                self.__tds_publisher.publish(tds_msg)


def main(args=None):
    GPIO.setmode(GPIO.BCM)

    rclpy.init(args=args)
    quality_observer = QualityObserver()
    rclpy.spin(quality_observer)
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except:
        GPIO.cleanup()
