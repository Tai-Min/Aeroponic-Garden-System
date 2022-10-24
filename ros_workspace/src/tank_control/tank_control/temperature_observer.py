import time
import rclpy
from rclpy.node import Node, SetParametersResult, Parameter, Publisher
from tank_msgs.msg import Measurement, TemperatureState
from .submodules.state_to_string import temperature_state_str
from .submodules.hardware import read_ds18b20


class TemperatureObserver(Node):
    def __init__(self) -> None:
        super().__init__("temperature_observer")

        self.declare_parameter("water_tank_frame", "water_tank")
        self.declare_parameter("water_tank_sensor_addr", "")
        self.declare_parameter("min_water_temp", 0.0)
        self.declare_parameter("max_water_temp", 0.0)

        self.declare_parameter("nutri_tank_frame", "nutri_tank")
        self.declare_parameter("nutri_tank_sensor_addr", "")
        self.declare_parameter("min_nutri_temp", 0.0)
        self.declare_parameter("max_nutri_temp", 0.0)

        self.declare_parameter("max_recv_errors", 10)

        self.__min_water_temp = self.get_parameter(
            "min_water_temp").get_parameter_value().double_value
        self.get_logger().info(
            f"Minimum water temperature is: {self.__min_water_temp}°C")
        self.__max_water_temp = self.get_parameter(
            "max_water_temp").get_parameter_value().double_value
        self.get_logger().info(
            f"Maximum water temperature is: {self.__max_water_temp}°C")
        self.__min_nutri_temp = self.get_parameter(
            "min_nutri_temp").get_parameter_value().double_value
        self.get_logger().info(
            f"Minimum nutrition temperature is: {self.__min_nutri_temp}°C")
        self.__max_nutri_temp = self.get_parameter(
            "max_nutri_temp").get_parameter_value().double_value
        self.get_logger().info(
            f"Maximum nutrition temperature is: {self.__max_nutri_temp}°C")

        self.__water_tank_frame = self.get_parameter(
            "water_tank_frame").get_parameter_value().string_value
        self.get_logger().info(
            f"Water tank frame is: {self.__water_tank_frame}")
        self.__nutri_tank_frame = self.get_parameter(
            "nutri_tank_frame").get_parameter_value().string_value
        self.get_logger().info(
            f"Nutri tank frame is: {self.__nutri_tank_frame}")

        self.__water_sensor_addr = self.get_parameter(
            "water_tank_sensor_addr").get_parameter_value().string_value
        self.get_logger().info(
            f"Water tank sensor address is: {self.__water_sensor_addr}")
        self.__nutri_sensor_addr = self.get_parameter(
            "nutri_tank_sensor_addr").get_parameter_value().string_value
        self.get_logger().info(
            f"Nutri tank sensor address is: {self.__nutri_sensor_addr}")

        self.__max_recv_errors = self.get_parameter(
            "max_recv_errors").get_parameter_value().integer_value
        self.get_logger().info(
            f"Observer will fail at: {self.__max_recv_errors} errors in a row")

        self.__fst_water = True
        self.__fst_nutri = True
        self.__water_previous_state = TemperatureState.UNKNOWN
        self.__nutri_previous_state = TemperatureState.UNKNOWN
        self.__water_err_cntr = 0
        self.__nutri_err_cntr = 0

        self.__water_temp_publisher = self.create_publisher(
            Measurement, f"{self.__water_tank_frame}/liquid_temp", 10)
        self.__nutri_temp_publisher = self.create_publisher(
            Measurement, f"{self.__nutri_tank_frame}/liquid_temp", 10)
        self.__water_state_publisher = self.create_publisher(
            TemperatureState, f"{self.__water_tank_frame}/liquid_temp_state", 10)
        self.__nutri_state_publisher = self.create_publisher(
            TemperatureState, f"{self.__nutri_tank_frame}/liquid_temp_state", 10)
        self.__timer = self.create_timer(0.1, self.__on_timer_callback)

        self.add_on_set_parameters_callback(self.__on_set_parameters_callback)

        self.get_logger().info("Temperature observer is running")

    def __on_set_parameters_callback(self, params: list) -> SetParametersResult:
        success = True
        param: Parameter

        # Check if all passed params are valid.
        for param in params:
            if param.type != Parameter.Type.DOUBLE:
                success = False
                break
            if param.name not in ["min_water_temp", "max_water_temp",
                                  "min_nutri_temp", "max_nutri_temp"]:
                success = False
                break

        if success:
            for param in params:
                if param.name == "min_water_temp":
                    self.__min_water_temp = float(param.value)
                elif param.name == "max_water_temp":
                    self.__max_water_temp = float(param.value)
                elif param.name == "min_nutri_temp":
                    self.__min_nutri_temp = float(param.value)
                elif param.name == "max_nutri_temp":
                    self.__max_nutri_temp = float(param.value)

        return SetParametersResult(successful=success)

    def __measure_and_publish(self, fst: bool, sensor_addr: int, previous_state: TemperatureState,
                              min_temp: float, max_temp: float, frame: str,
                              state_publisher: Publisher, temp_publisher: Publisher, err_cntr: int) -> tuple:
        temp, is_error = read_ds18b20(sensor_addr)

        state_msg = TemperatureState()

        if is_error:
            state_msg.state = TemperatureState.UNKNOWN
        elif temp < min_temp:
            state_msg.state = TemperatureState.TOO_COLD
        elif temp > max_temp:
            state_msg.state = TemperatureState.TOO_HOT
        else:
            state_msg.state = TemperatureState.GOOD

        if state_msg.state != previous_state or fst:
            if not fst:
                self.get_logger().info(
                    f"Liquid state changed from {previous_state}"
                    f" ({temperature_state_str(previous_state)}) to"
                    f" {state_msg.state} ({temperature_state_str(state_msg.state)})")

            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = frame
            state_publisher.publish(state_msg)
            previous_state = state_msg.state

            fst = False

        if state_msg.state != TemperatureState.UNKNOWN:
            temp_msg = Measurement()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = frame
            temp_msg.val = float(temp)
            temp_publisher.publish(temp_msg)

        if is_error:
            err_cntr += 1
            self.get_logger().warn(
                f"Read error while reading DS18B20 sensor {sensor_addr}"
                f" increasing error counter of this sensor to {err_cntr} / {self.__max_recv_errors}")

            if err_cntr == self.__max_recv_errors:
                self.get_logger().error(
                    "Too many consecutive read errors, stopping node")
                self.destroy_node()
        else:
            err_cntr = 0

        return (fst, previous_state, err_cntr)

    def __on_timer_callback(self) -> None:
        self.__fst_water, self.__water_previous_state, self.__water_err_cntr = self.__measure_and_publish(
            self.__fst_water, self.__water_sensor_addr, self.__water_previous_state,
            self.__min_water_temp, self.__max_water_temp, self.__water_tank_frame,
            self.__water_state_publisher, self.__water_temp_publisher, self.__water_err_cntr)

        self.__fst_nutri, self.__nutri_previous_state, self.__nutri_err_cntr = self.__measure_and_publish(
            self.__fst_nutri, self.__nutri_sensor_addr, self.__nutri_previous_state,
            self.__min_nutri_temp, self.__max_nutri_temp, self.__nutri_tank_frame,
            self.__nutri_state_publisher, self.__nutri_temp_publisher, self.__nutri_err_cntr)


def main(args=None):
    rclpy.init(args=args)
    temperature_observer = TemperatureObserver()
    rclpy.spin(temperature_observer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
