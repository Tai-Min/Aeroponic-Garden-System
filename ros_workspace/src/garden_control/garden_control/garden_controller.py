import threading
import queue
import datetime
import rclpy
import time
from rclpy.node import Node
from tank_msgs.msg import LevelState, TemperatureState, QualityState
from tank_msgs.action import DrivePump
from garden_msgs.msg import Command, State
from rclpy.action import ActionClient


class GardenController(Node):
    def __init__(self) -> None:
        super().__init__("garden_controller")

        self.declare_parameter("queue_size", 1)
        self.declare_parameter("min_time_between_controls", 300)
        self.declare_parameter("desired_temperature", 30.0)
        self.declare_parameter("desired_pressure", 104.0)
        self.declare_parameter("desired_humidity", 80.0)
        self.declare_parameter("command_wait_time", 5)
        self.declare_parameter("water_pumping_time", 10)
        self.declare_parameter("nutri_pumping_time", 10)

        queue_size = self.get_parameter(
            "queue_size").get_parameter_value().integer_value
        self.__min_time_between_controls = self.get_parameter(
            "min_time_between_controls").get_parameter_value().integer_value

        self.__desired_temperature = self.get_parameter(
            "desired_temperature").get_parameter_value().double_value
        self.__desired_pressure = self.__desired_temperature = self.get_parameter(
            "desired_pressure").get_parameter_value().double_value
        self.__desired_humidity = self.__desired_temperature = self.get_parameter(
            "desired_humidity").get_parameter_value().double_value

        self.__command_wait_time = self.get_parameter(
            "command_wait_time").get_parameter_value().integer_value
        self.__water_pumping_time = self.get_parameter(
            "water_pumping_time").get_parameter_value().integer_value
        self.__nutri_pumping_time = self.get_parameter(
            "nutri_pumping_time").get_parameter_value().integer_value

        self.__field_controller_state = {}

        self.__state_sub = self.create_subscription(
            State, "state", self.__on_state_received_callback, 10)

        self.__cmd_publisher = self.create_publisher(Command, "command", 10)

        self.__water_state = LevelState.UNKNOWN
        self.__nutri_state = LevelState.UNKNOWN

        self.__water_state_sub = self.create_subscription(
            LevelState, "/water_tank/state", self.__on_water_state_received_callback, 10)
        self.__nutri_state_sub = self.create_subscription(
            LevelState, "/nutri_tank/state", self.__on_nutri_state_received_callback, 10)

        self.__water_temp_state = TemperatureState.UNKNOWN
        self.__nutri_temp_state = TemperatureState.UNKNOWN

        self.__water_temp_state_sub = self.create_subscription(
            TemperatureState, "/water_tank/liquid_temperature_state", self.__on_water_temp_state_received_callback, 10)
        self.__nutri_temp_state_sub = self.create_subscription(
            TemperatureState, "/nutri_tank/liquid_temperature_state", self.__on_nutri_temp_state_received_callback, 10)

        self.__nutri_quality_state = QualityState.UNKNOWN

        self.__nutri_quality_sub = self.create_subscription(
            QualityState, "/nutri_tank/quality", self.__on_nutri_qual_state_received_callback, 10)

        self.__drive_water_client = ActionClient(
            self, DrivePump, "/water_tank/drive")
        self.__drive_nutri_client = ActionClient(
            self, DrivePump, "/nutri_tank/drive")

        self.__queue = queue.Queue(queue_size)

        threading.Thread(target=self.__queue_worker, daemon=True)

    def __del__(self) -> None:
        self.__queue.join()

    def __on_state_received_callback(self, msg: State) -> None:
        device_id = msg.header.frame_id

        if device_id not in self.__field_controller_state:
            self.__field_controller_state[device_id] = {
                "temperature": -255,
                "pressure": -255,
                "humidity": -255,
                "classification": -255,
                "last_control_date": datetime.datetime.min
            }
            self.get_logger().info(
                f"Registered new Field Controller with ID {device_id}")

        if msg.type == State.TYPE_TEMPERATURE:
            self.__field_controller_state[device_id]["temperature"] = msg.value
        if msg.type == State.TYPE_PRESSURE:
            self.__field_controller_state[device_id]["pressure"] = msg.value
        if msg.type == State.TYPE_HUMIDITY:
            self.__field_controller_state[device_id]["humidity"] = msg.value
        if msg.type == State.TYPE_CLASSIFICATION:
            self.__field_controller_state[device_id]["classification"] = int(
                msg.value)

        diff = datetime.datetime.now()
        diff -= self.__field_controller_state[device_id]["last_control_date"]

        if (self.__water_state == LevelState.GOOD or self.__water_state == LevelState.LEVEL_CRITICAL_HIGH) and \
            (self.__nutri_state == LevelState.GOOD or self.__nutri_state == LevelState.LEVEL_CRITICAL_HIGH) and \
            self.__water_temp_state == TemperatureState.GOOD and self.__nutri_temp_state == TemperatureState.GOOD and \
            self.__nutri_quality_state == QualityState.GOOD and diff.total_seconds() > self.__min_time_between_controls and \
                self.__field_controller_state[device_id]["classification"] == 0 and \
                self.__field_controller_state[device_id]["temperature"] > -255 and \
                self.__field_controller_state[device_id]["pressure"] > -255 and \
                self.__field_controller_state[device_id]["humidity"] > -255:

            self.__queue.put(device_id)

    def __on_water_state_received_callback(self, msg: LevelState) -> None:
        self.__water_state = msg

    def __on_nutri_state_received_callback(self, msg: LevelState) -> None:
        self.__nutri_state = msg

    def __on_water_temp_state_received_callback(self, msg: TemperatureState) -> None:
        self.__water_temp_state = msg

    def __on_nutri_temp_state_received_callback(self, msg: TemperatureState) -> None:
        self.__nutri_temp_state = msg

    def __on_nutri_qual_state_received_callback(self, msg: QualityState) -> None:
        self.__nutri_quality_state = msg

    def __perform_control(self, device_id: str) -> None:
        self.get_logger().info(f"Performing control on {device_id}")

        temp_cmd = Command()
        temp_cmd.header.frame_id = device_id
        temp_cmd.header.stamp = self.get_clock().now().to_msg()
        temp_cmd.endpoint = Command.FAN_EP

        if self.__field_controller_state[device_id]["temperature"] <= self.__desired_temperature:
            temp_cmd.value = 0.0
            self.get_logger().info(f"Fan on {device_id} set to off")
        elif self.__field_controller_state[device_id]["temperature"] > self.__desired_temperature:
            temp_cmd.value = 1.0
            self.get_logger().info(f"Fan on {device_id} set to on")
        self.__cmd_publisher.publish(temp_cmd)

        hum_cmd = Command()
        hum_cmd.header.frame_id = device_id
        hum_cmd.header.stamp = self.get_clock().now().to_msg()

        if self.__field_controller_state[device_id]["humidity"] <= self.__desired_temperature:
            self.get_logger().info(f"Driving relays on {device_id}")

            # Open relays.
            hum_cmd.value = 1.0

            hum_cmd.endpoint = Command.WATER_EP
            self.__cmd_publisher.publish(hum_cmd)

            hum_cmd.endpoint = Command.NUTRI_EP
            self.__cmd_publisher.publish(hum_cmd)

            # Wait for relays.
            time.sleep(self.__command_wait_time)

            # Drive pumps.
            pump_msg = DrivePump.Goal()
            pump_msg.seconds = self.__water_pumping_time
            self.__drive_water_client.wait_for_server()
            self.__drive_water_client.send_goal_async(pump_msg)

            pump_msg.seconds = self.__nutri_pumping_time
            self.__drive_nutri_client.wait_for_server()
            self.__drive_nutri_client.send_goal_async(pump_msg)

            # Wait pumps to finish.
            time.sleep(
                max([self.__water_pumping_time, self.__nutri_pumping_time] + self.__command_wait_time))

            # Close relays.
            hum_cmd.header.stamp = self.get_clock().now().to_msg()
            hum_cmd.value = 0.0
            hum_cmd.endpoint = Command.WATER_EP
            self.__cmd_publisher.publish(hum_cmd)

            hum_cmd.endpoint = Command.NUTRI_EP
            self.__cmd_publisher.publish(hum_cmd)

        self.get_logger().info(
            f"Control of {device_id} done, next potential control in {self.__min_time_between_controls} seconds")

        self.__field_controller_state[device_id]["last_control_date"] = datetime.datetime.now(
        )

    def __queue_worker(self) -> None:
        while True:
            device_id = self.__queue.get()
            self.get_logger().info(f"Working on {device_id}")
            self.__perform_control(device_id)
            self.__queue.task_done()


def main(args=None):
    rclpy.init(args=args)
    controller = GardenController()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
