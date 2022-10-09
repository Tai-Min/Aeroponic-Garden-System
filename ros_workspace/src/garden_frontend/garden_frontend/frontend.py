import re
import ast
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from tank_msgs.msg import Measurement, LevelState, TemperatureState, PumpState, QualityState


class Frontend(Node):
    def __init__(self) -> None:
        super().__init__("frontend")

        self.__overseer_id = 1
        self.__is_realtime = True
        self.__can_edit_config = True
        self.__publish_interval = 30.0

        self.__mqtt_topic_prefix = "FO/" + str(self.__overseer_id) + "/"

        # Water tank.
        self.__water_tank_level_sub = self.create_subscription(
            Measurement, "water_tank/level", self.__water_tank_level_cb, 10)
        self.__water_tank_level = Measurement()

        self.__water_tank_state_sub = self.create_subscription(
            LevelState, "water_tank/state", self.__water_tank_state_cb, 10)
        self.__water_tank_state = LevelState()

        self.__water_tank_liquid_temp_sub = self.create_subscription(
            Measurement, "water_tank/liquid_temperature", self.__water_tank_liquid_temp_cb, 10)
        self.__water_tank_liquid_temp = Measurement()

        self.__water_tank_liquid_temp_state_sub = self.create_subscription(
            TemperatureState, "water_tank/liquid_temperature_state", self.__water_tank_liquid_temp_state_cb, 10)
        self.__water_tank_liquid_temp_state = TemperatureState()

        self.__water_tank_pump_state_sub = self.create_subscription(
            PumpState, "water_tank/pump_state", self.__water_tank_pump_state_cb, 10)
        self.__water_tank_pump_state = PumpState()

        # Nutri tank.
        self.__nutri_tank_level_sub = self.create_subscription(
            Measurement, "nutri_tank/level", self.__nutri_tank_level_cb, 10)
        self.__nutri_tank_level = Measurement()

        self.__nutri_tank_state_sub = self.create_subscription(
            LevelState, "nutri_tank/state", self.__nutri_tank_state_cb, 10)
        self.__nutri_tank_state = LevelState()

        self.__nutri_tank_liquid_temp_sub = self.create_subscription(
            Measurement, "nutri_tank/liquid_temperature", self.__nutri_tank_liquid_temp_cb, 10)
        self.__nutri_tank_liquid_temp = Measurement()

        self.__nutri_tank_liquid_temp_state_sub = self.create_subscription(
            TemperatureState, "nutri_tank/liquid_temperature_state", self.__nutri_tank_liquid_temp_state_cb, 10)
        self.__nutri_tank_liquid_temp_state = TemperatureState()

        self.__nutri_tank_pump_state_sub = self.create_subscription(
            PumpState, "nutri_tank/pump_state", self.__nutri_tank_pump_state_cb, 10)
        self.__nutri_tank_pump_state = PumpState()

        self.__nutri_tank_tds_sub = self.create_subscription(
            Measurement, "nutri_tank/tds", self.__nutri_tank_tds_cb, 10)
        self.__nutri_tank_tds = Measurement()

        self.__nutri_tank_ph_sub = self.create_subscription(
            Measurement, "nutri_tank/ph", self.__nutri_tank_ph_cb, 10)
        self.__nutri_tank_ph = Measurement()

        self.__nutri_tank_quality_sub = self.create_subscription(
            QualityState, "nutri_tank/quality", self.__nutri_tank_quality_cb, 10)
        self.__nutri_tank_quality = QualityState()

        if not self.__is_realtime:
            self.__timer = self.create_timer(
                self.__publish_interval, self.__non_rt_cb)

        self.__mqtt_client = mqtt.Client()
        self.__mqtt_client.on_connect = self.__on_mqtt_connect
        # self.__mqtt_client.on_message = self.__on_mqtt_msg
        # self.__mqtt_client.connect("localhost")
        self.__mqtt_client.loop_start()

    def __del__(self) -> None:
        self.__mqtt_client.loop_stop()

    def __on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        self.get_logger().info("Connected to MQTT broker")

        if self.__can_edit_config:
            self.__mqtt_client.subscribe(
                self.__mqtt_topic_prefix + "config/#")

    def __get_measurement_dict(self, name: str, msg: Measurement, payload: dict) -> None:
        payload[name] = {
            "value": msg.val,
            "timestamp": msg.header.stamp
        }
        return payload

    def __get_level_state_dict(self, name: str, msg: LevelState, payload: dict) -> None:
        pass

    def __get_temperature_state_dict(self, name: str, msg: TemperatureState, payload: dict) -> None:
        pass

    def __get_pump_state_dict(self, name: str, msg: PumpState, payload: dict) -> None:
        pass

    def __get_quality_state_dict(self, name: str, msg: QualityState, payload: dict) -> None:
        pass

    def __water_tank_level_cb(self, msg: Measurement) -> None:
        self.__water_tank_level = msg
        if self.__is_realtime:
            payload = self.__get_measurement_dict(msg, {})
            self.__mqtt_client.publish(
                self.__mqtt_topic_prefix + "water_tank/level", payload)

    def __water_tank_state_cb(self, msg: LevelState) -> None:
        self.__water_tank_state = msg
        if self.__is_realtime:
            payload = self.__get_level_state_dict(msg)
            self.__mqtt_client.publish(
                self.__mqtt_topic_prefix + "water_tank/state", payload)

    def __non_rt_cb(self) -> None:
        payload = {}
        self.__get_measurement_dict("", self.__water_tank_level)
        self.__get_level_state_dict("", self.__water_tank_state)
        self.__get_measurement_dict("", self.__water_tank_liquid_temp)
        self.__get_temperature_state_dict(
            "", self.__water_tank_liquid_temp_state)
        self.__get_pump_state_dict("", self.__water_tank_pump_state)

        self.__get_measurement_dict("", self.__nutri_tank_level)
        self.__get_level_state_dict("", self.__nutri_tank_state)
        self.__get_measurement_dict("", self.__nutri_tank_liquid_temp)
        self.__get_temperature_state_dict(
            "", self.__nutri_tank_liquid_temp_state)
        self.__get_pump_state_dict("", self.__nutri_tank_pump_state)
        self.__get_measurement_dict("", self.__nutri_tank_tds)
        self.__get_measurement_dict("", self.__nutri_tank_ph)
        self.__get_quality_state_dict("", self.__nutri_tank_quality)


def main(args=None):
    rclpy.init(args=args)
    frontend = Frontend()
    rclpy.spin(frontend)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
