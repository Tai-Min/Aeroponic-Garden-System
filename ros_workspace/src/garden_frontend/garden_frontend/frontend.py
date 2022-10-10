import re
import ast
from typing import Any
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from tank_msgs.msg import Measurement, LevelState, TemperatureState, PumpState, QualityState
from tank_control.submodules import state_to_string as s2s


class Frontend(Node):
    def __init__(self) -> None:
        super().__init__("frontend")

        self.declare_parameter("overseer_id", 1)
        self.declare_parameter("is_realtime", False)
        self.declare_parameter("can_edit_config", False)
        self.declare_parameter("publish_interval", 30.0)

        self.__overseer_id = self.get_parameter(
            "overseer_id").get_parameter_value().integer_value
        self.__is_realtime = self.get_parameter(
            "is_realtime").get_parameter_value().bool_value
        self.__can_edit_config = self.get_parameter(
            "can_edit_config").get_parameter_value().bool_value
        self.__publish_interval = self.get_parameter(
            "publish_interval").get_parameter_value().double_value

        self.__mqtt_topic_prefix = "FO/" + str(self.__overseer_id) + "/"
        self.__mqtt_config_topic = self.__mqtt_topic_prefix + "config/#"
        self.__mqtt_telemetry_topic = self.__mqtt_topic_prefix + "telemetry"

        # Water tank.
        self.__water_tank_level_topic = "water_tank/level"
        self.__water_tank_level = Measurement()
        self.__water_tank_level_sub = self.create_subscription(
            Measurement, self.__water_tank_level_topic,
            lambda msg: self.__publish_if_rt(
                self.__water_tank_level_topic, self.__water_tank_level, msg),
            10)

        self.__water_tank_state_topic = "water_tank/state"
        self.__water_tank_state = LevelState()
        self.__water_tank_state_sub = self.create_subscription(
            LevelState, self.__water_tank_state_topic,
            lambda msg: self.__publish_if_rt(
                self.__water_tank_state_topic, self.__water_tank_state, msg),
            10)

        self.__water_tank_liquid_temp_topic = "water_tank/liquid_temperature"
        self.__water_tank_liquid_temp = Measurement()
        self.__water_tank_liquid_temp_sub = self.create_subscription(
            Measurement, self.__water_tank_liquid_temp_topic,
            lambda msg: self.__publish_if_rt(
                self.__water_tank_liquid_temp_topic, self.__water_tank_liquid_temp, msg),
            10)

        self.__water_tank_liquid_temp_state_topic = "water_tank/liquid_temperature_state"
        self.__water_tank_liquid_temp_state = TemperatureState()
        self.__water_tank_liquid_temp_state_sub = self.create_subscription(
            TemperatureState, self.__water_tank_liquid_temp_state_topic,
            lambda msg: self.__publish_if_rt(
                self.__water_tank_liquid_temp_state_topic, self.__water_tank_liquid_temp_state, msg),
            10)

        self.__water_tank_pump_state_topic = "water_tank/pump_state"
        self.__water_tank_pump_state = PumpState()
        self.__water_tank_pump_state_sub = self.create_subscription(
            PumpState, self.__water_tank_pump_state_topic,
            lambda msg: self.__publish_if_rt(
                self.__water_tank_pump_state_topic, self.__water_tank_pump_state, msg),
            10)

        # Nutri tank.
        self.__nutri_tank_level_topic = "nutri_tank/level"
        self.__nutri_tank_level = Measurement()
        self.__nutri_tank_level_sub = self.create_subscription(
            Measurement, self.__nutri_tank_level_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_level_topic, self.__nutri_tank_level, msg),
            10)

        self.__nutri_tank_state_topic = "nutri_tank/state"
        self.__nutri_tank_state = LevelState()
        self.__nutri_tank_state_sub = self.create_subscription(
            LevelState, self.__nutri_tank_state_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_state_topic, self.__nutri_tank_state, msg),
            10)

        self.__nutri_tank_liquid_temp_topic = "nutri_tank/liquid_temperature"
        self.__nutri_tank_liquid_temp = Measurement()
        self.__nutri_tank_liquid_temp_sub = self.create_subscription(
            Measurement, self.__nutri_tank_liquid_temp_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_liquid_temp_topic, self.__nutri_tank_liquid_temp, msg),
            10)

        self.__nutri_tank_liquid_temp_state_topic = "nutri_tank/liquid_temperature_state"
        self.__nutri_tank_liquid_temp_state = TemperatureState()
        self.__nutri_tank_liquid_temp_state_sub = self.create_subscription(
            TemperatureState, self.__nutri_tank_liquid_temp_state_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_liquid_temp_state_topic, self.__nutri_tank_liquid_temp_state, msg),
            10)

        self.__nutri_tank_pump_state_topic = "nutri_tank/pump_state"
        self.__nutri_tank_pump_state = PumpState()
        self.__nutri_tank_pump_state_sub = self.create_subscription(
            PumpState, self.__nutri_tank_pump_state_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_pump_state_topic, self.__nutri_tank_pump_state, msg),
            10)

        self.__nutri_tank_tds_topic = "nutri_tank/tds"
        self.__nutri_tank_tds = Measurement()
        self.__nutri_tank_tds_sub = self.create_subscription(
            Measurement, self.__nutri_tank_tds_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_tds_topic, self.__nutri_tank_tds, msg),
            10)

        self.__nutri_tank_ph_topic = "nutri_tank/ph"
        self.__nutri_tank_ph = Measurement()
        self.__nutri_tank_ph_sub = self.create_subscription(
            Measurement, self.__nutri_tank_ph_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_ph_topic, self.__nutri_tank_ph, msg),
            10)

        self.__nutri_tank_quality_topic = "nutri_tank/quality"
        self.__nutri_tank_quality = QualityState()
        self.__nutri_tank_quality_sub = self.create_subscription(
            QualityState, self.__nutri_tank_quality_topic,
            lambda msg: self.__publish_if_rt(
                self.__nutri_tank_quality_topic, self.__nutri_tank_quality, msg),
            10)

        if not self.__is_realtime:
            self.__timer = self.create_timer(
                self.__publish_interval, self.__non_rt_cb)

        self.__mqtt_client = mqtt.Client()
        self.__mqtt_client.on_connect = self.__on_mqtt_connect
        self.__mqtt_client.on_message = self.__on_mqtt_msg
        # self.__mqtt_client.connect("localhost")
        self.__mqtt_client.loop_start()

    def __del__(self) -> None:
        self.__mqtt_client.loop_stop()

    def __on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        self.get_logger().info("Connected to MQTT broker")

        if self.__can_edit_config:
            self.__mqtt_client.subscribe(
                self.__mqtt_config_topic)

    def __on_mqtt_msg(self, client, userdata, msg) -> None:
        pass

    def __get_measurement_dict(self, name: str, msg: Measurement, payload: dict = None) -> dict:
        if payload == None:
            payload = {}
        payload[name] = {
            "value": msg.val,
            "timestamp": {
                "sec": msg.header.stamp.sec,
                "nsec": msg.header.stamp.nanosec
            }
        }
        return payload

    def __get_state_dict(self, conv, pretty_conv, name: str, msg: Any, payload: dict = None) -> dict:
        if payload == None:
            payload = {}
        payload[name] = {
            "value": conv(msg.state),
            "value_pretty": pretty_conv(msg.state),
            "timestamp": {
                "sec": msg.header.stamp.sec,
                "nsec": msg.header.stamp.nanosec
            }
        }
        return payload

    def __get_level_state_dict(self, name: str, msg: LevelState, payload: dict = None) -> dict:
        return self.__get_state_dict(s2s.level_state_str, s2s.level_state_pretty_str, name, msg, payload)

    def __get_temperature_state_dict(self, name: str, msg: TemperatureState, payload: dict = None) -> None:
        return self.__get_state_dict(s2s.temperature_state_str, s2s.temperature_state_pretty_str, name, msg, payload)

    def __get_pump_state_dict(self, name: str, msg: PumpState, payload: dict = None) -> dict:
        return self.__get_state_dict(s2s.pump_state_str, s2s.pump_state_pretty_str, name, msg, payload)

    def __get_quality_state_dict(self, name: str, msg: QualityState, payload: dict = None) -> dict:
        return self.__get_state_dict(s2s.quality_state_str, s2s.quality_state_pretty_str, name, msg, payload)

    def __publish_if_rt(self, name: str, msg_stored: Any, msg_new: Any) -> None:
        msg_stored = msg_new
        if self.__is_realtime:
            if type(msg_new) is Measurement:
                payload = self.__get_measurement_dict(name, msg_new)
            elif type(msg_new) is LevelState:
                payload = self.__get_level_state_dict(name, msg_new)
            elif type(msg_new) is TemperatureState:
                payload = self.__get_temperature_state_dict(name, msg_new)
            elif type(msg_new) is PumpState:
                payload = self.__get_pump_state_dict(name, msg_new)
            elif type(msg_new) is QualityState:
                payload = self.__get_quality_state_dict(name, msg_new)
            self.__mqtt_client.publish(
                self.__mqtt_topic_prefix + "telemetry", payload)

    def __non_rt_cb(self) -> None:
        payload = {}
        payload = self.__get_measurement_dict(
            "", self.__water_tank_level, payload)
        payload = self.__get_level_state_dict(
            "", self.__water_tank_state, payload)
        payload = self.__get_measurement_dict(
            "", self.__water_tank_liquid_temp, payload)
        payload = self.__get_temperature_state_dict(
            "", self.__water_tank_liquid_temp_state, payload)
        payload = self.__get_pump_state_dict(
            "", self.__water_tank_pump_state, payload)

        payload = self.__get_measurement_dict(
            "", self.__nutri_tank_level, payload)
        payload = self.__get_level_state_dict(
            "", self.__nutri_tank_state, payload)
        payload = self.__get_measurement_dict(
            "", self.__nutri_tank_liquid_temp, payload)
        payload = self.__get_temperature_state_dict(
            "", self.__nutri_tank_liquid_temp_state, payload)
        payload = self.__get_pump_state_dict(
            "", self.__nutri_tank_pump_state, payload)
        payload = self.__get_measurement_dict(
            "", self.__nutri_tank_tds, payload)
        payload = self.__get_measurement_dict(
            "", self.__nutri_tank_ph, payload)
        payload = self.__get_quality_state_dict(
            "", self.__nutri_tank_quality, payload)

        self.__mqtt_client.publish(
            self.__mqtt_topic_prefix + "telemetry", payload)


def main(args=None):
    rclpy.init(args=args)
    frontend = Frontend()
    rclpy.spin(frontend)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
