import re
import ast
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from garden_msgs.msg import Command, State


class ZigbeeBridge(Node):
    def __init__(self) -> None:
        super().__init__("zigbee_bridge")

        self.declare_parameter("namespace", "field")

        self.__namespace = self.get_parameter(
            "namespace").get_parameter_value().string_value

        self.__cmd_subscriber = self.create_subscription(
            Command,
            "command",
            lambda cmd: self.__forward_command(cmd.header.frame_id, cmd), 10)

        self.__state_publisher = self.create_publisher(
            State, "state", 10)

        self.__mqtt_client = mqtt.Client()
        self.__mqtt_client.on_connect = self.__on_mqtt_connect
        self.__mqtt_client.on_message = self.__on_mqtt_msg
        self.__mqtt_client.connect("localhost")
        self.__mqtt_client.loop_start()

    def __del__(self) -> None:
        self.__mqtt_client.loop_stop()

    def __on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        self.get_logger().info("Connected to MQTT broker")
        self.__mqtt_client.subscribe(self.__namespace + "/#")

    def __is_valid_topic(self, topic: str) -> bool:
        pattern = "^%s\/0x[a-z0-9]{16}\/?.*$" % self.__namespace

        res = re.search(pattern, topic)
        if res:
            return True
        return False

    def __get_device_addr(self, topic: str) -> str:
        assert (self.__is_valid_topic(topic))

        pattern = "0x[a-z0-9]{16}"
        res = re.search(pattern, topic)
        return res.group(0)

    def __key_to_state(self, key: str) -> int:
        if key == "temperature":
            return State.TYPE_TEMPERATURE
        elif key == "pressure":
            return State.TYPE_PRESSURE
        elif key == "humidity":
            return State.TYPE_HUMIDITY
        elif key == "onOff":
            return State.TYPE_CLASSIFICATION
        return State.TYPE_UNKNOWN

    def __on_mqtt_msg(self, client, userdata, msg) -> None:
        if not self.__is_valid_topic(msg.topic):
            self.get_logger().warn(f"Incorrect topic: {msg.topic}, skipping")
            return

        device = self.__get_device_addr(msg.topic)
        self.get_logger().info(
            f"Received payload {msg.payload} from {device}, processing", throttle_duration_sec=10)

        payload = msg.payload.decode("utf-8")
        try:
            payload = ast.literal_eval(payload)
        except:
            self.get_logger().warn(
                f"Payload {msg.payload} of {device} couldn't be parsed, skipping")
            return

        msg = State()
        msg.header.frame_id = device
        msg.header.stamp = self.get_clock().now().to_msg()

        for k in payload:
            msg.type = self.__key_to_state(k)
            if msg.type != State.TYPE_UNKNOWN:
                msg.value = float(payload[k])
                self.__state_publisher.publish(msg)
                self.get_logger().info(
                    f"Publishing {k}: {msg.value} of {device}")
            else:
                self.get_logger().warn(
                    f"Can't convert {k} to any known state enum, skipping", throttle_duration_sec=10)

    def __is_command_onOff(self, cmd: Command) -> bool:
        if cmd.endpoint in [Command.FAN_EP, Command.LED_STRAT_EP,
                            Command.WATER_EP, Command.NUTRI_EP]:
            return True
        return False

    def __is_command_level(self, cmd: Command) -> bool:
        if cmd.endpoint in [Command.LED1_EP, Command.LED2_EP]:
            return True
        return False

    def __forward_command(self, device: str, command: Command) -> None:
        if self.__is_command_onOff(command):
            self.__send_onOff_msg(device, command.endpoint, command.value)
        elif self.__is_command_level(command):
            self.__send_level_msg(device, command.endpoint, command.value)
        else:
            self.get_logger().warn(
                f"Unknown type of command type ({command.endpoint}), skipping")

    def __send_onOff_msg(self, device: str, endpoint: int, state: int) -> None:
        if state:
            state = 1

        self.__send_msg(
            device, endpoint,
            "{\"write\":{\"cluster\":\"genOnOff\",\"options\":{},\"payload\":{\"onOff\":%d}}}" % state)

    def __send_level_msg(self, device: str, endpoint: int, level: int) -> None:
        self.__send_msg(
            device, endpoint,
            "{\"write\":{\"cluster\":\"genLevelCtrl\",\"options\":{},\"payload\":{\"currentLevel\":%d}}}" % level)

    def __send_msg(self, device: str, endpoint: int, payload: str) -> None:
        topic = self.__namespace + "/" + device + "/" + str(endpoint) + "/set"
        self.__mqtt_client.publish(topic, payload)


def main(args=None):
    rclpy.init(args=args)
    bridge = ZigbeeBridge()
    rclpy.spin(bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
