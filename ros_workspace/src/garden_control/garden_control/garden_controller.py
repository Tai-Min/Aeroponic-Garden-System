import rclpy
from rclpy.node import Node
from tank_msgs.msg import State
from tank_msgs.action import DrivePump
from rclpy.action import ActionClient

class GardenController(Node):
    def __init__(self) -> None:
        super().__init__("garden_controller")

        self.__water_state = State.UNKNOWN
        self.__nutri_state = State.UNKNOWN

        self.__water_state_sub = self.create_subscription(State, "/water_tank/state")
        self.__nutri_state_sub = self.create_subscription(State, "/nutri_tank/state", )

        self.__drive_water_client = ActionClient(self, DrivePump, "/water_tank/drive")
        self.__drive_nutri_client = ActionClient(self, DrivePump, "/nutri_tank/drive")

    def __on_water_state_received_callback(self, msg: State) -> None:
        pass

    def __on_nutri_state_received_callback(self, msg: State) -> None:
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = GardenController()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
