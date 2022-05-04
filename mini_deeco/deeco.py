import rclpy
import shortuuid
from typing import Any
from rclpy.node import Node as ROSNode


class UUID(str):
    def __init__(self):
        pass

class Group:
    def __init__(self):
        self.members = []


class Identifiable:
    def __init__(self, uuid=None):
        self.__uuid = uuid if uuid is not None else self.gen_new_uuid()

    @staticmethod
    def gen_new_uuid() -> UUID:
        return shortuuid.uuid()

    @property
    def uuid(self):
        return self.__uuid

    def isid(self, uuid: str) -> bool:
        return self.__uuid == uuid

    def same_uuid(self, other: Any):
        if isinstance(other, Identifiable):
            return other.uuid == self.uuid
        return False


class Simulation(ROSNode):
    def __init__(self, time_limit: float, frequency: float = 0.1, name: str = "deeco_simulation"):
        super(Simulation, self).__init__("deeco_simulation")
        self.frequency = frequency
        self.time_limit = time_limit

        self._ros_executor = rclpy.executors.SingleThreadedExecutor()
        self._ros_executor.add_node(self)
        self.create_timer(self.time_limit, self.end)

    def add_nodes_to_executor(self, *nodes):
        for node in nodes:
            self.executor.add_node(node)

    def end(self):
        print('\nSimulation Ended')
        rclpy.shutdown()

    def start(self):
        try:
            self.executor.spin()
        except Exception as e:
            print('Mission encountered error. Aborting ROS.')
            rclpy.shutdown()
            raise e