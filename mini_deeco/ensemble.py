import shortuuid
import rclpy
from deeco_actions.action import KnowledgeExchange
from abc import abstractmethod


class Ensemble:
    def __init__(self):
        pass

    @abstractmethod
    def membership(self, candidate):
        pass


class MissionEnsemble(Ensemble):
    def __init__(self, sim, name: str = "", frequency: float = None):
        super(MissionEnsemble, self).__init__()
        self.frequency = frequency if frequency is not None else sim.frequency
        self.ros_node = sim
        self.name = name
        self._uuid = shortuuid.uuid()
        self.clients = []

        self._action_client = rclpy.action.ActionClient(
            self.ros_node,
            KnowledgeExchange,
            f'ensemble_{name}'
        )

        self.ros_node.create_timer(self.frequency, self._request_goals_to_clients)

    def add_clients(self, *candidates):
        self.clients += [rclpy.action.ActionClient(self.ros_node, KnowledgeExchange, candidate.server_name)
                         for candidate in candidates if self.membership(candidate)]

    def _request_goals_to_clients(self):
        for client in self.clients:
            msg = self.request_callback(client)
            goal_future = client.send_goal_async(msg, feedback_callback=self.feedback_callback)
            goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        if self.response_callback(future):
            goal_handle = future.result()
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.result_callback)

    def membership(self, candidate):
        pass

    # Action request to component server
    @abstractmethod
    def request_callback(self, client) -> KnowledgeExchange:
        pass

    # Callback for client server connection response
    @staticmethod
    def response_callback(future) -> bool:
        return future.result().accepted

    # Callback for knowledge transfer feedback
    @abstractmethod
    def feedback_callback(self, feedback):
        feedback_msg = feedback.feedback
        print(f'feedback_msg: {feedback_msg}')

    # Callback for component action server results
    @abstractmethod
    def result_callback(self, future):
        result = future.result().result
        print(f'result: {result}')