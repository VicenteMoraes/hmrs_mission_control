import rclpy
from queue import Queue

#from deeco.plugins.ensemblereactor import EnsembleMember
from mission_control.common_descriptors.navigation_sd import Move
from typing import List, Mapping

from mini_deeco.knowledge import BaseKnowledge
from mini_deeco.component import Component
from mini_deeco.deeco import UUID, Simulation
#from deeco.core import BaseKnowledge, Component, ComponentRole, Node, UUID
#from deeco.core import process

from ..core import BatteryTimeConstantDischarge, LocalMission, MissionContext, Worker
from ..processes.integration import MissionHandler, MissionUnnexpectedError
from ..processes.coalition_formation import CoalitionFormationProcess
from ..processes.supervision import SupervisionProcess


class MissionCoordinator:
    def __init__(self):
        self.missions: List[MissionContext] = []
        self.active_workers: dict[UUID, Worker] = {}
    
    def update_worker(self, member: Worker):
        self.active_workers[member.uuid] = member.knowledge


class Coordinator(Component):
    # Knowledge definition
    class Knowledge(MissionCoordinator, BaseKnowledge):
        def __init__(self):
            super(Coordinator.Knowledge, self).__init__()

    # Component initialization
    def __init__(self, sim: Simulation,
                 name: str = "",
                 server_name: str = "",
                 frequency: float = 0,
                 required_skills=None,
                 cf_process: CoalitionFormationProcess = None,
                 supervision_process: SupervisionProcess = None,
                 requests: List = None):
        super(Coordinator, self).__init__(sim=sim, name=name, server_name=server_name, frequency=frequency)
        self.cf_process = cf_process
        self.supervision_process = supervision_process
        self.name = name
        self.knowledge = self.Knowledge()
        self.requests = Queue()
        if requests is not None:
            self.add_requests(requests)

        # Initialize knowledge

        print("Coordinator " + str(self.name) + " created")
        self.ros_node.create_timer(1, self.coalition_formation)

    def add_requests(self, requests):
        for request in requests:
            self.requests.put(request)

    def coalition_formation(self):
        for mission_context in list(self.get_missions_with_pending_assignments()):
            workers = list(map(self.transform_worker, self.get_free_workers().items()))
            self.cf_process.run(mission_context, workers, self)

    def get_missions_with_pending_assignments(self):
        for mission_context in self.knowledge.missions:
            if self.has_pending_assignment(mission_context):
                yield mission_context
        
        for new_mission_context in self.handle_requests():
            yield new_mission_context

        return

    @staticmethod
    def has_pending_assignment(mission_context: MissionContext):
        return any(filter(lambda lm: lm.assignment_status == LocalMission.AssignmentStatus.NOT_ASSIGNED, mission_context.local_missions))

    def handle_requests(self):
        while not self.requests.empty():
            request = self.requests.get()
            mission_context = MissionContext(request_id = request.id, global_plan=request.task)
            print(f'coordinator {self._uuid} got has a new mission {mission_context}')
            self.knowledge.missions.append(mission_context)
            yield mission_context
        return

    def start_mission(self, mission_context):
        print(f'mission started {mission_context}')
    
    def update_assigments(self, mission_context: MissionContext):
        print(f'received an update for {mission_context}')
    
    def no_coalition_available(self, mission_context: MissionContext):
        print(f'no coalition available for {mission_context}')
    
    # @process(period_ms=1000)
    def supervision(self, node):
        for active_mission in self.get_active_missions():
            task_updates = self.get_pending_updates(active_mission)
            assigned_workers = self.get_assigned_workers(active_mission)
            self.supervision_process.run(active_mission, task_updates)

    def report_progress(self, acive_mission):
        def log():
            pass
        return log

    def end_mission(self, mission_context):
        print('mission ended')
        self.knowledge.missions.remove(mission_context)
        self.node.log_mission_end(mission_context)


    def completed_assignment():
        print(f'completed assignment')
    
    def notify_operator():
        print(f'op notification requested')

    def status_update_to_user():
        print(f'user update requested')

    def queue_request(self, request):
        print(f'request received')

    def handle_unnexpected_error(error: MissionUnnexpectedError):
        """ 
            Should handle internal error in the mission management process
        """
        print(error.message)
        print(error.orignal_error)

    @staticmethod
    def transform_worker(uuid_worker):
        uuid, worker_knowledge = uuid_worker
        worker = Worker(
            uuid=uuid,
            name=worker_knowledge.name,
            location=worker_knowledge.location,
            skills=worker_knowledge.skills,
            capabilities=[
                Move(avg_speed = worker_knowledge.avg_speed, u='m/s')
            ],
            resources=[
                BatteryTimeConstantDischarge(
                    battery=worker_knowledge.battery,
                    discharge_rate=worker_knowledge.battery_discharge_rate,
                    minimum_useful_level=0.05
                )
            ])
        return worker

    def get_free_workers(self) -> Mapping[UUID, Worker]:
        workers = dict(self.knowledge.active_workers)
        missions = self.knowledge.missions 
        assigned_workers = []
        
        for mission in missions:
            for local_mission in mission.local_missions:
                if local_mission.worker and local_mission.worker.uuid in workers:
                    workers.pop(local_mission.worker.uuid)
        
    
        return workers

