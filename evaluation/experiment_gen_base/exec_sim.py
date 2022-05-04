import rclpy
from lagom import Container

from mission_control.core import Battery
from mini_deeco.deeco import Simulation

from utils.logger import ContextualLogger
from utils.timer import Timer
from mission_control.deeco_integration.deeco_timer import DeecoTimer

from mission_control.deeco_integration.mission_coordination_ensemble import MissionCoordinationEnsemble
from mission_control.deeco_integration.robot import Robot
from mission_control.deeco_integration.coordinator import Coordinator
from mission_control.deeco_integration.plugins.requests_queue import RequestsQueue
from mission_control.deeco_integration.mission_coordination_ensemble import MissionCoordinationEnsemble


from mission_control.processes.coalition_formation import CoalitionFormationProcess

from resources.world_lab_samples import *
from .to_executor import prep_plan
from .scenario import Scenario

print("Running simulation")
class SimExec:
    def __init__(self, container: Container):
        self.cf_process: CoalitionFormationProcess = container[CoalitionFormationProcess]
        self.cl = container[ContextualLogger]

    @staticmethod
    def instantiate_robot_component(sim: Simulation, battery_charge, **initial_knowledge):
        return Robot(sim=sim,
                     battery=Battery(capacity=1, charge=battery_charge),
                     **initial_knowledge)

    def run(self, scenario: Scenario, limit_s=5):
        print(f'init scenario {scenario.code}')
        self.cl.start_group_context(f'{scenario.code}')
        requests = scenario.requests
        robots_initial_conf = scenario.robots
        
        cf_process = self.cf_process

        sim = Simulation(limit_s, name=f"{scenario.code}")

        # Add identity replicas plugin (provides replicas using deep copies of original knowledge)
        #IdentityReplicas(sim)

        # Add simple network device
        #SimpleNetwork(sim)
        
        # wire sim timer with container timer
        #timer:DeecoTimer = container[Timer]
        #timer.scheduler = sim.scheduler

        # create coordinator
        #coord_node = Node(sim)
        # node plugins
        #KnowledgePublisher(coord_node)
        #RequestsQueue(coord_node, requests)
        #EnsembleReactor(coord_node, [ MissionCoordinationEnsemble() ])

        # mission coordinator component
        coord = Coordinator(sim=sim, name='mission_coordinator', required_skills=[], cf_process=cf_process,
                            requests=requests)

        robots = []


        # instantiate workers
        for r in robots_initial_conf:
            robot = self.instantiate_robot_component(sim, **r)
            # save to report
            robots.append(robot)

        # Run the simulation
        #sim.run(limit_ms)
        sim.start()
        self.cl.end_all_contexts()

        # get results
        local_plans = list(map(prep_plan, robots))
        
        for local_plan, robot in zip(local_plans, scenario.robots):
            robot['local_plan'] = local_plan
        
        return {
            'components': {
                'coordinator': coord.knowledge,
                'robots': list(map(lambda r : r.knowledge, robots))
            },
            'local_plans': local_plans,
            'missions': coord.knowledge.missions
        }