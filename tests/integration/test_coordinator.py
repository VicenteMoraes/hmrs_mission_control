from mission_control.core import MissionContext, LocalMission, Worker
from mission_control.deeco_integration.coordinator import Coordinator

import rclpy
from mini_deeco.deeco import Simulation

class NodeMock():
    def __init__(self):
        self.id = 0

def test_get_free_workers():
    w1 = Worker(location=None, capabilities=[], skills=[])
    w2 = Worker(location=None, capabilities=[], skills=[])
    w3 = Worker(location=None, capabilities=[], skills=[])

    mission1 = MissionContext(request_id=0, global_plan=None)
    mission1.local_missions = [LocalMission(None, None, None, worker=w1)]

    rclpy.init()
    sim = Simulation(time_limit=1, name="free_workers_test")
    
    coord = Coordinator(sim=sim,  required_skills=None, cf_process=None)
    coord.knowledge.missions = [mission1]
    coord.knowledge.active_workers = {w.uuid: w for w in [w1, w2, w3]}

    w2_and_w3 = {w.uuid: w for w in [w2, w3]}

    assert w2_and_w3 == dict(coord.get_free_workers())

    
