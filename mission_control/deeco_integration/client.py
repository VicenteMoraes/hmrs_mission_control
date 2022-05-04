from mini_deeco.knowledge import BaseKnowledge
from mini_deeco.component import Component
from mini_deeco.deeco import Simulation

class MissionClient():
    def __init__(self):
        super().__init__()
        self.requests = []
        self.mission_status = []

# Component
class Client(Component):
	# Knowledge definition
	class Knowledge(BaseKnowledge, MissionClient):
		def __init__(self):
			super().__init__()

	# Component initialization
	def __init__(self, sim: Simulation, name: str = "", frequency: float = 0):
		super(Client, self).__init__(sim=sim, name=name, frequency=frequency)

	# Processes follow
	def add_request(self, request):
		self.knowledge.requests.append(request)
