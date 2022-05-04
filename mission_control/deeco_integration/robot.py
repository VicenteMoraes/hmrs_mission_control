from random import Random
from typing import List
from mini_deeco.deeco import Simulation
from mini_deeco.knowledge import BaseKnowledge
from mini_deeco.component import Component
#from deeco.core import Node
#from deeco.core import process
from mini_deeco.position import Position

from mission_control.core import Battery, LocalMission, POI


# Roles
class Worker():
	def __init__(self):
		self.name = None
		self.skills = None
		self.local_mission: LocalMission = None
		self.location: POI = None
		self.battery: Battery = None
		self.battery_discharge_rate: float = None
		self.avg_speed: float = None

# Component
class Robot(Component):
	@staticmethod
	def gen_position():
		return Position(x=Robot.random.uniform(0, 1), y=Robot.random.uniform(0, 1))

	# Knowledge definition
	class Knowledge(BaseKnowledge, Worker):
		pass

	# Component initialization
	def __init__(self, sim: Simulation,
				name: str = "",
				frequency: float = 0,
				skills: List[str] = None,
				location: POI = None,
				battery: Battery = None,
				battery_discharge_rate = None,
				local_mission: LocalMission=None,
				avg_speed = 0, 
				id = 0):

		super().__init__(sim=sim, name=name, frequency=frequency)

		# Initialize knowledge
		self.knowledge.name = name
		self.knowledge.skills = skills
		self.knowledge.location = location
		self.knowledge.battery = battery
		self.knowledge.battery_discharge_rate = battery_discharge_rate
		self.knowledge.local_mission = local_mission
		self.knowledge.avg_speed = avg_speed

		print("Robot " + str(self.name) + " created")

		self.ros_node.create_timer(1, self.sequencing)

	def sequencing(self):
		if self.knowledge.local_mission:
			print(self.knowledge.local_mission)

	#def sense_task_execution_status(self, node: Node):
	#	pass
	#	TODO

	# @process(period_ms=100)
	# def sense_position(self, node: Node):
	# 	self.knowledge.position = node.positionProvider.get()
