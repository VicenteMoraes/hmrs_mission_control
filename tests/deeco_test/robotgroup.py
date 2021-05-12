from abc import abstractmethod

from deeco.core import EnsembleDefinition, BaseKnowledge
from deeco.core import ComponentRole
from deeco.position import Position
from .robot import Robot, Rover


# Role
class Group(ComponentRole):
    def __init__(self):
        super().__init__()
        self.center = None
        self.members = []


class RobotGroup(EnsembleDefinition):
    class RobotGroupKnowledge(BaseKnowledge, Group):
        def __init__(self):
            super().__init__()

        def __str__(self):
            return self.__class__.__name__ + " centered at " + str(self.center) + " with component ids " + str(list(map(lambda x: x.id, self.members)))

    def fitness(self, a: Robot.Knowledge, b: Robot.Knowledge):
        return 1.0 / a.position.dist_to(b.position)

    def membership(self, a: Robot, b: Robot):
        assert type(a) == Robot.Knowledge
        assert type(b) == Robot.Knowledge
        return True

    def knowledge(self, a: Robot.Knowledge, b: Robot.Knowledge):
        knowledge = self.RobotGroupKnowledge()
        knowledge.center = Position.average(a.position, b.position)
        knowledge.members = [a, b]

        return knowledge

    def __str__(self):
        return self.__class__.__name__




class RobotGroup(EnsembleDefinition):
    class RobotGroupKnowledge(BaseKnowledge, Group):
        def __init__(self):
            super().__init__()

        def __str__(self):
            return self.__class__.__name__ + " centered at " + str(self.center) + " with component ids " + str(list(map(lambda x: x.id, self.members)))

    def fitness(self, a: Rover, b: Rover):
        return 1.0 / a.position.dist_to(b.position)

    def membership(self, a: Rover, b: Rover):
        assert isinstance(a, Rover)
        assert isinstance(b, Rover)
        return True

    def knowledge_exchange(self, a: Rover, b: Rover):
        knowledge = self.RobotGroupKnowledge()
        knowledge.center = Position.average(a.position, b.position)
        knowledge.members = [a, b]

        return (a, [])

    def __str__(self):
        return self.__class__.__name__
