#!/usr/bin/env python3
from enum import Enum
from typing import Callable, List

class BTStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BTNode:
    def tick(self) -> BTStatus:
        raise NotImplementedError

class ConditionNode(BTNode):
    def __init__(self, condition: Callable[[], bool]):
        self.condition = condition
    def tick(self) -> BTStatus:
        return BTStatus.SUCCESS if self.condition() else BTStatus.FAILURE

class ActionNode(BTNode):
    def __init__(self, action: Callable[[], BTStatus]):
        self.action = action
    def tick(self) -> BTStatus:
        return self.action()

class SequenceNode(BTNode):
    def __init__(self, children: List[BTNode]):
        self.children = children
    def tick(self) -> BTStatus:
        for child in self.children:
            status = child.tick()
            if status != BTStatus.SUCCESS:
                return status
        return BTStatus.SUCCESS

class SelectorNode(BTNode):
    def __init__(self, children: List[BTNode]):
        self.children = children
    def tick(self) -> BTStatus:
        for child in self.children:
            status = child.tick()
            if status != BTStatus.FAILURE:
                return status
        return BTStatus.FAILURE
