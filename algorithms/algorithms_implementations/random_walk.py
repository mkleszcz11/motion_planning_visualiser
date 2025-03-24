import random
import math
from core.algorithm import Algorithm
from core.node import TreeNode

class RandomWalkAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map = map,
                         benchmark_manager = benchmark_manager)        
        if map.start:
            start_node = TreeNode(map.start.x, map.start.y)
            self.nodes.append(start_node)

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        if self.is_complete():
            if self.map.goal and self.get_nearest_node((self.map.goal.x, self.map.goal.y)) not in self.nodes:
                goal_node = TreeNode(self.map.goal.x, self.map.goal.y)
                self.nodes.append(goal_node)
                self.reconstruct_path()
                self.finalize_benchmark()
            return

        if not self.nodes:
            return

        last_node = self.nodes[-1]
        new_x = last_node.x + random.uniform(-self.step_size, self.step_size)
        new_y = last_node.y + random.uniform(-self.step_size, self.step_size)

        new_x = max(0, min(self.map.width, new_x))
        new_y = max(0, min(self.map.height, new_y))

        if not self.is_collision(new_x, new_y) and not self.is_edge_collision(last_node.x, last_node.y, new_x, new_y):
            new_node = TreeNode(new_x, new_y, last_node)
            last_node.add_child(new_node)
            self.nodes.append(new_node)
            self.steps += 1

            if self.is_complete():
                goal_node = TreeNode(self.map.goal.x, self.map.goal.y)
                self.nodes.append(goal_node)
                self.reconstruct_path()
                self.finalize_benchmark()
