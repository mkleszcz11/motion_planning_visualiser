import random
import math
from core.algorithm import Algorithm
from core.node import TreeNode

BIAS = 1/5

class RandomWalkBiasedAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map = map,
                         benchmark_manager = benchmark_manager)        
        if map.start:
            start_node = TreeNode(map.start.x, map.start.y)
            goal_node = TreeNode(map.goal.x, map.goal.y)
            self.nodes.append(start_node)
            self.start_node = start_node
            self.goal_node = goal_node

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        last_node = self.nodes[-1]

        # Introduce bias, from time to time, move towards the goal
        if random.random() < BIAS and self.map.goal:
            vector = (self.map.goal.x - last_node.x, self.map.goal.y - last_node.y)
            dir_x = vector[0] / math.sqrt(vector[0]**2 + vector[1]**2)
            dir_y = vector[1] / math.sqrt(vector[0]**2 + vector[1]**2)
            new_x = last_node.x + dir_x * self.step_size
            new_y = last_node.y + dir_y * self.step_size
        else:
            new_x = last_node.x + random.uniform(-self.step_size, self.step_size)
            new_y = last_node.y + random.uniform(-self.step_size, self.step_size)

        new_x = max(0, min(self.map.width, new_x))
        new_y = max(0, min(self.map.height, new_y))

        if not self.is_collision(new_x, new_y) and not self.is_edge_collision(last_node.x, last_node.y, new_x, new_y):
            new_node = TreeNode(new_x, new_y, parent = last_node)
            last_node.add_child(new_node)
            self.nodes.append(new_node)
            self.steps += 1

            if self.is_complete():
                self.reconstruct_path()
                self.finalize_benchmark()
