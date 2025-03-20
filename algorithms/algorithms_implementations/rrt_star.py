import random
import math
from core.algorithm import Algorithm
from core.node import Node

class RRTStarAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None, enable_rewiring=True, enable_shortcut=False,
                 enable_restarting=False, restart_threshold=0.01, max_restarts = 100): # Added parameters
        super().__init__(map=map, benchmark_manager=benchmark_manager)
        self.enable_rewiring = enable_rewiring
        self.enable_shortcut = enable_shortcut
        self.enable_restarting = enable_restarting
        self.best_path_cost = float('inf')  # Track the best path cost found so far
        self.path_found = False
        self.iterations_since_improvement = 0  # Track iterations without improvement
        self.restart_threshold = restart_threshold  # Threshold for restarting (relative improvement)
        self.restart_count = 0 # counter
        self.max_restarts = max_restarts # maximum number of restarts


        if map.start:
            start_node = Node(map.start.x, map.start.y)
            self.nodes.append(start_node)

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        # 1. Sample a random configuration
        q_rand = self.get_random_sample()

        # 2. Find the nearest node in the tree
        q_nearest = self.get_nearest_node(q_rand)

        # 3. Steer from q_nearest towards q_rand
        q_new = self.extend_toward(q_nearest, q_rand)

        if q_new is None:  # extend_toward can return None
            return

        if not self.is_collision(q_new.x, q_new.y):
            # --- RRT* Specific Steps ---
            q_near_nodes = self.get_near_nodes(q_new)
            q_min = q_nearest
            c_min = self.cost(q_nearest) + self.distance(q_nearest.get_position(), q_new.get_position())

            for q_near in q_near_nodes:
                if not self.is_edge_collision(q_near.x, q_near.y, q_new.x, q_new.y):
                    c_near = self.cost(q_near) + self.distance(q_near.get_position(), q_new.get_position())
                    if c_near < c_min:
                        c_min = c_near
                        q_min = q_near

            q_new.parent = q_min
            q_min.add_child(q_new)
            self.nodes.append(q_new)

            if self.enable_rewiring:
                for q_near in q_near_nodes:
                    if not self.is_edge_collision(q_new.x, q_new.y, q_near.x, q_near.y):
                        c_new = self.cost(q_new) + self.distance(q_new.get_position(), q_near.get_position())
                        if c_new < self.cost(q_near):
                            if q_near.parent is not None:
                                q_near.parent.remove_child(q_near)
                            q_near.parent = q_new
                            q_new.add_child(q_near)
            self.steps += 1
            if self.is_complete():
                self.path_found = True

        # --- Actions AFTER a path is found ---
        if self.path_found:
            if self.enable_rewiring:
                self.rewire_tree()

            if self.enable_shortcut:
                self.shortcut_path()

            if self.enable_restarting:
                current_cost = self.cost(self.get_nearest_node((self.map.goal.x, self.map.goal.y)))
                if current_cost < self.best_path_cost * (1 - self.restart_threshold): # Check for significant improvement
                    self.best_path_cost = current_cost
                    self.iterations_since_improvement = 0  # Reset counter
                else:
                    self.iterations_since_improvement += 1
                if self.iterations_since_improvement > 100 and self.restart_count < self.max_restarts :  # Limit restarts, and wait for iterations
                    self.restart()
            self.reconstruct_path() # Moved reconstruct_path here
            self.finalize_benchmark() # Keep this at the end

    def get_random_sample(self):
        return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node, to_position):
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return Node(to_position[0], to_position[1], from_node)
        else:
            theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return Node(new_x, new_y, from_node)

    def get_near_nodes(self, q_new):
        radius = self.calculate_radius()
        near_nodes = []
        for node in self.nodes:
            if self.distance(node.get_position(), q_new.get_position()) < radius:
                near_nodes.append(node)
        return near_nodes

    def calculate_radius(self):
        n = len(self.nodes)
        if n == 0:
            return float('inf')
        d = 2
        gamma = self.calculate_gamma()
        radius = gamma * (math.log(n) / n)**(1/d)
        return min(radius, self.step_size)

    def calculate_gamma(self):
        d = 2
        zeta_d = math.pi
        gamma = 2 * (1 + 1/d)**(1/d) * (self.map.width * self.map.height / zeta_d)**(1/d)
        return gamma

    def cost(self, node):
        cost = 0
        current = node
        while current.parent is not None:
            cost += self.distance(current.get_position(), current.parent.get_position())
            current = current.parent
        return cost

    def rewire_tree(self):
        for node in self.nodes:
            if node.parent is not None:
                near_nodes = self.get_near_nodes(node)
                for near_node in near_nodes:
                    if not self.is_edge_collision(node.x, node.y, near_node.x, near_node.y):
                        new_cost = self.cost(near_node) + self.distance(near_node.get_position(), node.get_position())
                        if new_cost < self.cost(node):
                            node.parent.remove_child(node)
                            node.parent = near_node
                            near_node.add_child(node)


    def shortcut_path(self):
        """Attempts to shorten the current path by connecting non-adjacent nodes."""
        if len(self.path) < 3:  # Need at least 3 nodes in the path
            return

        for i in range(len(self.path) - 2):  # Iterate through nodes in the path
            for j in range(i + 2, len(self.path)):  # Try to connect to nodes further down
                if not self.is_edge_collision(self.path[i].x, self.path[i].y, self.path[j].x, self.path[j].y):
                    # Check for collision-free connection
                    if self.cost(self.path[i]) + self.distance(self.path[i].get_position(), self.path[j].get_position()) < self.cost(self.path[j]):
                        # Make sure new connection is better
                        if self.path[j].parent is not None:
                            self.path[j].parent.remove_child(self.path[j])
                        self.path[i].add_child(self.path[j])
                        self.path[j].parent = self.path[i]


    def restart(self):
        #print("Restarting...") # Removed print for cleaner output.
        self.nodes = []
        if self.map.start:
            start_node = Node(self.map.start.x, map.start.y)
            self.nodes.append(start_node)
        self.path = []
        self.path_found = False
        self.iterations_since_improvement = 0  # Reset counter
        self.restart_count += 1 # increment restart count

    # def reconstruct_path(self):
    #     if self.map.goal is None:
    #         return
    #
    #     self.path = []
    #     nearest_node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))
    #     if nearest_node is None:
    #         return
    #
    #     if self.distance(nearest_node.get_position(), (self.map.goal.x, self.map.goal.y)) >= self.step_size:
    #         return
    #
    #     node = nearest_node
    #     while node is not None:
    #         self.path.append(node)
    #         node = node.parent
    #     self.path.reverse()