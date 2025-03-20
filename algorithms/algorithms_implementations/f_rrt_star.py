import random
import math
from core.algorithm import Algorithm
from core.node import Node


class FRRTStarAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None, enable_rewiring=True, enable_shortcut=True,
                 enable_restarting=False, restart_threshold=0.01, max_restarts=100, dichotomy_tolerance=0.01):
        super().__init__(map=map, benchmark_manager=benchmark_manager, step_size=20)  # Increased step size
        self.enable_rewiring = enable_rewiring
        self.enable_shortcut = enable_shortcut
        self.enable_restarting = enable_restarting
        self.best_path_cost = float('inf')
        self.path_found = False
        self.iterations_since_improvement = 0
        self.restart_threshold = restart_threshold
        self.restart_count = 0
        self.max_restarts = max_restarts
        self.path = []  # Path for shortcutting
        self.dichotomy_tolerance = dichotomy_tolerance  # For createNode

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

        if q_new is None:
            return

        if not self.is_collision(q_new.x, q_new.y):
            # --- F-RRT* Specific Steps ---
            q_reachest = self.find_reachest(q_nearest, q_rand)
            q_create = self.create_node(q_reachest, q_rand)

            if q_create is not None:  # create_node can return None
                if q_create.parent:  # ALWAYS remove child
                    q_create.parent.remove_child(q_create)
                q_create.parent = q_reachest
                q_reachest.add_child(q_create)
                self.nodes.append(q_create)

                # Connect q_new to q_create
                if q_new.parent:  # ALWAYS remove child
                    q_new.parent.remove_child(q_new)
                q_new.parent = q_create
                q_create.add_child(q_new)
                self.nodes.append(q_new)
            else:
                return  # If we cannot create a node, do not try to add it

            if self.enable_rewiring:
                q_near_nodes = self.get_near_nodes(q_new)  # Get near nodes *after* adding to tree
                self.rewire_tree(q_new, q_near_nodes)  # Call rewire_tree *here*

            self.steps += 1

            if self.is_complete():
                self.reconstruct_path()
                if self.enable_shortcut:
                    self.shortcut_path()

                current_cost = self.compute_path_length()
                if not self.path_found or current_cost < self.best_path_cost:
                    self.best_path_cost = current_cost
                    self.path_found = True
                    self.iterations_since_improvement = 0

                if self.enable_restarting:
                    if current_cost >= self.best_path_cost * (1 + self.restart_threshold):
                        self.iterations_since_improvement += 1
                    else:
                        self.iterations_since_improvement = 0

                    if self.iterations_since_improvement > 100 and self.restart_count < self.max_restarts:
                        self.restart()
                self.finalize_benchmark()  # Called *only* if complete!
        else:
            self.reconstruct_path()

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

    def find_reachest(self, q_nearest, q_rand):
        """
        Finds the furthest ancestor of q_nearest that can connect to q_rand without collision.
        """
        current_node = q_nearest
        while current_node.parent is not None and not self.is_edge_collision(current_node.parent.x, current_node.parent.y, q_rand[0], q_rand[1]):
            current_node = current_node.parent
        return current_node

    def create_node(self, q_reachest, q_rand):
        """
        Creates a new node between q_reachest and q_rand using a binary search (dichotomy) method.
        """
        q_allow = q_reachest  # Start with q_reachest
        q_forbid = Node(q_rand[0], q_rand[1])  # Use q_rand as a Node object for consistent handling

        # First, check if we can connect directly to q_rand.  If so, just return a node at q_rand
        if not self.is_edge_collision(q_reachest.x, q_reachest.y, q_forbid.x, q_forbid.y):
            return Node(q_forbid.x, q_forbid.y, parent=q_reachest)  # Return a *new* Node at q_rand

        # Dichotomy process
        while self.distance(q_allow.get_position(), q_forbid.get_position()) > self.dichotomy_tolerance:
            mid_x = (q_allow.x + q_forbid.x) / 2
            mid_y = (q_allow.y + q_forbid.y) / 2
            q_mid = Node(mid_x, mid_y)  # Create a temporary Node for q_mid

            if not self.is_edge_collision(q_mid.x, q_mid.y, q_forbid.x, q_forbid.y):
                q_allow = q_mid  # Move q_allow closer to q_forbid
            else:
                q_forbid = q_mid  # Move q_forbid closer to q_allow

        if q_allow == q_reachest:
            return None

        return Node(q_allow.x, q_allow.y, parent=q_reachest)  # Return a *new* Node

    def get_near_nodes(self, q_new):
        radius = self.calculate_radius()
        if not math.isfinite(radius):
            radius = 10 * self.step_size
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
        radius = gamma * (math.log(n + 1e-9) / (n + 1e-9)) ** (1 / d)  # Add small value to n
        return min(radius, 10 * self.step_size)  # Keep step size reasonable

    def calculate_gamma(self):
        d = 2
        zeta_d = math.pi
        gamma = 2 * (1 + 1 / d) ** (1 / d) * (self.map.width * self.map.height / zeta_d) ** (1 / d)
        return gamma

    def cost(self, node, depth=0):
        if depth > 1000:
            print("ERROR: cost() recursion depth exceeded!")
            return float('inf')

        cost = 0
        current = node
        while current.parent is not None:
            cost += self.distance(current.get_position(), current.parent.get_position())
            current = current.parent
            depth += 1
        return cost

    def rewire_tree(self, q_new, q_near_nodes):  # Pass q_new and near nodes
        for q_near in q_near_nodes:
            if not self.is_edge_collision(q_new.x, q_new.y, q_near.x, q_near.y):
                c_new = self.cost(q_new) + self.distance(q_new.get_position(), q_near.get_position())
                if c_new < self.cost(q_near):
                    if q_near.parent:
                        q_near.parent.remove_child(q_near)
                    q_near.parent = q_new
                    q_new.add_child(q_near)  # Correctly add child

    def shortcut_path(self):
        if self.path is None or len(self.path) < 3:
            return

        i = 0
        while i < len(self.path) - 2:
            j = i + 2
            while j < len(self.path):
                if not self.is_edge_collision(self.path[i].x, self.path[i].y, self.path[j].x, self.path[j].y):
                    new_parent = self.path[i]
                    child_to_reconnect = self.path[j]

                    for k in range(i + 1, j):
                        if self.path[k].parent:
                            self.path[k].parent.remove_child(self.path[k])

                    new_parent.add_child(child_to_reconnect)
                    child_to_reconnect.parent = new_parent

                    temp_path = []
                    current = child_to_reconnect
                    while current != new_parent:
                        temp_path.append(current)
                        current = current.parent
                    temp_path.append(new_parent)
                    temp_path.reverse()

                    self.path = self.path[:i + 1] + temp_path + self.path[j + 1:]
                    j = i + 2
                else:
                    j += 1
            i += 1

    def restart(self):
        self.nodes = []
        if self.map.start:
            start_node = Node(self.map.start.x, map.start.y)
            self.nodes.append(start_node)
        self.path = []
        self.path_found = False
        self.iterations_since_improvement = 0
        self.restart_count += 1

    def reconstruct_path(self):
        if self.map.goal is None:
            return

        self.shortest_path = []
        node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))
        if node is None:
            return

        while node is not None:
            self.shortest_path.append(node)
            node = node.parent
        self.shortest_path.reverse()