import random
import math
from core.algorithm import Algorithm
from core.node import TreeNode

class RRTStarAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None, enable_rewiring=True, enable_shortcut=True,
                 enable_restarting=False, restart_threshold=0.01, max_restarts = 100):
        super().__init__(map=map, benchmark_manager=benchmark_manager)
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


        if map.start:
            start_node = TreeNode(map.start.x, map.start.y)
            self.nodes.append(start_node)

    def step(self):
        print("==== Entering step() ====")  # Clear separator
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        q_rand = self.get_random_sample()
        print(f"  q_rand: {q_rand}")

        q_nearest = self.get_nearest_node(q_rand)
        print(f"  q_nearest: {q_nearest}")

        q_new = self.extend_toward(q_nearest, q_rand)
        print(f"  q_new: {q_new}")

        if q_new is None:
            print("  q_new is None. Returning.")
            return

        print(f"  Checking collision for q_new: ({q_new.x}, {q_new.y})")
        if not self.is_collision(q_new.x, q_new.y):
            print("    q_new is collision-free")
            q_near_nodes = self.get_near_nodes(q_new)
            print(f"    q_near_nodes: {q_near_nodes}")
            q_min = q_nearest
            print(f"    q_min (initial): {q_min}")
            c_min = self.cost(q_nearest) + self.distance(q_nearest.get_position(), q_new.get_position())
            print(f"    c_min: {c_min}")

            print("    Entering loop over q_near_nodes")
            for i, q_near in enumerate(q_near_nodes):  # Add index for clarity
                print(f"      Loop {i}: q_near: {q_near}")
                print(f"      Checking edge collision: ({q_near.x}, {q_near.y}) to ({q_new.x}, {q_new.y})")
                if not self.is_edge_collision(q_near.x, q_near.y, q_new.x, q_new.y):
                    print("        Edge is collision-free")
                    c_near = self.cost(q_near) + self.distance(q_near.get_position(), q_new.get_position())
                    print(f"        c_near: {c_near}")
                    if c_near < c_min:
                        print("        c_near < c_min: True")
                        c_min = c_near
                        q_min = q_near
                        print(f"        New q_min: {q_min}")
                    else:
                        print("        c_near < c_min: False")
                else:
                    print("        Edge collision detected!")
            print("    Exiting loop over q_near_nodes")

            print(f"    Before rewiring: q_new.parent = {q_new.parent}")
            if q_new.parent:
                print(f"      Removing q_new from parent: {q_new.parent}")
                q_new.parent.remove_child(q_new)
            q_new.parent = q_min
            print(f"    q_new.parent set to: {q_min}")
            q_min.add_child(q_new)
            print(f"    q_new added as child of: {q_min}")
            self.nodes.append(q_new)
            print(f"    q_new added to self.nodes.  len(self.nodes) = {len(self.nodes)}")

            if self.enable_rewiring:
                print("    Entering rewire_tree()")
                self.rewire_tree(q_new, q_near_nodes)  # Call the function
                print("    Exiting rewire_tree()")

            self.steps += 1
            print(f"    Steps incremented: {self.steps}")

            if self.is_complete():  # <--- Correct Placement
                print("    Entering is_complete() block")
                self.reconstruct_path()  # <--- reconstruct path *only* if complete
                if self.enable_shortcut:
                    print("      Entering shortcut_path()")
                    self.shortcut_path()
                    print("      Exiting shortcut_path()")

                current_cost = self.compute_path_length()
                print(f"      Current cost: {current_cost}")
                if not self.path_found or current_cost < self.best_path_cost:
                    print("      New best path found!")
                    self.best_path_cost = current_cost
                    self.path_found = True
                    self.iterations_since_improvement = 0
                else:
                     print("      No improvement")

                if self.enable_restarting:
                    if current_cost >= self.best_path_cost * (1 + self.restart_threshold):
                         self.iterations_since_improvement += 1
                    else:
                        self.iterations_since_improvement = 0

                    if self.iterations_since_improvement > 100 and self.restart_count < self.max_restarts:
                        print("        Restarting...")
                        self.restart()
                print("  Before finalize_benchmark()") # Moved print
                self.finalize_benchmark() # Called *only* if complete!
            else:
                print("Not complete")

        # REMOVED ELSE statement

        # self.finalize_benchmark()  <--- REMOVE THIS LINE.  It's in the wrong place!
        print("==== Exiting step() ====")

    def get_random_sample(self):
        return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node, to_position):
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return TreeNode(to_position[0], to_position[1], from_node)
        else:
            theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return TreeNode(new_x, new_y, from_node)

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
        radius = gamma * (math.log(n) / n)**(1/d)
        return min(radius, 10*self.step_size)

    def calculate_gamma(self):
        d = 2
        zeta_d = math.pi
        gamma = 2 * (1 + 1/d)**(1/d) * (self.map.width * self.map.height / zeta_d)**(1/d)
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

                    self.path = self.path[:i+1] + temp_path + self.path[j+1:]
                    j = i + 2
                else:
                    j += 1
            i += 1

    def restart(self):
        self.nodes = []
        if self.map.start:
            start_node = TreeNode(self.map.start.x, map.start.y)
            self.nodes.append(start_node)
        self.path = []
        self.path_found = False
        self.iterations_since_improvement = 0
        self.restart_count += 1

    # def reconstruct_path(self):
    #     logger.info("Reconstructing path...")
    #     if self.map.goal is None:
    #         return
    #
    #     logger.info("Calculating shortest path...")
    #     self.shortest_path = []
    #     node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))
    #     if node is None:  # CRITICAL: Handle the case where no path is found
    #         logger.warning("No path found to goal.")
    #         return
    #
    #     while node is not None:
    #         self.shortest_path.append(node)
    #         node = node.parent
    #     self.shortest_path.reverse()