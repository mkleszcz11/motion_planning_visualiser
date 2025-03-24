import random
import math
from core.algorithm import Algorithm
from core.node import TreeNode


class RRTConnectAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map=map, benchmark_manager=benchmark_manager)
        self.tree_a = []  # Tree rooted at the start
        self.tree_b = []  # Tree rooted at the goal

        if map.start:
            start_node = TreeNode(map.start.x, map.start.y)
            self.tree_a.append(start_node)
            self.nodes.append(start_node)  # Keep track of all nodes for visualization
        if map.goal:
            goal_node = TreeNode(map.goal.x, map.goal.y)
            self.tree_b.append(goal_node)
            # self.nodes.append(goal_node) # Do not append goal, it messes with visualizations.

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        # 1. Sample a random configuration
        q_rand = self.get_random_sample()

        # 2. Extend tree_a towards q_rand
        self.extend_and_connect(self.tree_a, self.tree_b, q_rand)  # No longer check return value here

        # Check for completion *after* extending and *before* swapping
        if self.is_complete():
            self.reconstruct_path()
            self.finalize_benchmark()

        # 3. Swap trees (important for balanced exploration)
        self.tree_a, self.tree_b = self.tree_b, self.tree_a
        self.steps += 1 # Increment steps *after* the swap

    def extend_and_connect(self, tree_from, tree_to, q_target):
        """Extends tree_from towards q_target and tries to connect to tree_to.

        Args:
            tree_from: The tree to extend.
            tree_to:   The tree to try to connect to.
            q_target:  The target configuration (can be q_rand or a node from tree_to).

        Returns:
            True if the trees are connected, False otherwise.  <-- Return value is still useful
        """
        q_near = self.get_nearest_node_in_tree(q_target, tree_from)
        q_new = self.extend_toward(q_near, q_target)

        if q_new is None:  # extend_toward may return None
            return False

        if not self.is_collision(q_new.x, q_new.y) and not self.is_edge_collision(q_near.x, q_near.y, q_new.x, q_new.y):
            q_near.add_child(q_new)
            tree_from.append(q_new)
            self.nodes.append(q_new)  # Add to the overall node list for visualization

            # --- Connect Heuristic (try to connect to tree_to) ---
            q_connect_near = self.get_nearest_node_in_tree(q_new.get_position(),
                                                           tree_to)  # find the nearest in the *other* tree
            q_connect = q_new  # Initialize

            while True:  # Keep connecting
                q_next = self.extend_toward(q_connect_near,
                                            q_connect.get_position())  # Extend from tree_to towards q_connect

                if q_next is None:  # Extend may return None
                    break

                if not self.is_collision(q_next.x, q_next.y) and not self.is_edge_collision(q_connect_near.x,
                                                                                            q_connect_near.y, q_next.x,
                                                                                            q_next.y):
                    q_connect_near.add_child(q_next)
                    tree_to.append(q_next)

                    if self.distance(q_next.get_position(), q_connect.get_position()) < self.step_size:
                        # --- Trees Connected! ---
                        self.reconstruct_path_connect(tree_from, tree_to, q_new, q_next)  # Pass the connecting nodes
                        #self.finalize_benchmark()  <-- Don't finalize here! Do it in step()
                        return True  # Connection made

                    q_connect_near = q_next  # Keep extending to connect

                else:  # Collision
                    break  # Stop extending this branch

        return False  # No connection made

    def get_random_sample(self):
        return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node, to_position):
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            # If within step_size, we go all the way.
            return TreeNode(to_position[0], to_position[1], from_node)  # Return the actual node
        else:
            theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return TreeNode(new_x, new_y, from_node)  # Return the new node.

    def get_nearest_node_in_tree(self, position, tree):
        """Finds the nearest node to a given position within a specific tree."""
        min_dist = float('inf')
        nearest_node = None
        for node in tree:
            dist = self.distance(node.get_position(), position)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def reconstruct_path_connect(self, tree_a, tree_b, q_a, q_b):
        """Reconstructs the path when using RRT-Connect.
           Needs to handle two separate trees.
        """

        # Find the nodes in each tree that connect the trees
        node_a = q_a
        node_b = q_b

        # Path from start to connection point
        path_a = []
        while node_a:
            path_a.append(node_a)
            node_a = node_a.parent
        path_a.reverse()  # Reverse to get start -> connection

        # Path from goal to connection point
        path_b = []
        while node_b:
            path_b.append(node_b)
            node_b = node_b.parent
        # Don't reverse path_b, we want goal -> connection

        # Combine paths
        self.path = path_a + path_b[::-1]  # Concatenate and make sure path b is in correct order