import random
import math
from core.algorithm import Algorithm
from core.node import Node


class RRTConnectAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map=map, benchmark_manager=benchmark_manager)

        # Initialize two trees (start and goal trees)
        self.tree_start = []  # Start tree
        self.tree_goal = []  # Goal tree

        # Path reconstruction and tree connection info
        self.path = []
        self.joint_start = None  # Node where start tree connects to the goal tree
        self.joint_goal = None  # Node where goal tree connects to the start tree

        # Ensure the map contains valid start and goal points
        if not map.start or not map.goal:
            raise ValueError("Start and goal points are required.")

        # Add start and goal nodes to their respective trees
        self.tree_start.append(Node(map.start.x, map.start.y))
        self.tree_goal.append(Node(map.goal.x, map.goal.y))

    def step(self):
        """Perform a single iteration step of the RRT-Connect algorithm."""
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        # Alternate between the start and goal trees for the connection
        active_tree, passive_tree = (self.tree_start, self.tree_goal)

        # Sample a random point in the search space
        random_sample = self.get_random_sample()

        # Perform step for the active tree: Extend toward the sampled point
        nearest_active = self.get_nearest_node_in_tree(random_sample, active_tree)
        new_node_active = self.extend_toward(nearest_active, random_sample)

        # Check for collisions to add the new node
        if new_node_active and not self.is_collision(new_node_active.x, new_node_active.y):
            if not self.is_edge_collision(nearest_active.x, nearest_active.y, new_node_active.x, new_node_active.y):
                nearest_active.add_child(new_node_active)
                active_tree.append(new_node_active)

                # Try to connect the second (passive) tree
                nearest_passive = self.get_nearest_node_in_tree((new_node_active.x, new_node_active.y), passive_tree)
                new_node_passive = self.extend_toward(nearest_passive, (new_node_active.x, new_node_active.y))

                # Iteratively connect the passive tree
                while new_node_passive and not self.is_collision(new_node_passive.x, new_node_passive.y):
                    if not self.is_edge_collision(nearest_passive.x, nearest_passive.y, new_node_passive.x,
                                                  new_node_passive.y):
                        nearest_passive.add_child(new_node_passive)
                        passive_tree.append(new_node_passive)

                        # Check if connection between trees is complete
                        if self.distance((new_node_active.x, new_node_active.y),
                                         (new_node_passive.x, new_node_passive.y)) < self.step_size:
                            self.joint_start = new_node_active
                            self.joint_goal = new_node_passive
                            self.reconstruct_path()  # Reconstruct the path
                            self.finalize_benchmark()  # Finalize benchmark if needed
                            return

                    # Otherwise, extend further
                    nearest_passive = new_node_passive
                    new_node_passive = self.extend_toward(nearest_passive, (new_node_active.x, new_node_active.y))

        # Swap active and passive trees for the next iteration
        self.tree_start, self.tree_goal = self.tree_goal, self.tree_start
        self.steps += 1

    def get_random_sample(self):
        """Generate a random sample within the map boundaries."""
        return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node, to_position):
        """Extend from a node toward a target position, respecting the step size."""
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return Node(to_position[0], to_position[1], from_node)  # Create a new node at the target

        # Calculate direction and generate a new node along the line
        theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)
        return Node(new_x, new_y, from_node)

    def get_nearest_node_in_tree(self, target_position, tree):
        """Find the nearest node to a given position within a tree."""
        nearest_node = None
        min_distance = float('inf')
        for node in tree:
            dist = self.distance(node.get_position(), target_position)
            if dist < min_distance:
                nearest_node = node
                min_distance = dist
        return nearest_node

    def reconstruct_path(self):
        """Reconstruct the path from the start to the goal by joining the trees."""
        # Build the path from the start tree
        node = self.joint_start
        start_path = []
        while node:
            start_path.append(node)
            node = node.parent
        start_path = start_path[::-1]  # Reverse to get the correct order

        # Build the path from the goal tree
        node = self.joint_goal
        goal_path = []
        while node:
            goal_path.append(node)
            node = node.parent

        # Combine paths
        self.path = start_path + goal_path

    def is_complete(self):
        """Return True if the path has been fully reconstructed."""
        return len(self.path) > 0
