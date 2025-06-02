import random
import math
import typing as t

from core.algorithm import Algorithm
from core.node import TreeNode
from core.map import Map
from benchmarks.benchmark_manager import BenchmarkManager

BIAS = 10/100  # Original probability of sampling the goal directly


class RRTStarBetaAlgorithm(Algorithm):  # Rename this to RRTStarBetaAlgorithm if that's your class name
    def __init__(self,
                 map: Map,
                 benchmark_manager: BenchmarkManager = None,
                 alpha: float = 10/100,  # Probability of sampling within the cone
                 beta: float = math.pi * 0.95):  # Inclusive angle of the cone in radians

        super().__init__(map=map,
                         benchmark_manager=benchmark_manager)

        self.alpha = alpha
        self.beta = beta

        if not (0 <= self.alpha <= 1):
            raise ValueError("alpha (probability of sampling in cone) must be between 0 and 1.")
        if not (0 <= self.beta <= math.pi):
            raise ValueError("beta (cone angle in radians) must be between 0 and pi.")

        if self.map.start:
            self.start_node = TreeNode(self.map.start.x, self.map.start.y)
            self.nodes.append(self.start_node)
        else:
            self.start_node = None
            # Consider raising an error if map.start is None, as RRT fundamentally needs it.

        if self.map.goal:
            self.goal_node = TreeNode(self.map.goal.x, self.map.goal.y)
        else:
            self.goal_node = None
            # Goal biasing and cone sampling towards goal will be affected.

    def _get_leaf_nodes(self) -> t.List[TreeNode]:
        """Identifies leaf nodes in the tree.
        A node is considered a leaf if no other node in self.nodes lists it as a parent.
        """
        if not self.nodes:
            return []

        # Collect all nodes that are referenced as a parent by at least one other node.
        nodes_that_are_parents = set()
        for node in self.nodes:
            if node.parent:  # Check if the node itself has a parent
                nodes_that_are_parents.add(node.parent)

        # A leaf node is any node in our tree that is NOT in the set of nodes_that_are_parents.
        leaf_nodes_list = [node for node in self.nodes if node not in nodes_that_are_parents]

        # If leaf_nodes_list is empty, but the tree is not (e.g. only start_node exists),
        # it means start_node (which has no parent itself) is the only leaf.
        # This case should be covered correctly by the list comprehension above.
        # For example, if self.nodes = [start_node (parent=None)],
        # nodes_that_are_parents will be empty.
        # leaf_nodes_list will be [start_node].
        return leaf_nodes_list

    def _get_random_leaf_node(self) -> t.Optional[TreeNode]:
        """Selects a random leaf node from the current set of leaf nodes in the tree."""
        leaf_nodes = self._get_leaf_nodes()
        if not leaf_nodes:
            return None
        return random.choice(leaf_nodes)

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        sample = self.get_random_sample()
        nearest_node = self.get_nearest_node(sample)  # Nearest node in tree to the sample
        new_node = self.extend_toward(nearest_node, sample)

        if new_node and not self.is_collision(new_node.x, new_node.y):
            if not self.is_edge_collision(nearest_node.x, nearest_node.y, new_node.x, new_node.y):
                # Ensure new_node.cost is set correctly.
                # Typically: new_node.cost = nearest_node.cost + self.distance(nearest_node.get_position(), new_node.get_position())
                # This should be handled by TreeNode constructor or add_child, or explicitly here.

                nearest_node.add_child(new_node)  # This method should ideally also set/update new_node's cost
                self.nodes.append(new_node)
                self.steps += 1

                self.rewire_tree(new_node)  # Optimizes parent for new_node

                if self.is_complete():
                    self.reconstruct_path()
                    self.finalize_benchmark()

    def get_random_sample(self) -> t.Tuple[float, float]:
        rand_val = random.random()

        # 1. Goal Biasing
        if self.goal_node and rand_val < BIAS:  # Ensure goal_node exists
            return (self.goal_node.x, self.goal_node.y)

        # 2. Cone Sampling
        # Conditions for attempting cone sampling:
        # - alpha probability is greater than 0.
        # - A goal node exists (for cone orientation).
        # - There are nodes in the tree (so a leaf node can potentially be found).
        can_attempt_cone_sample = (self.alpha > 0 and
                                   self.goal_node is not None and
                                   bool(self.nodes))  # True if self.nodes is not empty

        if can_attempt_cone_sample and rand_val < BIAS + self.alpha:
            # sample_in_cone will internally handle fallbacks if a suitable leaf isn't found
            # or if the chosen leaf is at the goal.
            return self.sample_in_cone()

        # 3. Uniform Sampling (Default)
        else:
            return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def sample_in_cone(self) -> t.Tuple[float, float]:
        """Samples a point within a cone. The cone originates from a randomly selected
           leaf node of the current tree, is oriented towards self.goal_node,
           and the sample is at a fixed distance of self.step_size from the origin node."""

        cone_origin_node = self._get_random_leaf_node()

        # Fallback to uniform sampling if:
        # - No suitable leaf node could be found (e.g., tree is empty, though checked before).
        # - self.goal_node is None (though also checked before calling).
        if not cone_origin_node or not self.goal_node:
            return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

        origin_pos = (cone_origin_node.x, cone_origin_node.y)
        goal_pos = (self.goal_node.x, self.goal_node.y)

        dx = goal_pos[0] - origin_pos[0]
        dy = goal_pos[1] - origin_pos[1]

        # If the chosen cone origin is (practically) at the goal, cone direction is undefined. Fallback.
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:  # Using a small tolerance for float comparison
            return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

        angle_to_goal = math.atan2(dy, dx)  # Centerline of the cone
        sample_radius = self.step_size  # Fixed radius for the sample point

        # Try to generate a sample within map bounds
        for _ in range(100):  # Max attempts to find an in-bounds sample
            random_angle_offset = random.uniform(-self.beta / 2, self.beta / 2)  # Random offset within cone angle
            sample_angle = angle_to_goal + random_angle_offset

            # Calculate sample point coordinates
            sample_x = origin_pos[0] + sample_radius * math.cos(sample_angle)
            sample_y = origin_pos[1] + sample_radius * math.sin(sample_angle)

            # Check if the sample is within map boundaries
            if 0 <= sample_x <= self.map.width and 0 <= sample_y <= self.map.height:
                return (sample_x, sample_y)

        # Fallback if N attempts failed to produce an in-bounds sample
        return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node: TreeNode, to_position: t.Tuple[float, float]) -> TreeNode:
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return TreeNode(to_position[0], to_position[1], parent=from_node)
        else:
            theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return TreeNode(new_x, new_y, parent=from_node)

    def rewire_tree(self, new_node: TreeNode):
        # Radius for finding neighbors. The original RRTStarBiased snippet used step_size * 3.
        # Your RRTStarAlgorithm example used step_size * 6.
        # Keeping it as step_size * 3 for consistency with the snippet being modified.
        radius = self.step_size * 3
        nodes_to_check_as_parent = self.get_near_nodes(new_node, radius)

        for potential_parent_node in nodes_to_check_as_parent:
            if not self.is_edge_collision(potential_parent_node.x, potential_parent_node.y, new_node.x, new_node.y):
                cost_via_potential_parent = potential_parent_node.cost + self.distance(
                    potential_parent_node.get_position(), new_node.get_position())

                if cost_via_potential_parent < new_node.cost:
                    # If TreeNode's parent setter and child management are robust, this is okay.
                    # The chosen _get_leaf_nodes method relies on parent pointers,
                    # which should be correctly updated here.
                    new_node.parent = potential_parent_node
                    new_node.cost = cost_via_potential_parent

        # (A full RRT* rewire would also check if new_node can be a better parent for its neighbors.
        # This part is omitted to match the provided rewire_tree structure.)

    def get_near_nodes(self, node: TreeNode, radius: float) -> t.List[TreeNode]:
        near_nodes = []
        radius_sq = radius ** 2  # Compare squared distances for efficiency
        for potential_node in self.nodes:
            if potential_node == node:  # A node is not its own neighbor
                continue
            dist_sq = (potential_node.x - node.x) ** 2 + (potential_node.y - node.y) ** 2
            if dist_sq < radius_sq:
                near_nodes.append(potential_node)
        return near_nodes
