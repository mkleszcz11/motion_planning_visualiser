import random
import math
import typing as t
from core.algorithm import Algorithm
from core.node import TreeNode

from core.logger import logger

class RRTConnectAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map=map, benchmark_manager=benchmark_manager)

        # Initialize two trees (start and goal trees)
        self.tree_start = []  # Start tree
        self.tree_goal = []  # Goal tree
        self.start_tree_as_active = True # Flag to indicate which tree is active

        # Path reconstruction and tree connection info
        self.path = []

        # Ensure the map contains valid start and goal points
        if not map.start or not map.goal:
            raise ValueError("Start and goal points are required.")

        # Add start and goal nodes to their respective trees
        start_node = TreeNode(map.start.x, map.start.y)
        goal_node = TreeNode(map.goal.x, map.goal.y)
        self.tree_start.append(start_node)
        self.tree_goal.append(goal_node)
        self.start_node = start_node
        self.goal_node = goal_node
        self.nodes.append(start_node)

        # Flag to indicate if the trees are connected
        self.connected = False

    def step(self):
        """
        1. Start with start_tree
        2. Sample a random point
        3. Find the nearest node in the tree
        4. Extend toward the sample
        5. Check for collisions
        6. If no collision, add the new node
        7. Connect this node to the tree (start_tree)
        8. Check if I can see the closest node in the opposite tree, if so, connect the two trees 
        OLD -> 9. Check if this node is close enough to any of the nodes in the goal_tree
        9. If yes, redirect the goal_tree and connect the two trees
        10. If there were no issues, continue growing the same tree, swap otherwise
        """
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        # Sample a random point in the search space
        random_sample = self.get_random_sample()

        # Start with the start tree as the active tree
        if self.start_tree_as_active:
            active_tree = self.tree_start
            passive_tree = self.tree_goal
        else:
            active_tree = self.tree_goal
            passive_tree = self.tree_start    

        # Perform step for the active tree: Extend toward the sampled point
        nearest_active = self.get_nearest_node_in_tree(random_sample, active_tree)
        new_node_active = self.extend_toward(nearest_active, random_sample)

        # Check for collisions to add the new node
        if new_node_active:
            if not self.is_collision(new_node_active.x, new_node_active.y) and\
               not self.is_edge_collision(nearest_active.x, nearest_active.y, new_node_active.x, new_node_active.y):
                    nearest_active.add_child(new_node_active)
                    active_tree.append(new_node_active)
                    self.nodes.append(new_node_active)
                    self.steps += 1

                    #can_connect, node_to_which_i_can_connect = self.are_trees_close_enough(new_node = new_node_active, passive_tree = passive_tree)
                    can_connect, node_to_which_i_can_connect = self.are_trees_connectable(new_node_active, passive_tree)

                    if can_connect:
                        self.redirect_goal_tree_and_connect(new_node_active, node_to_which_i_can_connect)
                        # self.reverse_tree_path(self.tree_start, self.tree_goal, new_node_active, self.get_nearest_node_in_tree(new_node_active.get_position(), passive_tree))
                        self.reconstruct_path()
                        self.finalize_benchmark()
            else:
                # Swap active and passive trees for the next iteration
                self.start_tree_as_active = not self.start_tree_as_active
        else:
            logger.error("Failed to extend the tree toward the random sample.")

    def get_random_sample(self):
        """Generate a random sample within the map boundaries."""
        return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node, to_position):
        """Extend from a node toward a target position, respecting the step size."""
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return TreeNode(to_position[0], to_position[1], from_node)  # Create a new node at the target

        # Calculate direction and generate a new node along the line
        theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)
        return TreeNode(new_x, new_y, from_node)

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
    
    def redirect_goal_tree_and_connect(self, new_node, node_to_which_i_can_connect):
        """
        Connect two trees when they are close enough.

        1. Flip the goal tree - Swap parent and child relationship.
        2. Connect the two trees.
        """
        if node_to_which_i_can_connect is None:
            return

        # Step 1: Reverse parent-child relationship along the connection path in the goal tree
        node_in_goal_tree = None
        node_in_start_tree = None
        if self.start_tree_as_active:
            node_in_goal_tree = node_to_which_i_can_connect
            node_in_start_tree = new_node
        else:
            node_in_goal_tree = new_node
            node_in_start_tree = node_to_which_i_can_connect

        self.reverse_goal_tree(node_in_goal_tree)

        # Step 2: Connect the new node to the reversed goal tree
        node_in_goal_tree.parent = node_in_start_tree
        node_in_start_tree.add_child(node_in_goal_tree)

    def reverse_goal_tree(self, node):
        """
        Reverse the parent-child relationship along the path from the connected node.

        Args:
            node (TreeNode): Node where the connection occurred.
        """
        parent = node.parent
        while parent is not None:
            # Reverse parent-child relationship
            parent.remove_child(node)
            node.add_child(parent)

            # Reverse references
            grandparent = parent.parent
            parent.parent = node
            node = parent
            parent = grandparent

    def are_trees_close_enough(self, new_node, passive_tree) -> t.Tuple[(bool, TreeNode|None)]:
        """Return True if the new node is close enough to any node in the passive tree. (and there is no collision)"""
        for node in passive_tree:
            if self.distance(node.get_position(), new_node.get_position()) < self.step_size:
                if not self.is_edge_collision(node.x, node.y, new_node.x, new_node.y):
                    self.connected = True
                    return True, node
        return False, None
    
    def are_trees_connectable(self, new_node, passive_tree) -> t.Tuple[(bool, TreeNode|None)]:
        """Return True if the new node can be connected to the closest node in the passive tree. I do not care about the distance."""
        closest_node = self.get_nearest_node_in_tree(new_node.get_position(), passive_tree)
        if not closest_node:
            return False, None

        if not self.is_edge_collision(closest_node.x, closest_node.y, new_node.x, new_node.y):
            self.connected = True
            return True, closest_node
        return False, None

    def is_complete(self):
        """Return True if the trees are connected."""
        return self.connected

    def clear_nodes(self):
        self.nodes = []
        self.tree_start = []
        self.tree_goal = []
        self.steps = 0
        self.connected = False
        self.start_tree_as_active = True
        self.path = []
        self.start_node = None

        if self.map.start:
            self.start_node = TreeNode(self.map.start.x, self.map.start.y)
        
        if self.map.goal:
            self.goal_node = TreeNode(self.map.goal.x, self.map.goal.y)
        
        self.tree_start.append(self.start_node)
        self.tree_goal.append(self.goal_node)

        self.nodes.append(self.start_node)
