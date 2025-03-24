import unittest
from core.map import Map
from core.node import TreeNode
from benchmarks.benchmark_manager import BenchmarkManager
from algorithms.algorithms_implementations.rrt import RRTAlgorithm

import math

class TestRRT(unittest.TestCase):
    def setUp(self):
        self.map = Map(100, 100)
        self.map.set_start(5, 5)
        self.map.set_goal(95, 95)
        self.benchmark_manager = BenchmarkManager()
        self.rrt = RRTAlgorithm(self.map, benchmark_manager=self.benchmark_manager)

    def test_initialization(self):
        """Test if RRT initializes correctly."""
        self.assertEqual(len(self.rrt.nodes), 1)
        self.assertEqual(self.rrt.nodes[0].x, 5)
        self.assertEqual(self.rrt.nodes[0].y, 5)

    def test_tree_growth(self):
        """Test if tree expands when the algorithm steps."""
        num_initial_nodes = len(self.rrt.nodes)
        self.rrt.step()
        self.assertEqual(len(self.rrt.nodes), num_initial_nodes + 1)

    def test_goal_detection(self):
        """Test if RRT correctly detects the goal."""
        # Map is empty, it should not be complete within 5000 steps
        for _ in range(5000):
            self.rrt.step()
            if self.rrt.is_complete():
                break
        self.assertTrue(self.rrt.is_complete())

    def test_extend_toward(self):
        """Test if new node is correctly extended toward sample."""
        start_node = TreeNode(5, 5)
        sample = (10, 10)
        new_node = self.rrt.extend_toward(start_node, sample)

        # New node should not exceed step size
        distance = math.sqrt((new_node.x - start_node.x)**2 + (new_node.y - start_node.y)**2)
        self.assertLessEqual(distance, self.rrt.step_size)

        # New node should be in the correct direction
        theta = math.atan2(sample[1] - start_node.y, sample[0] - start_node.x)
        expected_x = start_node.x + self.rrt.step_size * math.cos(theta)
        expected_y = start_node.y + self.rrt.step_size * math.sin(theta)

        self.assertAlmostEqual(new_node.x, expected_x, places=1)
        self.assertAlmostEqual(new_node.y, expected_y, places=1)

    def test_path_creation(self):
        """Test if a valid path is created after reaching goal."""
        for _ in range(5000):
            self.rrt.step()
            if self.rrt.is_complete():
                break
        cost = self.rrt.calculate_shortest_path_cost()
        self.assertGreater(cost, math.sqrt(90**2 + 90**2))
