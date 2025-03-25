import unittest
from core.map import Map
from core.node import TreeNode
from benchmarks.benchmark_manager import BenchmarkManager
from benchmarks.benchmark_result import BenchmarkResult
from core.algorithm import Algorithm

import math

class MockAlgorithm(Algorithm):
    """
    Since Algorithm is an abstract class, we need to create a mock subclass to test it.
    """
    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        # Mock logic for testing
        if not self.nodes:
            self.clear_nodes()

class TestAlgorithm(unittest.TestCase):
    def setUp(self):
        self.map = Map(100, 100)
        self.map.set_start(5, 5)
        self.map.set_goal(95, 95)
        self.benchmark_manager = BenchmarkManager()
        self.algorithm = MockAlgorithm(self.map, benchmark_manager=self.benchmark_manager)

    def test_initialization(self):
        self.assertEqual(len(self.algorithm.nodes), 0)

    def test_clear_nodes(self):
        self.algorithm.clear_nodes()
        self.assertEqual(len(self.algorithm.nodes), 1)
        # Algorithm should have only one node, the start node
        self.assertEqual(self.algorithm.nodes[0].x, self.map.start.x)
        self.assertEqual(self.algorithm.nodes[0].y, self.map.start.y)

    def test_get_nearest_node(self):
        self.algorithm.clear_nodes()
        position_to_validate = (10, 10)
        node = self.algorithm.get_nearest_node(position_to_validate)
        self.assertEqual(node.x, 5)
        self.assertEqual(node.y, 5)
        
        # Add a new node close to the goal
        some_new_node = TreeNode(14, 13)
        self.algorithm.nodes.append(some_new_node)
        nearest_node = self.algorithm.get_nearest_node(position_to_validate)
        self.assertEqual(nearest_node.x, 14)
        self.assertEqual(nearest_node.y, 13)
        self.assertEqual(nearest_node, some_new_node)

    def test_collision(self):
        self.map.add_obstacle(20, 20, 5, 5)
        self.assertTrue(self.algorithm.is_collision(22, 22))
        self.assertTrue(self.algorithm.is_collision(20, 20))
        self.assertFalse(self.algorithm.is_collision(30, 30))
        self.assertTrue(self.algorithm.is_collision(-10, -10))
        self.assertFalse(self.algorithm.is_collision(0, 0))

    def test_edge_collision(self):
        self.map.add_obstacle(20, 20, 5, 5)
        # Edge touches corner of obstacle
        self.assertTrue(self.algorithm.is_edge_collision(19, 19, 23, 23))
        # Edge goes through obstacle
        self.assertTrue(self.algorithm.is_edge_collision(18, 22, 27, 22))
        # Edge goes through obstacle edge
        self.assertTrue(self.algorithm.is_edge_collision(18, 20, 27, 20))
        # Edge starts inside obstacle and goes through it (should never happen)
        self.assertTrue(self.algorithm.is_edge_collision(22, 22, 27, 22))
        # Edge starts outside obstacle and finishes inside it (should never happen)
        self.assertTrue(self.algorithm.is_edge_collision(18, 22, 22, 22))
        # Edge starts and finishes outside obstacle
        self.assertFalse(self.algorithm.is_edge_collision(10, 10, 30, 22))

    def test_distance(self):
        dist = self.algorithm.distance((0, 0), (3, 4))
        self.assertEqual(dist, 5)

    def test_reconstruct_path_and_calculate_cost(self):
        # start
        #    \
        #   node_1 - node_2  - node_3
        #         \         \
        #           node_5   node_4 - node_6
        #                                   \
        #                                    goal
        ########################################
        self.algorithm.clear_nodes()
        
        start_node = TreeNode(self.map.start.x, self.map.start.y)
        goal_node = TreeNode(self.map.goal.x, self.map.goal.y)
        self.algorithm.nodes.append(start_node)
        self.algorithm.start_node = start_node
        self.algorithm.goal_node = goal_node
        
        # I should not be able to calculate the cost if path is not set.
        self.assertEqual(self.algorithm.calculate_shortest_path_cost(), float('inf'))

        node_1 = TreeNode(10, 10, parent=self.algorithm.start_node)
        node_2 = TreeNode(20, 10, parent=node_1)
        node_3 = TreeNode(30, 10, parent=node_2)
        node_4 = TreeNode(30, 20, parent=node_2)
        node_5 = TreeNode(20, 20, parent=node_1)
        node_6 = TreeNode(40, 20, parent=node_4)
        self.algorithm.goal_node.parent = node_6
        self.algorithm.nodes.append(node_1)
        self.algorithm.nodes.append(node_2)
        self.algorithm.nodes.append(node_3)
        self.algorithm.nodes.append(node_4)
        self.algorithm.nodes.append(node_5)
        self.algorithm.nodes.append(node_6)

        self.algorithm.reconstruct_path()
        path = self.algorithm.shortest_path

        expected_path = [self.algorithm.start_node, node_1, node_2, node_4, node_6, self.algorithm.goal_node]
        self.assertEqual(path, expected_path)
        
        # Calculate the cost of the path
        cost = self.algorithm.calculate_shortest_path_cost()
        
        # I know this is not elegant, but that's the best I can do right now.
        start_node_1_cost = math.sqrt((node_1.x - start_node.x)**2 + (node_1.y - start_node.y)**2)
        node_1_node_2_cost = math.sqrt((node_2.x - node_1.x)**2 + (node_2.y - node_1.y)**2)
        node_2_node_4_cost = math.sqrt((node_4.x - node_2.x)**2 + (node_4.y - node_2.y)**2)
        node_4_node_6_cost = math.sqrt((node_6.x - node_4.x)**2 + (node_6.y - node_4.y)**2)
        node_6_goal_node_cost = math.sqrt((self.algorithm.goal_node.x - node_6.x)**2 + (self.algorithm.goal_node.y - node_6.y)**2)

        expected_cost = start_node_1_cost + node_1_node_2_cost + node_2_node_4_cost + node_4_node_6_cost + node_6_goal_node_cost

        self.assertAlmostEqual(cost, expected_cost, places=4)

    def test_benchmarking(self):
        # TODO
        pass

if __name__ == "__main__":
    unittest.main()
