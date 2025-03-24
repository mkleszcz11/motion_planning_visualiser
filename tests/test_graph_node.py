import unittest
from core.node import GraphNode

class TestGraphNode(unittest.TestCase):
    def test_create_node(self):
        node = GraphNode(5, 5)
        self.assertEqual(node.x, 5)
        self.assertEqual(node.y, 5)
        self.assertEqual(node.edges, {})

    def test_add_edge(self):
        node1 = GraphNode(5, 5)
        node2 = GraphNode(10, 10)
        node1.add_edge(node2, 5)
        self.assertIn(node2, node1.edges)
        self.assertIn(node1, node2.edges)
        self.assertEqual(node1.edges[node2], 5)
        self.assertEqual(node2.edges[node1], 5)

    def test_remove_edge(self):
        node1 = GraphNode(5, 5)
        node2 = GraphNode(10, 10)
        node1.add_edge(node2, 5)
        node1.remove_edge(node2)
        self.assertNotIn(node2, node1.edges)
        self.assertNotIn(node1, node2.edges)
