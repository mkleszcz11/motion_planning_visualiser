import unittest
from core.node import TreeNode

class TestTreeNode(unittest.TestCase):
    def test_create_node(self):
        node = TreeNode(5, 5)
        self.assertEqual(node.x, 5)
        self.assertEqual(node.y, 5)
        self.assertIsNone(node.parent)
        self.assertEqual(node.children, [])

    def test_add_child(self):
        parent = TreeNode(5, 5)
        child = TreeNode(10, 10, parent=parent)
        parent.add_child(child)
        self.assertIn(child, parent.children)
        self.assertEqual(child.parent, parent)

    def test_remove_child(self):
        parent = TreeNode(5, 5)
        child = TreeNode(10, 10, parent)
        parent.add_child(child)
        self.assertIn(child, parent.children)
        parent.remove_child(child)
        self.assertNotIn(child, parent.children)

    def test_calculate_cost(self):
        parent = TreeNode(0, 0)
        child = TreeNode(3, 4, parent)
        parent.add_child(child)
        child.calculate_cost()
        self.assertEqual(child.cost, 5)  # 3-4-5 triangle
