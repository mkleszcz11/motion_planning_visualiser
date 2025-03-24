
from core.algorithm import Algorithm
from core.map import Map

class PRMAlgorithm(Algorithm):
    """Probabilistic Road Maps algorithm implementation.
    
    IMPORTANT: PRM is based on graph, not tree. 
    
    It works in the following way:
    1. Generate a set of random samples (with validation).
    2. Find neighbors for each sample (with validation -
        neighbors are connected with edges).
    3. Find the shortest path between the start and goal nodes.
        For that use Dijkstra or A* algorithm.
    """
    pass