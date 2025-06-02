# post_processing/algorithms/path_shortening.py

import random
import math # Needed for hypot (distance calculation)
from typing import List, TYPE_CHECKING

from core.post_processing_algorithm import PostProcessingAlgorithm
from core.logger import logger
# Import Node for type hinting within the calculation method
from core.node import Node

# Use forward reference for Algorithm
if TYPE_CHECKING:
    from core.algorithm import Algorithm
    # from core.node import Node # Already imported above

class PathShortening(PostProcessingAlgorithm):
    """
    Post-processing algorithm that attempts to shorten a path by randomly
    connecting non-adjacent nodes if the direct path is collision-free.
    Logs the change in Euclidean path length.
    """

    def __init__(self, **kwargs):
        """Initializes the PathShortening algorithm."""
        super().__init__(**kwargs)

    def _calculate_path_length(self, path: List[Node]) -> float:
        """Calculates the total Euclidean distance of a path."""
        if not path or len(path) < 2:
            return 0.0

        total_length = 0.0
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i+1]
            # Ensure nodes are valid and have coordinates
            if p1 and p2 and hasattr(p1, 'x') and hasattr(p1, 'y') and hasattr(p2, 'x') and hasattr(p2, 'y'):
                # Use math.hypot for distance calculation
                total_length += math.hypot(p2.x - p1.x, p2.y - p1.y)
            else:
                logger.warning("Skipping segment in length calculation due to invalid node.")
        return total_length

    def process(self, original_path: List[Node], algorithm_instance: 'Algorithm', **kwargs) -> List[Node]:
        """
        Applies the random shortcut path shortening method.

        Args:
            original_path: The input path (list of Node objects).
            algorithm_instance: The planning algorithm instance for collision checks.
            **kwargs: Must contain 'iterations' (int) for the number of shortening attempts.

        Returns:
            The potentially shortened path (list of Node objects).
        """
        if not original_path or len(original_path) < 2: # Need at least 2 nodes for length/processing
            logger.debug("Path too short for processing, returning original.")
            return original_path

        if not algorithm_instance:
            logger.error("PathShortening: Algorithm instance is required for collision checking.")
            return original_path

        iterations = kwargs.get('iterations')
        if iterations is None:
            logger.error("PathShortening: 'iterations' parameter is required.")
            return original_path

        logger.info(f"Running PathShortening for {iterations} iterations...")

        # Calculate initial Euclidean length for logging
        original_euclidean_length = self._calculate_path_length(original_path)
        logger.info(f"Initial path: {len(original_path)} nodes, Length: {original_euclidean_length:.2f}")

        # Work on a copy
        current_path = list(original_path)

        # --- Shortening Loop ---
        for i in range(iterations):
            if len(current_path) < 3: break # Cannot shorten further

            idx1 = random.randint(0, len(current_path) - 2)
            idx2 = random.randint(idx1 + 1, len(current_path) - 1)

            if idx2 == idx1 + 1: continue # Already adjacent

            node1 = current_path[idx1]
            node2 = current_path[idx2]

            is_clear = False
            try:
                # Use algorithm's collision check
                has_collision = algorithm_instance.is_edge_collision(node1.x, node1.y, node2.x, node2.y)
                is_clear = not has_collision
            except AttributeError:
                 logger.error(f"PathShortening: Algorithm instance ({algorithm_instance.__class__.__name__}) lacks 'is_edge_collision' method.")
                 return current_path # Stop processing
            except Exception as e:
                 logger.error(f"PathShortening: Error during collision check: {e}", exc_info=True)
                 return current_path # Stop processing

            if is_clear:
                # Apply shortcut: Remove nodes between idx1 and idx2 (exclusive of idx2)
                current_path = current_path[:idx1+1] + current_path[idx2:]
                # Note: No need to recalculate length inside the loop unless debugging detailed changes

        # --- Final Length Calculation and Logging ---
        final_euclidean_length = self._calculate_path_length(current_path)
        final_node_count = len(current_path)

        # Updated log message using calculated Euclidean lengths
        logger.info(f"PathShortening finished. Nodes: {len(original_path)} -> {final_node_count}. Length: {original_euclidean_length:.2f} -> {final_euclidean_length:.2f}")

        return current_path