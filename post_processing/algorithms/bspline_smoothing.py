# post_processing/algorithms/bspline_smoothing.py

import numpy as np
import math # Needed for collision checking helper potentially, though using algo's method
from typing import List, TYPE_CHECKING

# Ensure scipy is installed: pip install scipy
try:
    import scipy.interpolate as si
except ImportError:
    si = None # Handle missing scipy gracefully

from core.post_processing_algorithm import PostProcessingAlgorithm
from core.logger import logger
from core.node import Node # Need Node to create the output path

# Use forward reference for Algorithm
if TYPE_CHECKING:
    from core.algorithm import Algorithm

class BSplineSmoothing(PostProcessingAlgorithm):
    """
    Post-processing algorithm that smooths a path using B-splines.
    Includes collision checking for the generated path segments.
    If any segment of the smoothed path collides, the original path is returned.
    """

    def __init__(self, **kwargs):
        """
        Initializes the BSplineSmoothing algorithm.
        """
        super().__init__(**kwargs)
        if si is None:
             logger.error("BSplineSmoothing requires SciPy. Please install it (`pip install scipy`).")

    def _check_spline_collisions(self, smoothed_nodes: List[Node], algorithm_instance: 'Algorithm') -> bool:
        """
        Checks if any linear segment connecting consecutive nodes in the smoothed path collides.

        Args:
            smoothed_nodes: List of Node objects representing the points on the smoothed spline.
            algorithm_instance: The planning algorithm instance for collision checks.

        Returns:
            True if a collision is detected, False otherwise.
        """
        if not algorithm_instance:
            logger.error("Collision check requires a valid algorithm instance.")
            return True # Assume collision if checker is unavailable

        if len(smoothed_nodes) < 2:
            return False # No segments to check

        logger.debug(f"Checking collisions for {len(smoothed_nodes)-1} smoothed path segments...")
        for i in range(len(smoothed_nodes) - 1):
            node1 = smoothed_nodes[i]
            node2 = smoothed_nodes[i+1]

            try:
                # Use the planning algorithm's edge collision checker
                has_collision = algorithm_instance.is_edge_collision(node1.x, node1.y, node2.x, node2.y)
                if has_collision:
                    logger.warning(f"Collision detected in smoothed path segment: ({node1.x:.1f},{node1.y:.1f}) -> ({node2.x:.1f},{node2.y:.1f})")
                    return True # Collision found
            except AttributeError:
                 logger.error(f"BSplineSmoothing: Algorithm instance ({algorithm_instance.__class__.__name__}) lacks 'is_edge_collision' method.")
                 return True # Treat as collision if method is missing
            except Exception as e:
                 logger.error(f"BSplineSmoothing: Error during collision check for segment {i}: {e}", exc_info=True)
                 return True # Treat as collision on error

        logger.debug("Smoothed path segments appear collision-free.")
        return False # No collisions found

    def process(self, original_path: List[Node], algorithm_instance: 'Algorithm', **kwargs) -> List[Node]:
        """
        Applies B-spline smoothing and checks for collisions.

        Args:
            original_path: The input path (list of Node objects).
            algorithm_instance: The planning algorithm instance for collision checks.
            **kwargs: Should contain 'num_samples' (int) for the number of points on the output spline
                      and optionally 'degree' (int, default 5).

        Returns:
            The smoothed, collision-checked path as a list of new Node objects,
            or the original path if smoothing fails or the result collides.
        """
        if si is None:
            logger.error("Cannot perform B-Spline smoothing, SciPy is not installed.")
            return original_path

        if not original_path or len(original_path) < 2:
            logger.debug("Path too short for B-spline smoothing, returning original.")
            return original_path

        num_samples = kwargs.get('num_samples')
        degree = kwargs.get('degree', 5) # Default to Quintic

        if num_samples is None:
            logger.error("BSplineSmoothing: 'num_samples' parameter is required.")
            return original_path
        if num_samples < 2:
             logger.warning("BSplineSmoothing: 'num_samples' must be at least 2. Using 2.")
             num_samples = 2

        logger.info(f"Running BSplineSmoothing (degree={degree}) with {num_samples} samples...")

        # --- Generate B-Spline Points ---
        smoothed_path_nodes = [] # Initialize empty list
        try:
            control_vertices = np.array([[node.x, node.y] for node in original_path])
            count = control_vertices.shape[0]
            safe_degree = np.clip(degree, 1, max(1, count - 1))
            if safe_degree != degree: logger.warning(f"B-spline degree clipped from {degree} to {safe_degree} due to control points ({count}).")

            if count <= safe_degree:
                 logger.warning(f"Not enough control points ({count}) for B-spline degree {safe_degree}. Returning original path.")
                 return original_path

            # Using normalized knot vector [0, 1] range
            kv = np.array([0]*safe_degree + list(range(count-safe_degree+1)) + [count-safe_degree]*safe_degree, dtype='float')
            # Check for zero denominator if count == safe_degree (should be prevented by check above)
            denominator = count - safe_degree
            if denominator <= 0:
                 logger.error(f"Invalid state for knot vector calculation: count={count}, safe_degree={safe_degree}")
                 return original_path
            kv /= denominator

            u = np.linspace(0.0, 1.0, num_samples)
            tck = (kv, control_vertices.T, safe_degree)
            smoothed_points_np = np.array(si.splev(u, tck)).T

            # Convert result back to Node list
            for point in smoothed_points_np:
                smoothed_path_nodes.append(Node(point[0], point[1]))

        except Exception as e:
             logger.error(f"Error during B-spline calculation: {e}", exc_info=True)
             return original_path # Return original path on spline generation error

        if not smoothed_path_nodes:
             logger.error("B-spline calculation resulted in an empty path.")
             return original_path

        logger.info(f"Generated {len(smoothed_path_nodes)} points for smoothed path. Now checking collisions.")

        # --- Collision Check the Smoothed Path Segments ---
        collision_detected = self._check_spline_collisions(smoothed_path_nodes, algorithm_instance)

        if collision_detected:
            logger.warning("Collision detected in the generated B-spline path. Returning the original path.")
            return original_path
        else:
            logger.info("BSplineSmoothing finished successfully. Path is smoothed and collision-free (based on segment checks).")
            return smoothed_path_nodes