# post_processing/algorithms/recursive_critical_bspline.py

import numpy as np
import math
from typing import List, TYPE_CHECKING, Tuple, Optional

try:
    import scipy.interpolate as si
except ImportError:
    si = None

from core.post_processing_algorithm import PostProcessingAlgorithm
from core.logger import logger
from core.node import Node

if TYPE_CHECKING:
    from core.algorithm import Algorithm

class RecursiveCriticalBSpline(PostProcessingAlgorithm):
    """
    Recursively applies Critical Point B-Spline smoothing up to a maximum depth.
    In each step, it attempts to replace segments between the current path nodes
    with collision-free B-spline sections.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if si is None:
             logger.error("RecursiveCriticalBSpline requires SciPy (`pip install scipy`).")

    def _map_original_index_to_spline_indices(self, original_index: int, num_original_points: int, num_spline_points: int) -> Tuple[int, int]:
        """Maps an original path segment (i to i+1) to start/end indices on the spline."""
        # (Same implementation as in CriticalPointBSpline)
        if num_original_points <= 1: return (0, num_spline_points - 1)
        if num_spline_points <= 1: return (0, 0)
        u_start = original_index / max(1, num_original_points - 1)
        u_end = (original_index + 1) / max(1, num_original_points - 1)
        j_start = int(round(u_start * (num_spline_points - 1)))
        j_end = int(round(u_end * (num_spline_points - 1)))
        j_start = max(0, j_start)
        j_end = max(j_start, min(num_spline_points - 1, j_end))
        if original_index == num_original_points - 2: j_end = num_spline_points - 1
        return j_start, j_end

    def _smooth_one_iteration(self, current_path: List[Node], algorithm_instance: 'Algorithm', num_samples: int, degree: int) -> Optional[List[Node]]:
        """Performs one pass of the critical point B-spline smoothing."""
        if not current_path or len(current_path) < 2: return current_path # Cannot process

        logger.debug(f"  Smoothing iteration input: {len(current_path)} nodes.")

        # --- 1. Generate Full B-Spline for this iteration ---
        full_spline_nodes = []
        num_current_nodes = len(current_path)
        try:
            control_vertices = np.array([[n.x, n.y] for n in current_path])
            count = control_vertices.shape[0]
            safe_degree = np.clip(degree, 1, max(1, count - 1))
            # Cannot generate spline if not enough points for the degree
            if count <= safe_degree:
                logger.debug(f"  Not enough points ({count}) for degree {safe_degree} in this iteration. Returning current path.")
                return current_path # Return current path if spline cannot be generated

            denominator = count - safe_degree
            # This check should be redundant due to the one above, but belt-and-suspenders
            if denominator <= 0: logger.error(f"Invalid state: count={count}, degree={safe_degree}"); return None # Indicate failure

            kv = np.array([0]*safe_degree + list(range(count-safe_degree+1)) + [count-safe_degree]*safe_degree, dtype='float')
            kv /= denominator
            # Use the same total number of samples for consistency across iterations
            u = np.linspace(0.0, 1.0, num_samples)
            tck = (kv, control_vertices.T, safe_degree)
            smoothed_points_np = np.array(si.splev(u, tck)).T
            for point in smoothed_points_np: full_spline_nodes.append(Node(point[0], point[1]))
        except Exception as e:
             logger.error(f"  Error during B-spline calculation in iteration: {e}", exc_info=True)
             return None # Indicate failure
        if not full_spline_nodes:
             logger.error("  B-spline calculation resulted in empty path in iteration.")
             return None # Indicate failure

        num_spline_points = len(full_spline_nodes)
        logger.debug(f"  Generated {num_spline_points} spline points for iteration.")

        # --- 2. Reconstruct Path Segment by Segment ---
        iteration_output_path = [current_path[0]] # Start with first node of *this iteration's input*
        collision_reverts = 0

        for i in range(num_current_nodes - 1): # Iterate through segments of current_path
            original_node_start = current_path[i]
            original_node_end = current_path[i+1]
            j_start, j_end = self._map_original_index_to_spline_indices(i, num_current_nodes, num_spline_points)

            spline_segment_valid = True
            if j_end > j_start:
                for j in range(j_start, j_end):
                    spline_seg_start = full_spline_nodes[j]
                    spline_seg_end_idx = min(j + 1, num_spline_points - 1)
                    spline_seg_end = full_spline_nodes[spline_seg_end_idx]
                    if spline_seg_start.x == spline_seg_end.x and spline_seg_start.y == spline_seg_end.y: continue

                    try:
                        has_collision = algorithm_instance.is_edge_collision(spline_seg_start.x, spline_seg_start.y, spline_seg_end.x, spline_seg_end.y)
                        if has_collision: spline_segment_valid = False; break
                    except Exception as e: logger.error(f"  Collision check error: {e}"); return None # Indicate failure

            if spline_segment_valid and j_end > j_start:
                # Append spline points
                for k in range(j_start + 1, j_end + 1):
                    k_safe = min(k, num_spline_points - 1)
                    node_to_add = full_spline_nodes[k_safe]
                    if not iteration_output_path or (iteration_output_path[-1].x != node_to_add.x or iteration_output_path[-1].y != node_to_add.y):
                        iteration_output_path.append(node_to_add)
            else:
                # Append original end node
                collision_reverts += 1
                node_to_add = original_node_end
                if not iteration_output_path or (iteration_output_path[-1].x != node_to_add.x or iteration_output_path[-1].y != node_to_add.y):
                    iteration_output_path.append(node_to_add)

        logger.debug(f"  Iteration finished. Output: {len(iteration_output_path)} nodes. Reverts: {collision_reverts}")
        return iteration_output_path


    def _recursive_smooth(self, current_path: List[Node], algorithm_instance: 'Algorithm', num_samples: int, degree: int, current_depth: int, max_depth: int) -> List[Node]:
        """Recursive helper function."""

        # --- Base Cases ---
        if current_depth >= max_depth:
            logger.info(f"Reached max recursion depth ({max_depth}). Returning path from depth {current_depth-1}.")
            return current_path
        if not current_path or len(current_path) < max(2, degree + 1): # Need enough points for spline
            logger.info(f"Path too short ({len(current_path)}) for further smoothing at depth {current_depth}. Returning.")
            return current_path

        logger.info(f"--- Recursive Smoothing: Depth {current_depth}/{max_depth} ---")

        # --- Perform One Smoothing Iteration ---
        smoothed_path = self._smooth_one_iteration(current_path, algorithm_instance, num_samples, degree)

        # --- Handle Smoothing Failure ---
        if smoothed_path is None:
            logger.error(f"Smoothing iteration failed at depth {current_depth}. Returning previous path.")
            return current_path # Return the last valid path

        # --- Check for Convergence (Simple Node Count Check) ---
        # A more robust check might compare actual node positions or total length
        if len(smoothed_path) == len(current_path):
             # Add a check to see if nodes are actually the same, preventing infinite loops if reverts happen
             is_identical = True
             for n1, n2 in zip(current_path, smoothed_path):
                 if n1.x != n2.x or n1.y != n2.y:
                     is_identical = False
                     break
             if is_identical:
                 logger.info(f"Path converged (identical nodes) at depth {current_depth}. Stopping recursion.")
                 return smoothed_path # No change, stop recursion

        # --- Recursive Call ---
        return self._recursive_smooth(smoothed_path, algorithm_instance, num_samples, degree, current_depth + 1, max_depth)


    def process(self, original_path: List[Node], algorithm_instance: 'Algorithm', **kwargs) -> List[Node]:
        """
        Applies recursive B-spline smoothing.

        Args:
            original_path: The input path (list of Node objects).
            algorithm_instance: The planning algorithm instance for collision checks.
            **kwargs: Should contain 'num_samples' (int) for total spline points per iteration,
                      optionally 'degree' (int, default 3),
                      optionally 'max_depth' (int, default 3).

        Returns:
            The recursively smoothed hybrid path.
        """
        if si is None: logger.error("Cannot smooth, SciPy not installed."); return original_path

        # Default recursion depth
        max_depth = kwargs.get('max_depth', 3)
        # Get other parameters needed by the recursive helper
        default_samples = max(50, len(original_path) * 5)
        num_samples = kwargs.get('num_samples', default_samples)
        degree = kwargs.get('degree', 3)

        logger.info(f"Starting RecursiveCriticalBSpline (max_depth={max_depth}, deg={degree}, {num_samples} samples)...")

        # Initial checks
        if not original_path or len(original_path) < 2: return original_path
        if not algorithm_instance: logger.error("Algorithm instance required."); return original_path
        if num_samples < 2: logger.warning("'num_samples' < 2, using 2."); num_samples = 2

        # Start the recursion
        final_path = self._recursive_smooth(
            current_path=original_path,
            algorithm_instance=algorithm_instance,
            num_samples=num_samples,
            degree=degree,
            current_depth=0,
            max_depth=max_depth
        )

        logger.info(f"RecursiveCriticalBSpline finished. Final path has {len(final_path)} points.")
        return final_path