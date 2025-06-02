# post_processing/algorithms/critical_point_bspline.py

import numpy as np
import math
from typing import List, TYPE_CHECKING, Tuple

try:
    import scipy.interpolate as si
except ImportError:
    si = None

from core.post_processing_algorithm import PostProcessingAlgorithm
from core.logger import logger
from core.node import Node

if TYPE_CHECKING:
    from core.algorithm import Algorithm

class CriticalPointBSpline(PostProcessingAlgorithm):
    """
    Post-processing algorithm that smooths a path using B-splines between
    original path nodes (critical points). If the spline section corresponding
    to an original segment collides, the original segment is used instead.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if si is None:
             logger.error("CriticalPointBSpline requires SciPy (`pip install scipy`).")

    def _map_original_index_to_spline_indices(self, original_index: int, num_original_points: int, num_spline_points: int) -> Tuple[int, int]:
        """Maps an original path segment (i to i+1) to start/end indices on the spline."""
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

    def process(self, original_path: List[Node], algorithm_instance: 'Algorithm', **kwargs) -> List[Node]:
        """Applies B-spline smoothing segmentally."""
        if si is None: logger.error("Cannot smooth, SciPy not installed."); return original_path
        if not original_path or len(original_path) < 2: logger.debug("Path too short."); return original_path
        if not algorithm_instance: logger.error("Algorithm instance required."); return original_path

        # Increase default samples for better collision checking approximation
        default_samples = max(50, len(original_path) * 5) # Example heuristic
        num_samples = kwargs.get('num_samples', default_samples)
        if num_samples == default_samples: logger.info(f"Using default num_samples: {num_samples}")
        else: logger.info(f"Using provided num_samples: {num_samples}")

        degree = kwargs.get('degree', 3) # Cubic default

        if num_samples < 2: logger.warning("'num_samples' < 2, using 2."); num_samples = 2

        logger.info(f"Running CriticalPointBSpline (deg={degree}, {num_samples} total samples)...")

        # --- 1. Generate Full B-Spline Points ---
        full_spline_nodes = []
        num_original = len(original_path)
        try:
            control_vertices = np.array([[n.x, n.y] for n in original_path])
            count = control_vertices.shape[0]
            safe_degree = np.clip(degree, 1, max(1, count - 1))
            if safe_degree != degree: logger.warning(f"Degree clipped: {degree} -> {safe_degree} (CPs={count}).")
            if count <= safe_degree: logger.warning(f"Not enough CPs ({count}) for degree {safe_degree}. Using original."); return original_path
            denominator = count - safe_degree
            if denominator <= 0: logger.error(f"Invalid state: count={count}, degree={safe_degree}"); return original_path
            kv = np.array([0]*safe_degree + list(range(count-safe_degree+1)) + [count-safe_degree]*safe_degree, dtype='float')
            kv /= denominator
            u = np.linspace(0.0, 1.0, num_samples)
            tck = (kv, control_vertices.T, safe_degree)
            smoothed_points_np = np.array(si.splev(u, tck)).T
            for point in smoothed_points_np: full_spline_nodes.append(Node(point[0], point[1]))
        except Exception as e: logger.error(f"Error during B-spline calculation: {e}", exc_info=True); return original_path
        if not full_spline_nodes: logger.error("B-spline calculation resulted in empty path."); return original_path

        logger.info(f"Generated {len(full_spline_nodes)} spline points. Reconstructing segmented path...")
        num_spline_points = len(full_spline_nodes)

        # --- 2. Reconstruct Path Segment by Segment ---
        final_path = [original_path[0]]
        for i in range(num_original - 1): # Iterate through original segments 0..N-2
            original_node_start = original_path[i]; original_node_end = original_path[i+1]
            j_start, j_end = self._map_original_index_to_spline_indices(i, num_original, num_spline_points)

            spline_segment_valid = True
            if j_end > j_start:
                for j in range(j_start, j_end): # Check spline segments j to j+1, where j goes up to j_end-1
                    spline_seg_start = full_spline_nodes[j]
                    spline_seg_end_idx = min(j + 1, num_spline_points - 1)
                    spline_seg_end = full_spline_nodes[spline_seg_end_idx]

                    if spline_seg_start.x == spline_seg_end.x and spline_seg_start.y == spline_seg_end.y: continue # Skip zero-length

                    # Add specific logging for the very last check
                    is_last_segment_check = (i == num_original - 2) and (j == j_end - 1)
                    if is_last_segment_check:
                         logger.debug(f"Checking FINAL mapped spline segment: {j}->{spline_seg_end_idx} "
                                      f"({spline_seg_start.x:.1f},{spline_seg_start.y:.1f}) -> "
                                      f"({spline_seg_end.x:.1f},{spline_seg_end.y:.1f})")

                    try:
                        has_collision = algorithm_instance.is_edge_collision(spline_seg_start.x, spline_seg_start.y, spline_seg_end.x, spline_seg_end.y)
                        if has_collision:
                            if is_last_segment_check: logger.warning("FINAL mapped spline segment COLLIDED!")
                            logger.debug(f"Collision in spline range [{j_start},{j_end}] for orig seg {i}. Seg {j}->{spline_seg_end_idx} failed.")
                            spline_segment_valid = False; break
                    except AttributeError: logger.error("Collision checker missing."); return original_path
                    except Exception as e: logger.error(f"Collision check error: {e}"); return original_path

            # Build final path based on validity
            if spline_segment_valid and j_end > j_start:
                # Append valid spline points
                for k in range(j_start + 1, j_end + 1):
                    k_safe = min(k, num_spline_points - 1)
                    node_to_add = full_spline_nodes[k_safe]
                    if not final_path or (final_path[-1].x != node_to_add.x or final_path[-1].y != node_to_add.y): final_path.append(node_to_add)
            else:
                # Append original end node
                node_to_add = original_node_end
                if not final_path or (final_path[-1].x != node_to_add.x or final_path[-1].y != node_to_add.y): final_path.append(node_to_add)

        logger.info(f"CriticalPointBSpline finished. Final path has {len(final_path)} points.")
        return final_path