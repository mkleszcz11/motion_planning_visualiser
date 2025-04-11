# visualisation_panel.py

import sys

from PyQt5.QtWidgets import (QWidget, QPushButton, QGraphicsScene, QGraphicsView,
                             QGraphicsRectItem, QGraphicsEllipseItem, QSpinBox, QLabel, QVBoxLayout,
                             QGraphicsLineItem, QComboBox, QDoubleSpinBox)
# Import pyqtSignal for creating signals
from PyQt5.QtCore import Qt, QRectF, QLineF, QTimer, pyqtSignal
from PyQt5.QtGui import QPen, QColor

from core.map import Map
from core.node import TreeNode, Node # Import base Node if needed for path type checking
from core.logger import logger

from algorithms.algorithm_manager import AlgorithmManager
from maps.maps_manager import MapsManager
from benchmarks.benchmark_manager import BenchmarkManager

SCALE = 6  # Scale factor for display

class VisualisationPanel(QWidget):
    # Define the signal: emits (Map object, path list) when a path is found
    # Using 'object' for Map allows flexibility if subclassing Map later
    # Using 'list' ensures the receiver gets a standard Python list
    path_found = pyqtSignal(object, list, object) # Map, Path List, Algorithm Instance

    def __init__(self, maps_manager: MapsManager, algorithm_manager: AlgorithmManager, panel_id: str):
        super().__init__()
        self.maps_manager = maps_manager
        self.algorithm_manager = algorithm_manager
        self.benchmark_manager = BenchmarkManager()
        self.panel_id = panel_id
        logger.info(f"Initializing VisualisationPanel '{self.panel_id}'")

        # --- Initialize Map and Algorithm ---
        self.map = Map(100, 100) # Default empty map
        self.algorithm = None

        # --- Graphics Scene and View ---
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setFixedSize(int(100 * SCALE) + 16, int(100 * SCALE) + 16) # Initial size

        # --- Controls (Copied from previous version) ---
        self.algorithm_selector = QComboBox()
        for algo in self.algorithm_manager.algorithms:
            self.algorithm_selector.addItem(algo["name"])
        self.algorithm_selector.currentIndexChanged.connect(self.select_algorithm)

        self.map_selector = QComboBox()
        for map_obj in maps_manager.maps:
            self.map_selector.addItem(map_obj["name"])
        self.map_selector.currentIndexChanged.connect(self.load_map)

        self.start_button = QPushButton('Set Start')
        self.goal_button = QPushButton('Set Goal')
        self.reset_path_button = QPushButton('Reset Path')
        self.iterate_button = QPushButton('Iterate')
        self.auto_iterate_button = QPushButton('Auto Iterate')
        self.stop_auto_iterate_button = QPushButton('Stop Auto Iterate')
        self.execute_till_solution_button = QPushButton('Execute Till Solution')

        self.step_input = QSpinBox()
        self.step_input.setRange(1, 1000)
        self.step_input.setValue(5)
        self.step_label = QLabel('Steps:')

        self.interval_input = QDoubleSpinBox()
        self.interval_input.setRange(0.001, 5.0)
        self.interval_input.setSingleStep(0.001)
        self.interval_input.setDecimals(3)
        self.interval_input.setValue(0.001)
        self.interval_label = QLabel('Interval (s):')

        self.step_size_input = QDoubleSpinBox()
        self.step_size_input.setRange(0.01, 30)
        self.step_size_input.setValue(2)
        self.step_size_input.setSingleStep(0.01)
        self.step_size_input_label = QLabel("Step Size:")
        self.step_size_input.valueChanged.connect(self.update_step_size)

        # --- Timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.iterate_one_step)

        # --- Button Connections ---
        self.start_button.clicked.connect(self.set_start)
        self.goal_button.clicked.connect(self.set_goal)
        self.reset_path_button.clicked.connect(self.reset_path)
        self.iterate_button.clicked.connect(self.iterate)
        self.auto_iterate_button.clicked.connect(self.start_auto_iterate)
        self.stop_auto_iterate_button.clicked.connect(self.stop_auto_iterate)
        self.execute_till_solution_button.clicked.connect(self.execute_till_solution)

        # --- Layout ---
        layout = QVBoxLayout()
        layout.addWidget(QLabel(f"--- Panel {self.panel_id} ---"))
        layout.addWidget(self.algorithm_selector)
        layout.addWidget(self.map_selector)
        layout.addWidget(self.view)
        layout.addWidget(self.start_button)
        layout.addWidget(self.goal_button)
        layout.addWidget(self.reset_path_button)
        layout.addWidget(self.step_label)
        layout.addWidget(self.step_input)
        layout.addWidget(self.interval_label)
        layout.addWidget(self.interval_input)
        layout.addWidget(self.step_size_input_label)
        layout.addWidget(self.step_size_input)
        layout.addWidget(self.iterate_button)
        layout.addWidget(self.auto_iterate_button)
        layout.addWidget(self.stop_auto_iterate_button)
        layout.addWidget(self.execute_till_solution_button)
        self.setLayout(layout)

        # --- State Variables ---
        self.start_mode = False
        self.goal_mode = False
        self.path_found_emitted = False # Flag to track if path signal was emitted

        # --- Mouse Interaction ---
        self.view.mousePressEvent = self.on_mouse_press

        # --- Initial Setup ---
        self.load_map() # This will trigger initialise_algorithm and draw_map

    # --- Coordinate Conversion ---
    def map_to_display(self, x, y):
        return x * SCALE, y * SCALE

    def display_to_map(self, x, y):
        return x / SCALE, y / SCALE

    # --- Drawing Methods ---
    def draw_map(self):
        self.scene.clear()
        if not self.map: return

        # Update view size and scene rect based on current map
        self.view.setFixedSize(int(self.map.width * SCALE) + 16, int(self.map.height * SCALE) + 16)
        self.scene.setSceneRect(0, 0, int(self.map.width * SCALE), int(self.map.height * SCALE))

        if self.algorithm:
            point_size = self.algorithm.step_size * 2 * SCALE
        else:
            point_size = 2 * SCALE # Default size if no algorithm

        # Draw obstacles
        for ox, oy, w, h in self.map.get_obstacles():
            dx, dy = self.map_to_display(ox, oy)
            dw, dh = w * SCALE, h * SCALE
            obstacle = QGraphicsRectItem(dx, dy, dw, dh)
            obstacle.setBrush(Qt.darkGray)
            self.scene.addItem(obstacle)

        # Draw start point
        if self.map.start:
            sx, sy = self.map_to_display(self.map.start.x, self.map.start.y)
            start = QGraphicsEllipseItem(sx - point_size / 2 , sy - point_size / 2, point_size, point_size)
            start.setBrush(Qt.green)
            self.scene.addItem(start)

        # Draw goal point
        if self.map.goal:
            gx, gy = self.map_to_display(self.map.goal.x, self.map.goal.y)
            goal = QGraphicsEllipseItem(gx - point_size / 2, gy - point_size / 2, point_size, point_size)
            goal.setBrush(Qt.red)
            self.scene.addItem(goal)

        # Draw algorithm specific elements (tree/graph)
        if self.algorithm is not None:
            if self.algorithm.architecture == "tree":
                self.draw_tree()
            elif self.algorithm.architecture == "graph":
                self.draw_graph()
            elif self.algorithm.architecture is not None:
                 logger.warning(f"Panel {self.panel_id}: Algorithm architecture '{self.algorithm.architecture}' drawing not implemented!")

        self.view.viewport().update()

    def draw_tree(self):
        # (Code from previous version - remains the same)
        if self.algorithm:
            # Draw active/passive trees if applicable (e.g., BiRRT)
            if hasattr(self.algorithm, "tree_start") and hasattr(self.algorithm, "tree_goal"):
                # Draw active tree (e.g., blue)
                for node in self.algorithm.tree_start:
                    if node.parent:
                        x1, y1 = self.map_to_display(node.parent.x, node.parent.y)
                        x2, y2 = self.map_to_display(node.x, node.y)
                        line = QGraphicsLineItem(QLineF(x1 + SCALE/2, y1 + SCALE/2, x2 + SCALE/2, y2 + SCALE/2))
                        line.setPen(QPen(QColor("blue"), 2))
                        self.scene.addItem(line)
                # Draw passive tree (e.g., gray or another color)
                for node in self.algorithm.tree_goal:
                    if node.parent:
                        x1, y1 = self.map_to_display(node.parent.x, node.parent.y)
                        x2, y2 = self.map_to_display(node.x, node.y)
                        line = QGraphicsLineItem(QLineF(x1 + SCALE/2, y1 + SCALE/2, x2 + SCALE/2, y2 + SCALE/2))
                        line.setPen(QPen(QColor(150, 150, 150), 2)) # Example: Gray for passive tree
                        self.scene.addItem(line)
            else:
                # Draw single tree (e.g., RRT)
                for node in self.algorithm.get_nodes():
                    if isinstance(node, TreeNode) and node.parent:
                        x1, y1 = self.map_to_display(node.parent.x, node.parent.y)
                        x2, y2 = self.map_to_display(node.x, node.y)
                        line = QGraphicsLineItem(QLineF(x1 + SCALE/2, y1 + SCALE/2, x2 + SCALE/2, y2 + SCALE/2))
                        line.setPen(QPen(QColor("blue"), 2))
                        self.scene.addItem(line)

            # Draw shortest path in green if complete
            if self.algorithm.is_complete():
                try:
                    if self.algorithm.shortest_path and hasattr(self.algorithm.shortest_path[0], 'parent'):
                         for node in self.algorithm.shortest_path:
                            if node.parent: # Check if node has a parent to draw segment
                                x1, y1 = self.map_to_display(node.parent.x, node.parent.y)
                                x2, y2 = self.map_to_display(node.x, node.y)
                                line = QGraphicsLineItem(QLineF(x1 + SCALE/2, y1 + SCALE/2, x2 + SCALE/2, y2 + SCALE/2))
                                line.setPen(QPen(QColor("green"), 3, Qt.SolidLine)) # Explicitly SolidLine
                                self.scene.addItem(line)
                except (TypeError, IndexError, AttributeError) as e:
                     logger.error(f"Panel {self.panel_id}: Error drawing shortest path (tree): {e}. Path data: {self.algorithm.shortest_path}")


    def draw_graph(self):
        # (Code from previous version - remains the same)
        if not self.algorithm: return

        pen_edge = QPen(QColor("blue"), 1)
        pen_path = QPen(QColor("green"), 3, Qt.SolidLine) # Explicitly SolidLine
        radius = 2 # Display radius for nodes

        # Draw graph edges
        for node in self.algorithm.get_nodes():
            x1, y1 = self.map_to_display(node.x, node.y)
            if hasattr(node, 'edges'):
                for neighbour in node.edges:
                    x2, y2 = self.map_to_display(neighbour.x, neighbour.y)
                    line = QGraphicsLineItem(QLineF(x1, y1, x2, y2))
                    line.setPen(pen_edge)
                    self.scene.addItem(line)

        # Draw graph nodes
        for node in self.algorithm.get_nodes():
            x, y = self.map_to_display(node.x, node.y)
            ellipse = QGraphicsEllipseItem(x - radius, y - radius, radius * 2, radius * 2)
            ellipse.setBrush(QColor("lightblue"))
            ellipse.setPen(QPen(Qt.NoPen)) # No outline for nodes
            self.scene.addItem(ellipse)

        # Draw the shortest path
        if self.algorithm.is_complete() and self.algorithm.shortest_path:
            try:
                if len(self.algorithm.shortest_path) > 1:
                    for i in range(1, len(self.algorithm.shortest_path)):
                        n1 = self.algorithm.shortest_path[i - 1]
                        n2 = self.algorithm.shortest_path[i]
                        x1, y1 = self.map_to_display(n1.x, n1.y)
                        x2, y2 = self.map_to_display(n2.x, n2.y)
                        line = QGraphicsLineItem(QLineF(x1, y1, x2, y2))
                        line.setPen(pen_path)
                        self.scene.addItem(line)
            except (TypeError, IndexError, AttributeError) as e:
                 logger.error(f"Panel {self.panel_id}: Error drawing shortest path (graph): {e}. Path data: {self.algorithm.shortest_path}")

    # --- Signal Emission ---
    def check_and_emit_path(self):
        """Checks if a path is complete and emits the path_found signal if not already done."""
        if self.algorithm and self.algorithm.is_complete() and self.algorithm.shortest_path and not self.path_found_emitted:
             logger.info(f"Panel {self.panel_id}: Path found! Emitting signal.")
             try:
                 # Ensure path is emitted as a standard list
                 path_list = list(self.algorithm.shortest_path)
                 if path_list: # Don't emit empty paths
                     self.path_found.emit(self.map, path_list, self.algorithm)
                     self.path_found_emitted = True # Set flag after successful emission
                 else:
                      logger.warning(f"Panel {self.panel_id}: Algorithm complete but shortest_path is empty, not emitting.")
             except Exception as e:
                 logger.error(f"Panel {self.panel_id}: Error emitting path_found signal: {e}")

    # --- Control Actions ---
    def update_step_size(self):
        if self.algorithm:
            new_step_size = self.step_size_input.value()
            logger.info(f"Panel {self.panel_id}: Updating step size to {new_step_size}")
            self.algorithm.step_size = new_step_size
            # No need to re-initialize, just redraw map potentially for point size
            self.draw_map()

    def set_start(self):
        logger.info(f"Panel {self.panel_id}: Set Start mode activated.")
        self.start_mode = True
        self.goal_mode = False

    def set_goal(self):
        logger.info(f"Panel {self.panel_id}: Set Goal mode activated.")
        self.goal_mode = True
        self.start_mode = False

    def reset_simulation_data(self):
        """Resets algorithm data (nodes, path) but keeps map and selections."""
        self.stop_auto_iterate()
        self.path_found_emitted = False # Reset flag
        if self.algorithm:
            logger.info(f"Panel {self.panel_id}: Resetting algorithm data.")
            # Reinitialize the algorithm cleanly with current map settings
            self.initialise_algorithm() # This clears nodes and resets state
        else:
            self.scene.clear() # Just clear scene if no algorithm
        self.draw_map() # Redraw the base map

    def reset_path(self):
        """Clears the generated path/tree/graph but keeps start/goal/algorithm instance."""
        self.stop_auto_iterate()
        self.path_found_emitted = False # Reset flag as path is no longer valid
        if self.algorithm:
            logger.info(f"Panel {self.panel_id}: Resetting path/nodes only.")
            self.algorithm.clear_nodes() # Clears the generated structure
            # Depending on the algorithm, might need another method like reset_solution()
        else:
             logger.warning(f"Panel {self.panel_id}: No algorithm to reset path for.")
        self.draw_map() # Redraw map without the path/tree

    def iterate_one_step(self):
        """Performs one step of the algorithm and checks for completion."""
        if self.algorithm and not self.algorithm.is_complete():
            self.algorithm.step()
            self.draw_map()
            self.check_and_emit_path() # Check completion after the step
        else:
            # If already complete or no algorithm, stop the timer
            if self.algorithm and self.algorithm.is_complete():
                 # Ensure signal is emitted if somehow missed
                 self.check_and_emit_path()
            self.stop_auto_iterate()

    def iterate(self):
        """Performs a batch of algorithm steps."""
        if self.algorithm is None or self.map.start is None or self.map.goal is None:
            logger.warning(f"Panel {self.panel_id}: Set both start and goal before running the algorithm!")
            return

        steps = self.step_input.value()
        logger.info(f"Panel {self.panel_id}: Iterating {steps} steps.")
        goal_reached_in_loop = False
        for i in range(steps):
            if not self.algorithm.is_complete():
                self.algorithm.step()
            else:
                if not goal_reached_in_loop:
                    logger.info(f"Panel {self.panel_id}: Goal reached during iteration {i+1}.")
                    goal_reached_in_loop = True
                # Decide whether to break or continue for remaining steps
                # break # Option 1: Stop as soon as goal is found
                pass # Option 2: Complete all requested steps

        self.draw_map() # Draw the final state after the loop
        self.check_and_emit_path() # Check completion status after the loop

    def start_auto_iterate(self):
        if self.algorithm is None or self.map.start is None or self.map.goal is None:
            logger.warning(f"Panel {self.panel_id}: Set both start and goal before auto-iterating!")
            return
        if self.algorithm.is_complete():
            logger.info(f"Panel {self.panel_id}: Cannot auto-iterate, goal already reached.")
            self.check_and_emit_path() # Make sure signal was sent
            return

        interval = int(self.interval_input.value() * 1000)
        if interval <= 0:
             logger.warning(f"Panel {self.panel_id}: Auto-iterate interval too low ({interval}ms). Setting to 1ms.")
             interval = 1 # Prevent zero or negative interval
        logger.info(f"Panel {self.panel_id}: Starting auto-iteration with interval {interval}ms.")
        self.timer.start(interval)

    def stop_auto_iterate(self):
        if self.timer.isActive():
            logger.info(f"Panel {self.panel_id}: Stopping auto-iteration.")
            self.timer.stop()

    def execute_till_solution(self):
        """Runs the algorithm until completion or max steps."""
        if self.algorithm is None or self.map.start is None or self.map.goal is None:
            logger.warning(f"Panel {self.panel_id}: Set both start and goal before running the algorithm!")
            return

        if self.algorithm.is_complete():
             logger.info(f"Panel {self.panel_id}: Algorithm already complete.")
             self.draw_map() # Ensure current state is drawn
             self.check_and_emit_path() # Ensure signal was sent
             return

        logger.info(f"Panel {self.panel_id}: Executing algorithm until solution...")
        count = 0
        max_steps = 100000 # Safety break

        while not self.algorithm.is_complete() and count < max_steps:
            self.algorithm.step()
            count += 1
            # Optional: Update UI less frequently for very long runs
            # if count % 500 == 0: self.draw_map(); QApplication.processEvents()

        if self.algorithm.is_complete():
            logger.info(f"Panel {self.panel_id}: Goal reached after {count} steps!")
        else:
             logger.warning(f"Panel {self.panel_id}: Execution stopped after {max_steps} steps without reaching goal.")

        self.draw_map()  # Final update
        self.check_and_emit_path() # Check status after completion/timeout

    # --- Setup and Selection ---
    def select_algorithm(self):
        """Handles algorithm selection from the dropdown."""
        # Only initialize if start and goal are set
        if self.map and self.map.start is not None and self.map.goal is not None:
            self.initialise_algorithm()
        else:
             logger.debug(f"Panel {self.panel_id}: Algorithm selection changed, deferring initialization until map/start/goal are set.")


    def initialise_algorithm(self):
        """Initialises or re-initialises the algorithm instance based on current selections."""
        selected_algorithm_name = self.algorithm_selector.currentText()
        self.path_found_emitted = False # Reset emission flag whenever algorithm changes

        if selected_algorithm_name and self.map and self.map.start and self.map.goal:
            logger.info(f"Panel {self.panel_id}: Initialising algorithm '{selected_algorithm_name}'.")
            try:
                self.algorithm = self.algorithm_manager.get_algorithm(
                    selected_algorithm_name,
                    map_instance=self.map,
                    benchmark_manager=self.benchmark_manager
                )
                self.algorithm.step_size = self.step_size_input.value()
                # Ensure algorithm internal state is clean (e.g., no nodes from previous run)
                self.algorithm.clear_nodes()
                logger.info(f"Panel {self.panel_id}: Algorithm '{selected_algorithm_name}' initialized.")
            except Exception as e:
                 logger.error(f"Panel {self.panel_id}: Failed to initialize algorithm '{selected_algorithm_name}': {e}", exc_info=True)
                 self.algorithm = None
        else:
            # Conditions not met for initialization
            self.algorithm = None
            missing = []
            if not selected_algorithm_name: missing.append("Algorithm Name")
            if not self.map: missing.append("Map")
            elif not self.map.start: missing.append("Start Point")
            elif not self.map.goal: missing.append("Goal Point")
            logger.warning(f"Panel {self.panel_id}: Cannot initialise algorithm - missing prerequisites: {', '.join(missing)}.")

        self.draw_map() # Redraw the scene


    def load_map(self):
        """Loads a map selected from the dropdown."""
        self.stop_auto_iterate()
        self.path_found_emitted = False # Reset emission flag
        selected_map_name = self.map_selector.currentText()
        logger.info(f"Panel {self.panel_id}: Loading map '{selected_map_name}'.")

        map_config = self.maps_manager.get_map(selected_map_name)
        if map_config:
            # Create a new Map instance
            self.map = Map(map_config.width, map_config.height)
            for obs in map_config.obstacles:
                self.map.add_obstacle(*obs)

            # Set start/goal, handling cases where they might be None in config
            if map_config.default_start:
                self.map.set_start(map_config.default_start[0], map_config.default_start[1])
            else: self.map.start = None
            if map_config.default_goal:
                 self.map.set_goal(map_config.default_goal[0], map_config.default_goal[1])
            else: self.map.goal = None

            logger.info(f"Panel {self.panel_id}: Map '{selected_map_name}' loaded. Start: {self.map.start}, Goal: {self.map.goal}")

            # Re-initialise algorithm AFTER map and its points are set
            self.initialise_algorithm() # This calls draw_map internally

        else:
            logger.error(f"Panel {self.panel_id}: Could not find map config for '{selected_map_name}'.")
            self.map = Map(100,100) # Reset to default empty map
            self.algorithm = None
            self.draw_map() # Draw the empty map state

    # --- Mouse Event Handling ---
    def on_mouse_press(self, event):
        """Handles mouse clicks on the QGraphicsView to set start/goal."""
        if event.button() == Qt.LeftButton:
            scene_pos = self.view.mapToScene(event.pos())
            map_x, map_y = self.display_to_map(scene_pos.x(), scene_pos.y())

            # Basic boundary check
            if not (0 <= map_x <= self.map.width and 0 <= map_y <= self.map.height):
                logger.warning(f"Panel {self.panel_id}: Click outside map boundaries ignored.")
                return

            point_updated = False
            if self.start_mode:
                logger.info(f"Panel {self.panel_id}: Setting start to ({map_x:.2f}, {map_y:.2f}).")
                self.map.set_start(map_x, map_y)
                self.start_mode = False
                point_updated = True
            elif self.goal_mode:
                logger.info(f"Panel {self.panel_id}: Setting goal to ({map_x:.2f}, {map_y:.2f}).")
                self.map.set_goal(map_x, map_y)
                self.goal_mode = False
                point_updated = True

            if point_updated:
                self.path_found_emitted = False # Reset flag as path is invalid now
                # Re-initialize the current algorithm with the new point
                if self.algorithm:
                     logger.info(f"Panel {self.panel_id}: Re-initializing algorithm due to start/goal change.")
                     # Option 1: Full re-init (clears everything) - Safer for most tree algos
                     self.initialise_algorithm()
                     # Option 2: Update start/goal if algorithm supports it (better for graph algos)
                     # if hasattr(self.algorithm, 'reinintialise_start_and_goal'):
                     #     self.algorithm.reinintialise_start_and_goal(start=self.map.start, goal=self.map.goal)
                     #     self.reset_path() # Keep graph, just clear nodes/path related to old start/goal
                     # else:
                     #     self.initialise_algorithm() # Fallback to full re-init
                else:
                     # If no algorithm exists yet, try initializing now that points might be set
                     self.initialise_algorithm()

                self.draw_map() # Redraw with the new point

            else:
                # Pass event to default handler if not in set mode (for potential future use like panning/zooming)
                QGraphicsView.mousePressEvent(self.view, event)
        else:
             # Pass other button presses
             QGraphicsView.mousePressEvent(self.view, event)