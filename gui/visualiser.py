import sys

from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QGraphicsScene, QGraphicsView, 
                             QGraphicsRectItem, QGraphicsEllipseItem, QSpinBox, QLabel, QVBoxLayout, 
                             QWidget, QGraphicsLineItem, QComboBox, QDoubleSpinBox)
from PyQt5.QtCore import Qt, QRectF, QLineF, QTimer
from PyQt5.QtGui import QPen, QColor

from core.map import Map
from core.node import Node
from core.logger import logger

from algorithms.algorithm_manager import AlgorithmManager
from maps.maps_manager import MapsManager
from benchmarks.benchmark_manager import BenchmarkManager

SCALE = 6  # Scale factor for display

class Visualiser(QMainWindow):
    def __init__(self, maps_manager: MapsManager):
        super().__init__()
        self.maps_manager = maps_manager
        self.algorithm_manager = AlgorithmManager()
        self.benchmark_manager = BenchmarkManager()
        self.map = Map(100, 100)
        #self.map = self.maps_manager.get_map("Simple Map") # Take simple map from maps manager
        self.algorithm = None
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setFixedSize(int(self.map.width * SCALE) + 16, int(self.map.height * SCALE) + 16)

        # Algorithm selection dropdown
        self.algorithm_selector = QComboBox()
        for algo in self.algorithm_manager.algorithms:
            self.algorithm_selector.addItem(algo["name"])
        self.algorithm_selector.currentIndexChanged.connect(self.select_algorithm)

        # Map selection dropdown
        self.map_selector = QComboBox()
        for map_obj in maps_manager.maps:
            self.map_selector.addItem(map_obj["name"])
        self.map_selector.currentIndexChanged.connect(self.load_map) # Load map (first on the list?)

        self.start_button = QPushButton('Set Start')
        self.goal_button = QPushButton('Set Goal')
        #self.reset_button = QPushButton('Reset')
        self.reset_path_button = QPushButton('Reset Path')
        self.iterate_button = QPushButton('Iterate')
        self.auto_iterate_button = QPushButton('Auto Iterate')
        self.stop_auto_iterate_button = QPushButton('Stop Auto Iterate')
        self.execute_till_solution_button = QPushButton('Execute Till Solution')

        self.step_input = QSpinBox()
        self.step_input.setRange(1, 1000)
        self.step_input.setValue(5)
        self.step_label = QLabel('Steps:')

        # Interval input (float)
        self.interval_input = QDoubleSpinBox()
        self.interval_input.setRange(0.001, 5.0)
        self.interval_input.setSingleStep(0.001)
        self.interval_input.setDecimals(3)
        self.interval_input.setValue(0.1)
        self.interval_label = QLabel('Interval (s):')
        
        # Step size input
        self.step_size_input = QDoubleSpinBox()
        self.step_size_input.setRange(0.01, 30)
        self.step_size_input.setValue(2)
        self.step_size_input.setSingleStep(0.01)
        self.step_size_input_label = QLabel("Step Size:")
        self.step_size_input.valueChanged.connect(self.update_step_size)

        # Timer for auto-iteration
        self.timer = QTimer()
        self.timer.timeout.connect(self.iterate_one_step)

        # Button connections
        self.start_button.clicked.connect(self.set_start)
        self.goal_button.clicked.connect(self.set_goal)
        #self.reset_button.clicked.connect(self.reset_simulation)
        self.reset_path_button.clicked.connect(self.reset_path)
        self.iterate_button.clicked.connect(self.iterate)
        self.auto_iterate_button.clicked.connect(self.start_auto_iterate)
        self.stop_auto_iterate_button.clicked.connect(self.stop_auto_iterate)
        self.execute_till_solution_button.clicked.connect(self.execute_till_solution)

        layout = QVBoxLayout()
        layout.addWidget(self.algorithm_selector)
        layout.addWidget(self.map_selector)
        layout.addWidget(self.view)
        layout.addWidget(self.start_button)
        layout.addWidget(self.goal_button)
        #layout.addWidget(self.reset_button)
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

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.setWindowTitle("Motion Planning Visualiser")
        self.setGeometry(100, 100, int(self.map.width * SCALE) + 20, int(self.map.height * SCALE) + 200)

        self.start_mode = False
        self.goal_mode = False

        self.view.mousePressEvent = self.on_mouse_press

        self.load_map()
        self.select_algorithm() # This must be called after load_map, as it needs start and goal points
        self.draw_map()

    def map_to_display(self, x, y):
        return x * SCALE, y * SCALE

    def display_to_map(self, x, y):
        return x / SCALE, y / SCALE

    def draw_map(self):
        self.scene.clear()
        if self.algorithm:
            # point_size is twice the step size
            # Example:
            # Step size is 4, which means if my algorithm is 4 units away from
            # the goal it is considered as done, point "diameter" should be 8
            point_size = self.algorithm.step_size * 2 * SCALE
        else:
            point_size = 1
            logger.error("Algorithm not defined!")

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

        # Draw full tree in blue
        if self.algorithm:
            for node in self.algorithm.get_nodes():
                if isinstance(node, Node) and node.parent:
                    x1, y1 = self.map_to_display(node.parent.x, node.parent.y)
                    x2, y2 = self.map_to_display(node.x, node.y)
                    line = QGraphicsLineItem(QLineF(x1 + SCALE/2, y1 + SCALE/2, x2 + SCALE/2, y2 + SCALE/2))
                    line.setPen(QPen(QColor("blue"), 2))
                    self.scene.addItem(line)

            # Draw shortest path in green
            if self.algorithm.is_complete():
                for node in self.algorithm.shortest_path:
                    if node.parent:
                        x1, y1 = self.map_to_display(node.parent.x, node.parent.y)
                        x2, y2 = self.map_to_display(node.x, node.y)
                        line = QGraphicsLineItem(QLineF(x1 + SCALE/2, y1 + SCALE/2, x2 + SCALE/2, y2 + SCALE/2))
                        line.setPen(QPen(QColor("green"), 3))
                        self.scene.addItem(line)

        self.update()

    def update_step_size(self):
        if self.algorithm:
            self.algorithm.step_size = self.step_size_input.value()
            self.draw_map()

    def set_start(self):
        self.start_mode = True
        self.goal_mode = False

    def set_goal(self):
        self.goal_mode = True
        self.start_mode = False
        
    def reset_simulation(self):
        self.stop_auto_iterate()
        self.map.reset()
        # self.algorithm = None
        self.draw_map()
        
    def reset_path(self):
        self.stop_auto_iterate()
        if self.algorithm:
            self.algorithm.clear_nodes()
        self.draw_map()

    def iterate_one_step(self):
        if self.algorithm and not self.algorithm.is_complete():
            self.algorithm.step()
            self.draw_map()
        else:
            self.stop_auto_iterate()

    def iterate(self):
        # Only run if algorithm is defined and both start/goal points are set
        if self.algorithm is None or self.map.start is None or self.map.goal is None:
            logger.warning("Set both start and goal before running the algorithm!")
            return

        steps = self.step_input.value()
        for _ in range(steps):
            if not self.algorithm.is_complete():
                self.algorithm.step()
            else:
                logger.info("Goal reached!")
                # Force one last draw to display final segment
                self.draw_map()
                return

        # Draw after every iteration to show growing path
        self.draw_map()

    def start_auto_iterate(self):
        interval = int(self.interval_input.value() * 1000)  # Convert to milliseconds and cast to int
        self.timer.start(interval)
        
    def stop_auto_iterate(self):
        self.timer.stop()

    def execute_till_solution(self):
        if self.algorithm is None or self.map.start is None or self.map.goal is None:
            logger.warning("Set both start and goal before running the algorithm!")
            return
        
        logger.info("Executing solution...")
        while not self.algorithm.is_complete():
            self.algorithm.step()

        logger.info("Goal reached!")
        self.draw_map()  # Final update to show the last segment

    def select_algorithm(self):
        # Only create an algorithm if both start and goal are defined
        if self.map.start is not None and self.map.goal is not None:
            self.initialise_algorithm()

    def initialise_algorithm(self):
        selected_algorithm = self.algorithm_selector.currentText()

        if selected_algorithm:
            self.algorithm = self.algorithm_manager.get_algorithm(
                selected_algorithm,
                map_instance = self.map,
                benchmark_manager = self.benchmark_manager
            )
            step_size = self.step_size_input.value()
            self.algorithm.step_size = step_size
            
        self.draw_map()
        
    def load_map(self):
        self.reset_simulation()
        selected_map = self.map_selector.currentText()

        map_config = self.maps_manager.get_map(selected_map)
        if map_config:
            self.map = Map(map_config.width, map_config.height)
            for obs in map_config.obstacles:
                self.map.add_obstacle(*obs)
            self.map.set_start(map_config.default_start[0], map_config.default_start[1])
            self.map.set_goal(map_config.default_goal[0], map_config.default_goal[1])

        self.draw_map()

    def on_mouse_press(self, event):
        x, y = self.display_to_map(event.pos().x(), event.pos().y())

        if self.start_mode:
            self.map.set_start(x, y)
            self.start_mode = False
            # Re-initialise algorithm when start is defined
            self.initialise_algorithm()

        elif self.goal_mode:
            self.map.set_goal(x, y)
            self.goal_mode = False
            # Re-initialise algorithm when goal is defined
            self.initialise_algorithm()

        self.draw_map()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Visualiser()
    window.show()
    sys.exit(app.exec_())
