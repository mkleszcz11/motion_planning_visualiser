# main_app.py

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout
from PyQt5.QtCore import QSize

# --- Import your core classes ---
# from core.map import Map # No longer needed directly here
from core.logger import logger
from maps.maps_manager import MapsManager
from algorithms.algorithm_manager import AlgorithmManager

# --- Import the panel classes ---
from visualisation_panel import VisualisationPanel, SCALE
from post_processing_panel import PostProcessingPanel # Import the new panel

class MainWindow(QMainWindow):
    def __init__(self, maps_manager: MapsManager, algorithm_manager: AlgorithmManager):
        super().__init__()
        self.maps_manager = maps_manager
        self.algorithm_manager = algorithm_manager

        self.setWindowTitle("Motion Planning - Algorithm & Post-Processing")

        # Create the two panels
        self.panel_algo = VisualisationPanel(self.maps_manager, self.algorithm_manager, panel_id="Algorithm")
        self.panel_post = PostProcessingPanel(panel_id="PostProcessing")

        # --- Main Layout ---
        main_layout = QHBoxLayout()
        main_layout.addWidget(self.panel_algo)   # Left panel
        main_layout.addWidget(self.panel_post) # Right panel

        # --- Central Widget ---
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # --- Signal/Slot Connection ---
        # Connect the path_found signal from panel_algo to the update_data slot of panel_post
        self.panel_algo.path_found.connect(self.panel_post.update_data)
        logger.info("Connected AlgorithmPanel.path_found signal to PostProcessingPanel.update_data slot.")

        # --- Initial Window Size ---
        # Estimate size (similar to before, maybe slightly wider if needed)
        initial_map_width = self.panel_algo.map.width if self.panel_algo.map else 100
        initial_map_height = self.panel_algo.map.height if self.panel_algo.map else 100
        estimated_view_width = int(initial_map_width * SCALE) + 50
        estimated_controls_height = 400 # Controls height below view
        # Add some width for the post-processing controls
        total_width = (estimated_view_width * 2) + 150 # Increased spacing/margins slightly
        total_height = int(initial_map_height * SCALE) + estimated_controls_height + 50

        self.resize(QSize(total_width, total_height))


def main():
    app = QApplication(sys.argv)

    # Create managers once
    maps_manager = MapsManager()
    algorithm_manager = AlgorithmManager()

    # Create the main window and pass managers
    main_window = MainWindow(maps_manager=maps_manager, algorithm_manager=algorithm_manager)
    main_window.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    logger.info("Starting Dual Panel Application (Algorithm + PostProcessing)")
    main()