from core.map import Map
from maps.maps_manager import MapsManager
from gui.visualiser import Visualiser
from PyQt5.QtWidgets import QApplication
import sys

def main():
    app = QApplication(sys.argv)
    maps_manager = MapsManager()

    window = Visualiser(maps_manager = maps_manager)
    window.draw_map()

    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
