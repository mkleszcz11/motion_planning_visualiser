# Motion Planner Visualizer
An interactive 2D visualizer for motion planning algorithms using Python and PyQt5.  
This project allows you to simulate and benchmark different motion planning algorithms like **Random Walk** and **Biased Random Walk** in dynamic environments with customizable maps.  

---

## 🚀 Project Overview  
The Motion Planner Visualizer is designed to:  
✅ Simulate motion planning algorithms in real-time.  
✅ Provide step-by-step execution and visualization.  
✅ Benchmark different algorithms on multiple maps automatically.  
✅ Export benchmark results for analysis.  
✅ Allow easy extension with new algorithms and maps.  

---

## 📁 Project Structure
```bash
motion_planner_visualizer/
├── algorithms/                  
│   ├── algorithm_manager.py        # Handles algorithm registration and selection
│   ├── algorithms_implementations/ # Implementations of different algorithms
├── benchmarks/                    
│   ├── benchmark_manager.py        # Handles benchmark execution and storage
│   ├── benchmark_result.py         # Stores benchmark results
├── core/                          
│   ├── algorithm.py                # Base class for defining algorithms
│   ├── logger.py                   # Handles logging across the project
│   ├── map.py                      # Handles map structure and properties
│   ├── maps_manager.py             # Handles map registration and loading
│   ├── node.py                     # Data structure for nodes
├── gui/                           
│   ├── visualiser.py               # Main PyQt5 visualiser window
├── maps/                          
│   ├── map_config.py               # Data structure for map properties
│   ├── maps_manager.py             # Handles loading and registration of maps
│   ├── maps/                       # Map files (narrow_passage, maze, etc.)
├── test_runner/                   
│   ├── combine_heatmaps.py         # Combines heatmaps into a single comparison image
│   ├── test_runner.py              # Runs benchmark tests and saves results
│   ├── test_analyse.py             # Generates comparison tables and heatmaps
│   ├── logs/                       # Logs from test runs
│   ├── results/                    # Stores benchmark results and heatmaps
├── gui_main.py                     # Entry point for running the GUI
├── run_tests.py                    # Entry point for running the test runner
├── requirements.txt                # List of dependencies
└── README.md                       # Project documentation
```
---

## 🏆 Features  
✅ Step-through execution of algorithms.  
✅ Auto execution with adjustable interval.  
✅ Full execution until solution is found.  
✅ Real-time visualization of nodes and paths.  
✅ Automatic benchmarking of algorithms and maps.  
✅ Export benchmark results to CSV.  

---

## How to run:

### 1. Create a Virtual Environment  
```bash
python3 -m venv env
```

#### 2. Activate the Virtual Environment  
Linux/Mac:
```bash
source env/bin/activate
```
Windows:
```bash
.\env\Scripts\activate
```

### 3. Install Dependencies  
```bash
pip install -r requirements.txt
```

### 4. Run the Project (GUI)  
```bash
python gui_main.py
```

---

## How to use:

#### Step 1: Load a Map
 * From the dropdown list, select one of the available maps.

#### Step 2: Choose an Algorithm
 * From the algorithm dropdown, select the algorithm to use.

#### Step 3: Run the Algorithm
 * Manual Execution → Click "Iterate" to step through manually.
 * Auto Execution → Click "Auto Iterate" to run automatically at a fixed interval.
 * Full Execution → Click "Execute Till Solution" to find the solution as fast as possible.

---

## How to add a new algorithm:

#### Step 1: Add a new algorithm class 
 * Create a new Python file in `algorithms/algorithms_implementations/` folder.
 * Define a new class that inherits from `Algorithm` class in `core/algorithm.py`.
 * Implement all of the required methods, check random_walk.py or rrt.py for reference.

#### Step 2: Register the new algorithm
 * Open `algorithms/algorithm_manager.py`.
 * Import the new algorithm class.
 * Add the new algorithm to the `algorithms` dictionary.

#### Step 3: Run the new algorithm
 * Restart the application.
 * The new algorithm should now be available in the dropdown list.

---

## How to add a new map:

#### Step 1: Add a new map file
 * Create a new Python file in `maps/maps/` folder.
 * Define a new map using the `MapConfig` class, check existing map files for reference.

#### Step 2: Register the new map
 * Open `maps/maps_manager.py`.
 * Import the new map file.
 * Add the new map to the `maps` dictionary.

#### Step 3: Load the new map
 * Restart the application.
 * The new map should now be available in the dropdown list.

---

## Benchmarking:
Benchmarking allows you to compare the performance of different algorithms on different maps. They are essential to run the algorithm in automatic tests and compare their performance.

Currently we measure the following metrics:
 * Execution Time
 * Number of Nodes Expanded
 * Path Length (if a solution is found)

To run a benchmark mechanism to start and finish a benachmark must be implemented into the algorithm implementation. The methods are:
    * `start_benchmark()` - Must be called to start the benchmark.
    * `finish_benchmark()` - Must be called to finish a benchmark.

Note: For now time measurement in Benchamark might not be reliable, we are not measuring only the algorithm execution time but also the GUI update time and other stuff.
TODO -> Investgate how to measure only the algorithm execution time.
---

## Testing and Comparison:
 This interface allows to test different algorithms and compare their performance on different maps.

 ### How to Run Tests:
```bash
python run_tests.py
```

* The test runner will execute the algorithms on the selected maps and store the results in a CSV file.
* Once finished, it will automatically analyse the results and generate comparison tables and heatmaps.

 ### Example Config for `run_tests.py`:
```python
test_runner = TestRunner(
    algorithms=['RRT', 'Biased Random Walk'],
    maps=['Cluttered Map', 'Rooms Map'],
    runs_per_test=3,
    step_size=5,
    output_file="benchmark_results.csv",
)
```
