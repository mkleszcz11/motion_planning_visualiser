class BenchmarkResult:
    def __init__(self,
                 algorithm_name,
                 steps,
                 execution_time,
                 start_point,
                 goal_point,
                 step_size,
                 path_length,
                 shortest_path):
        self.algorithm_name = algorithm_name
        self.steps = steps
        self.execution_time = execution_time
        self.start_point = start_point
        self.goal_point = goal_point
        self.step_size = step_size
        self.path_length = path_length
        self.path = shortest_path # List of points


    def __str__(self):
        # List of nodes:
        # nodes = [f"({n.x:.2f}, {n.y:.2f})" for n in self.path]
        return f"{self.algorithm_name}: Length={self.path_length:.2f}, Steps={self.steps}, Time={self.execution_time:.4f}s"#, Nodes={nodes}"
