class BenchmarkResult:
    def __init__(self, algorithm_name, path_length, steps, execution_time, path):
        self.algorithm_name = algorithm_name
        self.path_length = path_length
        self.steps = steps
        self.execution_time = execution_time
        self.path = path

    def __str__(self):
        # List of nodes:
        # nodes = [f"({n.x:.2f}, {n.y:.2f})" for n in self.path]
        return f"{self.algorithm_name}: Length={self.path_length:.2f}, Steps={self.steps}, Time={self.execution_time:.4f}s"#, Nodes={nodes}"
