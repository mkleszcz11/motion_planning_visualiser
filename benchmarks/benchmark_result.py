class BenchmarkResult:
    def __init__(self, algorithm_name, path_length, steps, execution_time):
        self.algorithm_name = algorithm_name
        self.path_length = path_length
        self.steps = steps
        self.execution_time = execution_time

    def __str__(self):
        return f"{self.algorithm_name}: Length={self.path_length:.2f}, Steps={self.steps}, Time={self.execution_time:.4f}s"
