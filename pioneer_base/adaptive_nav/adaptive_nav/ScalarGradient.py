

class ScalarGradient:
    def __init__(self, num_robots=3):
        self.num_robots = num_robots
        # Array of robot position vectors in [x, y, z] where x, y is position in the plane
        # and z is the "height" in the scalar field.
        self.robot_positions = [self.num_robots*[0.0] for _ in range(3)] 

    def get_velocity(self) -> [float]:
        bearing = self.calc_gradient()
        # Determine velocity based on the bearing and mode [ascent, descent, contour folowing]

    def calc_gradient(self) -> float:
        #math here to return gradient bearing
        return 0.0  # Placeholder for actual gradient calculation