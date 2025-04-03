import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading


from pioneer_interfaces.msg import ClusterInfo


class ROS2Listener(Node):
    def __init__(self):
        super().__init__('pyqt_graph_listener')
        self.latest_value = [0.0] * 9
        self.subscription = self.create_subscription(
            ClusterInfo,
            '/sim/cluster_info',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        desired = msg.cluster_desired.data
        curent = msg.cluster.data
        diff = 0.0
        ind = 0
        for i in range(len(desired)):
            if i in [3,4,5]:
                continue
            diff += (desired[i] - curent[i])**2
            print(f"{i}: desired: {desired[i]}, current: {curent[i]}, diff: {(desired[i] - curent[i])**2}")
            self.latest_value[ind] = curent[i]
            ind += 1
        self.latest_value[ind] = diff
        print(f"diff: {self.latest_value}")

class RealtimeGraph(QWidget):
    def __init__(self, ros_node: ROS2Listener, update_interval_ms=500, max_points=50, rows=2, cols=3):
        super().__init__()
        self.ros_node = ros_node
        self.update_interval_ms = update_interval_ms
        self.max_points = max_points
        self.rows = rows
        self.cols = cols
        self.num_plots = rows * cols

        self.data_x = list(range(self.max_points))
        self.data_ys = [[0.0] * self.max_points for _ in range(self.num_plots)]

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.axes = []
        self.lines = []

        # Create subplots in a grid
        for i in range(self.num_plots):
            ax = self.figure.add_subplot(self.rows, self.cols, i + 1)
            line, = ax.plot(self.data_x, self.data_ys[i], 'r-')
            ax.set_ylim(0, 1)
            ax.set_title(f"Graph {i+1}")
            self.axes.append(ax)
            self.lines.append(line)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_graph)
        self.timer.start(self.update_interval_ms)

    def update_graph(self):
        new_value = self.ros_node.latest_value
        for i in range(self.num_plots):
            self.data_ys[i] = self.data_ys[i][1:] + [new_value[i]]  # Update data
            self.lines[i].set_ydata(self.data_ys[i])
            self.axes[i].relim()
            self.axes[i].autoscale_view()
        self.canvas.draw()

class MainWindow(QMainWindow):
    def __init__(self, ros_node: ROS2Listener, interval_ms):
        super().__init__()
        self.setWindowTitle("ROS2 Realtime Graph")
        self.graph = RealtimeGraph(ros_node, update_interval_ms=interval_ms)
        self.setCentralWidget(self.graph)
        self.resize(800, 600)


def main():
    rclpy.init()
    ros_node = ROS2Listener()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node, interval_ms=500)
    window.show()
    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
