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
        self.latest_value = 0.0
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
        for i in range(len(desired)):
            if i in [3,4,5]:
                continue
            diff += (desired[i] - curent[i])**2
            print(f"{i}: desired: {desired[i]}, current: {curent[i]}, diff: {(desired[i] - curent[i])**2}")
        self.latest_value = diff
        print(f"diff: {self.latest_value}")

class RealtimeGraph(QWidget):
    def __init__(self, ros_node: ROS2Listener, update_interval_ms=500, max_points=50):
        super().__init__()
        self.ros_node = ros_node
        self.update_interval_ms = update_interval_ms
        self.max_points = max_points
        self.data_x = list(range(self.max_points))
        self.data_y = [0.0] * self.max_points

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.line, = self.ax.plot(self.data_x, self.data_y, 'r-')

        self.ax.set_ylim(0, 1)  
        self.ax.set_title("Realtime Graph")

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_graph)
        self.timer.start(self.update_interval_ms)

    def update_graph(self):
        new_value = self.ros_node.latest_value
        self.data_y = self.data_y[1:] + [new_value]
        self.line.set_ydata(self.data_y)
        self.ax.relim()
        self.ax.autoscale_view()
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
