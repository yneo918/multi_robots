import sys
import rclpy
import json
import time
import csv
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# PyQt6 changes:
# Note that enum types are divided into Qt.AlignmentFlag, Qt.ItemFlag, Qt.TextFlag, etc.
from PyQt6.QtCore import (
    Qt, QSize, pyqtSignal, QObject
)
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QHeaderView
)
from PyQt6.QtGui import QFontMetrics
from threading import Thread, Event

from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import Pose2D


class CSVLogger:
    """
    Logger class for thread-safe writing of Pose2D data.
    """
    def __init__(self, filename):
        self.filename = filename
        self.lock = threading.Lock()
        
        # Initialize CSV file (write header row):
        # Use "w" to overwrite existing file, or "a" to append to existing file
        with open(self.filename, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["message_name", "timestamp", "x", "y", "theta"])

    def log_pose2d(self, topic_name, x, y, theta):
        """
        Append Pose2D message to CSV.
        topic_name: Topic name (treated as "message name" here)
        """
        timestamp = time.time()
        with self.lock:
            with open(self.filename, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([topic_name, timestamp, x, y, theta])


class UpdateSignal(QObject):
    """
    Signal for accepting GUI updates from other threads.
    Receives topic_name(str), formatted_msg(str) and passes them thread-safely to GUI update.
    """
    new_message = pyqtSignal(str, str)


class ROS2Pose2DMonitor(Node):
    """
    Node that subscribes only to geometry_msgs/msg/Pose2D type topics
    and saves received content to CSV.
    """
    def __init__(self, update_signal, csv_logger):
        super().__init__('ros2_pose2d_monitor')
        self.update_signal = update_signal
        self.csv_logger = csv_logger
        self.subscribers = {}

        # Periodically update topic list and subscribe only to Pose2D topics
        self.create_timer(1.0, self.update_topics)

    def update_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for topic, types in topic_names_and_types:
            if topic not in self.subscribers and "geometry_msgs/msg/Pose2D" in types:
                msg_type = get_message("geometry_msgs/msg/Pose2D")
                self.create_pose2d_subscriber(topic, msg_type)

    def create_pose2d_subscriber(self, topic_name, msg_type):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        def callback(msg):
            # Format received Pose2D message to JSON and pass to GUI
            msg_dict = {
                "x": msg.x,
                "y": msg.y,
                "theta": msg.theta
            }
            formatted_msg = json.dumps(msg_dict, indent=2)
            self.update_signal.new_message.emit(topic_name, formatted_msg)

            # Write to CSV
            self.csv_logger.log_pose2d(topic_name, msg.x, msg.y, msg.theta)

        sub = self.create_subscription(
            msg_type, topic_name, callback, qos
        )
        self.subscribers[topic_name] = sub


class ROS2GUI(QWidget):
    """
    Simple application (PyQt6 compatible) that monitors Pose2D type topics,
    displays their messages in GUI, and saves them to CSV.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Pose2D Monitor (Subscribe Only, CSV Logging) - PyQt6")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Topic", "Latest Pose2D"])
        self.layout.addWidget(self.table)
        self.setLayout(self.layout)

        self.table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeMode.Stretch)
        self.table.setWordWrap(True)

        self.update_signal = UpdateSignal()
        self.update_signal.new_message.connect(self.update_table)

        # Initialize CSV logger
        self.csv_logger = CSVLogger("pose2d_log.csv")

        # Start ROS node
        self.node = ROS2Pose2DMonitor(self.update_signal, self.csv_logger)
        self.ros_thread_stop_event = Event()
        self.ros_thread = Thread(target=self.run_ros_spin, daemon=True)
        self.ros_thread.start()

    def run_ros_spin(self):
        rclpy.spin(self.node)

    def closeEvent(self, event):
        # Stop ROS when closing window
        self.node.destroy_node()
        rclpy.shutdown()
        self.ros_thread_stop_event.set()
        super().closeEvent(event)

    def update_table(self, topic, msg):
        row = self.get_row_for_topic(topic)
        self.table.setItem(row, 0, QTableWidgetItem(topic))

        item = QTableWidgetItem(msg)
        # In PyQt6, enums like alignment use AlignmentFlag
        item.setTextAlignment(Qt.AlignmentFlag.AlignTop)

        # Make item selectable only, not editable
        flags = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable
        item.setFlags(flags)

        # QFontMetrics: use TextFlag reference for boundingRect signature
        font_metrics = QFontMetrics(self.table.font())
        text_rect = font_metrics.boundingRect(
            0, 0,
            self.table.columnWidth(1),
            0,
            int(Qt.TextFlag.TextWordWrap),
            msg
        )
        item.setSizeHint(QSize(self.table.columnWidth(1), text_rect.height() + 10))

        self.table.setItem(row, 1, item)
        self.table.resizeRowsToContents()

    def get_row_for_topic(self, topic):
        for row in range(self.table.rowCount()):
            if self.table.item(row, 0) and self.table.item(row, 0).text() == topic:
                return row
        self.table.insertRow(self.table.rowCount())
        return self.table.rowCount() - 1


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = ROS2GUI()
    gui.show()
    exit_code = app.exec()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
