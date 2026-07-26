# main.py
# Main application entry point for spine_robot_gui ROS2 package.
# Entry point: spine_robot_gui.main:main
#
# Run:
#   ros2 run spine_robot_gui main

import sys
import rclpy
from PyQt5 import QtCore, QtWidgets

from spine_robot_gui.gui import Ui_MainWindow
from spine_robot_gui.gui_node import GuiNode
from spine_robot_gui.serial_node import SerialNode


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Needle Drive Controller")

        # Instantiate ROS2 nodes.
        # gui_node wires buttons and publishes commands to 'gui2ard'.
        # serial_node subscribes to 'gui2ard', forwards to Arduino,
        # and calls gui_node.reset_ui() when the Arduino sends "false".
        self.gui_node    = GuiNode(self.ui)
        self.serial_node = SerialNode(self.ui, self.gui_node)

        # Drive rclpy.spin_once() from a QTimer every 50ms so ROS2
        # callbacks are processed without blocking the Qt event loop.
        self._ros_timer = QtCore.QTimer(self)
        self._ros_timer.timeout.connect(self._spin_ros)
        self._ros_timer.start(50)

    def _spin_ros(self):
        """Process any pending ROS2 callbacks for both nodes."""
        rclpy.spin_once(self.gui_node,    timeout_sec=0)
        rclpy.spin_once(self.serial_node, timeout_sec=0)

    def closeEvent(self, event):
        """Clean up on window close."""
        self._ros_timer.stop()
        self.gui_node.destroy_node()
        self.serial_node.destroy_node()
        event.accept()


def main(args=None):
    """ROS2 entry point — called by `ros2 run spine_robot_gui main`."""
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()

    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
