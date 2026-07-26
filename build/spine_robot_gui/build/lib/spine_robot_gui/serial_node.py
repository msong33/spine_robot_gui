# serial_node.py
# ROS2 node for serial communication with the Arduino Mega.
# Subscribes to 'gui2ard' topic (published by gui_node) and forwards
# commands to the Arduino. Reads Arduino responses and updates the GUI.
#
# Arduino response format:
#   "true"        -> Task started (GUI already locked by gui_node)
#   "false"       -> Task finished or stopped — triggers full GUI reset
#   "CYCLE:X,Y"   -> Cycle progress: X completed out of Y total

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String


class SerialNode(Node):
    def __init__(self, ui, gui_node):
        super().__init__('serial_node')
        self.ui = ui
        self.gui_node = gui_node  # Reference to call gui_node.reset_ui()

        # Connect to Arduino Mega serial port.
        # Update port if needed: '/dev/ttyUSB0' or 'COM3' on Windows.
        try:
            self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1.0)
            self.get_logger().info("Serial port opened: /dev/ttyACM1")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        # Subscribe to gui2ard topic to receive commands from gui_node
        self.ard_subscription = self.create_subscription(
            String, 'gui2ard', self.arduino_subscriber_callback, 10
        )

        # Timer: poll serial port every 100ms for Arduino responses
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Internal buffer for accumulating partial serial lines
        self._serial_buffer = ""

    # ------------------------------------------------------------------
    # Subscriber callback: forward GUI commands to Arduino
    # ------------------------------------------------------------------

    def arduino_subscriber_callback(self, msg):
        """Forward command string received from gui_node to the Arduino."""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn("Serial port not open — cannot send command.")
            return

        cmd = msg.data
        self.serial_port.write(cmd.encode())
        self.get_logger().info(f"Sent to Arduino: {cmd.strip()}")

    # ------------------------------------------------------------------
    # Timer callback: read and parse Arduino responses
    # ------------------------------------------------------------------

    def timer_callback(self):
        """Read available serial data from Arduino and update GUI."""
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            waiting = self.serial_port.in_waiting
            if waiting == 0:
                return

            raw = self.serial_port.read(waiting)
            decoded = raw.decode(errors='ignore')

            # Accumulate into buffer and process complete lines only
            self._serial_buffer += decoded
            while '\n' in self._serial_buffer:
                line, self._serial_buffer = self._serial_buffer.split('\n', 1)
                line = line.strip()
                if line:
                    self._handle_line(line)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")

    # ------------------------------------------------------------------
    # Line parser
    # ------------------------------------------------------------------

    def _handle_line(self, line: str):
        """Parse a complete line from the Arduino and update GUI state."""
        self.get_logger().info(f"From Arduino: {line}")

        if line == "true":
            # Task started — GUI already locked when button was pressed
            pass

        elif line == "false":
            # Task finished or stopped — reset entire GUI to idle state
            self.gui_node.reset_ui()

        elif line.startswith("CYCLE:"):
            # Format: "CYCLE:X,Y" e.g. "CYCLE:3,10" = 3 of 10 cycles done
            self._handle_cycle_update(line)

        # All other lines (Arduino debug output) are logged only

    def _handle_cycle_update(self, line: str):
        """Parse CYCLE:X,Y and update whichever drive page is active."""
        try:
            parts = line[6:].split(',')  # Strip "CYCLE:" prefix
            current = int(parts[0])
            total   = int(parts[1])
        except (IndexError, ValueError):
            self.get_logger().warn(f"Could not parse cycle update: {line}")
            return

        text = f"({current} of {total} cycles complete)"

        # Update whichever drive page's label is currently visible
        if self.ui.p3_LblTaskRunningUpdate.isVisible():
            self.ui.p3_LblTaskRunningUpdate.setText(text)
        elif self.ui.p4_LblTaskRunningUpdate.isVisible():
            self.ui.p4_LblTaskRunningUpdate.setText(text)
