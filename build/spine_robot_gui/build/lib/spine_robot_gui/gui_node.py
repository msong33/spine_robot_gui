# gui_node.py
# ROS2 node for the GUI.
# Publishes user commands to the 'gui2ard' topic, which serial_node
# subscribes to and forwards to the Arduino Mega via serial.
#
# Command format published to 'gui2ard':
#   "1\n" through "7\n"     -> Setup page single-action commands
#   "8,<cycles>,<speed>\n"  -> Forward needle drive
#   "9,<cycles>,<speed>\n"  -> Backward needle drive
#   "S"                     -> Stop current task (no newline)

import rclpy
from rclpy.node import Node
from functools import partial
from std_msgs.msg import String


class GuiNode(Node):
    def __init__(self, ui):
        super().__init__('gui_node')
        self.ui = ui

        # Publishes commands to serial_node via 'gui2ard' topic
        self.gui_publisher = self.create_publisher(String, 'gui2ard', 10)

        # Needle distance constant (mm per cycle)
        self.needle_distance_per_cycle = 2

        self._wire_buttons()
        self._wire_spinboxes()
        self._init_ui_state()

    # ------------------------------------------------------------------
    # Wiring
    # ------------------------------------------------------------------

    def _wire_buttons(self):
        ui = self.ui

        # Page 2: Setup
        ui.p2_TopClampBtnTighten.clicked.connect( partial(self.setup_callback, 1))
        ui.p2_TopClampBtnRelease.clicked.connect( partial(self.setup_callback, 2))
        ui.p2_BotClampBtnTighten.clicked.connect( partial(self.setup_callback, 3))
        ui.p2_BotClampBtnRelease.clicked.connect( partial(self.setup_callback, 4))
        ui.p2_ScrewBtnContract.clicked.connect(   partial(self.setup_callback, 5))
        ui.p2_ScrewBtnExtend.clicked.connect(     partial(self.setup_callback, 6))
        ui.p2_BtnReleaseAllClamps.clicked.connect(partial(self.setup_callback, 7))
        ui.p2_StopWgtBtn.clicked.connect(self.stop_pub_callback)

        # Page 3: Forward Needle Drive
        ui.p3_BtnTaskRun.clicked.connect(partial(self.drive_callback, 8))
        ui.p3_StopWgtBtn.clicked.connect(self.stop_pub_callback)

        # Page 4: Backward Needle Drive
        ui.p4_BtnTaskRun.clicked.connect(partial(self.drive_callback, 9))
        ui.p4_StopWgtBtn.clicked.connect(self.stop_pub_callback)

    def _wire_spinboxes(self):
        # Update needle distance label when cycle count changes
        self.ui.p3_CyclesSpinBox.valueChanged.connect(self._update_needle_distance)
        self.ui.p4_CyclesSpinBox.valueChanged.connect(self._update_needle_distance)

    def _init_ui_state(self):
        """Set initial UI state: hide all task-running indicators."""
        self.ui.p2_LblTaskRunning.hide()
        self.ui.p3_LblTaskRunning.hide()
        self.ui.p3_LblTaskRunningUpdate.hide()
        self.ui.p4_LblTaskRunning.hide()
        self.ui.p4_LblTaskRunningUpdate.hide()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def setup_callback(self, case):
        """Publish a single-action setup command (cases 1-7) to the Arduino."""
        msg = String()
        msg.data = f"{case}\n"
        self.gui_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data.strip()}")

        # Lock UI while task runs
        self.ui.selectFunctionComboBox.setEnabled(False)
        self.ui.p2_TopClampBtnTighten.setEnabled(False)
        self.ui.p2_TopClampBtnRelease.setEnabled(False)
        self.ui.p2_BotClampBtnTighten.setEnabled(False)
        self.ui.p2_BotClampBtnRelease.setEnabled(False)
        self.ui.p2_ScrewBtnContract.setEnabled(False)
        self.ui.p2_ScrewBtnExtend.setEnabled(False)
        self.ui.p2_BtnReleaseAllClamps.setEnabled(False)
        self.ui.p2_LblTaskRunning.show()
        self.ui.p2_StopWgtBtn.setEnabled(True)

    def drive_callback(self, case):
        """Publish a drive command (case 8 or 9) with cycles and speed to the Arduino."""
        if case == 8:
            cycles = int(self.ui.p3_CyclesSpinBox.value())
            speed  = int(self.ui.p3_SpeedSldr.value())
        else:
            cycles = int(self.ui.p4_CyclesSpinBox.value())
            speed  = int(self.ui.p4_SpeedSldr.value())

        # Validate before sending
        if cycles == 0:
            self.get_logger().warn("Drive command ignored: cycles = 0")
            return
        if speed == 0:
            self.get_logger().warn("Drive command ignored: speed = 0")
            return

        msg = String()
        msg.data = f"{case},{cycles},{speed}\n"
        self.gui_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data.strip()}")

        # Lock UI while task runs
        self.ui.selectFunctionComboBox.setEnabled(False)

        if case == 8:
            self.ui.p3_CyclesSpinBox.setEnabled(False)
            self.ui.p3_SpeedSldr.setEnabled(False)
            self.ui.p3_BtnTaskRun.setEnabled(False)
            self.ui.p3_LblTaskRunningUpdate.setText(f"(0 of {cycles} cycles complete)")
            self.ui.p3_LblTaskRunning.show()
            self.ui.p3_LblTaskRunningUpdate.show()
            self.ui.p3_StopWgtBtn.setEnabled(True)

        elif case == 9:
            self.ui.p4_CyclesSpinBox.setEnabled(False)
            self.ui.p4_SpeedSldr.setEnabled(False)
            self.ui.p4_BtnTaskRun.setEnabled(False)
            self.ui.p4_LblTaskRunningUpdate.setText(f"(0 of {cycles} cycles complete)")
            self.ui.p4_LblTaskRunning.show()
            self.ui.p4_LblTaskRunningUpdate.show()
            self.ui.p4_StopWgtBtn.setEnabled(True)

    def stop_pub_callback(self):
        """Publish stop signal 'S' to interrupt any running task."""
        msg = String()
        msg.data = "S"
        self.gui_publisher.publish(msg)
        self.get_logger().info("Publishing: STOP")

        # Disable stop buttons immediately — full reset happens when
        # serial_node receives "false" from the Arduino
        self.ui.p2_StopWgtBtn.setEnabled(False)
        self.ui.p3_StopWgtBtn.setEnabled(False)
        self.ui.p4_StopWgtBtn.setEnabled(False)

    # ------------------------------------------------------------------
    # UI helpers
    # ------------------------------------------------------------------

    def _update_needle_distance(self):
        """Update needle distance labels based on current cycle spinbox values."""
        p3_dist = int(self.ui.p3_CyclesSpinBox.value()) * self.needle_distance_per_cycle
        p4_dist = int(self.ui.p4_CyclesSpinBox.value()) * self.needle_distance_per_cycle
        self.ui.p3_CyclesLblDistance.setText(str(p3_dist))
        self.ui.p4_CyclesLblDistance.setText(str(p4_dist))

    def reset_ui(self):
        """
        Restore all UI elements to their idle state.
        Called by serial_node when the Arduino sends 'false'.
        """
        ui = self.ui
        ui.selectFunctionComboBox.setEnabled(True)

        # Page 2 Setup
        ui.p2_TopClampBtnTighten.setEnabled(True)
        ui.p2_TopClampBtnRelease.setEnabled(True)
        ui.p2_BotClampBtnTighten.setEnabled(True)
        ui.p2_BotClampBtnRelease.setEnabled(True)
        ui.p2_ScrewBtnContract.setEnabled(True)
        ui.p2_ScrewBtnExtend.setEnabled(True)
        ui.p2_BtnReleaseAllClamps.setEnabled(True)
        ui.p2_LblTaskRunning.hide()
        ui.p2_StopWgtBtn.setEnabled(False)

        # Page 3 Forward Drive
        ui.p3_CyclesSpinBox.setEnabled(True)
        ui.p3_SpeedSldr.setEnabled(True)
        ui.p3_BtnTaskRun.setEnabled(True)
        ui.p3_LblTaskRunning.hide()
        ui.p3_LblTaskRunningUpdate.hide()
        ui.p3_StopWgtBtn.setEnabled(False)

        # Page 4 Backward Drive
        ui.p4_CyclesSpinBox.setEnabled(True)
        ui.p4_SpeedSldr.setEnabled(True)
        ui.p4_BtnTaskRun.setEnabled(True)
        ui.p4_LblTaskRunning.hide()
        ui.p4_LblTaskRunningUpdate.hide()
        ui.p4_StopWgtBtn.setEnabled(False)
