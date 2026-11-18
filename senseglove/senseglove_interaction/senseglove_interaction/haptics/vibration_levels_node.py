#!/usr/bin/env python3
"""
Continuous vibration levels → Nova 2 custom waveforms.

Subscribes: vibration_commands (std_msgs/Float64MultiArray), relative to the glove namespace
    data = [thumb_tip, index_tip, palm_index, palm_pinky]                        levels 0-100
    data = [thumb_tip, index_tip, palm_index, palm_pinky, frequency_hz, pulse_s]  optional waveform overrides
Publishes:  vibration_waveform (senseglove_msgs/Nova2WaveformCommand)

Every message re-triggers a short pulse on each motor above min_level. The
publisher must send faster than 1 / pulse_s for the vibration to feel
continuous; it fades out on its own pulse_s after the commands stop.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from senseglove_msgs.msg import Nova2WaveformCommand

MOTORS = (
    Nova2WaveformCommand.MOTOR_THUMB_TIP,
    Nova2WaveformCommand.MOTOR_INDEX_TIP,
    Nova2WaveformCommand.MOTOR_PALM_INDEX,
    Nova2WaveformCommand.MOTOR_PALM_PINKY,
)


class VibrationLevelsNode(Node):
    def __init__(self):
        super().__init__('vibration_levels_node')

        self.declare_parameter('min_level', 1.0)
        self.declare_parameter('frequency_hz', 180.0)
        self.declare_parameter('pulse_s', 0.1)
        self.declare_parameter('decay_s', 0.02)

        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Nova2WaveformCommand, 'vibration_waveform', 10)
        self._sub = self.create_subscription(
            Float64MultiArray, 'vibration_commands', self._callback, sub_qos)

        self.get_logger().info(
            f"Vibration levels: {self._sub.topic_name} -> {self._pub.topic_name}")

    def _callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) not in (4, 6):
            self.get_logger().warn(
                f"Expected 4 or 6 vibration values, got {len(data)}",
                throttle_duration_sec=5.0)
            return

        frequency = float(self.get_parameter('frequency_hz').value)
        pulse = float(self.get_parameter('pulse_s').value)
        if len(data) == 6:
            frequency, pulse = float(data[4]), float(data[5])

        min_level = float(self.get_parameter('min_level').value)
        decay = float(self.get_parameter('decay_s').value)

        for motor, level in zip(MOTORS, data[:4]):
            if level <= min_level:
                continue
            cmd = Nova2WaveformCommand()
            cmd.motor_location = motor
            cmd.amplitude = float(min(level, 100.0) / 100.0)
            cmd.frequency_start = frequency
            cmd.frequency_end = frequency
            cmd.wave_type = Nova2WaveformCommand.WAVE_SINE
            cmd.attack_time = 0.0
            cmd.sustain_time = pulse
            cmd.decay_time = decay
            cmd.frequency_switch_time = 0.0
            cmd.frequency_switch_factor = 1.0
            cmd.repeat_amount = 1
            cmd.infinite = False
            self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VibrationLevelsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
