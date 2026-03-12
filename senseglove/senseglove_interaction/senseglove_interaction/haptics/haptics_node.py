#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient as ParameterClient
from rclpy.duration import Duration as RclDuration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from std_msgs.msg import Header, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HapticsNode(Node):
    def __init__(self):
        super().__init__('senseglove_haptics_node')
        self.get_logger().info('Initializing haptics node...')

        self.declare_parameter('controller_node', '/senseglove/glove0/lh/haptics_controller')
        self.declare_parameter('publish_topic', '/senseglove/glove0/lh/haptics_controller/joint_trajectory')
        self.declare_parameter('subscribe_topic', 'haptics_commands')
        self.declare_parameter('publish_rate', 60)
        self.declare_parameter('default_efforts', [0.0, 0.0, 0.0, 0.0,
                                                   0.0, 0.0,
                                                   0.0, 0.0, 0.0])

        self.controller_node = self.get_parameter('controller_node').value
        self.publish_topic = self.get_parameter('publish_topic').value
        subscribe_topic = self.get_parameter('subscribe_topic').value
        self.publish_rate = int(self.get_parameter('publish_rate').value)
        self.current_efforts = list(self.get_parameter('default_efforts').value)

        self.joint_names = self._fetch_joint_list()

        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        sub_qos = qos_profile_sensor_data
        sub_qos.depth = 1

        self.pub = self.create_publisher(JointTrajectory, self.publish_topic, pub_qos)
        self.sub = self.create_subscription(
            Float64MultiArray, subscribe_topic, self._callback, sub_qos)

        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Publishing to {self.publish_topic}")
        self.get_logger().info(f"Subscribed to {subscribe_topic}")
        self.get_logger().info("Joints:\n  " + "\n  ".join(self.joint_names))

    def _fetch_joint_list(self):
        self.get_logger().info(f"Buscando el servidor de parámetros de {self.controller_node}...")
        
        temp_node = rclpy.create_node('temp_param_client_node')
        client = ParameterClient(temp_node, self.controller_node)

        while not client.wait_for_services(timeout_sec=2.0):
            self.get_logger().info(f"Esperando a que el controlador {self.controller_node} inicie...")
            if not rclpy.ok():
                temp_node.destroy_node()
                return ['dummy']

        self.get_logger().info("¡Servicio encontrado! Solicitando parámetros...")
      
        future = client.get_parameters(['joints'])
        
        rclpy.spin_until_future_complete(temp_node, future)
        
        result = future.result()
        temp_node.destroy_node() 

        if result and result.values:
            joints = list(result.values[0].string_array_value)
            if joints:
                self.get_logger().info("¡Joints obtenidos con éxito!")
                return joints

        self.get_logger().warn(f"El controlador {self.controller_node} no tiene el parámetro 'joints' o está vacío.")
        return ['dummy']

    def _callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(
                f"Expected {len(self.joint_names)} values, got {len(msg.data)}")
            return
        self.current_efforts = list(msg.data)

    def _on_timer(self):
        self._apply_efforts(self.current_efforts)

    def _apply_efforts(self, efforts):
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        # PID gains (P=1.0) convert positions directly to effort commands
        point.positions = [float(x) for x in efforts]
        point.time_from_start = RclDuration(seconds=0.05).to_msg()
        traj.points.append(point)

        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = HapticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()