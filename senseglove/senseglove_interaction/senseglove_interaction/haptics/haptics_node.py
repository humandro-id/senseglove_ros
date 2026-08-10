#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RclDuration
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.srv import GetParameters

from std_msgs.msg import Header, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data



class HapticsNode(Node):
    def __init__(self):
        # 1. Permite declarar automáticamente los parámetros inyectados desde el launch.py
        super().__init__(
            'senseglove_haptics_node',
            automatically_declare_parameters_from_overrides=True
        )
        self.get_logger().info('Initializing haptics node...')

        # 2. Declarar parámetros opcionales/adicionales
        self.declare_parameter('hold_time', 2.0)
        # Esfuerzos en escala 0-100 (el hardware interface los normaliza a 0-1 para el SDK)
        self.declare_parameter('default_efforts', [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0])

        # 3. Leer los parámetros inyectados por launch.py
        self.controller_node = self.get_parameter('controller_node').value
        self.publish_topic = self.get_parameter('publish_topic').value
        subscribe_topic = self.get_parameter('subscribe_topic').value
        self.publish_rate = int(self.get_parameter('publish_rate').value)

        # Obtener la lista de articulaciones del controlador
        self.joint_names = self._fetch_joint_list()
        self.current_efforts = self._map_efforts_to_joints(
            list(self.get_parameter('default_efforts').value))

        pub_qos = qos_profile_sensor_data
        pub_qos.depth = 1

        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Publisher y Subscriber configurados con los tópicos recibidos del launch
        self.pub = self.create_publisher(
            JointTrajectory, self.publish_topic, pub_qos)
        self.sub = self.create_subscription(
            Float64MultiArray, subscribe_topic, self._callback, sub_qos)

        # Timer para enviar comandos periódicamente
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Publishing to: {self.publish_topic}")
        self.get_logger().info(f"Subscribed to: {subscribe_topic}")
        self.get_logger().info("Joints:\n  " + "\n  ".join(self.joint_names))
        self.get_logger().info("Playing default efforts")

    def _fetch_joint_list(self):
        """Bloquea hasta obtener la lista de joints del controlador.

        Reintenta indefinidamente: si el controlador todavía no está cargado
        (o aún no expone el parámetro 'joints'), espera y vuelve a intentar en
        lugar de continuar con una lista inválida.
        """
        service_name = f'{self.controller_node.rstrip("/")}/get_parameters'
        client = self.create_client(GetParameters, service_name)

        attempt = 0
        while rclpy.ok():
            attempt += 1

            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(
                    f"[intento {attempt}] Esperando el servicio {service_name}...")
                continue

            req = GetParameters.Request()
            req.names = ['joints']
            future = client.call_async(req)

            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                result = future.result()
                if result and result.values and result.values[0].string_array_value:
                    joints = list(result.values[0].string_array_value)
                    self.get_logger().info(
                        f"Joints obtenidos de {self.controller_node} "
                        f"tras {attempt} intento(s).")
                    return joints
            except Exception as e:
                self.get_logger().warn(f"Failed to fetch joints parameter: {e}")

            self.get_logger().warn(
                f"[intento {attempt}] El controlador {self.controller_node} aún no "
                "expone el parámetro 'joints' (o está vacío). Reintentando en 2 s...")
            time.sleep(2.0)

        # Solo se llega aquí si ROS se está cerrando durante la espera
        raise ExternalShutdownException()

    def _map_efforts_to_joints(self, values):
        """Adapta el array del robot al tamaño del haptics_controller.

        Acepta:
          - N valores, donde N == len(joint_names)
          - 9 valores en layout legacy:
              [thumb, index, middle, ring, thumb_buzz, index_buzz,
               palm_index_buzz, palm_pinky_buzz, palm_strap]
            → se toman FFB [0..3] + strap [8] cuando el controlador tiene 5 joints
        """
        n = len(self.joint_names)
        if len(values) == n:
            return [float(x) for x in values]

        if len(values) == 9 and n == 5:
            # Legacy 9 → controller 5 (FFB + strap). Buzzers [4..7] se ignoran aquí.
            return [float(values[i]) for i in (0, 1, 2, 3, 8)]

        if len(values) == 9 and n == 9:
            return [float(x) for x in values]

        self.get_logger().warn(
            f"Expected {n} or 9 effort values (legacy), got {len(values)}")
        return None

    def _callback(self, msg: Float64MultiArray):
        mapped = self._map_efforts_to_joints(list(msg.data))
        if mapped is None:
            return
        self.current_efforts = mapped

    def _on_timer(self):
        self._apply_efforts(self.current_efforts)

    def _apply_efforts(self, efforts):
        if len(efforts) != len(self.joint_names):
            self.get_logger().warn(
                f"Skipping publish: efforts size {len(efforts)} != "
                f"joints size {len(self.joint_names)}")
            return

        traj = JointTrajectory()
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.joint_names)
        point.effort = [float(x) for x in efforts]
        point.time_from_start = RclDuration(seconds=0.05).to_msg()
        traj.points.append(point)

        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HapticsNode()
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