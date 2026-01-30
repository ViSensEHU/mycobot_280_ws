#!/usr/bin/env python3
import time
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from packaging import version

# --- LÓGICA DEL FABRICANTE: Verificación de Versión ---
import pymycobot
MIN_REQUIRE_VERSION = '3.6.1'
current_version = pymycobot.__version__
print(f'Versión actual de pymycobot: {current_version}')

if version.parse(current_version) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(f'La versión de pymycobot debe ser {MIN_REQUIRE_VERSION} o superior.')
else:
    from pymycobot import MyCobot280

class EroboticsInterface(Node):
    def __init__(self):
        super().__init__('erobotics_interface_node')
        
        # --- 1. CONEXIÓN ROBUSTA (Estilo Fabricante) ---
        self.get_logger().info("Buscando robot conectado...")
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        
        port = ""
        if self.robot_m5:
            port = self.robot_m5
        elif self.robot_wio:
            port = self.robot_wio
            
        if not port:
            self.get_logger().error("¡No se encontró ningún puerto (ttyUSB/ttyACM)!")
            # En producción, aquí podrías lanzar una excepción o salir
        else:
            self.get_logger().info(f"Robot encontrado en puerto: {port}, baud: 115200")
            try:
                self.mc = MyCobot280(port, 115200)
                time.sleep(0.05)
                # Resetear modo si es necesario (del script original)
                if self.mc.get_fresh_mode() == 0:
                    self.mc.set_fresh_mode(1)
                    time.sleep(0.05)
                self.get_logger().info("¡Conexión con MyCobot280 exitosa!")
                # Ir a una posición segura inicial (Opcional)
                # self.mc.send_angles([0,0,0,0,0,0], 30)
            except Exception as e:
                self.get_logger().error(f"Error al conectar con el robot: {e}")

        # --- 2. CONFIGURACIÓN DE ROS2 ---
        
        # Nombres de las articulaciones. 
        # IMPORTANTE: Deben coincidir EXACTAMENTE con tu URDF.
        # El ejemplo del fabricante usaba estos nombres largos:
        self.joint_names = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]
        # SI TU URDF USA 'joint1', 'joint2'... DESCOMENTA LA SIGUIENTE LÍNEA:
        # self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Publicador de estado (Feedback real del robot hacia ROS)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joints) # 20Hz
        
        # Servidor de Acción (Para recibir órdenes de MoveIt)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info('Action Server listo para recibir trayectorias de MoveIt.')

    def publish_joints(self):
        """
        Lee los ángulos del robot real y los publica en /joint_states
        para que MoveIt/RViz sepan dónde está el robot realmente.
        """
        if not hasattr(self, 'mc'): return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        try:
            # get_angles() devuelve lista en GRADOS
            angles = self.mc.get_angles()
            
            if angles and len(angles) == 6:
                # Convertir Grados a Radianes para ROS
                radians = [math.radians(x) for x in angles]
                msg.position = radians
                self.joint_pub.publish(msg)
        except Exception as e:
            # Evitar saturar el log si hay error de lectura puntual
            pass

    def execute_callback(self, goal_handle):
        """
        Recibe la trayectoria planificada por MoveIt y la ejecuta en el robot.
        """
        self.get_logger().info('Ejecutando trayectoria...')
        
        # Obtener los puntos de la trayectoria
        trajectory_points = goal_handle.request.trajectory.points
        
        # NOTA: MoveIt envía puntos con tiempos (time_from_start).
        # Para una implementación perfecta, se debería interpolar.
        # Esta implementación básica recorre los puntos secuencialmente.
        
        for point in trajectory_points:
            target_pos_rad = point.positions
            
            # Convertir Radianes de ROS a Grados del Robot
            target_pos_deg = [math.degrees(x) for x in target_pos_rad]
            
            # Velocidad fija para moverse entre puntos (se puede ajustar)
            # En un driver avanzado, calcularíamos velocidad basada en el tiempo del punto.
            speed = 80 
            
            self.mc.send_angles(target_pos_deg, speed)
            
            # Espera simple para dar tiempo al movimiento.
            # Ajustar según la densidad de puntos de la trayectoria.
            time.sleep(0.001) 

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = EroboticsInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()