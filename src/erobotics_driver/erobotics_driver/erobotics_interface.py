#!/usr/bin/env python3
import time
import math
import os
import rclpy
import numpy as np 
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from packaging import version
import pymycobot

# --- VERIFICACIÓN LIBRERÍA ---
MIN_REQUIRE_VERSION = '3.6.1'
current_version = pymycobot.__version__
if version.parse(current_version) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(f'Versión pymycobot insuficiente. Mínimo {MIN_REQUIRE_VERSION}')
else:
    from pymycobot import MyCobot280

class EroboticsInterface(Node):
    def __init__(self):
        super().__init__('erobotics_interface_node')
        
        # --- 1. CONEXIÓN ---
        self.get_logger().info("Buscando robot...")
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        port = self.robot_m5 if self.robot_m5 else self.robot_wio
        
        if not port:
            self.get_logger().error("¡Puerto no encontrado!")
        else:
            self.get_logger().info(f"Robot encontrado en puerto: {port}, baud: 115200")
            try:
                self.mc = MyCobot280(port, 115200)
                time.sleep(0.1)
                self.mc.set_fresh_mode(1) # IMPORTANTE: Modo refresco rápido
                time.sleep(0.1)
                self.get_logger().info("¡Conexión con MyCobot280 exitosa!")
                # Ir a una posición segura inicial (Opcional)
                self.mc.send_angles([0,0,0,0,0,45], 30) # La posición inicial con garra. En º
                self.get_logger().info(f"Conectado en {port}")
            except Exception as e:
                self.get_logger().error(f"Error conexión: {e}")

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
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.2, self.publish_joints) # 5Hz
        
        # Servidor de Acción (Para recibir órdenes de MoveIt)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info('Action Server listo para recibir trayectorias de MoveIt.')

    def publish_joints(self):
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
        except:
            pass

    def interpolate_point(self, points, current_time):
        """
        Encuentra los dos puntos entre los que estamos y calcula la posición intermedia.
        """

        excecMode = self.mc.get_fresh_mode()

        self.get_logger().info(f'Modo de ejecución: {excecMode}')

        # Si la trayectoria ha terminado o solo tiene un punto
        if current_time >= points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9:
            return [math.degrees(p) for p in points[-1].positions]
        
        # Buscar el segmento de tiempo actual
        for i in range(len(points) - 1):
            p_start = points[i]
            p_end = points[i+1]
            
            t_start = p_start.time_from_start.sec + p_start.time_from_start.nanosec * 1e-9
            t_end = p_end.time_from_start.sec + p_end.time_from_start.nanosec * 1e-9
            
            if t_start <= current_time <= t_end:
                # Estamos en este segmento. Calcular porcentaje de avance (alpha)
                duration = t_end - t_start
                if duration <= 0: return [math.degrees(p) for p in points[i].positions]
                
                alpha = (current_time - t_start) / duration
                
                # Interpolar cada articulación
                interpolated_deg = []
                for j in range(6):
                    start_pos = p_start.positions[j]
                    end_pos = p_end.positions[j]
                    # Interpolación lineal simple: inicio + (diferencia * porcentaje)
                    rad_val = start_pos + (end_pos - start_pos) * alpha
                    interpolated_deg.append(math.degrees(rad_val))
                
                return interpolated_deg
        
        return [math.degrees(p) for p in points[-1].positions]

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        points = trajectory.points
        
        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        self.get_logger().info(f'Iniciando interpolación. Duración total: {points[-1].time_from_start.sec}s')

        start_time = time.time()
        final_time = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
        
        # Bucle de Control (frecuencia fija)
        # 20 Hz = 0.05s. Suficiente para fluidez, no satura serial.
        control_period = 0.05 
        
        while True:
            now = time.time()
            elapsed = now - start_time
            
            # 1. Calcular ángulos para el instante actual
            target_deg = self.interpolate_point(points, elapsed)
            
            # 2. Enviar al robot (Velocidad alta porque son pasos pequeños)
            if hasattr(self, 'mc'):
                # Usamos set_angles o send_angles con velocidad alta
                # para que intente llegar al setpoint inmediatamente
                self.mc.send_angles(target_deg, 100)
            
            # 3. Salir si terminamos
            if elapsed > final_time:
                break
                
            # 4. Mantener el ritmo del bucle
            time.sleep(control_period)

        # Asegurar posición final exacta
        final_deg = [math.degrees(p) for p in points[-1].positions]
        if hasattr(self, 'mc'):
            self.mc.send_angles(final_deg, 80)
            
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = EroboticsInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()