#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class MazeSolver:
    def __init__(self):
        # Inicialización del nodo
        rospy.init_node('maze_solver', anonymous=True)

        # Se suscribe al topic /scan para obtener las mediciones del LiDAR
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)


        #PARA TRABAJAR CON ESTE CODIGO CON EL TB2, DESCOMENTAMOS ESTA LINEA
        #self.cmd_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        

        #Y COMENTAMOS ESTA (CAMBIAN LOS TOPICOS DE ORDENES DE MOVIMIENTO ENTRE EL TB3 Y EL TB2)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


        # Variable para almacenar el último mensaje de scan
        self.scan_data = None

        # Configuración de la frecuencia de actualización
        self.rate = rospy.Rate(10)  # 10 Hz

        # Parámetros de velocidad
        self.forward_speed = 0.2     # m/s
        self.angular_speed = 1     # rad/s
        self.safe_distance = 0.3     # Distancia de seguridad en metros

        # Selección de la estrategia de seguimiento de pared: 'right' o 'left'
        self.wall_following = 'right'
    
    def scan_callback(self, data):
        # Almacena la información del LiDAR para su procesamiento
        self.scan_data = data
        rospy.loginfo("Scan recibido, seq: {}".format(data.header.seq))

    def get_ranges(self, start_angle, end_angle):
        """
        Devuelve las mediciones del LiDAR correspondientes a un rango angular.
        Se eliminan los valores que no sean numéricos (nan o inf).
        """
        if self.scan_data is None:
            return np.array([])
        angle_min = self.scan_data.angle_min
        increment = self.scan_data.angle_increment
        num_points = len(self.scan_data.ranges)

        # Se calculan los índices inicial y final de la región deseada
        start_index = int((start_angle - angle_min) / increment)
        end_index = int((end_angle - angle_min) / increment)
        start_index = max(0, start_index)
        end_index = min(num_points - 1, end_index)

        ranges = np.array(self.scan_data.ranges[start_index:end_index])
        # Se filtran los valores no válidos
        valid_ranges = ranges[np.isfinite(ranges)]
        return valid_ranges

    def run(self):
        while not rospy.is_shutdown():
            if self.scan_data is None:
                rospy.loginfo("Esperando datos del LiDAR...")
                self.rate.sleep()
                continue

            # Procesamiento de los sectores:
            # Sector frontal: ±10° (±0.1745 rad)
            front_data = self.get_ranges(-0.1745, 0.1745)
            front_distance = np.min(front_data) if front_data.size > 0 else float('inf')

            # Sector derecho: -30° a -15° (aprox. -0.5236 a -0.2618 rad)
            right_data = self.get_ranges(-0.5236, -0.2618)
            right_distance = np.min(right_data) if right_data.size > 0 else float('inf')

            # Sector izquierdo: 15° a 30° (0.2618 a 0.5236 rad)
            left_data = self.get_ranges(0.2618, 0.5236)
            left_distance = np.min(left_data) if left_data.size > 0 else float('inf')

            # Inicialización del comando de movimiento
            move_cmd = Twist()

            # Evaluación para evitar colisiones:
            if front_distance < self.safe_distance:
                # Obstáculo detectado en el frente, se detiene el avance y se gira.
                move_cmd.linear.x = 0.0

                # Se decide la dirección de giro comparando las distancias laterales.
                # Alternativamente se puede imponer la estrategia de "seguir la mano derecha" o "seguir la mano izquierda"
                if left_distance > right_distance:
                    move_cmd.angular.z = self.angular_speed
                else:
                    move_cmd.angular.z = -self.angular_speed
            else:
                # Camino libre en el frente: avanzar
                move_cmd.linear.x = self.forward_speed
                # Corrección para mantener la pared a un rango deseado (seguimiento de pared)
                if self.wall_following == 'right':
                    if right_distance > self.safe_distance:
                        # Si la pared se aleja, gira ligeramente a la derecha para acercarla
                        move_cmd.angular.z = -0.2
                    elif right_distance < self.safe_distance * 0.8:
                        # Si la pared está demasiado cerca, gira ligeramente a la izquierda
                        move_cmd.angular.z = 0.2
                    else:
                        move_cmd.angular.z = 0.0
                elif self.wall_following == 'left':
                    if left_distance > self.safe_distance:
                        move_cmd.angular.z = 0.1
                    elif left_distance < self.safe_distance * 0.8:
                        move_cmd.angular.z = -0.1
                    else:
                        move_cmd.angular.z = 0.0

            # Se publica el comando de velocidad
            self.cmd_pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        solver = MazeSolver()
        solver.run()
    except rospy.ROSInterruptException:
        pass
