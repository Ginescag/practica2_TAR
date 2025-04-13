#!/usr/bin/env python3
import rospy
import sys
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MovimientoTurtlebot:
    def __init__(self):
        rospy.init_node("movimiento_node", anonymous=True)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Estados actuales
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Ajustes de velocidad
        self.vel_lineal = 0.2
        self.vel_angular = 0.5

        # Control del bucle
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Convertir cuaterniones a ángulos de Euler
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

    def move_distance(self, distance):
        # Avanza en línea recta la distancia pasada por parámetro (en metros)
        inicio_x, inicio_y = self.x, self.y
        direction = 1 if distance >= 0.0 else -1
        twist = Twist()

        while not rospy.is_shutdown():
            dist_recorrida = math.sqrt((self.x - inicio_x)**2 + (self.y - inicio_y)**2)
            if dist_recorrida >= abs(distance):
                break
            twist.linear.x = direction * self.vel_lineal
            self.pub_vel.publish(twist)
            self.rate.sleep()

        # Parar
        twist.linear.x = 0.0
        self.pub_vel.publish(twist)

    def rotate_angle(self, angle_rads):
        # Gira el robot angle_rads (radianes)
        # Positivo = izquierda, negativo = derecha
        ang_inicial = self.yaw
        ang_objetivo = ang_inicial + angle_rads
        # Normalizar a [-pi, pi]
        ang_objetivo = math.atan2(math.sin(ang_objetivo), math.cos(ang_objetivo))
        twist = Twist()

        while not rospy.is_shutdown():
            error = ang_objetivo - self.yaw
            error = math.atan2(math.sin(error), math.cos(error))  # normalizar
            if abs(error) < 0.01:
                break
            giro_signo = 1 if error > 0 else -1
            twist.angular.z = giro_signo * self.vel_angular
            self.pub_vel.publish(twist)
            self.rate.sleep()

        # Parar
        twist.angular.z = 0.0
        self.pub_vel.publish(twist)

    def aparcar(self):
        self.move_distance(1.5)
        self.rotate_angle(math.pi / -2)
        self.move_distance(1.5)
        self.rotate_angle(-math.pi)

def main():
    nodo = MovimientoTurtlebot()

    rospy.sleep(2.0)

    nodo.aparcar()

if __name__ == "__main__":
    main()
