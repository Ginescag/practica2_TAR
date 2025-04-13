#!/usr/bin/env python3
import rospy
import sys
import math
from math import sqrt
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

    
    def movimiento_0(self):
        # Dibujar linea recta
        self.move_distance(2.0)

    def movimiento_1(self):
        # Dibujar triangulo
        for _ in range(3):
            self.move_distance(3.0)
            self.rotate_angle(-2 * math.pi / 3)  # 120°

    def movimiento_2(self):
        # Dibujar cuadrado
        for _ in range(4):
            self.move_distance(1.0)
            self.rotate_angle(math.pi / -2)  # 90°

    def movimiento_3(self):
        # Dibujar infinito
        self.move_distance(0.5)
        self.rotate_angle(3 * (math.pi / -4)) #135º
        self.move_distance(sqrt(2*(0.5**2)))
        self.rotate_angle(3 * (math.pi / 4))

        self.move_distance(0.5)
        self.rotate_angle(3 * (math.pi/ 4))
        self.move_distance(sqrt(2*(0.5 * 0.5)))
        self.rotate_angle(3* (math.pi/ -4))

    def movimiento_4(self):
        # Dibujar cuadrado 10 veces
        for _ in range(10):
            self.movimiento_2()

    def ejecutar_movimiento(self, modo):
        if modo == 0:
            self.movimiento_0()
        elif modo == 1:
            self.movimiento_1()
        elif modo == 2:
            self.movimiento_2()
        elif modo == 3:
            self.movimiento_3()
        elif modo == 4:
            self.movimiento_4()

def main():
    nodo = MovimientoTurtlebot()

    if len(sys.argv) < 2:
        print("Uso: rosrun p3_pkg movimiento.py [modo]")
        print("   0 -> Avanzar 2m")
        print("   1 -> Triángulo equilátero 3m")
        print("   2 -> Cuadrado 1m")
        print("   3 -> Infinito")
        print("   4 -> Cuadrado 1m 10 veces")
        sys.exit(1)

    modo = int(sys.argv[1])

    # Esperamos un par de segundos a que se lean valores iniciales de odometría
    rospy.sleep(2.0)

    # Ejecutar el movimiento
    nodo.ejecutar_movimiento(modo)

if __name__ == "__main__":
    main()
