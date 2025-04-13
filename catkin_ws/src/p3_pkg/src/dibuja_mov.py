#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import math

class RecorridoPlotter:
    def __init__(self):
        rospy.init_node("dibuja_mov_node", anonymous=True)
        self.x_data = []
        self.y_data = []
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_data.append(x)
        self.y_data.append(y)

    def run(self):
        # Ponemos spin para que escuche hasta que se haga Ctrl+C
        rospy.spin()
        # Una vez terminamos (Ctrl+C en el terminal), generamos y mostramos la trayectoria
        self.plot_data()

    def plot_data(self):
        plt.figure()
        plt.title("Recorrido del Turtlebot")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.plot(self.x_data, self.y_data, marker='o')
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    plotter = RecorridoPlotter()
    plotter.run()
