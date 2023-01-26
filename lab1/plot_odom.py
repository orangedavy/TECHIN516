#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from matplotlib.animation import FuncAnimation


class Visualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.xy, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(-3,3)
        self.ax.set_ylim(-3,3)
        return self.xy

    def getXY(self, pose):
        # TODO: Add in the array position the correct information from the message msg.pose
        # You may need to use rosmsg info in the terminal
        position = pose.pose.positions
        self.x_data.append(position.x)
        self.y_data.append(position.y)

    def visualize_callback(self, msg):
        self.getXY(msg.pose)

    def update_plot(self, frame):
        self.xy.set_data(self.x_data, self.y_data)
        return self.xy


def main():
    # TODO: Create a subscriber node that subscribe to the odometry topic, it uses the visualize_callback method in the Visualizer class as the callback function
    vis = Visualizer()
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)

    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/odom', Odometry, vis.visualize_callback)
    rospy.spin()

    plt.show(block=True)

if __name__ == '__main__':
    main()
