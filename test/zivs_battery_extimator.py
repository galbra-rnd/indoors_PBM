#!/usr/bin/env python
import sys
import rospy
import cv2

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
import time
import numpy as np

IN_TOPIC = "/mavros/battery"
OUT_TOPIC = "/battery_estimator/time_left_secs"
usage = """ Usage:
Subscribe to /mavros/battery message. Extract cell voltage.
Publish /battery_estimator/time_left_secs
"""


class BatteryEstimator:

    def __init__(self):
        self.time_left_insecs_publisher = rospy.Publisher(
            OUT_TOPIC, Float32, queue_size=1)

        self.mavros_battery_sub = rospy.Subscriber(
            IN_TOPIC, BatteryState, self.mavros_battery_callback)

        self.num_of_cells = -1
        self.possible_battery_vals = None
        self.possible_time_of_flight_secs = None

        self.MAX_VOLT_PER_CELL = 4.2
        self.MIN_VOLT_PER_CELL = 3.2
        self.MAX_FLIGHT_TIME_MINUTES = 12

    def mavros_battery_callback(self, ros_data):
        cell_voltage = ros_data.cell_voltage
        secs = self.estimate(cell_voltage)

        msg = Float32()
        msg.data = secs
        self.time_left_insecs_publisher.publish(msg)

    def estimate(self, cell_voltage):
        """Dummy estimator 

        Args:
             cell_voltage (tuple): contains voltage per each cell.

        Returns:
            float: Estimated time left for flight in secs
        """

        if self.num_of_cells is -1:
            self.num_of_cells = len(cell_voltage)
            self.possible_battery_vals = np.random.uniform(low=self.MIN_VOLT_PER_CELL*self.num_of_cells,
                                                           high=self.MAX_VOLT_PER_CELL*self.num_of_cells, size=(50,))
            self.possible_battery_vals.sort()
            self.possible_time_of_flight_secs = np.random.uniform(
                low=0, high=self.MAX_FLIGHT_TIME_MINUTES*60, size=(50,))
            self.possible_time_of_flight_secs.sort()
            return -1

        over_all = sum(cell_voltage)

        x = np.interp(over_all, self.possible_battery_vals,
                      self.possible_time_of_flight_secs)

        return x


def main(args):
    be = BatteryEstimator()
    rospy.init_node('battery_estimator_mock', anonymous=True)
    rospy.loginfo(usage)
    rospy.loginfo("\ninitiating\n")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
