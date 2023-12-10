# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from time import sleep
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import numpy as np

import serial


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')  # type: ignore
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=0.01)
        self.ser.reset_input_buffer()
        self.calibrated = False

    def listener_callback(self, msg):
        while not self.calibrated:
            output_line = self.ser.readline().decode().rstrip()
            if output_line:
                print(output_line)
            self.calibrated = output_line == "Done calibrate!"

        DISTANCE_THRESHOLD = 0.5
        LEFT_DISTANCE_RANGE = (0.1, 0.4)

        full_ranges = np.array(msg.ranges)
        num_points = len(full_ranges)

        front_range = full_ranges[-round(num_points/32):] + \
            full_ranges[:round(num_points/32)]
        left_range = full_ranges[round(
            num_points*23/32):round(num_points*25/32)]

        is_front = (np.nanmean(front_range) <= DISTANCE_THRESHOLD) and not (
            np.nanmean(front_range) == np.nan)
        is_left = LEFT_DISTANCE_RANGE[0] <= np.nanmean(left_range) <= LEFT_DISTANCE_RANGE[1] and not (
            np.nanmean(left_range) == np.nan)
        is_close_left = np.nanmean(left_range) < LEFT_DISTANCE_RANGE[0] and not (
            np.nanmean(left_range) == np.nan)
        is_far_left = LEFT_DISTANCE_RANGE[1] < np.nanmean(left_range) and not (
            np.nanmean(left_range) == np.nan)

        out_data = "y" if is_front else "n"
        if is_left:
            out_data += "l"
        elif is_close_left:
            out_data += "c"
        elif is_far_left:
            out_data += "f"

        self.ser.write(f"{out_data}\n".encode())
        self.get_logger().info(f'Sending: {out_data}')

        # sleep(0.1)

        output_line = self.ser.readline().decode().strip()
        if output_line:
            self.get_logger().info(f'Received: {output_line}')
        # output_line = self.ser.readline().decode().strip()
        # self.get_logger().info(f'Received: {output_line}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
