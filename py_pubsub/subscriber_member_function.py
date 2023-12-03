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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import numpy as np

import serial

ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
ser.reset_input_buffer()

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber') # type: ignore
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.ranges)
        DISTANCE_THRESHOLD = 0.25
        NUM_DIVISIONS = 5
        
        full_ranges = np.array(msg.ranges)
        num_points = len(full_ranges)
        
        front_range = full_ranges[-round(num_points/2**NUM_DIVISIONS):] + full_ranges[:round(num_points/2**NUM_DIVISIONS)]
        left_range = full_ranges[round(num_points/2**NUM_DIVISIONS)*NUM_DIVISIONS*4:round(num_points/2**NUM_DIVISIONS)*(NUM_DIVISIONS+1)*4]
        
        is_front = np.nanmean(front_range) <= DISTANCE_THRESHOLD
        is_left = np.nanmean(left_range) <= DISTANCE_THRESHOLD
        
        out_data = ""
        if is_front:
            out_data = "y"
        elif is_left:
            out_data = str(np.nanmean(left_range))
        else:
            out_data = "n"
        
        ser.write((out_data + "\n").encode("utf-8"))
        output_line = ser.readline().decode('utf-8').rstrip()
        print(output_line)            


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
