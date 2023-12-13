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

import os
import shutil
from time import sleep
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import numpy as np

import serial
import cv2
import matplotlib.pyplot as plt

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
        self.cam = cv2.VideoCapture(0)
        # sleep(3)
    
    def capture_frames(self, frames_dir="frames", file_name="frame"):        
        while True:
            ret, frame = self.cam.read()
        
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if cv2.countNonZero(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)) > 0:
                plt.imsave(f"{frames_dir}/{file_name}.jpg", frame)
                break

    def getSign(self, frame, signs, threshold=0.7, signs_dir="traffic_signs"):        
        templates = [cv2.imread(f"{signs_dir}/{sign}.png", 0) for sign in signs]
        all_signs = []
        for scint in range(100, 400, 10):
            scale = scint/100.0
            for stemp in templates:
                all_signs.append((cv2.resize(stemp, (int(64*scale), int(64*scale))), scale))
                
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = 0
        curMaxVal = 0
        curMaxTemplate = -1
        curMaxLoc = (0, 0)
        curScale = 0
        for (template, scale) in all_signs:
            res = cv2.matchTemplate(frame_gray, template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            if max_val > threshold and max_val > curMaxVal:
                curMaxVal = max_val
                curMaxTemplate = c
                curMaxLoc = max_loc
                curScale = scale
            c = c + 1
        if curMaxTemplate == -1:
            return (-1, (0, 0), 0, 0)
        else:
            return (curMaxTemplate % len(templates), curMaxLoc, curScale, curMaxVal)        

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

        output_line = self.ser.readline().decode().strip()
        if output_line:
            self.get_logger().info(f'Received: {output_line}')
            if output_line == "c":
                
                # frames capture are in /frames/frame[1-10].jpg
                # return: s: stop, l: lefxt, r: right
                signs = {"stop": 0, "left": 0, "right": 0, "around": 0}
                
                for _ in range(5):
                    curSign = list(signs)[template]
                    self.capture_frames()
                    frame = cv2.imread(f"frames/frame.jpg")
                    template = -1
                    (template, top_left, scale, val) = self.getSign(frame, list(signs))
                    if template != -1:
                        bottom_right = (top_left[0] + int(64*scale),
                                        top_left[1] + int(64*scale))
                        cv2.rectangle(frame, top_left, bottom_right, 255, 2)
                        cv2.putText(frame, str(round(val, 5)), (20, 350),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                        cv2.putText(frame, curSign, (20, 450),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
                        print("Detected:", curSign)
                        signs[curSign] += 1
                    else:
                        print("Sign not detected!")
                
                out_data = max(signs, key=lambda a: signs[a])[0]
                self.ser.write(f"{list(signs)[template][0]}\n".encode())
                self.get_logger().info(f'Sending: {list(signs)[template]}')
                while (output_line:=self.ser.readline().decode().strip()) != out_data:
                    print("Wrong output:", output_line)
                    sleep(0.01)
                self.get_logger().info(f'Received: {output_line}')
        else:
            out_data = "y" if is_front else "n"
            if is_left:
                out_data += "l"
            elif is_close_left:
                out_data += "c"
            elif is_far_left:
                out_data += "f"
                
            self.ser.write(f"{out_data}\n".encode())
            self.get_logger().info(f'Sending: {out_data}')

            output_line = self.ser.readline().decode().strip()
            if output_line:
                self.get_logger().info(f'Received: {output_line}')


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
