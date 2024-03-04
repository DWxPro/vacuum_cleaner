#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import time

class BumperDistance(Node):

    def __init__(self):
        super().__init__("bumper_distance")

        self.settings_left =  {"field_of_view": 30.0, "min_range": 0.0, "max_range": 1.0, "frame_id": "distance_sensor_left"}
        self.settings_front = {"field_of_view": 30.0, "min_range": 0.0, "max_range": 1.0, "frame_id": "distance_sensor_front"}
        self.settings_right = {"field_of_view": 30.0, "min_range": 0.0, "max_range": 1.0, "frame_id": "distance_sensor_right"}
        
        self.scale_factor = 0.0001

        self.distance_left_pub  = self.create_publisher(Range, "/bumper/distance/left",  10)
        self.distance_front_pub = self.create_publisher(Range, "/bumper/distance/front", 10)
        self.distance_right_pub = self.create_publisher(Range, "/bumper/distance/right", 10)
        
        self.ser = serial.Serial('/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_304C61E62BB46493-if00', 9600, timeout=1)
        
        self.get_logger().info("node \"bumper_distance\" has been started")
              
    def read_serial_data(self):
        while True:
            if self.ser.in_waiting > 0:
                serial_data = self.ser.readline().decode('utf-8').split(" ")
                range_left  = float(serial_data[1]) * self.scale_factor
                range_front = float(serial_data[3]) * self.scale_factor
                range_right = float(serial_data[5]) * self.scale_factor
                
                self.publish_distance(self.distance_left_pub,  self.settings_left,  range_left)
                self.publish_distance(self.distance_front_pub, self.settings_front, range_front)
                self.publish_distance(self.distance_right_pub, self.settings_right, range_right)

                self.get_logger().info(f"left: {range_left} front: {range_front} right: {range_right}")
            time.sleep(0.02)

    def publish_distance(self, pub, settings, range):
        msg = Range()
        msg.radiation_type = 1
        msg.field_of_view = settings["field_of_view"]
        msg.min_range = settings["min_range"]
        msg.max_range = settings["max_range"]
        msg.header.frame_id = settings["frame_id"]
        msg.range = range
        
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BumperDistance()

    try:
        node.read_serial_data()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

