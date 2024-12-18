#!/usr/bin/env python3

import rclpy
import tkinter
import threading
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self, canvas):
        super().__init__('lidar_subscriber')
        self.canvas = canvas  # Tkinter canvas to draw on
        self.angle_increment = None  # To store the angle increment from Lidar
        # Subscribe to /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10  # QoS depth
        )
        self.subscription
        self.get_logger().info("Lidar Subscriber Node started. Listening to /scan...")

    def lidar_callback(self, msg):
        self.angle_increment = msg.angle_increment 
        self.canvas.delete("all") 
        colors = ["blue", "red", "green", "yellow", "black", "white", "purple", "orange"]
        color_counter = 0
        center_x, center_y = 400, 400
        objects = []
        object_counter = 0
        cluster_counter = 0
        last_pos = 0
        counter = 1

        lines = []
        line_counter = 1
        line_average = 0

        # Loop through the Lidar data and separate mesurments by distance
        for i, pos in enumerate(msg.ranges):
            if pos != float('inf'):
                line_average = (pos + line_average) / line_counter
            if abs(pos-line_average) > 0.8 or pos == float('inf'):
                line_average = 0
                lines.append(line_counter)
                line_counter = 1
            else:
                line_counter += 1
            if abs(pos-last_pos) > 0.5 or pos == float('inf'):
                objects.append(counter)
                counter = 1
            else:
                counter += 1
            
            if pos == float('inf'):
                last_pos = 0
            else:
                last_pos = pos


        # Remove Ones from objects
        objects = [obj for obj in objects if obj != 1]
        self.get_logger().info(f"Received Objects Data: {objects}")
        self.get_logger().info(f"Received Lines Data: {lines}")

        
        # Loop through the Lidar data and draw circles based on distances
        for i, distance in enumerate(msg.ranges):
            # If we have drawn the current cluster, move to the next one
            if object_counter < len(objects) and cluster_counter == objects[object_counter]:
                color_counter = (color_counter + 1) % len(colors)  
                object_counter += 1
                cluster_counter = 0
            
            if distance != float('inf'):
                angle = msg.angle_min + i * self.angle_increment
                x = center_x + distance * math.cos(angle) * 100
                y = center_y + distance * math.sin(angle) * 100
                # Draw a small circle at the calculated position
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill=colors[color_counter])

                cluster_counter += 1 
        
        # Draw lines based on the 'lines' list
        start_index = 0
        for length in lines:
            if length == 1:
                start_index += 1
                continue  # Skip lines with a length of 1

            end_index = start_index + length - 1
            if end_index >= len(msg.ranges):  # Avoid index errors
                break

            # Calculate the start and end positions
            start_distance = msg.ranges[start_index]
            end_distance = msg.ranges[end_index]
            if start_distance == float('inf') or end_distance == float('inf'):
                start_index = end_index + 1
                continue

            start_angle = msg.angle_min + start_index * self.angle_increment
            end_angle = msg.angle_min + end_index * self.angle_increment

            start_x = center_x + start_distance * math.cos(start_angle) * 100
            start_y = center_y + start_distance * math.sin(start_angle) * 100
            end_x = center_x + end_distance * math.cos(end_angle) * 100
            end_y = center_y + end_distance * math.sin(end_angle) * 100

            # Draw the line
            self.canvas.create_line(start_x, start_y, end_x, end_y, fill="black", width=2)

            start_index = end_index + 1

        self.get_logger().info(f"Received Lidar Data: {msg.ranges}")

def ros_spin():
    rclpy.spin(node)


def main(args=None):
    global node

    # Tkinter window and canvas setup
    root = tkinter.Tk()
    canvas = tkinter.Canvas(root, width=800, height=800)
    canvas.pack()

    # Initialize ROS 2 node and pass the canvas to the subscriber
    rclpy.init(args=args)
    node = LidarSubscriber(canvas)

    # Run ROS 2 node in a separate thread
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    # Tkinter main loop
    try:
        root.mainloop()
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
