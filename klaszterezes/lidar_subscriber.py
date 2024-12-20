#!/usr/bin/env python3

import rclpy
import tkinter
import threading
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray


class LidarSubscriber(Node):
    def __init__(self, canvas):
        super().__init__('lidar_subscriber')
        self.canvas = canvas  # Tkinter canvas to draw on
        self.angle_increment = None  # To store the angle increment from Lidar

        # Publishers for objects, lines, and ranges
        self.objects_publisher = self.create_publisher(Int32MultiArray, '/lidar_objects', 10)
        self.lines_publisher = self.create_publisher(Int32MultiArray, '/lidar_lines', 10)
        self.ranges_publisher = self.create_publisher(Float32MultiArray, '/lidar_ranges', 10)

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
        line_counter = 0
        line_average = 0

        # Loop through the Lidar data and separate mesurments by distance
        for i, pos in enumerate(msg.ranges):
            if pos == float('inf'):
                # Append 1 for infinite values and reset counters
                if line_counter > 0:
                    lines.append(line_counter)  # Append the length of the current line
                lines.append(1)  # Append 1 for the infinite value
                line_average = 0
                line_counter = 0
            else:
                # Update line average and counter for valid values
                if line_counter == 0:
                    line_average = pos
                else:
                    line_average = (pos + line_average * line_counter) / (line_counter + 1)
                
                # Check if the point is part of the current line
                if abs(pos - line_average) > 0.15:
                    lines.append(line_counter)  # Append the length of the current line
                    line_average = pos  # Start a new line
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

        # Append the final line length if there are remaining points
        if line_counter > 0:
            lines.append(line_counter)


        # Remove Ones from objects
        objects = [obj for obj in objects if obj != 1]

        # Publish objects, lines, and ranges
        objects_msg = Int32MultiArray(data=objects)
        self.objects_publisher.publish(objects_msg)

        lines_msg = Int32MultiArray(data=lines)
        self.lines_publisher.publish(lines_msg)

        ranges_msg = Float32MultiArray(data=msg.ranges)
        self.ranges_publisher.publish(ranges_msg)

        self.get_logger().info(f"Published Objects: {objects}")
        self.get_logger().info(f"Published Lines: {lines}")
        self.get_logger().info(f"Published Ranges: {msg.ranges}")

        # Visualisation
        line_start = True
        line_x = 0
        line_y = 0

        line_counter = 1
        lines_counter = 0

        
        # Loop through the Lidar data and draw circles and lines based on distances
        for i, distance in enumerate(msg.ranges):
            # If we have drawn the current cluster, move to the next one
            if object_counter < len(objects) and cluster_counter == objects[object_counter]:
                color_counter = (color_counter + 1) % len(colors)  
                object_counter += 1
                cluster_counter = 0

            if distance == float('inf'):
                line_start = True
                if lines_counter < len(lines)-1:
                    lines_counter += 1

            
            if distance != float('inf'):
                angle = msg.angle_min + i * self.angle_increment
                if line_start:
                    line_start = False
                    line_x = center_x + distance * math.cos(angle) * 100
                    line_y = center_y + distance * math.sin(angle) * 100
                x = center_x + distance * math.cos(angle) * 100
                y = center_y + distance * math.sin(angle) * 100
                # Draw a small circle at the calculated position
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill=colors[color_counter])
                
                # Draw the line
                if lines[lines_counter] <= 1:
                    if lines_counter < len(lines)-1:
                        lines_counter += 1
                    line_start = True
                    line_counter = 0
                elif (lines[lines_counter] == line_counter and lines_counter < len(lines)):
                    if lines_counter < len(lines)-1:
                        lines_counter += 1
                    line_counter = 0
                    self.canvas.create_line(line_x, line_y, x, y, width=3)
                    line_start = True

                line_counter += 1
                cluster_counter += 1 
        

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
