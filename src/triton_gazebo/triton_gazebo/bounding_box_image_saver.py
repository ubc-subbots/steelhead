#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from triton_interfaces.msg import DetectionBox
import cv_bridge
from cv2 import cv2
import numpy as np
import random
from ament_index_python.packages import get_package_share_directory

class BoundingBoxImageSaver(Node):

    def __init__(self):
        super().__init__('bounding_box_image_saver')
        self.subscribe_image = self.create_subscription(Image, "/triton/gazebo_drivers/front_camera/underwater/image_raw", self.save_image, 10)
        self.subscriber_bbox = self.create_subscription(DetectionBox, "/triton/gazebo_drivers/front_camera/bounding_box", self.save_bbox, 10)

    def save_image(self, msg: Image):
        self.get_logger().info("Getting image...")
        br = cv_bridge.CvBridge()
        im = br.imgmsg_to_cv2(msg, "passthrough")
        self.current_image = im

    def save_bbox(self, msg: DetectionBox):
        if (msg.width <= 0 or msg.height <= 0):
            self.get_logger().info("Bounding box has zero size.")
            return

        try:
            rows,cols,channels = self.current_image.shape
           
            centre_x = (msg.x + msg.width/2)/cols
            centre_y = (msg.y + msg.height/2)/rows
            width = msg.width/cols
            height = msg.height/rows

            name = "image"+str(random.randint(0,2**16-1))
            self.get_logger().info("Saving..."+name)
            txt_string = f"{msg.class_id} {centre_x} {centre_y} {width} {height}"

            data_dir = os.path.join(get_package_share_directory("triton_gazebo"),"data")
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)

            f = open(os.path.join(data_dir, name + ".txt"), "w")
            f.write(txt_string)
            f.close()
            cv2.imwrite(os.path.join(data_dir, name + ".png"), self.current_image)

            image_with_box = self.current_image.copy()
            image_with_box = cv2.rectangle(image_with_box,(int(msg.x),int(msg.y)),(int(msg.x+msg.width),int(msg.y+msg.height)),(0,0,255),1)
            cv2.imwrite(os.path.join(data_dir, name + "_box" + ".png"), image_with_box)
        except AttributeError as e:
            self.get_logger().info("No image yet.")
            pass
        

def main(args=None):
    rclpy.init(args=args)
    subscriber = BoundingBoxImageSaver()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()