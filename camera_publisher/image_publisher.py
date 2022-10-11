import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import cv2
from cv_bridge import CvBridge
import time

import argparse

class ImagePublisherNode(Node):

    # def __init__(self, imgfile, caminfo_file, camera_name = 'camera', img_topic = '/image_rect', caminfo_topic = '/camera_info'):
    def __init__(self):
        super().__init__('image_publisher')

        # self._imgfile = imgfile
        # self._caminfo_file = caminfo_file
        # self._camera_name = camera_name
        # self._img_topic = img_topic
        # self._caminfo_topic = caminfo_topic

        self.declare_parameters(
            namespace='',
            parameters= [
                ('imgfile', ''),
                ('caminfo_file', ''),
                ('camera_name', 'camera'),
                ('img_topic', '/image_rect'),
                ('caminfo_topic', '/camera_info')
            ]
        )
        self._imgfile = self.get_parameter('imgfile').get_parameter_value().string_value
        self._caminfo_file = self.get_parameter('caminfo_file').get_parameter_value().string_value
        self._camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self._img_topic = self.get_parameter('img_topic').get_parameter_value().string_value
        self._caminfo_topic = self.get_parameter('caminfo_topic').get_parameter_value().string_value

        self._img_publisher = self.create_publisher(Image, self._img_topic, 10)
        self._caminfo_publisher = self.create_publisher(CameraInfo, self._caminfo_topic, 10)

        self._bridge = CvBridge()

        self._image = self._bridge.cv2_to_imgmsg(cv2.imread(self._imgfile, cv2.IMREAD_GRAYSCALE), 'mono8')
        self._image.header.frame_id = self._camera_name

        fs = cv2.FileStorage()
        fs.open(self._caminfo_file, cv2.FILE_STORAGE_READ)

        self._caminfo = CameraInfo()
        self._caminfo.header.frame_id = self._camera_name
        self._caminfo.height = int(fs.getNode('image_height').real())
        self._caminfo.width = int(fs.getNode('image_width').real())
        self._caminfo.distortion_model = fs.getNode('distortion_model').string()
        self._caminfo.d = fs.getNode('distortion_coefficients').mat().flatten().tolist()
        self._caminfo.k = fs.getNode('camera_matrix').mat().flatten().tolist()
        self._caminfo.r = fs.getNode('rectification_matrix').mat().flatten().tolist()
        self._caminfo.p = fs.getNode('projection_matrix').mat().flatten().tolist()

        self._t = self.create_timer(1/30, self.pub_timer)

    def pub_timer(self):
        t = self.get_clock().now().to_msg()
        self._image.header.stamp = t
        self._caminfo.header.stamp = t

        self._img_publisher.publish(self._image)
        self._caminfo_publisher.publish(self._caminfo)

def main(args = None):
    rclpy.init(args=args)

    # parser = argparse.ArgumentParser()
    # parser.add_argument('imgfile')
    # parser.add_argument('caminfo_file')
    # args = parser.parse_args()
    
    node = ImagePublisherNode()

    rclpy.spin(node)

if __name__ == '__main__':
    main()