from ssl import SOL_SOCKET
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import cv2
from cv_bridge import CvBridge

import numpy as np

import socket
import select

class CameraPublisherNode(Node):
	
	def __init__(self):
		super().__init__('image_publisher')

		self.declare_parameters(
			namespace='',
			parameters= [
				('caminfo_file', ''),
				('camera_name', 'camera'),
				('img_topic', 'image_rect'),
				('caminfo_topic', 'camera_info')
			]
		)
		self._caminfo_file = self.get_parameter('caminfo_file').get_parameter_value().string_value
		self._camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
		self._img_topic = self.get_parameter('img_topic').get_parameter_value().string_value
		self._caminfo_topic = self.get_parameter('caminfo_topic').get_parameter_value().string_value

		self._img_publisher = self.create_publisher(Image, f"{self._camera_name}/{self._img_topic}", 10)
		self._caminfo_publisher = self.create_publisher(CameraInfo, f"{self._camera_name}/{self._caminfo_topic}", 10)

		self._bridge = CvBridge()

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

		self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self._socket.setsockopt(SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self._socket.bind(("localhost", 1100))
		self._socket.listen()
		self._client, self._addr = self._socket.accept()
		self._t = threading.Thread(target = self.client_handler)
		self._t.start()

	def client_handler(self):
		try:
			while True:
				readable, _, _ = select.select([self._client], [], [])
				if readable:
					data = self._client.recv(self._caminfo.height*self._caminfo.width*4)
					if not (len(data) == self._caminfo.height*self._caminfo.width*4):
						continue
					im = np.frombuffer(data, np.uint8)

					#reshape into opencv image format
					img_array = np.reshape(im,(self._caminfo.height,self._caminfo.width,4))

					#convert to rospy and publish
					self._image = self._bridge.cv2_to_imgmsg(cv2.cvtColor(img_array, cv2.COLOR_BGRA2GRAY), "mono8")
					self._image.header.frame_id = self._camera_name
					t = self.get_clock().now().to_msg()
					self._image.header.stamp = t
					self._caminfo.header.stamp = t

					self._img_publisher.publish(self._image)
					self._caminfo_publisher.publish(self._caminfo)
		finally:
			self._client.close()

def main(args = None):
	rclpy.init(args=args)
	
	node = CameraPublisherNode()

	rclpy.spin(node)

if __name__ == '__main__':
	main()