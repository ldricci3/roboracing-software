import numpy as numpy
import cv2
import imutils
import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Image


def config_callback(config):
	rospy.loginfo("Exposure set to {double_param}".format(**config))

def callback(data):
	client = dynamic_reconfigure.client.Client("dynamic_reconfigure", timeout=30, config_callback=config_callback)

	bridge = CvBridge()

	try:
		im = bridge.imgmsg._to_cv2(data)
	except CvBridgeError as e:
		print e
		return None

	cropped_im = im[im.shape[0]/2 : image.shape[0]]

	avg = numpy.mean(cropped_im)

	curr_value = client.get_configuration().get("shutter_speed")

	p = curr_value * ((avg - 128)/128)
	p = p/10

	new_value = p + shutter_speed

	client.update_configuration({"shutter_speed": new_value})



if name == __main__:
	rospy.init_node("dynamic_reconfigure_camera_client")

	rospy.Subscriber("/image/mono", Image, callback)

	rospy.spin()