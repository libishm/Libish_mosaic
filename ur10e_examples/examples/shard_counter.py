import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import argparse
import random as rng

...
# Import ROS libraries and messages
...


# Import OpenCV libraries and tools

...

# Initialize the CvBridge class
bridge = CvBridge()
pub = None

# Define a function to show the image in an OpenCV Window


def show_image(img):
    cv.imshow("Image Window", img)
    cv.waitKey(3)

 # Define a callback for the Image message


def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError:
        rospy.logerr("CvBridge Error: {0}".format(e))   

    # Count the shards
    numShards = 0
    if pub is not None:
        pub.publish(numShards)
    # # Show the converted image
    show_image(cv_image)
#write the image to a file
# __path__ = "/home/ur10e/catkin_ws/src/ur10e_examples/examples/"
    cv.imwrite("/home/libish/segmented_shards/numShards.jpg", cv_image)

 # Initalize a subscriber to the "/camera/rgb/image_rect_color topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber(
    "/camera/color/image_rect_color", Image, image_callback)
# Adding a publisher to the "/shardCount" topic with the Int32 message type
pub = rospy.Publisher('/shardCount', Int32, queue_size=10)
rospy.init_node('imageProcessor', anonymous=True)
# rospy.init_node('shardCounter', anonymous=True)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv.destroyAllWindows()

#  # Initialize an OpenCV Window named "Image Window"
# cv.namedWindow("Image Window", 1)

# rng.seed(12345)

# def thresh_callback(val):
#     threshold = val

#     # Detect edges using Canny
#     canny_output = cv.Canny(src_gray, threshold, threshold * 4)

#     # Find contours
#     contours, hierarchy = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

#     # Draw contours
#     drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
#     for i in range(len(contours)):
#         # color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
#         color = (0, 0, 255)
#         cv.drawContours(drawing, contours, i, color, 2, cv.LINE_8, hierarchy, 0)

#     # Show in a window
#     cv.imshow('Contours', drawing)
#     l = len(contours)
#     print(l)
#     for c in range(len(contours)):
#      n_contour = contours[c]
#      for d in range(len(n_contour)):
#         XY_Coordinates = n_contour[d]
#         print(XY_Coordinates)

# # Load source image
# parser = argparse.ArgumentParser(description='Code for Finding contours in your image.')
# # parser.add_argument('--input', help='Path to input image.', default='/home/libish/Mrac_02/Libish_mosaic/industrial_reconstruction/industrial_reconstruction/scripts/2.png')
# parser.add_argument('--input', help='Path to input image.', default= cv_image)

# args = parser.parse_args()

# src = cv.imread(cv.samples.findFile(args.input))
# if src is None:
#     print('Could not open or find the image:', args.input)
#     exit(0)

# # Convert image to gray and blur it
# src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
# src_gray = cv.blur(src_gray, (3,3))

# # Create Window
# source_window = 'Source'
# cv.namedWindow(source_window)
# cv.imshow(source_window, src)
# max_thresh = 255
# thresh = 116 # initial threshold
# cv.createTrackbar('Canny Thresh:', source_window, thresh, max_thresh, thresh_callback)
# thresh_callback(thresh)
# k = cv.waitKey(0)


# extract the image from the message
#     try:
#         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#     except CvBridgeError, e:
#         rospy.logerr("CvBridge Error: {0}".format(e))
#
#     # Show the converted image
#     show_image(cv_image)

# extract the contours from the image
#     contours, hierarchy = cv2.findContours(cv_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#     #draw the contours
#     cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
#     #show the image
#     show_image(cv_image)

# list lenght of contours
# shardCount =len(contours)

# publish lenght of contours as a ros message


#  # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
# while not rospy.is_shutdown():
#      rospy.spin()
