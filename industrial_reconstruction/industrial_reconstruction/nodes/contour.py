...
 # Import ROS libraries and messages
...

# Import OpenCV libraries and tools
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import argparse
import random as rng

...

 # Initialize the CvBridge class
bridge = CvBridge()

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
     except CvBridgeError, e:
         rospy.logerr("CvBridge Error: {0}".format(e))

     # Show the converted image
     show_image(cv_image)

 # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/rgb/camera_depth_image_raw", image_callback, image_callback)

 # Initialize an OpenCV Window named "Image Window"
cv.namedWindow("Image Window", 1)

rng.seed(12345)

def thresh_callback(val):
    threshold = val

    # Detect edges using Canny
    canny_output = cv.Canny(src_gray, threshold, threshold * 4)

    # Find contours
    contours, hierarchy = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Draw contours
    drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        # color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        color = (0, 0, 255)
        cv.drawContours(drawing, contours, i, color, 2, cv.LINE_8, hierarchy, 0)

    # Show in a window
    cv.imshow('Contours', drawing)
    l = len(contours)
    print(l)

# Load source image
parser = argparse.ArgumentParser(description='Code for Finding contours in your image tutorial.')
parser.add_argument('--input', help='Path to input image.', default='/home/libish/Mrac_02/Libish_mosaic/industrial_reconstruction/industrial_reconstruction/scripts/2.png')

args = parser.parse_args()

src = cv.imread(cv.samples.findFile(args.input))
if src is None:
    print('Could not open or find the image:', args.input)
    exit(0)

# Convert image to gray and blur it
src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
src_gray = cv.blur(src_gray, (3,3))

# Create Window
source_window = 'Source'
cv.namedWindow(source_window)
cv.imshow(source_window, src)
max_thresh = 255
thresh = 116 # initial threshold
cv.createTrackbar('Canny Thresh:', source_window, thresh, max_thresh, thresh_callback)
thresh_callback(thresh)
k = cv.waitKey(0)

if k == ord("q"):
    cv.destroyAllWindows()

    
#extract the image from the message
#     try:
#         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#     except CvBridgeError, e:
#         rospy.logerr("CvBridge Error: {0}".format(e))
#
#     # Show the converted image
#     show_image(cv_image)

#extract the contours from the image
#     contours, hierarchy = cv2.findContours(cv_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#     #draw the contours
#     cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
#     #show the image
#     show_image(cv_image)

#list lenght of contours
#     print len(contours)

#publish lenght of contours as a ros message

 # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
 while not rospy.is_shutdown():
     rospy.spin()

