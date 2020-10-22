#!/usr/bin/env python 
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
masked_image = None
masked = None
indexes = None
counter1 = 0
counter2 = 0
global previous_error
global integral
previous_error = 0
integral = 0
cv_image_2 = None
#callback does all of the work: reads image, masks, finds center of the "blob", which is the line
#and then uses a PID algorithm to keep itself on the line
def cv_callback(msg):
    t = Twist()
    #convert image to CVimage to process in opencv
    cv_image = bridge.imgmsg_to_cv2(msg)
    #create a second cvimage for outputting image responses to make sure output is correct
    global cv_image_2
    cv_image_2 = cv_image
    global image_in_hsv
    #change image to HSV and mask it so everything other than the yellow line is black
    image_in_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 30, 50, 75])
    upper_yellow = numpy.array([ 100, 255, 255])
    mask = cv2.inRange(image_in_hsv,  lower_yellow, upper_yellow)
    global previous_error
    global integral
    cv_image_2.setflags(write = 1)
    #find the height, width and depth of the image and take a sliver of it so the tracking is more contained
    h, w, d = cv_image.shape
    search_top = 3 * h /4
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    global cx
    #find the outline and the center of the "circle" that is created around the center of the "blob" that is the line
    moments = cv2.moments(mask)
    if moments['m00'] > 0:
            cx = int(moments['m10']/moments['m00']) + 100
            cy = int(moments['m01']/moments['m00'])
            cv2.circle(mask, (cx, cy), 20, (0, 0 ,255), -1)
    print moments
    t = Twist()
    #PID steering from previous PA: the error is how far we are from the middle, then we determine the output through
    #calculations: error/100 was derived from follower_p, but output/25 was taken from tuning and testing to get the perfect following
    error = cx - w/2
    integral = integral + (error * 2)
    derivative = (error - previous_error) / 2
    output = (.121 * (error/100)) + (.121 * (error/100)) + (.121 * (error/100))
    t.angular.z = -output/25
    t.linear.x = .1
    vel_pub.publish(t)
    #output the masked image to make sure we are seeing the correct outputs
    masked = cv2.bitwise_and(image_in_hsv, image_in_hsv, mask=mask)
    unmasked = cv2.cvtColor(masked, cv2.COLOR_HSV2RGB)
    real_img = bridge.cv2_to_imgmsg(unmasked)
    img_publisher.publish(real_img)
    
   
#initiate various objects
img_publisher = rospy.Publisher('/my_image', Image, queue_size=1)
rospy.init_node('Line_follower')
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
cv_sub = rospy.Subscriber('camera/rgb/image_raw', Image, cv_callback)
bridge = cv_bridge.CvBridge()
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    #the loop is just to make sure it will stop with ctrl c, it spins because all the work is done in the callback
    rospy.spin()