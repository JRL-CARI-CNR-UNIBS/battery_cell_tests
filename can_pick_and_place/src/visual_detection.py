#!/usr/bin/env python3

import rospy
import rospkg
import sys
import cv2
from tf2_ros import StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
from numpy import mean

path = '/home/gauss/projects/personal_ws/src/vision_system'
sys.path.append(path)

from realsense import Realsense


def can_deteciont(req):
    rs = Realsense()
    
    rs.acquireOnceBoth()

    img = rs.colorFrame

    cv2.imwrite('/home/gauss/Pictures/can_tests/img1.png', img)

    height, width = img.shape[:2]
    height = int(height*0.6)

    cropped = img[0:height, 0:width]
    cv2.imwrite('/home/gauss/Pictures/can_tests/img2.png', cropped)

    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY) 
    contours, _ = cv2.findContours( 
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.imwrite('/home/gauss/Pictures/can_tests/img3.png', gray)
    cv2.imwrite('/home/gauss/Pictures/can_tests/img4.png', threshold)

    i = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        approx = cv2.approxPolyDP( 
                contour, 0.01 * cv2.arcLength(contour, True), True)

        if (area < 450 or area > 750 or (len(approx)) < 6):
            continue

        cv2.drawContours(img, [contour], 0, (0, 0, 255), 1) 

        M = cv2.moments(contour) 
        if M['m00'] != 0.0: 
            x = int(M['m10']/M['m00']) 
            y = int(M['m01']/M['m00']) 
        else:
            print('M not found')
            return False

        print(x)
        print(y)

        # print(contour)
        # x_array = []
        # y_array = []
        # for point in contour:
        #     x_array.append(point[0][0])
        #     y_array.append(point[0][1])
        
        # print(x_array)
        # print(y_array)
        # x = int(mean(x_array))
        # y = int(mean(y_array))
        # print(x)
        # print(y)

        i +=1

        cv2.circle(img, (x,y), radius=0, color=(0, 0, 255), thickness=2)

        [r_x, r_y, r_z] = rs.deproject(x,y)
        if (r_z == 0):
            print("depth no detected")
            return False

        tf_pub = rospy.Publisher('/tf',TFMessage, queue_size=1)
        broadcaster = StaticTransformBroadcaster()

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = rs.camera_frame_name
        transform_stamped.child_frame_id = 'rs_can'

        transform_stamped.transform.translation.x = r_x
        transform_stamped.transform.translation.y = r_y
        transform_stamped.transform.translation.z = r_z

        transform_stamped.transform.rotation.x = 0.977
        transform_stamped.transform.rotation.y = -0.007
        transform_stamped.transform.rotation.z = -0.012
        transform_stamped.transform.rotation.w = 0.211

        tf_msg = TFMessage()
        tf_msg.transforms.append(transform_stamped)
        tf_pub.publish(tf_msg)
        broadcaster.sendTransform(transform_stamped)

    cv2.imwrite('/home/gauss/Pictures/can_tests/img5.png', img)

    if i == 0:
        print("Can not found")
        return False
    if i > 1:
        print("To0 many objects detected")
        return False

    return []

def visual_detection():
    rospy.init_node('visual_detection')
    s = rospy.Service('can_deteciont', Empty, can_deteciont)
    print("Ready to detect objects.")
    rospy.spin()

if __name__ == '__main__':
    visual_detection()
