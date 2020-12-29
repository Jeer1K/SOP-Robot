#! /usr/bin/env python

import cv2
import dlib
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
detector = dlib.get_frontal_face_detector()

old_faces = []

pub1 = rospy.Publisher("dlib/image_raw_grey",Image,queue_size=1)
def image_callback(msg):
    
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg,"bgr8")
        cv2_img = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
    except CvBridgeError, e:
        print(e)
    else:
       
        image1 = cv2.resize(cv2_img, (0, 0), fx=0.4, fy=0.4)

        
        
        
        

        #cv2.imshow("image",  image)
        #cv2.waitKey(1)
       
        
        old_faces1 = []
        faces1 = detector(image1, 1)
        for face in faces1:
            tracker = dlib.correlation_tracker()
            tracker.start_track(image1, face)
            old_faces1.append(tracker)
        for i, tracker in enumerate(old_faces1):
            quality = tracker.update(image1)
            if quality > 7:
                pos = tracker.get_position()
                pos = dlib.rectangle(
                    int(pos.left()),
                    int(pos.top()),
                    int(pos.right()),
                    int(pos.bottom()),
                )
                cv2.rectangle(image1, (pos.left(), pos.top()), (pos.right(), pos.bottom()),
                              (100, 200, 100))
        msg_out1 = bridge.cv2_to_imgmsg(image1,encoding="mono8")
        pub1.publish(msg_out1)
        
        
        
def main():
    old_faces = []
    rospy.init_node("image_listener_grey")
    image_topic = "/usb_cam/image_raw"
    rospy.Subscriber(image_topic,Image,image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
    
