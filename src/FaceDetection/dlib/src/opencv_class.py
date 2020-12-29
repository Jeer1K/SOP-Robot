#! /usr/bin/env python

import cv2
import dlib
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
detector = dlib.get_frontal_face_detector()

old_faces = []

pub = rospy.Publisher("opencv/image_raw",Image,queue_size=1)

def image_callback(msg):
    
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        image = cv2.resize(cv2_img, (0, 0), fx=0.4, fy=0.4)
       

        faces = detector(image, 1)
        
        
        old_faces = []
        for face in faces:
                tracker = cv2.TrackerBoosting_create()
                box = (face.left(), face.top(), face.width(), face.height())
                tracker.init(image, box)
                old_faces.append(tracker)
        
        for i, tracker in enumerate(old_faces):
                ok, bbox = tracker.update(image)
                if ok:
                    cv2.rectangle(image, (int(bbox[0]), int(bbox[1])), (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])),
                                   (100, 200, 100))
                else:
                    old_faces.pop(i)

        #cv2.imshow("image",  image)
        #cv2.waitKey(1)
        msg_out = bridge.cv2_to_imgmsg(image,encoding="bgr8")
        pub.publish(msg_out)
        
    
        
        
        
def main():
    old_faces = []
    rospy.init_node("image_listener_opencv")
    image_topic = "/usb_cam/image_raw"
    rospy.Subscriber(image_topic,Image,image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
    



