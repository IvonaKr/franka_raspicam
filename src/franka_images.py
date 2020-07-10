#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

class SavePictures():

    def __init__(self):

        rospy.init_node('image_saver', anonymous=True)
        self.flag = False

        self.id_sub = rospy.Subscriber("/touch_id", String, self.id_callback)
        self.start_sub = rospy.Subscriber("/record_start", Bool, self.start_callback)
        self.stop_sub = rospy.Subscriber("/record_stop", Bool, self.stop_callback)
        self.camera_sub = rospy.Subscriber("/raspicam_node/image",Image, self.image_callback)
        self.frames = []

    def id_callback(self,ros_msg):
        self.id = ros_msg.data 

    def start_callback(self,ros_msg):
        start = ros_msg.data
        self.flag = True
        
    
    def stop_callback(self,ros_msg):
        stop = ros_msg.data
        self.flag = False
        
        try:
            #outFolder = "/home/ivan/diplomski_ws/src/raspicam_features/src/franka/images/"

            outFolder = "/home/franka/ivona_dipl/images/"

            # outFolder = rospy.get_param('~dir1')
            
            self.frames_pom = np.array(self.frames)
            
            print((self.frames_pom).shape)
            print((self.frames_pom[0]).shape)
            height, width, layers = (self.frames_pom[0]).shape
            #print('tu sam')
            size = (width,height)
            
            pathOut = outFolder + "video_" + self.id + ".avi"
            #print('pokusavam spremiti')
            out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), 30.0, size)
            

            for frame in self.frames:
                out.write(frame)
            out.release()
            print('spremljeno')
        except AttributeError:
            pass

        
        self.frames = []
       

    def image_callback(self, ros_image):

        if self.flag:

            image = bridge.imgmsg_to_cv2(ros_image, "8UC3")
            self.frames.append(image)
            
           



def main():
    

   

    sp = SavePictures()

    try:
      rospy.spin()
    except KeyboardInterrupt:
       print("Shutting down")
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()