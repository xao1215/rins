#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import math
import time
#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseArray, Twist
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA,Int64,String,Float32MultiArray
from nav_msgs.msg import Odometry
# from sound_play.msg import SoundRequest
# from sound_play.libsoundplay import SoundClient
from os.path import dirname, join
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import random
import face_recognition


class face_localizer:


    def __init__(self):
        rospy.init_node('face_localizer', anonymous=True)

        self.bridge = CvBridge()
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")
        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)
        self.dims = (0, 0, 0)

        
        self.recognizer_pub = rospy.Publisher('recognized', Float32MultiArray, queue_size=1000)
        
        self.rewards = []
        def get_reward(msg):
            rospy.loginfo("GOT REWARD")
            # self.reward = msg.data
            self.rewards.append(msg.data)
        self.reward_sub = rospy.Subscriber('/reward', Float32MultiArray, get_reward)

        self.park_pub = rospy.Publisher('/park', String, queue_size=1000)

        self.poster_colors = []
        def get_parking(msg):
            self.poster_colors.append(msg.data)
            rospy.loginfo(f"got color={msg.data}")
        self.park_sub = rospy.Subscriber('/park', String, get_parking, queue_size=1000)


        self.delay = 2
        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi",Twist, queue_size=50)

        self.arm = ""
        def doshit(data):
            time.sleep(1)
            self.arm = data.data.strip()
        self.arm_sub = rospy.Subscriber("/arm_command", String, doshit)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # self.soundhandle = SoundClient()
        # self.voice = 'voice_kal_diphone'
        # self.volume = 1.0


    def find_faces(self):

        if self.arm != "cyl":
            return

        # print('I got a new image!')

        try:
            rgb_image_message = rospy.wait_for_message("/arm_camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)



        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]
        # Tranform image to gayscale
        # gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        # img = cv2.equalizeHist(gray)

        # Detect the faces in the image
        # face_rectangles = self.face_detector(rgb_image, 0)

        if h > 0 and w > 0:
            new_image = np.copy(rgb_image)
            is_not_grayscale = np.any(np.abs(np.diff(rgb_image, axis=-1)) != 0, axis=-1)
            new_image[~is_not_grayscale] = 0
            new_image[is_not_grayscale] = 1
            suml = np.sum(new_image[:,:50].flatten())
            sumr = np.sum(new_image[:,-50:].flatten())

            if suml > 0 and sumr == 0:
                print("MOVEL")
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0.395
                self.twist_pub.publish(twist)
            elif sumr > 0 and suml == 0:
                print("MOVER")
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = -0.395
                self.twist_pub.publish(twist)



        #rostopic pub /arm_command std_msgs/String "data: 'cyl'"

        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()




        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence>0.95:
                
                if self.delay > 0:
                    self.delay -= 1
                    rospy.loginfo("delayiing")
                    return;

                
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]


                # cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                # cv2.rectangle(rgb_image, (0, 0), (50, h), (200, 0, 100), 3)
                # cv2.rectangle(rgb_image, (w-50, 0), (w-1, h), (200, 0, 100), 3)

                # cv2.imshow("ImWindow", rgb_image)
                # cv2.waitKey(1)
                # # return;

                # face_region = rgb_image[y1:y2, x1:x2]
                # face = face_recognition.face_locations(rgb_image)

                vec = face_recognition.face_encodings(rgb_image, [(y1,x2,y2,x1)])
                # pubi = Float32MultiArray(data=vec[0])
                # self.recognizer_pub.publish( pubi )

                dists = face_recognition.face_distance(self.rewards, vec[0])
                print(dists)
                count = np.count_nonzero(dists > 5)
                most = np.inf

                if len(vec) > 0:
                
                    for i in range(len(self.rewards)):
                        compar = face_recognition.compare_faces([self.rewards[i]], vec[0])
                        print(compar)
                        if compar[0] and i == np.argmin(dists):
                        # if compar[0]:
                            self.recognizer_pub.publish( Float32MultiArray(data=[1.0]) )
                            self.park_pub.publish( self.poster_colors[i] )
                            self.delay = 2
                            rospy.sleep(1.5)
                            return

                    self.recognizer_pub.publish( Float32MultiArray(data=[0.0]) )
                    self.delay = 2
                
                break

            else:
                self.delay = 2
                pubi = Float32MultiArray(data=[])
                self.recognizer_pub.publish( pubi )
                break

                


                        
                    


def main():

        face_finder = face_localizer()
        rate = rospy.Rate(1)
        # rospy.sleep(5.)
        while not rospy.is_shutdown():
            face_finder.find_faces()
            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
