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
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from os.path import dirname, join
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import random
import face_recognition


class face_localizer:


    def __init__(self):
        rospy.init_node('face_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")

        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Publiser for the visualization markers
        
        self.recognizer_pub = rospy.Publisher('recognized', Float32MultiArray, queue_size=1000)
        
        self.reward = None
        def get_reward(msg):
            rospy.loginfo("GOT REWARD")
            # rospy.loginfo(msg)
            self.reward = msg.data
        self.reward_sub = rospy.Subscriber('/reward', Float32MultiArray, get_reward)

        self.delay = 2
        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi",Twist, queue_size=50)

        self.arm = ""
        def doshit(data):
            time.sleep(1)
            self.arm = data.data.strip()
        self.arm_sub = rospy.Subscriber("/arm_command", String, doshit)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0


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



     

        # print(rgb_image[~is_not_grayscale])


        # Set the dimensions of the image
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
                print("MOVER")
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0.2
                self.twist_pub.publish(twist)
            elif sumr > 0 and suml == 0:
                print("MOVEL")
#rostopic pub /arm_command std_msgs/String "data: 'cyl'"

                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = -0.17
                self.twist_pub.publish(twist)


        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()
        # print(face_detections.shape)
        # print(face_detections[0,0].shape)

        # vec1 = [-0.13795538246631622, 0.061852604150772095, 0.09006159007549286, 0.1016073003411293, -0.07362175732851028, -0.11852101981639862, -0.02643529325723648, -0.18959768116474152, 0.08735057711601257, -0.09347183257341385, 0.11634308099746704, -0.0831608921289444, -0.25498872995376587, -0.006751105189323425, -0.06190497800707817, 0.12885166704654694, -0.14177803695201874, -0.08893252164125443, -0.07881265878677368, -0.07995526492595673, 0.00983712263405323, 0.03455875813961029, 0.06351334601640701, 0.010522286407649517, -0.0663922131061554, -0.30574649572372437, -0.06204711273312569, -0.10262848436832428, 0.06711842864751816, 0.04549292102456093, -0.007861821912229061, 0.10026569664478302, -0.2107429951429367, -0.05947033315896988, 0.020427891984581947, 0.12229566276073456, -0.06443183124065399, -0.015331516042351723, 0.3092231750488281, -0.04010308161377907, -0.1580399125814438, 0.01934598758816719, 0.03940997272729874, 0.22693221271038055, 0.19166973233222961, -0.05195224657654762, 0.026014620438218117, -0.09559503942728043, 0.11909458041191101, -0.284292608499527, 0.010035485029220581, 0.23386012017726898, 0.0423722118139267, 0.0840066522359848, 0.0425189733505249, -0.129638671875, 0.028680313378572464, 0.09641537070274353, -0.1986120045185089, 0.12527409195899963, 0.11247304081916809, -0.15471969544887543, -0.004694861359894276, -0.006448522675782442, 0.11488169431686401, 0.0517740435898304, -0.049398504197597504, -0.10844298452138901, 0.16880866885185242, -0.2154344618320465, 0.0033777039498090744, 0.07382961362600327, -0.10178562998771667, -0.17111451923847198, -0.35524871945381165, -0.015331042930483818, 0.4494364261627197, 0.13947249948978424, -0.06296730786561966, 0.07278092205524445, -0.12590451538562775, -0.1165153831243515, 0.04503132775425911, 0.03441604971885681, -0.03209064155817032, -0.061897628009319305, -0.10231798887252808, 0.10039244592189789, 0.20041637122631073, -0.01914394274353981, -0.028847096487879753, 0.19145451486110687, 0.007058639079332352, 0.005464818328619003, 0.007232202682644129, 0.0678127259016037, -0.12331747263669968, -0.11853623390197754, -0.14536701142787933, -0.042833734303712845, 0.006006250157952309, -0.1494498997926712, 0.05523525923490524, 0.0909096822142601, -0.19843512773513794, 0.13644129037857056, -0.04356732219457626, -0.030914265662431717, -0.016071908175945282, -0.024323102086782455, -0.11977341026067734, 0.0636252611875534, 0.20315203070640564, -0.1518036276102066, 0.25613248348236084, 0.13831153512001038, -0.012736385688185692, 0.13448025286197662, 0.051639728248119354, 0.05444230139255524, -0.07896649837493896, -0.030078507959842682, -0.1252928227186203, -0.1384488195180893, -0.028398215770721436, 0.0672670230269432, 0.08189349621534348, 0.08442730456590652]




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

                # face_region = rgb_image[y1:y2, x1:x2]
                # face = face_recognition.face_locations(rgb_image)

                vec = face_recognition.face_encodings(rgb_image, [(y1,x2,y2,x1)])
                pubi = Float32MultiArray(data=vec[0])
                self.recognizer_pub.publish( pubi )

                if len(vec) > 0:
                    print(face_recognition.compare_faces([self.reward], vec[0]))
                    if face_recognition.compare_faces([self.reward], vec[0])[0]:
                        self.recognizer_pub.publish( Float32MultiArray(data=[1.0]) )
                    else:
                        self.recognizer_pub.publish( Float32MultiArray(data=[0.0]) )
                    self.delay = 2
                    # print(vec[0],self.reward)
                break

                # Visualize the extracted face
                # try:
                #     cv2.imshow("ImWindow", face_region)
                #     cv2.waitKey(1)
                # except Exception as e:
                #     print(e)
            else:
                self.delay = 2
                pubi = Float32MultiArray(data=[])
                self.recognizer_pub.publish( pubi )
                # self.recognizer_pub.publish( '' )
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
