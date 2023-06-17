#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import math
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
        self.delay = 2
        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi",Twist, queue_size=50)

        self.arm = ""
        def doshit(data):
            rospy.sleep(0.5)
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

        print('I got a new image!')

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
                twist.angular.z = 0.15
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
                twist.angular.z = -0.15
                self.twist_pub.publish(twist)


        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()
        # print(face_detections.shape)
        # print(face_detections[0,0].shape)

        vec1 = [-0.08448592,  0.02780541,  0.01730084, -0.04765084, -0.09500638,
       -0.00439726,  0.03771864, -0.05092044,  0.13887392, -0.11239615,
        0.15770856,  0.03520457, -0.31810117, -0.0468499 ,  0.05206267,
        0.13269851, -0.09736396, -0.05650488, -0.15357809, -0.10187391,
       -0.00449973,  0.07130985,  0.10896843,  0.02119571, -0.09374757,
       -0.36909801, -0.0837078 , -0.09850118,  0.00072278, -0.04116699,
       -0.03160951,  0.09632961, -0.06830925,  0.016726  ,  0.0197359 ,
        0.09536502, -0.14287873, -0.10795528,  0.25371137,  0.06761247,
       -0.10499734, -0.11221032,  0.01744668,  0.26058152,  0.18877712,
       -0.00581103,  0.12926337, -0.03543531,  0.03942911, -0.2376315 ,
        0.08655408,  0.11868708,  0.07565185,  0.08228702,  0.11645524,
       -0.15795518,  0.05910831,  0.06471434, -0.15775514,  0.04504938,
        0.09244304, -0.10079256, -0.09208282, -0.03144315,  0.25167909,
        0.12735167, -0.09564064, -0.10771119,  0.17664349, -0.20412223,
       -0.06695671,  0.01212422, -0.17430922, -0.16927493, -0.18182287,
        0.02311506,  0.36347425,  0.21733566, -0.12241915, -0.03791935,
       -0.05252503, -0.07046589,  0.11090904,  0.08728698, -0.11696315,
       -0.03733665, -0.09496866,  0.08294217,  0.18606859, -0.04497548,
       -0.06672512,  0.19814636,  0.07500114, -0.00961408,  0.05832277,
       -0.07243416, -0.05684882,  0.01143615, -0.10413817,  0.00800654,
        0.07423899, -0.20282847,  0.03234023,  0.05628428, -0.14054883,
        0.17278408, -0.0336709 , -0.04175285,  0.0238065 , -0.0341855 ,
       -0.21091215, -0.0215057 ,  0.19325644, -0.16167042,  0.14995778,
        0.19377871,  0.06234102,  0.14648226,  0.06040868,  0.15584385,
       -0.00539487,  0.00273583, -0.18365256, -0.03777637,  0.05064691,
        0.02806997,  0.07312253,  0.04854042]


        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence>0.95:

                
                # new_image = np.copy(rgb_image)
                # is_not_grayscale = np.any(np.abs(np.diff(rgb_image, axis=-1)) != 0, axis=-1)
                # new_image[~is_not_grayscale] = [0]
                # new_image[is_not_grayscale] = [133]
                # new_image[:,:50] = [200,100,100]

                # cv2.imshow("ImWindow", new_image)
                # cv2.waitKey(1)

                if self.delay > 0:
                    

        

                    self.delay -= 1
                    rospy.loginfo("delayiing")
                    return;
                # rospy.loginfo(confidence)


                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                face = face_recognition.face_locations(rgb_image)
                # print(face)
                # print(y1,x2,y2,x1)
                vec = face_recognition.face_encodings(rgb_image, [(y1,x2,y2,x1)])
                pubi = Float32MultiArray(data=vec[0])
                self.recognizer_pub.publish( pubi )


                # print(vec)
                if len(vec) > 0:
                    print(face_recognition.compare_faces([vec1], vec[0]))
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
