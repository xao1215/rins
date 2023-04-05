#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Vector3
from std_msgs.msg import ColorRGBA, Int64
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import tf2_ros
import math


goal = None
face_deteced = None
goal_sent = None
num = None


def generate_goal(rotation, x, y):
    # try:
    #     trans = buffer.lookup_transform('map', 'base_link', rospy.Time(0))
    # except Exception as e:
    #     print(e)
    try:
        odometry = rospy.wait_for_message("/odom", Odometry)
    except Exception as e:
        print(e)    

    q = [odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w]
    (_,_,angle) = euler_from_quaternion(q)
    quaternion = quaternion_from_euler(0, 0, angle+3.1)
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    return goal

def update_goal(face_pose, tf2_buffer):
    global goal, face_detected, goal_sent
    # goal.target_pose.pose.position.x = msg.target_pose.pose.position.x
    # goal.target_pose.pose.position.y = msg.target_pose.pose.position.y

    # moment_in_past = rospy.Time.now() - rospy.Duration(nsecs=700000000)			
    # trans = tf2_buffer.lookup_transform('map', 'base_link', moment_in_past, rospy.Duration(1.))
    
    
    # try:
    #     odometry = rospy.wait_for_message("/odom", Odometry)
    # except Exception as e:
    #     print(e)
        
    # distance_to_face= 0
    # # vector_x = odometry.pose.pose.position.x - face_pose.position.x
    # # vector_y = odometry.pose.pose.position.y - face_pose.position.y
    # vector_x = trans.transform.translation.x - face_pose.position.x
    # vector_y = trans.transform.translation.y - face_pose.position.y
    # d = math.hypot(vector_x,vector_y)

    # if d > distance_to_face:
    #     newd = distance_to_face - d
    #     ratio=newd/d
    #     goal.target_pose.pose.position.x = trans.transform.translation.x + (vector_x * ratio)
    #     goal.target_pose.pose.position.y = trans.transform.translation.y + (vector_y * ratio)
    # else:
    #     goal.target_pose.pose.position.x = trans.transform.translation.x
    #     goal.target_pose.pose.position.y = trans.transform.translation.y

    print(face_pose.orientation.w,face_pose.orientation.z)

    goal.target_pose.pose.position.x = face_pose.position.x
    goal.target_pose.pose.position.y = face_pose.position.y
    goal.target_pose.pose.orientation.x = face_pose.orientation.x
    goal.target_pose.pose.orientation.y = face_pose.orientation.y
    goal.target_pose.pose.orientation.z = 0.707 # face_pose.orientation.z
    goal.target_pose.pose.orientation.w = -0.707 #face_pose.orientation.w
    print(goal)

    face_detected = True
    goal_sent = False

def get_num(msg):
    global num
    num = msg.data

def start_service():
    global goal,face_detected,goal_sent,num
    face_detected = False
    goal_sent = False
    num = 0
    rospy.init_node('goals_node')
    goal = MoveBaseGoal()
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    # tf2_buffer = tf2_ros.Buffer()
    # tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    # odometry_subscriber = rospy.Subscriber ('/odom', Odometry, get_rotation)
    goal_subscriber = rospy.Subscriber('/face_position', Pose, update_goal, callback_args=tf2_buffer)
    face_num = rospy.Subscriber('/face_num', Int64, get_num)

    soundhandle = SoundClient()
    rospy.sleep(1)
    voice = 'voice_kal_diphone'
    volume = 1.0
    
    markers_pub = rospy.Publisher('/face_markers', MarkerArray, queue_size=1000)
    marker_array = MarkerArray()

    nc = rospy.Subscriber('face_num', Int64, get_num, queue_size=1000)


    id = 0

    # try:
    #     odometry = rospy.Subscriber ('/odom', Odometry)
    # except Exception as e:
    #     print(e)

    q = quaternion_from_euler(0, 0, 1.57)
    
    xs = [ 0.066, 1.7, 2.87, 2.05, 1.5, -0.6, -1 ]
    ys = [ 0.955, -1.2, -0.52, 2.5, 0.7, 0.2, 1.9 ]
    goals = [ i for i in range( len(xs) ) ]

    for i in range( len(goals) ):
        goals[i] = MoveBaseGoal()
        goals[i].target_pose.header.frame_id = "map";
        goals[i].target_pose.header.stamp = rospy.Time.now()
        goals[i].target_pose.pose.orientation.x = q[0]
        goals[i].target_pose.pose.orientation.y = q[1]
        goals[i].target_pose.pose.orientation.z = q[2]
        goals[i].target_pose.pose.orientation.w = q[3]
        goals[i].target_pose.pose.position.x = xs[i]
        goals[i].target_pose.pose.position.y = ys[i]



    # goals[0].target_pose.pose.position.x = 0.066306
    # goals[0].target_pose.pose.position.y = 0.955869
    # goals[1].target_pose.pose.position.x = 2.8784
    # goals[1].target_pose.pose.position.y = -0.521742
    # goals[2].target_pose.pose.position.x = 2.05
    # goals[2].target_pose.pose.position.y = 2.5;
    # goals[3].target_pose.pose.position.x = 1.5;
    # goals[3].target_pose.pose.position.y = 0.7;
    # goals[4].target_pose.pose.position.x = -0.6;
    # goals[4].target_pose.pose.position.y = 0.2;
    # goals[5].target_pose.pose.position.x = -1;
    # goals[5].target_pose.pose.position.y = 1.9;
    

    client.send_goal(goals[0])
    which = 0
    rotation = 0
    r = rospy.Rate(1.5)

    while not rospy.is_shutdown():

        print(num)
        id = id + 1

        if num == 3:
            print("FOUND ALL")
            break

        if face_detected and not goal_sent:
            client.cancel_goal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            # goal.target_pose.pose.orientation.w = 1
            # goal.target_pose.pose.orientation.x = 0
            # goal.target_pose.pose.orientation.y = 0
            # goal.target_pose.pose.orientation.z = 0

            pose = Pose()
            pose.position.x = goal.target_pose.pose.position.x
            pose.position.y = goal.target_pose.pose.position.y
            pose.position.z = goal.target_pose.pose.position.z
            marker = Marker()
            marker.header.stamp = rospy.Time(0)
            marker.header.frame_id = 'map'
            marker.pose = pose
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.id = id
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA(0, 0, 1, 1)
            marker_array.markers.append(marker)
            markers_pub.publish(marker_array)

            id = id +1
            try:
                odometry = rospy.wait_for_message("/odom", Odometry)
            except Exception as e:
                print(e)
            pose = Pose()
            pose.position.x = odometry.pose.pose.position.x
            pose.position.y = odometry.pose.pose.position.y
            pose.position.z = odometry.pose.pose.position.z
            marker = Marker()
            marker.header.stamp = rospy.Time(0)
            marker.header.frame_id = 'map'
            marker.pose = pose
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.id = id
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA(1, 0, 0, 1)
            marker_array.markers.append(marker)
            markers_pub.publish(marker_array)

            client.send_goal(goal)
            goal_sent = True

        state = client.get_state()
        print(client.get_goal_status_text())


        if state == 3:
            if face_detected:
                soundhandle.say("GREETINGS", voice, volume)
                print("FACE DETECTION GOAL REACHED")
                rotation = 0
                which += 1
                rospy.sleep(2)
                client.send_goal(goals[which])
                face_detected = False
                goal_sent = False
            else:
                if rotation == 2:
                    print("ROTATIONS DONE")
                    if which == len(goals)-1:
                        which = 0
                        continue
                    which += 1
                    rotation = 0
                    client.send_goal(goals[which])
                else:
                    print("GOAL REACHED")
                    # rotation_goal = generate_goal(rotation, goals[which].target_pose.pose.position.x, goals[which].target_pose.pose.position.y )
                    # client.send_goal( rotation_goal )
                    # rotation += 1
                    rotation = 2                    

        elif state == 2 or state == 8 or state == 4 or state == 9:
            if face_detected:
                client.send_goal(goals[which])
                face_detected = False
                rotation = 0
                goal_sent = False
            else:
                print("GOAL N {} FAILED".format(which+1));
                if which == len(goals)-1:
                    which = 0
                    continue
                which += 1
                client.send_goal(goals[which])
    


        r.sleep()



if __name__ == '__main__':
    start_service()
