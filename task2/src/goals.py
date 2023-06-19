#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Vector3,Twist
from std_msgs.msg import ColorRGBA, Int64, String, Float32MultiArray
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import tf2_ros
import numpy as np


goal = None
face_deteced = None
goal_sent = None
num = None
ring_num = 0
prison_pos = None
cylinder_num = 0
marker_array = None
markers_pub = None
id = 0
recognized = None
poster_num = 0
hint = False
parking = None

cylinder_pd = dict()   # R G B Y
ring_pd = dict()  # R G B L



def generate_goal(rotation, x, y):
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

    goal.target_pose.pose.position.x = face_pose.position.x
    goal.target_pose.pose.position.y = face_pose.position.y
    goal.target_pose.pose.orientation.x = face_pose.orientation.x
    goal.target_pose.pose.orientation.y = face_pose.orientation.y
    goal.target_pose.pose.orientation.z = face_pose.orientation.z
    goal.target_pose.pose.orientation.w = face_pose.orientation.w

    face_detected = True
    goal_sent = False

def get_num(msg):
    global num
    # num = msg.data

def get_posters(msg):
    global poster_num
    poster_num = msg.data
    rospy.loginfo(poster_num)
    rospy.loginfo(f"posters={poster_num}")

def get_parking(msg):
    global parking
    parking = msg.data
    rospy.loginfo(f"got parking={parking}")


def get_recog(msg):
    global recognized
    # recognized = msg.data.strip()
    recognized = msg.data
    rospy.loginfo(recognized)

def get_ring_num(msg):
    # print("UPDATING RINGS")

    global ring_num, ring_pd
    ring_num  =  len(msg.markers)/2

    #get parking loc under ring
    p = msg.markers[-1].pose
    c = msg.markers[-1].color
    if c.g > 0.9 and c.r < 0.3 and c.b < 0.3: #green
        ring_pd["G"] = p
    elif c.r > 0.9 and c.g < 0.35 and c.b < 0.35: #red
        ring_pd["R"] = p
    elif c.g > 0.9 and c.b > 0.9: # blue
        ring_pd["B"] = p
    else:
        ring_pd["L"] = p


def get_cylinder_num(msg):
    global cylinder_num,cylinder_pd, marker_array, markers_pub,id
    cylinder_num  =  len(msg.markers)/2

    # rospy.loginfo("UPDATING CYLINDERS")

    p = msg.markers[-1].pose
    pp = msg.markers[-2].pose
    c = msg.markers[-1].color


    #calc cylinder parking spot and orientation
    k = np.array([ ( p.position.x - pp.position.x ), ( p.position.y - pp.position.y )   ]) 
    k /= np.linalg.norm(np.array([  ( p.position.x - pp.position.x ), ( p.position.y - pp.position.y )  ]))
    k *= 0.48

    # ang = np.arctan2( k[1] , k[0] ) * 180 / np.pi - 180
    ang = np.arctan2( k[1] , k[0] ) 
    x,y,z,w = quaternion_from_euler(0,0,ang+np.pi)

    pous = Pose()
    # pous.position.x = p.position.x
    # pous.position.y = p.position.y
    # pous.position.z = p.position.z
    pous.position.x = pp.position.x + k[0]
    pous.position.y = pp.position.y + k[1]
    pous.position.z = pp.position.z
    pous.orientation.x = x
    pous.orientation.y = y
    pous.orientation.z = z
    pous.orientation.w = w

    id = id +1
    marker = Marker()
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = 'map'
    marker.pose = pous
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.frame_locked = False
    marker.id = id
    marker.scale = Vector3(0.2, 0.04, 0.4)
    marker.color = ColorRGBA(0.15, 0.15, 0.3, 1)
    marker_array.markers.append(marker)
    markers_pub.publish(marker_array)

    rospy.loginfo(c)

    #get parking loc for cyl
    if c.g > 0.7 and c.r < 0.65 and c.b < 0.65: #green
        cylinder_pd["G"] = pous
    elif c.r > 0.7 and c.g < 0.65 and c.b < 0.65: #red
        cylinder_pd["R"] = pous
    elif c.g > 0.75 and c.r > 0.75 and c.b < 0.65: # yellow
        cylinder_pd["Y"] = pous
    else:
        cylinder_pd["B"] = pous

def start_service():
    global parking,hint,goal,face_detected,goal_sent,num,ring_num,prison_pos,cylinder_num,ring_pd,cylinder_pd,marker_array,markers_pub,id,recognized,poster_num
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
    ring_num = rospy.Subscriber('/detected_rings', MarkerArray, get_ring_num)
    cylinder_sub = rospy.Subscriber('/detected_cylinders', MarkerArray, get_cylinder_num)
    arm_pub = rospy.Publisher("/arm_command", String, queue_size=100)
    soundhandle = SoundClient()
    rospy.sleep(1)
    voice = 'voice_kal_diphone'
    volume = 1.0
    markers_pub = rospy.Publisher('/face_markers', MarkerArray, queue_size=1000)
    marker_array = MarkerArray()
    nc = rospy.Subscriber('face_num', Int64, get_num, queue_size=1000)
    recognizer_sub = rospy.Subscriber('recognized', Float32MultiArray, get_recog, queue_size=100)
    arm_pub.publish( 'retract' )
    twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi",Twist, queue_size=50)
    posters = rospy.Subscriber('/posters', Int64, get_posters, queue_size=10)
    park_sub = rospy.Subscriber('/park', String, get_parking, queue_size=1000)
    goal_pub = rospy.Publisher("/goal",String, queue_size=50)



    xs = [  0.145,  -0.075,   3.425,  1.8,   1.1,     2.3,   1.15,   -0.7 ]
    ys = [  -0.975,  0.8425,    -0.475, 2.65,  0.07,  0.975, -1.15, 1.65  ]     
    goals = [ i for i in range( len(xs) ) ]
    for i in range( len(goals) ):
        goals[i] = MoveBaseGoal()
        goals[i].target_pose.header.frame_id = "map";
        goals[i].target_pose.header.stamp = rospy.Time.now()
        if i == 10:
            q = quaternion_from_euler(0, 0, -1.2)
        elif  i == 5:
            q = quaternion_from_euler(0, 0, -2.6)
        elif i == 4 or i == 0:
            q = quaternion_from_euler(0, 0, 0.8)
        elif i == 1  or i == 7 or i == 6:
            q = quaternion_from_euler(0, 0, -2.4)
        elif i == 2 :
            q = quaternion_from_euler(0, 0, 1.5)
        elif i == 3:
            q = quaternion_from_euler(0, 0, -2.8)
        else:
            q = quaternion_from_euler(0, 0, 0)
        goals[i].target_pose.pose.orientation.x = q[0]
        goals[i].target_pose.pose.orientation.y = q[1]
        goals[i].target_pose.pose.orientation.z = q[2]
        goals[i].target_pose.pose.orientation.w = q[3]
        goals[i].target_pose.pose.position.x = xs[i]
        goals[i].target_pose.pose.position.y = ys[i]


    client.send_goal(goals[0])
    which = 0
    rotation = 0
    r = rospy.Rate(2)
    prison_sent = False
    park = False
    goto_prison = False
    goto_cyl = False
    cyl_reached = False

    cyls = []
    cyl_i = 0

    def get_colors(string):
        find = ["red","green","blue","yellow"]
        clrs = ["R","G","B","Y"]
        for substring in string.replace(","," ").split():
            if substring in find:
                rospy.loginfo(substring)
                cyls.append( clrs[ find.index(substring) ] )
                hint = True
        return hint
    
    while not rospy.is_shutdown():
        
        id = id + 1
        state = client.get_state()
        # print(client.get_goal_status_text())
        # print(which)


        if cylinder_num == 4 and not goto_cyl and poster_num == 3 and len(cyls) == 2:
            prison_pos = cylinder_pd[ cyls[cyl_i] ]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = prison_pos.position.x
            goal.target_pose.pose.position.y = prison_pos.position.y
            goal.target_pose.pose.orientation.x = prison_pos.orientation.x
            goal.target_pose.pose.orientation.y = prison_pos.orientation.y
            goal.target_pose.pose.orientation.z = prison_pos.orientation.z
            goal.target_pose.pose.orientation.w = prison_pos.orientation.w
            client.send_goal(goal)
            r.sleep()
            arm_pub.publish( 'cyl' )
            rospy.loginfo("PUBLISHING CYL")
            goto_cyl = True 
            r.sleep()
            continue
        if goto_cyl and not cyl_reached:
            if state == 3:
                cyl_reached = True
            else:
                r.sleep()
                continue
        if goto_cyl and cyl_reached:
            if (recognized == None or recognized == [] or recognized == () or recognized == "") and state == 3 :
                # rospy.loginfo("moving")
                twist = Twist()
                twist.linear.x = 0.029
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                twist_pub.publish(twist)
            else:
                rospy.loginfo(cyl_i)
                rospy.loginfo(goto_cyl)
                rospy.loginfo(cyl_reached)
                rospy.loginfo("RECOGNIZED A FACE")
                if recognized[0] < 0.5:
                    rospy.loginfo("NOT THE RIGHT ONE")
                    cyl_i += 1
                    goto_cyl = False
                    cyl_reached = False
                    arm_pub.publish( 'retract' )
                    recognized = None
                    # continue
                else:
                    rospy.loginfo("RIGHT ONE")
                    arm_pub.publish( 'retract' )
                    goto_cyl = False
                    cyl_reached = False
                    recognized = None
                    #temporary
                    cylinder_num = 5
                    ring_num = 5
                    # continue
            r.sleep()
            continue




        if ring_num == 5 and cylinder_num == 5:
            goto_prison = True
        if goto_prison and not prison_sent:
            prison_pos = ring_pd[ parking ]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = prison_pos.position.x
            goal.target_pose.pose.position.y = prison_pos.position.y
            goal.target_pose.pose.orientation.x = prison_pos.orientation.x
            goal.target_pose.pose.orientation.y = prison_pos.orientation.y
            goal.target_pose.pose.orientation.z = prison_pos.orientation.z
            goal.target_pose.pose.orientation.w = prison_pos.orientation.w
            client.send_goal(goal)
            prison_sent = True
            r.sleep()
            continue
        if goto_prison and prison_sent and not park:
            if state == 3:
                park = True
                arm_pub.publish( 'extend' )
            else:
                r.sleep()
                continue
        if park:
            r.sleep()
            print("parking phase")
            continue






        if face_detected and not goal_sent:
            client.cancel_goal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            goal_sent = True

     


        if state == 3:
            if face_detected:
                soundhandle.say("GREETINGS", voice, volume)
                print("FACE DETECTION GOAL REACHED")

                if len(cyls) != 2:
                    goal_pub.publish( "question" )
                    try:
                        msg = rospy.wait_for_message('/input', String, timeout=15)
                        print(msg.data)
                        if get_colors(msg.data):
                            goal_pub.publish( "end" )
                    except Exception as e:
                        rospy.loginfo("no hint")

                num += 1
                rotation = 0
                # which += 1
                print(num)
                rospy.sleep(2)
                if num == 11:
                    print("FOUND ALL")
                    break
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
