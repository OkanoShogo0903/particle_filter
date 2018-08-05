from visualization_msgs.msg import Marker
import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist,Quaternion,PoseWithCovarianceStamped,PoseArray,Pose
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates

def visualizeParticle(particles):
    particle_msg = PoseArray()
    particle_msg.header.frame_id = "map"
    particle_msg.header.stamp = rospy.Time.now()
    for i in range(0,len(particles)):
        pose = Pose()
        pose.position.x = particles[i,0]
        pose.position.y = particles[i,1]
        pose.position.z = 0
        alpha = particles[i,2]
        q = tf.transformations.quaternion_from_euler(0,0,alpha)

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        particle_msg.poses.append(pose)
    particle_pub.publish(particle_msg)


def visualizeLocalizedRobotPose(particles):
    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "localized_robot_pose"
    marker_data.id = 1
    marker_data.action = Marker.ADD

    # x,y sum
    x,y,pitch = particles.mean(axis=0)

    # setting
    q = tf.transformations.quaternion_from_euler(0,0,pitch)
    marker_data.pose.position.x = x
    marker_data.pose.position.y = y
    marker_data.pose.position.z = 0

    marker_data.pose.orientation.x = q[0]
    marker_data.pose.orientation.y = q[1]
    marker_data.pose.orientation.z = q[2]
    marker_data.pose.orientation.w = q[3]

    marker_data.color.r = 0.0
    marker_data.color.g = 1.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0

    marker_data.scale.x = 0.3
    marker_data.scale.y = 0.1
    marker_data.scale.z = 0.1

    marker_data.lifetime = rospy.Duration()
    marker_data.type = 0
    predict_marker_pub.publish(marker_data)

def visualizeSimulatedRobotPoseCB(model_state):
    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "simulated_robot_pose"
    marker_data.id = 1
    marker_data.action = Marker.ADD
    marker_data.pose.position.x = model_state.pose[2].position.x
    marker_data.pose.position.y = model_state.pose[2].position.y
    marker_data.pose.position.z = 0
    marker_data.pose.orientation.x = model_state.pose[2].orientation.x
    marker_data.pose.orientation.y = model_state.pose[2].orientation.y
    marker_data.pose.orientation.z = model_state.pose[2].orientation.z
    marker_data.pose.orientation.w = model_state.pose[2].orientation.w
    marker_data.color.r = 0.0
    marker_data.color.g = 0.0
    marker_data.color.b = 1.0
    marker_data.color.a = 1.0

    marker_data.scale.x = 0.3
    marker_data.scale.y = 0.1
    marker_data.scale.z = 0.1
    marker_data.lifetime = rospy.Duration()
    marker_data.type = 0
    gazebo_marker_pub.publish(marker_data)
    

particle_pub = rospy.Publisher('/particle_info',PoseArray,queue_size=1)
predict_marker_pub = rospy.Publisher('/predict_pose',Marker,queue_size=1)
gazebo_marker_pub = rospy.Publisher('/gazebo_pose',Marker,queue_size=1)
