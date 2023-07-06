#!/usr/bin/env python3
import rospy
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf

# Calculation of the current TCP-position and -orientation of Robot2
class GetFK(object):
    def __init__(self, fk_link, frame_id):
        
        self.fk_link = fk_link
        self.frame_id = frame_id
        self.fk_srv = rospy.ServiceProxy('/robot22/compute_fk', GetPositionFK)
        self.fk_srv.wait_for_service()
        self.last_js = None
        self.data_to_send = Float64MultiArray()
        self.js_sub = rospy.Subscriber('/robot2/joint_states', JointState, self.js_cb, queue_size=10)
        self.pub_TCP_robot2 = rospy.Publisher("/robot2/TCP_robot2", Float64MultiArray, queue_size=10)
        
    def get_current_fk_pose(self):
        resp = self.get_current_fk()
        if len(resp.pose_stamped) >= 1:
            return resp.pose_stamped[0]
        return None

    def get_current_fk(self):
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.sleep(0.1)
            self.h = self.get_fk(self.last_js)
            tcp = self.tcpcalculate(self.h)
        return tcp

    def get_fk(self, joint_state, fk_link=None, frame_id=None):
        
        if fk_link is None:
            fk_link = self.fk_link

        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [self.fk_link]
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
            return resp

    def tcpcalculate(self, resp):
        quaternion = [
            resp.pose_stamped[0].pose.orientation.x,
            resp.pose_stamped[0].pose.orientation.y,
            resp.pose_stamped[0].pose.orientation.z,
            resp.pose_stamped[0].pose.orientation.w]

        x = resp.pose_stamped[0].pose.position.x
        y = resp.pose_stamped[0].pose.position.y
        z = resp.pose_stamped[0].pose.position.z

        tcp = [x, y, z, quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, z), (quaternion[0], quaternion[1], quaternion[2], quaternion[3]), rospy.Time.now(), "robot22_tf/tool0", "robot22_tf/base_link")
        self.data_to_send.data = tcp
        self.pub_TCP_robot2.publish(self.data_to_send) 
        #print(tcp1)
        return tcp

    def js_cb(self, data):
        self.last_js = data

if __name__ == '__main__':
    rospy.init_node('FK_rob2')
    while not rospy.is_shutdown():
        rate = rospy.Rate(500)
        gfk = GetFK('tool0', 'base_link')
        resp = gfk.get_current_fk()