#!/usr/bin/python3
# coding=utf8
import sys
import time
import rospy
import rospkg

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from llm_msgs.msg import task_plan
from models.Armpi_Pro_code.armpi_pro_kinematics.kinematics import ik_transform
from models.Armpi_Pro_code.armpi_pro_common.armpi_pro import bus_servo_control

class PickandPlace():
    def __init__(self):
        rospy.init_node('pick_and_place_node')
        self.package_path = rospkg.RosPack().get_path('manipulation')
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.finish_sign = rospy.Publisher('/finish_sign', Bool, queue_size=1)
        self.move_base = rospy.Publisher('/move_base', Point, queue_size=1)
        self.inverse_kinematics = ik_transform.ArmIK()
        self.target_pose = PoseStamped()
        self.move_base_pose = Point()
        self.init_pose = (0.0, 0.18, 0.13)
        self.mode = ""
        self.finish = ""
        
        rospy.Subscriber('/target_pose', PoseStamped, self.target_callback, queue_size = 1)
        rospy.Subscriber('/task_plan', task_plan, self.llm_callback, queue_size=1)
        # rospy.Subscriber('/finish_sign', Bool, self.finish_callback, queue_size=1)
        
    def target_callback(self, msg):
        self.target_pose = msg
        # print("x is " , self.target_pose.pose.position.x)
                
    def llm_callback(self, msg):
        llm_message = msg
        self.mode = llm_message.mode
        
    def arm_planning(self):
        target = self.inverse_kinematics.setPitchRanges((self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z), -145, -180, 0)
        # target = self.inverse_kinematics.setPitchRanges(self.init_pose)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 50), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        time.sleep(1.5)
        self.grasping(self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z)
        self.basket_planning()
        self.placing()
        self.finish = True
        self.finish_sign.publish(self.finish)
        self.init()
        time.sleep(10)
    
    def init(self):
        target = self.inverse_kinematics.setPitchRanges(self.init_pose, -90, -180, 0)
        if target: 
            servo_data = target[1]
            bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 50), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        time.sleep(1.5)
    
    def grasping(self, target_x, target_y, target_z):
        target = self.inverse_kinematics.setPitchRanges((target_x, target_y, target_z), -145, -180, 0)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 450), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        time.sleep(1.5)
        
    def basket_planning(self):
        bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 450), (2, 500), (3, 900),
                            (4, 365),(5, 460),(6, 500)))
        time.sleep(1.5)
        
    def placing(self):
        bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 200), (2, 500), (3, 900),
                            (4, 365),(5, 460),(6, 500)))
        time.sleep(1.5)
        
def main():
    print("********RUNNING PICK & PLACE NODE************")
    p_p = PickandPlace()
    int_pos = p_p.init_pose
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if p_p.mode == "Grasping":
            # if abs(p_p.move_base.x) < 0.012 and abs(p_p.move_base.y) < 0.26:
                p_p.arm_planning()
            # else:
                # p_p.init()
        else:
            p_p.init()
        rate.sleep()

# Main
if __name__ == '__main__':
    main()
