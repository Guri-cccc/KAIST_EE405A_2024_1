#!/usr/bin/python3
# coding=utf8
# Date:2022/03/30
import sys
import math
import rospy
import smbus2
from threading import Thread
from std_msgs.msg import *
from chassis_control.msg import *
from armpi_pro import Misc

ENCODER_MOTOR_MODULE_ADDRESS = 0x34

th = None
slow_en = True


class EncoderMotorController:
    def __init__(self, i2c_port, motor_type=3):
        self.i2c_port = i2c_port
        with smbus2.SMBus(self.i2c_port) as bus:
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 20, [motor_type, ])

    def set_speed(self, speed, motor_id=None, offset=0):
        global th
        # 通过IIC发布控制信息到电机驱动板
        with smbus2.SMBus(self.i2c_port) as bus:
            try:
                if motor_id is None:
                    bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 51 + offset, speed)
                else:
                    if 0 < motor_id <= 4:
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                    else:
                        raise ValueError("Invalid motor id")

            except Exception as e:
                th = None
                print(e)

# 麦轮子底盘速度处理
class MecanumChassis:
    # A = 110  # mm
    # B = 97.5  # mm
    # WHEEL_DIAMETER = 96.5  # mm
    # PULSE_PER_CYCLE = 44
    def __init__(self, a=110, b=97.5, wheel_diameter=96.5, pulse_per_cycle=44 * 178):
        self.motor_controller = EncoderMotorController(1)
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.pulse_per_cycle = pulse_per_cycle
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def speed_covert(self, speed):
        """
        covert speed mm/s to pulse/10ms
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter) * self.pulse_per_cycle * 0.01  # pulse/10ms

    def reset_motors(self):
        for i in range(1, 5):
            self.motor_controller.set_speed(i, 0)
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def set_velocity(self, velocity, direction, angular_rate, fake=False):
        """
        Use polar coordinates to control moving
        motor3 v2|  ↑  |v1 motor1
                 |     |
        motor4 v3|     |v4 motor2
        :param velocity: mm/s
        :param direction: Moving direction 0~360deg, 180deg<--- ↑ ---> 0deg
        :param angular_rate:  The speed at which the chassis rotates
        :param fake:
        :return:
        """
        velocity = -velocity
        angular_rate = -angular_rate
        
        rad_per_deg = math.pi / 180
        vx = velocity * math.cos(direction * rad_per_deg)
        vy = velocity * math.sin(direction * rad_per_deg)
        vp = angular_rate * (self.a + self.b)
        v1 = vy - vx + vp
        v2 = vy + vx - vp
        v3 = vy - vx - vp
        v4 = vy + vx + vp
        v_s = [int(self.speed_covert(v)) for v in [-v1, v4, v2, -v3]]
        if fake:
            return v_s

        self.motor_controller.set_speed(v_s)
        self.velocity = velocity
        self.direction = direction
        self.angular_rate = angular_rate
    
    # 控制XY方向平移函数
    def translation(self, velocity_x, velocity_y, fake=False):
        global slow_en
        
        velocity = math.sqrt(velocity_x ** 2 + velocity_y ** 2)
        if velocity_x == 0:
            direction = 90 if velocity_y >= 0 else 270  # pi/2 90deg, (pi * 3) / 2  270deg
        else:
            if velocity_y == 0:
                direction = 0 if velocity_x > 0 else 180
            else:
                direction = math.atan(velocity_y / velocity_x)  # θ=arctan(y/x) (x!=0)
                direction = direction * 180 / math.pi
                if velocity_x < 0:
                    direction += 180
                else:
                    if velocity_y < 0:
                        direction += 360
        if fake:
            return velocity, direction
        
        else:
            th = Thread(target=slow_velocity,args=(velocity, direction, 0))
            th.setDaemon(True)
            th.start()


last_velocity = 0
last_angular = 0
last_direction = 90
# 缓变速处理函数
def slow_velocity(velocity, direction, angular):
    global th
    global last_velocity
    global last_angular
    global last_direction

    added_v = 30
    added_a = 0.2
    diff_velocity = velocity - last_velocity
    diff_angular = angular - last_angular

    if abs(diff_velocity) >= added_v or abs(diff_angular) >= added_a:
        if diff_velocity > added_v:
            dv = added_v
        elif diff_velocity < -added_v:
            dv = -added_v
        else:
            dv = 0

        if diff_angular > added_a:
            da = added_a
        elif diff_angular < -added_a:
            da = -added_a
        else:
            da = 0

        direction_ = direction
        if velocity == 0 and direction <= 0 and angular == 0:
            direction = last_direction
        last_direction = direction_

        while abs(diff_velocity) > added_v or abs(diff_angular) > added_a:
            if abs(diff_velocity) >= added_v:
                last_velocity += dv
            if abs(diff_angular) >= added_a:
                last_angular += da

            diff_velocity = velocity - last_velocity
            diff_angular = angular - last_angular
            last_velocity = round(last_velocity,2)
            last_angular = round(last_angular,2)
            chassis.set_velocity(last_velocity,direction,last_angular)
            rospy.sleep(0.05)

        last_velocity = velocity
        last_angular = angular
        chassis.set_velocity(last_velocity,direction,last_angular)

    else:
        last_angular = angular
        last_velocity = velocity
        last_direction = direction
        chassis.set_velocity(velocity,direction,angular)

    th = None

# 平移控制回调函数
def Set_Translation(msg):
    
    velocity_x = msg.velocity_x
    velocity_y = msg.velocity_y
    chassis.translation(velocity_x, velocity_y)

# 普通控制回调函数
def Set_Velocity(msg):
    global th

    velocity = msg.velocity
    direction = msg.direction
    angular = round(msg.angular,2)
    
    if th is None: # 通过子线程去控制缓变速
        th = Thread(target=slow_velocity,args=(velocity, direction, angular))
        th.setDaemon(True)
        th.start()


if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('chassis_control', log_level=rospy.DEBUG)
    # app通信服务
    set_velocity_sub = rospy.Subscriber('/chassis_control/set_velocity', SetVelocity, Set_Velocity)
    set_translation_sub = rospy.Subscriber('/chassis_control/set_translation', SetTranslation, Set_Translation)

    chassis = MecanumChassis()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
