#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy

from sensor_msgs.msg import Joy,Imu
from geometry_msgs.msg import Twist

from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64


USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

body = [0.3762, 0.0935]
 
legs = [0.,0.08, 0.213, 0.213] 


a1_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

command_topics = ["/aliengo_gazebo/FR_hip_joint/command",
                  "/aliengo_gazebo/FR_thigh_joint/command",
                  "/aliengo_gazebo/FR_calf_joint/command",
                  "/aliengo_gazebo/FL_hip_joint/command",
                  "/aliengo_gazebo/FL_thigh_joint/command",
                  "/aliengo_gazebo/FL_calf_joint/command",
                  "/aliengo_gazebo/RR_hip_joint/command",
                  "/aliengo_gazebo/RR_thigh_joint/command",
                  "/aliengo_gazebo/RR_calf_joint/command",
                  "/aliengo_gazebo/RL_hip_joint/command",
                  "/aliengo_gazebo/RL_thigh_joint/command",
                  "/aliengo_gazebo/RL_calf_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))


if USE_IMU:
    rospy.Subscriber("aliengo_imu/base_link_orientation",Imu,a1_robot.imu_orientation)

rospy.Subscriber("/twist_mux/cmd_vel", Twist, a1_robot.cmd_vel_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

while not rospy.is_shutdown():

    leg_positions = a1_robot.run()
    a1_robot.change_controller()

    dx = a1_robot.state.body_local_position[0]
    dy = a1_robot.state.body_local_position[1]
    dz = a1_robot.state.body_local_position[2]
    
    roll = a1_robot.state.body_local_orientation[0]
    pitch = a1_robot.state.body_local_orientation[1]
    yaw = a1_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)

        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
    except:
        pass

    rate.sleep()
