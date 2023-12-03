#!usr/bin/env python3
import math
import numpy as np
from control.matlab import ss,c2d,dlqr

import rospy
from std_msgs.msg import Float64MultiArray
# from gy_87.msg import rpy

# default height of bot
default_length = 21/100

class pi_control:
    """
        Main Node on Raspberry Pi running control loop
        1.  Subscribes to IMU sensor for [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot]
            Topic: /gy_87/rpy_data
        2.  Subscribes to Arduino for encoder data of Wheel Motor [x, x_dot]
            Topic: encoder_state
        3.  Calculates LQR feedback using state variables subscribed in (1) and (2) and publishes to Wheel Motor Arduino
            Topic: pwmData
        4.  Hip Motor: Subscribe from motor for current angle to update height of bot in model/
            To-Do: UI for desired length and publish to Arduino hip motor angle required for given Length

        Attributes: 
            euler_angles, angular_velocity, position_data: state variables
            A, B : State Matrix, Input Matrix
            Q, Rl : Weight Matrix for states, weights on the control input in LQR cost function
            imu_sub, encoder_sub: Subscribers for getting state variables
            lqr_pub: Publisher lqr feedback output u = -k.x to Wheel Motors
    """
    def __init__(self):
        self.imu_sub = rospy.Subscriber('/gy_87/rpy_data', Float64MultiArray, self.imu_callback)
        self.encoder_sub = rospy.Subscriber('encoder_state', Float64MultiArray, self.wheel_encoder_callback)

        self.euler_angles = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.position_data = [0.0, 0.0]

        self.A, self.B  = self.calculate_A_B_matrix(default_length)
        self.Q, self.Rl  = self.set_Q_R_matrix()

        self.lqr_pub = rospy.Publisher('pwmData', Float64MultiArray, queue_size=100) 

    def calculate_A_B_matrix(self, length):
        """
            Calculate discretized state and input matrices A, B for a particular height of bot

            Args:
                length: length variable in inverted pendulum on a cart model

            Returns:
                [disc_A, disc_B] : dicretized A, B matrices
        """
        g = 9.81
        m = 0.034
        R = 0.066
        Jw = m*R*R
        M = 1.16
        D = 0.2
        H = 0.07
        W = 0.22

        L = length   
        Jpsi = (M*L*L)/3

        Jphi = M*(W*W * D*D)/12
        Jm = (m*R*R)/2
        Rm = 10
        Kb = 0.0006589
        Kt = 0.0375
        n = 52.1
        fm = 0.0022
        fpi = 0
        fw = 0
        alpha = (n*Kt)/Rm
        beta = ((n*Kt*Kb)/Rm)+fm

        E11 = ((2*m+M)*(R*R))+(2*Jw)+(2*(n*n)*Jm)
        E12 = (M*L*R)-(2*(n*n)*Jm)
        E21 = (M*L*R)-(2*(n*n)*Jm)
        E22 = (M*(L*L))+Jpsi+(2*(n*n)*Jm)

        x11 = -(g*M*L*E12)/((E11*E22)-(E12*E21));
        x12 = -2*((((beta+fw)*E22)+(beta*E12))/((E11*E22)-(E12*E21)));
        x13 = ((2*beta)*(E22+E12))/((E11*E22)-(E12*E21));
        x21 = (g*M*L*E11)/((E11*E22)-(E12*E21));
        x22 = 2*((((beta+fw)*E12)+(beta*E11))/((E11*E22)-(E12*E21)));
        x23 = -((2*beta)*(E11+E12))/((E11*E22)-(E12*E21));
        x31 = ((W*W)*(beta+fw))/((2*R*R)*((.5*m*(W*W))+Jphi+(.5*((W/R)*(W/R))*(Jw+(n*n)*Jm))));
        B4 = alpha*(E22 + E12)/((E11*E22)-(E12*E21));
        B5 = -alpha*(E11 + E12)/((E11*E22)-(E12*E21));
        B6 = (W*alpha)/((2*R)*((.5*m*(W*W))+Jphi+(.5*((W/R)*(W/R))*(Jw+(n*n)*Jm))));

        A = np.matrix([[0,0,0,1,0,0],
                        [0,0,0,0,1,0],
                        [0,0,0,0,0,1],
                        [0,x11,0,x12,x13,0],
                        [0,x21,0,x22,x23,0],
                        [0,0,0,0,0,x31]])
        
        B = np.matrix([[0,0],
                        [0,0],
                        [0,0],
                        [B4,B4],
                        [B5,B5],
                        [-B6,B6]])

        # C, D : Output matrix, Feed-forward matrix
        C = np.identity(6, dtype = int)
        D = np.zeros((6, 2), dtype = int)

        systemCont = ss(A,B,C,D)

        # Convert a continuous time system to discrete time by sampling (sample time = 0.007s 150Hz)
        systemDisc = c2d(systemCont, 0.007)

        return [systemDisc.A,systemDisc.B]

    def set_Q_R_matrix(self):
        """
            Setter function for Q and R matrices representing weights on states and control input respectively

            Args:
                none

            Returns:
                [Q, R]
        """
        
        Q = np.matrix([ [100,0,0,0,0,0],
                        [0,1000,0,0,0,0],
                        [0,0,1,0,0,0],
                        [0,0,0,1,0,0],
                        [0,0,0,0,10,0],
                        [0,0,0,0,0,1] ])
            
        Rl = np.matrix([[1,0],
                       [0,1]])

        return [Q,Rl]

    def imu_callback(self, msg):
        """
            Callback for receiving IMU data and updating states

            Args:
                Float64MultiArray [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot]

            Returns:
                none
        """
        self.euler_angles[0]= msg.data[0]
        self.euler_angles[1]= msg.data[1]
        self.euler_angles[2]= msg.data[2]
        self.angular_velocity[0] = msg.data[3]
        self.angular_velocity[1] = msg.data[4]
        self.angular_velocity[2] = msg.data[5]

    def wheel_encoder_callback(self, msg):
        """
            Callback for receiving encoder data and updating states

            Args:
                Float64MultiArray [x, x_dot]

            Returns:
                none
        """
        self.position_data[0]= msg.data[0]
        self.position_data[1]= msg.data[1]

    def publish_lqr(self, event = None):
        """
            Calculate feedback from lqr control loop and publish output u = -k.x to wheel motors as pwm

            Args:
                none

            Returns:
                none
        """
        msg = Float64MultiArray()
        y_setpoint = [0.000,0,0,0,0,0]
        y_current = [self.position_data[0], self.euler_angles[1], self.euler_angles[2], 
                     self.position_data[1], self.angular_velocity[1], self.angular_velocity[2]] 

        self.A,self.B = self.calculate_A_B_matrix(default_length)

        # Discrete-time linear quadratic regulator
        k,S,E = dlqr(self.A,self.B,self.Q,self.Rl)

        c = [10,10,10,5,1,1]
        for i in range(2):
            for j in range(6):
                if(i < 1):
                    u1 += k[i,j]*((y_current[j]-y_setpoint[j])*c[j])
                else:
                    u2 += k[i,j]*((y_current[j]-y_setpoint[j])*c[j])
        
        # bottom-capping pwm to 25
        if(abs(u1) < 25):
            u1 = (abs(u1)/u1) * 25
        if(abs(u2) < 25):
            u2 = (abs(u2)/u2) * 25

        msg.data = [-int(u1), int(u2)]
        self.lqr_pub.publish(msg)

    def length_to_motor_angle(L):
        """
            To-Do: Giving angle to Hip Motor for a particular height
        """
        a = 0.08
        b = 0.14
        L3 = 0.06
        c = math.sqrt((L*L)/10000 + L3*L3)
        theta = math.acos((a*a + c*c - b*b)/(2*a*c)) - 0.05

def main():
    """
        1. Init pi_control object containing attributes & functions defining state-space model
        2. Init ros node rpi_control which uses rosTimer to publish lqr feedback pwm to wheel motors at frequency of 150 Hz

        Args:
            none

        Returns:
            none
    """
    rospy.init_node('rpi_control', anonymous=False)
    pi_control_node = pi_control()
    print("Control Node Init")

    timer = rospy.Timer(rospy.Duration(0.01), pi_control_node.publish_lqr)
    rospy.spin()    
    timer.shutdown()

# Main Loop
if __name__ == '__main__':
    main()
