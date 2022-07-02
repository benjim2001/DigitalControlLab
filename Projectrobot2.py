
import math 

import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import enum

from PIDClass import *


class MessageType(enum.IntEnum):
    BALL_NOT_REACHABLE = 1
    BALL_BY_TEAMMATE = 2
    BALL_BY_OPPONENT = 3


class MyRobot2(RCJSoccerRobot):
    pcontroll = PID(35, 0.001, 0.005, 0, 0)
    Tcontroll = PID(65, 25, 0, 10, -10)

    BPcontroll = PID(10, 3, 0, 10, -10)
    BTcontroll = PID(25, 4, 0, 10, -10)
    GPcontroll = PID(12, 0, 0.003, 0, 0)
    GTcontroll = PID(10, 20, 0, 20, -20)

    def run(self):
        ball_data = None
        dt = None
        inplace = False
        inplace_tick = 0
        l = 0
        headingp = 0
        positionp = 0

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                if data['waiting_for_kickoff']:
                    headingp = 0
                    positionp = 0
                    l = 0
                    inplace = False

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()['robot_id']  # noqa: F841
                    if team_data == MessageType.BALL_BY_OPPONENT:
                        # Handel grabbing the ball
                        get_ball = True
                    if team_data == MessageType.BALL_BY_TEAMMATE:
                        # Don't grab the ball
                        get_ball = False
                    if team_data == MessageType.BALL_NOT_REACHABLE:
                        # Return with ball data in next packets
                        pass

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    self.send_data_to_team(MessageType.BALL_NOT_REACHABLE)
                    ball_data = None

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841

                compass_values = self.compass.getValues()

                t1 = time.time()
                T = 0.000003
                # Write scripts of the robot here

                if ball_data and ball_data['strength'] > 10 and inplace:
                    headingp = 0
                    positionp = 0
                    l = 0
                    e_ball_position = 1 / ball_data['strength'] * 2.5
                    e_ball_theta = math.atan2(
                        ball_data["direction"][0], ball_data["direction"][1]) - (math.pi / 2)
                    e_goal_distance = math.sqrt(
                        (robot_pos[0])**2 + (-8 + robot_pos[1])**2)

                    v_ball_position = self.BPcontroll.update(
                        e_ball_position, dt)
                    v_ball_theta = self.BTcontroll.update(
                        e_ball_theta, dt)
                    v_goal_theta = self.GTcontroll.update(heading, dt)
                    v_goal_position = self.GPcontroll.update(
                        e_goal_distance, dt)

                    vr = 0
                    vl = 0

                    if abs(v_ball_position) > 0.01:
                        w = v_ball_theta
                        v = v_ball_position
                        if abs(w) < 0.1: 
                            w = 0
                            self.BPcontroll.p_error = 50
                            self.BPcontroll.i_gain = 0
                        else: 
                            self.BPcontroll.p_error = 10
                            self.BPcontroll.i_gain = 5

                            v = self.BPcontroll.update(e_ball_position, dt)
                        vr = (2 * v + 0.085 * w) / 0.04
                        vl = (2 * v - 0.085 * w) / 0.04
                    elif abs(heading) > 0.1:
                        if heading > 0:
                            vr = -v_goal_theta
                            vl = v_goal_theta
                        else: 
                            vr = v_goal_theta
                            vl = -v_goal_theta
                    else:
                        vl = 10
                        vr = 10

                    self.left_motor.setVelocity(vl)
                    self.right_motor.setVelocity(vr)

                elif abs(robot_pos[0]) > 0.01 and (headingp == 0 or positionp == 0):
                    if abs(heading - math.pi / 2) > 0.1 and headingp == 0:
                        u = self.Tcontroll.update(
                            heading - math.pi / 2, dt)
                        self.left_motor.setVelocity(-u)
                        self.right_motor.setVelocity(u)

                    elif abs(robot_pos[0]) > 0.01 and positionp == 0:
                        u = self.pcontroll.update(-robot_pos[0], dt)
                        if u > 0:
                            u += 3
                        elif u < 0: 
                            u -= 3

                        self.left_motor.setVelocity(u)
                        self.right_motor.setVelocity(u)
                elif abs(robot_pos[1] - 0.55) > 0.01:
                    if abs(heading) > 0.1:
                        u = self.Tcontroll.update(heading, dt)
                        self.left_motor.setVelocity(-u)
                        self.right_motor.setVelocity(u)
                        headingp = 1

                    elif abs(robot_pos[1] - 0.55) > 0.01:
                        u = self.pcontroll.update(
                            robot_pos[1] - 0.55, dt)
                        
                        if u > 0:
                            u += 3
                        elif u < 0: 
                            u -= 3

                        self.left_motor.setVelocity(u)
                        self.right_motor.setVelocity(u)
                        positionp = 1
                        if abs(robot_pos[1] - 0.55) < 0.1:
                            inplace = True
                        else: 
                            inplace = False
                else:
                    inplace = True
                    if l < 20:
                        self.left_motor.setVelocity(5)
                        self.right_motor.setVelocity(5)
                    else:
                        self.left_motor.setVelocity(-5)
                        self.right_motor.setVelocity(-5)

                t2 = time.time()
                dt = t2 - t1
                # time.sleep(dt - T)

                if inplace:
                    if abs(robot_pos[1] - 0.55) > 0.5:
                        inplace = False
                    inplace_tick += 1

                    if inplace_tick == 60:
                        headingp = 0
                        positionp = 0
                        l = 0
                        inplace = False

                l += 1
                if l == 40:
                    l = 0