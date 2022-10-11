#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import tf
import math as m
import numpy as np
from enum import Enum
from time import sleep
# msgs
from std_msgs.msg import Float32, Int16, UInt8
from geometry_msgs.msg import PoseStamped
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from lidar_camera_calibration.msg import Signmsg
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import PoseStamped

rospy.init_node("state_machine")

try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")

    from stanley import Stanley
    from state import State, OdomState
    from path_selector import PathSelector, PathType, MissionState
    from pid_tuner import PID

except Exception as ex:
    rospy.logfatal(ex)
    rospy.logfatal("Import Error : State Machine - erp42_control")

try:
    sys.path.append(rospkg.RosPack().get_path("parking") + "/src")
    from horizontal_parking2 import HorizontalParking, HorParkingState
    
except Exception as ex:
    rospy.logfatal(ex)
    rospy.logfatal("Import Error : State Machine - parking")

try:
    sys.path.append(rospkg.RosPack().get_path("mission") + "/src")
    from static_bs import Obstacle
    from delivery import Delivery
    from traffic import Traffic

except Exception as ex:
    rospy.logfatal(ex)
    rospy.logfatal("Import Error : State Machine - mission")


max_steer = rospy.get_param("/state_machine/max_steer", 30.0)  # DEG

def wait_for_stop(duration):
    global current_time, last_time, r, cmd_pub
    
    msg = ControlMessage(0, 0, 2, 0, 0, 80, 0)

    dt = 0
    last_time = rospy.Time.now()
    while dt < duration:
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()

        if dt > duration:
            last_time = current_time

        cmd_pub.publish(msg)
        r.sleep()


class StateMachine(object):
    def __init__(self, state):
        """
            Important Objects for Driving
        """

        # Car State Object. Subscribe odometry and update [x, y, v, theta]
        self.state = state
        self.pid = PID()
        # Stanley Controller Object
        self.stanley = Stanley()
        self.stanley.setCGain(0.05)

        self.target_idx = 0

        # Path Selector. Use path length and stanley's target idx, request path to DB. And then, update self.path
        self.selector = PathSelector(self.state)
        self.path_response = rospy.Subscriber(
            "/path_response", PathResponse, callback=self.path_callback)
        self.path = PathResponse()

        # Traffic Sign
        self.traffic = Traffic()

        # Delivery
        self.delivery = Delivery()
        self.panel_id = None
        self.panel = None
        self.dis = None
    
        # Parking
        self.horizontal_parking = HorizontalParking(
            state=self.state, stanley=self.stanley, cmd_pub=cmd_pub)

        # Static
        self.static = Obstacle(state=self.state)

        # right
        self.right = False

        """
            Fields
        """

        # Mission State for state control
        self.mission_state = MissionState.DRIVING
        # Time to prevent request next math without delay
        self.request_time = rospy.Time.now()

        sleep(5.)   # delay

        # Start!
        self.selector.makeRequest()

        self.mission_state = self.selector.path.mission_type

    def path_callback(self, msg):
        # When path response is accepted, reset target idx and update path
        self.target_idx = 0
        self.path = msg

    def switchPath(self):
        # When current path is almost end
        if len(self.path.cx) - 20 < self.target_idx:

            rospy.logwarn("Try to change path...!")

            current_time = rospy.Time.now()
            dt = (current_time - self.request_time).to_sec()

            # prevent duplicated request
            if dt > 3:
                # not end
                if self.selector.path.next is not None:
                    self.request_time = rospy.Time.now()
                    self.selector.goNext()
                    self.selector.makeRequest()

                    # next path is end
                    if self.selector.path.next is None:
                        self.mission_state = MissionState.END
                        rospy.loginfo("Missin State : End")

                    # currnet path is not end
                    else:
                        self.mission_state = self.selector.path.mission_type
                        rospy.loginfo("Mission State : %s" %
                                      str(self.mission_state))

                self.target_idx = 0

    def mainControl(self, desired_speed=0):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)

            self.target_idx = target_idx

            desired_speed = self.selector.path.desired_speed if desired_speed == 0 else desired_speed
            di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

            speed = self.pid.PIDControl(desired_value=desired_speed)
            # speed = desired_speed * 1.5
            # TODO: REPARI
            rospy.logwarn(str(speed))
                        
            return ControlMessage(0, 0, 2, int(speed), int(m.degrees(-di)), 0, 0)

        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)


    def trafficControl(self):
        desired_speed = self.selector.path.desired_speed

        if len(self.path.cx) - 80 < self.target_idx:
            # Almost in front of traffic sign
            desired_speed *= 0.5
            self.traffic.main()
            # rospy.logfatal(str(self.traffic.msg.straight))

            if len(self.path.cx) - 30 < self.target_idx:
                # Stop if required
                try:
                    if self.selector.path.path_type == PathType.STRAIGHT:
                        if self.traffic.msg.straight == 0:
                            raise Exception()

                    elif self.selector.path.path_type == PathType.LEFT or self.selector.path.path_type == PathType.UTURN:
                        if self.traffic.msg.left == 0:
                            raise Exception()

                    else:
                        rospy.logfatal("Invalid Path Type!")
                        raise Exception()

                except Exception:
                    # Stop
                    return ControlMessage(0, 0, 2, 0, 0, 150, 0)

        msg = self.mainControl(desired_speed=desired_speed)

        return msg
    
    # have to change after school bs test
    def deliveryControl(self):
        desired_speed = self.selector.path.desired_speed
        
        if self.mission_state == MissionState.DELIVERY_A:
            self.delivery.delivery_A()
            self.panel = self.delivery.panel_A
            self.panel_id = self.delivery.panel_id
            
        elif self.mission_state == MissionState.DELIVERY_B:
            if self.panel_id == None:
                self.panel_id = [10,0,1000]
            
            self.delivery.delivery_B(self.panel_id)
            self.panel = self.delivery.panel_B
            
        # between delivery sign and erp42 distance
        if self.panel != None:
            self.dis = np.hypot([self.state.x - self.panel.x],
                            [self.state.y - self.panel.y])

        if self.dis != None and self.dis < 15:
            # Move to panel
            if self.delivery.is_delivery_path is True:
                desired_speed *= 0.5
                msg = self.mainControl(desired_speed=desired_speed)
                
                if self.delivery.delivery == False:
                    dists = []

                    for i in range(len(self.path.cx)):
                        dist = np.hypot(
                            self.path.cx[i] - self.panel.x, self.path.cy[i] - self.panel.y)
                        dists.append(dist)

                    dists = np.array(dists)
                    self.delivery.target_idx = np.argmin(dists)

                
                if self.target_idx >= self.delivery.target_idx - 5:
                    
                    wait_for_stop(5)
                    
                    self.delivery.is_delivery_path = False
                    self.request_time = rospy.Time.now()
                    self.selector.goNext()
                    self.selector.makeRequest()
                    self.mission_state = self.selector.path.mission_type
                    
                    self.panel = None
                    self.dis = None

            else:
                self.request_time = rospy.Time.now()
                self.selector.goNext()
                self.selector.makeRequest()
                
                self.delivery.is_delivery_path = True

                msg = self.mainControl(desired_speed=desired_speed)

        else:
            # Keep tracking main path
            msg = self.mainControl(desired_speed=desired_speed)

        return msg


    def staticControl(self):
        desired_speed = self.selector.path.desired_speed

        self.static.main()
        
        if self.target_idx > len(self.path.cx) - 60:
            msg = self.trafficControl()
        
        else:
            if self.static.obs_state == True:
                msg = self.static.msg
            else:
                msg = self.mainControl(desired_speed=desired_speed)
        
        return msg


    def horizontalParkingControl(self):
        desired_speed = self.selector.path.desired_speed

        if self.horizontal_parking.search_path is None:
            self.horizontal_parking.setSearchPath(self.path)

        msg = self.horizontal_parking.control()
        
        if self.horizontal_parking.parking_state == HorParkingState.DONE:
            msg = self.mainControl(desired_speed=desired_speed)
            
        return msg
    
    def rightControl(self):
        desired_speed = self.selector.path.desired_speed
        
        if self.right == False and len(self.path.cx) - 25 < self.target_idx:
            wait_for_stop(3)
            
            self.right = True
            
        return self.mainControl(desired_speed=desired_speed)   
    

    def endControl(self):
        if len(self.path.cx) - 5 < self.target_idx:
            return ControlMessage(0, 0, 2, 0, 0, 0, 0)

        return self.mainControl()

    def makeControlMessage(self):
        # Exception
        if len(self.path.cx) == 0:
            rospy.logwarn("No Path Data...")
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        # Try Update Path
        self.switchPath()

        # Switch
        if self.mission_state == MissionState.DRIVING:
            msg = self.mainControl()

        elif self.mission_state == MissionState.TRAFFIC:
            msg = self.trafficControl()

        elif self.mission_state == MissionState.DELIVERY_A or self.mission_state == MissionState.DELIVERY_B:
            msg = self.deliveryControl()

        elif self.mission_state == MissionState.STATIC:
            msg = self.staticControl()

        elif self.mission_state == MissionState.PARKING:
            msg = self.horizontalParkingControl()

        elif self.mission_state == MissionState.RIGHT:
            self.right = False
            msg = self.rightControl()

        elif self.mission_state == MissionState.END:
            msg = self.endControl()

        else:
            rospy.logfatal("Invalide Mission State!")
            msg = ControlMessage(0, 0, 2, 3, 0, 0, 0)

        return msg


if __name__ == "__main__":
    # state = State(odometry_topic="/odometry/kalman", test=True)
    state = OdomState(odometry_topic="/odometry/kalman", hz=30, test=False)

    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)
    
    mission_state_pub = rospy.Publisher("/mission_state", UInt8, queue_size=1) 

    last_time = rospy.Time.now()
    current_time = rospy.Time.now()

    controller = StateMachine(state=state)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        mission_state_pub.publish(int(controller.mission_state))
        
        if cmd_pub.get_num_connections() > 0 or True:
            msg = controller.makeControlMessage()
            cmd_pub.publish(msg)
        
        r.sleep()
