#!/usr/bin/env python

import sys
import os
from parking.src.reeds_shepp_path_planning import Path
import rospy
import rospkg
import tf
import math as m
import numpy as np
from enum import Enum
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from lidar_camera_calibration.msg import Signmsg
from std_msgs.msg import Float32, Int16
from time import sleep
from geometry_msgs.msg import PoseStamped
from mission.msg import obTF
# parking, static not done 
try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")
    from speed_supporter import SpeedSupporter
    from stanley import Stanley
    from state import State, OdomState
    from path_selector import PathSelector, PathType
    
    sys.path.append(rospkg.RosPack().get_path("mission") + "/src")
    from parking_final import Parking
    from obstacle_final import obstacle

except Exception as ex:
    rospy.logfatal(ex)
    rospy.logfatal("Import Error : State Machine")


max_steer = rospy.get_param("/state_machine/max_steer", 25.0)  # DEG
speed_control_enable = rospy.get_param(
    "/state_machine/speed_control_enable", True)


class MissionState(Enum):
    DRIVING = 0
    TRAFFIC = 1
    DELIVERY = 2
    STATIC = 3
    DYNAMIC = 4
    PARKING = 5
    END = 9


class StateMachine(object):
    def __init__(self, state):
        """
            Important Objects for Driving
        """

        # Car State Object. Subscribe odometry and update [x, y, v, theta]
        self.state = state

        # Stanley Controller Object
        self.stanley = Stanley()
        self.target_idx = 0

        # Speed Supporter Object. Accelerate in straight track, Decelerate in corner track
        self.supporter = SpeedSupporter()

        # Path Selector. Use path length and stanley's target idx, request path to DB. And then, update self.path
        self.selector = PathSelector(self.state)
        self.path_response = rospy.Subscriber(
            "/path_response", PathResponse, callback=self.path_callback)
        self.path = PathResponse()

        # Traffic Sign
        self.trafficSub = rospy.Subscriber(
            "/sign_publish", Signmsg, callback=self.trafficCallback
        )
        self.trafficSign = Signmsg()

        # Delivery
        self.deliverySub = rospy.Subscriber('/delivery_AB', PoseStamped, callback=self.deliveryCallback)
        self.deliverySign = PoseStamped()
        
        # Dynamic
        self.dynamicSub = rospy.Subscriber('/ob_TF', obTF, callback=self.dynamicCallback)
        self.dynamicSign = obTF()
        
        # Parking
        self.parking = Parking()
        
        # Static
        self.static = obstacle()
        
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

        # currnet path is not end
        if self.selector.path.end.is_end is True:
            # next path's end point may have traffic sign
            self.mission_state = MissionState.TRAFFIC

        else:
            # Ignore traffic sign
            self.mission_state = MissionState.DRIVING

    def path_callback(self, msg):
        # When path response is accepted, reset target idx and update path
        # Upcasting to PathResponseWithType
        self.target_idx = 0
        self.path = msg
        # self.path = PathResponseWithType(msg.start, msg.end,
        #                                  msg.path_id, list(msg.cx), list(msg.cy), list(msg.cyaw))

    def trafficCallback(self, msg):
        self.trafficSign = msg

    def deliveryCallback(self, msg):
        self.deliverySign = msg
        
        self.sign_x = self.deliverySign.pose.position.x
        self.sign_y = self.deliverySign.pose.position.y
        
    def dynamicCallback(self, msg):
        self.dynamicSign = msg

    def switchPath(self):
        # When current path is almost end
        if len(self.path.cx) * 0.95 < self.target_idx:

            rospy.loginfo("Try to change path...!")

            current_time = rospy.Time.now()
            dt = (current_time - self.request_time).to_sec()

            # prevent duplicated request
            if dt > 1:
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
                    elif self.selector.path.end.is_end is True:

                        if self.path.path_id == "delivery_A" or self.path.path_id == "delivery_B":
                            self.mission_state = MissionState.DELIVERY
                            rospy.loginfo("Missin State : Delivery")
                        
                        elif self.path.path_id == "static":
                            self.mission_state = MissionState.STATIC
                            rospy.loginfo("Missin State : Static")
    
                        else:
                            # next path's end point may have traffic sign
                            self.mission_state = MissionState.TRAFFIC
                            rospy.loginfo("Missin State : Traffic")
                            
                    else:
                        if self.path.path_id == "parking":
                            self.mission_state = MissionState.PARKING
                            rospy.loginfo("Missin State : Parking")
                        
                        elif self.path.path_id == "static":
                            self.mission_state = MissionState.STATIC
                            rospy.loginfo("Missin State : Static")
                        
                        elif self.path.path_id == "dynamic":
                            self.mission_state = MissionState.DYNAMIC
                            rospy.loginfo("Missin State : Dynamic")
                        
                        else:                            
                            # Ignore traffic sign
                            self.mission_state = MissionState.DRIVING
                            rospy.loginfo("Missin State : Driving")
        
                self.target_idx = 0
    
    def drivingControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        msg = ControlMessage()

        desired_speed = self.selector.path.desired_speed

        if speed_control_enable is True:
            speed, brake = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                                  desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)
        else:
            speed = desired_speed
            brake = 0

        msg.Speed = int(speed)
        msg.Steer = m.degrees(-di)
        msg.Gear = 2
        msg.brake = brake

        return msg

    def trafficControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        desired_speed = self.selector.path.desired_speed

        if len(self.path.cx) * 0.8 < self.target_idx:
            # Almost in front of traffic sign
            desired_speed *= 0.5

            if self.selector.path.next.path_type == PathType.STRAIGHT:
                if self.trafficSign.straight == 1:
                    # Non-Stop
                    rospy.loginfo("STRAIGHT : GO!!")
                    return ControlMessage(0, 0, 2, int(desired_speed), m.degrees(-di), 0, 0)
                else:
                    # Stop
                    rospy.loginfo("STRAIGHT : STOP!!")
                    return ControlMessage(0, 0, 2, 0, 0, 100, 0)

            elif self.selector.path.next.path_type == PathType.RIGHT:
                """
                    TO DO : Add right turn logic
                """
                rospy.logwarn("Path type : RIGHT")
                return ControlMessage(0, 0, 2, int(desired_speed), m.degrees(-di), 0, 0)

            elif self.selector.path.next.path_type == PathType.LEFT:
                if self.trafficSign.left == 1:
                    rospy.loginfo("LEFT : GO!!")
                    return ControlMessage(0, 0, 2, int(desired_speed), m.degrees(-di), 0, 0)
                else:
                    # Stop
                    rospy.loginfo("LEFT : STOP!!")
                    return ControlMessage(0, 0, 2, 0, 0, 100, 0)

            elif self.selector.path.next.path_type == PathType.NONE:
                # Nonetype
                rospy.logfatal("Path Type is None!!")
                return ControlMessage(0, 0, 2, 0, 0, 0, 0)

            elif self.selector.path.nex.path_type == PathType.UTURN:
                if self.trafficSign.left == 1 and self.trafficSign.straight == 0:
                    rospy.loginfo("UTURN : GO!!")
                    return ControlMessage(0, 0, 2, int(desired_speed), m.degrees(-di), 0, 0)
                else:
                    rospy.loginfo("UTURN : STOP!!")
                    return ControlMessage(0, 0, 2, 0, 0, 100, 0)
            else:
                rospy.logfatal("What the fuck")
                print(self.selector.path.next.path_type)
                return ControlMessage(0, 0, 2, 0, 0, 0, 0)

        else:
            # Far away from traffic sign
            speed, brake = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                                  desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)

            msg = ControlMessage()
            msg.Speed = int(speed)
            msg.Steer = m.degrees(-di)
            msg.Gear = 2
            msg.brake = brake

            return msg
        
    def deliveryControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        self.target_idx = target_idx
    
        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        desired_speed = self.selector.path.desired_speed
        
        dis = np.hypot([self.state.x-self.sign_x], [self.state.y - self.sign_y])
        
        if dis < 10 : # have to change tolerance
            rospy.loginfo("traking local path")
            # switch path 
            if self.selector.path.start == 'A1':  #have to change path start point (before delivery missionn path)   
                self.selector.goNext()
                self.selector.makequest()
                di, target_idx = self.stanley.stanley_control(
                    self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)

                self.target_idx = target_idx
            
                di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

                desired_speed = self.selector.path.desired_speed
                           
            if dis < 0.1:
                # delivery mission end
                rospy.loginfo("arrive at delivery sign")
                
                self.selector.goNext()
                self.selector.makequest()
                
                if self.selector.path.end.is_end is True:
                    self.mission_state = MissionState.TRAFFIC
                else:
                    self.mission_state == MissionState.DRIVING
                    
                return ControlMessage(0, 0, 2, 0, 0, 200, 0)
        
        speed, brake = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                                desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)
        msg = ControlMessage()
        msg.Speed = int(speed)
        msg.Steer = m.degrees(-di)
        msg.Gear = 2
        msg.brake = brake
        
        return msg
    
    def staticControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)    
        self.target_idx = target_idx
        
        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        desired_speed = self.selector.path.desired_speed
        
        if len(self.path.cx) - 150 < self.target_idx:
            if self.selector.path.end.is_end is True:
                self.mission_state = MissionState.TRAFFIC
            else:
                self.mission_state == MissionState.DRIVING
                
        return self.static.msg
    
    def dynamicControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)            
        
        self.target_idx = target_idx
        
        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        desired_speed = self.selector.path.desired_speed
        
        if self.dynamicSign.front_left == 1 or self.dynamicSign.front_right == 1:
            rospy.loginfo("obstacle exist : STOP!")
            return ControlMessage(0, 0, 2, 0, 0, 200, 0)
        
        else:
            rospy.loginfo("obstacle dose not exist : GO!")
            speed, brake = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                                  desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)
            msg = ControlMessage()
            msg.Speed = int(speed)
            msg.Steer = m.degrees(-di)
            msg.Gear = 2
            msg.brake = brake
            
            return msg    
    
    def parkingControl(self):
        try:
            di, target_idx = self.stanley.stanley_control(
                self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
        except IndexError as ie:
            rospy.logwarn(ie)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)
        except Exception as ex:
            rospy.logfatal(ex)
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)    
        self.target_idx = target_idx
        
        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        desired_speed = self.selector.path.desired_speed   
        
        if not self.parking.parking_state == 'complete':
            rospy.loginfo("parking")
            return self.parking.msg
        
        else:
            rospy.loginfo("parking complete!")
            speed, brake = self.supporter.control(current_value=self.state.v * 3.6,   # m/s to kph
                                                  desired_value=desired_speed, max_value=int(desired_speed + 2), min_value=5)
            msg = ControlMessage()
            msg.Speed = int(speed)
            msg.Steer = m.degrees(-di)
            msg.Gear = 2
            msg.brake = brake

            self.mission_state == MissionState.DRIVING
            
            return msg    
        
    def endControl(self):
        if len(self.path.cx) * 0.9 < self.target_idx:
            return ControlMessage(0, 0, 2, 0, 0, 0, 0)

        return self.drivingControl()

    def makeControlMessage(self):
        # Exception
        if len(self.path.cx) == 0:
            rospy.logwarn("No Path Data...")
            return ControlMessage(0, 0, 2, 3, 0, 0, 0)

        # Try Update Path
        self.switchPath()

        # Switch
        if self.mission_state == MissionState.DRIVING:
            msg = self.drivingControl()

        elif self.mission_state == MissionState.TRAFFIC:
            msg = self.trafficControl()

        elif self.mission_state == MissionState.DELIVERY:
            msg = self.deliveryControl()

        elif self.mission_state == MissionState.STATIC:
            msg = self.staticControl()

        elif self.mission_state == MissionState.DYNAMIC:
            msg = self.dynamicControl()

        elif self.mission_state == MissionState.PARKING:
            msg= self.parkingControl

        elif self.mission_state == MissionState.END:
            msg = self.endControl()

        else:
            rospy.logfatal("Invalide Mission State!")
            msg = ControlMessage(0, 0, 2, 3, 0, 0, 0)

        return msg


if __name__ == "__main__":
    rospy.init_node("stanley_controller")

    # state = State(odometry_topic="/odometry/kalman")
    state = OdomState(odometry_topic="/odometry/kalman")

    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    controller = StateMachine(state=state)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if cmd_pub.get_num_connections() > 0 or True:
            msg = controller.makeControlMessage()
            cmd_pub.publish(msg)
            print(controller.mission_state)
            print(msg)
        r.sleep()
