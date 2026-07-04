#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import time
import math
import tempfile
import subprocess

from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from custom_interfaces.action import ShootTarget
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

import cv2 as cv
import numpy as np
from numpy.typing import NDArray
from cv_bridge import CvBridge

class TargetPerceptor(Node):

    def __init__(self)->None:
        super().__init__("target_shooter")

        self.finding_target:bool = False

        self.prev_pan_error:int = 0
        self.prev_tilt_error:int = 0
        self.total_pan_error:int = 0
        self.total_tilt_error:int = 0
        self.kp_pan:float = 0.0002
        self.ki_pan:float = 0.0000007
        self.kd_pan:float = 0.00055
        self.kp_tilt:float = 0.0002
        self.ki_tilt:float = 0.0000007
        self.kd_tilt:float = 0.00055

        self.shooter_pan:float = 0.0
        self.shooter_tilt:float = 0.0
        self.shooter_tries = 3
        
        self.latest_image : NDArray = None
        self.cv_image_bridge = CvBridge()

        self.latest_scan = LaserScan()

        self.target_x:int = 0
        self.target_y:int = 0

        self.robot_x:float = 0.0
        self.robot_y:float = 0.0
        self.robot_yaw:float = 0.0

        self.data_lock = threading.Lock()

        pose_group = MutuallyExclusiveCallbackGroup()
        sensor_group = MutuallyExclusiveCallbackGroup()
        action_group = MutuallyExclusiveCallbackGroup()

        self.camera_feed = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            qos_profile_sensor_data,
            callback_group = sensor_group
        )

        self.laser_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.store_laser_data,
            qos_profile_sensor_data,
            callback_group = sensor_group
        )

        self.shoot = ActionServer(
            self,
            ShootTarget,
            "shooter_server",
            self.shoot_down,
            callback_group = action_group
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
            callback_group = action_group
        )

        self.tilt_publisher = self.create_publisher(
            Float64,
            "/turret/tilt_cmd",
            10,
            callback_group = action_group
        )

        self.pan_publisher = self.create_publisher(
            Float64,
            "/turret/pan_cmd",
            10,
            callback_group = action_group
        )

        self.pose_subscriber = self.create_subscription(
            Pose,
            "/model/my_robot/pose",
            self.get_robot_coords,
            10,
            callback_group = pose_group
        )



    def process_image(self, image:Image)->None:
        with self.data_lock:
            self.latest_image = self.cv_image_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

            if self.finding_target and self.target_x != -1:
                cv.circle(self.latest_image, (self.target_x,self.target_y), 5, (0,0,255), -1)

            cv.imshow("Robot Camera Feed", self.latest_image)
            cv.waitKey(1)

    def store_laser_data(self, scan:LaserScan)->None:
        with self.data_lock: self.latest_scan = scan

    def get_robot_coords(self,msg:Pose)->None:
        with self.data_lock:
            self.robot_x = msg.position.x
            self.robot_y = msg.position.y

            q = msg.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_pan_and_tilt(self)->None:
        pan = Float64()
        tilt = Float64()

        pan.data = self.shooter_pan
        tilt.data = self.shooter_tilt

        self.pan_publisher.publish(pan)
        self.tilt_publisher.publish(tilt)


    def find_target(self) -> tuple[int,int]:
        pixel_x = pixel_y = -1

        with self.data_lock: hsv_image = cv.cvtColor(self.latest_image, cv.COLOR_BGR2HSV)
        lower_blue = np.array([85,80,50])
        upper_blue = np.array([140,255,255])

        masked_image = cv.inRange(hsv_image,lower_blue,upper_blue)

        contours,_ = cv.findContours(masked_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            target = max(contours, key = cv.contourArea)
            M = cv.moments(target)

            if M['m00'] != 0:
                pixel_x = int(M['m10']/M['m00'])
                pixel_y = int(M['m01']/M['m00'])

        with self.data_lock: self.target_x, self.target_y = pixel_x, pixel_y

        return pixel_x,pixel_y


    def shoot_down(self, goal_handle) -> ShootTarget.Result:
        sweep = 0
        rotate_time = time.time()
        rotate_interval = 0.1

        self.finding_target = True
        self.prev_pan_error = 0
        self.prev_tilt_error = 0
        self.total_pan_error = 0
        self.total_tilt_error = 0
        self.shooter_tries = 5

        response = ShootTarget.Result()
        feedback_msg = ShootTarget.Feedback()

        pixel_x, pixel_y = self.find_target()
        out_of_sight = (pixel_x == -1)

        with self.data_lock: img_height, img_width = self.latest_image.shape[:2]

        self.pub_feedback(goal_handle,feedback_msg,False,False,-1.0)

        while out_of_sight:
            current_time = time.time()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                response.shot = False
                return response

            if sweep == 0:
                if current_time - rotate_time >= rotate_interval:
                    self.shooter_pan += 0.1
                    self.publish_pan_and_tilt()
                    rotate_time = current_time

                if(self.shooter_pan >= 0.7): 
                    self.shooter_pan = 0.7
                    self.publish_pan_and_tilt()
                    sweep = 1
            elif sweep == 1:
                if current_time - rotate_time >= rotate_interval:
                    self.shooter_pan -= 0.1
                    self.publish_pan_and_tilt()
                    rotate_time = current_time

                if(self.shooter_pan <= -0.7):
                    self.shooter_pan = -0.7
                    self.publish_pan_and_tilt()
                    sweep = 2
            elif sweep == 2:
                if current_time - rotate_time >= rotate_interval:
                    self.shooter_pan += 0.1
                    self.publish_pan_and_tilt()
                    rotate_time = current_time

                if abs(self.shooter_pan) <= 0.001:
                    sweep = 3
                    rotate_time = current_time
                    self.shooter_pan = 0.0
                    self.publish_pan_and_tilt()
            elif sweep == 3:
                self.create_velocity_msg(2.0)

                if current_time - rotate_time >= 3:
                    self.create_velocity_msg(0.0)

                    goal_handle.abort()
                    response.shot = False
                    return response

            pixel_x, pixel_y = self.find_target()
            out_of_sight = (pixel_x == -1)

            time.sleep(0.03)

        self.pub_feedback(goal_handle,feedback_msg,True,False,-1.0)
        self.create_velocity_msg(0.0)

        while (abs(pixel_x - img_width/2) > 5) or (abs(pixel_y - (img_height/2)) > 5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                response.shot = False
                return response
            
            pixel_x, pixel_y = self.find_target()

            if pixel_x == -1:
                goal_handle.abort()
                response.shot = False
                return response

            self.pid_pan_tilt(pixel_x, img_width/2, pixel_y, (img_height/2))

            time.sleep(0.03)

        self.pub_feedback(goal_handle,feedback_msg,True,True,self.get_target_distance())

        before_shot_x, before_shot_y = pixel_x, pixel_y
        target_shot = False

        while (not target_shot) and self.shooter_tries != 0:
            self.fire_projectile()
            self.shooter_tries -= 1

            time.sleep(2)
            pixel_x, pixel_y = self.find_target()
            target_shot = (abs(pixel_x-before_shot_x)+abs(pixel_y-before_shot_y))>20 or pixel_x == -1

        self.shooter_pan = self.shooter_tilt = 0.0
        self.publish_pan_and_tilt()
        self.create_velocity_msg(0.0)

        self.finding_target = False

        goal_handle.succeed()
        response.shot = target_shot
        return response
    
    def pid_pan_tilt(self, current_x, target_x, current_y, target_y):
        error = target_x - current_x
        self.total_pan_error += error
        derivative = error - self.prev_pan_error
        self.prev_pan_error = error

        pan_change = (self.kp_pan * error + self.ki_pan * self.total_pan_error + self.kd_pan * derivative)
        self.shooter_pan += pan_change
        self.shooter_pan = max(min(self.shooter_pan, 1.57), -1.57)
        
        error = target_y - current_y
        self.total_tilt_error += error
        derivative = error - self.prev_tilt_error
        self.prev_tilt_error = error

        tilt_change = (self.kp_tilt * error + self.ki_tilt * self.total_tilt_error + self.kd_tilt * derivative)
        self.shooter_tilt += tilt_change
        self.shooter_tilt = max(min(self.shooter_tilt, 0.785), -0.200)

        self.publish_pan_and_tilt()
    
    def create_velocity_msg(self, angular_vel:float) -> None:
        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_vel

        self.velocity_publisher.publish(msg)

    def get_target_distance(self) -> float:
        with self.data_lock: data = self.latest_scan
        if data is None: return -1.0

        distance = 100.0
        valid = False

        index = int((self.shooter_pan+math.pi) * 180.0/math.pi)
        begin = index-2
        end = index+2

        for i in range(begin, end+1):
            safe_index = i % 360;
            d = data.ranges[safe_index]

            if 0.12 < d < 10.0: 
                distance = min(distance,d)
                valid = True
            
        return distance if valid else -1.0
    
    def fire_projectile(self) -> None:
        with self.data_lock:
            rx = self.robot_x
            ry = self.robot_y
            ryaw = self.robot_yaw
            pan = self.shooter_pan
            tilt = self.shooter_tilt

        print(rx,ry,ryaw)

        turrent_base_x = rx + (0.1 * math.cos(ryaw))
        turrent_base_y = ry + (0.1 * math.sin(ryaw))
        turrent_base_z = 0.22

        raw_yaw = ryaw+pan
        yaw = math.atan2(math.sin(raw_yaw), math.cos(raw_yaw))

        bullet_x = turrent_base_x + (0.25 * math.cos(tilt) * math.cos(yaw))
        bullet_y = turrent_base_y + (0.25 * math.cos(tilt) * math.sin(yaw))
        bullet_z = turrent_base_z + (0.25 * math.sin(tilt))

        print(rx,ry,ryaw,bullet_x,bullet_y)

        speed = 15.0
        vx = speed * math.cos(tilt) * math.cos(yaw)
        vy = speed * math.cos(tilt) * math.sin(yaw)
        vz = speed * math.sin(tilt)

        bullet_sdf = f"""<?xml version="1.0"?>
        <sdf version="1.8">
          <model name="bullet_{int(time.time()*1000)}">
            <link name="link">
              <visual name="vis">
                <geometry><sphere><radius>0.02</radius></sphere></geometry>
                <material><ambient>1 0 0 1</ambient><emission>1 0 0 1</emission></material>
              </visual>
              <collision name="col">
                <geometry><sphere><radius>0.02</radius></sphere></geometry>
              </collision>
              <inertial>
                <mass>0.5</mass>
                <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
              </inertial>
            </link>
            <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
                <initial_linear>{vx} {vy} {vz}</initial_linear>
            </plugin>
          </model>
        </sdf>
        """

        with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as temp:
            temp.write(bullet_sdf)
            temp_path = temp.name

        model_name = f"bullet_{int(time.time()*1000)}"
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-file", temp_path,
            "-name", model_name,
            "-x", str(bullet_x),
            "-y", str(bullet_y),
            "-z", str(bullet_z),
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0"
        ]
        
        subprocess.Popen(spawn_cmd)

    def pub_feedback(self, goal_handle, feedback_msg:ShootTarget.Feedback, target_found:bool, target_locked:bool, distance:float):
        feedback_msg.target_found = target_found
        feedback_msg.target_locked = target_locked
        feedback_msg.distance = distance

        goal_handle.publish_feedback(feedback_msg)


def main(args=None)->None:
    rclpy.init(args=args)

    node=None
    try:
        node = TargetPerceptor()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor: executor.shutdown()
        if node: node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()