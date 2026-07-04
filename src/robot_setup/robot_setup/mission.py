#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from custom_interfaces.action import ShootTarget

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # type: ignore

import math
import time
import threading

class Mission(Node):

    def __init__(self)->None:
        super().__init__("mission_call")

        self.move_to_next_target = False
        self.data_lock = threading.Lock()

        self.action_client = ActionClient(
            self,
            ShootTarget,
            'shooter_server'
        )
        
    def call_shooter(self)->None:
        goal_msg = ShootTarget.Goal()

        print("Waiting for Server...")
        self.action_client.wait_for_server()
        print("Sending Shoot Request...")

        self._send_goal_future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future)->None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            print("Shoot Request Rejected!!!")
            with self.data_lock: 
                self.move_to_next_target = True
            return

        print("Shoot Request Accepted...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            print(f"Target Shot: {result.result.shot}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            print("Could not find Target | Target Lost")

        with self.data_lock: self.move_to_next_target = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if feedback.target_found and feedback.target_locked:
            print(f"Target Found: {feedback.target_found} | Target Locked: {feedback.target_locked} | Distance: {feedback.distance}m")
        else:
            print(f"Target Found: {feedback.target_found} | Target Locked: {feedback.target_locked}")


def build_pose(navigator: BasicNavigator, x:float, y:float, heading_degrees: float)->PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    yaw = math.radians(heading_degrees)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw/2.0)
    pose.pose.orientation.w = math.cos(yaw/2.0)

    return pose     

def main(args=None)->None:
    rclpy.init(args=args)

    navigator = BasicNavigator()
    navigator.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    shooter_node = Mission()
    shooter_executor = SingleThreadedExecutor()
    shooter_executor.add_node(shooter_node)
    spin_thread = threading.Thread(target=shooter_executor.spin, daemon=True)

    try:
        spin_thread.start()

        print("[NAV2] Starting nav2...")
        navigator.waitUntilNav2Active()
        print("[NAV2] Nav2 Active")

        waypoints = [
            ("TARGET 1", build_pose(navigator,2.5,-2.1,120.0)),
            ("TARGET 2", build_pose(navigator,2.77,3.05,90.0)),
            ("TARGET 3", build_pose(navigator,5.3,-1.7,180.0))
        ]
        
        for station_name, target_pose in waypoints:
            print(f"\n[DISPATCH] Routing to:{station_name}")
            navigator.goToPose(target_pose)

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    dist = feedback.distance_remaining
                    eta = feedback.estimated_time_remaining.sec

                    print(f"Distance: {dist:.2f}m | ETA: {eta}s     ",end='\r')
            
            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                print(f"\n[SUCCESS] Arrived at {station_name}")

                shooter_node.call_shooter()
                while True:
                    with shooter_node.data_lock:
                        if shooter_node.move_to_next_target:
                            shooter_node.move_to_next_target = False
                            break
                    time.sleep(0.5)

            elif result == TaskResult.CANCELED:
                print(f"\n[CANCELLED] CANCELLED at {station_name}")
                print(f"\nRouting for next station...")
            elif result == TaskResult.FAILED:
                print(f"\n[FAILED] Navigation failed for {station_name}")
                print(f"\nRouting for next station...")

    except KeyboardInterrupt:
        pass
    finally:
        if navigator: navigator.destroy_node()
        if shooter_node: shooter_node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
