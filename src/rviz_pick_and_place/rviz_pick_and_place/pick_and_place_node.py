#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SingleArmPickAndPlace(Node):
    def __init__(self):
        super().__init__('single_arm_ik_node')
        self.get_logger().info("Single Arm MoveIt 2 IK Node Started!")
        
        # Publishers for scene updates and joint trajectories
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        
        # IK Service Client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
            
        time.sleep(2.0) # Wait for environment to stabilize
        self.run_project()

    def run_project(self):
        # 1. Environment me Target Box spawn karo
        self.get_logger().info("1. Spawning target box in front of Panda...")
        self.spawn_target_cube()
        time.sleep(2.0) 
        
        # 2. Pre-grasp position par jao (Joint Planning)
        self.get_logger().info("2. Moving to Pre-Grasp position...")
        self.move_arm([0.0, -0.4, 0.0, -2.0, 0.0, 1.5, 0.7])
        
        # 3. Box ke exact coordinate ke liye IK calculate karo (Cartesian Planning)
        self.get_logger().info("3. Computing IK to reach the box...")
        grasp_pose = Pose()
        # Coordinates thode paas aur standard reach me set karte hain
        grasp_pose.position.x = 0.35  
        grasp_pose.position.y = 0.0
        grasp_pose.position.z = 0.30  # Thoda niche taaki gripper touch kare

        # Grip orientation: 180-degree rotation around X-axis (facing down)
        grasp_pose.orientation.x = 1.0
        grasp_pose.orientation.y = 0.0
        grasp_pose.orientation.z = 0.0
        grasp_pose.orientation.w = 0.0

        calculated_joints = self.compute_inverse_kinematics(grasp_pose)
        
        if calculated_joints:
            self.get_logger().info("IK Solution found! Moving to target box...")
            self.move_arm(calculated_joints)
            
            # 4. Box ko attach karo (Grasp)
            self.get_logger().info("4. Grasping the box...")
            self.attach_target_cube()
            time.sleep(2.0)

            # 5. Box lekar nayi jagah jao
            self.get_logger().info("5. Moving to Place Position...")
            self.move_arm([0.5, 0.2, 0.2, -1.8, 0.0, 1.6, 0.7]) 

            # 6. Box ko chhod do (Release)
            self.get_logger().info("6. Releasing the box...")
            self.detach_target_cube(x=0.25, y=0.28, z=0.15)
        else:
            self.get_logger().error("IK Solution NOT found! Target position out of reach.")

    def compute_inverse_kinematics(self, target_pose):
        request = GetPositionIK.Request()
        req_info = PositionIKRequest()
        req_info.group_name = "panda_arm"
        req_info.pose_stamped.header.frame_id = "panda_link0"
        req_info.pose_stamped.pose = target_pose
        req_info.avoid_collisions = True 
        request.ik_request = req_info
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.error_code.val == 1: # 1 means SUCCESS
            return list(response.solution.joint_state.position[:7])
        return None

    def spawn_target_cube(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        cube = CollisionObject()
        cube.header.frame_id = "panda_link0"
        cube.id = "target_cube"
        box_shape = SolidPrimitive()
        box_shape.type = SolidPrimitive.BOX
        box_shape.dimensions = [0.05, 0.05, 0.05]
        cube_pose = Pose()
        cube_pose.position.x = 0.35
        cube_pose.position.y = 0.0
        cube_pose.position.z = 0.15
        cube_pose.orientation.w = 1.0
        cube.primitives.append(box_shape)
        cube.primitive_poses.append(cube_pose)
        cube.operation = CollisionObject.ADD
        scene_msg.world.collision_objects.append(cube)
        self.scene_pub.publish(scene_msg)

    def move_arm(self, joint_positions):
        traj = JointTrajectory()
        traj.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 3 
        traj.points.append(point)
        self.joint_pub.publish(traj)
        time.sleep(4.0)

    def attach_target_cube(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        attached_cube = AttachedCollisionObject()
        attached_cube.link_name = "panda_hand" 
        attached_cube.touch_links = ["panda_leftfinger", "panda_rightfinger", "panda_hand"]
        cube = CollisionObject()
        cube.header.frame_id = "panda_hand"
        cube.id = "target_cube"
        box_shape = SolidPrimitive()
        box_shape.type = SolidPrimitive.BOX
        box_shape.dimensions = [0.05, 0.05, 0.05]
        cube_pose = Pose()
        cube_pose.position.z = 0.06
        cube_pose.orientation.w = 1.0
        cube.primitives.append(box_shape)
        cube.primitive_poses.append(cube_pose)
        cube.operation = CollisionObject.ADD
        attached_cube.object = cube
        scene_msg.robot_state.attached_collision_objects.append(attached_cube)
        scene_msg.robot_state.is_diff = True
        self.scene_pub.publish(scene_msg)

    def detach_target_cube(self, x, y, z):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        detach_obj = AttachedCollisionObject()
        detach_obj.link_name = "panda_hand"
        detach_obj.object.id = "target_cube"
        detach_obj.object.operation = CollisionObject.REMOVE
        scene_msg.robot_state.attached_collision_objects.append(detach_obj)
# Part B: Usi box ko new Side coordinates par ADD karo (Place effect!)
        cube = CollisionObject()
        cube.header.frame_id = "panda_link0"
        cube.id = "target_cube"
        box_shape = SolidPrimitive()
        box_shape.type = SolidPrimitive.BOX
        box_shape.dimensions = [0.05, 0.05, 0.05]
        cube_pose = Pose()
        cube_pose.position.x = x
        cube_pose.position.y = y
        cube_pose.position.z = z
        cube_pose.orientation.w = 1.0
        cube.primitives.append(box_shape)
        cube.primitive_poses.append(cube_pose)
        cube.operation = CollisionObject.ADD
        scene_msg.world.collision_objects.append(cube)

        self.scene_pub.publish(scene_msg)
        time.sleep(1.0)
def main(args=None):
    rclpy.init(args=args)
    node = SingleArmPickAndPlace()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
