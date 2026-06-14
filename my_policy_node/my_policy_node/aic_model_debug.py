import importlib
import inspect
import numpy as np
import rclpy
import threading

from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, TargetMode
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from my_policy_node.my_policy import MyPolicy

class AicModelDebug(Node):
    def __init__(self):
        super().__init__("aic_model_debug")
        self.get_logger().info("Try to load policy...")
        # 1. Load Policy (Change 'WaveArm' to your package name if different)
        # self.declare_parameter("policy", "my_policy")
        # policy_module_name = self.get_parameter("policy").get_parameter_value().string_value
        
        try:
            # policy_module = importlib.import_module(policy_module_name)
            # policy_module_classes = inspect.getmembers(policy_module, inspect.isclass)
            # expected_class = policy_module_name.split(".")[-1]
            # self._policy_class = next(cls for name, cls in policy_module_classes if name == expected_class)
            self._policy_class = MyPolicy
            self._policy = self._policy_class(self)
            self.get_logger().info("Debug Node Ready with Hardcoded Policy.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load policy: {e}")
            raise

        # 2. Setup Utilities
        self.bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._observation_msg = None
        self._depth_image = None
        self._policy_started = False
        self._depth_started = False

        # 3. ROS Communications
        self.observation_sub = self.create_subscription(
            Observation, "observations", self.observation_callback, 10
        )

        self.center_depth_sub = self.create_subscription(
            Image, "/center_camera/depth_image", self.center_depth_callback, 10
        )
        
        # Publishers (Dummy placeholders so your policy doesn't crash on move_robot)
        self.motion_update_pub = self.create_publisher(MotionUpdate, "/aic_controller/pose_commands", 2)
        self.joint_motion_update_pub = self.create_publisher(JointMotionUpdate, "/aic_controller/joint_commands", 2)

        # 4. Instantiate Policy Immediately
        self._policy = self._policy_class(self)
        self.get_logger().info("Debug Node Ready. Play your bag file to begin.")

    def observation_callback(self, msg):
        self._observation_msg = msg
        self.get_logger().info("Start observation callback")
        # Automatically trigger the policy once we have data
        if not self._policy_started:
            self._policy_started = True
            self.get_logger().info("First observation received! Starting Policy Thread...")
            threading.Thread(target=self.run_debug_policy).start()

    def center_depth_callback(self, msg):
        self._depth_image = msg
        self.get_logger().info("Start observation callback")
        if not self._depth_started:
            self._depth_started = True
        

    def run_debug_policy(self):
        dummy_task = Task()
        # No more relying on these strings for TF lookups!
        
        self.get_logger().info("Starting Vision-Based Policy...")
        
        # We loop internally now to see the depth map live
        while rclpy.ok():
            obs = self._observation_msg
            if obs is None:
                continue
                
            # Trigger your logic
            # Note: You'll need to update your insert_cable to not 'wait' for TFs
            success = self._policy.insert_cable(
                task=dummy_task,
                get_observation=obs, # Pass the actual data
                move_robot=self.move_robot,
                send_feedback=lambda fb: None
            )
            
            # For debugging vision, we might want to loop fast
            import time
            time.sleep(0.1) 


    # def run_debug_policy(self):
    #     """Mocks the Action Server execution"""
    #     # Create a dummy task object since we aren't getting one from an Action Goal
    #     dummy_task = Task()
    #     # dummy_task.target_module_name = "sample_module" 
    #     # dummy_task.port_name = "port_1"
        
    #     self.get_logger().info("Executing Policy.insert_cable()...")
        
    #     result = self._policy.insert_cable(
    #         task=dummy_task,
    #         get_observation=lambda: self._observation_msg,
    #         move_robot=self.move_robot,
    #         send_feedback=lambda fb: self.get_logger().info(f"FEEDBACK: {fb}")
    #     )
        
    #     self.get_logger().info(f"Policy Finished. Result: {result}")

    def move_robot(self, motion_update=None, joint_motion_update=None):
        # Just log and publish for debugging
        if motion_update:
            self.motion_update_pub.publish(motion_update)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = AicModelDebug()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
