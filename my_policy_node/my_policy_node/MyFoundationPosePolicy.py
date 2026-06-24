#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#


import numpy as np

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from transforms3d.quaternions import quat2mat
from scipy.spatial.transform import Rotation as R



import sys
import os
import gc
os.environ["PYTORCH_ALLOC_CONF"] = "expandable_segments:True"

import argparse
import trimesh
from typing import Callable, Protocol
from sensor_msgs.msg import Image

import torch
import nvdiffrast.torch as dr

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


QuaternionTuple = tuple[float, float, float, float]
# GetDepthCallback = Callable[[], Image]

print("Finish importing!")

class MyFoundationPosePolicy(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("Start initializing...")
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0
        self._max_integrator_windup = 0.05
        self._task = None
        self.bridge = CvBridge()
        self.port_transform = None
        

        self.get_logger().info("Start importing and initializing FoundationPose...")

        home_dir = os.path.expanduser("~")
        foundation_pose_path = os.path.join(home_dir, "ws_aic", "src", "aic", "FoundationPose")
        
        if foundation_pose_path not in sys.path:
            sys.path.insert(0, foundation_pose_path)

        import estimater
        import datareader
        

        # Define paths manually
        mesh_file = "/home/ubuntu/ws_aic/src/aic/aic_assets/models/NIC Card/nic_card_visual.glb"
        self.est_refine_iter = 4
        self.track_refine_iter = 2
        
        # Load the 3D target geometry
        # mesh = trimesh.load(mesh_file)
        loaded_geometry = trimesh.load(mesh_file)        

        if isinstance(loaded_geometry, trimesh.Scene):
            mesh = loaded_geometry.dump(concatenate=True)
        else:
            mesh = loaded_geometry

        mesh.visual = trimesh.visual.ColorVisuals(mesh) 


        # Spin up the neural net predictors
        scorer = estimater.ScorePredictor()
        refiner = estimater.PoseRefinePredictor()
        
        # Initialize the high-speed GPU rendering context
        glctx = dr.RasterizeCudaContext()
        
        # Instantiate the engine in RGB-Only mode (render_only=True)
        self.est = estimater.FoundationPose(
            model_pts=mesh.vertices, 
            model_normals=mesh.vertex_normals, 
            mesh=mesh, 
            scorer=scorer, 
            refiner=refiner, 
            glctx=glctx
        )
        self.get_logger().info("FoundationPose initialized.")

        # Instantiate SAM2
        self.get_logger().info("Start initializing SAM2...")


        checkpoint = "./sam2_source/checkpoints/sam2.1_hiera_large.pt"
        model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
        self.predictor = SAM2ImagePredictor(build_sam2(model_cfg, checkpoint))
        self.get_logger().info("SAM2 initialized.")

        # Track the sequence state
        self.is_first_frame = True
        self.current_pose = None

        

        self.get_logger().info("Initialization complete!")

    def _wait_for_tf(
        self, target_frame: str, source_frame: str, timeout_sec: float = 10.0
    ) -> bool:
        """Wait for a TF frame to become available."""
        start = self.time_now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                )
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(
                        f"Waiting for transform '{source_frame}' -> '{target_frame}'... -- are you running eval with `ground_truth:=true`?"
                    )
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(
            f"Transform '{source_frame}' not available after {timeout_sec}s"
        )
        return False

    def calc_gripper_pose(
        self,
        port_transform: Transform,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        reset_xy_integrator: bool = False,
    ) -> Pose:
        """Find the gripper pose that results in plug alignment."""
        q_port = (
            port_transform.rotation.w,
            port_transform.rotation.x,
            port_transform.rotation.y,
            port_transform.rotation.z,
        )
        plug_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            f"{self._task.cable_name}/{self._task.plug_name}_link",
            Time(),
        )
        q_plug = (
            plug_tf_stamped.transform.rotation.w,
            plug_tf_stamped.transform.rotation.x,
            plug_tf_stamped.transform.rotation.y,
            plug_tf_stamped.transform.rotation.z,
        )
        q_plug_inv = (
            -q_plug[0],
            q_plug[1],
            q_plug[2],
            q_plug[3],
        )
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        gripper_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            "gripper/tcp",
            Time(),
        )
        q_gripper = (
            gripper_tf_stamped.transform.rotation.w,
            gripper_tf_stamped.transform.rotation.x,
            gripper_tf_stamped.transform.rotation.y,
            gripper_tf_stamped.transform.rotation.z,
        )
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)
        q_gripper_slerp = quaternion_slerp(q_gripper, q_gripper_target, slerp_fraction)

        gripper_xyz = (
            gripper_tf_stamped.transform.translation.x,
            gripper_tf_stamped.transform.translation.y,
            gripper_tf_stamped.transform.translation.z,
        )
        port_xy = (
            port_transform.translation.x,
            port_transform.translation.y,
        )
        plug_xyz = (
            plug_tf_stamped.transform.translation.x,
            plug_tf_stamped.transform.translation.y,
            plug_tf_stamped.transform.translation.z,
        )
        plug_tip_gripper_offset = (
            gripper_xyz[0] - plug_xyz[0],
            gripper_xyz[1] - plug_xyz[1],
            gripper_xyz[2] - plug_xyz[2],
        )

        tip_x_error = port_xy[0] - plug_xyz[0]
        tip_y_error = port_xy[1] - plug_xyz[1]

        if reset_xy_integrator:
            self._tip_x_error_integrator = 0.0
            self._tip_y_error_integrator = 0.0
        else:
            self._tip_x_error_integrator = np.clip(
                self._tip_x_error_integrator + tip_x_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )
            self._tip_y_error_integrator = np.clip(
                self._tip_y_error_integrator + tip_y_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )

        self.get_logger().info(
            f"pfrac: {position_fraction:.3} xy_error: {tip_x_error:0.3} {tip_y_error:0.3}   integrators: {self._tip_x_error_integrator:.3} , {self._tip_y_error_integrator:.3}"
        )

        i_gain = 0.15

        target_x = port_xy[0] + i_gain * self._tip_x_error_integrator
        target_y = port_xy[1] + i_gain * self._tip_y_error_integrator
        target_z = port_transform.translation.z + z_offset - plug_tip_gripper_offset[2]

        blend_xyz = (
            position_fraction * target_x + (1.0 - position_fraction) * gripper_xyz[0],
            position_fraction * target_y + (1.0 - position_fraction) * gripper_xyz[1],
            position_fraction * target_z + (1.0 - position_fraction) * gripper_xyz[2],
        )

        return Pose(
            position=Point(
                x=blend_xyz[0],
                y=blend_xyz[1],
                z=blend_xyz[2],
            ),
            orientation=Quaternion(
                w=q_gripper_slerp[0],
                x=q_gripper_slerp[1],
                y=q_gripper_slerp[2],
                z=q_gripper_slerp[3],
            ),
        )

    def show_cv2_prediction(self, rgb_img, pose_matrix, K_matrix):
        """
        Draws the 3D tracking axis and pops up an immediate, lightweight OpenCV window.
        """
        # 1. Convert RGB to BGR because OpenCV displays images in BGR format
        vis_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
        
        # 2. Extract rotation and translation from the (4,4) matrix
        R = pose_matrix[:3, :3]
        tvec = pose_matrix[:3, 3].reshape(3, 1)
        rvec, _ = cv2.Rodrigues(R)
        
        # 3. Define 3D axis points (10cm length)
        axis_length = 0.1
        axis_points = np.float32([[0, 0, 0], 
                                [axis_length, 0, 0], 
                                [0, axis_length, 0], 
                                [0, 0, axis_length]]).reshape(-1, 3)
        
        # 4. Project points onto 2D image plane
        img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, K_matrix, distCoeffs=None)
        img_points = img_points.astype(int).reshape(-1, 2)
        
        # 5. Draw the axis lines (BGR format: Blue, Green, Red)
        origin = tuple(img_points[0])
        cv2.line(vis_img, origin, tuple(img_points[1]), (0, 0, 255), 3) # X-Axis (Red)
        cv2.line(vis_img, origin, tuple(img_points[2]), (0, 255, 0), 3) # Y-Axis (Green)
        cv2.line(vis_img, origin, tuple(img_points[3]), (255, 0, 0), 3) # Z-Axis (Blue)
        
        cv2.putText(vis_img, "FoundationPose Live Tracking", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # 6. Pop open the native desktop window
        cv2.imshow("FoundationPose Tracker Monitor", vis_img)
        
        # CRITICAL: cv2.imshow requires cv2.waitKey(1) to process internal UI render events
        cv2.waitKey(30)

    def get_center_camera_rgbd(self, get_observation):
        """
        Get both rgb and depth image from actual aic node, transform into cv2
        """
        observation, depth_msg = get_observation()
        if observation is None or depth_msg is None:
            self.get_logger().error("Could not find observation or depth")
            return None, None, None
        center_image = observation.center_image
        center_cv_image = self.bridge.imgmsg_to_cv2(center_image, desired_encoding='bgr8').copy()
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').copy()
        
        center_camera_info = observation.center_camera_info
        K_matrix = np.array(center_camera_info.k).reshape(3, 3).copy()
        return center_cv_image, depth_image, K_matrix

    def get_port_transform_in_base_frame(self, T_camera_card: np.ndarray) -> Transform:
        """
        Transforms the NIC card pose from the Camera Frame into the Robot Base Link Frame.
        We use the averaged difference between NIC card and port pose to statically trasnform the frame.
        
        Input:  T_camera_card -> (4,4) numpy array tracking matrix from FoundationPose
        Output: Transform     -> ROS 2 Geometry Transform relative to 'base_link'
        """
        camera_frame = "center_camera/optical"
        base_frame = "base_link"
        
        try:
            camera_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                base_frame,
                camera_frame,
                Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not look up camera frame '{camera_frame}' relative to '{base_frame}': {ex}")
            return None

        T_base_camera = np.eye(4)
        
        T_base_camera[0, 3] = camera_tf_stamped.transform.translation.x
        T_base_camera[1, 3] = camera_tf_stamped.transform.translation.y
        T_base_camera[2, 3] = camera_tf_stamped.transform.translation.z
        
        q_cam = [
            camera_tf_stamped.transform.rotation.x,
            camera_tf_stamped.transform.rotation.y,
            camera_tf_stamped.transform.rotation.z,
            camera_tf_stamped.transform.rotation.w
        ]
        T_base_camera[:3, :3] = R.from_quat(q_cam).as_matrix()

        T_base_card = T_base_camera @ T_camera_card

        T_card_port = np.eye(4)
        
        T_card_port[:3, 3] = [0.01230, -0.03510, 0.00600] 
        
        averaged_quat = [0.71400, 0.00400, -0.01300, -0.70000]
        T_card_port[:3, :3] = R.from_quat(averaged_quat).as_matrix()
        
        T_base_port = T_base_card @ T_card_port

        port_transform = Transform()
        
        port_transform.translation.x = float(T_base_port[0, 3])
        port_transform.translation.y = float(T_base_port[1, 3])
        port_transform.translation.z = float(T_base_port[2, 3])
        
        q_port_output = R.from_matrix(T_base_port[:3, :3]).as_quat()
        port_transform.rotation.x = float(q_port_output[0])
        port_transform.rotation.y = float(q_port_output[1])
        port_transform.rotation.z = float(q_port_output[2])
        port_transform.rotation.w = float(q_port_output[3])

        return port_transform



    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        # get_depth: GetDepthCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info(f"MyFoundationPosePoliocy.insert_cable() task: {task}")
        self._task = task

        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        cable_tip_frame = f"{task.cable_name}/{task.plug_name}_link"

        for frame in [port_frame, cable_tip_frame]:
            if not self._wait_for_tf("base_link", frame):
                return False

        try:
            port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                port_frame,
                Time(),
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not look up port transform: {ex}")
            return False
        self.port_transform = port_tf_stamped.transform

        z_offset = 0.2


        for t in range(0, 50):
            interp_fraction = t / 50.0
            center_cv_image, depth_image, K_matrix = self.get_center_camera_rgbd(get_observation)
            try:
                if self.is_first_frame:
                    self.get_logger().info("Frame 0: Localizing target SAM2...")
                    
                    # Use SAM2 to get high quality 2D mask of the target
                    with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
                        sam_input_image = cv2.cvtColor(center_cv_image, cv2.COLOR_BGR2RGB)

                        # height, width, channels = sam_input_image.shape
                        # self.get_logger().info(f"!!! Actual Sam image Size: Width={width}px, Height={height}px !!!")

                        self.predictor.set_image(sam_input_image)
                        
                        # Manually set a prompt point
                        input_point = np.array([[592, 507]]) 
                        input_label = np.array([1])
                        
                        masks, scores, _ = self.predictor.predict(
                            point_coords=input_point,
                            point_labels=input_label,
                            multimask_output=False
                        )
                    
                    # Extract the 2D mask array (SAM2 shapes it as [1, H, W])
                    final_mask = masks[0].astype(np.uint8)
                    try:
                        # Register the 6D orientation matrix with FoundationPose using the mask from SAM2
                        with torch.inference_mode(), torch.no_grad():
                            raw_pose = self.est.register(
                                K=K_matrix, 
                                rgb=center_cv_image,
                                depth=depth_image,
                                ob_mask=final_mask,
                                iteration=self.est_refine_iter,
                            )
                            self.get_logger().info("Leave registration.")
                            self.current_pose = raw_pose.copy()
                        self.is_first_frame = False
                        gc.collect()
                        torch.cuda.empty_cache()
                        self.get_logger().info("FoundationPose registration successful! Target locked.")
                    except Exception as e:
                        self.get_logger().error(f"FoundationPose registration failed on frame 0: {e}")
                        return False
                    
                else:
                    # 2nd frame onward, using only FoundationPose to predict the pose
                    with torch.inference_mode(), torch.no_grad():
                        self.current_pose = self.est.track_one(
                            K=K_matrix,
                            rgb=center_cv_image,
                            depth=depth_image,
                            iteration=self.track_refine_iter
                        )
                
                self.show_cv2_prediction(center_cv_image, self.current_pose, K_matrix)

                predicted_port_transform = self.get_port_transform_in_base_frame(self.current_pose)


                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(
                        predicted_port_transform,
                        slerp_fraction=interp_fraction,
                        position_fraction=interp_fraction,
                        z_offset=z_offset,
                        reset_xy_integrator=True,
                    ),
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed during interpolation: {ex}")
            self.sleep_for(0.1)

        # Descend until the cable is inserted into the port.
        while True:
            center_cv_image, depth_image, K_matrix = self.get_center_camera_rgbd(get_observation)
            if z_offset < -0.015:
                break

            z_offset -= 0.0005
            self.get_logger().info(f"z_offset: {z_offset:0.5}")
            try:
                with torch.inference_mode(), torch.no_grad():
                        self.current_pose = self.est.track_one(
                            K=K_matrix,
                            rgb=center_cv_image,
                            depth=depth_image,
                            iteration=self.track_refine_iter
                        )
                
                self.show_cv2_prediction(center_cv_image, self.current_pose, K_matrix)
                predicted_port_transform = self.get_port_transform_in_base_frame(self.current_pose)

                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(predicted_port_transform, z_offset=z_offset),
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed during insertion: {ex}")
            self.sleep_for(0.1)

            self.show_cv2_prediction(center_cv_image, self.current_pose, K_matrix)


        self.get_logger().info("Waiting for connector to stabilize...")
        self.sleep_for(5.0)

        self.get_logger().info("MyFoundationPosePoliocy.insert_cable() exiting...")
        return True

