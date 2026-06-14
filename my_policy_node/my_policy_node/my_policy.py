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

# This code is adopting from cheatcode example

import numpy as np

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from transforms3d.quaternions import quat2mat

QuaternionTuple = tuple[float, float, float, float]


class MyPolicy(Policy):
    def __init__(self, parent_node):
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0
        self._max_integrator_windup = 0.05
        self._task = None
        self.bridge = CvBridge()
        super().__init__(parent_node)

    # def cal_depth(
    #         self,
    #         left_image: np.ndarray,
    #         right_image: np.ndarray,
    #         baseline: float,
    #         focal_length: float = 1236.63
    # ):
        
    #     stero = cv2.StereoSGBM.create(
    #         minDisparity=0,
    #         numDisparities=16*5,
    #         blockSize=3,
    #         P1=8 * 3 * 3**2,
    #         P2=32 * 3 * 3**2,
    #         disp12MaxDiff=1,
    #         uniquenessRatio=15,
    #         speckleWindowSize=100,
    #         speckleRange=32)
        
    #     disparity = stero.compute(left_image, right_image).astype(np.float32) / 16.0  # SGBM returns disparity scalled by 16. devide that back
    #     disparity[disparity <= 0] = 0.1
    #     depth_image = (focal_length * baseline) / disparity

    #     return depth_image
    
    
    def cal_depth(self,
            left_image: np.ndarray,
            right_image: np.ndarray,
            baseline: float,
            focal_length: float
        ):
        # SGBM works best on Grayscale images
        if len(left_image.shape) == 3:
            left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
        else:
            left_gray, right_gray = left_image, right_image

        stereo = cv2.StereoSGBM.create(
            minDisparity=0,
            numDisparities=128, # Must be divisible by 16
            blockSize=9,       # Slightly larger for stability
            P1=8 * 3 * 9**2,   # Use the same blockSize here
            P2=32 * 3 * 9**2,
            disp12MaxDiff=2,
            uniquenessRatio=3,
            speckleWindowSize=100,
            speckleRange=2
        )
        
        # SGBM returns fixed-point disparity (scaled by 16)
        disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
        
        # Mask out invalid disparity (0 or negative)
        depth_image = np.zeros_like(disparity)
        mask = disparity > 0
        depth_image[mask] = (focal_length * baseline) / disparity[mask]

        # # This uses a small 3x3 pixel block to 'close' up tiny missing holes in the depth data
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # depth_smoothed = cv2.morphologyEx(depth_image, cv2.MORPH_CLOSE, kernel)

        # # You can also run a median filter to erase random 'speckle' spikes without blurring the edges
        # depth_final = cv2.medianBlur(depth_smoothed, 3)



        # Temporary Debug Print
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            self.get_logger().info(
                f"DEPTH CHECK -> Min: {np.min(valid_depths):.2f}m, "
                f"Max: {np.max(valid_depths):.2f}m, "
                f"Mean: {np.mean(valid_depths):.2f}m"
            )


        return depth_image


    
    # def stereo_rect(
    #         self,
    #         left_image: np.ndarray,
    #         left_camera_info: CameraInfo,
    #         right_image: np.ndarray,
    #         right_camera_info: CameraInfo,
    #         R: np.ndarray,
    #         T: np.ndarray
    # ):
    #     D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    #     image_size = (int(left_camera_info.width), int(left_camera_info.height))

    #     R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(left_camera_info.k, D, right_camera_info.k, D, 
    #                                                 image_size, R, T)

    #     map_left_x, map_left_y = cv2.initUndistortRectifyMap(left_camera_info.k, D, R1, P1, image_size, cv2.CV_32FC1)
    #     map_right_x, map_right_y = cv2.initUndistortRectifyMap(right_camera_info.k, D, R2, P2, image_size, cv2.CV_32FC1)
    #     left_image_rect = cv2.remap(left_image, map_left_x, map_left_y, cv2.INTER_LINEAR)
    #     right_image_rect = cv2.remap(right_image, map_right_x, map_right_y, cv2.INTER_LINEAR)

    #     baseline = abs(P2[0,3] / P2[0,0])

    #     return left_image_rect, right_image_rect, baseline

    def stereo_rect(
        self,
        left_image: np.ndarray,
        left_camera_info: CameraInfo,
        right_image: np.ndarray,
        right_camera_info: CameraInfo,
        R: np.ndarray,
        T: np.ndarray
    ):
        # 1. Force K into 3x3 matrices
        K_left = np.array(left_camera_info.k).reshape(3, 3)
        K_right = np.array(right_camera_info.k).reshape(3, 3)
        
        # 2. Distortion must be float64 for OpenCV C++ backend stability
        D = np.zeros(5, dtype=np.float64)
        image_size = (int(left_camera_info.width), int(left_camera_info.height))

        # 3. Perform Rectification
        # Note: Use cv2.CALIB_ZERO_DISPARITY to align the principal points
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K_left, D, K_right, D, 
            image_size, R, T, 
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=1.0
        )

        # 4. Generate Maps
        map_left_x, map_left_y = cv2.initUndistortRectifyMap(K_left, D, R1, P1, image_size, cv2.CV_32FC1)
        map_right_x, map_right_y = cv2.initUndistortRectifyMap(K_right, D, R2, P2, image_size, cv2.CV_32FC1)
        
        left_image_rect = cv2.remap(left_image, map_left_x, map_left_y, cv2.INTER_LINEAR)
        right_image_rect = cv2.remap(right_image, map_right_x, map_right_y, cv2.INTER_LINEAR)

        # 5. Calculate New focal length and baseline from P2
        f_rect = P2[0, 0]
        baseline = abs(P2[0, 3] / f_rect)

        self.get_logger().info(f"--- STEREO PARAMETERS DEBUG ---")
        self.get_logger().info(f"Calculated f_rect: {f_rect:.2f} px")
        self.get_logger().info(f"Calculated baseline: {baseline:.4f} meters")
        self.get_logger().info(f"Focal * Baseline Factor: {f_rect * baseline:.4f}")


        return left_image_rect, right_image_rect, baseline, f_rect
    

    # def _wait_for_tf(
    #     self, target_frame: str, source_frame: str, timeout_sec: float = 10.0
    # ) -> bool:
    #     """Wait for a TF frame to become available."""
    #     start = self.time_now()
    #     timeout = Duration(seconds=timeout_sec)
    #     attempt = 0
    #     while (self.time_now() - start) < timeout:
    #         try:
    #             self._parent_node._tf_buffer.lookup_transform(
    #                 target_frame,
    #                 source_frame,
    #                 Time(),
    #             )
    #             return True
    #         except TransformException:
    #             if attempt % 20 == 0:
    #                 self.get_logger().info(
    #                     f"Waiting for transform '{source_frame}' -> '{target_frame}'... -- are you running eval with `ground_truth:=true`?"
    #                 )
    #             attempt += 1
    #             self.sleep_for(0.1)
    #     self.get_logger().error(
    #         f"Transform '{source_frame}' not available after {timeout_sec}s"
    #     )
    #     return False


    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info(f"MyPolicy.insert_cable() task: {task}")
        self._task = task

        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        cable_tip_frame = f"{task.cable_name}/{task.plug_name}_link"

        # Wait for both the port and cable tip TFs to become available.
        # These come via ground_truth and may not be immediate.
        # for frame in [port_frame, cable_tip_frame]:
        #     if not self._wait_for_tf("base_link", frame):
        #         return False

        try:
            # port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            #     "base_link",
            #     port_frame,
            #     Time(),
            # )
            # left_camera_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            #     "base_link",
            #     "left_camera/optical",
            #     Time(),
            # )
            # right_camera_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            #     "base_link",
            #     "right_camera/optical",
            #     Time(),
            # )
            # camera_tf_right_to_left = self._parent_node._tf_buffer.lookup_transform(
            #     "left_camera/optical",
            #     "right_camera/optical",
            #     Time(),
            # )

            # camera_tf_right_to_left = self._parent_node._tf_buffer.lookup_transform(
            #     "right_camera/optical",
            #     "left_camera/optical",
            #     Time(),
            # )
            
            camera_tf_center_to_right = self._parent_node._tf_buffer.lookup_transform(
                "right_camera/optical",
                "center_camera/optical",
                Time(),
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not look up port transform: {ex}")
            return False
        # left_camera_transform = left_camera_tf_stamped.transform
        # right_camera_transform = right_camera_tf_stamped.transform
        # left_to_right_camera_transform = camera_tf_right_to_left.transform
        # left_to_center_camera_transform = camera_tf_center_to_left.transform
        center_to_right_camera_transform = camera_tf_center_to_right.transform


        # R_quat = left_to_right_camera_transform.rotation
        # R_quat = left_to_center_camera_transform.rotation
        R_quat = center_to_right_camera_transform.rotation
        R_quat_wxyz = [R_quat.w, R_quat.x, R_quat.y, R_quat.z]
        R_mat = quat2mat(R_quat_wxyz)

        # T = left_to_right_camera_transform.translation
        # T = left_to_center_camera_transform.translation
        T = center_to_right_camera_transform.translation
        T_vec = np.array([T.x, T.y, T.z], dtype=np.float64).reshape(3, 1)

        # left_image = get_observation.left_image
        # left_cv_image = self.bridge.imgmsg_to_cv2(left_image, desired_encoding='bgr8')
        # left_camera_info = get_observation.left_camera_info

        right_image = get_observation.right_image
        right_cv_image = self.bridge.imgmsg_to_cv2(right_image, desired_encoding='bgr8')
        right_camera_info = get_observation.right_camera_info

        center_image = get_observation.center_image
        center_cv_image = self.bridge.imgmsg_to_cv2(center_image, desired_encoding='bgr8')
        center_camera_info = get_observation.center_camera_info
        
        # left_image_rect, right_image_rect, baseline, f_rect = self.stereo_rect(left_cv_image, left_camera_info, right_cv_image, right_camera_info, R_mat, T_vec)
        # depth_image = self.cal_depth(left_image_rect, right_image_rect, baseline, f_rect)

        # left_image_rect, center_image_rect, baseline, f_rect = self.stereo_rect(left_cv_image, left_camera_info, center_cv_image, center_camera_info, R_mat, T_vec)
        # depth_image = self.cal_depth(left_image_rect, center_image_rect, baseline, f_rect)

        center_image_rect, right_image_rect, baseline, f_rect = self.stereo_rect(center_cv_image, center_camera_info, right_cv_image, right_camera_info, R_mat, T_vec)

        center_image_rect = cv2.GaussianBlur(center_image_rect, (3, 3), 0)
        right_image_rect = cv2.GaussianBlur(right_image_rect, (3, 3), 0)

        depth_image = self.cal_depth(center_image_rect, right_image_rect, baseline, f_rect)


        # 3. Visualization logic
        # We normalize the depth to 0-255 so we can actually see it.
 
        valid_mask = depth_image > 0


        # A: Close up tiny isolated structural holes (pixels)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        depth_closed = cv2.morphologyEx(depth_image, cv2.MORPH_CLOSE, kernel)

        # B: Smooth flat planes while respecting structural boundaries
        depth_final = cv2.medianBlur(depth_closed, 5)

        depth_display = np.zeros_like(depth_final, dtype=np.uint8)
        depth_display[valid_mask] = np.clip(depth_final[valid_mask], 0, 2.0) / 2.0 * 255
        
        # Apply a colormap so it's easier to read (Hotter = Closer)
        depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
        depth_colormap[~valid_mask] = [0, 0, 0] 

        # cv2.imshow("Rectified Left", left_image_rect)
        cv2.imshow("Rectified Right", center_image_rect)
        cv2.imshow("Depth Map", depth_colormap)
        # cv2.imshow("image Left", left_cv_image)
        # cv2.imshow("image right", right_image)
        # center_image = get_observation.center_image
        # cv2.imshow("image center", center_image)
        
        # Wait for a brief moment to allow the window to refresh
        cv2.waitKey(1)

        # 4. Success Signal
        # A simple way to check success is to see if we have valid depth data
        valid_pixels = np.count_nonzero(depth_image > 0)
        total_pixels = depth_image.size
        success_rate = (valid_pixels / total_pixels) * 100

        if success_rate > 10: # If at least 10% of the image has depth
            self.get_logger().info(f"Depth calculation successful. Valid pixels: {success_rate:.2f}%")
            return True
        else:
            self.get_logger().error("Depth calculation failed: Image is mostly empty or invalid.")
            return False


        # z_offset = 0.2

        
        # Descend until the cable is inserted into the port.
        # while True:
        #     if z_offset < -0.015:
        #         break

        #     z_offset -= 0.0005
        #     self.get_logger().info(f"z_offset: {z_offset:0.5}")
        #     try:
        #         self.set_pose_target(
        #             move_robot=move_robot,
        #             pose=self.calc_gripper_pose(port_transform, z_offset=z_offset),
        #         )
        #     except TransformException as ex:
        #         self.get_logger().warn(f"TF lookup failed during insertion: {ex}")
        #     self.sleep_for(0.05)

        # self.get_logger().info("Waiting for connector to stabilize...")
        # self.sleep_for(5.0)

        # self.get_logger().info("CheatCode.insert_cable() exiting...")
        # return True
