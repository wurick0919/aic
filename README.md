# AIC with RGBD

This project is an application utilizing  [**AI for Industry Challenge toolkit**](https://github.com/intrinsic-dev/aic). **AIC** is an open competition held by Intrinsic and Open Robotics, for developers and roboticists aimed at solving some of the hardest, high-impact problems in robotics and manufacturing.

The aim of the project is to conduct (some cool name like vision based insersion task?)(in industrial setting, inserting a SFP module into a port of a NIC card).
We utilise SAM2 and FoundationPose to achive XXX?, Maybe also mentioning AWS remote cloud computing?
(an demonstration video)
[](assets/video_with_gz.gif) 




## Architechture

We modify gazebo to expose depth. Instead of using 3 cameras, we only use one rgbd. Use sam2 to create high quality mask(an image of the mask), and Foundation pose to predict target pose, using the target pose and PI control to guide the robot to finish insertion task. (maybe an )


## Environment and setup
Using aws ec2, g5.2xlarge. mention the gpu, ram and vram.

This project shares a similar environment setup with AIC toolkit, uses pixi env to manage dependancies, and distrobox for simulation env, where gazebo is run.
Some part of the dependencies of SAM2 and FP are also added, but other package needed to be manually installed. Intruction of setting up still under contruction.

## Technical stacks?
ros2 kilted kaiju
pixi
distrobox
gazebo
OpenCV
SAM
FP

## Challanges
talk about grey frame issue

## Repo Structure

Most of the work is being done in my_policy_node package, where aic_model_depth is the main node, and MyFoundationPosePolicy contian the actual workflow.



## License

This project is licensed under the Apache License 2.0 - see the individual package files for details.


# Autonomous Cable Insertion via 6D Visual Servoing

This repository contains an autonomous robotic insertion system developed for the [**AI for Industry Challenge (AIC)**](https://github.com/intrinsic-dev/aic) hosted by Intrinsic and Open Robotics. 

The objective of this project is to solve a high-impact manufacturing problem: **dynamically tracking and inserting an SFP module into a network interface card (NIC) port** inside a simulated industrial workstation. The entire architecture is developed, tested, and executed on a cloud-based AWS EC2 instance.

![System Demonstration](assets/螢幕錄影%202026-06-13%20晚上11.54.17.mov)
*Figure 1: Closed-loop visual servoing tracking the card and executing alignment.*

---

## Architecture & Frame Chains

To optimize performance and streamline the sensor footprint, we modified the default Gazebo simulation environment to expose raw depth maps, allowing us to collapse the system down from three standard cameras to a **single Eye-in-Hand RGB-D sensor**.

The control and perception pipeline operates in a continuous loop:
1. **Segmentation:** Segment Anything 2 (SAM2) processes the RGB feed to generate a high-precision binary mask of the target card.
2. **6D Tracking:** FoundationPose utilizes the RGB-D data and the SAM2 mask to predict the exact 6D object pose relative to the camera matrix.
3. **Control:** The system computes the coordinate transform chain (`base_link` → `camera` → `object_frame`) and calculates a static CAD offset to target the port. A Proportional-Integral (PI) velocity controller minimizes alignment errors to guide the arm to a successful insertion.

| Raw Camera View | SAM2 Segmented Mask |
| :---: | :---: |
| ![Raw View](assets/截圖%202026-06-13%20下午4.59.13.png) | ![SAM2 Mask](path/to/sam2_mask.jpg) |

---

## Technical Stack

* **Robotics Framework:** ROS 2 (Kilted Kaiju)
* **Simulation Engine:** Gazebo / Distrobox
* **Perception Frameworks:** Segment Anything 2 (SAM2), FoundationPose, OpenCV
* **Environment Management:** Pixi

---

## Compute Infrastructure & Hardware Challenges

### Host Specs
* **Instance:** AWS EC2 `g5.2xlarge`
* **GPU:** 1x NVIDIA A10G ($24\text{ GB}$ VRAM)
* **vCPU / RAM:** 8 vCPUs / $32\text{ GB}$ System RAM

### The "Gray Frame" Challenge & Production Mitigation
During close-proximity approach maneuvers, the image topic stream would intermittently drop entirely and turn completely gray. 

* **The Cause:** This was diagnosed as a system-wide resource contention issue rather than an algorithm bug. As the robot gripper neared the card, Gazebo's multi-threaded physics engine hit a computational spike calculating complex contact meshes. Running concurrently with our heavy TensorRT/PyTorch deep learning model on a single instance, this spike starved the CPU cores managing the ROS 2 DDS middleware layer. This caused local UDP network packets to fragment and drop.
* **The Solution:** Rather than scaling up to an expensive multi-GPU instance, the pipeline was made production-resilient via software configuration. We implemented a custom **Best-Effort QoS Profile (Depth=1)** on the image subscriber to prevent data packet queues from buffering. We paired this with an image-variance safety mask: if a corrupted gray frame is detected, the policy temporarily suspends visual tracking updates and uses dead reckoning via the last verified static CAD offset to complete the insertion safely.

---

## Repository Structure

The core workflow and logic are contained within the following package structure:

```text
your-robotics-repo/
├── .gitignore                      # Excludes heavy dependencies, weights, and .pixi
├── README.md                       # Project overview and architecture guide
└── my_policy_node/                 # Primary ROS 2 package
    └── src/
        └── cable_insertion/
            ├── aic_model_depth.py  # Main ROS 2 executable node
            └── workflow.py         # MyFoundationPosePolicy execution script
