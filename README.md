# Implementation of FoundationPose and SAM2 on AIC 

This repository contains an autonomous robotic insertion system developed based on the [**AI for Industry Challenge**](https://github.com/intrinsic-dev/aic) hosted by Intrinsic and Open Robotics.

The objective of this project is to perform the task of **dynamically tracking and inserting an SFP module into a network interface card (NIC) port** inside a simulated industrial workstation. The entire architecture is developed, tested, and executed on a cloud-based AWS EC2 instance.

![demo video](assets/video_with_gz_fast.gif) 




## Architecture
This project is based on the aformentioned [AIC toolkit](https://github.com/intrinsic-dev/aic), in which the simulation environment, control pipeline and some example control strategies are provided.

We modified the default Gazebo simulation environment to expose raw depth maps, allowing us to collapse the system down from three standard cameras to a **single Eye-in-Hand RGB-D sensor**.

Then we implement SAM2 and FoundationPose to generate high quality 6D pose estimation, and perform PI control to guide the SFP module into the target port.

1. **Raw image from Gazebo**
![demo video](assets/center_camera_image_raw.png)
2. **Segmentation:** [Segment Anything 2 (SAM2)](https://github.com/facebookresearch/sam2.git) processes the RGB feed to generate a high-precision binary mask of the target card.
![demo video](assets/sam2_mask_output.png)
3. **6D Tracking:** [FoundationPose](https://github.com/NVlabs/FoundationPose.git) utilizes the RGB-D data and the SAM2 mask to predict the exact 6D object pose relative to the camera matrix.
![demo video](assets/foundationpose_pose_output.png)
4. **Control:** The system computes the coordinate transform chain (`base_link` → `camera` → `object_frame`) and calculates a static CAD offset to target the port. A Proportional-Integral (PI) velocity controller from the toolkit is used to minimize alignment errors to guide the arm to a successful insertion.
![demo video](assets/output_video_fast.gif)



## Environment and setup

### Minimum Computation Specification by aic:
* **OS: Ubuntu 24.04**
* **CPU: 4-8 cores**
* **RAM: 32GB+**
* **GPU: NVIDIA RTX 2070+ or equivalent**
* **VRAM: 8GB+**


### Setup
This project shares a similar environment setup with AIC toolkit, using [pixi](pixi.toml) to manage dependencies, and distrobox for simulation env, where gazebo is run.

The dependencies of SAM2 and FoundationPose are also added into [pixi dependencies](pixi.toml), but some inidividual packages needs to be manually installed inside pixi environemnt.

Refer to [getting start](https://github.com/intrinsic-dev/aic/blob/main/docs/getting_started.md) from aic toolkit for docker, pixi, distrobox and Nvidia container toolkit.

After set up, inside pixi environment, install dependencies for SAM2 and FoundationPose:
```bash
pip install open3d pyrender
pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable" --no-build-isolation
git clone https://github.com/NVlabs/nvdiffrast
cd nvdiffrast
CUDA_HOME=$CONDA_PREFIX TORCH_CUDA_ARCH_LIST="8.6" pip install . --no-build-isolation   # change version depends on hardware
cd ..
```

Install FoundationPose
```bash
git clone https://github.com/NVlabs/FoundationPose.git
cd FoundationPose
./build_all_conda.py
LD_PRELOAD=$CONDA_PREFIX/lib/libjpeg.so python run_demo.py
cd ..
```

Install SAM2
```bash
git clone https://github.com/facebookresearch/sam2.git && cd sam2
pip install --no-build-isolation -e .
cd checkpoints && \
./download_ckpts.sh && \
cd ..
mv sam2 sam2_source
```

## Technical Stack

* **Robotics Framework:** ROS 2 Kilted Kaiju
* **Simulation Engine:** Gazebo, Distrobox
* **Perception Frameworks:** Segment Anything 2 (SAM2), FoundationPose, OpenCV
* **Environment Management:** Pixi, pip

## Notes
During close-proximity approach maneuvers, the image topic stream would intermittently drop entirely and turn completely gray.
Preliminary tests shows no sign of hardware resource starvation from CPU, memory or vRAM. Further tests are needed.

## Repo Structure

The core workflow and logic are contained within the following package structure:

```text
aic/
├── my_policy_node/my_policy_node   # Primary ROS 2 package
|   ├── aic_model_depth.py          # Main ROS 2 executable node
|   └── MyFoundationPosePolicy.py   # Vision tracking & policy execution script
└── aic_*                           # Packages from aic toolkit
            
            
```

## License

This project is licensed under the Apache License 2.0 - see the individual package files for details.

